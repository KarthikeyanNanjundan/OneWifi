/************************************************************************************
  If not stated otherwise in this file or this component's LICENSE file the
  following copyright and licenses apply:

  Copyright 2018 RDK Management

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
 **************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#include "wifi_hal.h"
#include "wifi_ipc_server.h"
#include "wifi_ctrl.h"
#include "wifi_events.h"      /* push_event_to_ctrl_queue()              */
#include "wifi_util.h"        /* wifi_util_error_print / dbg             */

/* ── helpers ─────────────────────────────────────────────────────── */

static wifi_ipc_server_req_t *req_alloc(const char *cmd, unsigned int cmd_len)
{
    wifi_ipc_server_req_t *req = calloc(1, sizeof(*req));
    if (!req) return NULL;

    unsigned int n = cmd_len < sizeof(req->cmd) - 1 ? cmd_len : sizeof(req->cmd) - 1;
    memcpy(req->cmd, cmd, n);
    req->cmd[n] = '\0';

    if (pthread_mutex_init(&req->lock, NULL) != 0) {
        free(req);
        return NULL;
    }
    if (pthread_cond_init(&req->cond, NULL) != 0) {
        pthread_mutex_destroy(&req->lock);
        free(req);
        return NULL;
    }
    return req;
}

static void req_free(wifi_ipc_server_req_t *req)
{
    if (!req) return;
    free(req->result);
    pthread_cond_destroy(&req->cond);
    pthread_mutex_destroy(&req->lock);
    free(req);
}

/* Returns true if ctrl thread completed in time; must be called with lock held */
static bool req_wait(wifi_ipc_server_req_t *req)
{
    struct timespec dl;
    clock_gettime(CLOCK_REALTIME, &dl);
    dl.tv_sec += WIFI_IPC_SERVER_TIMEOUT_SEC;

    while (!req->done) {
        int rc = pthread_cond_timedwait(&req->cond, &req->lock, &dl);
        if (rc == ETIMEDOUT) {
            wifi_util_error_print(WIFI_CTRL,
                "%s:%d timeout waiting for ctrl thread (cmd='%s')\n",
                __func__, __LINE__, req->cmd);
            return false;
        }
        if (rc != 0 && rc != EINTR) return false;
    }
    return true;
}

/* ── Bus callback thread ─────────────────────────────────────────── */

bus_error_t wifiapi_call_method(char const *methodName,
                                bus_data_prop_t *inParams,
                                bus_data_prop_t *outParams,
                                void            *asyncHandle)
{
    (void)methodName;
    (void)asyncHandle;

    if (!inParams || !outParams ||
        inParams->value.data_type != bus_data_type_string ||
        !inParams->value.raw_data.bytes) {
        wifi_util_error_print(WIFI_CTRL, "%s:%d invalid inputs\n", __func__, __LINE__);
        return bus_error_invalid_input;
    }

    unsigned int cmd_len = inParams->value.raw_data_len > 0
        ? inParams->value.raw_data_len
        : (unsigned int)strlen((char *)inParams->value.raw_data.bytes);

    /* 1. Allocate rendezvous struct */
    wifi_ipc_server_req_t *req = req_alloc((char *)inParams->value.raw_data.bytes, cmd_len);
    if (!req) return bus_error_out_of_resources;

    /* 2. Push POINTER into ctrl queue — memcpy copies sizeof(ptr) bytes */
    wifi_ipc_server_req_t *req_ptr = req;
    if (push_event_to_ctrl_queue(&req_ptr, sizeof(req_ptr),
            wifi_event_type_wifiapi,
            wifi_event_type_wifiapi_ipc_call, NULL) != 0) {
        wifi_util_error_print(WIFI_CTRL,
            "%s:%d push_event_to_ctrl_queue failed cmd='%s'\n",
            __func__, __LINE__, req->cmd);
        req_free(req);
        return bus_error_general;
    }

    /* 3. Block until ctrl core thread completes or timeout */
    pthread_mutex_lock(&req->lock);
    bool ok = req_wait(req);
    pthread_mutex_unlock(&req->lock);

    /* 4. Populate outParams */
    const char *rs = (ok && req->success && req->result) ? req->result : "IPC call failed";
    size_t rlen = strlen(rs);
    outParams->value.raw_data.bytes = calloc(rlen + 1, 1);
    if (!outParams->value.raw_data.bytes) {
        req_free(req);
        return bus_error_out_of_resources;
    }
    memcpy(outParams->value.raw_data.bytes, rs, rlen);
    outParams->value.raw_data_len = (unsigned int)rlen;
    outParams->value.data_type    = bus_data_type_string;
    outParams->is_data_set        = true;
    outParams->status             = bus_error_success;
    outParams->ref_count          = 1;
    outParams->next_data          = NULL;

    wifi_util_dbg_print(WIFI_CTRL, "%s:%d cmd='%s' ok=%d result='%s'\n",
        __func__, __LINE__, req->cmd, ok, rs);

    req_free(req);
    return bus_error_success;
}

/* ── IPC command handlers (run on ctrl core thread) ─────────────── */

#define IPC_RESULT_BUF_SIZE  4096

static void ipc_handle_get_neighboring_wifi_status(char **args, unsigned int num_args,
    char *buf, unsigned int bufsz)
{
    if (num_args < 2) {
        snprintf(buf, bufsz,
            "wifi_getNeighboringWiFiStatus: too few args (need radio_index)\n");
            return;
    }
    int radio_index = atoi(args[1]);
    wifi_neighbor_ap2_t *ap_array = NULL;
    unsigned int ap_count = 0;
    int len = 0;

    if (wifi_hal_getNeighboringWiFiStatus(radio_index, &ap_array,
            &ap_count) != RETURN_OK) {
        snprintf(buf, bufsz, "wifi_getNeighboringWiFiStatus: failed\n");
        return;
    }

    len += snprintf(buf + len, bufsz - (unsigned int)len,
        "\nwifi_getNeighboringWiFiStatus: number of results: %u\n\n", ap_count);

    for (unsigned int i = 0; i < ap_count; i++) {
        if ((unsigned int)len >= bufsz) break;
        len += snprintf(buf + len, bufsz - (unsigned int)len,
            "ssid: %s\nbssid: %s\n"
            "mode: %s\nchannel: %u\nsignal strength: %d\n"
            "security mode: %s\nencryption mode: %s\n"
            "frequency band: %s\nsupported standards: %s\n"
            "operating standards: %s\noperating bandwidth: %s\n"
            "beacon period: %u\nnoise: %d\nbasic rates: %s\n"
            "supported data rates: %s\ndtim period: %u\n"
            "channel utilization: %u\n\n",
            ap_array[i].ap_SSID, ap_array[i].ap_BSSID,
            ap_array[i].ap_Mode, ap_array[i].ap_Channel,
            ap_array[i].ap_SignalStrength,
            ap_array[i].ap_SecurityModeEnabled, ap_array[i].ap_EncryptionMode,
            ap_array[i].ap_OperatingFrequencyBand,
            ap_array[i].ap_SupportedStandards, ap_array[i].ap_OperatingStandards,
            ap_array[i].ap_OperatingChannelBandwidth,
            ap_array[i].ap_BeaconPeriod, ap_array[i].ap_Noise,
            ap_array[i].ap_BasicDataTransferRates,
            ap_array[i].ap_SupportedDataTransferRates,
            ap_array[i].ap_DTIMPeriod, ap_array[i].ap_ChannelUtilization);
    }
    free(ap_array);
}

/* Parses cmd string and dispatches to the right handler.
 * Fills buf with NUL-terminated result text. */
static void ipc_execute_command(const char *cmd, char *buf, unsigned int bufsz)
{
    char input[sizeof(((wifi_ipc_server_req_t *)0)->cmd)];
    char *args[16];
    unsigned int num_args = 0;
    char *saveptr = NULL;

    strncpy(input, cmd, sizeof(input) - 1);
    input[sizeof(input) - 1] = '\0';

    char *tok = strtok_r(input, " ", &saveptr);
    while (tok && num_args < 16) {
        args[num_args++] = tok;
        tok = strtok_r(NULL, " ", &saveptr);
    }
    if (num_args == 0) {
        snprintf(buf, bufsz, "ipc: empty command\n");
        return;
    }

    if (strcmp(args[0], "wifi_startNeighborScan") == 0) {
        wifiapi_handle_start_neighbor_scan(args, num_args, buf, bufsz);
        return;
    }
    if (strcmp(args[0], "wifi_getNeighboringWiFiStatus") == 0) {
        ipc_handle_get_neighboring_wifi_status(args, num_args, buf, bufsz);
        return;
    }

    snprintf(buf, bufsz, "ipc: unknown command '%s'\n", args[0]);
}

/* ── Ctrl core thread handler ────────────────────────────────────── */

void wifi_ipc_server_handle_req(void *data, unsigned int len)
{
    if (!data || len != sizeof(wifi_ipc_server_req_t *)) {
        wifi_util_error_print(WIFI_CTRL,
            "%s:%d bad event data data=%p len=%u expected=%zu\n",
            __func__, __LINE__, data, len, sizeof(wifi_ipc_server_req_t *));
        return;
    }

    wifi_ipc_server_req_t *req;
    memcpy(&req, data, sizeof(req));
    if (!req) return;

    /* Execute command on ctrl core thread, write result into static buffer */
    static char result_buf[IPC_RESULT_BUF_SIZE];
    memset(result_buf, 0, sizeof(result_buf));
    ipc_execute_command(req->cmd, result_buf, sizeof(result_buf));

    req->result  = strdup(result_buf[0] ? result_buf : "IPC call failed");
    req->success = (req->result != NULL) && (strstr(req->result, "failed") == NULL)
                   && (strstr(req->result, "error") == NULL)
                   && (strstr(req->result, "unknown") == NULL);

    if (!req->success) {
        wifi_util_error_print(WIFI_CTRL,
            "%s:%d command failed cmd='%s' result='%s'\n",
            __func__, __LINE__, req->cmd, result_buf);
    }

    /* Unblock bus callback thread */
    pthread_mutex_lock(&req->lock);
    req->done = true;
    pthread_cond_signal(&req->cond);
    pthread_mutex_unlock(&req->lock);
}
