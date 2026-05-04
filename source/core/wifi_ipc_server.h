/*
 * wifi_ipc_server.h — OneWifi server-side synchronous IPC
 *
 * External callers (OpenSync, etc.) invoke Device.WiFi.WiFiAPI.call.
 * The bus callback fires on a bus thread. All HAL must run on the
 * ctrl core thread. This module bridges the two with a condvar rendezvous.
 */
#ifndef WIFI_IPC_SERVER_H
#define WIFI_IPC_SERVER_H

#include <pthread.h>
#include <stdbool.h>
#include "bus.h"
#include "wifi_events.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Seconds the bus thread will wait before declaring timeout */
#define WIFI_IPC_SERVER_TIMEOUT_SEC  10

/*
 * Rendezvous struct — allocated on the bus callback thread,
 * pointer passed through the event queue to the ctrl core thread.
 */
typedef struct {
    char             cmd[256];   /* input: NUL-terminated command      */
    char            *result;     /* output: heap-alloc'd result string  */
    pthread_mutex_t  lock;
    pthread_cond_t   cond;
    bool             done;       /* ctrl thread sets true when complete */
    bool             success;    /* ctrl thread sets true on valid result */
} wifi_ipc_server_req_t;

/*
 * wifiapi_call_method() — bus method handler for Device.WiFi.WiFiAPI.call
 * Runs on bus callback thread. Posts event, blocks, returns result.
 */
bus_error_t wifiapi_call_method(char const *methodName,
                                bus_data_prop_t *inParams,
                                bus_data_prop_t *outParams,
                                void            *asyncHandle);

/*
 * wifi_ipc_server_handle_req() — ctrl core thread handler.
 * Called by handle_wifiapi_event() for wifi_event_type_wifiapi_ipc_call.
 * @data: sizeof(wifi_ipc_server_req_t*) bytes holding the req pointer.
 */
void wifi_ipc_server_handle_req(void *data, unsigned int len);

#ifdef __cplusplus
}
#endif
#endif /* WIFI_IPC_SERVER_H */
