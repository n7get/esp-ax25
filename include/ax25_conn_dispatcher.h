#ifndef AX25_CONN_DISPATCHER_H
#define AX25_CONN_DISPATCHER_H

#include "ax25_conn.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AX25_CONN_DISPATCH_ACTION_ERROR,
    AX25_CONN_DISPATCH_ACTION_DISCONNECT,
    AX25_CONN_DISPATCH_ACTION_LINK_RESET,
    AX25_CONN_DISPATCH_ACTION_CONNECT,
    AX25_CONN_DISPATCH_ACTION_FINAL,
    AX25_CONN_DISPATCH_ACTION_DATA,
    AX25_CONN_DISPATCH_ACTION_FRAME,
} ax25_conn_dispatch_action_type_t;

typedef struct {
    ax25_conn_dispatch_action_type_t type;
    union {
        ax25_conn_error_t error;
        struct {
            ax25_address_t remote_addr;
            bool is_local;
        } connect;
        struct {
            uint8_t data[AX25_MAX_INFO_LEN];
            size_t len;
        } data;
        ax25_frame_t frame;
    } payload;
} ax25_conn_dispatch_action_t;

esp_err_t ax25_conn_dispatcher_init_once(void);
esp_err_t ax25_conn_dispatcher_enqueue(ax25_conn_t* ctx,
                                       const ax25_conn_dispatch_action_t* action);
void ax25_conn_dispatcher_quiesce(ax25_conn_t* ctx);

#ifdef __cplusplus
}
#endif

#endif