//
//    Copyright (C) 2026 Robert Ambrose N7GET
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

/**
 * @file ax25_kiss_tcp_server.h
 * @brief KISS-over-TCP server built on ax25_phy_tcp_server
 *
 * Outbound AX.25 frames submitted via ax25_kiss_tcp_server_conn_send() are
 * KISS-encoded and forwarded directly to the underlying PHY transport within
 * the caller's task context.  The PHY transport owns async I/O isolation.
 */

#ifndef AX25_KISS_TCP_SERVER_H
#define AX25_KISS_TCP_SERVER_H

#include <stdbool.h>
#include <stdint.h>

#include "ax25_frame.h"
#include "ax25_kiss.h"
#include "ax25_phy_tcp_server.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ax25_kiss_tcp_server_conn ax25_kiss_tcp_server_conn_t;
typedef struct ax25_kiss_tcp_server      ax25_kiss_tcp_server_t;

typedef void (*ax25_kiss_tcp_server_on_connected_t)(ax25_kiss_tcp_server_conn_t *conn);
typedef void (*ax25_kiss_tcp_server_on_disconnected_t)(ax25_kiss_tcp_server_conn_t *conn);
typedef void (*ax25_kiss_tcp_server_on_rx_frame_t)(ax25_kiss_tcp_server_conn_t *conn,
                                                    const ax25_frame_t *frame);



struct ax25_kiss_tcp_server_conn {
    ax25_kiss_tcp_server_t *server;
    ax25_phy_tcp_server_conn_t *tcp_conn;
    ax25_kiss_decoder_t       kiss_decoder;
    volatile bool             in_use;
    volatile bool             running;
    void                     *user_data;
};

struct ax25_kiss_tcp_server {
    uint16_t                               port;
    ax25_kiss_tcp_server_on_connected_t    on_connected;
    ax25_kiss_tcp_server_on_disconnected_t on_disconnected;
    ax25_kiss_tcp_server_on_rx_frame_t     on_rx_frame;
    void                                  *user_data;
    uint32_t                               conn_task_stack_size;
    UBaseType_t                            conn_task_priority;
    volatile bool                          running;
    int                                    listen_sock;
    TaskHandle_t                           accept_task_handle;
    ax25_phy_tcp_server_t                  tcp_server;
    ax25_kiss_tcp_server_conn_t           *conns;
};

static inline void ax25_kiss_tcp_server_conn_set_user_data(
    ax25_kiss_tcp_server_conn_t *conn, void *user_data)
{
    conn->user_data = user_data;
}

static inline void *ax25_kiss_tcp_server_conn_get_user_data(
    const ax25_kiss_tcp_server_conn_t *conn)
{
    return conn->user_data;
}

static inline void *ax25_kiss_tcp_server_get_user_data(
    const ax25_kiss_tcp_server_conn_t *conn)
{
    return conn->server->user_data;
}

esp_err_t ax25_kiss_tcp_server_init(ax25_kiss_tcp_server_on_connected_t on_connected,
                                    ax25_kiss_tcp_server_on_disconnected_t on_disconnected,
                                    ax25_kiss_tcp_server_on_rx_frame_t on_rx_frame,
                                    void *user_data,
                                    ax25_kiss_tcp_server_t *ctx);

void ax25_kiss_tcp_server_deinit(ax25_kiss_tcp_server_t *ctx);

esp_err_t ax25_kiss_tcp_server_conn_send(ax25_kiss_tcp_server_conn_t *conn,
                                         const ax25_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* AX25_KISS_TCP_SERVER_H */
