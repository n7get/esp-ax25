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
 * @file ax25_monitor_tcp_server.h
 * @brief AX.25 monitor TCP server
 *
 * Each connected client receives every AX.25 frame seen by the router via a
 * dedicated promiscuous router port. Frames are KISS-encoded and sent to the
 * client inline within the router port callback; the PHY transport owns async
 * I/O isolation. Data received from a client is silently discarded.
 */

#ifndef AX25_MONITOR_TCP_SERVER_H
#define AX25_MONITOR_TCP_SERVER_H

#include <stdbool.h>
#include <stdint.h>

#include "ax25_frame.h"
#include "ax25_phy_tcp_server.h"
#include "ax25_router.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ax25_monitor_tcp_server_conn ax25_monitor_tcp_server_conn_t;
typedef struct ax25_monitor_tcp_server      ax25_monitor_tcp_server_t;

typedef void (*ax25_monitor_tcp_server_on_connected_t)(ax25_monitor_tcp_server_conn_t *conn);
typedef void (*ax25_monitor_tcp_server_on_disconnected_t)(ax25_monitor_tcp_server_conn_t *conn);

struct ax25_monitor_tcp_server_conn {
    ax25_monitor_tcp_server_t  *server;
    ax25_phy_tcp_server_conn_t *tcp_conn;
    ax25_router_port_t          router_port;
    volatile bool               in_use;
    volatile bool               running;
    void                       *user_data;
};

struct ax25_monitor_tcp_server {
    uint16_t                                  port;
    ax25_monitor_tcp_server_on_connected_t    on_connected;
    ax25_monitor_tcp_server_on_disconnected_t on_disconnected;
    void                                     *user_data;
    uint32_t                                  conn_task_stack_size;
    UBaseType_t                               conn_task_priority;
    volatile bool                             running;
    int                                       listen_sock;
    TaskHandle_t                              accept_task_handle;
    ax25_phy_tcp_server_t                     tcp_server;
    ax25_monitor_tcp_server_conn_t           *conns;
};

static inline void ax25_monitor_tcp_server_conn_set_user_data(
    ax25_monitor_tcp_server_conn_t *conn, void *user_data)
{
    conn->user_data = user_data;
}

static inline void *ax25_monitor_tcp_server_conn_get_user_data(
    const ax25_monitor_tcp_server_conn_t *conn)
{
    return conn->user_data;
}

static inline void *ax25_monitor_tcp_server_get_user_data(
    const ax25_monitor_tcp_server_conn_t *conn)
{
    return conn->server->user_data;
}

/**
 * @brief Initialise the monitor TCP server
 *
 * Starts a TCP listener using settings from ax25_config (for example,
 * net.monitor.port). When a client connects a promiscuous router port is
 * registered automatically; it is removed when the client disconnects.
 * Routed frames are KISS-encoded and sent to the client inline within the
 * router port callback.
 *
 * @param on_connected     Optional callback on client connect
 * @param on_disconnected  Optional callback on client disconnect
 * @param user_data        Opaque user pointer available via accessors
 * @param ctx              Caller-allocated server context to initialise
 * @return ESP_OK on success, or an error code
 */
esp_err_t ax25_monitor_tcp_server_init(
    ax25_monitor_tcp_server_on_connected_t on_connected,
    ax25_monitor_tcp_server_on_disconnected_t on_disconnected,
    void *user_data,
    ax25_monitor_tcp_server_t *ctx);

/**
 * @brief Stop and deinitialise the monitor TCP server
 *
 * Removes any active promiscuous router ports, closes all client connections,
 * and stops the listener task.
 *
 * @param ctx  Server context previously initialised with ax25_monitor_tcp_server_init()
 */
void ax25_monitor_tcp_server_deinit(ax25_monitor_tcp_server_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* AX25_MONITOR_TCP_SERVER_H */
