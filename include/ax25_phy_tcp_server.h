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
 * @file ax25_phy_tcp_server.h
 * @brief Reusable TCP server transport
 */

#ifndef AX25_PHY_TCP_SERVER_H
#define AX25_PHY_TCP_SERVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ax25_phy_tcp_server_conn ax25_phy_tcp_server_conn_t;
typedef struct ax25_phy_tcp_server      ax25_phy_tcp_server_t;

typedef void (*ax25_phy_tcp_server_on_connected_t)(ax25_phy_tcp_server_conn_t *conn);
typedef void (*ax25_phy_tcp_server_on_disconnected_t)(ax25_phy_tcp_server_conn_t *conn);
typedef void (*ax25_phy_tcp_server_on_rx_data_t)(ax25_phy_tcp_server_conn_t *conn,
                                                 const uint8_t *data,
                                                 size_t len);

typedef struct {
    uint16_t port;
    ax25_phy_tcp_server_on_connected_t    on_connected;
    ax25_phy_tcp_server_on_disconnected_t on_disconnected;
    ax25_phy_tcp_server_on_rx_data_t      on_rx_data;
    void                                 *user_data;
    uint32_t                              accept_task_stack_size;
    UBaseType_t                           accept_task_priority;
    uint32_t                              conn_task_stack_size;
    UBaseType_t                           conn_task_priority;
} ax25_phy_tcp_server_config_t;

struct ax25_phy_tcp_server_conn {
    ax25_phy_tcp_server_t *server;
    int                    sock;
    volatile bool          in_use;
    volatile bool          running;
    TaskHandle_t           tcp_rx_task_handle;
    StaticTask_t          *tcp_rx_task_tcb_storage;
    StackType_t           *tcp_rx_task_stack_storage;
    TaskHandle_t           tcp_tx_task_handle;
    StaticTask_t          *tcp_tx_task_tcb_storage;
    StackType_t           *tcp_tx_task_stack_storage;
    SemaphoreHandle_t      send_mutex;
    QueueHandle_t          tcp_tx_queue;
    StaticQueue_t         *tcp_tx_queue_state;
    uint8_t               *tcp_tx_queue_storage;
    QueueHandle_t          tcp_tx_free_queue;
    StaticQueue_t         *tcp_tx_free_queue_state;
    uint8_t               *tcp_tx_free_queue_storage;
    void                  *tcp_tx_item_pool;
    void                  *user_data;
};

struct ax25_phy_tcp_server {
    uint16_t                               port;
    ax25_phy_tcp_server_on_connected_t     on_connected;
    ax25_phy_tcp_server_on_disconnected_t  on_disconnected;
    ax25_phy_tcp_server_on_rx_data_t       on_rx_data;
    void                                  *user_data;
    uint32_t                               conn_task_stack_size;
    UBaseType_t                            conn_task_priority;
    uint32_t                               tx_task_stack_size;
    UBaseType_t                            tx_task_priority;
    uint32_t                               tx_queue_depth;
    volatile bool                          running;
    int                                    listen_sock;
    TaskHandle_t                           accept_task_handle;
    SemaphoreHandle_t                      conns_mutex;
    size_t                                 max_clients;   /* set by init */
    ax25_phy_tcp_server_conn_t            *conns;         /* heap-allocated */
};

static inline void ax25_phy_tcp_server_conn_set_user_data(
    ax25_phy_tcp_server_conn_t *conn, void *user_data)
{
    conn->user_data = user_data;
}

static inline void *ax25_phy_tcp_server_conn_get_user_data(
    const ax25_phy_tcp_server_conn_t *conn)
{
    return conn->user_data;
}

static inline void *ax25_phy_tcp_server_get_user_data(
    const ax25_phy_tcp_server_conn_t *conn)
{
    return conn->server->user_data;
}

esp_err_t ax25_phy_tcp_server_init(const char *port_key,
                                   ax25_phy_tcp_server_on_connected_t on_connected,
                                   ax25_phy_tcp_server_on_disconnected_t on_disconnected,
                                   ax25_phy_tcp_server_on_rx_data_t on_rx_data,
                                   void *user_data,
                                   ax25_phy_tcp_server_t *ctx);

void ax25_phy_tcp_server_deinit(ax25_phy_tcp_server_t *ctx);

esp_err_t ax25_phy_tcp_server_conn_send(ax25_phy_tcp_server_conn_t *conn,
                                        const uint8_t *data,
                                        size_t len);

#ifdef __cplusplus
}
#endif

#endif /* AX25_PHY_TCP_SERVER_H */
