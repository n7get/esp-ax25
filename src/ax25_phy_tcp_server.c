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
 * @file ax25_phy_tcp_server.c
 * @brief Reusable TCP server transport
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "ax25_agwpe.h"
#include "ax25_config.h"
#include "ax25_kiss.h"
#include "ax25_phy_tcp_server.h"

static const char *TAG = "AX25_PHY_TCP_SRV";

static size_t get_default_max_clients_for_port_key(const char *port_key)
{
    if (port_key == NULL) {
        return 4;
    }

    if (strcmp(port_key, "net.log.port") == 0) {
        return (size_t)ax25_cfg_get_int_global("net.log.max_conns", 1);
    }
    if (strcmp(port_key, "net.monitor.port") == 0) {
        return (size_t)ax25_cfg_get_int_global("net.monitor.max_conns", 1);
    }
    if (strcmp(port_key, "net.agwpe.port") == 0) {
        return (size_t)ax25_cfg_get_int_global("net.agwpe.max_clients", 4);
    }

    return (size_t)ax25_cfg_get_int_global("net.kiss.max_conns", 4);
}

static uint32_t get_default_conn_task_stack_for_port_key(const char *port_key)
{
    if (port_key == NULL) {
        return (uint32_t)ax25_cfg_get_int_global("net.kiss.conn_task_stack", 0);
    }

    if (strcmp(port_key, "net.log.port") == 0) {
        return (uint32_t)ax25_cfg_get_int_global("net.log.conn_task_stack", 0);
    }
    if (strcmp(port_key, "net.monitor.port") == 0) {
        return (uint32_t)ax25_cfg_get_int_global("net.monitor.conn_task_stack", 0);
    }
    if (strcmp(port_key, "net.agwpe.port") == 0) {
        return (uint32_t)ax25_cfg_get_int_global("net.agwpe.conn_task_stack", 0);
    }

    return (uint32_t)ax25_cfg_get_int_global("net.kiss.conn_task_stack", 0);
}

static uint32_t get_min_conn_task_stack_for_port_key(const char *port_key)
{
    if (port_key == NULL) {
        return 0;
    }

    if (strcmp(port_key, "net.agwpe.port") == 0 ||
        strcmp(port_key, "net.kiss.port") == 0) {
        return 6144u;
    }

    return 0;
}

static uint32_t get_default_accept_task_stack_for_port_key(const char *port_key)
{
    if (port_key == NULL) {
        return 0;
    }

    if (strcmp(port_key, "net.kiss.port") == 0) {
        return (uint32_t)ax25_cfg_get_int_global("net.kiss.accept_task_stack", 0);
    }

    return 0;
}

#ifndef CONFIG_AX25_PHY_TCP_SERVER_ACCEPT_TASK_STACK_SIZE
#define CONFIG_AX25_PHY_TCP_SERVER_ACCEPT_TASK_STACK_SIZE 4096
#endif

#ifndef CONFIG_AX25_PHY_TCP_SERVER_CONN_TASK_STACK_SIZE
#define CONFIG_AX25_PHY_TCP_SERVER_CONN_TASK_STACK_SIZE 4096
#endif

#define AX25_PHY_TCP_SERVER_DEFAULT_TX_TASK_STACK_SIZE 3072u
#define AX25_PHY_TCP_SERVER_DEFAULT_TX_TASK_PRIORITY 5u
#define AX25_PHY_TCP_SERVER_DEFAULT_TX_QUEUE_DEPTH 8u
#define AX25_PHY_TCP_SERVER_TX_ITEM_CAPACITY \
    ((AX25_KISS_MAX_ENCODED_SIZE > AGWPE_MAX_FRAME_SIZE) ? AX25_KISS_MAX_ENCODED_SIZE : AGWPE_MAX_FRAME_SIZE)

typedef struct {
    size_t len;
    uint8_t data[AX25_PHY_TCP_SERVER_TX_ITEM_CAPACITY];
} ax25_phy_tcp_server_tx_item_t;

static void *tcp_server_calloc_prefer_psram(size_t count, size_t elem_size)
{
#if CONFIG_SPIRAM
    void *ptr = heap_caps_calloc(count, elem_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr != NULL) {
        return ptr;
    }
#endif

    return calloc(count, elem_size);
}

static StackType_t *tcp_server_alloc_task_stack_prefer_psram(uint32_t stack_size_bytes)
{
    size_t stack_bytes = ((size_t)stack_size_bytes + sizeof(StackType_t) - 1u) & ~(sizeof(StackType_t) - 1u);

#if CONFIG_SPIRAM
    StackType_t *stack = (StackType_t *)heap_caps_malloc(stack_bytes,
                                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (stack != NULL) {
        memset(stack, 0, stack_bytes);
        return stack;
    }
#endif

    return (StackType_t *)calloc(1, stack_bytes);
}

static StaticTask_t *tcp_server_alloc_task_tcb(void)
{
    return (StaticTask_t *)calloc(1, sizeof(StaticTask_t));
}

static void tcp_server_free_task_storage(StaticTask_t **task_tcb_storage,
                                         StackType_t **task_stack_storage)
{
    if (task_stack_storage != NULL && *task_stack_storage != NULL) {
        free(*task_stack_storage);
        *task_stack_storage = NULL;
    }
    if (task_tcb_storage != NULL && *task_tcb_storage != NULL) {
        free(*task_tcb_storage);
        *task_tcb_storage = NULL;
    }
}

static esp_err_t tcp_server_create_static_queue(size_t queue_depth,
                                                size_t item_size,
                                                QueueHandle_t *queue,
                                                StaticQueue_t **queue_state,
                                                uint8_t **queue_storage)
{
    if (queue == NULL || queue_state == NULL || queue_storage == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *queue = NULL;
    *queue_state = NULL;
    *queue_storage = NULL;

    *queue_state = tcp_server_calloc_prefer_psram(1, sizeof(**queue_state));
    if (*queue_state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    *queue_storage = tcp_server_calloc_prefer_psram(queue_depth, item_size);
    if (*queue_storage == NULL) {
        free(*queue_state);
        *queue_state = NULL;
        return ESP_ERR_NO_MEM;
    }

    *queue = xQueueCreateStatic((UBaseType_t)queue_depth,
                                (UBaseType_t)item_size,
                                *queue_storage,
                                *queue_state);
    if (*queue == NULL) {
        free(*queue_storage);
        free(*queue_state);
        *queue_storage = NULL;
        *queue_state = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static void tcp_server_destroy_static_queue(QueueHandle_t *queue,
                                            StaticQueue_t **queue_state,
                                            uint8_t **queue_storage)
{
    if (queue != NULL && *queue != NULL) {
        vQueueDelete(*queue);
        *queue = NULL;
    }
    if (queue_storage != NULL && *queue_storage != NULL) {
        free(*queue_storage);
        *queue_storage = NULL;
    }
    if (queue_state != NULL && *queue_state != NULL) {
        free(*queue_state);
        *queue_state = NULL;
    }
}

static void tcp_server_conn_recycle_tx_item(ax25_phy_tcp_server_conn_t *conn,
                                            ax25_phy_tcp_server_tx_item_t *item)
{
    if (conn == NULL || item == NULL || conn->tcp_tx_free_queue == NULL) {
        return;
    }

    item->len = 0;
    if (xQueueSend(conn->tcp_tx_free_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "tx item recycle failed for connection slot");
    }
}

static void tcp_server_conn_reset_tx_item_pool(ax25_phy_tcp_server_conn_t *conn)
{
    if (conn == NULL || conn->tcp_tx_item_pool == NULL || conn->tcp_tx_free_queue == NULL) {
        return;
    }

    ax25_phy_tcp_server_tx_item_t *item = NULL;
    while (conn->tcp_tx_queue != NULL && xQueueReceive(conn->tcp_tx_queue, &item, 0) == pdTRUE) {
        item = NULL;
    }
    while (xQueueReceive(conn->tcp_tx_free_queue, &item, 0) == pdTRUE) {
        item = NULL;
    }

    ax25_phy_tcp_server_tx_item_t *items = (ax25_phy_tcp_server_tx_item_t *)conn->tcp_tx_item_pool;
    for (size_t i = 0; i < conn->server->tx_queue_depth; i++) {
        items[i].len = 0;
        item = &items[i];
        if (xQueueSend(conn->tcp_tx_free_queue, &item, 0) != pdTRUE) {
            ESP_LOGE(TAG, "failed to seed tx item pool");
            break;
        }
    }
}

static void tcp_server_conn_flush_tx_queue(ax25_phy_tcp_server_conn_t *conn)
{
    if (conn == NULL || conn->tcp_tx_queue == NULL) {
        return;
    }

    ax25_phy_tcp_server_tx_item_t *item = NULL;
    while (xQueueReceive(conn->tcp_tx_queue, &item, 0) == pdTRUE) {
        tcp_server_conn_recycle_tx_item(conn, item);
        item = NULL;
    }
}

static void tcp_server_conn_tx_task(void *arg)
{
    ax25_phy_tcp_server_conn_t *conn = (ax25_phy_tcp_server_conn_t *)arg;
    ax25_phy_tcp_server_tx_item_t *item = NULL;

    while (conn->server->running && conn->running) {
        if (xQueueReceive(conn->tcp_tx_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        if (!conn->server->running || !conn->running) {
            tcp_server_conn_recycle_tx_item(conn, item);
            item = NULL;
            break;
        }

        xSemaphoreTake(conn->send_mutex, portMAX_DELAY);
        if (conn->sock < 0) {
            xSemaphoreGive(conn->send_mutex);
            tcp_server_conn_recycle_tx_item(conn, item);
            item = NULL;
            continue;
        }

        size_t offset = 0;
        while (conn->server->running && conn->running && offset < item->len) {
            ssize_t written = send(conn->sock,
                                   item->data + offset,
                                   item->len - offset,
                                   MSG_NOSIGNAL);
            if (written > 0) {
                offset += (size_t)written;
            } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                ESP_LOGE(TAG, "send failed: %s", strerror(errno));
                break;
            }
        }

        xSemaphoreGive(conn->send_mutex);
        tcp_server_conn_recycle_tx_item(conn, item);
        item = NULL;
    }

    conn->tcp_tx_task_handle = NULL;
    vTaskDelete(NULL);
}

static void tcp_server_conn_rx_task(void *arg)
{
    ax25_phy_tcp_server_conn_t *conn = (ax25_phy_tcp_server_conn_t *)arg;
    ax25_phy_tcp_server_t *srv = conn->server;
    uint8_t rx_buf[256];
    bool warned_low_stack = false;

    if (srv->on_connected) {
        srv->on_connected(conn);
    }

    while (srv->running) {
        if (!warned_low_stack) {
            UBaseType_t stack_words = uxTaskGetStackHighWaterMark(NULL);
            if (stack_words > 0 && stack_words < 128) {
                ESP_LOGW(TAG, "Low stack watermark in conn task slot=%d: %u words",
                         (int)(conn - srv->conns),
                         (unsigned)stack_words);
                warned_low_stack = true;
            }
        }

        ssize_t len = recv(conn->sock, rx_buf, sizeof(rx_buf), 0);
        if (len > 0) {
            if (srv->on_rx_data) {
                srv->on_rx_data(conn, rx_buf, (size_t)len);
            }
            continue;
        }
        if (len == 0) {
            ESP_LOGI(TAG, "Client disconnected");
            break;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
        }
        if (srv->running) {
            ESP_LOGW(TAG, "recv error: %s", strerror(errno));
        }
        break;
    }

    xSemaphoreTake(conn->send_mutex, portMAX_DELAY);
    int sock = conn->sock;
    conn->sock = -1;
    conn->running = false;
    xSemaphoreGive(conn->send_mutex);
    close(sock);

    for (int i = 0; i < 30 && conn->tcp_tx_task_handle != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (conn->tcp_tx_task_handle != NULL) {
        vTaskDelete(conn->tcp_tx_task_handle);
        conn->tcp_tx_task_handle = NULL;
    }

    tcp_server_conn_flush_tx_queue(conn);

    if (srv->on_disconnected) {
        srv->on_disconnected(conn);
    }

    conn->user_data = NULL;
    conn->in_use = false;
    conn->tcp_rx_task_handle = NULL;
    vTaskDelete(NULL);
}

static void accept_task(void *arg)
{
    ax25_phy_tcp_server_t *srv = (ax25_phy_tcp_server_t *)arg;
    bool warned_low_stack = false;

    while (srv->running) {
        if (!warned_low_stack) {
            UBaseType_t stack_words = uxTaskGetStackHighWaterMark(NULL);
            if (stack_words > 0 && stack_words < 128) {
                ESP_LOGW(TAG, "Low stack watermark in accept task: %u words",
                         (unsigned)stack_words);
                warned_low_stack = true;
            }
        }

        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int conn_sock = accept(srv->listen_sock,
                               (struct sockaddr *)&client_addr,
                               &client_addr_len);

        if (conn_sock < 0) {
            if (!srv->running || srv->listen_sock < 0 ||
                errno == EBADF || errno == ENOTSOCK) {
                break;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            if (srv->running) {
                ESP_LOGW(TAG, "accept failed: %s", strerror(errno));
            }
            continue;
        }

        ax25_phy_tcp_server_conn_t *conn = NULL;
        xSemaphoreTake(srv->conns_mutex, portMAX_DELAY);
        for (size_t i = 0; i < srv->max_clients; i++) {
            if (!srv->conns[i].in_use) {
                conn = &srv->conns[i];
                conn->in_use = true;
                break;
            }
        }
        xSemaphoreGive(srv->conns_mutex);

        if (conn == NULL) {
            ESP_LOGW(TAG, "Max clients (%d) reached, rejecting connection",
                     (int)srv->max_clients);
            close(conn_sock);
            continue;
        }

        struct timeval recv_tv = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(conn_sock, SOL_SOCKET, SO_RCVTIMEO,
                   &recv_tv, sizeof(recv_tv));

        struct timeval send_tv = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(conn_sock, SOL_SOCKET, SO_SNDTIMEO,
               &send_tv, sizeof(send_tv));

        conn->sock = conn_sock;
        conn->user_data = NULL;
        conn->running = true;

        tcp_server_conn_reset_tx_item_pool(conn);
        tcp_server_conn_flush_tx_queue(conn);

        ESP_LOGI(TAG, "Client connected [slot %d] from %s:%u",
                 (int)(conn - srv->conns),
                 inet_ntoa(client_addr.sin_addr),
                 (unsigned)ntohs(client_addr.sin_port));

        char task_name[20];
        snprintf(task_name, sizeof(task_name), "tcp_srv_%d",
                 (int)(conn - srv->conns));

        conn->tcp_tx_task_handle = xTaskCreateStatic(tcp_server_conn_tx_task,
                                                 "tcp_srv_tx",
                                                 srv->tx_task_stack_size / sizeof(StackType_t),
                                                 conn,
                                                 srv->tx_task_priority,
                                                 conn->tcp_tx_task_stack_storage,
                                                 conn->tcp_tx_task_tcb_storage);
        if (conn->tcp_tx_task_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create TX task for slot %d",
                     (int)(conn - srv->conns));
            close(conn_sock);
            conn->sock = -1;
            conn->running = false;
            conn->in_use = false;
            continue;
        }

        conn->tcp_rx_task_handle = xTaskCreateStatic(tcp_server_conn_rx_task,
                                              task_name,
                                              srv->conn_task_stack_size / sizeof(StackType_t),
                                              conn,
                                              srv->conn_task_priority,
                                              conn->tcp_rx_task_stack_storage,
                                              conn->tcp_rx_task_tcb_storage);
        if (conn->tcp_rx_task_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create connection task for slot %d",
                     (int)(conn - srv->conns));
            conn->running = false;
            for (int i = 0; i < 20 && conn->tcp_tx_task_handle != NULL; i++) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            if (conn->tcp_tx_task_handle != NULL) {
                vTaskDelete(conn->tcp_tx_task_handle);
                conn->tcp_tx_task_handle = NULL;
            }
            tcp_server_conn_flush_tx_queue(conn);
            close(conn_sock);
            conn->sock = -1;
            conn->in_use = false;
        }
    }

    srv->accept_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t ax25_phy_tcp_server_init(const char *port_key,
                                   ax25_phy_tcp_server_on_connected_t on_connected,
                                   ax25_phy_tcp_server_on_disconnected_t on_disconnected,
                                   ax25_phy_tcp_server_on_rx_data_t on_rx_data,
                                   void *user_data,
                                   ax25_phy_tcp_server_t *ctx)
{
    if (!ctx || !port_key || !on_rx_data) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(ctx, 0, sizeof(*ctx));

    /* Read port from ax25_config using protocol-specific key */
    uint16_t port = (uint16_t)ax25_cfg_get_int_global(port_key, 8001);
    if (port == 0) {
        ESP_LOGE(TAG, "TCP port from %s must not be 0", port_key);
        return ESP_ERR_INVALID_ARG;
    }

    ctx->max_clients = get_default_max_clients_for_port_key(port_key);
    if (ctx->max_clients < 1) {
        ctx->max_clients = 1;
    }
    ctx->conns = tcp_server_calloc_prefer_psram(ctx->max_clients,
                                                sizeof(ax25_phy_tcp_server_conn_t));
    if (ctx->conns == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ctx->port = port;
    ctx->on_connected = on_connected;
    ctx->on_disconnected = on_disconnected;
    ctx->on_rx_data = on_rx_data;
    ctx->user_data = user_data;
    ctx->conn_task_stack_size = get_default_conn_task_stack_for_port_key(port_key);
    if (ctx->conn_task_stack_size == 0) {
        ctx->conn_task_stack_size = (uint32_t)CONFIG_AX25_PHY_TCP_SERVER_CONN_TASK_STACK_SIZE;
    }
    uint32_t min_conn_task_stack = get_min_conn_task_stack_for_port_key(port_key);
    if (min_conn_task_stack > 0 && ctx->conn_task_stack_size < min_conn_task_stack) {
        ESP_LOGW(TAG,
                 "Clamping %s conn task stack from %u to %u",
                 port_key,
                 (unsigned)ctx->conn_task_stack_size,
                 (unsigned)min_conn_task_stack);
        ctx->conn_task_stack_size = min_conn_task_stack;
    }
    ctx->conn_task_priority = (UBaseType_t)ax25_cfg_get_int_global("net.kiss.conn_task_priority", 5);
    ctx->tx_task_stack_size = (uint32_t)ax25_cfg_get_int_global("net.kiss.tx_task_stack", 0);
    if (ctx->tx_task_stack_size == 0) {
        ctx->tx_task_stack_size = AX25_PHY_TCP_SERVER_DEFAULT_TX_TASK_STACK_SIZE;
    }
    ctx->tx_task_priority = (UBaseType_t)ax25_cfg_get_int_global("net.kiss.tx_task_priority", AX25_PHY_TCP_SERVER_DEFAULT_TX_TASK_PRIORITY);
    ctx->tx_queue_depth = (uint32_t)ax25_cfg_get_int_global("net.kiss.tx_queue_depth", AX25_PHY_TCP_SERVER_DEFAULT_TX_QUEUE_DEPTH);
    if (ctx->tx_queue_depth == 0) {
        ctx->tx_queue_depth = AX25_PHY_TCP_SERVER_DEFAULT_TX_QUEUE_DEPTH;
    }
    ctx->listen_sock = -1;

    uint32_t accept_stk = get_default_accept_task_stack_for_port_key(port_key);
    if (accept_stk == 0) {
        accept_stk = (uint32_t)CONFIG_AX25_PHY_TCP_SERVER_ACCEPT_TASK_STACK_SIZE;
    }
    UBaseType_t accept_prio = (UBaseType_t)ax25_cfg_get_int_global("net.kiss.accept_task_priority", 5);

    for (size_t i = 0; i < ctx->max_clients; i++) {
        ctx->conns[i].server = ctx;
        ctx->conns[i].sock = -1;
        ctx->conns[i].in_use = false;
        ctx->conns[i].running = false;
        ctx->conns[i].tcp_rx_task_handle = NULL;
        ctx->conns[i].tcp_rx_task_tcb_storage = NULL;
        ctx->conns[i].tcp_rx_task_stack_storage = NULL;
        ctx->conns[i].tcp_tx_task_handle = NULL;
        ctx->conns[i].tcp_tx_task_tcb_storage = NULL;
        ctx->conns[i].tcp_tx_task_stack_storage = NULL;
        ctx->conns[i].user_data = NULL;
        ctx->conns[i].tcp_tx_queue_state = NULL;
        ctx->conns[i].tcp_tx_queue_storage = NULL;
        ctx->conns[i].tcp_tx_free_queue_state = NULL;
        ctx->conns[i].tcp_tx_free_queue_storage = NULL;
        ctx->conns[i].tcp_tx_item_pool = NULL;
        ctx->conns[i].send_mutex = xSemaphoreCreateMutex();
        if (ctx->conns[i].send_mutex == NULL) {
            for (size_t j = 0; j < i; j++) {
                tcp_server_free_task_storage(&ctx->conns[j].tcp_rx_task_tcb_storage,
                                             &ctx->conns[j].tcp_rx_task_stack_storage);
                tcp_server_free_task_storage(&ctx->conns[j].tcp_tx_task_tcb_storage,
                                             &ctx->conns[j].tcp_tx_task_stack_storage);
                free(ctx->conns[j].tcp_tx_item_pool);
                ctx->conns[j].tcp_tx_item_pool = NULL;
                tcp_server_destroy_static_queue(&ctx->conns[j].tcp_tx_free_queue,
                                                &ctx->conns[j].tcp_tx_free_queue_state,
                                                &ctx->conns[j].tcp_tx_free_queue_storage);
                tcp_server_destroy_static_queue(&ctx->conns[j].tcp_tx_queue,
                                                &ctx->conns[j].tcp_tx_queue_state,
                                                &ctx->conns[j].tcp_tx_queue_storage);
                vSemaphoreDelete(ctx->conns[j].send_mutex);
                ctx->conns[j].send_mutex = NULL;
            }
            free(ctx->conns);
            ctx->conns = NULL;
            return ESP_ERR_NO_MEM;
        }

        ctx->conns[i].tcp_rx_task_tcb_storage = tcp_server_alloc_task_tcb();
        ctx->conns[i].tcp_rx_task_stack_storage = tcp_server_alloc_task_stack_prefer_psram(ctx->conn_task_stack_size);
        ctx->conns[i].tcp_tx_task_tcb_storage = tcp_server_alloc_task_tcb();
        ctx->conns[i].tcp_tx_task_stack_storage = tcp_server_alloc_task_stack_prefer_psram(ctx->tx_task_stack_size);
        if (ctx->conns[i].tcp_rx_task_tcb_storage == NULL ||
            ctx->conns[i].tcp_rx_task_stack_storage == NULL ||
            ctx->conns[i].tcp_tx_task_tcb_storage == NULL ||
            ctx->conns[i].tcp_tx_task_stack_storage == NULL) {
            tcp_server_free_task_storage(&ctx->conns[i].tcp_rx_task_tcb_storage,
                                         &ctx->conns[i].tcp_rx_task_stack_storage);
            tcp_server_free_task_storage(&ctx->conns[i].tcp_tx_task_tcb_storage,
                                         &ctx->conns[i].tcp_tx_task_stack_storage);
            vSemaphoreDelete(ctx->conns[i].send_mutex);
            ctx->conns[i].send_mutex = NULL;
            for (size_t j = 0; j < i; j++) {
                tcp_server_free_task_storage(&ctx->conns[j].tcp_rx_task_tcb_storage,
                                             &ctx->conns[j].tcp_rx_task_stack_storage);
                tcp_server_free_task_storage(&ctx->conns[j].tcp_tx_task_tcb_storage,
                                             &ctx->conns[j].tcp_tx_task_stack_storage);
                free(ctx->conns[j].tcp_tx_item_pool);
                ctx->conns[j].tcp_tx_item_pool = NULL;
                tcp_server_destroy_static_queue(&ctx->conns[j].tcp_tx_free_queue,
                                                &ctx->conns[j].tcp_tx_free_queue_state,
                                                &ctx->conns[j].tcp_tx_free_queue_storage);
                tcp_server_destroy_static_queue(&ctx->conns[j].tcp_tx_queue,
                                                &ctx->conns[j].tcp_tx_queue_state,
                                                &ctx->conns[j].tcp_tx_queue_storage);
                vSemaphoreDelete(ctx->conns[j].send_mutex);
                ctx->conns[j].send_mutex = NULL;
            }
            free(ctx->conns);
            ctx->conns = NULL;
            return ESP_ERR_NO_MEM;
        }
        if (tcp_server_create_static_queue(ctx->tx_queue_depth,
                                           sizeof(ax25_phy_tcp_server_tx_item_t *),
                                           &ctx->conns[i].tcp_tx_queue,
                                           &ctx->conns[i].tcp_tx_queue_state,
                                           &ctx->conns[i].tcp_tx_queue_storage) != ESP_OK) {
            tcp_server_free_task_storage(&ctx->conns[i].tcp_rx_task_tcb_storage,
                                         &ctx->conns[i].tcp_rx_task_stack_storage);
            tcp_server_free_task_storage(&ctx->conns[i].tcp_tx_task_tcb_storage,
                                         &ctx->conns[i].tcp_tx_task_stack_storage);
            vSemaphoreDelete(ctx->conns[i].send_mutex);
            ctx->conns[i].send_mutex = NULL;
            for (size_t j = 0; j < i; j++) {
                tcp_server_free_task_storage(&ctx->conns[j].tcp_rx_task_tcb_storage,
                                             &ctx->conns[j].tcp_rx_task_stack_storage);
                tcp_server_free_task_storage(&ctx->conns[j].tcp_tx_task_tcb_storage,
                                             &ctx->conns[j].tcp_tx_task_stack_storage);
                free(ctx->conns[j].tcp_tx_item_pool);
                ctx->conns[j].tcp_tx_item_pool = NULL;
                tcp_server_destroy_static_queue(&ctx->conns[j].tcp_tx_free_queue,
                                                &ctx->conns[j].tcp_tx_free_queue_state,
                                                &ctx->conns[j].tcp_tx_free_queue_storage);
                tcp_server_destroy_static_queue(&ctx->conns[j].tcp_tx_queue,
                                                &ctx->conns[j].tcp_tx_queue_state,
                                                &ctx->conns[j].tcp_tx_queue_storage);
                vSemaphoreDelete(ctx->conns[j].send_mutex);
                ctx->conns[j].send_mutex = NULL;
            }
            free(ctx->conns);
            ctx->conns = NULL;
            return ESP_ERR_NO_MEM;
        }
        if (tcp_server_create_static_queue(ctx->tx_queue_depth,
                                           sizeof(ax25_phy_tcp_server_tx_item_t *),
                                           &ctx->conns[i].tcp_tx_free_queue,
                                           &ctx->conns[i].tcp_tx_free_queue_state,
                                           &ctx->conns[i].tcp_tx_free_queue_storage) != ESP_OK) {
            tcp_server_destroy_static_queue(&ctx->conns[i].tcp_tx_queue,
                                            &ctx->conns[i].tcp_tx_queue_state,
                                            &ctx->conns[i].tcp_tx_queue_storage);
            tcp_server_free_task_storage(&ctx->conns[i].tcp_rx_task_tcb_storage,
                                         &ctx->conns[i].tcp_rx_task_stack_storage);
            tcp_server_free_task_storage(&ctx->conns[i].tcp_tx_task_tcb_storage,
                                         &ctx->conns[i].tcp_tx_task_stack_storage);
            vSemaphoreDelete(ctx->conns[i].send_mutex);
            ctx->conns[i].send_mutex = NULL;
            for (size_t j = 0; j < i; j++) {
                tcp_server_free_task_storage(&ctx->conns[j].tcp_rx_task_tcb_storage,
                                             &ctx->conns[j].tcp_rx_task_stack_storage);
                tcp_server_free_task_storage(&ctx->conns[j].tcp_tx_task_tcb_storage,
                                             &ctx->conns[j].tcp_tx_task_stack_storage);
                free(ctx->conns[j].tcp_tx_item_pool);
                ctx->conns[j].tcp_tx_item_pool = NULL;
                tcp_server_destroy_static_queue(&ctx->conns[j].tcp_tx_free_queue,
                                                &ctx->conns[j].tcp_tx_free_queue_state,
                                                &ctx->conns[j].tcp_tx_free_queue_storage);
                tcp_server_destroy_static_queue(&ctx->conns[j].tcp_tx_queue,
                                                &ctx->conns[j].tcp_tx_queue_state,
                                                &ctx->conns[j].tcp_tx_queue_storage);
                vSemaphoreDelete(ctx->conns[j].send_mutex);
                ctx->conns[j].send_mutex = NULL;
            }
            free(ctx->conns);
            ctx->conns = NULL;
            return ESP_ERR_NO_MEM;
        }
        ctx->conns[i].tcp_tx_item_pool = tcp_server_calloc_prefer_psram(ctx->tx_queue_depth,
                                            sizeof(ax25_phy_tcp_server_tx_item_t));
        if (ctx->conns[i].tcp_tx_item_pool == NULL) {
            tcp_server_destroy_static_queue(&ctx->conns[i].tcp_tx_free_queue,
                                            &ctx->conns[i].tcp_tx_free_queue_state,
                                            &ctx->conns[i].tcp_tx_free_queue_storage);
            tcp_server_destroy_static_queue(&ctx->conns[i].tcp_tx_queue,
                                            &ctx->conns[i].tcp_tx_queue_state,
                                            &ctx->conns[i].tcp_tx_queue_storage);
            tcp_server_free_task_storage(&ctx->conns[i].tcp_rx_task_tcb_storage,
                                         &ctx->conns[i].tcp_rx_task_stack_storage);
            tcp_server_free_task_storage(&ctx->conns[i].tcp_tx_task_tcb_storage,
                                         &ctx->conns[i].tcp_tx_task_stack_storage);
            vSemaphoreDelete(ctx->conns[i].send_mutex);
            ctx->conns[i].send_mutex = NULL;
            for (size_t j = 0; j < i; j++) {
                tcp_server_free_task_storage(&ctx->conns[j].tcp_rx_task_tcb_storage,
                                             &ctx->conns[j].tcp_rx_task_stack_storage);
                tcp_server_free_task_storage(&ctx->conns[j].tcp_tx_task_tcb_storage,
                                             &ctx->conns[j].tcp_tx_task_stack_storage);
                free(ctx->conns[j].tcp_tx_item_pool);
                ctx->conns[j].tcp_tx_item_pool = NULL;
                tcp_server_destroy_static_queue(&ctx->conns[j].tcp_tx_free_queue,
                                                &ctx->conns[j].tcp_tx_free_queue_state,
                                                &ctx->conns[j].tcp_tx_free_queue_storage);
                tcp_server_destroy_static_queue(&ctx->conns[j].tcp_tx_queue,
                                                &ctx->conns[j].tcp_tx_queue_state,
                                                &ctx->conns[j].tcp_tx_queue_storage);
                vSemaphoreDelete(ctx->conns[j].send_mutex);
                ctx->conns[j].send_mutex = NULL;
            }
            free(ctx->conns);
            ctx->conns = NULL;
            return ESP_ERR_NO_MEM;
        }
        tcp_server_conn_reset_tx_item_pool(&ctx->conns[i]);
    }

    ctx->conns_mutex = xSemaphoreCreateMutex();
    if (ctx->conns_mutex == NULL) {
        goto err_conns_mutex;
    }

    int lsock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (lsock < 0) {
        ESP_LOGE(TAG, "socket() failed: %s", strerror(errno));
        goto err_socket;
    }

    int yes = 1;
    setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct timeval accept_tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(lsock, SOL_SOCKET, SO_RCVTIMEO, &accept_tv, sizeof(accept_tv));

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(ctx->port),
    };
    if (bind(lsock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "bind() on port %u failed: %s",
                 (unsigned)ctx->port, strerror(errno));
        close(lsock);
        goto err_socket;
    }

    if (listen(lsock, (int)ctx->max_clients) < 0) {
        ESP_LOGE(TAG, "listen() failed: %s", strerror(errno));
        close(lsock);
        goto err_socket;
    }

    ctx->listen_sock = lsock;
    ESP_LOGI(TAG, "Listening on port %u (max %d clients)",
             (unsigned)ctx->port, (int)ctx->max_clients);

    ctx->running = true;

    if (xTaskCreate(accept_task, "tcp_srv_accept",
                    accept_stk, ctx, accept_prio,
                    &ctx->accept_task_handle) != pdPASS) {
        ctx->running = false;
        close(lsock);
        ctx->listen_sock = -1;
        goto err_socket;
    }

    return ESP_OK;

err_socket:
    vSemaphoreDelete(ctx->conns_mutex);
    ctx->conns_mutex = NULL;
err_conns_mutex:
    for (size_t i = 0; i < ctx->max_clients; i++) {
        if (ctx->conns[i].tcp_tx_free_queue) {
            tcp_server_destroy_static_queue(&ctx->conns[i].tcp_tx_free_queue,
                                            &ctx->conns[i].tcp_tx_free_queue_state,
                                            &ctx->conns[i].tcp_tx_free_queue_storage);
        }
        tcp_server_free_task_storage(&ctx->conns[i].tcp_rx_task_tcb_storage,
                                     &ctx->conns[i].tcp_rx_task_stack_storage);
        tcp_server_free_task_storage(&ctx->conns[i].tcp_tx_task_tcb_storage,
                                     &ctx->conns[i].tcp_tx_task_stack_storage);
        free(ctx->conns[i].tcp_tx_item_pool);
        ctx->conns[i].tcp_tx_item_pool = NULL;
        tcp_server_destroy_static_queue(&ctx->conns[i].tcp_tx_queue,
                                        &ctx->conns[i].tcp_tx_queue_state,
                                        &ctx->conns[i].tcp_tx_queue_storage);
        if (ctx->conns[i].send_mutex) {
            vSemaphoreDelete(ctx->conns[i].send_mutex);
            ctx->conns[i].send_mutex = NULL;
        }
    }
    free(ctx->conns);
    ctx->conns = NULL;
    ctx->max_clients = 0;
    return ESP_ERR_NO_MEM;
}

void ax25_phy_tcp_server_deinit(ax25_phy_tcp_server_t *ctx)
{
    if (!ctx || !ctx->running) {
        return;
    }

    ctx->running = false;

    for (size_t i = 0; i < ctx->max_clients; i++) {
        xSemaphoreTake(ctx->conns[i].send_mutex, portMAX_DELAY);
        int sock = ctx->conns[i].sock;
        ctx->conns[i].running = false;
        xSemaphoreGive(ctx->conns[i].send_mutex);
        if (sock >= 0) {
            shutdown(sock, SHUT_RDWR);
        }
    }

    if (ctx->listen_sock >= 0) {
        shutdown(ctx->listen_sock, SHUT_RDWR);
        close(ctx->listen_sock);
        ctx->listen_sock = -1;
    }

    for (int i = 0; i < 30 && ctx->accept_task_handle != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (ctx->accept_task_handle != NULL) {
        vTaskDelete(ctx->accept_task_handle);
        ctx->accept_task_handle = NULL;
    }

    for (size_t i = 0; i < ctx->max_clients; i++) {
        for (int j = 0; j < 30 && ctx->conns[i].tcp_rx_task_handle != NULL; j++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (ctx->conns[i].tcp_rx_task_handle != NULL) {
            vTaskDelete(ctx->conns[i].tcp_rx_task_handle);
            ctx->conns[i].tcp_rx_task_handle = NULL;
        }
        for (int j = 0; j < 30 && ctx->conns[i].tcp_tx_task_handle != NULL; j++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (ctx->conns[i].tcp_tx_task_handle != NULL) {
            vTaskDelete(ctx->conns[i].tcp_tx_task_handle);
            ctx->conns[i].tcp_tx_task_handle = NULL;
        }
        tcp_server_conn_flush_tx_queue(&ctx->conns[i]);
    }

    vSemaphoreDelete(ctx->conns_mutex);
    ctx->conns_mutex = NULL;

    for (size_t i = 0; i < ctx->max_clients; i++) {
        tcp_server_destroy_static_queue(&ctx->conns[i].tcp_tx_free_queue,
                                        &ctx->conns[i].tcp_tx_free_queue_state,
                                        &ctx->conns[i].tcp_tx_free_queue_storage);
        tcp_server_free_task_storage(&ctx->conns[i].tcp_rx_task_tcb_storage,
                                     &ctx->conns[i].tcp_rx_task_stack_storage);
        tcp_server_free_task_storage(&ctx->conns[i].tcp_tx_task_tcb_storage,
                                     &ctx->conns[i].tcp_tx_task_stack_storage);
        free(ctx->conns[i].tcp_tx_item_pool);
        ctx->conns[i].tcp_tx_item_pool = NULL;
        tcp_server_destroy_static_queue(&ctx->conns[i].tcp_tx_queue,
                                        &ctx->conns[i].tcp_tx_queue_state,
                                        &ctx->conns[i].tcp_tx_queue_storage);
        vSemaphoreDelete(ctx->conns[i].send_mutex);
        ctx->conns[i].send_mutex = NULL;
    }

    free(ctx->conns);
    ctx->conns = NULL;
    ctx->max_clients = 0;
}

esp_err_t ax25_phy_tcp_server_conn_send(ax25_phy_tcp_server_conn_t *conn,
                                        const uint8_t *data,
                                        size_t len)
{
    if (!conn || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (len > AX25_PHY_TCP_SERVER_TX_ITEM_CAPACITY) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (conn->send_mutex == NULL || conn->tcp_tx_queue == NULL ||
        conn->tcp_tx_free_queue == NULL || conn->tcp_tx_item_pool == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(conn->send_mutex, portMAX_DELAY);

    if (conn->sock < 0 || !conn->running || !conn->server->running) {
        xSemaphoreGive(conn->send_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreGive(conn->send_mutex);

    ax25_phy_tcp_server_tx_item_t *item = NULL;
    if (xQueueReceive(conn->tcp_tx_free_queue, &item, 0) != pdTRUE || item == NULL) {
        return ESP_ERR_NO_MEM;
    }

    item->len = len;
    memcpy(item->data, data, len);

    if (xQueueSend(conn->tcp_tx_queue, &item, 0) != pdTRUE) {
        tcp_server_conn_recycle_tx_item(conn, item);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
