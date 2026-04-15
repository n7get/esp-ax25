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
 * @file ax25_agwpe_server.c
 * @brief AGWPE server implementation for esp-ax25
 *
 * Bridges a single AGWPE TCP client to the AX.25 stack.  Handles AGWPE
 * protocol commands from the client, manages AX.25 connections via
 * ax25_conn, and routes frames through ax25_router.
 *
 * ## Implemented AGWPE frame types (based on direwolf server.c reference)
 *
 * ### From client:
 *  'P'  Application Login          — silently accepted
 *  'R'  Version request            — replies with version 2005.127
 *  'G'  Port information request   — replies with port description
 *  'g'  Port capabilities request  — replies with default capabilities
 *  'X'  Register callsign          — stores callsign, replies with success
 *  'x'  Unregister callsign        — removes callsign
 *  'k'  Toggle raw frame reception — toggles raw monitoring
 *  'm'  Toggle monitor reception   — toggles monitor mode
 *  'K'  Send raw AX.25 frame       — parses and routes via ax25_router
 *  'V'  Send UI via digipeaters    — builds UI frame and routes
 *  'M'  Send UI (no digipeaters)   — builds UI frame and routes
 *  'C'  Connect request            — initiates ax25_conn connection
 *  'v'  Connect via digipeaters    — initiates connection (digis stored)
 *  'c'  Connect with custom PID    — initiates connection with PID
 *  'D'  Send connected data        — sends via matching ax25_conn
 *  'd'  Disconnect request         — shuts down matching ax25_conn
 *  'y'  Outstanding on port        — replies with 0 (simplified)
 *  'Y'  Outstanding on connection  — replies with pending frame count
 *  'H'  Heard stations request     — not implemented (no-op)
 *
 * ### To client:
 *  'R'  Version response
 *  'G'  Port information response
 *  'g'  Port capabilities response
 *  'X'  Register callsign response
 *  'K'  Raw received frame         (when raw enabled)
 *  'U'  Monitored UI frame         (when monitor enabled)
 *  'I'  Monitored I frame          (when monitor enabled)
 *  'S'  Monitored supervisory      (when monitor enabled)
 *  'C'  Connection established
 *  'D'  Connected data received
 *  'd'  Disconnected
 *  'y'  Outstanding frames on port
 *  'Y'  Outstanding frames on connection
 */

#include "ax25_agwpe_server.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "ax25_agwpe.h"
#include "ax25_frame.h"
#include "ax25_address.h"
#include "ax25_config.h"
#include "ax25_conn.h"
#include "ax25_print.h"
#include "ax25_router.h"

#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "agwpe_srv";

#ifdef CONFIG_AX25_AGWPE_LOG_ROUTER_FRAMES
#define AX25_AGWPE_LOG_ROUTER_FRAMES CONFIG_AX25_AGWPE_LOG_ROUTER_FRAMES
#else
#define AX25_AGWPE_LOG_ROUTER_FRAMES 0
#endif

static void agwpe_log_router_frame(const char *label, const ax25_frame_t *frame)
{
#if AX25_AGWPE_LOG_ROUTER_FRAMES
    ax25_print_frame(label, frame);
#else
    (void)label;
    (void)frame;
#endif
}

typedef struct {
    bool initialized;
    SemaphoreHandle_t mutex;
    StaticSemaphore_t mutex_buf;
    ax25_router_port_t router_port;
    ax25_agwpe_server_t *clients[AX25_AGWPE_SERVER_MAX_CLIENTS];
    uint16_t client_refcnt[AX25_AGWPE_SERVER_MAX_CLIENTS];
    bool client_pending_remove[AX25_AGWPE_SERVER_MAX_CLIENTS];
} agwpe_manager_state_t;

static agwpe_manager_state_t s_manager;

static void manager_router_on_frame(const ax25_frame_t *frame, void *user_data);

static void *agwpe_alloc_prefer_psram(size_t size, bool *used_psram)
{
    if (used_psram != NULL) {
        *used_psram = false;
    }

#if defined(CONFIG_SPIRAM) && CONFIG_SPIRAM
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr != NULL) {
        if (used_psram != NULL) {
            *used_psram = true;
        }
        return ptr;
    }
#endif

    return malloc(size);
}

static void log_heap_integrity_checkpoint(const char *stage)
{
    bool ok = heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG,
             "Heap integrity at %s: %s",
             stage != NULL ? stage : "<unknown>",
             ok ? "ok" : "FAILED");
}

/*******************************************************************************
 * Forward declarations – internal helpers
 ******************************************************************************/

/* Queue an AGWPE frame for sending to the client */
static esp_err_t enqueue_agwpe_frame(ax25_agwpe_server_t *srv,
                                      const agwpe_frame_t *frame);

/* FreeRTOS task that drains the TX queue */
static void tx_task_func(void *arg);

/* Router callback (promiscuous port) */
static void router_on_frame(const ax25_frame_t *frame, void *user_data);

/* ax25_conn callbacks */
static void conn_on_connect(ax25_address_t remote_addr,
                            bool is_local_initiated, void *user_data);
static void conn_on_disconnect(void *user_data);
static void conn_on_data(const uint8_t *data, size_t len, void *user_data);
static void conn_on_frame(const ax25_frame_t *frame, void *user_data);
static void conn_on_error(const ax25_conn_error_t *error, void *user_data);
static void conn_router_on_frame(const ax25_frame_t *frame, void *user_data);

/* Slot management */
static agwpe_conn_slot_t *find_conn_slot(ax25_agwpe_server_t *srv,
                                          const char *local_call,
                                          const char *remote_call);
static agwpe_conn_slot_t *find_conn_slot_any_state(ax25_agwpe_server_t *srv,
                                                   const char *local_call,
                                                   const char *remote_call);
static agwpe_conn_slot_t *alloc_conn_slot(ax25_agwpe_server_t *srv);
static void free_conn_slot(ax25_agwpe_server_t *srv, agwpe_conn_slot_t *slot);
static void sweep_released_conn_slots(ax25_agwpe_server_t *srv);
static int conn_slot_index(ax25_agwpe_server_t *srv, const agwpe_conn_slot_t *slot);
static esp_err_t register_conn_router_port(ax25_agwpe_server_t *srv,
                                           agwpe_conn_slot_t *slot,
                                           const char *local_call,
                                           conn_cb_ctx_t *cb_ctx);
static void unregister_conn_router_port(agwpe_conn_slot_t *slot);
static bool resolve_valid_conn_callback_ctx(conn_cb_ctx_t *ctx,
                                            ax25_agwpe_server_t **srv_out,
                                            agwpe_conn_slot_t **slot_out);
/* AGWPE command handlers (from client) */
static void handle_version_req(ax25_agwpe_server_t *srv,
                                const agwpe_frame_t *frame);
static void handle_port_info_req(ax25_agwpe_server_t *srv,
                                  const agwpe_frame_t *frame);
static void handle_port_cap_req(ax25_agwpe_server_t *srv,
                                 const agwpe_frame_t *frame);
static void handle_register_call(ax25_agwpe_server_t *srv,
                                  const agwpe_frame_t *frame);
static void handle_unregister_call(ax25_agwpe_server_t *srv,
                                    const agwpe_frame_t *frame);
static void handle_enable_raw(ax25_agwpe_server_t *srv,
                               const agwpe_frame_t *frame);
static void handle_enable_monitor(ax25_agwpe_server_t *srv,
                                   const agwpe_frame_t *frame);
static void handle_send_raw(ax25_agwpe_server_t *srv,
                             const agwpe_frame_t *frame);
static void handle_send_unproto(ax25_agwpe_server_t *srv,
                                 const agwpe_frame_t *frame);
static void handle_send_unproto_via(ax25_agwpe_server_t *srv,
                                     const agwpe_frame_t *frame);
static void handle_connect(ax25_agwpe_server_t *srv,
                            const agwpe_frame_t *frame);
static void handle_connect_via(ax25_agwpe_server_t *srv,
                                const agwpe_frame_t *frame);
static void handle_connect_pid(ax25_agwpe_server_t *srv,
                                const agwpe_frame_t *frame);
static void handle_send_data(ax25_agwpe_server_t *srv,
                              const agwpe_frame_t *frame);
static void handle_disconnect(ax25_agwpe_server_t *srv,
                               const agwpe_frame_t *frame);
static void handle_outstanding_port(ax25_agwpe_server_t *srv,
                                     const agwpe_frame_t *frame);
static void handle_outstanding_conn(ax25_agwpe_server_t *srv,
                                     const agwpe_frame_t *frame);

/* AX.25 → AGWPE monitoring helpers */
static void send_raw_to_client(ax25_agwpe_server_t *srv,
                                const ax25_frame_t *frame);
static void send_monitor_to_client(ax25_agwpe_server_t *srv,
                                    const ax25_frame_t *frame);
static void deliver_connected_data(ax25_agwpe_server_t *srv,
                                    agwpe_conn_slot_t *slot,
                                    const uint8_t *data, size_t len);
static esp_err_t parse_agwpe_digipeater_path(const uint8_t *data,
                                             size_t data_len,
                                             ax25_address_t *digipeaters,
                                             uint8_t *num_digipeaters,
                                             size_t *consumed_len);

static bool manager_has_client_nolock(const ax25_agwpe_server_t *client)
{
    for (int i = 0; i < AX25_AGWPE_SERVER_MAX_CLIENTS; i++) {
        if (s_manager.clients[i] == client && !s_manager.client_pending_remove[i]) {
            return true;
        }
    }
    return false;
}

/*******************************************************************************
 * Lifecycle
 ******************************************************************************/

static esp_err_t ax25_agwpe_server_init_internal(const ax25_agwpe_server_config_t *config,
                                                 ax25_agwpe_server_t *srv,
                                                 bool register_router_port)
{
    if (config == NULL || srv == NULL || config->on_agwpe_frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Zero the caller-provided storage */
    memset(srv, 0, sizeof(*srv));

    srv->max_conns = (size_t)ax25_cfg_get_int_global("net.agwpe.server.max_conns", 1);
    if (srv->max_conns < 1) {
        srv->max_conns = 1;
    }

    uint16_t tx_queue_depth = config->tx_queue_depth > 0
                                  ? config->tx_queue_depth
                                  : AX25_AGWPE_SERVER_TX_QUEUE_DEPTH;
    if (tx_queue_depth > AX25_AGWPE_SERVER_TX_QUEUE_DEPTH) {
        ESP_LOGW(TAG,
                 "Requested tx_queue_depth=%u exceeds compile-time max %u; capping",
                 (unsigned)tx_queue_depth,
                 (unsigned)AX25_AGWPE_SERVER_TX_QUEUE_DEPTH);
        tx_queue_depth = AX25_AGWPE_SERVER_TX_QUEUE_DEPTH;
    }

    uint32_t task_stack_size = config->task_stack_size > 0
                                   ? config->task_stack_size
                                   : AX25_AGWPE_SERVER_TASK_STACK_SIZE;
    if (task_stack_size > AX25_AGWPE_SERVER_TASK_STACK_SIZE) {
        ESP_LOGW(TAG,
                 "Requested task_stack_size=%lu exceeds compile-time max %u; capping",
                 (unsigned long)task_stack_size,
                 (unsigned)AX25_AGWPE_SERVER_TASK_STACK_SIZE);
        task_stack_size = AX25_AGWPE_SERVER_TASK_STACK_SIZE;
    }

    const size_t tx_storage_size = tx_queue_depth * sizeof(agwpe_frame_t);

    srv->tx_queue_storage = (uint8_t *)agwpe_alloc_prefer_psram(
        tx_storage_size, &srv->tx_queue_storage_psram);
    if (srv->tx_queue_storage == NULL) {
        return ESP_ERR_NO_MEM;
    }

    srv->monitor_pool = (monitor_buf_t *)agwpe_alloc_prefer_psram(
        AX25_AGWPE_SERVER_MONITOR_POOL_SIZE * sizeof(monitor_buf_t),
        &srv->monitor_pool_psram);
    if (srv->monitor_pool == NULL) {
        free(srv->tx_queue_storage);
        srv->tx_queue_storage = NULL;
        return ESP_ERR_NO_MEM;
    }

    srv->conns = (agwpe_conn_slot_t *)agwpe_alloc_prefer_psram(
        srv->max_conns * sizeof(agwpe_conn_slot_t),
        NULL);
    if (srv->conns == NULL) {
        free(srv->monitor_pool);
        srv->monitor_pool = NULL;
        free(srv->tx_queue_storage);
        srv->tx_queue_storage = NULL;
        return ESP_ERR_NO_MEM;
    }
    memset(srv->conns, 0, srv->max_conns * sizeof(agwpe_conn_slot_t));

    srv->cb_contexts = (conn_cb_ctx_t *)agwpe_alloc_prefer_psram(
        srv->max_conns * sizeof(conn_cb_ctx_t),
        &srv->cb_contexts_psram);
    if (srv->cb_contexts == NULL) {
        free(srv->conns);
        srv->conns = NULL;
        free(srv->monitor_pool);
        srv->monitor_pool = NULL;
        free(srv->tx_queue_storage);
        srv->tx_queue_storage = NULL;
        return ESP_ERR_NO_MEM;
    }
    memset(srv->cb_contexts, 0, srv->max_conns * sizeof(conn_cb_ctx_t));

    /* Copy configuration */
    srv->on_agwpe_frame = config->on_agwpe_frame;
    srv->user_data      = config->user_data;
    srv->port           = config->port;

    if (config->port_description != NULL) {
        snprintf(srv->port_desc, sizeof(srv->port_desc), "%s",
                 config->port_description);
    } else {
        snprintf(srv->port_desc, sizeof(srv->port_desc),
                 "Port%d esp-ax25 radio", srv->port + 1);
    }

    /* Create mutex (static) */
    srv->mutex = xSemaphoreCreateMutexStatic(&srv->mutex_buf);
    if (srv->mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Create outgoing AGWPE frame queue (static) */
    memset(srv->tx_queue_storage, 0, tx_storage_size);
    srv->tx_queue = xQueueCreateStatic(
        tx_queue_depth,
        sizeof(agwpe_frame_t),
        srv->tx_queue_storage,
        &srv->tx_queue_buf);
    if (srv->tx_queue == NULL) {
        free(srv->cb_contexts);
        srv->cb_contexts = NULL;
        free(srv->monitor_pool);
        srv->monitor_pool = NULL;
        free(srv->tx_queue_storage);
        srv->tx_queue_storage = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Create TX sender task (static) */
    UBaseType_t prio = config->task_priority > 0
                           ? config->task_priority
                           : AX25_AGWPE_SERVER_TASK_PRIORITY;
    srv->tx_task = xTaskCreateStatic(
        tx_task_func, "agwpe_tx",
        task_stack_size / sizeof(StackType_t),
        srv, prio,
        srv->tx_task_stack,
        &srv->tx_task_tcb);
    if (srv->tx_task == NULL) {
        vQueueDelete(srv->tx_queue);
        srv->tx_queue = NULL;
        free(srv->conns);
        srv->conns = NULL;
        free(srv->cb_contexts);
        srv->cb_contexts = NULL;
        free(srv->monitor_pool);
        srv->monitor_pool = NULL;
        free(srv->tx_queue_storage);
        srv->tx_queue_storage = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Initialise monitor output buffer pool */
    srv->monitor_pool_sem = xSemaphoreCreateCountingStatic(
        AX25_AGWPE_SERVER_MONITOR_POOL_SIZE,
        AX25_AGWPE_SERVER_MONITOR_POOL_SIZE,
        &srv->monitor_pool_sem_buf);
    if (srv->monitor_pool_sem == NULL) {
        vTaskDelete(srv->tx_task);
        srv->tx_task = NULL;
        vQueueDelete(srv->tx_queue);
        srv->tx_queue = NULL;
        free(srv->conns);
        srv->conns = NULL;
        free(srv->cb_contexts);
        srv->cb_contexts = NULL;
        free(srv->monitor_pool);
        srv->monitor_pool = NULL;
        free(srv->tx_queue_storage);
        srv->tx_queue_storage = NULL;
        return ESP_ERR_NO_MEM;
    }

    monitor_buf_t *monitor_pool = srv->monitor_pool;
    for (int i = 0; i < AX25_AGWPE_SERVER_MONITOR_POOL_SIZE; i++) {
        monitor_pool[i].in_use = false;
    }

    if (register_router_port) {
        /* Register a promiscuous router port so we see all AX.25 traffic
         * for monitoring and incoming connection detection. */
        memset(&srv->router_port, 0, sizeof(srv->router_port));
        srv->router_port.mode      = AX25_PORT_PROMISCUOUS;
        srv->router_port.on_tx_frame  = router_on_frame;
        srv->router_port.user_data = srv;

        esp_err_t err = ax25_router_register_port(&srv->router_port);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Could not register promiscuous router port: %s "
                     "(monitoring may not work)", esp_err_to_name(err));
            /* Non-fatal: server still works for direct ax25_in calls */
        } else {
            srv->router_port_registered = true;
        }
    }

    srv->initialized = true;

    ESP_LOGI(TAG, "AGWPE server initialised (port %d, %d conn slots, %d monitor bufs)",
             srv->port, (int)srv->max_conns,
             AX25_AGWPE_SERVER_MONITOR_POOL_SIZE);
    ESP_LOGI(TAG, "AGWPE memory placement: tx_queue=%s monitor_pool=%s cb_contexts=%s",
             srv->tx_queue_storage_psram ? "PSRAM" : "SRAM",
             srv->monitor_pool_psram ? "PSRAM" : "SRAM",
             srv->cb_contexts_psram ? "PSRAM" : "SRAM");
    return ESP_OK;
}

esp_err_t ax25_agwpe_server_init(const ax25_agwpe_server_config_t *config,
                                  ax25_agwpe_server_t *srv)
{
    return ax25_agwpe_server_init_internal(config, srv, true);
}

esp_err_t ax25_agwpe_server_manager_init(void)
{
    if (s_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_manager, 0, sizeof(s_manager));

    s_manager.mutex = xSemaphoreCreateMutexStatic(&s_manager.mutex_buf);
    if (s_manager.mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_manager.router_port, 0, sizeof(s_manager.router_port));
    s_manager.router_port.mode = AX25_PORT_PROMISCUOUS;
    s_manager.router_port.on_tx_frame = manager_router_on_frame;
    s_manager.router_port.user_data = &s_manager;

    esp_err_t err = ax25_router_register_port(&s_manager.router_port);
    if (err != ESP_OK) {
        vSemaphoreDelete(s_manager.mutex);
        s_manager.mutex = NULL;
        return err;
    }

    s_manager.initialized = true;
    ESP_LOGI(TAG, "AGWPE manager initialized (managed servers cap=%d)", AX25_AGWPE_SERVER_MAX_CLIENTS);
    return ESP_OK;
}

void ax25_agwpe_server_manager_deinit(void)
{
    if (!s_manager.initialized) {
        return;
    }

    ax25_router_remove_port(&s_manager.router_port);

    for (;;) {
        ax25_agwpe_server_t *to_free[AX25_AGWPE_SERVER_MAX_CLIENTS] = {0};
        size_t free_count = 0;
        bool pending_refs = false;

        xSemaphoreTake(s_manager.mutex, portMAX_DELAY);
        for (int i = 0; i < AX25_AGWPE_SERVER_MAX_CLIENTS; i++) {
            if (s_manager.clients[i] == NULL) {
                continue;
            }

            s_manager.client_pending_remove[i] = true;

            if (s_manager.client_refcnt[i] == 0u) {
                to_free[free_count++] = s_manager.clients[i];
                s_manager.clients[i] = NULL;
                s_manager.client_pending_remove[i] = false;
            } else {
                pending_refs = true;
            }
        }
        xSemaphoreGive(s_manager.mutex);

        for (size_t i = 0; i < free_count; i++) {
            ax25_agwpe_server_deinit(to_free[i]);
            free(to_free[i]);
        }

        if (!pending_refs) {
            break;
        }

        taskYIELD();
    }

    if (s_manager.mutex != NULL) {
        vSemaphoreDelete(s_manager.mutex);
    }
    memset(&s_manager, 0, sizeof(s_manager));
    ESP_LOGI(TAG, "AGWPE manager deinitialized");
}

esp_err_t ax25_agwpe_server_add_client(const ax25_agwpe_server_config_t *config,
                                        ax25_agwpe_server_t **out_client)
{
    if (config == NULL || out_client == NULL || config->on_agwpe_frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ax25_agwpe_server_t *client = (ax25_agwpe_server_t *)heap_caps_calloc(
        1,
        sizeof(*client),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (client == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = ax25_agwpe_server_init_internal(config, client, false);
    if (err != ESP_OK) {
        free(client);
        return err;
    }

    xSemaphoreTake(s_manager.mutex, portMAX_DELAY);
    int free_idx = -1;
    for (int i = 0; i < AX25_AGWPE_SERVER_MAX_CLIENTS; i++) {
        if (s_manager.clients[i] == NULL) {
            free_idx = i;
            break;
        }
    }
    if (free_idx >= 0) {
        s_manager.clients[free_idx] = client;
        s_manager.client_refcnt[free_idx] = 0u;
        s_manager.client_pending_remove[free_idx] = false;
    }
    xSemaphoreGive(s_manager.mutex);

    if (free_idx < 0) {
        ax25_agwpe_server_deinit(client);
        free(client);
        return ESP_ERR_NO_MEM;
    }

    *out_client = client;
    return ESP_OK;
}

void ax25_agwpe_server_remove_client(ax25_agwpe_server_t *client)
{
    if (client == NULL || !s_manager.initialized) {
        return;
    }

    ax25_agwpe_server_t *to_free = NULL;
    xSemaphoreTake(s_manager.mutex, portMAX_DELAY);
    for (int i = 0; i < AX25_AGWPE_SERVER_MAX_CLIENTS; i++) {
        if (s_manager.clients[i] == client) {
            s_manager.client_pending_remove[i] = true;
            if (s_manager.client_refcnt[i] == 0u) {
                to_free = s_manager.clients[i];
                s_manager.clients[i] = NULL;
                s_manager.client_pending_remove[i] = false;
            }
            break;
        }
    }
    xSemaphoreGive(s_manager.mutex);

    if (to_free == NULL) {
        return;
    }

    ax25_agwpe_server_deinit(to_free);
    free(to_free);
}

esp_err_t ax25_agwpe_server_client_agwpe_in(ax25_agwpe_server_t *client,
                                             const agwpe_frame_t *frame)
{
    if (!s_manager.initialized || client == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_manager.mutex, portMAX_DELAY);
    bool known = manager_has_client_nolock(client);
    xSemaphoreGive(s_manager.mutex);
    if (!known) {
        return ESP_ERR_INVALID_ARG;
    }

    return ax25_agwpe_server_agwpe_in(client, frame);
}

esp_err_t ax25_agwpe_server_client_ax25_in(ax25_agwpe_server_t *client,
                                            const ax25_frame_t *frame)
{
    if (!s_manager.initialized || client == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_manager.mutex, portMAX_DELAY);
    bool known = manager_has_client_nolock(client);
    xSemaphoreGive(s_manager.mutex);
    if (!known) {
        return ESP_ERR_INVALID_ARG;
    }

    return ax25_agwpe_server_ax25_in(client, frame);
}

struct ax25_router_port_t *ax25_agwpe_server_get_client_router_port(
    ax25_agwpe_server_t *client)
{
    return ax25_agwpe_server_get_router_port(client);
}

void ax25_agwpe_server_deinit(ax25_agwpe_server_t *server)
{
    if (server == NULL || !server->initialized) {
        return;
    }

    server->initialized = false;

    /* Remove router port if this instance currently owns one. */
    if (server->router_port_registered) {
        ax25_router_remove_port(&server->router_port);
        server->router_port_registered = false;
    }

    /* Disconnect all active connections */
    xSemaphoreTake(server->mutex, portMAX_DELAY);
    for (size_t i = 0; i < server->max_conns; i++) {
        if (server->conns[i].conn_router_port_registered) {
            unregister_conn_router_port(&server->conns[i]);
        }
        if (server->conns[i].in_use) {
            ax25_conn_shutdown(&server->conns[i].conn);
        }
        if (server->conns[i].conn.initialized) {
            ax25_conn_deinit(&server->conns[i].conn);
        }
        server->conns[i].in_use = false;
        server->conns[i].conn_needs_cleanup = false;
    }
    xSemaphoreGive(server->mutex);

    /* Stop TX task (static — just delete, no memory to free) */
    if (server->tx_task != NULL) {
        vTaskDelete(server->tx_task);
        server->tx_task = NULL;
    }

    /* Clean up queue (static — vQueueDelete is safe on static queues) */
    if (server->tx_queue != NULL) {
        vQueueDelete(server->tx_queue);
        server->tx_queue = NULL;
    }

    /* Clean up monitor pool semaphore */
    if (server->monitor_pool_sem != NULL) {
        vSemaphoreDelete(server->monitor_pool_sem);
        server->monitor_pool_sem = NULL;
    }

    /* Clean up mutex (static) */
    if (server->mutex != NULL) {
        vSemaphoreDelete(server->mutex);
        server->mutex = NULL;
    }

    if (server->cb_contexts != NULL) {
        free(server->cb_contexts);
        server->cb_contexts = NULL;
    }
    if (server->conns != NULL) {
        free(server->conns);
        server->conns = NULL;
    }
    if (server->monitor_pool != NULL) {
        free(server->monitor_pool);
        server->monitor_pool = NULL;
    }
    if (server->tx_queue_storage != NULL) {
        free(server->tx_queue_storage);
        server->tx_queue_storage = NULL;
    }

    /* No free() — caller owns the storage */
    ESP_LOGI(TAG, "AGWPE server deinitialised");
}

/*******************************************************************************
 * Frame ingress — AGWPE client commands
 ******************************************************************************/

esp_err_t ax25_agwpe_server_agwpe_in(ax25_agwpe_server_t *server,
                                      const agwpe_frame_t *frame)
{
    if (server == NULL || frame == NULL || !server->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "AGWPE in: kind='%c' (%s)", frame->header.data_kind,
             agwpe_kind_to_string(frame->header.data_kind));
#if CONFIG_AX25_AGWPE_LOG_FRAMES
    agwpe_print_frame("RX", frame);
#endif

    switch (frame->header.data_kind) {

        /* ---- Informational requests ---- */
        case 'P':   /* Application Login — silently accept */
            break;

        case 'R':   /* Version request */
            handle_version_req(server, frame);
            break;

        case 'G':   /* Port info request */
            handle_port_info_req(server, frame);
            break;

        case 'g':   /* Port capabilities request */
            handle_port_cap_req(server, frame);
            break;

        case 'H':   /* Heard stations — not implemented */
            ESP_LOGD(TAG, "'H' Heard stations: not implemented");
            break;

        /* ---- Callsign registration ---- */
        case 'X':   /* Register callsign */
            handle_register_call(server, frame);
            break;

        case 'x':   /* Unregister callsign */
            handle_unregister_call(server, frame);
            break;

        /* ---- Mode toggles ---- */
        case 'k':   /* Toggle raw reception */
            handle_enable_raw(server, frame);
            break;

        case 'm':   /* Toggle monitor reception */
            handle_enable_monitor(server, frame);
            break;

        /* ---- Transmit frames ---- */
        case 'K':   /* Send raw AX.25 frame */
            handle_send_raw(server, frame);
            break;

        case 'M':   /* Send unproto (no digis) */
            handle_send_unproto(server, frame);
            break;

        case 'V':   /* Send unproto via digipeaters */
            handle_send_unproto_via(server, frame);
            break;

        /* ---- Connected mode ---- */
        case 'C':   /* Connect */
            handle_connect(server, frame);
            break;

        case 'v':   /* Connect via digipeaters */
            handle_connect_via(server, frame);
            break;

        case 'c':   /* Connect with non-standard PID */
            handle_connect_pid(server, frame);
            break;

        case 'D':   /* Send connected data */
            handle_send_data(server, frame);
            break;

        case 'd':   /* Disconnect */
            handle_disconnect(server, frame);
            break;

        /* ---- Outstanding frame queries ---- */
        case 'y':   /* Outstanding frames on port */
            handle_outstanding_port(server, frame);
            break;

        case 'Y':   /* Outstanding frames on connection */
            handle_outstanding_conn(server, frame);
            break;

        default:
            ESP_LOGW(TAG, "Unhandled AGWPE frame kind '%c' (0x%02X)",
                     frame->header.data_kind, frame->header.data_kind);
            break;
    }

    sweep_released_conn_slots(server);

    return ESP_OK;
}

/*******************************************************************************
 * Frame ingress — AX.25 frames from router / physical layer
 ******************************************************************************/

esp_err_t ax25_agwpe_server_ax25_in(ax25_agwpe_server_t *server,
                                     const ax25_frame_t *frame)
{
    if (server == NULL || frame == NULL || !server->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(server->mutex, portMAX_DELAY);

    /* 1) If raw mode enabled, send 'K' raw frame to client */
    if (server->raw_enabled) {
        send_raw_to_client(server, frame);
    }

    /* 2) If monitor mode enabled, send U/I/S monitor frame to client */
    if (server->monitor_enabled) {
        send_monitor_to_client(server, frame);
    }

    /* 3) Connected-mode frames are routed through per-connection static
     *    router ports. Here we only handle incoming SABM that has no
     *    existing connection slot yet. */
    if (frame->type != AX25_FRAME_UI) {
        char src_call[AGWPE_CALLSIGN_LEN + 1];
        char dst_call[AGWPE_CALLSIGN_LEN + 1];
        ax25_address_to_string(&frame->source, src_call, sizeof(src_call));
        ax25_address_to_string(&frame->destination, dst_call, sizeof(dst_call));

        /* Also handle incoming SABM for a callsign we registered but have
         * no existing connection for — accept incoming connections. */
        if (frame->type == AX25_FRAME_U &&
            (frame->control & ~AX25_CTRL_PF_BIT) == (AX25_CTRL_SABM & ~AX25_CTRL_PF_BIT)) {
            /* For incoming (remote-initiated) connected-mode sessions, only
             * auto-accept once the full digipeater path has been traversed.
             * This avoids creating duplicate pending slots on pre-repeat SABM. */
            for (uint8_t i = 0; i < frame->num_digipeaters; i++) {
                if (!frame->digipeaters[i].has_been_repeated) {
                    xSemaphoreGive(server->mutex);
                    sweep_released_conn_slots(server);
                    return ESP_OK;
                }
            }

            /* Is the destination our registered callsign? */
            if (server->callsign_registered) {
                char reg_call[AGWPE_CALLSIGN_LEN + 1];
                agwpe_get_callsign(server->registered_call, reg_call);
                if (strcasecmp(dst_call, reg_call) == 0) {
                    /* Check if we already have a slot for this pair in any state.
                     * This suppresses duplicate allocations when SABM retransmits
                     * arrive while the first slot is not yet fully connected. */
                    agwpe_conn_slot_t *existing = find_conn_slot_any_state(
                        server, server->registered_call, src_call);
                    if (existing == NULL) {
                        /* Accept incoming connection: allocate a slot */
                        agwpe_conn_slot_t *slot = alloc_conn_slot(server);
                        if (slot != NULL) {
                            agwpe_set_callsign(slot->local_call, reg_call);
                            agwpe_set_callsign(slot->remote_call, src_call);
                            slot->pid  = AX25_PID_NONE;
                            slot->port = server->port;

                            /* Initialise ax25_conn */
                            ax25_address_t local_addr;
                            ax25_address_from_string(reg_call, &local_addr);

                            /* Use static callback context (parallel to conn slot) */
                            int cidx = conn_slot_index(server, slot);
                            if (cidx < 0 || (size_t)cidx >= server->max_conns) {
                                ESP_LOGE(TAG,
                                         "incoming SABM: invalid slot index for slot=%p conns=%p max=%u",
                                         (void *)slot,
                                         (void *)server->conns,
                                         (unsigned)server->max_conns);
                                free_conn_slot(server, slot);
                                xSemaphoreGive(server->mutex);
                                sweep_released_conn_slots(server);
                                return ESP_OK;
                            }
                            conn_cb_ctx_t *ctx = &server->cb_contexts[cidx];
                            ctx->server = server;
                            ctx->slot   = slot;
                            ctx->generation = slot->generation;

                            esp_err_t err = register_conn_router_port(server, slot, reg_call, ctx);
                            if (err != ESP_OK) {
                                ESP_LOGE(TAG, "Failed to register conn router port for incoming SABM: %s",
                                         esp_err_to_name(err));
                                free_conn_slot(server, slot);
                            } else {
                                log_heap_integrity_checkpoint("incoming SABM after router port register");
                                ax25_conn_callbacks_t cb = {
                                    .on_connect    = conn_on_connect,
                                    .on_disconnect = conn_on_disconnect,
                                    .on_data       = conn_on_data,
                                    .on_tx_frame      = conn_on_frame,
                                    .on_error      = conn_on_error,
                                };
                                ax25_conn_config_t cfg = AX25_CONN_CONFIG_DEFAULT();

                                log_heap_integrity_checkpoint("incoming SABM before ax25_conn_init");
                                err = ax25_conn_init(&slot->conn,
                                                     &local_addr, &cb,
                                                     ctx, &cfg);
                                log_heap_integrity_checkpoint("incoming SABM after ax25_conn_init");
                                if (err == ESP_OK) {
                                    /* Feed the SABM into the connection */
                                    ax25_conn_on_frame(&slot->conn, frame);
                                } else {
                                    ESP_LOGE(TAG, "Failed to init conn for incoming SABM");
                                    unregister_conn_router_port(slot);
                                    free_conn_slot(server, slot);
                                }
                            }
                        } else {
                            ESP_LOGW(TAG, "No free conn slots for incoming SABM");
                        }
                    }
                }
            }
        }
    }

    xSemaphoreGive(server->mutex);
    sweep_released_conn_slots(server);
    return ESP_OK;
}

/*******************************************************************************
 * Queries
 ******************************************************************************/

struct ax25_router_port_t *ax25_agwpe_server_get_router_port(
    ax25_agwpe_server_t *server)
{
    if (server == NULL || !server->initialized) {
        return NULL;
    }
    return (struct ax25_router_port_t *)&server->router_port;
}

/*******************************************************************************
 * TX task — drains the outgoing AGWPE queue
 ******************************************************************************/

static void tx_task_func(void *arg)
{
    ax25_agwpe_server_t *srv = (ax25_agwpe_server_t *)arg;
    agwpe_frame_t frame;

    for (;;) {
        if (xQueueReceive(srv->tx_queue, &frame, portMAX_DELAY) == pdTRUE) {
#if CONFIG_AX25_AGWPE_LOG_FRAMES
            agwpe_print_frame("TX", &frame);
#endif
            if (srv->on_agwpe_frame != NULL) {
                srv->on_agwpe_frame(&frame, srv->user_data);
            }
        }
    }
}

static void manager_router_on_frame(const ax25_frame_t *frame, void *user_data)
{
    typedef struct {
        ax25_agwpe_server_t *client;
        int idx;
    } manager_dispatch_target_t;

    (void)user_data;
    if (frame == NULL || !s_manager.initialized) {
        return;
    }

    agwpe_log_router_frame("agwpe_mgr_router_in", frame);

    manager_dispatch_target_t targets[AX25_AGWPE_SERVER_MAX_CLIENTS];
    size_t target_count = 0;

    xSemaphoreTake(s_manager.mutex, portMAX_DELAY);
    for (int i = 0; i < AX25_AGWPE_SERVER_MAX_CLIENTS; i++) {
        ax25_agwpe_server_t *client = s_manager.clients[i];
        if (client != NULL &&
            client->initialized &&
            !s_manager.client_pending_remove[i]) {
            s_manager.client_refcnt[i]++;
            targets[target_count].client = client;
            targets[target_count].idx = i;
            target_count++;
        }
    }
    xSemaphoreGive(s_manager.mutex);

    for (size_t i = 0; i < target_count; i++) {
        ax25_agwpe_server_ax25_in(targets[i].client, frame);

        ax25_agwpe_server_t *to_free = NULL;
        xSemaphoreTake(s_manager.mutex, portMAX_DELAY);
        int idx = targets[i].idx;
        if (idx >= 0 && idx < AX25_AGWPE_SERVER_MAX_CLIENTS) {
            if (s_manager.client_refcnt[idx] > 0u) {
                s_manager.client_refcnt[idx]--;
            }
            if (s_manager.client_refcnt[idx] == 0u &&
                s_manager.client_pending_remove[idx] &&
                s_manager.clients[idx] == targets[i].client) {
                to_free = s_manager.clients[idx];
                s_manager.clients[idx] = NULL;
                s_manager.client_pending_remove[idx] = false;
            }
        }
        xSemaphoreGive(s_manager.mutex);

        if (to_free != NULL) {
            ax25_agwpe_server_deinit(to_free);
            free(to_free);
        }
    }
}

/*******************************************************************************
 * Enqueue helper
 ******************************************************************************/

static esp_err_t enqueue_agwpe_frame(ax25_agwpe_server_t *srv,
                                      const agwpe_frame_t *frame)
{
    if (srv->tx_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Non-blocking enqueue; drop on overflow */
    if (xQueueSend(srv->tx_queue, frame, 0) != pdTRUE) {
        ESP_LOGW(TAG, "TX queue full — dropping AGWPE frame kind='%c'",
                 frame->header.data_kind);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/*******************************************************************************
 * Router callback
 ******************************************************************************/

/**
 * Called by ax25_router for every frame (promiscuous mode).
 * We simply forward into ax25_agwpe_server_ax25_in().
 */
static void router_on_frame(const ax25_frame_t *frame, void *user_data)
{
    ax25_agwpe_server_t *srv = (ax25_agwpe_server_t *)user_data;
    agwpe_log_router_frame("agwpe_router_in", frame);
    ax25_agwpe_server_ax25_in(srv, frame);
}

static void conn_router_on_frame(const ax25_frame_t *frame, void *user_data)
{
    ax25_agwpe_server_t *srv = NULL;
    agwpe_conn_slot_t *slot = NULL;
    conn_cb_ctx_t *ctx = (conn_cb_ctx_t *)user_data;
    if (!resolve_valid_conn_callback_ctx(ctx, &srv, &slot)) {
        return;
    }

    agwpe_log_router_frame("agwpe_conn_router_in", frame);

    ax25_conn_on_frame(&slot->conn, frame);
}

/*******************************************************************************
 * Connection slot management
 ******************************************************************************/

/**
 * @brief Find a connection slot matching local+remote callsigns.
 * Only returns slots with actively connected (not disconnecting) connections.
 * @note Caller must hold srv->mutex.
 */
static agwpe_conn_slot_t *find_conn_slot(ax25_agwpe_server_t *srv,
                                          const char *local_call,
                                          const char *remote_call)
{
    char lc[AGWPE_CALLSIGN_LEN + 1];
    char rc[AGWPE_CALLSIGN_LEN + 1];

    for (size_t i = 0; i < srv->max_conns; i++) {
        agwpe_conn_slot_t *s = &srv->conns[i];
        if (!s->in_use) continue;

        /* Skip slots with connections that are not actively connected */
        if (!ax25_conn_is_connected(&s->conn)) {
            continue;
        }

        agwpe_get_callsign(s->local_call, lc);
        agwpe_get_callsign(s->remote_call, rc);

        if (strcasecmp(lc, local_call) == 0 &&
            strcasecmp(rc, remote_call) == 0) {
            return s;
        }
    }
    return NULL;
}

/**
 * @brief Find a connection slot matching local+remote callsigns in any state.
 * @note Caller must hold srv->mutex.
 */
static agwpe_conn_slot_t *find_conn_slot_any_state(ax25_agwpe_server_t *srv,
                                                   const char *local_call,
                                                   const char *remote_call)
{
    char lc[AGWPE_CALLSIGN_LEN + 1];
    char rc[AGWPE_CALLSIGN_LEN + 1];

    for (size_t i = 0; i < srv->max_conns; i++) {
        agwpe_conn_slot_t *s = &srv->conns[i];
        if (!s->in_use) continue;

        agwpe_get_callsign(s->local_call, lc);
        agwpe_get_callsign(s->remote_call, rc);

        if (strcasecmp(lc, local_call) == 0 &&
            strcasecmp(rc, remote_call) == 0) {
            return s;
        }
    }
    return NULL;
}

/**
 * @brief Allocate a free connection slot.
 * @note Caller must hold srv->mutex.
 */
static agwpe_conn_slot_t *alloc_conn_slot(ax25_agwpe_server_t *srv)
{
    for (size_t i = 0; i < srv->max_conns; i++) {
        if (!srv->conns[i].in_use) {
            uint32_t generation = srv->conns[i].generation;
            if (srv->conns[i].conn.initialized) {
                ax25_conn_deinit(&srv->conns[i].conn);
            }
            memset(&srv->conns[i], 0, sizeof(agwpe_conn_slot_t));
            srv->conns[i].generation = generation;
            if (srv->conns[i].generation == 0u) {
                srv->conns[i].generation = 1u;
            }
            srv->conns[i].in_use = true;
            return &srv->conns[i];
        }
    }
    return NULL;
}

/**
 * @brief Release a connection slot.
 */
static void free_conn_slot(ax25_agwpe_server_t *srv, agwpe_conn_slot_t *slot)
{
    if (slot != NULL) {
        unregister_conn_router_port(slot);

        uint32_t next_generation = slot->generation + 1u;
        if (next_generation == 0u) {
            next_generation = 1u;
        }
        slot->generation = next_generation;
        slot->in_use = false;

        if (srv != NULL && srv->cb_contexts != NULL) {
            int idx = conn_slot_index(srv, slot);
            if (idx >= 0 && (size_t)idx < srv->max_conns) {
                srv->cb_contexts[idx].generation = slot->generation;
            }
        }
    }
}

static void sweep_released_conn_slots(ax25_agwpe_server_t *srv)
{
    if (srv == NULL || srv->conns == NULL) {
        return;
    }

    for (size_t i = 0; i < srv->max_conns; i++) {
        agwpe_conn_slot_t *slot = &srv->conns[i];

        if (slot->in_use || !slot->conn_needs_cleanup) {
            continue;
        }

        if (slot->conn.initialized) {
            ax25_conn_deinit(&slot->conn);
        }
        slot->conn_needs_cleanup = false;
    }
}

static esp_err_t register_conn_router_port(ax25_agwpe_server_t *srv,
                                           agwpe_conn_slot_t *slot,
                                           const char *local_call,
                                           conn_cb_ctx_t *cb_ctx)
{
    if (srv == NULL || slot == NULL || local_call == NULL || cb_ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int slot_idx = conn_slot_index(srv, slot);
    if (slot_idx < 0 || (size_t)slot_idx >= srv->max_conns) {
        ESP_LOGE(TAG,
                 "register_conn_router_port: invalid slot pointer=%p conns=%p max=%u",
                 (void *)slot,
                 (void *)srv->conns,
                 (unsigned)srv->max_conns);
        return ESP_ERR_INVALID_ARG;
    }

    if (&srv->cb_contexts[slot_idx] != cb_ctx) {
        ESP_LOGE(TAG,
                 "register_conn_router_port: callback context mismatch slot_idx=%d expected=%p got=%p",
                 slot_idx,
                 (void *)&srv->cb_contexts[slot_idx],
                 (void *)cb_ctx);
        return ESP_ERR_INVALID_ARG;
    }

    memset(&slot->conn_router_port, 0, sizeof(slot->conn_router_port));
    /* Use a pre-bound dynamic port so multiple active AGWPE connections can
     * share the same local callsign. The router still matches on destination,
     * but dynamic ports are allowed to coexist with duplicate destinations. */
    slot->conn_router_port.mode = AX25_PORT_DYNAMIC;
    slot->conn_router_port.on_tx_frame = conn_router_on_frame;
    slot->conn_router_port.user_data = cb_ctx;

    esp_err_t err = ax25_address_from_string(local_call, &slot->conn_router_port.destination);
    if (err != ESP_OK) {
        return err;
    }

    err = ax25_router_register_port(&slot->conn_router_port);
    if (err == ESP_OK) {
        slot->conn_router_port_registered = true;
    }
    return err;
}

static void unregister_conn_router_port(agwpe_conn_slot_t *slot)
{
    if (slot == NULL || !slot->conn_router_port_registered) {
        return;
    }

    ax25_router_remove_port(&slot->conn_router_port);
    slot->conn_router_port_registered = false;
}

/**
 * @brief Return the index of a connection slot within the server's conns array.
 * @return Slot index (0..MAX_CONNS-1), or -1 if not found.
 */
static int conn_slot_index(ax25_agwpe_server_t *srv, const agwpe_conn_slot_t *slot)
{
    ptrdiff_t idx = slot - srv->conns;
    if (idx >= 0 && (size_t)idx < srv->max_conns) {
        return (int)idx;
    }
    return -1;
}

/*******************************************************************************
 * Monitor buffer pool helpers
 ******************************************************************************/

/**
 * @brief Acquire a monitor output buffer (non-blocking).
 * @return Pointer to monitor_buf_t, or NULL if pool exhausted.
 */
static monitor_buf_t *monitor_buf_alloc(ax25_agwpe_server_t *srv)
{
    monitor_buf_t *monitor_pool = srv->monitor_pool;

    if (xSemaphoreTake(srv->monitor_pool_sem, 0) != pdTRUE) {
        return NULL;  /* pool exhausted */
    }
    for (int i = 0; i < AX25_AGWPE_SERVER_MONITOR_POOL_SIZE; i++) {
        if (!monitor_pool[i].in_use) {
            monitor_pool[i].in_use = true;
            agwpe_frame_init(&monitor_pool[i].frame);
            return &monitor_pool[i];
        }
    }
    /* Shouldn't happen if semaphore is correct */
    xSemaphoreGive(srv->monitor_pool_sem);
    return NULL;
}

/**
 * @brief Release a monitor output buffer back to the pool.
 */
static void monitor_buf_free(ax25_agwpe_server_t *srv, monitor_buf_t *buf)
{
    if (buf != NULL) {
        buf->in_use = false;
        xSemaphoreGive(srv->monitor_pool_sem);
    }
}

/*******************************************************************************
 * ax25_conn callbacks
 ******************************************************************************/




static void conn_on_connect(ax25_address_t remote_addr,
                            bool is_local_initiated, void *user_data)
{
    ax25_agwpe_server_t *srv = NULL;
    agwpe_conn_slot_t *slot = NULL;
    conn_cb_ctx_t *ctx = (conn_cb_ctx_t *)user_data;
    if (!resolve_valid_conn_callback_ctx(ctx, &srv, &slot)) {
        return;
    }

    ESP_LOGI(TAG, "AX.25 connection %s (local_initiated=%d)",
             is_local_initiated ? "established" : "received",
             is_local_initiated);

    /* Send 'C' (Connection Received/Established) to AGWPE client */
    agwpe_frame_t reply;
    agwpe_frame_init(&reply);
    reply.header.port      = slot->port;
    reply.header.data_kind = 'C';

    char remote_str[AGWPE_CALLSIGN_LEN + 1];
    char local_str[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(slot->remote_call, remote_str);
    agwpe_get_callsign(slot->local_call, local_str);

    agwpe_set_callsign(reply.header.call_from, remote_str);
    agwpe_set_callsign(reply.header.call_to, local_str);

    /* Build info string without printf-style formatting to keep stack usage low
     * in router port tasks. */
    char info[100];
    size_t pos = 0;
    const char *prefix = !is_local_initiated
                             ? "*** CONNECTED To Station "
                             : "*** CONNECTED With Station ";
    size_t prefix_len = strlen(prefix);
    if (prefix_len > sizeof(info) - 1) {
        prefix_len = sizeof(info) - 1;
    }
    memcpy(info, prefix, prefix_len);
    pos = prefix_len;

    size_t remote_len = strnlen(remote_str, AGWPE_CALLSIGN_LEN);
    if (remote_len > (sizeof(info) - 1 - pos)) {
        remote_len = sizeof(info) - 1 - pos;
    }
    memcpy(info + pos, remote_str, remote_len);
    pos += remote_len;

    if (pos < sizeof(info) - 1) {
        info[pos++] = '\r';
    }
    info[pos] = '\0';

    size_t info_len = pos + 1;  /* include NUL */
    if (info_len > AGWPE_MAX_DATA_LEN) {
        info_len = AGWPE_MAX_DATA_LEN;
    }
    memcpy(reply.data, info, info_len);
    reply.header.data_len = (uint32_t)info_len;

    enqueue_agwpe_frame(srv, &reply);
}

static void conn_on_disconnect(void *user_data)
{
    ax25_agwpe_server_t *srv = NULL;
    agwpe_conn_slot_t *slot = NULL;
    conn_cb_ctx_t *ctx = (conn_cb_ctx_t *)user_data;
    if (!resolve_valid_conn_callback_ctx(ctx, &srv, &slot)) {
        return;
    }

    ESP_LOGI(TAG, "AX.25 connection disconnected");

    /* Send 'd' (Disconnected) to AGWPE client */
    agwpe_frame_t reply;
    agwpe_frame_init(&reply);
    reply.header.port      = slot->port;
    reply.header.data_kind = 'd';

    char remote_str[AGWPE_CALLSIGN_LEN + 1];
    char local_str[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(slot->remote_call, remote_str);
    agwpe_get_callsign(slot->local_call, local_str);

    agwpe_set_callsign(reply.header.call_from, remote_str);
    agwpe_set_callsign(reply.header.call_to, local_str);

    char info[100];
    size_t pos = 0;
    const char *prefix = "*** DISCONNECTED From Station ";
    size_t prefix_len = strlen(prefix);
    if (prefix_len > sizeof(info) - 1) {
        prefix_len = sizeof(info) - 1;
    }
    memcpy(info, prefix, prefix_len);
    pos = prefix_len;

    size_t remote_len = strnlen(remote_str, AGWPE_CALLSIGN_LEN);
    if (remote_len > (sizeof(info) - 1 - pos)) {
        remote_len = sizeof(info) - 1 - pos;
    }
    memcpy(info + pos, remote_str, remote_len);
    pos += remote_len;

    if (pos < sizeof(info) - 1) {
        info[pos++] = '\r';
    }
    info[pos] = '\0';

    size_t info_len = pos + 1;
    if (info_len > AGWPE_MAX_DATA_LEN) {
        info_len = AGWPE_MAX_DATA_LEN;
    }
    memcpy(reply.data, info, info_len);
    reply.header.data_len = (uint32_t)info_len;

    enqueue_agwpe_frame(srv, &reply);

    slot->conn_needs_cleanup = true;
    free_conn_slot(srv, slot);
}

static void conn_on_data(const uint8_t *data, size_t len, void *user_data)
{
    ax25_agwpe_server_t *srv = NULL;
    agwpe_conn_slot_t *slot = NULL;
    conn_cb_ctx_t *ctx = (conn_cb_ctx_t *)user_data;
    if (!resolve_valid_conn_callback_ctx(ctx, &srv, &slot)) {
        return;
    }

    /* Send 'D' (Connected Data) to AGWPE client */
    deliver_connected_data(srv, slot, data, len);
}

/**
 * @brief Called by ax25_conn when it wants to transmit an AX.25 frame.
 *
 * We route it through ax25_router_send with our router port as the source.
 */
static void conn_on_frame(const ax25_frame_t *frame, void *user_data)
{
    ax25_agwpe_server_t *srv = NULL;
    agwpe_conn_slot_t *slot = NULL;
    conn_cb_ctx_t *ctx = (conn_cb_ctx_t *)user_data;

    if (frame == NULL ||
        !resolve_valid_conn_callback_ctx(ctx, &srv, &slot)) {
        return;
    }

    ESP_LOGD(TAG, "conn_on_frame: routing AX.25 frame via router");
    agwpe_log_router_frame("agwpe_conn_router_out", frame);
    ax25_router_send(frame,
                     (slot != NULL && slot->conn_router_port_registered)
                         ? &slot->conn_router_port
                         : &srv->router_port);
}

static void conn_on_error(const ax25_conn_error_t *error, void *user_data)
{
    ax25_agwpe_server_t *srv = NULL;
    agwpe_conn_slot_t *slot = NULL;
    conn_cb_ctx_t *ctx = (conn_cb_ctx_t *)user_data;
    if (!resolve_valid_conn_callback_ctx(ctx, &srv, &slot)) {
        return;
    }
    (void)srv;
    (void)slot;
    ESP_LOGW(TAG, "AX.25 conn error: %s (code=%d, retries=%d)",
             error->message ? error->message : "unknown",
             error->code, error->retry_count);
}

static bool resolve_valid_conn_callback_ctx(conn_cb_ctx_t *ctx,
                                            ax25_agwpe_server_t **srv_out,
                                            agwpe_conn_slot_t **slot_out)
{
    if (srv_out != NULL) {
        *srv_out = NULL;
    }
    if (slot_out != NULL) {
        *slot_out = NULL;
    }

    if (ctx == NULL || ctx->server == NULL || ctx->slot == NULL) {
        return false;
    }

    ax25_agwpe_server_t *srv = ctx->server;
    agwpe_conn_slot_t *slot = ctx->slot;

    if (!srv->initialized || !slot->in_use) {
        return false;
    }

    if (ctx->generation != slot->generation) {
        return false;
    }

    int idx = conn_slot_index(srv, slot);
    if (idx < 0 || (size_t)idx >= srv->max_conns) {
        return false;
    }

    if (&srv->cb_contexts[idx] != ctx) {
        return false;
    }

    if (srv_out != NULL) {
        *srv_out = srv;
    }
    if (slot_out != NULL) {
        *slot_out = slot;
    }
    return true;
}

/*******************************************************************************
 * AGWPE command handlers (from client)
 ******************************************************************************/

/* ---- 'R' Version request ---- */
static void handle_version_req(ax25_agwpe_server_t *srv,
                                const agwpe_frame_t *frame)
{
    (void)frame;
    agwpe_frame_t reply;
    agwpe_frame_init(&reply);
    reply.header.data_kind = 'R';
    reply.header.data_len  = 8;

    /* Version 2005.127 (matches direwolf for maximum compatibility) */
    uint32_t major = 2005;
    uint32_t minor = 127;
    reply.data[0] = (uint8_t)(major & 0xFF);
    reply.data[1] = (uint8_t)((major >> 8) & 0xFF);
    reply.data[2] = (uint8_t)((major >> 16) & 0xFF);
    reply.data[3] = (uint8_t)((major >> 24) & 0xFF);
    reply.data[4] = (uint8_t)(minor & 0xFF);
    reply.data[5] = (uint8_t)((minor >> 8) & 0xFF);
    reply.data[6] = (uint8_t)((minor >> 16) & 0xFF);
    reply.data[7] = (uint8_t)((minor >> 24) & 0xFF);

    enqueue_agwpe_frame(srv, &reply);
}

/* ---- 'G' Port information ---- */
static void handle_port_info_req(ax25_agwpe_server_t *srv,
                                  const agwpe_frame_t *frame)
{
    (void)frame;
    agwpe_frame_t reply;
    agwpe_frame_init(&reply);
    reply.header.data_kind = 'G';

    /* Format: "<count>;<PortN description>;" */
    char info[200];
    snprintf(info, sizeof(info), "1;%s;", srv->port_desc);

    size_t info_len = strlen(info) + 1;
    if (info_len > AGWPE_MAX_DATA_LEN) {
        info_len = AGWPE_MAX_DATA_LEN;
    }
    memcpy(reply.data, info, info_len);
    reply.header.data_len = (uint32_t)info_len;

    enqueue_agwpe_frame(srv, &reply);
}

/* ---- 'g' Port capabilities ---- */
static void handle_port_cap_req(ax25_agwpe_server_t *srv,
                                 const agwpe_frame_t *frame)
{
    agwpe_frame_t reply;
    agwpe_frame_init(&reply);
    reply.header.port      = frame->header.port;
    reply.header.data_kind = 'g';
    reply.header.data_len  = 12;

    /* Fill with plausible defaults (matches direwolf) */
    reply.data[0] = 0;      /* on_air_baud: 0 = 1200 */
    reply.data[1] = 0xFF;   /* traffic_level: not in autoupdate mode */
    reply.data[2] = 0x19;   /* tx_delay (250 ms) */
    reply.data[3] = 4;      /* tx_tail */
    reply.data[4] = 0xC8;   /* persist (200) */
    reply.data[5] = 4;      /* slot_time */
    reply.data[6] = 7;      /* max_frame */

    /* Count active connections */
    uint8_t active = 0;
    xSemaphoreTake(srv->mutex, portMAX_DELAY);
    for (size_t i = 0; i < srv->max_conns; i++) {
        if (srv->conns[i].in_use) active++;
    }
    xSemaphoreGive(srv->mutex);
    reply.data[7] = active;

    /* how_many_bytes (LE 32-bit at offset 8) */
    reply.data[8]  = 1;
    reply.data[9]  = 0;
    reply.data[10] = 0;
    reply.data[11] = 0;

    enqueue_agwpe_frame(srv, &reply);
}

/* ---- 'X' Register callsign ---- */
static void handle_register_call(ax25_agwpe_server_t *srv,
                                  const agwpe_frame_t *frame)
{
    xSemaphoreTake(srv->mutex, portMAX_DELAY);
    memcpy(srv->registered_call, frame->header.call_from,
           AGWPE_CALLSIGN_LEN);
    srv->callsign_registered = true;
    xSemaphoreGive(srv->mutex);

    char call_str[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(frame->header.call_from, call_str);
    ESP_LOGI(TAG, "Registered callsign: %s", call_str);

    /* Reply: 'X' with 1 byte data = 1 (success) */
    agwpe_frame_t reply;
    agwpe_frame_init(&reply);
    reply.header.data_kind = 'X';
    reply.header.port = frame->header.port;
    memcpy(reply.header.call_from, frame->header.call_from,
           AGWPE_CALLSIGN_LEN);
    reply.header.data_len = 1;
    reply.data[0] = 1;  /* success */

    enqueue_agwpe_frame(srv, &reply);
}

/* ---- 'x' Unregister callsign ---- */
static void handle_unregister_call(ax25_agwpe_server_t *srv,
                                    const agwpe_frame_t *frame)
{
    (void)frame;

    xSemaphoreTake(srv->mutex, portMAX_DELAY);
    srv->callsign_registered = false;
    memset(srv->registered_call, 0, AGWPE_CALLSIGN_LEN);
    xSemaphoreGive(srv->mutex);

    ESP_LOGI(TAG, "Unregistered callsign");
    /* No response expected per protocol */
}

/* ---- 'k' Toggle raw frame reception ---- */
static void handle_enable_raw(ax25_agwpe_server_t *srv,
                               const agwpe_frame_t *frame)
{
    (void)frame;
    xSemaphoreTake(srv->mutex, portMAX_DELAY);
    srv->raw_enabled = !srv->raw_enabled;
    xSemaphoreGive(srv->mutex);

    ESP_LOGI(TAG, "Raw frame reception %s",
             srv->raw_enabled ? "enabled" : "disabled");
}

/* ---- 'm' Toggle monitor reception ---- */
static void handle_enable_monitor(ax25_agwpe_server_t *srv,
                                   const agwpe_frame_t *frame)
{
    (void)frame;
    xSemaphoreTake(srv->mutex, portMAX_DELAY);
    srv->monitor_enabled = !srv->monitor_enabled;
    xSemaphoreGive(srv->mutex);

    ESP_LOGI(TAG, "Monitor reception %s",
             srv->monitor_enabled ? "enabled" : "disabled");
}

/* ---- 'K' Send raw AX.25 frame ---- */
static void handle_send_raw(ax25_agwpe_server_t *srv,
                             const agwpe_frame_t *frame)
{
    if (frame->header.data_len < 2) {
        ESP_LOGW(TAG, "'K' frame too short");
        return;
    }

    /* First byte is "TNC port" indicator (like direwolf), skip it */
    ax25_frame_t ax25;
    esp_err_t err = ax25_frame_parse(frame->data + 1,
                                      frame->header.data_len - 1, &ax25);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to parse raw AX.25 frame from 'K' command");
        return;
    }

    agwpe_log_router_frame("agwpe_router_out_k", &ax25);
    ax25_router_send(&ax25, &srv->router_port);
}

/* ---- 'M' Send unproto (no digipeaters) ---- */
static void handle_send_unproto(ax25_agwpe_server_t *srv,
                                 const agwpe_frame_t *frame)
{
    ax25_frame_t ax25;
    ax25_frame_init(&ax25);

    ax25.type    = AX25_FRAME_UI;
    ax25.control = AX25_CTRL_UI;
    ax25.pid     = frame->header.pid ? frame->header.pid : AX25_PID_NONE;

    /* Source and destination from AGWPE header */
    char from_str[AGWPE_CALLSIGN_LEN + 1];
    char to_str[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(frame->header.call_from, from_str);
    agwpe_get_callsign(frame->header.call_to, to_str);

    ax25_address_from_string(from_str, &ax25.source);
    ax25_address_from_string(to_str, &ax25.destination);

    /* Copy payload */
    size_t copy_len = frame->header.data_len;
    if (copy_len > AX25_MAX_INFO_LEN) {
        copy_len = AX25_MAX_INFO_LEN;
    }
    if (copy_len > 0) {
        memcpy(ax25.payload, frame->data, copy_len);
        ax25.payload_len = copy_len;
    }

    agwpe_log_router_frame("agwpe_router_out_m", &ax25);
    ax25_router_send(&ax25, &srv->router_port);
}

/* ---- 'V' Send unproto via digipeaters ---- */
static void handle_send_unproto_via(ax25_agwpe_server_t *srv,
                                     const agwpe_frame_t *frame)
{
    if (frame->header.data_len < 1) {
        ESP_LOGW(TAG, "'V' frame too short");
        return;
    }

    ax25_frame_t ax25;
    ax25_frame_init(&ax25);

    ax25.type    = AX25_FRAME_UI;
    ax25.control = AX25_CTRL_UI;
    ax25.pid     = frame->header.pid ? frame->header.pid : AX25_PID_NONE;

    /* Source and destination */
    char from_str[AGWPE_CALLSIGN_LEN + 1];
    char to_str[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(frame->header.call_from, from_str);
    agwpe_get_callsign(frame->header.call_to, to_str);

    ax25_address_from_string(from_str, &ax25.source);
    ax25_address_from_string(to_str, &ax25.destination);

    const uint8_t *p = frame->data;
    size_t consumed = 0;
    esp_err_t err = parse_agwpe_digipeater_path(frame->data,
                                                frame->header.data_len,
                                                ax25.digipeaters,
                                                &ax25.num_digipeaters,
                                                &consumed);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Invalid 'V' digipeater path: %s", esp_err_to_name(err));
        return;
    }
    p += consumed;

    /* Remaining bytes are payload */
    size_t copy_len = frame->header.data_len - consumed;
    if (copy_len > AX25_MAX_INFO_LEN) {
        copy_len = AX25_MAX_INFO_LEN;
    }
    if (copy_len > 0) {
        memcpy(ax25.payload, p, copy_len);
        ax25.payload_len = copy_len;
    }

    agwpe_log_router_frame("agwpe_router_out_v", &ax25);
    ax25_router_send(&ax25, &srv->router_port);
}

/* ---- 'C' / 'v' / 'c' Connect ---- */

/**
 * @brief Common connection initiation logic.
 */
static void initiate_connection(ax25_agwpe_server_t *srv,
                                 const agwpe_frame_t *frame,
                                 uint8_t pid,
                                 const ax25_address_t *digipeaters,
                                 uint8_t num_digipeaters)
{
    char from_str[AGWPE_CALLSIGN_LEN + 1];
    char to_str[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(frame->header.call_from, from_str);
    agwpe_get_callsign(frame->header.call_to, to_str);

    xSemaphoreTake(srv->mutex, portMAX_DELAY);

    /* Reject duplicate requests even while the first slot is still pending. */
    agwpe_conn_slot_t *existing = find_conn_slot_any_state(srv, from_str, to_str);
    if (existing != NULL) {
        ESP_LOGW(TAG, "Connection already exists: %s <-> %s", from_str, to_str);
        xSemaphoreGive(srv->mutex);
        return;
    }

    /* Allocate a slot */
    agwpe_conn_slot_t *slot = alloc_conn_slot(srv);
    if (slot == NULL) {
        ESP_LOGE(TAG, "No free connection slots");
        xSemaphoreGive(srv->mutex);

        /* Send 'd' disconnected to indicate failure */
        agwpe_frame_t reply;
        agwpe_frame_init(&reply);
        reply.header.port      = frame->header.port;
        reply.header.data_kind = 'd';
        memcpy(reply.header.call_from, frame->header.call_to,
               AGWPE_CALLSIGN_LEN);
        memcpy(reply.header.call_to, frame->header.call_from,
               AGWPE_CALLSIGN_LEN);

        char info[100];
        snprintf(info, sizeof(info),
                 "*** DISCONNECTED RETRYOUT With %s\r", to_str);
        size_t info_len = strlen(info) + 1;
        memcpy(reply.data, info, info_len);
        reply.header.data_len = (uint32_t)info_len;

        enqueue_agwpe_frame(srv, &reply);
        return;
    }

    agwpe_set_callsign(slot->local_call, from_str);
    agwpe_set_callsign(slot->remote_call, to_str);
    slot->pid  = pid;
    slot->port = frame->header.port;

    /* Use callback context storage parallel to conn slot. */
    int cidx = conn_slot_index(srv, slot);
    if (cidx < 0 || (size_t)cidx >= srv->max_conns) {
        ESP_LOGE(TAG,
                 "initiate_connection: invalid slot index for slot=%p conns=%p max=%u",
                 (void *)slot,
                 (void *)srv->conns,
                 (unsigned)srv->max_conns);
        free_conn_slot(srv, slot);
        xSemaphoreGive(srv->mutex);
        return;
    }
    conn_cb_ctx_t *ctx = &srv->cb_contexts[cidx];
    ctx->server = srv;
    ctx->slot   = slot;
    ctx->generation = slot->generation;

    esp_err_t err = register_conn_router_port(srv, slot, from_str, ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register conn router port: %s", esp_err_to_name(err));
        free_conn_slot(srv, slot);
        xSemaphoreGive(srv->mutex);
        return;
    }

    log_heap_integrity_checkpoint("initiate_connection after router port register");

    /* Initialise ax25_conn */
    ax25_address_t local_addr;
    ax25_address_from_string(from_str, &local_addr);

    ax25_conn_callbacks_t cb = {
        .on_connect    = conn_on_connect,
        .on_disconnect = conn_on_disconnect,
        .on_data       = conn_on_data,
        .on_tx_frame      = conn_on_frame,
        .on_error      = conn_on_error,
    };
    ax25_conn_config_t cfg = AX25_CONN_CONFIG_DEFAULT();

    log_heap_integrity_checkpoint("initiate_connection before ax25_conn_init");
    err = ax25_conn_init(&slot->conn, &local_addr, &cb, ctx, &cfg);
    log_heap_integrity_checkpoint("initiate_connection after ax25_conn_init");
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ax25_conn_init failed: %s", esp_err_to_name(err));
        unregister_conn_router_port(slot);
        free_conn_slot(srv, slot);
        xSemaphoreGive(srv->mutex);
        return;
    }

    /* Initiate connection */
    ax25_address_t remote_addr;
    ax25_address_from_string(to_str, &remote_addr);

    err = ax25_conn_connect(&slot->conn, &remote_addr, digipeaters, num_digipeaters);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ax25_conn_connect failed: %s", esp_err_to_name(err));
        ax25_conn_deinit(&slot->conn);
        unregister_conn_router_port(slot);
        free_conn_slot(srv, slot);
    }

    xSemaphoreGive(srv->mutex);
}

static void handle_connect(ax25_agwpe_server_t *srv,
                            const agwpe_frame_t *frame)
{
    initiate_connection(srv, frame, AX25_PID_NONE, NULL, 0);
}

static void handle_connect_via(ax25_agwpe_server_t *srv,
                                const agwpe_frame_t *frame)
{
    ax25_address_t digipeaters[AX25_MAX_DIGIPEATERS];
    uint8_t num_digipeaters = 0;
    size_t consumed = 0;
    esp_err_t err = parse_agwpe_digipeater_path(frame->data,
                                                frame->header.data_len,
                                                digipeaters,
                                                &num_digipeaters,
                                                &consumed);
    if (err != ESP_OK || consumed != frame->header.data_len) {
        ESP_LOGW(TAG, "Invalid 'v' connect-via path");
        return;
    }

    initiate_connection(srv, frame, AX25_PID_NONE, digipeaters, num_digipeaters);
}

static void handle_connect_pid(ax25_agwpe_server_t *srv,
                                const agwpe_frame_t *frame)
{
    initiate_connection(srv, frame, frame->header.pid, NULL, 0);
}

static esp_err_t parse_agwpe_digipeater_path(const uint8_t *data,
                                             size_t data_len,
                                             ax25_address_t *digipeaters,
                                             uint8_t *num_digipeaters,
                                             size_t *consumed_len)
{
    if (data == NULL || digipeaters == NULL || num_digipeaters == NULL || consumed_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (data_len < 1) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t ndigi = data[0];
    if (ndigi > AX25_MAX_DIGIPEATERS) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t expected_len = 1u + ((size_t)ndigi * AGWPE_CALLSIGN_LEN);
    if (data_len < expected_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(digipeaters, 0, sizeof(ax25_address_t) * AX25_MAX_DIGIPEATERS);
    for (uint8_t i = 0; i < ndigi; i++) {
        char digi_str[AGWPE_CALLSIGN_LEN + 1];
        const uint8_t *field = data + 1u + ((size_t)i * AGWPE_CALLSIGN_LEN);

        memset(digi_str, 0, sizeof(digi_str));
        memcpy(digi_str, field, AGWPE_CALLSIGN_LEN);
        digi_str[AGWPE_CALLSIGN_LEN] = '\0';
        for (int j = AGWPE_CALLSIGN_LEN - 1; j >= 0 && (digi_str[j] == '\0' || digi_str[j] == ' '); j--) {
            digi_str[j] = '\0';
        }

        esp_err_t err = ax25_address_from_string(digi_str, &digipeaters[i]);
        if (err != ESP_OK) {
            return err;
        }
        digipeaters[i].has_been_repeated = false;
        digipeaters[i].is_last = false;
    }

    *num_digipeaters = ndigi;
    *consumed_len = expected_len;
    return ESP_OK;
}

/* ---- 'D' Send connected data ---- */
static void handle_send_data(ax25_agwpe_server_t *srv,
                              const agwpe_frame_t *frame)
{
    char from_str[AGWPE_CALLSIGN_LEN + 1];
    char to_str[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(frame->header.call_from, from_str);
    agwpe_get_callsign(frame->header.call_to, to_str);

    xSemaphoreTake(srv->mutex, portMAX_DELAY);
    agwpe_conn_slot_t *slot = find_conn_slot(srv, from_str, to_str);
    if (slot == NULL) {
        ESP_LOGW(TAG, "No connection found for 'D': %s -> %s", from_str, to_str);
        xSemaphoreGive(srv->mutex);
        return;
    }

    esp_err_t err = ax25_conn_send_data(&slot->conn, frame->data,
                                         frame->header.data_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ax25_conn_send_data failed: %s", esp_err_to_name(err));
    }
    xSemaphoreGive(srv->mutex);
}

/* ---- 'd' Disconnect ---- */
static void handle_disconnect(ax25_agwpe_server_t *srv,
                               const agwpe_frame_t *frame)
{
    char from_str[AGWPE_CALLSIGN_LEN + 1];
    char to_str[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(frame->header.call_from, from_str);
    agwpe_get_callsign(frame->header.call_to, to_str);

    xSemaphoreTake(srv->mutex, portMAX_DELAY);
    agwpe_conn_slot_t *slot = find_conn_slot(srv, from_str, to_str);
    if (slot == NULL) {
        ESP_LOGW(TAG, "No connection found for 'd': %s -> %s", from_str, to_str);
        xSemaphoreGive(srv->mutex);
        return;
    }

    esp_err_t err = ax25_conn_shutdown(&slot->conn);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ax25_conn_shutdown failed: %s", esp_err_to_name(err));
        /* Force-release the slot */
        ax25_conn_deinit(&slot->conn);
        free_conn_slot(srv, slot);
    }
    xSemaphoreGive(srv->mutex);
}

/* ---- 'y' Outstanding frames on port ---- */
static void handle_outstanding_port(ax25_agwpe_server_t *srv,
                                     const agwpe_frame_t *frame)
{
    agwpe_frame_t reply;
    agwpe_frame_init(&reply);
    reply.header.port      = frame->header.port;
    reply.header.data_kind = 'y';
    reply.header.data_len  = 4;

    /* We don't maintain a per-port TX queue count like direwolf's tq_count,
     * so reply with 0. */
    memset(reply.data, 0, 4);

    enqueue_agwpe_frame(srv, &reply);
}

/* ---- 'Y' Outstanding frames on connection ---- */
static void handle_outstanding_conn(ax25_agwpe_server_t *srv,
                                     const agwpe_frame_t *frame)
{
    char from_str[AGWPE_CALLSIGN_LEN + 1];
    char to_str[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(frame->header.call_from, from_str);
    agwpe_get_callsign(frame->header.call_to, to_str);

    uint32_t count = 0;

    xSemaphoreTake(srv->mutex, portMAX_DELAY);
    agwpe_conn_slot_t *slot = find_conn_slot(srv, from_str, to_str);
    if (slot != NULL) {
        /* Count pending unacknowledged I-frames in the connection */
        /* ax25_conn stores tx_queue[8] with in_use flags */
        for (int i = 0; i < 8; i++) {
            if (slot->conn.tx_queue[i].in_use) {
                count++;
            }
        }
    }
    xSemaphoreGive(srv->mutex);

    agwpe_frame_t reply;
    agwpe_frame_init(&reply);
    reply.header.port      = frame->header.port;
    reply.header.data_kind = 'Y';
    memcpy(reply.header.call_from, frame->header.call_from,
           AGWPE_CALLSIGN_LEN);
    memcpy(reply.header.call_to, frame->header.call_to,
           AGWPE_CALLSIGN_LEN);
    reply.header.data_len = 4;

    /* Little-endian count */
    reply.data[0] = (uint8_t)(count & 0xFF);
    reply.data[1] = (uint8_t)((count >> 8) & 0xFF);
    reply.data[2] = (uint8_t)((count >> 16) & 0xFF);
    reply.data[3] = (uint8_t)((count >> 24) & 0xFF);

    enqueue_agwpe_frame(srv, &reply);
}

/*******************************************************************************
 * AX.25 → AGWPE monitoring helpers
 ******************************************************************************/

/**
 * @brief Send a raw 'K' frame to the client.
 *
 * Converts the parsed AX.25 frame back to raw bytes and wraps in a 'K'
 * AGWPE frame.
 */
static void send_raw_to_client(ax25_agwpe_server_t *srv,
                                const ax25_frame_t *frame)
{
    monitor_buf_t *mb = monitor_buf_alloc(srv);
    if (mb == NULL) {
        ESP_LOGW(TAG, "Monitor pool exhausted — dropping raw frame");
        return;
    }

    esp_err_t err = ax25_to_agwpe_raw(frame, srv->port, &mb->frame);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to convert AX.25 to AGWPE raw");
        monitor_buf_free(srv, mb);
        return;
    }

    /* Change data_kind to 'K' for received raw (direwolf uses 'K' for both
     * send and receive raw; the direction is implicit) */
    mb->frame.header.data_kind = 'K';

    /* Prepend the TNC port byte (like direwolf) */
    if (mb->frame.header.data_len + 1 <= AGWPE_MAX_DATA_LEN) {
        memmove(mb->frame.data + 1, mb->frame.data, mb->frame.header.data_len);
        mb->frame.data[0] = srv->port << 4;
        mb->frame.header.data_len += 1;
    }

    enqueue_agwpe_frame(srv, &mb->frame);
    monitor_buf_free(srv, mb);
}

/**
 * @brief Send a monitor frame ('U', 'I', or 'S') to the client.
 *
 * Maps AX.25 frame types to AGWPE monitor kinds:
 *  - UI → 'U'
 *  - I  → 'I'
 *  - S/U (non-UI) → 'S'
 */
static void send_monitor_to_client(ax25_agwpe_server_t *srv,
                                    const ax25_frame_t *frame)
{
    monitor_buf_t *mb = monitor_buf_alloc(srv);
    if (mb == NULL) {
        ESP_LOGW(TAG, "Monitor pool exhausted — dropping monitor frame");
        return;
    }
    agwpe_frame_t *reply = &mb->frame;
    agwpe_frame_init(reply);

    reply->header.port = srv->port;

    /* Set callsigns */
    char src_str[AGWPE_CALLSIGN_LEN + 1];
    char dst_str[AGWPE_CALLSIGN_LEN + 1];
    ax25_address_to_string(&frame->source, src_str, sizeof(src_str));
    ax25_address_to_string(&frame->destination, dst_str, sizeof(dst_str));
    agwpe_set_callsign(reply->header.call_from, src_str);
    agwpe_set_callsign(reply->header.call_to, dst_str);

    /* Determine monitor kind */
    switch (frame->type) {
        case AX25_FRAME_UI:
            reply->header.data_kind = 'U';
            reply->header.pid = frame->pid;
            break;
        case AX25_FRAME_I:
            reply->header.data_kind = 'I';
            reply->header.pid = frame->pid;
            break;
        case AX25_FRAME_S:
        case AX25_FRAME_U:
        default:
            reply->header.data_kind = 'S';
            break;
    }

    /* Build monitor text.  Format similar to direwolf:
     * " <chan>:Fm <src> To <dst> [HH:MM:SS]\r<payload>\r\0"
     */
    char text[128 + AX25_MAX_INFO_LEN];
    size_t pos = 0;

    /* Header text */
    pos += snprintf(text + pos, sizeof(text) - pos,
                    " %d:Fm %s To %s ", srv->port + 1, src_str, dst_str);

    /* Add digipeater path if present */
    if (frame->num_digipeaters > 0) {
        pos += snprintf(text + pos, sizeof(text) - pos, "Via ");
        for (uint8_t i = 0; i < frame->num_digipeaters; i++) {
            char digi_str[AGWPE_CALLSIGN_LEN + 1];
            ax25_address_to_string(&frame->digipeaters[i], digi_str,
                                    sizeof(digi_str));
            if (i > 0) {
                pos += snprintf(text + pos, sizeof(text) - pos, ",");
            }
            pos += snprintf(text + pos, sizeof(text) - pos, "%s", digi_str);
            if (frame->digipeaters[i].has_been_repeated) {
                pos += snprintf(text + pos, sizeof(text) - pos, "*");
            }
        }
        pos += snprintf(text + pos, sizeof(text) - pos, " ");
    }

    /* Frame type description */
    switch (frame->type) {
        case AX25_FRAME_UI:
            pos += snprintf(text + pos, sizeof(text) - pos,
                            "<UI pid=%02X Len=%zu >",
                            frame->pid, frame->payload_len);
            break;
        case AX25_FRAME_I:
            pos += snprintf(text + pos, sizeof(text) - pos,
                            "<I S%d R%d pid=%02X Len=%zu >",
                            ax25_frame_extract_ns(frame->control),
                            ax25_frame_extract_nr(frame->control),
                            frame->pid, frame->payload_len);
            break;
        case AX25_FRAME_S: {
            uint8_t stype = frame->control & 0x0F;
            const char *sname = "S";
            switch (stype) {
                case 0x01: sname = "RR"; break;
                case 0x05: sname = "RNR"; break;
                case 0x09: sname = "REJ"; break;
            }
            pos += snprintf(text + pos, sizeof(text) - pos,
                            "<%s R%d >", sname,
                            ax25_frame_extract_nr(frame->control));
            break;
        }
        case AX25_FRAME_U: {
            uint8_t utype = frame->control & 0xEF;  /* mask out P/F */
            const char *uname = "U";
            if (utype == (AX25_CTRL_SABM & 0xEF)) uname = "SABM";
            else if (utype == (AX25_CTRL_DISC & 0xEF)) uname = "DISC";
            else if (utype == (AX25_CTRL_DM & 0xEF)) uname = "DM";
            else if (utype == (AX25_CTRL_UA & 0xEF)) uname = "UA";
            else if (utype == (AX25_CTRL_FRMR & 0xEF)) uname = "FRMR";
            pos += snprintf(text + pos, sizeof(text) - pos,
                            "<%s >", uname);
            break;
        }
        default:
            pos += snprintf(text + pos, sizeof(text) - pos, "<? >");
            break;
    }

    /* Timestamp placeholder (no RTC on ESP32 typically) */
    pos += snprintf(text + pos, sizeof(text) - pos, "[--:--:--]\r");

    /* Append payload for I and UI frames */
    if ((frame->type == AX25_FRAME_UI || frame->type == AX25_FRAME_I) &&
        frame->payload_len > 0) {
        size_t payload_copy = frame->payload_len;
        if (pos + payload_copy + 2 > sizeof(text)) {
            payload_copy = sizeof(text) - pos - 2;
        }
        memcpy(text + pos, frame->payload, payload_copy);
        pos += payload_copy;
        text[pos++] = '\r';
    }

    text[pos++] = '\0';  /* NUL terminator included in data_len */

    /* Copy to AGWPE frame */
    size_t data_len = pos;
    if (data_len > AGWPE_MAX_DATA_LEN) {
        data_len = AGWPE_MAX_DATA_LEN;
    }
    memcpy(reply->data, text, data_len);
    reply->header.data_len = (uint32_t)data_len;

    enqueue_agwpe_frame(srv, reply);
    monitor_buf_free(srv, mb);
}

/**
 * @brief Send 'D' (connected data) to the AGWPE client.
 */
static void deliver_connected_data(ax25_agwpe_server_t *srv,
                                    agwpe_conn_slot_t *slot,
                                    const uint8_t *data, size_t len)
{
    agwpe_frame_t reply;
    agwpe_frame_init(&reply);

    reply.header.port      = slot->port;
    reply.header.data_kind = 'D';
    reply.header.pid       = slot->pid;

    /* call_from = remote, call_to = local (matches direwolf convention) */
    memcpy(reply.header.call_from, slot->remote_call, AGWPE_CALLSIGN_LEN);
    memcpy(reply.header.call_to, slot->local_call, AGWPE_CALLSIGN_LEN);

    size_t copy_len = len;
    if (copy_len > AGWPE_MAX_DATA_LEN) {
        copy_len = AGWPE_MAX_DATA_LEN;
    }
    if (copy_len > 0) {
        memcpy(reply.data, data, copy_len);
    }
    reply.header.data_len = (uint32_t)copy_len;

    enqueue_agwpe_frame(srv, &reply);
}
