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
 * @file main.c
 * @brief AX.25 scripted client example over UART KISS
 *
 * Flow:
 *  1) Connect to a remote AX.25 host via a KISS TNC on UART
 *  2) Log incoming text until a line starts with "ENTER COMMAND:"
 *  3) Send "j\r"
 *  4) Log incoming text until a line starts with "ENTER COMMAND:"
 *  5) Send "b\r"
 *  6) Disconnect cleanly
 *
 * Architecture
 * ============
 * app_main sets up the router, PHY, and ax25_conn, then initiates the
 * connection and returns.  When the connection is established the
 * on_connect callback creates a receive queue and spawns app_task.
 * app_task reads queued bytes line-by-line and runs the BBS interaction
 * script. Received data arrives via on_data which enqueues chunks;
 * on_disconnect tears down the queue and task.
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "ax25_conn.h"
#include "ax25_address.h"
#include "ax25_config.h"
#include "ax25_frame.h"
#include "ax25_router.h"
#include "ax25_phy_kiss_uart.h"
#include "ax25_log.h"

static const char *TAG    = "CLIENT";
static const char *PROMPT = "ENTER COMMAND:";

#define RX_CHUNK_MAX  AX25_MAX_INFO_LEN
#define RX_QUEUE_LEN  32

#define HEARD_CMD "j\r"
#define BYE_CMD   "b\r"

/* One received chunk carried through the queue. */
typedef struct {
    size_t  len;
    uint8_t data[RX_CHUNK_MAX];
} rx_chunk_t;

/**
 * Application context — shared between ax25_conn callbacks and app_task.
 *
 * Lifetime:
 *   conn        set before ax25_conn_init, stable for the process lifetime.
 *   app_port    registered before ax25_conn_init, stable for the process lifetime.
 *   queue        allocated in on_connect, freed in on_disconnect.
 *   task         spawned  in on_connect, deleted in on_disconnect.
 *   rx_current   filled from the queue on demand by read_byte.
 *   deadline     updated by wait_for_prompt before each blocking read.
 */
typedef struct {
    ax25_conn_t       *conn;
    ax25_router_port_t app_port;
    QueueHandle_t      queue;
    TaskHandle_t       task;
    TickType_t         deadline;
     rx_chunk_t         rx_current;
     size_t             rx_offset;
     bool               rx_chunk_valid;

    /* Static FreeRTOS storage for queue and task */
    StaticQueue_t      queue_buf;
    uint8_t            queue_storage[RX_QUEUE_LEN * sizeof(rx_chunk_t)];
    StaticTask_t       task_tcb;
    StackType_t        task_stack[4096 / sizeof(StackType_t)];
} app_ctx_t;

/* Forward declaration — called from on_connect. */
static void app_task(void *param);

/* ── PHY callbacks ───────────────────────────────────────────────────── */

/* Frame received from the TNC: inject into the router from phy_port. */
static void phy_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_router_send(frame, (ax25_router_port_t *)user_data);
}

/* Router delivers a frame to the PHY port: write it to the TNC. */
static void phy_port_output_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_phy_kiss_uart_send(frame, (ax25_phy_kiss_uart_t *)user_data);
}

/* ── App router port callback (router → conn) ────────────────────────── */

/**
 * The router delivers an ax25_frame_t to our app port.  Pass it directly
 * to ax25_conn for protocol processing.
 *
 * user_data is a pointer to the ax25_conn_t (set in app_main).
 */
static void app_port_on_frame(const ax25_frame_t *frame, void *user_data)
{
    ax25_conn_t *conn = (ax25_conn_t *)user_data;
    ax25_conn_on_frame(conn, frame);
}

/* ── ax25_conn callbacks ─────────────────────────────────────────────── */

/**
 * Connection established: create the receive queue and spawn app_task.
 */
static void on_connect(ax25_address_t remote_addr, bool is_local_initiated, void *user_data)
{
    app_ctx_t *ctx = (app_ctx_t *)user_data;
    (void)remote_addr;
    (void)is_local_initiated;

    ESP_LOGI(TAG, "AX.25 connected");

    ctx->queue = xQueueCreateStatic(RX_QUEUE_LEN, sizeof(rx_chunk_t),
                                      ctx->queue_storage, &ctx->queue_buf);
    if (!ctx->queue) {
        ESP_LOGE(TAG, "on_connect: queue create failed");
        return;
    }

    ctx->task = xTaskCreateStatic(app_task, "app_task",
                                   4096 / sizeof(StackType_t),
                                   ctx, 5,
                                   ctx->task_stack, &ctx->task_tcb);
    if (ctx->task == NULL) {
        ESP_LOGE(TAG, "on_connect: task create failed");
        vQueueDelete(ctx->queue);
        ctx->queue = NULL;
    }
}

/**
 * Connection torn down: delete the task, then delete the queue.
 *
 * Deletion order matters: the task must be removed before the queue so that
 * it can no longer block on xQueueReceive after the queue is freed.
 */
static void on_disconnect(void *user_data)
{
    app_ctx_t *ctx = (app_ctx_t *)user_data;
    ESP_LOGW(TAG, "AX.25 disconnected");

    if (ctx->task) {
        vTaskDelete(ctx->task);
        ctx->task = NULL;
    }
    ctx->rx_chunk_valid = false;
    ctx->rx_offset = 0;

    if (ctx->queue) {
        vQueueDelete(ctx->queue);
        ctx->queue = NULL;
    }
}

static void on_error(const ax25_conn_error_t *error, void *user_data)
{
    (void)user_data;
    ESP_LOGE(TAG, "AX.25 error %d: %s", error->code,
             error->message ? error->message : "");
}

/**
 * Data received from the remote station: enqueue a chunk for app_task.
 */
static void on_data(const uint8_t *data, size_t len, void *user_data)
{
    app_ctx_t *ctx = (app_ctx_t *)user_data;
    if (!data || len == 0 || !ctx->queue) {
        return;
    }

    rx_chunk_t chunk;
    chunk.len = (len > RX_CHUNK_MAX) ? RX_CHUNK_MAX : len;
    memcpy(chunk.data, data, chunk.len);

    if (xQueueSend(ctx->queue, &chunk, 0) != pdTRUE) {
        ESP_LOGW(TAG, "RX queue full, dropping %u bytes", (unsigned)chunk.len);
    }
}

/**
 * ax25_conn wants to transmit a frame: route it through the router,
 * using app_port as the source so the frame is not looped back to us.
 */
static void on_tx_frame(const ax25_frame_t *frame, void *user_data)
{
    app_ctx_t *ctx = (app_ctx_t *)user_data;
    ax25_router_send(frame, &ctx->app_port);
}

/* ── Queue-backed line reading ───────────────────────────────────────── */

/**
 * Read one byte from the queued receive stream.
 *
 * Returns 1 when a byte was read, 0 on timeout, or -1 on queue failure.
 */
static int read_byte(app_ctx_t *ctx, uint8_t *out)
{
    TickType_t  now       = xTaskGetTickCount();
    TickType_t  remaining = (ctx->deadline > now) ? (ctx->deadline - now) : 0;

    if (!ctx->rx_chunk_valid || ctx->rx_offset >= ctx->rx_current.len) {
        if (ctx->queue == NULL) {
            return -1;
        }
        if (xQueueReceive(ctx->queue, &ctx->rx_current, remaining) != pdTRUE) {
            return 0;
        }
        ctx->rx_offset = 0;
        ctx->rx_chunk_valid = true;
    }

    if (ctx->rx_offset >= ctx->rx_current.len) {
        return 0; /* timeout */
    }

    *out = ctx->rx_current.data[ctx->rx_offset++];
    if (ctx->rx_offset >= ctx->rx_current.len) {
        ctx->rx_chunk_valid = false;
    }

    return 1;
}

/**
 * Read bytes from the queued receive stream until a newline is found.
 * '\r' bytes are silently skipped.  The result is NUL-terminated in @p out.
 *
 * @return ESP_OK on newline, ESP_ERR_TIMEOUT on deadline, ESP_FAIL on error.
 */
static esp_err_t readline(app_ctx_t *ctx, char *out, size_t out_maxlen)
{
    size_t pos = 0;
    while (1) {
        uint8_t b;
        int r = read_byte(ctx, &b);
        if (r == 0) return ESP_ERR_TIMEOUT;
        if (r < 0)  return ESP_FAIL;
        if (b == '\n' || b == '\r') {
            out[pos] = '\0';
            return ESP_OK;
        }
        if (pos < out_maxlen - 1) {
            out[pos++] = (char)b;
        }
    }
}

/* ── Prompt detection ────────────────────────────────────────────────── */

/**
 * Read lines until one begins with @p prompt or the deadline expires.
 * Sets ctx->deadline from @p timeout_ms before blocking.
 */
static esp_err_t wait_for_prompt(app_ctx_t *ctx, const char *prompt,
                                 uint32_t timeout_ms)
{
    const size_t prompt_len = strlen(prompt);

    ctx->deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    char line[128];
    while (1) {
        esp_err_t err = readline(ctx, line, sizeof(line));
        if (err != ESP_OK) {
            return err;
        }
        ESP_LOGI(TAG, "BBS: %s", line);
        if (strncmp(line, prompt, prompt_len) == 0) {
            return ESP_OK;
        }
    }
}

/* ── Application task ────────────────────────────────────────────────── */

/**
 * Runs the BBS interaction script after the connection is established.
 * Created by on_connect; deleted by on_disconnect.
 */
static void app_task(void *param)
{
    app_ctx_t *ctx = (app_ctx_t *)param;
    ctx->rx_chunk_valid = false;
    ctx->rx_offset = 0;

    /* ── First prompt ── */
    ESP_LOGI(TAG, "Waiting for \"%s\" ...", PROMPT);
    if (wait_for_prompt(ctx, PROMPT, CONFIG_CLIENT_READ_TIMEOUT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Timed out waiting for first prompt");
        goto shutdown;
    }

    ESP_LOGI(TAG, "Sending: j");
    {
        esp_err_t err = ax25_conn_send_data(ctx->conn,
                                            (const uint8_t *)HEARD_CMD,
                                            sizeof(HEARD_CMD) - 1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "send_data(HEARD_CMD) failed: %d", err);
        }
    }

    /* ── Second prompt ── */
    ESP_LOGI(TAG, "Waiting for \"%s\" ...", PROMPT);
    if (wait_for_prompt(ctx, PROMPT, CONFIG_CLIENT_READ_TIMEOUT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Timed out waiting for second prompt");
        goto shutdown;
    }

    ESP_LOGI(TAG, "Sending: b");
    {
        esp_err_t err = ax25_conn_send_data(ctx->conn,
                                            (const uint8_t *)BYE_CMD,
                                            sizeof(BYE_CMD) - 1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "send_data(BYE_CMD) failed: %d", err);
        }
    }

shutdown:
    /* Initiate graceful disconnection; on_disconnect will clean us up. */
    ax25_conn_shutdown(ctx->conn);

    /* Suspend until on_disconnect deletes this task. */
    vTaskSuspend(NULL);
    vTaskDelete(NULL); /* not reached */
}

static esp_err_t configure_uart_keys_from_kconfig(void)
{
    char value[16];
    char err_msg[96] = {0};

    snprintf(value, sizeof(value), "%d", CONFIG_CLIENT_UART_NUM);
    if (ax25_cfg_set("uart.no", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.no: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_CLIENT_TNC_BAUD);
    if (ax25_cfg_set("uart.baud", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.baud: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_CLIENT_TNC_TX_PIN);
    if (ax25_cfg_set("uart.tx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.tx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_CLIENT_TNC_RX_PIN);
    if (ax25_cfg_set("uart.rx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.rx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ── app_main ────────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "AX.25 Client");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(ax25_cfg_init(NULL, 0));
    ESP_ERROR_CHECK(configure_uart_keys_from_kconfig());

    static ax25_conn_t conn;
    static app_ctx_t   app_ctx = { .conn = &conn };

    ax25_address_t local, remote;

    if (ax25_address_from_string(CONFIG_CLIENT_LOCAL_CALLSIGN, &local) != ESP_OK) {
        ESP_LOGE(TAG, "Invalid local callsign: %s", CONFIG_CLIENT_LOCAL_CALLSIGN);
        return;
    }

    if (ax25_address_from_string(CONFIG_CLIENT_REMOTE_CALLSIGN, &remote) != ESP_OK) {
        ESP_LOGE(TAG, "Invalid remote callsign: %s", CONFIG_CLIENT_REMOTE_CALLSIGN);
        return;
    }

    if (ax25_router_init() != ESP_OK) {
        ESP_LOGE(TAG, "Router init failed");
        return;
    }

    /*
     * App port: static port matching our local address.  The router delivers
     * incoming frames here; app_port_on_frame serialises them and passes them
     * to ax25_conn.  user_data is &conn so frame delivery does not require
     * going through app_ctx.
     */
    app_ctx.app_port.destination = local;
    app_ctx.app_port.mode        = AX25_PORT_STATIC;
    app_ctx.app_port.on_tx_frame    = app_port_on_frame;
    app_ctx.app_port.user_data   = &conn;
    if (ax25_router_register_port(&app_ctx.app_port) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register app port");
        ax25_router_deinit();
        return;
    }

    /* Initialize the UART KISS PHY. */
    static ax25_phy_kiss_uart_t phy;
    static ax25_router_port_t   phy_port;
    if (ax25_phy_kiss_uart_init(phy_frame_cb, &phy_port, &phy) != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed");
        ax25_router_remove_port(&app_ctx.app_port);
        ax25_router_deinit();
        return;
    }

    /* PHY port: default port that forwards frames to the TNC. */
    phy_port.mode      = AX25_PORT_DEFAULT;
    phy_port.on_tx_frame  = phy_port_output_cb;
    phy_port.user_data = &phy;
    if (ax25_router_register_port(&phy_port) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register PHY port");
        ax25_phy_kiss_uart_deinit(&phy);
        ax25_router_remove_port(&app_ctx.app_port);
        ax25_router_deinit();
        return;
    }

    /* Initialize ax25_conn.  app_ctx is the user_data for all callbacks. */
    const ax25_conn_callbacks_t cbs = {
        .on_connect    = on_connect,
        .on_disconnect = on_disconnect,
        .on_error      = on_error,
        .on_data       = on_data,
        .on_tx_frame      = on_tx_frame,
    };
    if (ax25_conn_init(&conn, &local, &cbs, &app_ctx, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "ax25_conn_init failed");
        ax25_phy_kiss_uart_deinit(&phy);
        ax25_router_remove_port(&phy_port);
        ax25_router_remove_port(&app_ctx.app_port);
        ax25_router_deinit();
        return;
    }

    /* Initiate the connection.  The rest of the script runs in app_task. */
    ESP_LOGI(TAG, "Connecting to %s ...", CONFIG_CLIENT_REMOTE_CALLSIGN);
    if (ax25_conn_connect(&conn, &remote) != ESP_OK) {
        ESP_LOGE(TAG, "ax25_conn_connect failed");
        ax25_conn_deinit(&conn);
        ax25_phy_kiss_uart_deinit(&phy);
        ax25_router_remove_port(&phy_port);
        ax25_router_remove_port(&app_ctx.app_port);
        ax25_router_deinit();
        return;
    }

    /* app_main's work is done; the connection and task live on independently. */
    vTaskDelay(portMAX_DELAY);
}
