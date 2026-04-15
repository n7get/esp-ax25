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
 * @file ax25_phy_kiss_uart.c
 * @brief UART KISS physical-layer driver
 */

#include <string.h>
#include "esp_log.h"

#include "ax25_buffer.h"
#include "ax25_frame.h"
#include "ax25_print.h"
#include "ax25_phy_kiss_uart.h"
#include "ax25_uart.h"

static const char *TAG = "AX25_PHY_UART";

// TODO: consider making these runtime-configurable via ax25_config
#define AX25_PHY_UART_WRITE_RETRIES    CONFIG_AX25_PHY_UART_WRITE_RETRIES
#define AX25_PHY_UART_WRITE_BACKOFF_MS CONFIG_AX25_PHY_UART_WRITE_BACKOFF_MS
#define AX25_PHY_UART_LOG_FRAMES       CONFIG_AX25_PHY_UART_LOG_FRAMES
#define AX25_PHY_UART_TX_QUEUE_DEPTH   8
#define AX25_PHY_UART_TX_TASK_STACK    3072
#define AX25_PHY_UART_RX_TASK_STACK    6144
#define AX25_PHY_UART_TX_TASK_PRIORITY 5

typedef struct {
    size_t len;
    uint8_t data[AX25_KISS_MAX_ENCODED_SIZE];
} ax25_phy_kiss_uart_tx_item_t;

static esp_err_t uart_write_with_retry(ax25_phy_kiss_uart_t *ctx,
                                       const uint8_t *data,
                                       size_t len);

/* -------------------------------------------------------------------------
 * Private helpers
 * ---------------------------------------------------------------------- */

/**
 * Called by the KISS decoder for each complete KISS frame.
 * Parses the AX.25 content and injects it into the global router.
 */
static void uart_phy_rx_frame_cb(uint8_t port_num, uint8_t command, const uint8_t *data, size_t len, void *user_data)
{
    if (command != 0) {
        return; /* ignore non-data commands */
    }

    ax25_frame_t frame;
    if (ax25_frame_parse(data, len, &frame) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to parse incoming frame (%zu bytes)", len);
        return;
    }
#if AX25_PHY_UART_LOG_FRAMES
    ax25_print_frame("uart_recv", &frame);
#endif
    ax25_phy_kiss_uart_t *phy = (ax25_phy_kiss_uart_t *)user_data;
    if (phy && phy->on_rx_frame) {
        phy->on_rx_frame(&frame, phy->user_data);
    }
}

/** Receive task: reads bytes from UART and feeds them to the KISS decoder or the RX tap. */
static void uart_phy_rx_task(void *arg)
{
    ax25_phy_kiss_uart_t *ctx = (ax25_phy_kiss_uart_t *)arg;
    uint8_t rx_buf[256];

    while (ctx->running) {
        int len = uart_read_bytes(ctx->uart_num, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(100));
        if (len > 0) {
            if (ctx->rx_tap) {
                ctx->rx_tap(rx_buf, (size_t)len, ctx->rx_tap_user_data);
            } else {
                ax25_kiss_decoder_process_bytes(&ctx->kiss_decoder, rx_buf, (size_t)len);
            }
        }
    }

    ctx->uart_rx_task_handle = NULL;
    vTaskDelete(NULL);
}

static void uart_phy_tx_task(void *arg)
{
    ax25_phy_kiss_uart_t *ctx = (ax25_phy_kiss_uart_t *)arg;
    ax25_phy_kiss_uart_tx_item_t item;

    while (ctx->running) {
        if (xQueueReceive(ctx->uart_tx_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        if (!ctx->running) {
            break;
        }

        if (ctx->tx_mutex != NULL) {
            xSemaphoreTake(ctx->tx_mutex, portMAX_DELAY);
        }

        esp_err_t err = uart_write_with_retry(ctx, item.data, item.len);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "queued UART write failed: %s", esp_err_to_name(err));
        }

        if (ctx->tx_mutex != NULL) {
            xSemaphoreGive(ctx->tx_mutex);
        }
    }

    ctx->uart_tx_task_handle = NULL;
    vTaskDelete(NULL);
}

static esp_err_t uart_write_with_retry(ax25_phy_kiss_uart_t *ctx,
                                       const uint8_t *data,
                                       size_t len)
{
    size_t offset = 0;
    int retries = 0;

    while (offset < len) {
        int written = uart_write_bytes(ctx->uart_num,
                                       (const char *)data + offset,
                                       len - offset);

        if (written > 0) {
            offset += (size_t)written;
            retries = 0;
            continue;
        }

        retries++;
        if (retries > AX25_PHY_UART_WRITE_RETRIES) {
            ESP_LOGE(TAG, "UART write failed after %d retries", AX25_PHY_UART_WRITE_RETRIES);
            return ESP_FAIL;
        }

        vTaskDelay(pdMS_TO_TICKS(AX25_PHY_UART_WRITE_BACKOFF_MS));
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t ax25_phy_kiss_uart_init(ax25_on_frame_t on_rx_frame, void *user_data, ax25_phy_kiss_uart_t *ctx)
{
    if (!ctx || !on_rx_frame) {
        return ESP_ERR_INVALID_ARG;
    }

    ctx->running             = false;
    ctx->uart_rx_task_handle = NULL;
    ctx->uart_tx_task_handle = NULL;
    ctx->uart_tx_queue       = NULL;
    ctx->tx_mutex            = NULL;
    ctx->on_rx_frame         = on_rx_frame;
    ctx->user_data           = user_data;

    // TODO: consider making these runtime-configurable via ax25_config?
    uint32_t    rx_stk    = AX25_PHY_UART_RX_TASK_STACK;
    UBaseType_t rx_prio   = 5;

    esp_err_t err = ax25_uart_init(&ctx->uart_num);
    if (err != ESP_OK) {
        return err;
    }

    ctx->tx_mutex = xSemaphoreCreateMutex();
    if (ctx->tx_mutex == NULL) {
        uart_driver_delete(ctx->uart_num);
        return ESP_ERR_NO_MEM;
    }

    ctx->uart_tx_queue = xQueueCreate(AX25_PHY_UART_TX_QUEUE_DEPTH,
                                      sizeof(ax25_phy_kiss_uart_tx_item_t));
    if (ctx->uart_tx_queue == NULL) {
        vSemaphoreDelete(ctx->tx_mutex);
        ctx->tx_mutex = NULL;
        uart_driver_delete(ctx->uart_num);
        return ESP_ERR_NO_MEM;
    }

    ax25_kiss_decoder_init(&ctx->kiss_decoder, uart_phy_rx_frame_cb, ctx);

    ctx->running = true;

    if (xTaskCreate(uart_phy_tx_task,
                    "uart_tx_task",
                    AX25_PHY_UART_TX_TASK_STACK,
                    ctx,
                    AX25_PHY_UART_TX_TASK_PRIORITY,
                    &ctx->uart_tx_task_handle) != pdPASS) {
        ctx->running = false;
        vQueueDelete(ctx->uart_tx_queue);
        ctx->uart_tx_queue = NULL;
        vSemaphoreDelete(ctx->tx_mutex);
        ctx->tx_mutex = NULL;
        uart_driver_delete(ctx->uart_num);
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(uart_phy_rx_task, "uart_rx_task", rx_stk, ctx, rx_prio, &ctx->uart_rx_task_handle) != pdPASS) {
        ctx->running = false;
        for (int i = 0; i < 20 && ctx->uart_tx_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (ctx->uart_tx_task_handle != NULL) {
            vTaskDelete(ctx->uart_tx_task_handle);
            ctx->uart_tx_task_handle = NULL;
        }
        vQueueDelete(ctx->uart_tx_queue);
        ctx->uart_tx_queue = NULL;
        vSemaphoreDelete(ctx->tx_mutex);
        ctx->tx_mutex = NULL;
        uart_driver_delete(ctx->uart_num);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void ax25_phy_kiss_uart_deinit(ax25_phy_kiss_uart_t *ctx)
{
    if (!ctx || !ctx->running) {
        return;
    }

    ctx->running = false;

    for (int i = 0; i < 20 && ctx->uart_rx_task_handle != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (ctx->uart_rx_task_handle != NULL) {
        vTaskDelete(ctx->uart_rx_task_handle);
        ctx->uart_rx_task_handle = NULL;
    }

    for (int i = 0; i < 20 && ctx->uart_tx_task_handle != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (ctx->uart_tx_task_handle != NULL) {
        vTaskDelete(ctx->uart_tx_task_handle);
        ctx->uart_tx_task_handle = NULL;
    }

    if (ctx->uart_tx_queue != NULL) {
        vQueueDelete(ctx->uart_tx_queue);
        ctx->uart_tx_queue = NULL;
    }

    if (ctx->tx_mutex != NULL) {
        vSemaphoreDelete(ctx->tx_mutex);
        ctx->tx_mutex = NULL;
    }

    ax25_uart_deinit(ctx->uart_num);
}

esp_err_t ax25_phy_kiss_uart_send(const ax25_frame_t *frame, ax25_phy_kiss_uart_t *ctx)
{
    if (!ctx || !frame) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!ctx->running || ctx->uart_tx_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

#if AX25_PHY_UART_LOG_FRAMES
    ax25_print_frame("uart_send", frame);
#endif

    ax25_buffer_t raw = {0};
    if (ax25_frame_build(frame, &raw) != ESP_OK) {
        ESP_LOGE(TAG, "ax25_frame_build failed");
        return ESP_FAIL;
    }

    uint8_t encoded[AX25_KISS_MAX_ENCODED_SIZE];
    size_t encoded_len = ax25_kiss_encode(0, 0, raw.data, raw.len, encoded, sizeof(encoded));
    if (encoded_len == 0) {
        ESP_LOGE(TAG, "ax25_kiss_encode failed");
        return ESP_FAIL;
    }

    ax25_phy_kiss_uart_tx_item_t item = {
        .len = encoded_len,
    };
    memcpy(item.data, encoded, encoded_len);

    if (xQueueSend(ctx->uart_tx_queue, &item, 0) != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t ax25_phy_kiss_uart_write_raw(const uint8_t *data, size_t len, ax25_phy_kiss_uart_t *ctx)
{
    if (!ctx || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!ctx->running || ctx->tx_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(ctx->tx_mutex, portMAX_DELAY);
    esp_err_t err = uart_write_with_retry(ctx, data, len);
    xSemaphoreGive(ctx->tx_mutex);

    return err;
}

void ax25_phy_kiss_uart_set_rx_tap(ax25_phy_uart_rx_tap_t tap, void *user_data, ax25_phy_kiss_uart_t *ctx)
{
    if (!ctx || !tap) {
        return;
    }

    ctx->rx_tap_user_data = user_data;
    ctx->rx_tap = tap;
}

void ax25_phy_kiss_uart_clear_rx_tap(ax25_phy_kiss_uart_t *ctx)
{
    if (!ctx) {
        return;
    }

    ctx->rx_tap = NULL;
    ctx->rx_tap_user_data = NULL;
}
