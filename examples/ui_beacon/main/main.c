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
 * @brief UI Beacon Example - Periodic AX.25 beacon transmitter with frame routing
 *
 * Demonstrates how to use ax25_router and ax25_phy_kiss_uart to decouple
 * frame reception from processing.  Received frames are forwarded through the
 * router to an RX queue that a dedicated task drains and logs.  Outgoing
 * beacon frames are routed through the default router port directly to the
 * UART KISS driver send API.
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "ax25_frame.h"
#include "ax25_address.h"
#include "ax25_config.h"
#include "ax25_print.h"
#include "ax25_router.h"
#include "ax25_phy_kiss_uart.h"

static const char *TAG = "UI_BEACON";

static esp_err_t configure_uart_keys_from_kconfig(void)
{
    char value[16];
    char err_msg[96] = {0};

    snprintf(value, sizeof(value), "%d", CONFIG_UI_BEACON_UART_NUM);
    if (ax25_cfg_set("uart.no", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.no: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_UI_BEACON_TNC_BAUD);
    if (ax25_cfg_set("uart.baud", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.baud: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_UI_BEACON_TNC_TX_PIN);
    if (ax25_cfg_set("uart.tx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.tx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_UI_BEACON_TNC_RX_PIN);
    if (ax25_cfg_set("uart.rx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.rx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/*******************************************************************************
 * Router integration
 ******************************************************************************/

/** Enqueue a frame onto the FreeRTOS queue passed as @p user_data. */
static void app_port_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_print_frame(AX25_PRINT_LABEL_RX, frame);
}

static void phy_port_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    if (ax25_phy_kiss_uart_send(frame, (ax25_phy_kiss_uart_t *)user_data) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send frame via UART KISS PHY");
    }
}

static void phy_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_router_send(frame, (ax25_router_port_t *)user_data);
}

/*******************************************************************************
 * Beacon task
 ******************************************************************************/

static void beacon_task(void *pvParameters)
{
    ax25_address_t local_addr;
    esp_err_t err = ax25_address_from_string(CONFIG_UI_BEACON_LOCAL_CALLSIGN, &local_addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse local callsign");
        vTaskDelete(NULL);
        return;
    }

    ax25_address_t dest;
    err = ax25_address_from_string(CONFIG_UI_BEACON_DEST, &dest);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse destination address");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Beacon task started, interval: %d ms",
             CONFIG_UI_BEACON_INTERVAL_MS);

    uint32_t beacon_count = 0;
    while (1) {
        beacon_count++;

        char message[256];
        snprintf(message, sizeof(message), "%s [Beacon #%lu]",
                 CONFIG_UI_BEACON_MESSAGE, (unsigned long)beacon_count);

        ESP_LOGI(TAG, "Sending beacon #%lu: %s",
                 (unsigned long)beacon_count, message);

        ax25_frame_t f = {0};
        f.type            = AX25_FRAME_UI;
        f.control         = AX25_CTRL_UI;
        f.source          = local_addr;
        f.destination     = dest;
        f.pid             = AX25_PID_TEXT;
        ax25_address_from_string("WIDE1-1", &f.digipeaters[0]);
        ax25_address_from_string("WIDE2-1", &f.digipeaters[1]);
        f.num_digipeaters = 2;
        f.payload_len     = strlen(message);
        memcpy(f.payload, message, f.payload_len);

        err = ax25_router_send(&f, (ax25_router_port_t *)pvParameters);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to route beacon: %d", err);
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_UI_BEACON_INTERVAL_MS));
    }
}

/*******************************************************************************
 * Application entry point
 ******************************************************************************/

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-AX25 UI Beacon Example");
    ESP_LOGI(TAG, "=============================");
    ESP_LOGI(TAG, "Local callsign: %s", CONFIG_UI_BEACON_LOCAL_CALLSIGN);
    ESP_LOGI(TAG, "Beacon destination: %s", CONFIG_UI_BEACON_DEST);
    ESP_LOGI(TAG, "Beacon interval: %d seconds", CONFIG_UI_BEACON_INTERVAL_MS / 1000);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(ax25_cfg_init(NULL, 0));
    ESP_ERROR_CHECK(configure_uart_keys_from_kconfig());

    /* Static storage ensures these contexts outlive app_main. */
    static ax25_phy_kiss_uart_t phy_ctx;
    static ax25_router_port_t   phy_port;
    static ax25_router_port_t   app_port;

    /* Initialise the router. */
    err = ax25_router_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize router: %d", err);
        return;
    }

    app_port.mode = AX25_PORT_PROMISCUOUS;
    app_port.on_tx_frame = app_port_frame_cb;
    err = ax25_router_register_port(&app_port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register App port: %d", err);
        ax25_router_deinit();
        return;
    }

    /* Initialise the UART KISS physical layer. We wire the default router
     * port to the PHY send function so beacon frames flow:
     *   router → phy_port callback → ax25_phy_kiss_uart_send() → UART. */
    err = ax25_phy_kiss_uart_init(phy_frame_cb, &phy_port, &phy_ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UART KISS driver: %d", err);
        ax25_router_remove_port(&app_port);
        ax25_router_deinit();
        return;
    }

    /* Register a default port wired to the PHY send callback. Outgoing beacon
     * frames fall through to this port because no named port matches their
     * destination. */
    phy_port.mode       = AX25_PORT_DEFAULT;
    phy_port.on_tx_frame   = phy_port_frame_cb;
    phy_port.user_data  = &phy_ctx;
    err = ax25_router_register_port(&phy_port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register TX port: %d", err);
        ax25_phy_kiss_uart_deinit(&phy_ctx);
        ax25_router_remove_port(&app_port);
        ax25_router_deinit();
        return;
    }

    ESP_LOGI(TAG, "Router and UART transport initialised");

    static StaticTask_t beacon_tcb;
    static StackType_t  beacon_stack[4096 / sizeof(StackType_t)];
    xTaskCreateStatic(beacon_task, "beacon_task",
                      4096 / sizeof(StackType_t),
                      &app_port, 5,
                      beacon_stack, &beacon_tcb);

    ESP_LOGI(TAG, "Tasks created, running...");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        ESP_LOGI(TAG, "App frames sent: %lu  dropped: %lu",
                 (unsigned long)app_port.frames_sent,
                 (unsigned long)app_port.frames_dropped);
        
        ESP_LOGI(TAG, "PHY frames sent: %lu  dropped: %lu",
                 (unsigned long)phy_port.frames_sent,
                 (unsigned long)phy_port.frames_dropped);

        uint32_t total    = heap_caps_get_total_size(MALLOC_CAP_8BIT);
        uint32_t free_now = esp_get_free_heap_size();
        uint32_t free_min = esp_get_minimum_free_heap_size();
        ESP_LOGI(TAG, "RAM: %lu used now, %lu peak used, %lu free of %lu total",
                 (unsigned long)(total - free_now),
                 (unsigned long)(total - free_min),
                 (unsigned long)free_now,
                 (unsigned long)total);
    }
}
