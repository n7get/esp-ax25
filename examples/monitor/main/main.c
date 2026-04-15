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
 * @brief AX.25 Monitor - Decode and log all frames from a KISS soundmodem
 *
 * Connects to a KISS-over-TCP soundmodem (e.g. Direwolf) on the local
 * network via WiFi, decodes every received AX.25 frame, and prints it
 * to the console with ax25_print_frame().
 *
 * Configuration (idf.py menuconfig -> "AX.25 Monitor Configuration"):
 *   - MONITOR_WIFI_SSID / MONITOR_WIFI_PASSWORD
 *   - MONITOR_WIFI_AP_SSID / MONITOR_WIFI_AP_PASSWORD
 *   - MONITOR_SOUNDMODEM_HOST / MONITOR_SOUNDMODEM_PORT
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"

#include "ax25_frame.h"
#include "ax25_config.h"
#include "ax25_print.h"
#include "ax25_router.h"
#include "ax25_phy_kiss_tcp_client.h"
#include "ax25_wifi.h"

static const char* TAG = "MONITOR";

// ---------------------------------------------------------------------------
// AX.25 receive
// ---------------------------------------------------------------------------

static void app_port_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_print_frame("App recv", frame);
}

static void phy_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_router_send(frame, (ax25_router_port_t *)user_data);
}

static void phy_port_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    ESP_LOGE(TAG, "Unexpected frame received on PHY port");
    ax25_print_frame("Phy send", frame);
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

void app_main(void) {
    ESP_LOGI(TAG, "ESP-AX25 Monitor");
    ESP_LOGI(TAG, "=================");
    ESP_LOGI(TAG, "Soundmodem: %s:%d",
             CONFIG_MONITOR_SOUNDMODEM_HOST, CONFIG_MONITOR_SOUNDMODEM_PORT);

    // NVS is required by the WiFi driver
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(ax25_cfg_init(NULL, 0));

    static ax25_wifi_t wifi_ctx;

    err = ax25_wifi_start(&wifi_ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
        return;
    }

    /* Static storage ensures these contexts outlive app_main. */
    static ax25_phy_kiss_tcp_client_t phy_ctx;
    static ax25_router_port_t         phy_port;
    static ax25_router_port_t         app_port;

    ESP_ERROR_CHECK(ax25_router_init());

    app_port.mode           = AX25_PORT_PROMISCUOUS;
    app_port.on_tx_frame       = app_port_frame_cb;
    err = ax25_router_register_port(&app_port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register App port: %d", err);
        ax25_router_deinit();
        return;
    }

    const ax25_phy_kiss_tcp_client_config_t phy_cfg = {
        .host               = CONFIG_MONITOR_SOUNDMODEM_HOST,
        .port               = CONFIG_MONITOR_SOUNDMODEM_PORT,
        .on_rx_frame           = phy_frame_cb,
        .user_data          = &phy_port,
    };
    if (ax25_phy_kiss_tcp_client_init(&phy_cfg, &phy_ctx) != ESP_OK) {
        ESP_LOGE(TAG, "PHY init failed");
        return;
    }

    phy_port.mode       = AX25_PORT_DEFAULT;
    phy_port.on_tx_frame   = phy_port_frame_cb;
    phy_port.user_data  = &phy_ctx;
    err = ax25_router_register_port(&phy_port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register TX port: %d", err);
        ax25_phy_kiss_tcp_client_deinit(&phy_ctx);
        ax25_router_remove_port(&app_port);
        ax25_router_deinit();
        return;
    }
    
    ESP_LOGI(TAG, "Monitoring started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        ESP_LOGI(TAG, "Frames sent: %lu  dropped: %lu",
                 (unsigned long)app_port.frames_sent,
                 (unsigned long)app_port.frames_dropped);

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
