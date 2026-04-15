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
 * @brief AX.25 AGWPE server example with UART default port
 *
 * Router setup:
 * - Default port: UART KISS TNC
 * - Client interface: TCP AGWPE (multiple clients)
 *
 * A singleton AGWPE manager is initialized once. Each TCP client connection
 * gets an attached managed AGWPE client handle.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"

#include "ax25_frame.h"
#include "ax25_config.h"
#include "ax25_router.h"
#include "ax25_phy_kiss_uart.h"
#include "ax25_phy_tcp_server.h"
#include "ax25_agwpe.h"
#include "ax25_agwpe_server.h"
#include "ax25_wifi.h"

static const char *TAG = "AGWPE_EX";

static esp_err_t configure_uart_keys_from_kconfig(void)
{
    char value[16];
    char err_msg[96] = {0};

    snprintf(value, sizeof(value), "%d", CONFIG_AGWPE_EXAMPLE_UART_NUM);
    if (ax25_cfg_set("uart.no", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.no: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_AGWPE_EXAMPLE_UART_BAUD_RATE);
    if (ax25_cfg_set("uart.baud", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.baud: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_AGWPE_EXAMPLE_UART_TX_PIN);
    if (ax25_cfg_set("uart.tx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.tx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_AGWPE_EXAMPLE_UART_RX_PIN);
    if (ax25_cfg_set("uart.rx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.rx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t configure_wifi_keys_from_kconfig(void)
{
    char err_msg[96] = {0};

    if (ax25_cfg_set("wifi.sta.ssid", CONFIG_AGWPE_EXAMPLE_WIFI_SSID,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.sta.ssid: %s", err_msg);
        return ESP_FAIL;
    }
    if (ax25_cfg_set("wifi.sta.password", CONFIG_AGWPE_EXAMPLE_WIFI_PASSWORD,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.sta.password: %s", err_msg);
        return ESP_FAIL;
    }
    if (ax25_cfg_set("wifi.ap.ssid", CONFIG_AGWPE_EXAMPLE_WIFI_AP_SSID,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.ap.ssid: %s", err_msg);
        return ESP_FAIL;
    }
    if (ax25_cfg_set("wifi.ap.password", CONFIG_AGWPE_EXAMPLE_WIFI_AP_PASSWORD,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.ap.password: %s", err_msg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t configure_tcp_port_from_kconfig(void)
{
    char err_msg[96] = {0};
    char value[16] = {0};
    int port = CONFIG_AGWPE_EXAMPLE_TCP_PORT;

    snprintf(value, sizeof(value), "%d", port);
    if (ax25_cfg_set("net.agwpe.port", value,
                    err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set net.agwpe.port: %s", err_msg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* UART default router port                                                    */
/* -------------------------------------------------------------------------- */

static void uart_port_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_phy_kiss_uart_send(frame, (ax25_phy_kiss_uart_t *)user_data);
}

static void uart_on_frame(const ax25_frame_t *frame, void *user_data)
{
    ax25_router_send(frame, (ax25_router_port_t *)user_data);
}

/* -------------------------------------------------------------------------- */
/* AGWPE TCP client handling                                                   */
/* -------------------------------------------------------------------------- */

typedef struct {
    bool                        in_use;
    ax25_phy_tcp_server_conn_t *conn;
    agwpe_decoder_t             decoder;
    ax25_agwpe_server_t        *client;
} agwpe_client_slot_t;

static agwpe_client_slot_t *s_clients;
static size_t               s_clients_len;
static StaticSemaphore_t   s_clients_mutex_buf;
static SemaphoreHandle_t   s_clients_mutex;

static agwpe_client_slot_t *alloc_client_slot(void)
{
    agwpe_client_slot_t *slot = NULL;

    xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
    for (size_t i = 0; i < s_clients_len; i++) {
        if (!s_clients[i].in_use) {
            slot = &s_clients[i];
            memset(slot, 0, sizeof(*slot));
            slot->in_use = true;
            break;
        }
    }
    xSemaphoreGive(s_clients_mutex);

    return slot;
}

static void free_client_slot(agwpe_client_slot_t *slot)
{
    if (slot == NULL) {
        return;
    }

    xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
    memset(slot, 0, sizeof(*slot));
    xSemaphoreGive(s_clients_mutex);
}

static void on_agwpe_frame_out(const agwpe_frame_t *frame, void *user_data)
{
    agwpe_client_slot_t *slot = (agwpe_client_slot_t *)user_data;
    if (slot == NULL || slot->conn == NULL) {
        return;
    }

    uint8_t out[AGWPE_MAX_FRAME_SIZE];
    size_t n = agwpe_frame_encode(frame, out, sizeof(out));
    if (n == 0) {
        ESP_LOGW(TAG, "Failed to encode AGWPE frame kind '%c'", frame->header.data_kind);
        return;
    }

    esp_err_t err = ax25_phy_tcp_server_conn_send(slot->conn, out, n);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to queue AGWPE frame for TCP transport: %s",
                 esp_err_to_name(err));
    }
}

static void on_agwpe_frame_in(const agwpe_frame_t *frame, void *user_data)
{
    agwpe_client_slot_t *slot = (agwpe_client_slot_t *)user_data;
    if (slot == NULL || slot->client == NULL) {
        return;
    }

    esp_err_t err = ax25_agwpe_server_client_agwpe_in(slot->client, frame);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ax25_agwpe_server_client_agwpe_in failed: %s", esp_err_to_name(err));
    }
}

static void tcp_on_connected(ax25_phy_tcp_server_conn_t *conn)
{
    agwpe_client_slot_t *slot = alloc_client_slot();
    if (slot == NULL) {
        ESP_LOGE(TAG, "No free AGWPE client slots");
        return;
    }

    slot->conn = conn;
    agwpe_decoder_init(&slot->decoder, on_agwpe_frame_in, slot);

    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame = on_agwpe_frame_out,
        .user_data = slot,
        .port = 0,
        .port_description = "UART KISS TNC",
        .tx_queue_depth = 0,
        .task_stack_size = 0,
        .task_priority = 0,
    };

    esp_err_t err = ax25_agwpe_server_add_client(&cfg, &slot->client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ax25_agwpe_server_add_client failed: %s", esp_err_to_name(err));
        free_client_slot(slot);
        return;
    }

    ax25_phy_tcp_server_conn_set_user_data(conn, slot);
    ESP_LOGI(TAG, "AGWPE client connected");
}

static void tcp_on_disconnected(ax25_phy_tcp_server_conn_t *conn)
{
    agwpe_client_slot_t *slot =
        (agwpe_client_slot_t *)ax25_phy_tcp_server_conn_get_user_data(conn);
    if (slot == NULL) {
        return;
    }

    if (slot->client != NULL) {
        ax25_agwpe_server_remove_client(slot->client);
        slot->client = NULL;
    }

    ax25_phy_tcp_server_conn_set_user_data(conn, NULL);
    free_client_slot(slot);
    ESP_LOGI(TAG, "AGWPE client disconnected");
}

static void tcp_on_data(ax25_phy_tcp_server_conn_t *conn,
                        const uint8_t *data,
                        size_t len)
{
    agwpe_client_slot_t *slot =
        (agwpe_client_slot_t *)ax25_phy_tcp_server_conn_get_user_data(conn);
    if (slot == NULL || slot->client == NULL) {
        return;
    }

    agwpe_decoder_process_bytes(&slot->decoder, data, len);
}

/* -------------------------------------------------------------------------- */
/* app_main                                                                    */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-AX25 AGWPE Server Example");
    ESP_LOGI(TAG, "============================");
    ESP_LOGI(TAG, "AGWPE TCP server: port %d", CONFIG_AGWPE_EXAMPLE_TCP_PORT);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(ax25_cfg_init(NULL, 0));
    ESP_ERROR_CHECK(configure_uart_keys_from_kconfig());
    ESP_ERROR_CHECK(configure_wifi_keys_from_kconfig());
    ESP_ERROR_CHECK(configure_tcp_port_from_kconfig());

    s_clients_len = (size_t)ax25_cfg_get_int_global("net.agwpe.server.max_conns", 2);
    if (s_clients_len < 1) {
        s_clients_len = 1;
    }
    s_clients = calloc(s_clients_len, sizeof(*s_clients));
    if (s_clients == NULL) {
        ESP_LOGE(TAG, "Failed to allocate AGWPE client slots (%d)", (int)s_clients_len);
        return;
    }

    static ax25_wifi_t wifi_ctx;

    err = ax25_wifi_start(&wifi_ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_ERROR_CHECK(ax25_router_init());
    ESP_ERROR_CHECK(ax25_agwpe_server_manager_init());

    static ax25_phy_kiss_uart_t uart_phy;
    static ax25_router_port_t   uart_port;

    uart_port.mode      = AX25_PORT_DEFAULT;
    uart_port.on_tx_frame  = uart_port_frame_cb;
    uart_port.user_data = &uart_phy;
    ESP_ERROR_CHECK(ax25_router_register_port(&uart_port));

    if (ax25_phy_kiss_uart_init(uart_on_frame, &uart_port, &uart_phy) != ESP_OK) {
        ESP_LOGE(TAG, "UART PHY init failed");
        return;
    }

    s_clients_mutex = xSemaphoreCreateMutexStatic(&s_clients_mutex_buf);
    if (s_clients_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create client mutex");
        return;
    }

    static ax25_phy_tcp_server_t tcp_srv;
    err = ax25_phy_tcp_server_init("net.agwpe.port",
                                   tcp_on_connected,
                                   tcp_on_disconnected,
                                   tcp_on_data,
                                   NULL,
                                   &tcp_srv);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AGWPE TCP server init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "AGWPE server running");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));

        ESP_LOGI(TAG, "UART port: sent=%lu dropped=%lu",
                 (unsigned long)uart_port.frames_sent,
                 (unsigned long)uart_port.frames_dropped);

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
