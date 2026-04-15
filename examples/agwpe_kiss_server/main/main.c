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
 * @brief AX.25 combined AGWPE + KISS TCP server with UART default router port
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
#include "ax25_kiss_tcp_server.h"
#include "ax25_phy_tcp_server.h"
#include "ax25_agwpe.h"
#include "ax25_agwpe_server.h"
#include "ax25_wifi.h"

static const char *TAG = "AGWPE_KISS_EX";

#define AGWPE_EXAMPLE_MAX_CLIENTS 1

static esp_err_t configure_uart_keys_from_kconfig(void)
{
    char value[16];
    char err_msg[96] = {0};

    snprintf(value, sizeof(value), "%d", CONFIG_AGWPE_KISS_EXAMPLE_UART_NUM);
    if (ax25_cfg_set("uart.no", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.no: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_AGWPE_KISS_EXAMPLE_UART_BAUD_RATE);
    if (ax25_cfg_set("uart.baud", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.baud: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_AGWPE_KISS_EXAMPLE_UART_TX_PIN);
    if (ax25_cfg_set("uart.tx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.tx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_AGWPE_KISS_EXAMPLE_UART_RX_PIN);
    if (ax25_cfg_set("uart.rx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.rx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t configure_wifi_keys_from_kconfig(void)
{
    char err_msg[96] = {0};

    if (ax25_cfg_set("wifi.sta.ssid", CONFIG_AGWPE_KISS_EXAMPLE_WIFI_SSID,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.sta.ssid: %s", err_msg);
        return ESP_FAIL;
    }
    if (ax25_cfg_set("wifi.sta.password", CONFIG_AGWPE_KISS_EXAMPLE_WIFI_PASSWORD,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.sta.password: %s", err_msg);
        return ESP_FAIL;
    }
    if (ax25_cfg_set("wifi.ap.ssid", CONFIG_AGWPE_KISS_EXAMPLE_WIFI_AP_SSID,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.ap.ssid: %s", err_msg);
        return ESP_FAIL;
    }
    if (ax25_cfg_set("wifi.ap.password", CONFIG_AGWPE_KISS_EXAMPLE_WIFI_AP_PASSWORD,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.ap.password: %s", err_msg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t configure_tcp_ports_from_kconfig(void)
{
    char err_msg[96] = {0};
    char value[16] = {0};
    int kiss_port = CONFIG_AGWPE_KISS_EXAMPLE_KISS_TCP_PORT;
    int agwpe_port = CONFIG_AGWPE_KISS_EXAMPLE_AGWPE_TCP_PORT;

    snprintf(value, sizeof(value), "%d", kiss_port);
    if (ax25_cfg_set("net.kiss.port", value,
                    err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set net.kiss.port: %s", err_msg);
        return ESP_FAIL;
    }
    snprintf(value, sizeof(value), "%d", agwpe_port);
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
/* KISS TCP client handling (dynamic router ports)                             */
/* -------------------------------------------------------------------------- */

typedef struct {
    ax25_router_port_t               port;
    bool                             in_use;
} kiss_port_slot_t;

static kiss_port_slot_t  *s_kiss_pool;
static size_t             s_kiss_pool_len;
static StaticSemaphore_t  s_kiss_pool_mutex_buf;
static SemaphoreHandle_t  s_kiss_pool_mutex;

static void kiss_port_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    esp_err_t err = ax25_kiss_tcp_server_conn_send(
        (ax25_kiss_tcp_server_conn_t *)user_data,
        frame);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to queue KISS frame for TCP client: %s",
                 esp_err_to_name(err));
    }
}

static void kiss_on_connected(ax25_kiss_tcp_server_conn_t *conn)
{
    xSemaphoreTake(s_kiss_pool_mutex, portMAX_DELAY);

    kiss_port_slot_t *slot = NULL;
    for (size_t i = 0; i < s_kiss_pool_len; i++) {
        if (!s_kiss_pool[i].in_use) {
            slot = &s_kiss_pool[i];
            slot->in_use = true;
            break;
        }
    }

    xSemaphoreGive(s_kiss_pool_mutex);

    if (slot == NULL) {
        ESP_LOGE(TAG, "kiss_on_connected: no free port slot");
        return;
    }

    memset(&slot->port, 0, sizeof(slot->port));
    slot->port.mode      = AX25_PORT_DYNAMIC;
    slot->port.on_tx_frame  = kiss_port_frame_cb;
    slot->port.user_data = conn;

    esp_err_t err = ax25_router_register_port(&slot->port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "kiss_on_connected: register_port failed (%d)", err);
        xSemaphoreTake(s_kiss_pool_mutex, portMAX_DELAY);
        slot->in_use = false;
        xSemaphoreGive(s_kiss_pool_mutex);
        return;
    }

    ax25_kiss_tcp_server_conn_set_user_data(conn, &slot->port);
    ESP_LOGI(TAG, "KISS client connected");
}

static void kiss_on_disconnected(ax25_kiss_tcp_server_conn_t *conn)
{
    ax25_router_port_t *port =
        (ax25_router_port_t *)ax25_kiss_tcp_server_conn_get_user_data(conn);
    if (port == NULL) {
        return;
    }

    ax25_router_remove_port(port);

    kiss_port_slot_t *slot = (kiss_port_slot_t *)port;
    xSemaphoreTake(s_kiss_pool_mutex, portMAX_DELAY);
    slot->in_use = false;
    xSemaphoreGive(s_kiss_pool_mutex);

    ESP_LOGI(TAG, "KISS client disconnected");
}

static void kiss_on_frame(ax25_kiss_tcp_server_conn_t *conn,
                          const ax25_frame_t *frame)
{
    ax25_router_port_t *port =
        (ax25_router_port_t *)ax25_kiss_tcp_server_conn_get_user_data(conn);
    if (port == NULL) {
        return;
    }

    ax25_router_send(frame, port);
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

static agwpe_client_slot_t s_agwpe_clients[AGWPE_EXAMPLE_MAX_CLIENTS];
static StaticSemaphore_t   s_agwpe_clients_mutex_buf;
static SemaphoreHandle_t   s_agwpe_clients_mutex;

static agwpe_client_slot_t *alloc_agwpe_client_slot(void)
{
    agwpe_client_slot_t *slot = NULL;

    xSemaphoreTake(s_agwpe_clients_mutex, portMAX_DELAY);
    for (int i = 0; i < AGWPE_EXAMPLE_MAX_CLIENTS; i++) {
        if (!s_agwpe_clients[i].in_use) {
            slot = &s_agwpe_clients[i];
            memset(slot, 0, sizeof(*slot));
            slot->in_use = true;
            break;
        }
    }
    xSemaphoreGive(s_agwpe_clients_mutex);

    return slot;
}

static void free_agwpe_client_slot(agwpe_client_slot_t *slot)
{
    if (slot == NULL) {
        return;
    }

    xSemaphoreTake(s_agwpe_clients_mutex, portMAX_DELAY);
    memset(slot, 0, sizeof(*slot));
    xSemaphoreGive(s_agwpe_clients_mutex);
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
        ESP_LOGW(TAG, "Failed to encode AGWPE frame kind '%c'",
                 frame->header.data_kind);
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
        ESP_LOGW(TAG, "ax25_agwpe_server_client_agwpe_in failed: %s",
                 esp_err_to_name(err));
    }
}

static void agwpe_on_connected(ax25_phy_tcp_server_conn_t *conn)
{
    agwpe_client_slot_t *slot = alloc_agwpe_client_slot();
    if (slot == NULL) {
        ESP_LOGE(TAG, "No free AGWPE client slots (max %d)", AGWPE_EXAMPLE_MAX_CLIENTS);
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
        free_agwpe_client_slot(slot);
        return;
    }

    ax25_phy_tcp_server_conn_set_user_data(conn, slot);
    ESP_LOGI(TAG, "AGWPE client connected");
}

static void agwpe_on_disconnected(ax25_phy_tcp_server_conn_t *conn)
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
    free_agwpe_client_slot(slot);
    ESP_LOGI(TAG, "AGWPE client disconnected");
}

static void agwpe_on_data(ax25_phy_tcp_server_conn_t *conn,
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
    ESP_LOGI(TAG, "ESP-AX25 AGWPE + KISS Server Example");
    ESP_LOGI(TAG, "===================================");
    ESP_LOGI(TAG, "AGWPE TCP port: %d",
             CONFIG_AGWPE_KISS_EXAMPLE_AGWPE_TCP_PORT);
    ESP_LOGI(TAG, "KISS TCP port: %d",
             CONFIG_AGWPE_KISS_EXAMPLE_KISS_TCP_PORT);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(ax25_cfg_init(NULL, 0));
    ESP_ERROR_CHECK(configure_uart_keys_from_kconfig());
    ESP_ERROR_CHECK(configure_wifi_keys_from_kconfig());
    ESP_ERROR_CHECK(configure_tcp_ports_from_kconfig());

    s_kiss_pool_len = (size_t)ax25_cfg_get_int_global("net.kiss.max_conns", 4);
    if (s_kiss_pool_len < 1) {
        s_kiss_pool_len = 1;
    }
    s_kiss_pool = calloc(s_kiss_pool_len, sizeof(*s_kiss_pool));
    if (s_kiss_pool == NULL) {
        ESP_LOGE(TAG, "Failed to allocate KISS port pool (%d)", (int)s_kiss_pool_len);
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

    s_kiss_pool_mutex = xSemaphoreCreateMutexStatic(&s_kiss_pool_mutex_buf);
    if (s_kiss_pool_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create KISS pool mutex");
        return;
    }

    s_agwpe_clients_mutex = xSemaphoreCreateMutexStatic(&s_agwpe_clients_mutex_buf);
    if (s_agwpe_clients_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create AGWPE clients mutex");
        return;
    }

    static ax25_kiss_tcp_server_t kiss_srv;
    err = ax25_kiss_tcp_server_init(kiss_on_connected,
                                    kiss_on_disconnected,
                                    kiss_on_frame,
                                    NULL,
                                    &kiss_srv);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "KISS TCP server init failed: %s", esp_err_to_name(err));
        return;
    }

    static ax25_phy_tcp_server_t agwpe_srv;
    err = ax25_phy_tcp_server_init("net.agwpe.port",
                                   agwpe_on_connected,
                                   agwpe_on_disconnected,
                                   agwpe_on_data,
                                   NULL,
                                   &agwpe_srv);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AGWPE TCP server init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Combined AGWPE + KISS server running");

    uint32_t prev_frames_sent = 0;
    uint32_t prev_frames_dropped = 0;
    uint32_t prev_used_now = 0;
    uint32_t prev_used_peak = 0;
    uint32_t prev_free_now = 0;
    uint32_t prev_total = 0;
    int stats_first = 1;
    int ram_first = 1;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));

        uint32_t frames_sent = uart_port.frames_sent;
        uint32_t frames_dropped = uart_port.frames_dropped;
        if (stats_first ||
            frames_sent != prev_frames_sent ||
            frames_dropped != prev_frames_dropped) {
            ESP_LOGI(TAG, "UART port: sent=%lu dropped=%lu",
                     (unsigned long)frames_sent,
                     (unsigned long)frames_dropped);
            prev_frames_sent = frames_sent;
            prev_frames_dropped = frames_dropped;
            stats_first = 0;
        }

        uint32_t total    = heap_caps_get_total_size(MALLOC_CAP_8BIT);
        uint32_t free_now = esp_get_free_heap_size();
        uint32_t free_min = esp_get_minimum_free_heap_size();
        uint32_t used_now = total - free_now;
        uint32_t used_peak = total - free_min;
        if (ram_first ||
            used_now != prev_used_now ||
            used_peak != prev_used_peak ||
            free_now != prev_free_now ||
            total != prev_total) {
            ESP_LOGI(TAG, "RAM: %lu used now, %lu peak used, %lu free of %lu total",
                     (unsigned long)used_now,
                     (unsigned long)used_peak,
                     (unsigned long)free_now,
                     (unsigned long)total);
            prev_used_now = used_now;
            prev_used_peak = used_peak;
            prev_free_now = free_now;
            prev_total = total;
            ram_first = 0;
        }
    }
}
