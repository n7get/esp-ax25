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
 * @brief AX.25 Router - Bridge a UART KISS TNC with TCP KISS and AGWPE clients
 *
 * Sets up a UART KISS PHY as the default router port.  Additional ports are
 * optional and controlled via menuconfig ("AX.25 Router Configuration"):
 *
 *   ROUTER_WIFI_ENABLE  - Connect to WiFi and start a TCP KISS server so that
 *                         applications such as Direwolf or pat can reach the
 *                         router over the network.
 *
 *   ROUTER_BT_ENABLE    - Accept a Bluetooth Classic SPP connection as a
 *                         dynamic router port.  Hold the pairing button for
 *                         5 seconds to enter discoverable mode.
 *
 * Configuration options:
 *   - ROUTER_WIFI_SSID / ROUTER_WIFI_PASSWORD
 *   - ROUTER_WIFI_AP_SSID / ROUTER_WIFI_AP_PASSWORD
 *   - ROUTER_UART_NUM / ROUTER_UART_BAUD_RATE
 *   - ROUTER_UART_TX_PIN / ROUTER_UART_RX_PIN
 *   - ROUTER_TCP_PORT           (TCP KISS server port)
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"

#include "ax25_frame.h"
#include "ax25_config.h"
#include "ax25_router.h"
#include "ax25_phy_kiss_uart.h"
#include "ax25_wifi.h"

#if CONFIG_ROUTER_WIFI_ENABLE
#include "freertos/semphr.h"
#include "ax25_kiss_tcp_server.h"
#endif

#if CONFIG_ROUTER_BT_ENABLE
#include "ax25_phy_kiss_bt.h"
#endif

static const char *TAG = "ROUTER";

static esp_err_t configure_uart_keys_from_kconfig(void)
{
    char value[16];
    char err_msg[96] = {0};

    snprintf(value, sizeof(value), "%d", CONFIG_ROUTER_UART_NUM);
    if (ax25_cfg_set("uart.no", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.no: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_ROUTER_UART_BAUD_RATE);
    if (ax25_cfg_set("uart.baud", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.baud: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_ROUTER_UART_TX_PIN);
    if (ax25_cfg_set("uart.tx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.tx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_ROUTER_UART_RX_PIN);
    if (ax25_cfg_set("uart.rx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.rx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t configure_wifi_keys_from_kconfig(void)
{
    char err_msg[96] = {0};

    if (ax25_cfg_set("wifi.sta.ssid", CONFIG_ROUTER_WIFI_SSID,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.sta.ssid: %s", err_msg);
        return ESP_FAIL;
    }
    if (ax25_cfg_set("wifi.sta.password", CONFIG_ROUTER_WIFI_PASSWORD,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.sta.password: %s", err_msg);
        return ESP_FAIL;
    }
    if (ax25_cfg_set("wifi.ap.ssid", CONFIG_ROUTER_WIFI_AP_SSID,
                     err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set wifi.ap.ssid: %s", err_msg);
        return ESP_FAIL;
    }
    if (ax25_cfg_set("wifi.ap.password", CONFIG_ROUTER_WIFI_AP_PASSWORD,
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
    int port = CONFIG_ROUTER_TCP_PORT;

    snprintf(value, sizeof(value), "%d", port);
    if (ax25_cfg_set("net.kiss.port", value,
                    err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set net.kiss.port: %s", err_msg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// UART PHY callbacks
// ---------------------------------------------------------------------------

/* Router → UART: send frame out to the TNC */
static void uart_router_tx_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_phy_kiss_uart_send(frame, (ax25_phy_kiss_uart_t *)user_data);
}

/* UART → router: forward received frame */
static void uart_router_rx_ingress_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_router_send(frame, (ax25_router_port_t *)user_data);
}

// ---------------------------------------------------------------------------
// TCP client port pool (only when WiFi is enabled)
// ---------------------------------------------------------------------------

#if CONFIG_ROUTER_WIFI_ENABLE

typedef struct {
    ax25_router_port_t               port;   /* must be first member */
    bool                             in_use;
} tcp_port_slot_t;

static tcp_port_slot_t   *s_tcp_pool;
static size_t             s_tcp_pool_len;
static StaticSemaphore_t  s_pool_mutex_buf;
static SemaphoreHandle_t  s_pool_mutex;

/* Router → TCP client: send frame to the connection that owns this port */
static void tcp_client_router_tx_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    esp_err_t err = ax25_kiss_tcp_server_conn_send(
        (ax25_kiss_tcp_server_conn_t *)user_data,
        frame);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to queue KISS frame for TCP client: %s",
                 esp_err_to_name(err));
    }
}

static void tcp_on_connected(ax25_kiss_tcp_server_conn_t *conn)
{
    xSemaphoreTake(s_pool_mutex, portMAX_DELAY);
    tcp_port_slot_t *slot = NULL;
    for (size_t i = 0; i < s_tcp_pool_len; i++) {
        if (!s_tcp_pool[i].in_use) {
            slot = &s_tcp_pool[i];
            slot->in_use = true;
            break;
        }
    }
    xSemaphoreGive(s_pool_mutex);

    if (slot == NULL) {
        ESP_LOGE(TAG, "tcp_on_connected: no free port slot");
        return;
    }

    /* Zero the port so that destination is clear for DYNAMIC binding. */
    memset(&slot->port, 0, sizeof(slot->port));
    slot->port.mode      = AX25_PORT_DYNAMIC;
    slot->port.on_tx_frame  = tcp_client_router_tx_frame_cb;
    slot->port.user_data = conn;

    esp_err_t err = ax25_router_register_port(&slot->port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tcp_on_connected: register_port failed (%d)", err);
        xSemaphoreTake(s_pool_mutex, portMAX_DELAY);
        slot->in_use = false;
        xSemaphoreGive(s_pool_mutex);
        return;
    }

    /* Store the port pointer in the connection for use by other callbacks. */
    ax25_kiss_tcp_server_conn_set_user_data(conn, &slot->port);
    ESP_LOGI(TAG, "TCP client registered as dynamic router port");
}

static void tcp_on_disconnected(ax25_kiss_tcp_server_conn_t *conn)
{
    ax25_router_port_t *port = (ax25_router_port_t *)ax25_kiss_tcp_server_conn_get_user_data(conn);
    if (port == NULL) {
        return; /* on_connected failed to allocate a slot */
    }

    ax25_router_remove_port(port);

    /* port is the first member of tcp_port_slot_t, so the cast is safe. */
    tcp_port_slot_t *slot = (tcp_port_slot_t *)port;
    xSemaphoreTake(s_pool_mutex, portMAX_DELAY);
    slot->in_use = false;
    xSemaphoreGive(s_pool_mutex);
    ESP_LOGI(TAG, "TCP client removed from router");
}

static void tcp_router_rx_ingress_cb(ax25_kiss_tcp_server_conn_t *conn, const ax25_frame_t *frame)
{
    ax25_router_port_t *port = (ax25_router_port_t *)ax25_kiss_tcp_server_conn_get_user_data(conn);
    ax25_router_send(frame, port);
}

#endif /* CONFIG_ROUTER_WIFI_ENABLE */

// ---------------------------------------------------------------------------
// Bluetooth SPP port (single permanent DYNAMIC port)
// ---------------------------------------------------------------------------

#if CONFIG_ROUTER_BT_ENABLE

static ax25_phy_kiss_bt_t  s_bt_phy;
static ax25_router_port_t  s_bt_port;

/* Router → BT: deliver frame to the connected SPP client (drop if none). */
static void bt_port_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_phy_kiss_bt_send(frame, (ax25_phy_kiss_bt_t *)user_data);
}

/* BT → router: forward received frame into the router. */
static void bt_on_frame(const ax25_frame_t *frame, void *user_data)
{
    ax25_router_send(frame, (ax25_router_port_t *)user_data);
}

#endif /* CONFIG_ROUTER_BT_ENABLE */

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-AX25 Router");
    ESP_LOGI(TAG, "================");
#if CONFIG_ROUTER_WIFI_ENABLE
    ESP_LOGI(TAG, "WiFi + TCP KISS server: enabled (port %d)", CONFIG_ROUTER_TCP_PORT);
#endif
#if CONFIG_ROUTER_BT_ENABLE
    ESP_LOGI(TAG, "Bluetooth SPP: enabled");
#endif

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(ax25_cfg_init(NULL, 0));
    ESP_ERROR_CHECK(configure_uart_keys_from_kconfig());
    ESP_ERROR_CHECK(configure_wifi_keys_from_kconfig());

    ESP_ERROR_CHECK(ax25_router_init());

    /* ---- UART PHY (default router port) ---- */
    static ax25_phy_kiss_uart_t   uart_phy;
    static ax25_router_port_t     uart_port;

    uart_port.mode      = AX25_PORT_DEFAULT;
    uart_port.on_tx_frame  = uart_router_tx_frame_cb;
    uart_port.user_data = &uart_phy;
    ESP_ERROR_CHECK(ax25_router_register_port(&uart_port));

    if (ax25_phy_kiss_uart_init(uart_router_rx_ingress_cb, &uart_port, &uart_phy) != ESP_OK) {
        ESP_LOGE(TAG, "UART PHY init failed");
        return;
    }

#if CONFIG_ROUTER_WIFI_ENABLE
    /* ---- WiFi (required before TCP server) ---- */
    static ax25_wifi_t wifi_ctx;

    s_tcp_pool_len = (size_t)ax25_cfg_get_int_global("net.kiss.max_conns", 4);
    if (s_tcp_pool_len < 1) {
        s_tcp_pool_len = 1;
    }
    s_tcp_pool = calloc(s_tcp_pool_len, sizeof(*s_tcp_pool));
    if (s_tcp_pool == NULL) {
        ESP_LOGE(TAG, "Failed to allocate TCP client pool (%d)", (int)s_tcp_pool_len);
        return;
    }

    s_pool_mutex = xSemaphoreCreateMutexStatic(&s_pool_mutex_buf);
    ESP_ERROR_CHECK(s_pool_mutex == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    err = ax25_wifi_start(&wifi_ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
        return;
    }

    /* ---- TCP KISS server ---- */
    static ax25_kiss_tcp_server_t tcp_srv;

    ESP_ERROR_CHECK(configure_tcp_port_from_kconfig());
    if (ax25_kiss_tcp_server_init(tcp_on_connected,
                                  tcp_on_disconnected,
                                  tcp_router_rx_ingress_cb,
                                  NULL,
                                  &tcp_srv) != ESP_OK) {
        ESP_LOGE(TAG, "TCP server init failed");
        return;
    }
#endif /* CONFIG_ROUTER_WIFI_ENABLE */

#if CONFIG_ROUTER_BT_ENABLE
    /* ---- Bluetooth SPP PHY (permanent DYNAMIC router port) ---- */
    s_bt_port.mode      = AX25_PORT_DYNAMIC;
    s_bt_port.on_tx_frame  = bt_port_frame_cb;
    s_bt_port.user_data = &s_bt_phy;
    ESP_ERROR_CHECK(ax25_router_register_port(&s_bt_port));

    const ax25_phy_kiss_bt_config_t bt_cfg = {
        .on_rx_frame  = bt_on_frame,
        .user_data = &s_bt_port,
    };
    if (ax25_phy_kiss_bt_init(&bt_cfg, &s_bt_phy) != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth PHY init failed");
        return;
    }
#endif /* CONFIG_ROUTER_BT_ENABLE */

    ESP_LOGI(TAG, "Router running");

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
