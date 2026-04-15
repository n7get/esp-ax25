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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "ax25_config.h"
#include "ax25_wifi.h"

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_LINK_BIT       BIT1

#define AX25_WIFI_DEFAULT_AP_SSID          "ESP-AX25"
#define AX25_WIFI_DEFAULT_STA_TIMEOUT_MS   15000
#define AX25_WIFI_DEFAULT_STA_MAX_WAITS    4

static const char *TAG = "AX25_WIFI";

typedef struct {
    char sta_ssid[AX25_CFG_VALUE_MAX_LEN];
    char sta_password[AX25_CFG_VALUE_MAX_LEN];
    char sta_hostname[AX25_CFG_VALUE_MAX_LEN];
    char ap_ssid[AX25_CFG_VALUE_MAX_LEN];
    char ap_password[AX25_CFG_VALUE_MAX_LEN];
    uint32_t sta_connect_timeout_ms;
    uint32_t sta_max_waits;
} ax25_wifi_runtime_cfg_t;

static bool is_empty_string(const char *s)
{
    return (s == NULL) || (s[0] == '\0');
}

static void load_runtime_config(ax25_wifi_runtime_cfg_t *cfg)
{
    if (cfg == NULL) {
        return;
    }

    ax25_cfg_get_str_global("wifi.sta.ssid", cfg->sta_ssid, sizeof(cfg->sta_ssid), "");
    ax25_cfg_get_str_global("wifi.sta.password", cfg->sta_password, sizeof(cfg->sta_password), "");
    ax25_cfg_get_str_global("wifi.sta.hostname", cfg->sta_hostname, sizeof(cfg->sta_hostname), "");
    ax25_cfg_get_str_global("wifi.ap.ssid", cfg->ap_ssid, sizeof(cfg->ap_ssid), "");
    ax25_cfg_get_str_global("wifi.ap.password", cfg->ap_password, sizeof(cfg->ap_password), "");

    cfg->sta_connect_timeout_ms = (uint32_t)ax25_cfg_get_int_global(
        "wifi.sta.connect_timeout_ms",
        AX25_WIFI_DEFAULT_STA_TIMEOUT_MS);
    cfg->sta_max_waits = (uint32_t)ax25_cfg_get_int_global(
        "wifi.sta.max_waits",
        AX25_WIFI_DEFAULT_STA_MAX_WAITS);
}

static void safe_strcpy(char *dst, size_t dst_size, const char *src)
{
    if (dst_size == 0) {
        return;
    }
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    strncpy(dst, src, dst_size);
    dst[dst_size - 1] = '\0';
}

static esp_err_t ensure_wifi_prerequisites(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_init_cfg);
    if (err != ESP_OK && err != ESP_ERR_WIFI_INIT_STATE) {
        return err;
    }

    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    ax25_wifi_t *ctx = (ax25_wifi_t *)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)event_data;
        xEventGroupClearBits(ctx->event_group, WIFI_CONNECTED_BIT | WIFI_LINK_BIT);
        ctx->retry_num++;
        ESP_LOGW(TAG, "STA disconnected (reason %d), retry %d", d->reason, ctx->retry_num);
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ctx->retry_num = 0;
        xEventGroupSetBits(ctx->event_group, WIFI_LINK_BIT);
        ESP_LOGI(TAG, "STA link up, waiting for DHCP");
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "STA connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(ctx->event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t start_sta(ax25_wifi_t *ctx, const ax25_wifi_runtime_cfg_t *config)
{
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!is_empty_string(config->sta_hostname)) {
        ESP_RETURN_ON_ERROR(esp_netif_set_hostname(sta_netif, config->sta_hostname),
                            TAG,
                            "set STA hostname failed");
    }

    wifi_config_t wifi_cfg = {};
    safe_strcpy((char *)wifi_cfg.sta.ssid, sizeof(wifi_cfg.sta.ssid), config->sta_ssid);
    safe_strcpy((char *)wifi_cfg.sta.password, sizeof(wifi_cfg.sta.password), config->sta_password);

    if (is_empty_string(config->sta_password)) {
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    } else {
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        wifi_cfg.sta.pmf_cfg.capable = true;
        wifi_cfg.sta.pmf_cfg.required = false;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set STA mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg), TAG, "set STA config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start WiFi failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "disable power save failed");

    if (!is_empty_string(config->sta_hostname)) {
        ESP_LOGI(TAG, "Using STA hostname \"%s\"", config->sta_hostname);
    }
    ESP_LOGI(TAG, "Connecting as STA to \"%s\"", config->sta_ssid);

    uint32_t timeout_ms = config->sta_connect_timeout_ms;
    if (timeout_ms == 0) {
        timeout_ms = AX25_WIFI_DEFAULT_STA_TIMEOUT_MS;
    }

    uint32_t max_waits = config->sta_max_waits;
    if (max_waits == 0) {
        max_waits = AX25_WIFI_DEFAULT_STA_MAX_WAITS;
    }

    for (uint32_t i = 0; i < max_waits; i++) {
        EventBits_t bits = xEventGroupWaitBits(ctx->event_group,
                                               WIFI_CONNECTED_BIT,
                                               pdFALSE,
                                               pdTRUE,
                                               pdMS_TO_TICKS(timeout_ms));
        if (bits & WIFI_CONNECTED_BIT) {
            ctx->mode = AX25_WIFI_MODE_STA;
            return ESP_OK;
        }

        bits = xEventGroupGetBits(ctx->event_group);
        if (bits & WIFI_LINK_BIT) {
            ESP_LOGW(TAG, "STA link up but no IP yet, reconnecting");
            esp_wifi_disconnect();
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "Still waiting for STA link...");
        }
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t start_ap(ax25_wifi_t *ctx, const ax25_wifi_runtime_cfg_t *config)
{
    const char *ap_ssid = is_empty_string(config->ap_ssid)
        ? AX25_WIFI_DEFAULT_AP_SSID
        : config->ap_ssid;

    wifi_config_t ap_cfg = {};
    safe_strcpy((char *)ap_cfg.ap.ssid, sizeof(ap_cfg.ap.ssid), ap_ssid);
    ap_cfg.ap.ssid_len = strlen((const char *)ap_cfg.ap.ssid);
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.channel = 1;

    if (is_empty_string(config->ap_password)) {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        safe_strcpy((char *)ap_cfg.ap.password, sizeof(ap_cfg.ap.password), config->ap_password);
        ap_cfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "set AP mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg), TAG, "set AP config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start AP failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "disable power save failed");

    ctx->mode = AX25_WIFI_MODE_AP;
    ESP_LOGW(TAG, "Started fallback AP SSID \"%s\"", ap_ssid);
    return ESP_OK;
}

esp_err_t ax25_wifi_start(ax25_wifi_t *ctx_public)
{
    if (ctx_public == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(ctx_public, 0, sizeof(*ctx_public));
    ax25_wifi_t *ctx = ctx_public;

    ctx->event_group = xEventGroupCreate();
    if (ctx->event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(ensure_wifi_prerequisites(), TAG, "WiFi prereq init failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, ctx, &ctx->wifi_handler),
        TAG, "register WIFI_EVENT handler failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, ctx, &ctx->ip_handler),
        TAG, "register IP_EVENT handler failed");

    ctx->handlers_registered = true;

    ax25_wifi_runtime_cfg_t config = {0};
    load_runtime_config(&config);

    if (!is_empty_string(config.sta_ssid)) {
        esp_err_t err = start_sta(ctx, &config);
        if (err == ESP_OK) {
            return ESP_OK;
        }

        ESP_LOGW(TAG, "STA connect failed (%s), switching to AP", esp_err_to_name(err));
        esp_wifi_stop();
    } else {
        ESP_LOGW(TAG, "STA SSID is empty, skipping STA and starting AP");
    }

    return start_ap(ctx, &config);
}
