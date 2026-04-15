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

#include "unity.h"

#include <string.h>

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ax25_config.h"
#include "ax25_wifi.h"

static void ensure_nvs_ready(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_erase());
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_init());
    } else {
        TEST_ASSERT_EQUAL(ESP_OK, err);
    }
}

static void reset_config_state(void)
{
    ax25_cfg_deinit();
    ensure_nvs_ready();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_init(NULL, 0));
}

static void config_set_or_fail(const char *parameter, const char *value)
{
    char err[96] = {0};
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                              ax25_cfg_set(parameter, value, err, sizeof(err)),
                              err);
}

static void cleanup_wifi_runtime(ax25_wifi_t *ctx)
{
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    esp_wifi_stop();

    if (ctx != NULL && ctx->handlers_registered) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, ctx->wifi_handler);
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ctx->ip_handler);
        ctx->handlers_registered = false;
    }

    if (sta != NULL) {
        esp_netif_destroy_default_wifi(sta);
    }
    if (ap != NULL) {
        esp_netif_destroy_default_wifi(ap);
    }

    esp_wifi_deinit();

    if (ctx != NULL && ctx->event_group != NULL) {
        vEventGroupDelete(ctx->event_group);
        ctx->event_group = NULL;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

TEST_CASE("Wi-Fi: start rejects null context", "[ax25_wifi]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_wifi_start(NULL));
}

TEST_CASE("Wi-Fi: empty STA SSID starts fallback AP with default SSID", "[ax25_wifi]")
{
    ax25_wifi_t ctx = {0};
    wifi_mode_t mode = WIFI_MODE_NULL;
    wifi_config_t ap_cfg = {0};

    reset_config_state();
    config_set_or_fail("wifi.sta.ssid", "");
    config_set_or_fail("wifi.ap.ssid", "");
    config_set_or_fail("wifi.ap.password", "");

    TEST_ASSERT_EQUAL(ESP_OK, ax25_wifi_start(&ctx));
    TEST_ASSERT_TRUE(ctx.handlers_registered);
    TEST_ASSERT_EQUAL(AX25_WIFI_MODE_AP, ctx.mode);

    TEST_ASSERT_EQUAL(ESP_OK, esp_wifi_get_mode(&mode));
    TEST_ASSERT_EQUAL(WIFI_MODE_AP, mode);
    TEST_ASSERT_EQUAL(ESP_OK, esp_wifi_get_config(WIFI_IF_AP, &ap_cfg));
    TEST_ASSERT_EQUAL_STRING("ESP-AX25", (const char *)ap_cfg.ap.ssid);
    TEST_ASSERT_EQUAL(WIFI_AUTH_OPEN, ap_cfg.ap.authmode);

    cleanup_wifi_runtime(&ctx);
    ax25_cfg_deinit();
}

TEST_CASE("Wi-Fi: configured STA hostname is applied to the STA netif", "[ax25_wifi]")
{
    ax25_wifi_t ctx = {0};
    esp_netif_t *sta = NULL;
    const char *hostname = NULL;

    reset_config_state();
    config_set_or_fail("wifi.sta.ssid", "test-ssid");
    config_set_or_fail("wifi.sta.password", "");
    config_set_or_fail("wifi.sta.hostname", "esp-ax25-test");
    config_set_or_fail("wifi.sta.connect_timeout_ms", "1");
    config_set_or_fail("wifi.sta.max_waits", "1");
    config_set_or_fail("wifi.ap.ssid", "");
    config_set_or_fail("wifi.ap.password", "");

    TEST_ASSERT_EQUAL(ESP_OK, ax25_wifi_start(&ctx));
    TEST_ASSERT_TRUE(ctx.handlers_registered);

    sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    TEST_ASSERT_NOT_NULL(sta);
    TEST_ASSERT_EQUAL(ESP_OK, esp_netif_get_hostname(sta, &hostname));
    TEST_ASSERT_NOT_NULL(hostname);
    TEST_ASSERT_EQUAL_STRING("esp-ax25-test", hostname);

    cleanup_wifi_runtime(&ctx);
    ax25_cfg_deinit();
}