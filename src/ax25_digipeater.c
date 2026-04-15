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
 * @file ax25_digipeater.c
 * @brief AX.25 digipeater module implementation
 */

#include "ax25_digipeater.h"

#include <string.h>

#include "esp_log.h"
#include "ax25_address.h"
#include "ax25_config.h"
#include "ax25_router.h"

static const char *TAG = "AX25_DIGI";

typedef struct {
    bool initialized;
    bool enabled;
    bool port_registered;
    ax25_router_port_t port;
    ax25_send_frame_fn_t on_transmit;
    void *user_data;
} ax25_digipeater_ctx_t;

static ax25_digipeater_ctx_t s_ctx = {0};

static void digipeater_port_on_frame(const ax25_frame_t *frame, void *user_data)
{
    (void)user_data;

    if (frame == NULL || s_ctx.on_transmit == NULL) {
        return;
    }

    ax25_frame_t frame_copy = *frame;
    esp_err_t err = s_ctx.on_transmit(&frame_copy, s_ctx.user_data);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "relay transmit failed: %s", esp_err_to_name(err));
    }
}

esp_err_t ax25_digipeater_init(const ax25_digipeater_config_t *config)
{
    if (config == NULL || config->on_transmit == NULL) {
        ESP_LOGE(TAG, "init: invalid config");
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx.initialized) {
        ESP_LOGE(TAG, "init: already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!ax25_cfg_is_initialized()) {
        ESP_LOGE(TAG, "init: ax25_config not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.on_transmit = config->on_transmit;
    s_ctx.user_data = config->user_data;

    char digi_str[AX25_MAX_CALLSIGN_LEN + 5] = {0};
    ax25_cfg_get_str("digi.callsign", digi_str, sizeof(digi_str));

    if (digi_str[0] == '\0') {
        s_ctx.initialized = true;
        s_ctx.enabled = false;
        ESP_LOGI(TAG, "disabled: digi.callsign is empty");
        return ESP_OK;
    }

    ax25_address_t digi_addr;
    if (ax25_address_from_string(digi_str, &digi_addr) != ESP_OK) {
        ESP_LOGE(TAG, "init: invalid digi.callsign '%s'", digi_str);
        memset(&s_ctx, 0, sizeof(s_ctx));
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_ctx.port, 0, sizeof(s_ctx.port));
    s_ctx.port.mode = AX25_PORT_DIGIPEATER;
    s_ctx.port.destination = digi_addr;
    s_ctx.port.on_tx_frame = digipeater_port_on_frame;
    s_ctx.port.user_data = NULL;

    esp_err_t err = ax25_router_register_port(&s_ctx.port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init: register digipeater port failed: %s", esp_err_to_name(err));
        memset(&s_ctx, 0, sizeof(s_ctx));
        return err;
    }

    s_ctx.port_registered = true;
    s_ctx.initialized = true;
    s_ctx.enabled = true;

    ESP_LOGI(TAG, "enabled: callsign=%s", digi_str);
    return ESP_OK;
}

void ax25_digipeater_deinit(void)
{
    if (!s_ctx.initialized) {
        return;
    }

    if (s_ctx.port_registered) {
        esp_err_t err = ax25_router_remove_port(&s_ctx.port);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "deinit: remove port failed: %s", esp_err_to_name(err));
        }
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
}

bool ax25_digipeater_is_initialized(void)
{
    return s_ctx.initialized;
}
