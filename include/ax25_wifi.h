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

#ifndef AX25_WIFI_H
#define AX25_WIFI_H

#include <stdbool.h>
#include <stdint.h>

#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_event.h"

typedef enum {
    AX25_WIFI_MODE_NONE = 0,
    AX25_WIFI_MODE_STA,
    AX25_WIFI_MODE_AP,
} ax25_wifi_mode_t;

typedef struct {
    EventGroupHandle_t event_group;
    int retry_num;
    bool handlers_registered;
    ax25_wifi_mode_t mode;
    esp_event_handler_instance_t wifi_handler;
    esp_event_handler_instance_t ip_handler;
} ax25_wifi_t;

/**
 * Starts Wi-Fi using values read from ax25_config:
 * - wifi.sta.ssid
 * - wifi.sta.password
 * - wifi.sta.hostname
 * - wifi.sta.connect_timeout_ms
 * - wifi.sta.max_waits
 * - wifi.ap.ssid
 * - wifi.ap.password
 *
 * If wifi.sta.ssid is empty, the module skips STA and starts fallback AP.
 */
esp_err_t ax25_wifi_start(ax25_wifi_t *ctx);

#endif /* AX25_WIFI_H */
