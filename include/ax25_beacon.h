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
 * @file ax25_beacon.h
 * @brief AX.25 periodic beacon transmission module
 *
 * Provides periodic transmission of unnumbered information (UI) frames
 * at a configurable interval. The beacon can be sent to any destination
 * with optional digipeater path. Source address, destination, via path,
 * period, and message text are all configurable via ax25_config.
 *
 * Call ax25_beacon_init() once after ax25_router_init() to enable periodic
 * beacon transmissions.  Beacon frames follow the same router rules as any
 * other frame.
 */

#ifndef AX25_BEACON_H
#define AX25_BEACON_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the beacon module
 *
 * Must be called once after ax25_router_init(). Creates an internal FreeRTOS
 * timer and worker task to transmit beacon frames at the configured interval.
 * Beacon frames follow the same router rules as any other frame.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t ax25_beacon_init(void);

/**
 * @brief Deinitialize the beacon module
 *
 * Stops and deletes the internal timer and worker task. Safe to call even if
 * not initialized.
 */
void ax25_beacon_deinit(void);

/**
 * @brief Check if beacon module is initialized
 *
 * @return true if ax25_beacon_init() has been called successfully
 */
bool ax25_beacon_is_initialized(void);

/**
 * @brief Trigger an immediate beacon transmission
 *
 * Sends a beacon frame immediately from the caller's context, useful for
 * testing or on-demand transmission. The beacon content is read from the
 * current configuration.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ax25_beacon_trigger(void);

#ifdef __cplusplus
}
#endif

#endif /* AX25_BEACON_H */
