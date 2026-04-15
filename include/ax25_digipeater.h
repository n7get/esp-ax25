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
 * @file ax25_digipeater.h
 * @brief AX.25 digipeater module
 *
 * Registers a router port in AX25_PORT_DIGIPEATER mode when digi.callsign
 * is configured. The router handles next-hop matching and H-bit updates.
 * This module provides transparent MAC-layer relay by invoking the
 * application-provided transmit callback directly from the router port task.
 *
 * The on_transmit callback must not block.
 */

#ifndef AX25_DIGIPEATER_H
#define AX25_DIGIPEATER_H

#include <stdbool.h>
#include "esp_err.h"
#include "ax25_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** Called for each frame selected for digipeating. Must not be NULL.
     *  Called from the router port task; must not block. */
    ax25_send_frame_fn_t on_transmit;

    /** Opaque pointer forwarded to on_transmit. */
    void *user_data;
} ax25_digipeater_config_t;

/**
 * @brief Initialize digipeater module.
 *
 * If digi.callsign is empty, initialization succeeds but does not register
 * a digipeater port (feature stays disabled).
 *
 * @param config Module configuration. Must be non-NULL and include a valid
 *               transmit callback.
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG for invalid args or malformed digi.callsign
 * @return ESP_ERR_INVALID_STATE if config subsystem not initialized, module
 *         already initialized, or router registration fails with that status
 */
esp_err_t ax25_digipeater_init(const ax25_digipeater_config_t *config);

/**
 * @brief Deinitialize digipeater module.
 *
 * Safe to call even if not initialized.
 */
void ax25_digipeater_deinit(void);

/**
 * @brief Check if module was initialized.
 */
bool ax25_digipeater_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif /* AX25_DIGIPEATER_H */
