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
 * @file ax25_print.h
 * @brief Human-readable pretty-printing of AX.25 frames
 *
 * Provides utilities for logging a complete AX.25 frame decode —
 * addresses, digipeater path, frame type with sequence numbers,
 * PID, and payload — at INFO level via ESP_LOGI.
 */

#ifndef AX25_PRINT_H
#define AX25_PRINT_H

#include "ax25_types.h"
#include "ax25_conn.h"
#include <stdint.h>
#include <stddef.h>

#define AX25_PRINT_LABEL_RX "RX"
#define AX25_PRINT_LABEL_TX "TX"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Pretty-print a parsed AX25Frame to the log at INFO level
 *
 * Emits multiple ESP_LOGI lines under the "AX25_PRINT" tag.
 *
 * @param label Short direction label, e.g. "RX" or "TX"
 * @param frame Parsed frame to print
 */
void ax25_print_frame(const char* label, const ax25_frame_t* frame);

/**
 * @brief Parse a raw AX.25 byte buffer and pretty-print its contents
 *
 * Calls ax25_frame_parse internally. Logs a parse-error line
 * if the buffer is not a valid AX.25 frame.
 *
 * @param label Short direction label, e.g. "RX" or "TX"
 * @param data  Raw frame bytes (no KISS framing, no FCS)
 * @param len   Number of bytes
 */
void ax25_print_buffer(const char* label, const uint8_t* data, size_t len);

/**
 * @brief Get frame type as string
 *
 * @param type Frame type
 * @return String representation
 */
const char* ax25_frame_type_str(ax25_frame_type_t type);

/**
 * @brief Get connection state as string
 *
 * @param state Connection state
 * @return String representation
 */
const char* ax25_conn_state_str(ax25_conn_state_t state);

/**
 * @brief Dump @p data as a hex + ASCII table via ESP_LOGI
 *
 * Emits a header line with @p label and byte count, then one line per 16
 * bytes showing the hex representation and printable ASCII characters.
 *
 * @param label Descriptive label printed on the header line (may be NULL)
 * @param data  Buffer to dump (no-op if NULL)
 * @param len   Number of bytes to dump (no-op if 0)
 */
void ax25_print_haxdump(const char* label, const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* AX25_PRINT_H */
