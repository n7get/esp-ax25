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
 * @file ax25_address.h
 * @brief AX.25 address encoding and decoding utilities
 * 
 * This file provides utilities for encoding and decoding AX.25 addresses
 * according to the AX.25 specification. Addresses are 7 bytes on the wire
 * with bit-shifted callsign and SSID encoding.
 */

#ifndef AX25_ADDRESS_H
#define AX25_ADDRESS_H

#include "ax25_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize an address structure with defaults
 * 
 * @param addr Pointer to address structure
 */
void ax25_address_init(ax25_address_t* addr);

/**
 * @brief Encode an AX.25 address to wire format
 * 
 * Encodes the callsign by shifting each character left by 1 bit and
 * packs the SSID with H and extension bits into the 7th byte.
 * 
 * Wire format (7 bytes):
 * - Bytes 0-5: Callsign characters, each shifted left 1 bit, space-padded
 * - Byte 6: SSID in bits 1-4, reserved bits, H bit, extension bit
 * 
 * @param addr The address structure to encode
 * @param out7_bytes Output buffer (must be at least 7 bytes)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if parameters invalid
 */
esp_err_t ax25_address_encode(const ax25_address_t* addr, uint8_t* out7_bytes);

/**
 * @brief Decode an AX.25 address from wire format
 * 
 * Reverses the encoding process: shifts characters right by 1 bit,
 * extracts SSID and control bits from the 7th byte.
 * 
 * @param in7_bytes Input buffer containing encoded address (must be 7 bytes)
 * @param addr Output address structure
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if parameters invalid
 */
esp_err_t ax25_address_decode(const uint8_t* in7_bytes, ax25_address_t* addr);

/**
 * @brief Parse a callsign string into an address structure
 * 
 * Parses strings in the format "CALLSIGN" or "CALLSIGN-SSID".
 * "CALLSIGN" and "CALLSIGN-0" are treated equivalently.
 * Examples: "N0CALL", "N0CALL-0", "GB7BBS-1", "VK2XYZ-15"
 * 
 * @param callsign_ssid String to parse (e.g., "N0CALL-1")
 * @param addr Output address structure
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if format invalid
 */
esp_err_t ax25_address_from_string(const char* callsign_ssid, ax25_address_t* addr);

/**
 * @brief Convert an address structure to string format
 * 
 * Produces "CALLSIGN" when SSID is 0, otherwise "CALLSIGN-SSID".
 * 
 * @param addr Address structure to convert
 * @param buf Output buffer for string
 * @param buf_len Size of output buffer
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE if buffer too small
 */
esp_err_t ax25_address_to_string(const ax25_address_t* addr, char* buf, size_t buf_len);

/**
 * @brief Compare two addresses for equality
 * 
 * Compares callsign and SSID only (ignores H and extension bits).
 * 
 * @param a First address
 * @param b Second address
 * @return true if addresses match, false otherwise
 */
bool ax25_address_equals(const ax25_address_t* a, const ax25_address_t* b);

/**
 * @brief Copy an address structure
 * 
 * @param dst Destination address
 * @param src Source address
 */
void ax25_address_copy(ax25_address_t* dst, const ax25_address_t* src);

#ifdef __cplusplus
}
#endif

#endif /* AX25_ADDRESS_H */
