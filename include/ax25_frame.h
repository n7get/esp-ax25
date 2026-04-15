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
 * @file ax25_frame.h
 * @brief AX.25 frame parsing and building utilities
 * 
 * This file provides functionality for parsing raw AX.25 frame bytes into
 * structured frame objects and building raw bytes from frame objects.
 */

#ifndef AX25_FRAME_H
#define AX25_FRAME_H

#include "ax25_types.h"
#include "ax25_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize a frame structure with defaults
 * 
 * @param frame Pointer to frame structure
 */
void ax25_frame_init(ax25_frame_t* frame);

/**
 * @brief Parse raw AX.25 frame bytes into a frame structure
 * 
 * Parses the address field (destination, source, digipeaters), control byte,
 * PID (if present), and information field. Handles variable-length digipeater
 * lists and correctly identifies frame types.
 * 
 * Note: Input data should NOT include KISS framing or FCS (these are
 * handled by the KISS layer and physical layer respectively).
 * 
 * @param data Pointer to raw frame data
 * @param len Length of frame data
 * @param frame Output frame structure
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if frame invalid
 */
esp_err_t ax25_frame_parse(const uint8_t* data, size_t len, ax25_frame_t* frame);

/**
 * @brief Build raw AX.25 frame bytes from a frame structure
 * 
 * Encodes the complete frame including addresses, control byte, PID,
 * and information field. Output is ready to be sent via KISS.
 * 
 * Note: Does NOT add KISS framing or FCS.
 * 
 * @param frame Frame structure to encode
 * @param out Output buffer to receive raw bytes
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if frame invalid
 */
esp_err_t ax25_frame_build(const ax25_frame_t* frame, ax25_buffer_t* out);

/**
 * @brief Identify frame type from control byte
 * 
 * Determines whether the frame is I, S, U, or UI based on the
 * control byte bit pattern.
 * 
 * @param control Control byte
 * @return Frame type
 */
ax25_frame_type_t ax25_frame_identify_type(uint8_t control);

/**
 * @brief Extract N(S) from I-frame control byte
 * 
 * @param control I-frame control byte
 * @return Send sequence number (0-7)
 */
uint8_t ax25_frame_extract_ns(uint8_t control);

/**
 * @brief Extract N(R) from I or S-frame control byte
 * 
 * @param control Control byte
 * @return Receive sequence number (0-7)
 */
uint8_t ax25_frame_extract_nr(uint8_t control);

/**
 * @brief Build I-frame control byte
 * 
 * @param ns Send sequence number (0-7)
 * @param nr Receive sequence number (0-7)
 * @param pf Poll/Final bit
 * @return Control byte
 */
uint8_t ax25_frame_build_i_control(uint8_t ns, uint8_t nr, bool pf);

/**
 * @brief Build RR (Receive Ready) control byte
 * 
 * @param nr Receive sequence number (0-7)
 * @param pf Poll/Final bit
 * @return Control byte
 */
uint8_t ax25_frame_build_rr_control(uint8_t nr, bool pf);

/**
 * @brief Build RNR (Receive Not Ready) control byte
 * 
 * @param nr Receive sequence number (0-7)
 * @param pf Poll/Final bit
 * @return Control byte
 */
uint8_t ax25_frame_build_rnr_control(uint8_t nr, bool pf);

/**
 * @brief Build REJ (Reject) control byte
 * 
 * @param nr Receive sequence number (0-7)
 * @param pf Poll/Final bit
 * @return Control byte
 */
uint8_t ax25_frame_build_rej_control(uint8_t nr, bool pf);

/**
 * @brief Check if P/F bit is set in control byte
 * 
 * @param control Control byte
 * @return true if P/F bit is set
 */
static inline bool ax25_frame_has_pf(uint8_t control) {
    return (control & AX25_CTRL_PF_BIT) != 0;
}

#ifdef __cplusplus
}
#endif

#endif /* AX25_FRAME_H */
