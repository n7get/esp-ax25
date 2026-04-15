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
 * @file ax25_types.h
 * @brief Core types, structures, and constants for AX.25 protocol implementation
 * 
 * This file contains all fundamental types, enumerations, and structures
 * used throughout the esp-ax25 component library (Pure C implementation).
 */

#ifndef AX25_TYPES_H
#define AX25_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * AX.25 Protocol Constants
 ******************************************************************************/

/** KISS frame end marker */
#define AX25_KISS_FEND      0xC0
/** KISS frame escape */
#define AX25_KISS_FESC      0xDB
/** KISS transposed frame end */
#define AX25_KISS_TFEND     0xDC
/** KISS transposed frame escape */
#define AX25_KISS_TFESC     0xDD

/** No layer 3 protocol / Plain text PID */
#define AX25_PID_NONE       0xF0
#define AX25_PID_TEXT       0xF0

/** Length of encoded address field (bytes) */
#define AX25_ADDRESS_LEN    7
/** Maximum number of digipeaters */
#define AX25_MAX_DIGIPEATERS 8
/** Maximum information field length */
#define AX25_MAX_INFO_LEN   256
/** Maximum callsign length (without SSID) */
#define AX25_MAX_CALLSIGN_LEN 6

/** Buffer size for maximum AX.25 frame + KISS overhead */
#define AX25_BUFFER_SIZE    (AX25_ADDRESS_LEN * (2 + AX25_MAX_DIGIPEATERS) + 2 + AX25_MAX_INFO_LEN + 10)

/** Default number of buffers in pool */
#define AX25_DEFAULT_BUFFER_POOL_SIZE 16

/*******************************************************************************
 * Control Byte Constants
 ******************************************************************************/

/** Unnumbered Information */
#define AX25_CTRL_UI        0x03
/** Set Async Balanced Mode */
#define AX25_CTRL_SABM      0x2F
/** Disconnect */
#define AX25_CTRL_DISC      0x43
/** Disconnect Mode */
#define AX25_CTRL_DM        0x0F
/** Unnumbered Acknowledge */
#define AX25_CTRL_UA        0x63
/** Frame Reject */
#define AX25_CTRL_FRMR      0x87

/** Receive Ready mask */
#define AX25_CTRL_RR_MASK   0x01
/** Receive Not Ready mask */
#define AX25_CTRL_RNR_MASK  0x05
/** Reject mask */
#define AX25_CTRL_REJ_MASK  0x09

/** I-frame mask */
#define AX25_CTRL_I_FRAME_MASK 0x01

/** Poll/Final bit */
#define AX25_CTRL_PF_BIT    0x10

/*******************************************************************************
 * Enumerations
 ******************************************************************************/

/**
 * @brief AX.25 frame types
 */
typedef enum {
    AX25_FRAME_UI,          /**< Unnumbered Information frame */
    AX25_FRAME_I,           /**< Information frame */
    AX25_FRAME_S,           /**< Supervisory frame */
    AX25_FRAME_U,           /**< Unnumbered frame */
    AX25_FRAME_UNKNOWN      /**< Unknown or invalid frame type */
} ax25_frame_type_t;

/**
 * @brief I/O mode for packet framing
 */
typedef enum {
    AX25_IO_MODE_KISS,      /**< KISS framing (default) */
    AX25_IO_MODE_RAW        /**< Raw packet mode (no KISS framing) */
} ax25_io_mode_t;

/*******************************************************************************
 * Structures
 ******************************************************************************/

/**
 * @brief AX.25 address structure (7 bytes on wire)
 * 
 * Represents a single AX.25 address with callsign and SSID.
 * Callsign is up to 6 ASCII characters, SSID is 0-15.
 */
typedef struct {
    char callsign[AX25_MAX_CALLSIGN_LEN + 1];   /**< Callsign (null-terminated, up to 6 chars) */
    uint8_t ssid;                               /**< Secondary Station Identifier (0-15) */
    bool has_been_repeated;                     /**< H bit: has this digipeater repeated the frame? */
    bool is_last;                               /**< Extension bit: is this the last address? */
} ax25_address_t;

/**
 * @brief Complete AX.25 frame structure
 * 
 * Represents a parsed AX.25 frame with destination, source, digipeaters,
 * control information, and payload.
 */
typedef struct {
    ax25_address_t destination;                 /**< Destination address */
    ax25_address_t source;                      /**< Source address */
    ax25_address_t digipeaters[AX25_MAX_DIGIPEATERS]; /**< Digipeater addresses */
    uint8_t num_digipeaters;                    /**< Number of digipeaters (0-8) */
    bool is_command;                            /**< C/R bit: true=command (dest bit7=1), false=response (src bit7=1) */
    ax25_frame_type_t type;                     /**< Frame type */
    uint8_t control;                            /**< Control byte */
    uint8_t pid;                                /**< Protocol ID (for I and UI frames) */
    uint8_t payload[AX25_MAX_INFO_LEN];         /**< Information field */
    size_t payload_len;                         /**< Length of payload */
} ax25_frame_t;

/*******************************************************************************
 * Callback Function Types
 ******************************************************************************/

/**
 * @brief Callback invoked for each received AX.25 frame
 *
 * Used by the router and PHY drivers to deliver decoded frames to the
 * application.  The frame pointer is only valid for the duration of the call;
 * copy the frame if it must outlive the callback.
 *
 * @param frame     Decoded AX.25 frame
 * @param user_data Opaque pointer supplied at configuration time
 */
typedef void (*ax25_on_frame_t)(const ax25_frame_t *frame, void *user_data);

/**
 * @brief Callback for sending raw AX.25 frames
 * 
 * @param data Pointer to raw frame data
 * @param len Length of frame data in bytes
 * @param user_data User context pointer
 * @return ESP_OK on success, error code otherwise
 */
typedef esp_err_t (*ax25_send_raw_fn_t)(const uint8_t* data, size_t len, void* user_data);

/**
 * @brief Callback for sending parsed AX.25 frames
 *
 * @param frame Pointer to AX.25 frame
 * @param user_data User context pointer
 * @return ESP_OK on success, error code otherwise
 */
typedef esp_err_t (*ax25_send_frame_fn_t)(const ax25_frame_t* frame, void* user_data);

/**
 * @brief Callback for connection established event
 * 
 * @param user_data User context pointer
 */
typedef void (*ax25_on_connected_fn_t)(void* user_data);

/**
 * @brief Callback for connection closed event
 * 
 * @param user_data User context pointer
 */
typedef void (*ax25_on_disconnected_fn_t)(void* user_data);

/**
 * @brief Callback for received data
 * 
 * @param data Pointer to received data
 * @param len Length of received data
 * @param user_data User context pointer
 */
typedef void (*ax25_on_data_fn_t)(const uint8_t* data, size_t len, void* user_data);

/**
 * @brief Callback for error conditions
 * 
 * @param error Error code
 * @param message Human-readable error message
 * @param user_data User context pointer
 */
typedef void (*ax25_on_error_fn_t)(esp_err_t error, const char* message, void* user_data);

/**
 * @brief Forward declaration of connection handle
 */
struct ax25_connection;
typedef struct ax25_connection ax25_connection_t;

/**
 * @brief Callback for incoming connection requests
 * 
 * @param conn Pointer to the new connection object
 * @param user_data User context pointer
 */
typedef void (*ax25_on_incoming_conn_fn_t)(ax25_connection_t* conn, void* user_data);

#ifdef __cplusplus
}
#endif

#endif /* AX25_TYPES_H */
