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
 * @file ax25_agwpe.h
 * @brief AGWPE (AGW Packet Engine) protocol encoder/decoder for TNC communication
 *
 * Implements AGWPE protocol framing for interfacing with packet radio TNCs
 * and software modems. Provides conversion between AX.25 frames and AGWPE format.
 *
 * AGWPE frame structure:
 * - 36-byte header followed by optional data payload
 * - Little-endian byte order for multi-byte fields
 */

#ifndef AX25_AGWPE_H
#define AX25_AGWPE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "ax25_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * AGWPE Protocol Constants
 ******************************************************************************/

/** AGWPE header size in bytes */
#define AGWPE_HEADER_SIZE       36

/** Maximum callsign length in AGWPE header (including SSID) */
#define AGWPE_CALLSIGN_LEN      10

/** Maximum AGWPE data payload size */
#define AGWPE_MAX_DATA_LEN      512

/** Maximum complete AGWPE frame size (header + data) */
#define AGWPE_MAX_FRAME_SIZE    (AGWPE_HEADER_SIZE + AGWPE_MAX_DATA_LEN)

/*******************************************************************************
 * AGWPE Data Kind (Frame Type) Constants
 ******************************************************************************/

/**
 * @brief AGWPE frame type codes (data_kind field)
 *
 * These are ASCII character codes used to identify different frame types.
 */
typedef enum {
    /** 'R' - AGWPE version request */
    AGWPE_KIND_VERSION_REQ      = 'R',
    /** 'R' - AGWPE version response (same as request) */
    AGWPE_KIND_VERSION_RESP     = 'R',
    /** 'G' - Request port information */
    AGWPE_KIND_PORT_INFO_REQ    = 'G',
    /** 'G' - Port information response */
    AGWPE_KIND_PORT_INFO_RESP   = 'G',
    /** 'g' - Request port capabilities */
    AGWPE_KIND_PORT_CAP_REQ     = 'g',
    /** 'g' - Port capabilities response */
    AGWPE_KIND_PORT_CAP_RESP    = 'g',
    /** 'X' - Register callsign */
    AGWPE_KIND_REGISTER_CALL    = 'X',
    /** 'x' - Unregister callsign */
    AGWPE_KIND_UNREGISTER_CALL  = 'x',
    /** 'H' - Request heard stations */
    AGWPE_KIND_HEARD_REQ        = 'H',
    /** 'C' - Connect request */
    AGWPE_KIND_CONNECT_REQ      = 'C',
    /** 'v' - Connect via digipeaters request */
    AGWPE_KIND_CONNECT_VIA_REQ  = 'v',
    /** 'c' - Connect response / connection established */
    AGWPE_KIND_CONNECT_RESP     = 'c',
    /** 'D' - Send connected data */
    AGWPE_KIND_SEND_DATA        = 'D',
    /** 'D' - Receive connected data */
    AGWPE_KIND_RECV_DATA        = 'D',
    /** 'd' - Disconnect request */
    AGWPE_KIND_DISCONNECT_REQ   = 'd',
    /** 'd' - Disconnect response */
    AGWPE_KIND_DISCONNECT_RESP  = 'd',
    /** 'M' - Send unproto (UI) frame */
    AGWPE_KIND_SEND_UNPROTO     = 'M',
    /** 'V' - Send unproto via digipeaters */
    AGWPE_KIND_SEND_UNPROTO_VIA = 'V',
    /** 'U' - Received monitored unproto frame */
    AGWPE_KIND_RECV_UNPROTO     = 'U',
    /** 'S' - Received monitored supervisory frame */
    AGWPE_KIND_RECV_SUPERVISORY = 'S',
    /** 'I' - Received monitored I-frame */
    AGWPE_KIND_RECV_I_FRAME     = 'I',
    /** 'T' - Received raw AX.25 frame */
    AGWPE_KIND_RECV_RAW         = 'T',
    /** 'K' - Send raw AX.25 frame */
    AGWPE_KIND_SEND_RAW         = 'K',
    /** 'k' - Enable raw frame reception */
    AGWPE_KIND_ENABLE_RAW       = 'k',
    /** 'm' - Enable monitoring */
    AGWPE_KIND_ENABLE_MONITOR   = 'm',
    /** 'Y' - Outstanding frames query */
    AGWPE_KIND_OUTSTANDING_REQ  = 'Y',
    /** 'y' - Outstanding frames response */
    AGWPE_KIND_OUTSTANDING_RESP = 'y',
    /** 'y' - Frames waiting on connection */
    AGWPE_KIND_FRAMES_WAITING   = 'y'
} agwpe_data_kind_t;

/*******************************************************************************
 * AGWPE Structures
 ******************************************************************************/

/**
 * @brief AGWPE frame header structure (36 bytes)
 *
 * All multi-byte fields are stored in little-endian format on the wire.
 * This structure uses native byte order; conversion is handled during
 * encoding/decoding.
 */
typedef struct __attribute__((packed)) {
    uint8_t  port;                          /**< Radio port number (0-255) */
    uint8_t  reserved1[3];                  /**< Reserved, must be 0 */
    uint8_t  data_kind;                     /**< Frame type code (ASCII) */
    uint8_t  reserved2[1];                  /**< Reserved, must be 0 */
    uint8_t  pid;                           /**< Protocol ID (for data frames) */
    uint8_t  reserved3[1];                  /**< Reserved, must be 0 */
    char     call_from[AGWPE_CALLSIGN_LEN]; /**< Source callsign (null-padded) */
    char     call_to[AGWPE_CALLSIGN_LEN];   /**< Destination callsign (null-padded) */
    uint32_t data_len;                      /**< Length of data following header */
    uint32_t user;                          /**< User-defined field */
} agwpe_header_t;

/**
 * @brief Complete AGWPE frame (header + data)
 */
typedef struct {
    agwpe_header_t header;                  /**< 36-byte frame header */
    uint8_t        data[AGWPE_MAX_DATA_LEN]; /**< Data payload */
} agwpe_frame_t;

/**
 * @brief AGWPE version information
 */
typedef struct {
    uint16_t major;     /**< Major version number */
    uint16_t minor;     /**< Minor version number (reserved, typically 0) */
} agwpe_version_t;

/**
 * @brief AGWPE port capabilities
 */
typedef struct {
    uint8_t on_air_baud;    /**< On-air baud rate indicator */
    uint8_t traffic_level;  /**< Current traffic level (0-255) */
    uint8_t tx_delay;       /**< TX delay in 10ms units */
    uint8_t tx_tail;        /**< TX tail in 10ms units */
    uint8_t persist;        /**< Persistence (0-255) */
    uint8_t slot_time;      /**< Slot time in 10ms units */
    uint8_t max_frame;      /**< Maximum frames to send */
    uint8_t active_conns;   /**< Number of active connections */
    uint32_t bytes_recv;    /**< Total bytes received */
} agwpe_port_caps_t;

/*******************************************************************************
 * Callback Function Types
 ******************************************************************************/

/**
 * @brief Callback invoked for each decoded AGWPE frame
 *
 * @param frame     Pointer to the decoded AGWPE frame
 * @param user_data User context pointer
 */
typedef void (*agwpe_frame_cb_t)(const agwpe_frame_t *frame, void *user_data);

/*******************************************************************************
 * AGWPE Decoder Context
 ******************************************************************************/

/**
 * @brief AGWPE decoder state
 */
typedef enum {
    AGWPE_STATE_HEADER,     /**< Waiting to receive header bytes */
    AGWPE_STATE_DATA        /**< Receiving data payload */
} agwpe_decoder_state_t;

/**
 * @brief AGWPE decoder context
 *
 * Assembles incoming bytes into complete AGWPE frames. The decoder
 * is not thread-safe; it should be driven from a single task.
 */
typedef struct {
    agwpe_decoder_state_t state;            /**< Current decoder state */
    agwpe_frame_t         frame;            /**< Frame being assembled */
    size_t                bytes_received;   /**< Bytes received for current part */
    agwpe_frame_cb_t      callback;         /**< Frame callback */
    void                 *user_data;        /**< User context */
} agwpe_decoder_t;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

/**
 * @brief Initialize an AGWPE frame structure with defaults
 *
 * @param frame Pointer to frame structure to initialize
 */
void agwpe_frame_init(agwpe_frame_t *frame);

/**
 * @brief Initialize an AGWPE decoder
 *
 * @param decoder   Pointer to decoder structure
 * @param callback  Function called when a complete frame is decoded
 * @param user_data Opaque pointer forwarded to callback
 */
void agwpe_decoder_init(agwpe_decoder_t *decoder,
                        agwpe_frame_cb_t callback,
                        void *user_data);

/**
 * @brief Reset decoder state, discarding any partial frame
 *
 * @param decoder Pointer to decoder
 */
void agwpe_decoder_reset(agwpe_decoder_t *decoder);

/*******************************************************************************
 * Decoding Functions
 ******************************************************************************/

/**
 * @brief Process a single incoming byte
 *
 * @param decoder Pointer to initialized decoder
 * @param byte    Byte to process
 */
void agwpe_decoder_process_byte(agwpe_decoder_t *decoder, uint8_t byte);

/**
 * @brief Process multiple incoming bytes
 *
 * @param decoder Pointer to initialized decoder
 * @param data    Pointer to data
 * @param len     Number of bytes to process
 */
void agwpe_decoder_process_bytes(agwpe_decoder_t *decoder,
                                  const uint8_t *data, size_t len);

/*******************************************************************************
 * Encoding Functions
 ******************************************************************************/

/**
 * @brief Encode an AGWPE frame to bytes
 *
 * Converts the frame structure to wire format (little-endian).
 *
 * @param frame    Pointer to frame to encode
 * @param out      Output buffer (must be at least AGWPE_HEADER_SIZE + frame->header.data_len)
 * @param out_size Size of output buffer
 * @return Number of bytes written, or 0 on error
 */
size_t agwpe_frame_encode(const agwpe_frame_t *frame,
                          uint8_t *out, size_t out_size);

/**
 * @brief Decode raw bytes into an AGWPE frame
 *
 * Parses a complete AGWPE frame from raw bytes.
 *
 * @param data   Raw frame data (header + payload)
 * @param len    Length of data
 * @param frame  Output frame structure
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if invalid
 */
esp_err_t agwpe_frame_decode(const uint8_t *data, size_t len,
                              agwpe_frame_t *frame);

/*******************************************************************************
 * AX.25 <-> AGWPE Conversion Functions
 ******************************************************************************/

/**
 * @brief Convert an AX.25 frame to AGWPE raw frame format ('K' frame)
 *
 * Encodes the AX.25 frame and wraps it in an AGWPE 'K' (raw) frame.
 *
 * @param ax25_frame Input AX.25 frame
 * @param port       Radio port number
 * @param agwpe_out  Output AGWPE frame
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ax25_to_agwpe_raw(const ax25_frame_t *ax25_frame,
                             uint8_t port,
                             agwpe_frame_t *agwpe_out);

/**
 * @brief Convert an AX.25 UI frame to AGWPE unproto format ('M' or 'V' frame)
 *
 * Creates an AGWPE unproto frame suitable for sending UI data.
 * If the AX.25 frame has digipeaters, creates a 'V' frame; otherwise 'M'.
 *
 * @param ax25_frame Input AX.25 UI frame
 * @param port       Radio port number
 * @param agwpe_out  Output AGWPE frame
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ax25_to_agwpe_unproto(const ax25_frame_t *ax25_frame,
                                 uint8_t port,
                                 agwpe_frame_t *agwpe_out);

/**
 * @brief Convert an AGWPE raw frame ('T' frame) to AX.25 frame
 *
 * Extracts the AX.25 frame data from an AGWPE 'T' (received raw) frame.
 *
 * @param agwpe_frame Input AGWPE frame (must be 'T' type)
 * @param ax25_out    Output AX.25 frame
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t agwpe_raw_to_ax25(const agwpe_frame_t *agwpe_frame,
                             ax25_frame_t *ax25_out);

/**
 * @brief Convert an AGWPE monitored frame to AX.25 frame
 *
 * Handles 'U' (unproto), 'I' (I-frame), and 'S' (supervisory) monitored frames.
 *
 * @param agwpe_frame Input AGWPE frame
 * @param ax25_out    Output AX.25 frame
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t agwpe_monitored_to_ax25(const agwpe_frame_t *agwpe_frame,
                                   ax25_frame_t *ax25_out);

/*******************************************************************************
 * AGWPE Frame Builder Functions
 ******************************************************************************/

/**
 * @brief Build an AGWPE version request frame
 *
 * @param agwpe_out Output AGWPE frame
 */
void agwpe_build_version_req(agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE port information request frame
 *
 * @param agwpe_out Output AGWPE frame
 */
void agwpe_build_port_info_req(agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE port capabilities request frame
 *
 * @param port      Radio port number
 * @param agwpe_out Output AGWPE frame
 */
void agwpe_build_port_cap_req(uint8_t port, agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE register callsign frame
 *
 * @param port      Radio port number
 * @param callsign  Callsign to register (with optional SSID)
 * @param agwpe_out Output AGWPE frame
 * @return ESP_OK on success
 */
esp_err_t agwpe_build_register_call(uint8_t port, const char *callsign,
                                     agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE unregister callsign frame
 *
 * @param port      Radio port number
 * @param callsign  Callsign to unregister
 * @param agwpe_out Output AGWPE frame
 * @return ESP_OK on success
 */
esp_err_t agwpe_build_unregister_call(uint8_t port, const char *callsign,
                                       agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE connect request frame
 *
 * @param port      Radio port number
 * @param from_call Source callsign
 * @param to_call   Destination callsign
 * @param agwpe_out Output AGWPE frame
 * @return ESP_OK on success
 */
esp_err_t agwpe_build_connect_req(uint8_t port,
                                   const char *from_call,
                                   const char *to_call,
                                   agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE connect via digipeaters request frame
 *
 * @param port          Radio port number
 * @param from_call     Source callsign
 * @param to_call       Destination callsign
 * @param digipeaters   Array of digipeater callsigns
 * @param num_digis     Number of digipeaters
 * @param agwpe_out     Output AGWPE frame
 * @return ESP_OK on success
 */
esp_err_t agwpe_build_connect_via_req(uint8_t port,
                                       const char *from_call,
                                       const char *to_call,
                                       const char *digipeaters[],
                                       uint8_t num_digis,
                                       agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE disconnect request frame
 *
 * @param port      Radio port number
 * @param from_call Source callsign
 * @param to_call   Destination callsign
 * @param agwpe_out Output AGWPE frame
 * @return ESP_OK on success
 */
esp_err_t agwpe_build_disconnect_req(uint8_t port,
                                      const char *from_call,
                                      const char *to_call,
                                      agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE send connected data frame
 *
 * @param port      Radio port number
 * @param from_call Source callsign
 * @param to_call   Destination callsign
 * @param data      Data to send
 * @param data_len  Length of data
 * @param pid       Protocol ID
 * @param agwpe_out Output AGWPE frame
 * @return ESP_OK on success
 */
esp_err_t agwpe_build_send_data(uint8_t port,
                                 const char *from_call,
                                 const char *to_call,
                                 const uint8_t *data,
                                 size_t data_len,
                                 uint8_t pid,
                                 agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE send unproto (UI) frame
 *
 * @param port      Radio port number
 * @param from_call Source callsign
 * @param to_call   Destination callsign
 * @param data      Data to send
 * @param data_len  Length of data
 * @param pid       Protocol ID
 * @param agwpe_out Output AGWPE frame
 * @return ESP_OK on success
 */
esp_err_t agwpe_build_send_unproto(uint8_t port,
                                    const char *from_call,
                                    const char *to_call,
                                    const uint8_t *data,
                                    size_t data_len,
                                    uint8_t pid,
                                    agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE send unproto via digipeaters frame
 *
 * @param port          Radio port number
 * @param from_call     Source callsign
 * @param to_call       Destination callsign
 * @param digipeaters   Array of digipeater callsigns
 * @param num_digis     Number of digipeaters
 * @param data          Data to send
 * @param data_len      Length of data
 * @param pid           Protocol ID
 * @param agwpe_out     Output AGWPE frame
 * @return ESP_OK on success
 */
esp_err_t agwpe_build_send_unproto_via(uint8_t port,
                                        const char *from_call,
                                        const char *to_call,
                                        const char *digipeaters[],
                                        uint8_t num_digis,
                                        const uint8_t *data,
                                        size_t data_len,
                                        uint8_t pid,
                                        agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE enable monitoring frame
 *
 * @param agwpe_out Output AGWPE frame
 */
void agwpe_build_enable_monitor(agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE enable raw frame reception frame
 *
 * @param agwpe_out Output AGWPE frame
 */
void agwpe_build_enable_raw(agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE outstanding frames query
 *
 * @param port      Radio port number
 * @param from_call Source callsign
 * @param to_call   Destination callsign
 * @param agwpe_out Output AGWPE frame
 * @return ESP_OK on success
 */
esp_err_t agwpe_build_outstanding_req(uint8_t port,
                                       const char *from_call,
                                       const char *to_call,
                                       agwpe_frame_t *agwpe_out);

/**
 * @brief Build an AGWPE heard stations request frame
 *
 * @param port      Radio port number
 * @param agwpe_out Output AGWPE frame
 */
void agwpe_build_heard_req(uint8_t port, agwpe_frame_t *agwpe_out);

/*******************************************************************************
 * AGWPE Response Parsing Functions
 ******************************************************************************/

/**
 * @brief Parse AGWPE version response
 *
 * @param frame      AGWPE frame (must be 'R' type with data)
 * @param version    Output version structure
 * @return ESP_OK on success
 */
esp_err_t agwpe_parse_version_resp(const agwpe_frame_t *frame,
                                    agwpe_version_t *version);

/**
 * @brief Parse AGWPE port capabilities response
 *
 * @param frame  AGWPE frame (must be 'g' type)
 * @param caps   Output capabilities structure
 * @return ESP_OK on success
 */
esp_err_t agwpe_parse_port_caps(const agwpe_frame_t *frame,
                                 agwpe_port_caps_t *caps);

/**
 * @brief Get number of outstanding frames from 'y' response
 *
 * @param frame         AGWPE frame (must be 'y' type)
 * @param outstanding   Output count of outstanding frames
 * @return ESP_OK on success
 */
esp_err_t agwpe_parse_outstanding_resp(const agwpe_frame_t *frame,
                                        uint32_t *outstanding);

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/**
 * @brief Get a human-readable name for an AGWPE frame type
 *
 * @param kind AGWPE data kind code
 * @return String name for the frame type
 */
const char *agwpe_kind_to_string(uint8_t kind);

/**
 * @brief Pretty-print an AGWPE frame to the log at INFO level
 *
 * Emits multiple ESP_LOGI lines under the "AGWPE_PRINT" tag, showing
 * frame kind, port, callsigns, PID, data length, and a hex/ASCII data dump.
 *
 * @param label Short direction label, e.g. "RX" or "TX"
 * @param frame AGWPE frame to print
 */
void agwpe_print_frame(const char *label, const agwpe_frame_t *frame);

/**
 * @brief Set callsign in AGWPE header field
 *
 * Copies the callsign and pads with nulls.
 *
 * @param dest   Destination buffer (must be AGWPE_CALLSIGN_LEN bytes)
 * @param call   Callsign string (null-terminated)
 */
void agwpe_set_callsign(char *dest, const char *call);

/**
 * @brief Extract callsign from AGWPE header field
 *
 * Copies the callsign, handling null padding.
 *
 * @param src    Source buffer (AGWPE_CALLSIGN_LEN bytes)
 * @param dest   Destination string (must be at least AGWPE_CALLSIGN_LEN + 1 bytes)
 */
void agwpe_get_callsign(const char *src, char *dest);

/**
 * @brief Check if an AGWPE frame is a request type
 *
 * @param kind AGWPE data kind code
 * @return true if the frame type is a request
 */
bool agwpe_is_request(uint8_t kind);

/**
 * @brief Check if an AGWPE frame is a monitored frame type
 *
 * @param kind AGWPE data kind code
 * @return true if the frame type is monitored data
 */
bool agwpe_is_monitored(uint8_t kind);

#ifdef __cplusplus
}
#endif

#endif /* AX25_AGWPE_H */
