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
 * @file ax25_kiss.h
 * @brief KISS protocol encoder/decoder for TNC communication
 *
 * Implements KISS (Keep It Simple Stupid) protocol framing for TNC
 * communication.  The decoder is designed for single-task use; it carries
 * no internal synchronisation primitives — the caller must ensure that
 * ax25_kiss_decoder_process_byte[s]() and ax25_kiss_decoder_reset() are
 * not called concurrently.
 */

#ifndef AX25_KISS_H
#define AX25_KISS_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "ax25_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum size of a KISS-encoded frame in bytes.
 *
 * Worst case: every byte in an AX25_BUFFER_SIZE-byte frame requires
 * escaping, plus the opening FEND, port/command byte, and closing FEND.
 */
#define AX25_KISS_MAX_ENCODED_SIZE  ((AX25_BUFFER_SIZE) * 2 + 4)

/**
 * @brief KISS decoder state
 */
typedef enum {
    KISS_STATE_WAIT_FEND,       /**< Waiting for FEND to start frame */
    KISS_STATE_WAIT_PORT_CMD,   /**< Waiting for port/command byte */
    KISS_STATE_RECEIVING        /**< Receiving frame data */
} ax25_kiss_state_t;

/**
 * @brief Callback invoked with each decoded KISS frame
 *
 * @param port      KISS port number (0-15)
 * @param command   KISS command (0 = data frame)
 * @param data      Pointer to raw AX.25 frame data
 * @param len       Length of frame data
 * @param user_data User context pointer
 */
typedef void (*ax25_kiss_frame_cb_t)(uint8_t port, uint8_t command,
                                      const uint8_t *data, size_t len,
                                      void *user_data);

/**
 * @brief KISS decoder context
 *
 * Allocate this structure (typically on the stack or as a static variable)
 * and pass it to ax25_kiss_decoder_init() before use.  The decoder
 * assembles incoming bytes into an internal fixed-size buffer; no dynamic
 * memory allocation is performed.
 *
 * The decoder is not thread-safe.  It is intended to be driven from a
 * single task.
 */
typedef struct {
    ax25_kiss_state_t    state;                       /**< Current decoder state */
    bool                 escaped;                     /**< Inside escape sequence */
    uint8_t              port;                        /**< Current frame port */
    uint8_t              command;                     /**< Current frame command */
    uint8_t              frame_buf[AX25_BUFFER_SIZE]; /**< Frame assembly buffer */
    size_t               frame_len;                   /**< Bytes assembled so far */
    ax25_kiss_frame_cb_t callback;                    /**< Frame callback */
    void                *user_data;                   /**< User context */
} ax25_kiss_decoder_t;

/**
 * @brief Initialise a KISS decoder
 *
 * Zeroes the structure and stores the callback.  Cannot fail.
 *
 * @param decoder   Pointer to the decoder structure to initialise
 * @param callback  Function called when a complete frame is decoded
 * @param user_data Opaque pointer forwarded to every callback invocation
 */
void ax25_kiss_decoder_init(ax25_kiss_decoder_t *decoder,
                            ax25_kiss_frame_cb_t callback,
                            void *user_data);

/**
 * @brief Process a single incoming byte
 *
 * @param decoder Pointer to an initialised decoder
 * @param byte    Byte to process
 */
void ax25_kiss_decoder_process_byte(ax25_kiss_decoder_t *decoder, uint8_t byte);

/**
 * @brief Process multiple incoming bytes
 *
 * @param decoder Pointer to an initialised decoder
 * @param data    Pointer to data
 * @param len     Number of bytes to process
 */
void ax25_kiss_decoder_process_bytes(ax25_kiss_decoder_t *decoder,
                                      const uint8_t *data, size_t len);

/**
 * @brief Reset decoder state, discarding any partially-assembled frame
 *
 * @param decoder Pointer to the decoder
 */
void ax25_kiss_decoder_reset(ax25_kiss_decoder_t *decoder);

/**
 * @brief Encode a raw AX.25 frame into KISS format
 *
 * Writes the encoded frame into @p out.  The caller must supply a buffer
 * of at least AX25_KISS_MAX_ENCODED_SIZE bytes (or @c (in_len * 2 + 4)
 * for frame-specific sizing).
 *
 * @param port      KISS port (0-15)
 * @param command   KISS command (0 = data frame)
 * @param data      Raw AX.25 frame data
 * @param in_len    Length of input data
 * @param out       Output buffer
 * @param out_size  Size of the output buffer in bytes
 * @return Number of bytes written, or 0 on error (NULL args or buffer too small)
 */
size_t ax25_kiss_encode(uint8_t port, uint8_t command,
                        const uint8_t *data, size_t in_len,
                        uint8_t *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* AX25_KISS_H */
