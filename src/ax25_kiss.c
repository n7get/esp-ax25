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
 * @file ax25_kiss.c
 * @brief KISS protocol encoder/decoder implementation
 */

#include "ax25_kiss.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "AX25_KISS";

void ax25_kiss_decoder_init(ax25_kiss_decoder_t *decoder,
                            ax25_kiss_frame_cb_t callback,
                            void *user_data)
{
    if (!decoder) {
        return;
    }
    memset(decoder, 0, sizeof(ax25_kiss_decoder_t));
    decoder->state     = KISS_STATE_WAIT_FEND;
    decoder->callback  = callback;
    decoder->user_data = user_data;
}

void ax25_kiss_decoder_process_byte(ax25_kiss_decoder_t *decoder, uint8_t byte)
{
    if (!decoder) {
        return;
    }

    switch (decoder->state) {
        case KISS_STATE_WAIT_FEND:
            if (byte == AX25_KISS_FEND) {
                decoder->state     = KISS_STATE_WAIT_PORT_CMD;
                decoder->escaped   = false;
                decoder->frame_len = 0;
            }
            break;

        case KISS_STATE_WAIT_PORT_CMD:
            if (byte == AX25_KISS_FEND) {
                /* Duplicate FEND — stay in this state. */
                break;
            }
            decoder->port    = (byte >> 4) & 0x0F;
            decoder->command = byte & 0x0F;
            decoder->state   = KISS_STATE_RECEIVING;
            break;

        case KISS_STATE_RECEIVING:
            if (decoder->escaped) {
                uint8_t decoded;
                if (byte == AX25_KISS_TFEND) {
                    decoded = AX25_KISS_FEND;
                } else if (byte == AX25_KISS_TFESC) {
                    decoded = AX25_KISS_FESC;
                } else {
                    ESP_LOGW(TAG, "invalid escape 0x%02X, discarding frame", byte);
                    decoder->escaped   = false;
                    decoder->frame_len = 0;
                    decoder->state     = KISS_STATE_WAIT_FEND;
                    break;
                }
                decoder->escaped = false;
                if (decoder->frame_len < AX25_BUFFER_SIZE) {
                    decoder->frame_buf[decoder->frame_len++] = decoded;
                } else {
                    ESP_LOGW(TAG, "frame too large, discarding");
                    decoder->frame_len = 0;
                    decoder->state     = KISS_STATE_WAIT_FEND;
                }
            } else if (byte == AX25_KISS_FESC) {
                decoder->escaped = true;
            } else if (byte == AX25_KISS_FEND) {
                if (decoder->frame_len > 0 && decoder->callback) {
                    decoder->callback(decoder->port, decoder->command,
                                      decoder->frame_buf, decoder->frame_len,
                                      decoder->user_data);
                }
                decoder->frame_len = 0;
                decoder->state     = KISS_STATE_WAIT_FEND;
            } else {
                if (decoder->frame_len < AX25_BUFFER_SIZE) {
                    decoder->frame_buf[decoder->frame_len++] = byte;
                } else {
                    ESP_LOGW(TAG, "frame too large, discarding");
                    decoder->frame_len = 0;
                    decoder->state     = KISS_STATE_WAIT_FEND;
                }
            }
            break;
    }
}

void ax25_kiss_decoder_process_bytes(ax25_kiss_decoder_t *decoder,
                                      const uint8_t *data, size_t len)
{
    if (!decoder || !data) {
        return;
    }
    for (size_t i = 0; i < len; i++) {
        ax25_kiss_decoder_process_byte(decoder, data[i]);
    }
}

void ax25_kiss_decoder_reset(ax25_kiss_decoder_t *decoder)
{
    if (!decoder) {
        return;
    }
    decoder->state     = KISS_STATE_WAIT_FEND;
    decoder->escaped   = false;
    decoder->frame_len = 0;
    decoder->port      = 0;
    decoder->command   = 0;
}

/**
 * @brief Write one byte with KISS escaping into @p out at position @p pos.
 *
 * @return New position on success, or 0 if the buffer is full.
 */
static size_t kiss_add_escaped_byte(uint8_t byte, uint8_t *out,
                                    size_t pos, size_t out_size)
{
    if (byte == AX25_KISS_FEND) {
        if (pos + 2 > out_size) {
            return 0;
        }
        out[pos++] = AX25_KISS_FESC;
        out[pos++] = AX25_KISS_TFEND;
    } else if (byte == AX25_KISS_FESC) {
        if (pos + 2 > out_size) {
            return 0;
        }
        out[pos++] = AX25_KISS_FESC;
        out[pos++] = AX25_KISS_TFESC;
    } else {
        if (pos + 1 > out_size) {
            return 0;
        }
        out[pos++] = byte;
    }
    return pos;
}

size_t ax25_kiss_encode(uint8_t port, uint8_t command,
                        const uint8_t *data, size_t in_len,
                        uint8_t *out, size_t out_size)
{
    if (!data || !out || out_size == 0) {
        return 0;
    }

    size_t pos = 0;

    /* Opening FEND */
    if (pos + 1 > out_size) {
        return 0;
    }
    out[pos++] = AX25_KISS_FEND;

    /* Port/command byte (escaped if needed) */
    uint8_t port_cmd = ((port & 0x0F) << 4) | (command & 0x0F);
    pos = kiss_add_escaped_byte(port_cmd, out, pos, out_size);
    if (pos == 0) {
        return 0;
    }

    /* Data bytes (escaped as needed) */
    for (size_t i = 0; i < in_len; i++) {
        pos = kiss_add_escaped_byte(data[i], out, pos, out_size);
        if (pos == 0) {
            return 0;
        }
    }

    /* Closing FEND */
    if (pos + 1 > out_size) {
        return 0;
    }
    out[pos++] = AX25_KISS_FEND;

    return pos;
}
