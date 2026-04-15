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
 * @file ax25_agwpe.c
 * @brief AGWPE protocol encoder/decoder implementation
 */

#include "ax25_agwpe.h"
#include "ax25_frame.h"
#include "ax25_address.h"
#include "ax25_buffer.h"
#include <string.h>
#include <ctype.h>
#include "esp_log.h"

static const char *TAG = "AGWPE_PRINT";

/*******************************************************************************
 * Private Helper Functions
 ******************************************************************************/

/**
 * @brief Read a 32-bit little-endian value from buffer
 */
static uint32_t read_le32(const uint8_t *buf)
{
    return (uint32_t)buf[0] |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

/**
 * @brief Write a 32-bit little-endian value to buffer
 */
static void write_le32(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

/**
 * @brief Read a 16-bit little-endian value from buffer
 */
static uint16_t read_le16(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

/**
 * @brief Convert AX.25 address to AGWPE callsign format
 */
static void ax25_addr_to_agwpe_call(const ax25_address_t *addr, char *dest)
{
    char buf[AGWPE_CALLSIGN_LEN + 1];
    ax25_address_to_string(addr, buf, AGWPE_CALLSIGN_LEN);
    agwpe_set_callsign(dest, buf);
}

/**
 * @brief Convert AGWPE callsign to AX.25 address
 */
static esp_err_t agwpe_call_to_ax25_addr(const char *src, ax25_address_t *addr)
{
    char buf[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(src, buf);
    return ax25_address_from_string(buf, addr);
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

void agwpe_frame_init(agwpe_frame_t *frame)
{
    if (frame == NULL) {
        return;
    }
    memset(frame, 0, sizeof(agwpe_frame_t));
}

void agwpe_decoder_init(agwpe_decoder_t *decoder,
                        agwpe_frame_cb_t callback,
                        void *user_data)
{
    if (decoder == NULL) {
        return;
    }
    memset(decoder, 0, sizeof(agwpe_decoder_t));
    decoder->state = AGWPE_STATE_HEADER;
    decoder->callback = callback;
    decoder->user_data = user_data;
}

void agwpe_decoder_reset(agwpe_decoder_t *decoder)
{
    if (decoder == NULL) {
        return;
    }
    decoder->state = AGWPE_STATE_HEADER;
    decoder->bytes_received = 0;
    memset(&decoder->frame, 0, sizeof(agwpe_frame_t));
}

/*******************************************************************************
 * Decoding Functions
 ******************************************************************************/

void agwpe_decoder_process_byte(agwpe_decoder_t *decoder, uint8_t byte)
{
    if (decoder == NULL) {
        return;
    }

    uint8_t *frame_bytes = (uint8_t *)&decoder->frame;

    switch (decoder->state) {
        case AGWPE_STATE_HEADER:
            frame_bytes[decoder->bytes_received++] = byte;
            
            if (decoder->bytes_received >= AGWPE_HEADER_SIZE) {
                /* Header complete - decode data_len (little-endian at offset 28) */
                uint32_t data_len = read_le32(&frame_bytes[28]);
                
                /* Clamp data length to maximum */
                if (data_len > AGWPE_MAX_DATA_LEN) {
                    data_len = AGWPE_MAX_DATA_LEN;
                }
                decoder->frame.header.data_len = data_len;
                
                if (data_len > 0) {
                    decoder->state = AGWPE_STATE_DATA;
                    decoder->bytes_received = 0;
                } else {
                    /* No data - frame is complete */
                    if (decoder->callback) {
                        decoder->callback(&decoder->frame, decoder->user_data);
                    }
                    agwpe_decoder_reset(decoder);
                }
            }
            break;

        case AGWPE_STATE_DATA:
            if (decoder->bytes_received < AGWPE_MAX_DATA_LEN) {
                decoder->frame.data[decoder->bytes_received] = byte;
            }
            decoder->bytes_received++;
            
            if (decoder->bytes_received >= decoder->frame.header.data_len) {
                /* Frame complete */
                if (decoder->callback) {
                    decoder->callback(&decoder->frame, decoder->user_data);
                }
                agwpe_decoder_reset(decoder);
            }
            break;
    }
}

void agwpe_decoder_process_bytes(agwpe_decoder_t *decoder,
                                  const uint8_t *data, size_t len)
{
    if (decoder == NULL || data == NULL) {
        return;
    }
    
    for (size_t i = 0; i < len; i++) {
        agwpe_decoder_process_byte(decoder, data[i]);
    }
}

/*******************************************************************************
 * Encoding Functions
 ******************************************************************************/

size_t agwpe_frame_encode(const agwpe_frame_t *frame,
                          uint8_t *out, size_t out_size)
{
    if (frame == NULL || out == NULL) {
        return 0;
    }

    size_t total_len = AGWPE_HEADER_SIZE + frame->header.data_len;
    if (out_size < total_len) {
        return 0;
    }

    /* Clear output buffer */
    memset(out, 0, AGWPE_HEADER_SIZE);

    /* Encode header fields */
    out[0] = frame->header.port;
    /* bytes 1-3: reserved */
    out[4] = frame->header.data_kind;
    /* byte 5: reserved */
    out[6] = frame->header.pid;
    /* byte 7: reserved */
    
    /* Copy callsigns (bytes 8-17 and 18-27) */
    memcpy(&out[8], frame->header.call_from, AGWPE_CALLSIGN_LEN);
    memcpy(&out[18], frame->header.call_to, AGWPE_CALLSIGN_LEN);
    
    /* Data length (bytes 28-31, little-endian) */
    write_le32(&out[28], frame->header.data_len);
    
    /* User field (bytes 32-35, little-endian) */
    write_le32(&out[32], frame->header.user);

    /* Copy data if present */
    if (frame->header.data_len > 0) {
        memcpy(&out[AGWPE_HEADER_SIZE], frame->data, frame->header.data_len);
    }

    return total_len;
}

esp_err_t agwpe_frame_decode(const uint8_t *data, size_t len,
                              agwpe_frame_t *frame)
{
    if (data == NULL || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (len < AGWPE_HEADER_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    agwpe_frame_init(frame);

    /* Decode header fields */
    frame->header.port = data[0];
    frame->header.data_kind = data[4];
    frame->header.pid = data[6];
    
    /* Copy callsigns */
    memcpy(frame->header.call_from, &data[8], AGWPE_CALLSIGN_LEN);
    memcpy(frame->header.call_to, &data[18], AGWPE_CALLSIGN_LEN);
    
    /* Data length */
    frame->header.data_len = read_le32(&data[28]);
    
    /* User field */
    frame->header.user = read_le32(&data[32]);

    /* Validate and copy data */
    if (frame->header.data_len > 0) {
        if (len < AGWPE_HEADER_SIZE + frame->header.data_len) {
            return ESP_ERR_INVALID_SIZE;
        }
        
        size_t copy_len = frame->header.data_len;
        if (copy_len > AGWPE_MAX_DATA_LEN) {
            copy_len = AGWPE_MAX_DATA_LEN;
        }
        memcpy(frame->data, &data[AGWPE_HEADER_SIZE], copy_len);
        frame->header.data_len = (uint32_t)copy_len;
    }

    return ESP_OK;
}

/*******************************************************************************
 * AX.25 <-> AGWPE Conversion Functions
 ******************************************************************************/

esp_err_t ax25_to_agwpe_raw(const ax25_frame_t *ax25_frame,
                             uint8_t port,
                             agwpe_frame_t *agwpe_out)
{
    if (ax25_frame == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    agwpe_frame_init(agwpe_out);
    
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_SEND_RAW;
    agwpe_out->header.pid = ax25_frame->pid;
    
    /* Set callsigns */
    ax25_addr_to_agwpe_call(&ax25_frame->source, agwpe_out->header.call_from);
    ax25_addr_to_agwpe_call(&ax25_frame->destination, agwpe_out->header.call_to);

    /* Build the raw AX.25 frame into the data field */
    /* We need to encode the AX.25 frame manually since we don't have a buffer pool */
    size_t pos = 0;

    /* Destination address */
    ax25_address_t dest_copy = ax25_frame->destination;
    dest_copy.is_last = false; /* destination is never the last address */
    ax25_address_encode(&dest_copy, &agwpe_out->data[pos]);
    pos += AX25_ADDRESS_LEN;

    /* Source address */
    ax25_address_t src_copy = ax25_frame->source;
    src_copy.is_last = (ax25_frame->num_digipeaters == 0);
    ax25_address_encode(&src_copy, &agwpe_out->data[pos]);
    pos += AX25_ADDRESS_LEN;

    /* Digipeaters */
    for (uint8_t i = 0; i < ax25_frame->num_digipeaters && i < AX25_MAX_DIGIPEATERS; i++) {
        ax25_address_t digi_copy = ax25_frame->digipeaters[i];
        digi_copy.is_last = (i == ax25_frame->num_digipeaters - 1);
        ax25_address_encode(&digi_copy, &agwpe_out->data[pos]);
        pos += AX25_ADDRESS_LEN;
    }

    /* Control byte */
    agwpe_out->data[pos++] = ax25_frame->control;

    /* PID (for I and UI frames) */
    if (ax25_frame->type == AX25_FRAME_I || ax25_frame->type == AX25_FRAME_UI) {
        agwpe_out->data[pos++] = ax25_frame->pid;
    }

    /* Payload */
    if (ax25_frame->payload_len > 0) {
        if (pos + ax25_frame->payload_len > AGWPE_MAX_DATA_LEN) {
            return ESP_ERR_NO_MEM;
        }
        memcpy(&agwpe_out->data[pos], ax25_frame->payload, ax25_frame->payload_len);
        pos += ax25_frame->payload_len;
    }

    agwpe_out->header.data_len = (uint32_t)pos;
    return ESP_OK;
}

esp_err_t ax25_to_agwpe_unproto(const ax25_frame_t *ax25_frame,
                                 uint8_t port,
                                 agwpe_frame_t *agwpe_out)
{
    if (ax25_frame == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    agwpe_frame_init(agwpe_out);
    
    agwpe_out->header.port = port;
    agwpe_out->header.pid = ax25_frame->pid;
    
    /* Set callsigns */
    ax25_addr_to_agwpe_call(&ax25_frame->source, agwpe_out->header.call_from);
    ax25_addr_to_agwpe_call(&ax25_frame->destination, agwpe_out->header.call_to);

    if (ax25_frame->num_digipeaters > 0) {
        /* 'V' frame - unproto via digipeaters */
        agwpe_out->header.data_kind = AGWPE_KIND_SEND_UNPROTO_VIA;
        
        /* Data format: <num_digis> <digi1>\0 <digi2>\0 ... <payload> */
        size_t pos = 0;
        agwpe_out->data[pos++] = ax25_frame->num_digipeaters;
        
        for (uint8_t i = 0; i < ax25_frame->num_digipeaters && i < AX25_MAX_DIGIPEATERS; i++) {
            char digi_call[AGWPE_CALLSIGN_LEN + 1];
            ax25_address_to_string(&ax25_frame->digipeaters[i], digi_call, AGWPE_CALLSIGN_LEN);
            size_t call_len = strlen(digi_call);
            if (pos + call_len + 1 > AGWPE_MAX_DATA_LEN) {
                return ESP_ERR_NO_MEM;
            }
            memcpy(&agwpe_out->data[pos], digi_call, call_len + 1);
            pos += call_len + 1;
        }
        
        /* Add payload */
        if (ax25_frame->payload_len > 0) {
            if (pos + ax25_frame->payload_len > AGWPE_MAX_DATA_LEN) {
                return ESP_ERR_NO_MEM;
            }
            memcpy(&agwpe_out->data[pos], ax25_frame->payload, ax25_frame->payload_len);
            pos += ax25_frame->payload_len;
        }
        
        agwpe_out->header.data_len = (uint32_t)pos;
    } else {
        /* 'M' frame - simple unproto */
        agwpe_out->header.data_kind = AGWPE_KIND_SEND_UNPROTO;
        
        if (ax25_frame->payload_len > AGWPE_MAX_DATA_LEN) {
            return ESP_ERR_NO_MEM;
        }
        memcpy(agwpe_out->data, ax25_frame->payload, ax25_frame->payload_len);
        agwpe_out->header.data_len = (uint32_t)ax25_frame->payload_len;
    }

    return ESP_OK;
}

esp_err_t agwpe_raw_to_ax25(const agwpe_frame_t *agwpe_frame,
                             ax25_frame_t *ax25_out)
{
    if (agwpe_frame == NULL || ax25_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 'T' frame contains raw AX.25 data */
    if (agwpe_frame->header.data_kind != AGWPE_KIND_RECV_RAW) {
        return ESP_ERR_INVALID_ARG;
    }

    return ax25_frame_parse(agwpe_frame->data, agwpe_frame->header.data_len, ax25_out);
}

esp_err_t agwpe_monitored_to_ax25(const agwpe_frame_t *agwpe_frame,
                                   ax25_frame_t *ax25_out)
{
    if (agwpe_frame == NULL || ax25_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ax25_frame_init(ax25_out);

    uint8_t kind = agwpe_frame->header.data_kind;

    /* Extract callsigns from header */
    esp_err_t err = agwpe_call_to_ax25_addr(agwpe_frame->header.call_from, &ax25_out->source);
    if (err != ESP_OK) {
        return err;
    }
    
    err = agwpe_call_to_ax25_addr(agwpe_frame->header.call_to, &ax25_out->destination);
    if (err != ESP_OK) {
        return err;
    }

    ax25_out->pid = agwpe_frame->header.pid;

    switch (kind) {
        case AGWPE_KIND_RECV_UNPROTO:
            ax25_out->type = AX25_FRAME_UI;
            ax25_out->control = AX25_CTRL_UI;
            break;

        case AGWPE_KIND_RECV_I_FRAME:
            ax25_out->type = AX25_FRAME_I;
            /* Control byte would need to be parsed from data if available */
            ax25_out->control = 0;
            break;

        case AGWPE_KIND_RECV_SUPERVISORY:
            ax25_out->type = AX25_FRAME_S;
            ax25_out->control = 0;
            break;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    /* Copy payload data */
    /* Note: AGWPE monitored frames may include header text before actual data */
    /* For simplicity, we copy all data as payload */
    if (agwpe_frame->header.data_len > 0) {
        size_t copy_len = agwpe_frame->header.data_len;
        if (copy_len > AX25_MAX_INFO_LEN) {
            copy_len = AX25_MAX_INFO_LEN;
        }
        memcpy(ax25_out->payload, agwpe_frame->data, copy_len);
        ax25_out->payload_len = copy_len;
    }

    return ESP_OK;
}

/*******************************************************************************
 * AGWPE Frame Builder Functions
 ******************************************************************************/

void agwpe_build_version_req(agwpe_frame_t *agwpe_out)
{
    if (agwpe_out == NULL) {
        return;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.data_kind = AGWPE_KIND_VERSION_REQ;
}

void agwpe_build_port_info_req(agwpe_frame_t *agwpe_out)
{
    if (agwpe_out == NULL) {
        return;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.data_kind = AGWPE_KIND_PORT_INFO_REQ;
}

void agwpe_build_port_cap_req(uint8_t port, agwpe_frame_t *agwpe_out)
{
    if (agwpe_out == NULL) {
        return;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_PORT_CAP_REQ;
}

esp_err_t agwpe_build_register_call(uint8_t port, const char *callsign,
                                     agwpe_frame_t *agwpe_out)
{
    if (callsign == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_REGISTER_CALL;
    agwpe_set_callsign(agwpe_out->header.call_from, callsign);
    return ESP_OK;
}

esp_err_t agwpe_build_unregister_call(uint8_t port, const char *callsign,
                                       agwpe_frame_t *agwpe_out)
{
    if (callsign == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_UNREGISTER_CALL;
    agwpe_set_callsign(agwpe_out->header.call_from, callsign);
    return ESP_OK;
}

esp_err_t agwpe_build_connect_req(uint8_t port,
                                   const char *from_call,
                                   const char *to_call,
                                   agwpe_frame_t *agwpe_out)
{
    if (from_call == NULL || to_call == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_CONNECT_REQ;
    agwpe_set_callsign(agwpe_out->header.call_from, from_call);
    agwpe_set_callsign(agwpe_out->header.call_to, to_call);
    return ESP_OK;
}

esp_err_t agwpe_build_connect_via_req(uint8_t port,
                                       const char *from_call,
                                       const char *to_call,
                                       const char *digipeaters[],
                                       uint8_t num_digis,
                                       agwpe_frame_t *agwpe_out)
{
    if (from_call == NULL || to_call == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (num_digis > 0 && digipeaters == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_CONNECT_VIA_REQ;
    agwpe_set_callsign(agwpe_out->header.call_from, from_call);
    agwpe_set_callsign(agwpe_out->header.call_to, to_call);

    /* Data format: <num_digis> <digi1>\0 <digi2>\0 ... */
    size_t pos = 0;
    agwpe_out->data[pos++] = num_digis;
    
    for (uint8_t i = 0; i < num_digis; i++) {
        if (digipeaters[i] == NULL) {
            continue;
        }
        size_t call_len = strlen(digipeaters[i]);
        if (pos + call_len + 1 > AGWPE_MAX_DATA_LEN) {
            return ESP_ERR_NO_MEM;
        }
        memcpy(&agwpe_out->data[pos], digipeaters[i], call_len + 1);
        pos += call_len + 1;
    }
    
    agwpe_out->header.data_len = (uint32_t)pos;
    return ESP_OK;
}

esp_err_t agwpe_build_disconnect_req(uint8_t port,
                                      const char *from_call,
                                      const char *to_call,
                                      agwpe_frame_t *agwpe_out)
{
    if (from_call == NULL || to_call == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_DISCONNECT_REQ;
    agwpe_set_callsign(agwpe_out->header.call_from, from_call);
    agwpe_set_callsign(agwpe_out->header.call_to, to_call);
    return ESP_OK;
}

esp_err_t agwpe_build_send_data(uint8_t port,
                                 const char *from_call,
                                 const char *to_call,
                                 const uint8_t *data,
                                 size_t data_len,
                                 uint8_t pid,
                                 agwpe_frame_t *agwpe_out)
{
    if (from_call == NULL || to_call == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (data_len > 0 && data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (data_len > AGWPE_MAX_DATA_LEN) {
        return ESP_ERR_NO_MEM;
    }

    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_SEND_DATA;
    agwpe_out->header.pid = pid;
    agwpe_set_callsign(agwpe_out->header.call_from, from_call);
    agwpe_set_callsign(agwpe_out->header.call_to, to_call);
    
    if (data_len > 0) {
        memcpy(agwpe_out->data, data, data_len);
        agwpe_out->header.data_len = (uint32_t)data_len;
    }
    
    return ESP_OK;
}

esp_err_t agwpe_build_send_unproto(uint8_t port,
                                    const char *from_call,
                                    const char *to_call,
                                    const uint8_t *data,
                                    size_t data_len,
                                    uint8_t pid,
                                    agwpe_frame_t *agwpe_out)
{
    if (from_call == NULL || to_call == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (data_len > 0 && data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (data_len > AGWPE_MAX_DATA_LEN) {
        return ESP_ERR_NO_MEM;
    }

    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_SEND_UNPROTO;
    agwpe_out->header.pid = pid;
    agwpe_set_callsign(agwpe_out->header.call_from, from_call);
    agwpe_set_callsign(agwpe_out->header.call_to, to_call);
    
    if (data_len > 0) {
        memcpy(agwpe_out->data, data, data_len);
        agwpe_out->header.data_len = (uint32_t)data_len;
    }
    
    return ESP_OK;
}

esp_err_t agwpe_build_send_unproto_via(uint8_t port,
                                        const char *from_call,
                                        const char *to_call,
                                        const char *digipeaters[],
                                        uint8_t num_digis,
                                        const uint8_t *data,
                                        size_t data_len,
                                        uint8_t pid,
                                        agwpe_frame_t *agwpe_out)
{
    if (from_call == NULL || to_call == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (num_digis > 0 && digipeaters == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (data_len > 0 && data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_SEND_UNPROTO_VIA;
    agwpe_out->header.pid = pid;
    agwpe_set_callsign(agwpe_out->header.call_from, from_call);
    agwpe_set_callsign(agwpe_out->header.call_to, to_call);

    /* Data format: <num_digis> <digi1>\0 <digi2>\0 ... <payload> */
    size_t pos = 0;
    agwpe_out->data[pos++] = num_digis;
    
    for (uint8_t i = 0; i < num_digis; i++) {
        if (digipeaters[i] == NULL) {
            continue;
        }
        size_t call_len = strlen(digipeaters[i]);
        if (pos + call_len + 1 > AGWPE_MAX_DATA_LEN) {
            return ESP_ERR_NO_MEM;
        }
        memcpy(&agwpe_out->data[pos], digipeaters[i], call_len + 1);
        pos += call_len + 1;
    }
    
    /* Add payload */
    if (data_len > 0) {
        if (pos + data_len > AGWPE_MAX_DATA_LEN) {
            return ESP_ERR_NO_MEM;
        }
        memcpy(&agwpe_out->data[pos], data, data_len);
        pos += data_len;
    }
    
    agwpe_out->header.data_len = (uint32_t)pos;
    return ESP_OK;
}

void agwpe_build_enable_monitor(agwpe_frame_t *agwpe_out)
{
    if (agwpe_out == NULL) {
        return;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.data_kind = AGWPE_KIND_ENABLE_MONITOR;
}

void agwpe_build_enable_raw(agwpe_frame_t *agwpe_out)
{
    if (agwpe_out == NULL) {
        return;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.data_kind = AGWPE_KIND_ENABLE_RAW;
}

esp_err_t agwpe_build_outstanding_req(uint8_t port,
                                       const char *from_call,
                                       const char *to_call,
                                       agwpe_frame_t *agwpe_out)
{
    if (from_call == NULL || to_call == NULL || agwpe_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_OUTSTANDING_REQ;
    agwpe_set_callsign(agwpe_out->header.call_from, from_call);
    agwpe_set_callsign(agwpe_out->header.call_to, to_call);
    return ESP_OK;
}

void agwpe_build_heard_req(uint8_t port, agwpe_frame_t *agwpe_out)
{
    if (agwpe_out == NULL) {
        return;
    }
    agwpe_frame_init(agwpe_out);
    agwpe_out->header.port = port;
    agwpe_out->header.data_kind = AGWPE_KIND_HEARD_REQ;
}

/*******************************************************************************
 * AGWPE Response Parsing Functions
 ******************************************************************************/

esp_err_t agwpe_parse_version_resp(const agwpe_frame_t *frame,
                                    agwpe_version_t *version)
{
    if (frame == NULL || version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (frame->header.data_kind != AGWPE_KIND_VERSION_RESP) {
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Version response data: 4 bytes - major (2 bytes LE), minor (2 bytes LE) */
    if (frame->header.data_len < 4) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    version->major = read_le16(frame->data);
    version->minor = read_le16(&frame->data[2]);
    
    return ESP_OK;
}

esp_err_t agwpe_parse_port_caps(const agwpe_frame_t *frame,
                                 agwpe_port_caps_t *caps)
{
    if (frame == NULL || caps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (frame->header.data_kind != AGWPE_KIND_PORT_CAP_RESP) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(caps, 0, sizeof(agwpe_port_caps_t));
    
    /* Port capabilities format varies, but typically includes baud rate info */
    if (frame->header.data_len >= 1) {
        caps->on_air_baud = frame->data[0];
    }
    if (frame->header.data_len >= 2) {
        caps->traffic_level = frame->data[1];
    }
    if (frame->header.data_len >= 3) {
        caps->tx_delay = frame->data[2];
    }
    if (frame->header.data_len >= 4) {
        caps->tx_tail = frame->data[3];
    }
    if (frame->header.data_len >= 5) {
        caps->persist = frame->data[4];
    }
    if (frame->header.data_len >= 6) {
        caps->slot_time = frame->data[5];
    }
    if (frame->header.data_len >= 7) {
        caps->max_frame = frame->data[6];
    }
    if (frame->header.data_len >= 8) {
        caps->active_conns = frame->data[7];
    }
    if (frame->header.data_len >= 12) {
        caps->bytes_recv = read_le32(&frame->data[8]);
    }
    
    return ESP_OK;
}

esp_err_t agwpe_parse_outstanding_resp(const agwpe_frame_t *frame,
                                        uint32_t *outstanding)
{
    if (frame == NULL || outstanding == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (frame->header.data_kind != AGWPE_KIND_OUTSTANDING_RESP) {
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Outstanding frames count is in the data_len field or first 4 bytes of data */
    if (frame->header.data_len >= 4) {
        *outstanding = read_le32(frame->data);
    } else {
        /* Some implementations use the user field */
        *outstanding = frame->header.user;
    }
    
    return ESP_OK;
}

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

const char *agwpe_kind_to_string(uint8_t kind)
{
    switch (kind) {
        case 'R': return "Version";
        case 'G': return "Port Info";
        case 'g': return "Port Caps";
        case 'X': return "Register Call";
        case 'x': return "Unregister Call";
        case 'H': return "Heard Stations";
        case 'C': return "Connect Request";
        case 'v': return "Connect Via";
        case 'c': return "Connect Response";
        case 'D': return "Data";
        case 'd': return "Disconnect";
        case 'M': return "Unproto";
        case 'V': return "Unproto Via";
        case 'U': return "Monitor Unproto";
        case 'S': return "Monitor Supervisory";
        case 'I': return "Monitor I-Frame";
        case 'T': return "Raw Frame";
        case 'K': return "Send Raw";
        case 'k': return "Enable Raw";
        case 'm': return "Enable Monitor";
        case 'Y': return "Outstanding Query";
        case 'y': return "Outstanding/Frames Waiting";
        default:  return "Unknown";
    }
}

void agwpe_set_callsign(char *dest, const char *call)
{
    if (dest == NULL) {
        return;
    }
    
    memset(dest, 0, AGWPE_CALLSIGN_LEN);
    
    if (call != NULL) {
        size_t len = strlen(call);
        if (len > AGWPE_CALLSIGN_LEN - 1) {
            len = AGWPE_CALLSIGN_LEN - 1;
        }
        memcpy(dest, call, len);
    }
}

void agwpe_get_callsign(const char *src, char *dest)
{
    if (dest == NULL) {
        return;
    }
    
    if (src == NULL) {
        dest[0] = '\0';
        return;
    }
    
    /* Copy up to AGWPE_CALLSIGN_LEN bytes, stopping at null */
    size_t i;
    for (i = 0; i < AGWPE_CALLSIGN_LEN && src[i] != '\0'; i++) {
        dest[i] = src[i];
    }
    dest[i] = '\0';
}

bool agwpe_is_request(uint8_t kind)
{
    switch (kind) {
        case AGWPE_KIND_VERSION_REQ:
        case AGWPE_KIND_PORT_INFO_REQ:
        case AGWPE_KIND_PORT_CAP_REQ:
        case AGWPE_KIND_REGISTER_CALL:
        case AGWPE_KIND_UNREGISTER_CALL:
        case AGWPE_KIND_HEARD_REQ:
        case AGWPE_KIND_CONNECT_REQ:
        case AGWPE_KIND_CONNECT_VIA_REQ:
        case AGWPE_KIND_DISCONNECT_REQ:
        case AGWPE_KIND_SEND_DATA:
        case AGWPE_KIND_SEND_UNPROTO:
        case AGWPE_KIND_SEND_UNPROTO_VIA:
        case AGWPE_KIND_SEND_RAW:
        case AGWPE_KIND_ENABLE_RAW:
        case AGWPE_KIND_ENABLE_MONITOR:
        case AGWPE_KIND_OUTSTANDING_REQ:
            return true;
        default:
            return false;
    }
}

bool agwpe_is_monitored(uint8_t kind)
{
    switch (kind) {
        case AGWPE_KIND_RECV_UNPROTO:
        case AGWPE_KIND_RECV_SUPERVISORY:
        case AGWPE_KIND_RECV_I_FRAME:
        case AGWPE_KIND_RECV_RAW:
            return true;
        default:
            return false;
    }
}

/*******************************************************************************
 * Pretty-printing
 ******************************************************************************/

void agwpe_print_frame(const char *label, const agwpe_frame_t *frame)
{
    if (frame == NULL) {
        return;
    }

    char from_str[AGWPE_CALLSIGN_LEN + 1];
    char to_str[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(frame->header.call_from, from_str);
    agwpe_get_callsign(frame->header.call_to, to_str);

    ESP_LOGI(TAG, "---- AGWPE Frame [%s] ----", label ? label : "");
    ESP_LOGI(TAG, "  Kind: '%c' (%s)", frame->header.data_kind,
             agwpe_kind_to_string(frame->header.data_kind));
    ESP_LOGI(TAG, "  Port: %u", frame->header.port);
    if (from_str[0] != '\0') {
        ESP_LOGI(TAG, "  From: %s", from_str);
    }
    if (to_str[0] != '\0') {
        ESP_LOGI(TAG, "  To:   %s", to_str);
    }
    if (frame->header.pid != 0) {
        ESP_LOGI(TAG, "  PID:  0x%02X", frame->header.pid);
    }
    ESP_LOGI(TAG, "  Len:  %u", (unsigned)frame->header.data_len);

    if (frame->header.data_len > 0) {
        uint32_t print_len = frame->header.data_len;
        if (print_len > AGWPE_MAX_DATA_LEN) {
            print_len = AGWPE_MAX_DATA_LEN;
        }
        for (uint32_t i = 0; i < print_len; i += 16) {
            char hex_buf[49] = {0};
            char ascii_buf[17] = {0};
            uint32_t line_len = (i + 16 <= print_len) ? 16 : (print_len - i);
            for (uint32_t j = 0; j < line_len; j++) {
                snprintf(hex_buf + j * 3, 4, "%02X ", frame->data[i + j]);
                ascii_buf[j] = isprint((unsigned char)frame->data[i + j])
                                   ? (char)frame->data[i + j] : '.';
            }
            ESP_LOGI(TAG, "  %04x: %-48s |%s|", (unsigned)i, hex_buf, ascii_buf);
        }
    }
}
