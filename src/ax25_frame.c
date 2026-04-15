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
 * @file ax25_frame.c
 * @brief Implementation of AX.25 frame parsing and building
 */

#include "ax25_frame.h"
#include "ax25_address.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "AX25_FRAME";

void ax25_frame_init(ax25_frame_t* frame) {
    if (!frame) {
        return;
    }
    memset(frame, 0, sizeof(ax25_frame_t));
    frame->type = AX25_FRAME_UNKNOWN;
    frame->pid = AX25_PID_NONE;
}

ax25_frame_type_t ax25_frame_identify_type(uint8_t control) {
    // I-frame: bit 0 = 0
    if ((control & 0x01) == 0) {
        return AX25_FRAME_I;
    }

    // S-frame: bits 0-1 = 01
    if ((control & 0x03) == 0x01) {
        return AX25_FRAME_S;
    }

    // U-frame: bits 0-1 = 11
    if ((control & 0x03) == 0x03) {
        // UI frame is a special case
        if ((control & 0xEF) == AX25_CTRL_UI) {
            return AX25_FRAME_UI;
        }
        return AX25_FRAME_U;
    }

    return AX25_FRAME_UNKNOWN;
}

uint8_t ax25_frame_extract_ns(uint8_t control) {
    return (control >> 1) & 0x07;
}

uint8_t ax25_frame_extract_nr(uint8_t control) {
    return (control >> 5) & 0x07;
}

uint8_t ax25_frame_build_i_control(uint8_t ns, uint8_t nr, bool pf) {
    uint8_t control = 0;
    control |= (ns & 0x07) << 1;
    control |= (nr & 0x07) << 5;
    if (pf) {
        control |= AX25_CTRL_PF_BIT;
    }
    return control;
}

uint8_t ax25_frame_build_rr_control(uint8_t nr, bool pf) {
    uint8_t control = AX25_CTRL_RR_MASK;
    control |= (nr & 0x07) << 5;
    if (pf) {
        control |= AX25_CTRL_PF_BIT;
    }
    return control;
}

uint8_t ax25_frame_build_rnr_control(uint8_t nr, bool pf) {
    uint8_t control = AX25_CTRL_RNR_MASK;
    control |= (nr & 0x07) << 5;
    if (pf) {
        control |= AX25_CTRL_PF_BIT;
    }
    return control;
}

uint8_t ax25_frame_build_rej_control(uint8_t nr, bool pf) {
    uint8_t control = AX25_CTRL_REJ_MASK;
    control |= (nr & 0x07) << 5;
    if (pf) {
        control |= AX25_CTRL_PF_BIT;
    }
    return control;
}

esp_err_t ax25_frame_parse(const uint8_t* data, size_t len, ax25_frame_t* frame) {
    if (!data || !frame) {
        return ESP_ERR_INVALID_ARG;
    }

    // Minimum: dst(7) + src(7) + control(1) = 15 bytes
    if (len < 15) {
        ESP_LOGE(TAG, "Frame too short: %zu bytes", len);
        return ESP_ERR_INVALID_ARG;
    }

    ax25_frame_init(frame);
    size_t pos = 0;

    // Parse destination address
    esp_err_t err = ax25_address_decode(data + pos, &frame->destination);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to decode destination address");
        return err;
    }
    pos += AX25_ADDRESS_LEN;

    // Parse source address
    err = ax25_address_decode(data + pos, &frame->source);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to decode source address");
        return err;
    }
    pos += AX25_ADDRESS_LEN;

    // Extract C/R bit: destination SSID bit 7 is the C bit in AX.25 v2.
    // ax25_address_decode stores bit 7 of the SSID byte in has_been_repeated,
    // so reinterpret it here and clear the field on both addresses.
    frame->is_command = frame->destination.has_been_repeated;
    frame->destination.has_been_repeated = false;
    frame->source.has_been_repeated = false;

    // Parse digipeaters (if any)
    frame->num_digipeaters = 0;
    while (!frame->source.is_last && pos + AX25_ADDRESS_LEN <= len) {
        if (frame->num_digipeaters >= AX25_MAX_DIGIPEATERS) {
            ESP_LOGW(TAG, "Too many digipeaters, truncating");
            break;
        }

        ax25_address_t* digi = &frame->digipeaters[frame->num_digipeaters];
        err = ax25_address_decode(data + pos, digi);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to decode digipeater address");
            return err;
        }
        pos += AX25_ADDRESS_LEN;
        frame->num_digipeaters++;

        if (digi->is_last) {
            break;
        }
    }

    // Parse control byte
    if (pos >= len) {
        ESP_LOGE(TAG, "No control byte");
        return ESP_ERR_INVALID_ARG;
    }
    frame->control = data[pos++];
    frame->type = ax25_frame_identify_type(frame->control);

    // Parse PID (if I or UI frame)
    if (frame->type == AX25_FRAME_I || frame->type == AX25_FRAME_UI) {
        if (pos >= len) {
            ESP_LOGE(TAG, "No PID byte for I/UI frame");
            return ESP_ERR_INVALID_ARG;
        }
        frame->pid = data[pos++];
    } else {
        frame->pid = 0;
    }

    // Parse payload (remaining bytes)
    frame->payload_len = 0;
    if (pos < len) {
        size_t payload_len = len - pos;
        if (payload_len > AX25_MAX_INFO_LEN) {
            ESP_LOGW(TAG, "Payload too large (%zu), truncating", payload_len);
            payload_len = AX25_MAX_INFO_LEN;
        }
        memcpy(frame->payload, data + pos, payload_len);
        frame->payload_len = payload_len;
    }

    return ESP_OK;
}

esp_err_t ax25_frame_build(const ax25_frame_t* frame, ax25_buffer_t* out) {
    if (!frame || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    if (frame->num_digipeaters > AX25_MAX_DIGIPEATERS) {
        ESP_LOGE(TAG, "Too many digipeaters: %u", frame->num_digipeaters);
        return ESP_ERR_INVALID_SIZE;
    }

    ax25_buffer_reset(out);

    uint8_t addr_buf[AX25_ADDRESS_LEN];
    esp_err_t err;

    // Encode destination address
    ax25_address_t dst = frame->destination;
    dst.is_last = false;  // Never last
    err = ax25_address_encode(&dst, addr_buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to encode destination address");
        return err;
    }
    if (frame->is_command) {
        addr_buf[6] |= 0x80;  // C bit: set in destination for command frames
    }
    ax25_buffer_append(out, addr_buf, AX25_ADDRESS_LEN);

    // Encode source address
    ax25_address_t src = frame->source;
    src.is_last = (frame->num_digipeaters == 0);
    err = ax25_address_encode(&src, addr_buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to encode source address");
        return err;
    }
    if (!frame->is_command) {
        addr_buf[6] |= 0x80;  // R bit: set in source for response frames
    }
    ax25_buffer_append(out, addr_buf, AX25_ADDRESS_LEN);

    // Encode digipeaters
    for (uint8_t i = 0; i < frame->num_digipeaters; i++) {
        ax25_address_t digi = frame->digipeaters[i];
        digi.is_last = (i == frame->num_digipeaters - 1);
        err = ax25_address_encode(&digi, addr_buf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to encode digipeater address");
            return err;
        }
        ax25_buffer_append(out, addr_buf, AX25_ADDRESS_LEN);
    }

    // Add control byte
    ax25_buffer_append_byte(out, frame->control);

    // Add PID (if I or UI frame)
    if (frame->type == AX25_FRAME_I || frame->type == AX25_FRAME_UI) {
        ax25_buffer_append_byte(out, frame->pid);
    }

    // Add payload
    if (frame->payload_len > 0) {
        ax25_buffer_append(out, frame->payload, frame->payload_len);
    }

    return ESP_OK;
}
