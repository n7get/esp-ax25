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
 * @file ax25_print.c
 * @brief Implementation of AX.25 frame pretty-printing
 */

#include "ax25_print.h"
#include "ax25_frame.h"
#include "ax25_address.h"
#include "esp_log.h"
#include <stdio.h>
#include <ctype.h>
#include <string.h>

static const char* TAG = "AX25_PRINT";

const char* ax25_frame_type_str(ax25_frame_type_t type) {
    switch (type) {
        case AX25_FRAME_UI: return "UI";
        case AX25_FRAME_I:  return "I";
        case AX25_FRAME_S:  return "S";
        case AX25_FRAME_U:  return "U";
        default:            return "Unknown";
    }
}

const char* ax25_conn_state_str(ax25_conn_state_t state) {
    switch (state) {
        case AX25_CONN_STATE_DISCONNECTED:        return "Disconnected";
        case AX25_CONN_STATE_AWAITING_CONNECTION: return "Awaiting Connection";
        case AX25_CONN_STATE_CONNECTED:           return "Connected";
        case AX25_CONN_STATE_AWAITING_RELEASE:    return "Awaiting Release";
        case AX25_CONN_STATE_TIMER_RECOVERY:      return "Timer Recovery";
        default:                                  return "Unknown";
    }
}

static const char* u_frame_name(uint8_t ctrl) {
    switch (ctrl & ~AX25_CTRL_PF_BIT) {
        case AX25_CTRL_UI:   return "UI";
        case AX25_CTRL_SABM: return "SABM";
        case AX25_CTRL_DISC: return "DISC";
        case AX25_CTRL_DM:   return "DM";
        case AX25_CTRL_UA:   return "UA";
        case AX25_CTRL_FRMR: return "FRMR";
        default:             return "U(?)";
    }
}

static const char* s_frame_name(uint8_t ctrl) {
    switch ((ctrl >> 2) & 0x03) {
        case 0: return "RR";
        case 1: return "RNR";
        case 2: return "REJ";
        default: return "S(?)";
    }
}

void ax25_print_haxdump(const char* label, const uint8_t* data, size_t len) {
    if (!data || len == 0) {
        return;
    }

    ESP_LOGI(TAG, "---- %s (len=%zu) ----", label ? label : "Buffer", len);
    for (size_t i = 0; i < len; i += 16) {
        char hex[49] = {};
        char ascii[17] = {};
        size_t line_len = (i + 16 <= len) ? 16 : (len - i);
        for (size_t j = 0; j < line_len; j++) {
            snprintf(hex + j * 3, sizeof(hex) - j * 3, "%02X ", data[i + j]);
            ascii[j] = isprint(data[i + j]) ? data[i + j] : '.';
        }
        ESP_LOGI(TAG, "%04zx: %-48s |%s|", i, hex, ascii);
    }
}

void ax25_print_frame(const char* label, const ax25_frame_t* frame) {
    if (!frame) {
        return;
    }

    char addr_buf[10];

    ESP_LOGI(TAG, "---- AX.25 Frame [%s] ----", label ? label : "");

    ax25_address_to_string(&frame->source, addr_buf, sizeof(addr_buf));
    ESP_LOGI(TAG, "  Src:  %s", addr_buf);

    ax25_address_to_string(&frame->destination, addr_buf, sizeof(addr_buf));
    ESP_LOGI(TAG, "  Dst:  %s", addr_buf);

    if (frame->num_digipeaters > 0) {
        char via[128] = {};
        int pos = 0;
        for (uint8_t i = 0; i < frame->num_digipeaters; i++) {
            ax25_address_to_string(&frame->digipeaters[i], addr_buf, sizeof(addr_buf));
            if (i > 0) {
                pos += snprintf(via + pos, sizeof(via) - pos, " > ");
            }
            pos += snprintf(via + pos, sizeof(via) - pos, "%s%s",
                           addr_buf,
                           frame->digipeaters[i].has_been_repeated ? "*" : "");
        }
        ESP_LOGI(TAG, "  Via:  %s", via);
    }

    uint8_t ctrl = frame->control;
    bool pf = (ctrl & AX25_CTRL_PF_BIT) != 0;

    if ((ctrl & 0x01) == 0) {
        // I-frame
        uint8_t ns = (ctrl >> 1) & 0x07;
        uint8_t nr = (ctrl >> 5) & 0x07;
        ESP_LOGI(TAG, "  Type: I  [N(S)=%u N(R)=%u P/F=%u]", ns, nr, (unsigned)pf);
        ESP_LOGI(TAG, "  PID:  0x%02X", frame->pid);
    } else if ((ctrl & 0x03) == 0x01) {
        // S-frame
        uint8_t nr = (ctrl >> 5) & 0x07;
        ESP_LOGI(TAG, "  Type: %s  [N(R)=%u P/F=%u]", s_frame_name(ctrl), nr, (unsigned)pf);
    } else {
        // U-frame
        ESP_LOGI(TAG, "  Type: %s  [P/F=%u]", u_frame_name(ctrl), (unsigned)pf);
        if ((ctrl & ~AX25_CTRL_PF_BIT) == AX25_CTRL_UI) {
            ESP_LOGI(TAG, "  PID:  0x%02X", frame->pid);
        }
    }

    ESP_LOGI(TAG, "  Ctrl: 0x%02X", ctrl);

    if (frame->payload_len > 0) {
        ax25_print_haxdump("Info", frame->payload, frame->payload_len);
    } else {
        ESP_LOGI(TAG, "  Info: (none)");
    }
}

void ax25_print_buffer(const char* label, const uint8_t* data, size_t len) {
    if (!data || len == 0) {
        return;
    }

    ax25_frame_t frame;
    esp_err_t err = ax25_frame_parse(data, len, &frame);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "---- AX.25 Frame [%s] (parse error 0x%x) ----", label ? label : "", err);
        return;
    }
    ax25_print_frame(label, &frame);
}
