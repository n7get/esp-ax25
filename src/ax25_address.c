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
 * @file ax25_address.c
 * @brief Implementation of AX.25 address encoding/decoding
 */

#include "ax25_address.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "esp_log.h"

static const char* TAG = "AX25_ADDR";

void ax25_address_init(ax25_address_t* addr) {
    if (!addr) {
        return;
    }
    memset(addr, 0, sizeof(ax25_address_t));
}

esp_err_t ax25_address_encode(const ax25_address_t* addr, uint8_t* out7_bytes) {
    if (!addr || !out7_bytes) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t callsign_len = strlen(addr->callsign);

    // Encode callsign (6 bytes, left-shifted by 1 bit, space-padded)
    for (int i = 0; i < 6; i++) {
        char c = (i < (int)callsign_len) ? toupper((unsigned char)addr->callsign[i]) : ' ';
        out7_bytes[i] = ((uint8_t)c) << 1;
    }

    // Encode SSID byte
    // Bit 7: H bit (has been repeated)
    // Bits 6-5: reserved (1,1)
    // Bits 4-1: SSID
    // Bit 0: extension bit (1 if last address)
    uint8_t ssid_byte = 0b01100000;  // Reserved bits set to 1
    ssid_byte |= (addr->ssid & 0x0F) << 1;
    if (addr->has_been_repeated) {
        ssid_byte |= 0b10000000;  // H bit
    }
    if (addr->is_last) {
        ssid_byte |= 0b00000001;  // Extension bit
    }
    out7_bytes[6] = ssid_byte;

    return ESP_OK;
}

esp_err_t ax25_address_decode(const uint8_t* in7_bytes, ax25_address_t* addr) {
    if (!in7_bytes || !addr) {
        return ESP_ERR_INVALID_ARG;
    }

    ax25_address_init(addr);

    // Decode callsign (shift right by 1 bit, remove trailing spaces)
    int callsign_len = 0;
    for (int i = 0; i < 6; i++) {
        char c = (char)(in7_bytes[i] >> 1);
        if (c != ' ') {
            addr->callsign[callsign_len++] = c;
        }
    }
    addr->callsign[callsign_len] = '\0';

    // Decode SSID byte
    uint8_t ssid_byte = in7_bytes[6];
    addr->ssid = (ssid_byte >> 1) & 0x0F;
    addr->has_been_repeated = (ssid_byte & 0b10000000) != 0;
    addr->is_last = (ssid_byte & 0b00000001) != 0;

    return ESP_OK;
}

esp_err_t ax25_address_from_string(const char* callsign_ssid, ax25_address_t* addr) {
    if (!callsign_ssid || !addr) {
        return ESP_ERR_INVALID_ARG;
    }

    ax25_address_init(addr);

    // Find dash separator
    const char* dash_pos = strchr(callsign_ssid, '-');

    if (dash_pos) {
        // Callsign-SSID format
        size_t callsign_len = dash_pos - callsign_ssid;
        if (callsign_len > AX25_MAX_CALLSIGN_LEN) {
            ESP_LOGE(TAG, "Callsign too long: %s", callsign_ssid);
            return ESP_ERR_INVALID_ARG;
        }

        // Copy callsign
        for (size_t i = 0; i < callsign_len; i++) {
            addr->callsign[i] = toupper((unsigned char)callsign_ssid[i]);
        }
        addr->callsign[callsign_len] = '\0';

        // Parse SSID
        int ssid = atoi(dash_pos + 1);
        if (ssid < 0 || ssid > 15) {
            ESP_LOGE(TAG, "Invalid SSID: %d", ssid);
            return ESP_ERR_INVALID_ARG;
        }
        addr->ssid = (uint8_t)ssid;
    } else {
        // Just callsign (SSID defaults to 0)
        size_t callsign_len = strlen(callsign_ssid);
        if (callsign_len > AX25_MAX_CALLSIGN_LEN) {
            ESP_LOGE(TAG, "Callsign too long: %s", callsign_ssid);
            return ESP_ERR_INVALID_ARG;
        }

        // Copy and uppercase callsign
        for (size_t i = 0; i < callsign_len; i++) {
            addr->callsign[i] = toupper((unsigned char)callsign_ssid[i]);
        }
        addr->callsign[callsign_len] = '\0';
        addr->ssid = 0;
    }

    return ESP_OK;
}

esp_err_t ax25_address_to_string(const ax25_address_t* addr, char* buf, size_t buf_len) {
    if (!addr || !buf) {
        return ESP_ERR_INVALID_ARG;
    }

    if (addr->ssid == 0) {
        size_t required_len = strlen(addr->callsign) + 1;
        if (buf_len < required_len) {
            return ESP_ERR_INVALID_SIZE;
        }
        snprintf(buf, buf_len, "%s", addr->callsign);
    } else {
        size_t required_len = strlen(addr->callsign) + 1 /* '-' */ + 2 /* max ssid "15" */ + 1 /* NUL */;
        if (buf_len < required_len) {
            return ESP_ERR_INVALID_SIZE;
        }
        snprintf(buf, buf_len, "%s-%d", addr->callsign, addr->ssid);
    }

    return ESP_OK;
}

bool ax25_address_equals(const ax25_address_t* a, const ax25_address_t* b) {
    if (!a || !b) {
        return false;
    }
    return (strcmp(a->callsign, b->callsign) == 0) && (a->ssid == b->ssid);
}

void ax25_address_copy(ax25_address_t* dst, const ax25_address_t* src) {
    if (!dst || !src) {
        return;
    }
    memcpy(dst, src, sizeof(ax25_address_t));
}
