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
 * @file test_ax25_address.c
 * @brief Unit tests for AX.25 address encoding/decoding
 */

#include "unity.h"
#include "ax25_address.h"
#include <string.h>

TEST_CASE("AX25Address: Encode and decode N0CALL-0", "[ax25_address]")
{
    ax25_address_t addr;
    esp_err_t err = ax25_address_from_string("N0CALL-0", &addr);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("N0CALL", addr.callsign);
    TEST_ASSERT_EQUAL_UINT8(0, addr.ssid);
    
    // Encode
    uint8_t encoded[7];
    err = ax25_address_encode(&addr, encoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    // Verify bit-shifting (N = 0x4E -> 0x9C when left-shifted)
    TEST_ASSERT_EQUAL_UINT8(0x4E << 1, encoded[0]);  // 'N'
    TEST_ASSERT_EQUAL_UINT8(0x30 << 1, encoded[1]);  // '0'
    TEST_ASSERT_EQUAL_UINT8(0x43 << 1, encoded[2]);  // 'C'
    TEST_ASSERT_EQUAL_UINT8(0x41 << 1, encoded[3]);  // 'A'
    TEST_ASSERT_EQUAL_UINT8(0x4C << 1, encoded[4]);  // 'L'
    TEST_ASSERT_EQUAL_UINT8(0x4C << 1, encoded[5]);  // 'L'
    
    // Decode back
    ax25_address_t decoded;
    err = ax25_address_decode(encoded, &decoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("N0CALL", decoded.callsign);
    TEST_ASSERT_EQUAL_UINT8(0, decoded.ssid);
}

TEST_CASE("AX25Address: Encode and decode GB7BBS-1", "[ax25_address]")
{
    ax25_address_t addr;
    esp_err_t err = ax25_address_from_string("GB7BBS-1", &addr);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("GB7BBS", addr.callsign);
    TEST_ASSERT_EQUAL_UINT8(1, addr.ssid);
    
    // Encode
    uint8_t encoded[7];
    err = ax25_address_encode(&addr, encoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    // Decode back
    ax25_address_t decoded;
    err = ax25_address_decode(encoded, &decoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("GB7BBS", decoded.callsign);
    TEST_ASSERT_EQUAL_UINT8(1, decoded.ssid);
}

TEST_CASE("AX25Address: Encode and decode VK2XYZ-15", "[ax25_address]")
{
    ax25_address_t addr;
    esp_err_t err = ax25_address_from_string("VK2XYZ-15", &addr);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("VK2XYZ", addr.callsign);
    TEST_ASSERT_EQUAL_UINT8(15, addr.ssid);
    
    // Encode
    uint8_t encoded[7];
    addr.is_last = true;  // Set extension bit
    err = ax25_address_encode(&addr, encoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    // Verify extension bit is set
    TEST_ASSERT_TRUE(encoded[6] & 0x01);
    
    // Decode back
    ax25_address_t decoded;
    err = ax25_address_decode(encoded, &decoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("VK2XYZ", decoded.callsign);
    TEST_ASSERT_EQUAL_UINT8(15, decoded.ssid);
    TEST_ASSERT_TRUE(decoded.is_last);
}

TEST_CASE("AX25Address: toString and fromString round-trip", "[ax25_address]")
{
    const char* input_calls[] = {"N0CALL-0", "GB7BBS-1", "VK2XYZ-15", "W1AW-5"};
    const char* expected_calls[] = {"N0CALL", "GB7BBS-1", "VK2XYZ-15", "W1AW-5"};
    
    for (int i = 0; i < 4; i++) {
        ax25_address_t addr;
        esp_err_t err = ax25_address_from_string(input_calls[i], &addr);
        TEST_ASSERT_EQUAL(ESP_OK, err);
        
        char buf[10];
        err = ax25_address_to_string(&addr, buf, sizeof(buf));
        TEST_ASSERT_EQUAL(ESP_OK, err);
        TEST_ASSERT_EQUAL_STRING(expected_calls[i], buf);
    }
}

TEST_CASE("AX25Address: CALL and CALL-0 parse equivalently", "[ax25_address]")
{
    ax25_address_t addr_a, addr_b;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_address_from_string("N0CALL", &addr_a));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_address_from_string("N0CALL-0", &addr_b));

    TEST_ASSERT_TRUE(ax25_address_equals(&addr_a, &addr_b));

    char buf_a[10], buf_b[10];
    TEST_ASSERT_EQUAL(ESP_OK, ax25_address_to_string(&addr_a, buf_a, sizeof(buf_a)));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_address_to_string(&addr_b, buf_b, sizeof(buf_b)));
    TEST_ASSERT_EQUAL_STRING("N0CALL", buf_a);
    TEST_ASSERT_EQUAL_STRING("N0CALL", buf_b);
}

TEST_CASE("AX25Address: H bit handling", "[ax25_address]")
{
    ax25_address_t addr;
    esp_err_t err = ax25_address_from_string("RELAY-0", &addr);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    addr.has_been_repeated = true;
    
    // Encode
    uint8_t encoded[7];
    err = ax25_address_encode(&addr, encoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    // Verify H bit is set (bit 7 of byte 6)
    TEST_ASSERT_TRUE(encoded[6] & 0x80);
    
    // Decode back
    ax25_address_t decoded;
    err = ax25_address_decode(encoded, &decoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(decoded.has_been_repeated);
}

TEST_CASE("AX25Address: Short callsign with space padding", "[ax25_address]")
{
    ax25_address_t addr;
    esp_err_t err = ax25_address_from_string("KA2-5", &addr);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("KA2", addr.callsign);
    TEST_ASSERT_EQUAL_UINT8(5, addr.ssid);
    
    // Encode
    uint8_t encoded[7];
    err = ax25_address_encode(&addr, encoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    // Verify space padding (0x20 << 1 = 0x40)
    TEST_ASSERT_EQUAL_UINT8(0x40, encoded[3]);  // Space
    TEST_ASSERT_EQUAL_UINT8(0x40, encoded[4]);  // Space
    TEST_ASSERT_EQUAL_UINT8(0x40, encoded[5]);  // Space
    
    // Decode back
    ax25_address_t decoded;
    err = ax25_address_decode(encoded, &decoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("KA2", decoded.callsign);
}

TEST_CASE("AX25Address: equals comparison", "[ax25_address]")
{
    ax25_address_t addr1, addr2;
    ax25_address_from_string("N0CALL-1", &addr1);
    ax25_address_from_string("N0CALL-1", &addr2);
    
    TEST_ASSERT_TRUE(ax25_address_equals(&addr1, &addr2));
    
    ax25_address_t addr3;
    ax25_address_from_string("N0CALL-2", &addr3);
    TEST_ASSERT_FALSE(ax25_address_equals(&addr1, &addr3));
    
    ax25_address_t addr4;
    ax25_address_from_string("W1AW-1", &addr4);
    TEST_ASSERT_FALSE(ax25_address_equals(&addr1, &addr4));
}

// ---------------------------------------------------------------------------
// Additional tests — error paths and edge cases
// ---------------------------------------------------------------------------

TEST_CASE("AX25Address: from_string rejects NULL inputs", "[ax25_address]")
{
    ax25_address_t addr;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_address_from_string(NULL, &addr));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_address_from_string("N0CALL-0", NULL));
}

TEST_CASE("AX25Address: from_string rejects oversized callsign", "[ax25_address]")
{
    ax25_address_t addr;
    /* 7-character callsign exceeds the 6-character maximum. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_address_from_string("ABCDEFG-0", &addr));
}

TEST_CASE("AX25Address: from_string rejects SSID > 15", "[ax25_address]")
{
    ax25_address_t addr;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_address_from_string("N0CALL-16", &addr));
}

TEST_CASE("AX25Address: to_string rejects undersized buffer", "[ax25_address]")
{
    ax25_address_t addr;
    ax25_address_from_string("N0CALL-0", &addr);
    char buf[5];  /* too small for "N0CALL-0\0" (9 bytes needed) */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE,
                      ax25_address_to_string(&addr, buf, sizeof(buf)));
}

TEST_CASE("AX25Address: init zeroes the structure", "[ax25_address]")
{
    ax25_address_t addr;
    /* Dirty the memory first. */
    memset(&addr, 0xFF, sizeof(addr));

    ax25_address_init(&addr);

    TEST_ASSERT_EQUAL_UINT8(0, addr.ssid);
    TEST_ASSERT_FALSE(addr.has_been_repeated);
    TEST_ASSERT_FALSE(addr.is_last);
    TEST_ASSERT_EQUAL_UINT8('\0', addr.callsign[0]);
}

TEST_CASE("AX25Address: copy produces independent deep copy", "[ax25_address]")
{
    ax25_address_t src, dst;
    ax25_address_from_string("N0CALL-3", &src);
    src.has_been_repeated = true;
    src.is_last = true;

    ax25_address_copy(&dst, &src);

    TEST_ASSERT_EQUAL_STRING(src.callsign, dst.callsign);
    TEST_ASSERT_EQUAL_UINT8(src.ssid, dst.ssid);
    TEST_ASSERT_EQUAL(src.has_been_repeated, dst.has_been_repeated);
    TEST_ASSERT_EQUAL(src.is_last, dst.is_last);

    /* Modifying src must not affect dst. */
    src.ssid = 9;
    TEST_ASSERT_EQUAL_UINT8(3, dst.ssid);
}

// ---------------------------------------------------------------------------
// Wire-format encode / decode — direct API
// ---------------------------------------------------------------------------

TEST_CASE("AX25Address: encode and decode wire format round-trip", "[ax25_address]")
{
    ax25_address_t original, decoded;
    ax25_address_from_string("VK2XYZ-7", &original);
    original.has_been_repeated = true;

    uint8_t wire[AX25_ADDRESS_LEN];
    esp_err_t err = ax25_address_encode(&original, wire);
    TEST_ASSERT_EQUAL(ESP_OK, err);

    err = ax25_address_decode(wire, &decoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);

    TEST_ASSERT_TRUE(ax25_address_equals(&original, &decoded));
    TEST_ASSERT_EQUAL_UINT8(original.ssid,           decoded.ssid);
    TEST_ASSERT_EQUAL(original.has_been_repeated,    decoded.has_been_repeated);
}

TEST_CASE("AX25Address: encode and decode reject NULL inputs", "[ax25_address]")
{
    ax25_address_t addr;
    ax25_address_from_string("N0CALL-0", &addr);
    uint8_t wire[AX25_ADDRESS_LEN];

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_address_encode(NULL, wire));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_address_encode(&addr, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_address_decode(NULL, &addr));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_address_decode(wire, NULL));
}
