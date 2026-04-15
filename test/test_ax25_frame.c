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
 * @file test_ax25_frame.c
 * @brief Unit tests for AX.25 frame parsing and building
 */

#include "unity.h"
#include "ax25_frame.h"
#include "ax25_address.h"
#include "ax25_buffer.h"
#include <string.h>

static ax25_buffer_pool_t test_pool;
static bool pool_initialized = false;

static void ensure_pool(void) {
    if (!pool_initialized) {
        ax25_buffer_pool_init(&test_pool, 4);
        pool_initialized = true;
    }
}

TEST_CASE("AX25Frame: Parse UI frame", "[ax25_frame]")
{
    ensure_pool();
    
    // Build a simple UI frame manually
    // DST: APRS-0, SRC: N0CALL-1, Control: UI (0x03), PID: 0xF0, Info: "TEST"
    uint8_t frame_data[100];
    size_t pos = 0;
    
    // Destination: APRS-0
    ax25_address_t dst;
    ax25_address_from_string("APRS-0", &dst);
    ax25_address_encode(&dst, &frame_data[pos]);
    pos += 7;
    
    // Source: N0CALL-1 (last address)
    ax25_address_t src;
    ax25_address_from_string("N0CALL-1", &src);
    src.is_last = true;
    ax25_address_encode(&src, &frame_data[pos]);
    pos += 7;
    
    // Control: UI
    frame_data[pos++] = AX25_CTRL_UI;
    
    // PID
    frame_data[pos++] = 0xF0;
    
    // Info
    const char* info = "TEST";
    memcpy(&frame_data[pos], info, strlen(info));
    pos += strlen(info);
    
    // Parse
    ax25_frame_t frame;
    esp_err_t err = ax25_frame_parse(frame_data, pos, &frame);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(AX25_FRAME_UI, frame.type);
    TEST_ASSERT_EQUAL_STRING("APRS", frame.destination.callsign);
    TEST_ASSERT_EQUAL_STRING("N0CALL", frame.source.callsign);
    TEST_ASSERT_EQUAL_UINT8(1, frame.source.ssid);
    TEST_ASSERT_EQUAL_UINT8(0xF0, frame.pid);
    TEST_ASSERT_EQUAL_UINT8(4, frame.payload_len);
    TEST_ASSERT_EQUAL_MEMORY("TEST", frame.payload, 4);
}

TEST_CASE("AX25Frame: Build UI frame", "[ax25_frame]")
{
    ensure_pool();
    
    ax25_frame_t frame;
    ax25_frame_init(&frame);
    ax25_address_from_string("APRS-0", &frame.destination);
    ax25_address_from_string("N0CALL-1", &frame.source);
    frame.type = AX25_FRAME_UI;
    frame.control = AX25_CTRL_UI;
    frame.pid = 0xF0;
    const char* info = "Hello";
    memcpy(frame.payload, info, strlen(info));
    frame.payload_len = strlen(info);
    
    ax25_buffer_t* buffer = ax25_buffer_alloc(&test_pool, 100);
    TEST_ASSERT_NOT_NULL(buffer);
    
    esp_err_t err = ax25_frame_build(&frame, buffer);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    // Should be: 7 (dst) + 7 (src) + 1 (control) + 1 (pid) + 5 (info) = 21 bytes
    TEST_ASSERT_EQUAL_UINT32(21, buffer->len);
    
    // Verify control byte
    TEST_ASSERT_EQUAL_UINT8(AX25_CTRL_UI, buffer->data[14]);
    
    // Verify PID
    TEST_ASSERT_EQUAL_UINT8(0xF0, buffer->data[15]);
    
    ax25_buffer_free(&test_pool, buffer);
}

TEST_CASE("AX25Frame: Round-trip UI frame", "[ax25_frame]")
{
    ensure_pool();
    
    // Build frame
    ax25_frame_t original;
    ax25_frame_init(&original);
    ax25_address_from_string("APRS-0", &original.destination);
    ax25_address_from_string("N0CALL-5", &original.source);
    original.type = AX25_FRAME_UI;
    original.control = AX25_CTRL_UI;
    original.pid = 0xF0;
    const char* info = "Test Message";
    memcpy(original.payload, info, strlen(info));
    original.payload_len = strlen(info);
    
    // Encode
    ax25_buffer_t* buffer = ax25_buffer_alloc(&test_pool, 100);
    TEST_ASSERT_NOT_NULL(buffer);
    
    esp_err_t err = ax25_frame_build(&original, buffer);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    // Decode
    ax25_frame_t decoded;
    err = ax25_frame_parse(buffer->data, buffer->len, &decoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    // Verify
    TEST_ASSERT_EQUAL(original.type, decoded.type);
    TEST_ASSERT_EQUAL_STRING(original.destination.callsign, decoded.destination.callsign);
    TEST_ASSERT_EQUAL_UINT8(original.destination.ssid, decoded.destination.ssid);
    TEST_ASSERT_EQUAL_STRING(original.source.callsign, decoded.source.callsign);
    TEST_ASSERT_EQUAL_UINT8(original.source.ssid, decoded.source.ssid);
    TEST_ASSERT_EQUAL_UINT8(original.pid, decoded.pid);
    TEST_ASSERT_EQUAL_UINT32(original.payload_len, decoded.payload_len);
    TEST_ASSERT_EQUAL_MEMORY(original.payload, decoded.payload, original.payload_len);
    
    ax25_buffer_free(&test_pool, buffer);
}

TEST_CASE("AX25Frame: Parse I-frame", "[ax25_frame]")
{
    ensure_pool();
    
    // Build an I-frame: N(S)=3, N(R)=5
    uint8_t frame_data[100];
    size_t pos = 0;
    
    // Destination
    ax25_address_t dst;
    ax25_address_from_string("DEST-0", &dst);
    ax25_address_encode(&dst, &frame_data[pos]);
    pos += 7;
    
    // Source (last)
    ax25_address_t src;
    ax25_address_from_string("SRC-0", &src);
    src.is_last = true;
    ax25_address_encode(&src, &frame_data[pos]);
    pos += 7;
    
    // Control: I-frame with N(S)=3, N(R)=5
    uint8_t control = ax25_frame_build_i_control(3, 5, false);
    frame_data[pos++] = control;
    
    // PID
    frame_data[pos++] = 0xF0;
    
    // Info
    const char* info = "DATA";
    memcpy(&frame_data[pos], info, strlen(info));
    pos += strlen(info);
    
    // Parse
    ax25_frame_t frame;
    esp_err_t err = ax25_frame_parse(frame_data, pos, &frame);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(AX25_FRAME_I, frame.type);
    TEST_ASSERT_EQUAL_UINT8(3, ax25_frame_extract_ns(frame.control));
    TEST_ASSERT_EQUAL_UINT8(5, ax25_frame_extract_nr(frame.control));
}

TEST_CASE("AX25Frame: Frame with digipeaters", "[ax25_frame]")
{
    ensure_pool();
    
    // Build frame with 2 digipeaters
    ax25_frame_t frame;
    ax25_frame_init(&frame);
    ax25_address_from_string("DEST-0", &frame.destination);
    ax25_address_from_string("SRC-0", &frame.source);
    
    ax25_address_from_string("RELAY1-0", &frame.digipeaters[0]);
    ax25_address_from_string("RELAY2-0", &frame.digipeaters[1]);
    frame.num_digipeaters = 2;
    
    frame.type = AX25_FRAME_UI;
    frame.control = AX25_CTRL_UI;
    frame.pid = 0xF0;
    const char* info = "VIA";
    memcpy(frame.payload, info, strlen(info));
    frame.payload_len = strlen(info);
    
    // Encode
    ax25_buffer_t* buffer = ax25_buffer_alloc(&test_pool, 100);
    TEST_ASSERT_NOT_NULL(buffer);
    
    esp_err_t err = ax25_frame_build(&frame, buffer);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    // Should be: 7 (dst) + 7 (src) + 7 (digi1) + 7 (digi2) + 1 (ctrl) + 1 (pid) + 3 (info)
    TEST_ASSERT_EQUAL_UINT32(33, buffer->len);
    
    // Decode
    ax25_frame_t decoded;
    err = ax25_frame_parse(buffer->data, buffer->len, &decoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8(2, decoded.num_digipeaters);
    TEST_ASSERT_EQUAL_STRING("RELAY1", decoded.digipeaters[0].callsign);
    TEST_ASSERT_EQUAL_STRING("RELAY2", decoded.digipeaters[1].callsign);
    
    ax25_buffer_free(&test_pool, buffer);
}

TEST_CASE("AX25Frame: Max digipeaters round-trip preserves H bits", "[ax25_frame]")
{
    ensure_pool();

    ax25_frame_t frame;
    ax25_frame_init(&frame);
    ax25_address_from_string("DEST-0", &frame.destination);
    ax25_address_from_string("SRC-0", &frame.source);
    for (uint8_t index = 0; index < AX25_MAX_DIGIPEATERS; index++) {
        char call[10];
        snprintf(call, sizeof(call), "R%u-%u", (unsigned)(index + 1), (unsigned)(index + 1));
        ax25_address_from_string(call, &frame.digipeaters[index]);
        frame.digipeaters[index].has_been_repeated = (index % 2) == 0;
    }
    frame.num_digipeaters = AX25_MAX_DIGIPEATERS;
    frame.type = AX25_FRAME_UI;
    frame.control = AX25_CTRL_UI;
    frame.pid = AX25_PID_NONE;

    ax25_buffer_t* buffer = ax25_buffer_alloc(&test_pool, 128);
    TEST_ASSERT_NOT_NULL(buffer);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_build(&frame, buffer));

    ax25_frame_t decoded;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_parse(buffer->data, buffer->len, &decoded));
    TEST_ASSERT_EQUAL_UINT8(AX25_MAX_DIGIPEATERS, decoded.num_digipeaters);
    for (uint8_t index = 0; index < AX25_MAX_DIGIPEATERS; index++) {
        TEST_ASSERT_EQUAL(frame.digipeaters[index].has_been_repeated,
                          decoded.digipeaters[index].has_been_repeated);
    }

    ax25_buffer_free(&test_pool, buffer);
}

TEST_CASE("AX25Frame: Build rejects more than max digipeaters", "[ax25_frame]")
{
    ensure_pool();

    ax25_frame_t frame;
    ax25_frame_init(&frame);
    ax25_address_from_string("DEST-0", &frame.destination);
    ax25_address_from_string("SRC-0", &frame.source);
    frame.type = AX25_FRAME_UI;
    frame.control = AX25_CTRL_UI;
    frame.pid = AX25_PID_NONE;
    frame.num_digipeaters = AX25_MAX_DIGIPEATERS + 1;

    ax25_buffer_t* buffer = ax25_buffer_alloc(&test_pool, 128);
    TEST_ASSERT_NOT_NULL(buffer);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, ax25_frame_build(&frame, buffer));
    ax25_buffer_free(&test_pool, buffer);
}

TEST_CASE("AX25Frame: Identify frame types", "[ax25_frame]")
{
    // I-frame (bit 0 = 0)
    TEST_ASSERT_EQUAL(AX25_FRAME_I, ax25_frame_identify_type(0b00000000));
    TEST_ASSERT_EQUAL(AX25_FRAME_I, ax25_frame_identify_type(0b11111110));
    
    // S-frame (bits 0-1 = 01)
    TEST_ASSERT_EQUAL(AX25_FRAME_S, ax25_frame_identify_type(0b00000001));
    TEST_ASSERT_EQUAL(AX25_FRAME_S, ax25_frame_identify_type(AX25_CTRL_RR_MASK));
    
    // UI frame
    TEST_ASSERT_EQUAL(AX25_FRAME_UI, ax25_frame_identify_type(AX25_CTRL_UI));
    
    // U-frame (bits 0-1 = 11, but not UI)
    TEST_ASSERT_EQUAL(AX25_FRAME_U, ax25_frame_identify_type(AX25_CTRL_SABM));
    TEST_ASSERT_EQUAL(AX25_FRAME_U, ax25_frame_identify_type(AX25_CTRL_UA));
}

TEST_CASE("AX25Frame: Build control bytes", "[ax25_frame]")
{
    // I-frame control
    uint8_t i_ctrl = ax25_frame_build_i_control(3, 5, false);
    TEST_ASSERT_EQUAL_UINT8(3, ax25_frame_extract_ns(i_ctrl));
    TEST_ASSERT_EQUAL_UINT8(5, ax25_frame_extract_nr(i_ctrl));
    TEST_ASSERT_FALSE(i_ctrl & AX25_CTRL_PF_BIT);
    
    // I-frame with P/F
    i_ctrl = ax25_frame_build_i_control(7, 2, true);
    TEST_ASSERT_EQUAL_UINT8(7, ax25_frame_extract_ns(i_ctrl));
    TEST_ASSERT_EQUAL_UINT8(2, ax25_frame_extract_nr(i_ctrl));
    TEST_ASSERT_TRUE(i_ctrl & AX25_CTRL_PF_BIT);
    
    // RR control
    uint8_t rr_ctrl = ax25_frame_build_rr_control(4, false);
    TEST_ASSERT_EQUAL_UINT8(4, ax25_frame_extract_nr(rr_ctrl));
    TEST_ASSERT_EQUAL(AX25_FRAME_S, ax25_frame_identify_type(rr_ctrl));
}

// ---------------------------------------------------------------------------
// Additional tests — S-frames, error paths, max payload
// ---------------------------------------------------------------------------

TEST_CASE("AX25Frame: RNR control byte is an S-frame with correct N(R)", "[ax25_frame]")
{
    uint8_t ctrl = ax25_frame_build_rnr_control(3, false);
    TEST_ASSERT_EQUAL(AX25_FRAME_S, ax25_frame_identify_type(ctrl));
    TEST_ASSERT_EQUAL_UINT8(3, ax25_frame_extract_nr(ctrl));
    TEST_ASSERT_EQUAL_UINT8(AX25_CTRL_RNR_MASK, ctrl & 0x0F);
}

TEST_CASE("AX25Frame: REJ control byte is an S-frame with correct N(R)", "[ax25_frame]")
{
    uint8_t ctrl = ax25_frame_build_rej_control(7, true);
    TEST_ASSERT_EQUAL(AX25_FRAME_S, ax25_frame_identify_type(ctrl));
    TEST_ASSERT_EQUAL_UINT8(7, ax25_frame_extract_nr(ctrl));
    TEST_ASSERT_TRUE(ctrl & AX25_CTRL_PF_BIT);
    TEST_ASSERT_EQUAL_UINT8(AX25_CTRL_REJ_MASK, ctrl & 0x0F);
}

TEST_CASE("AX25Frame: S-frame round-trip (build raw + parse)", "[ax25_frame]")
{
    ensure_pool();

    /* Manually build a minimal RR S-frame (no PID, no payload). */
    uint8_t raw[15];   /* 7 dest + 7 src + 1 control = 15 bytes */

    ax25_address_t dst, src;
    ax25_address_from_string("DEST-0", &dst);
    ax25_address_from_string("SRC-0",  &src);
    src.is_last = true;

    ax25_address_encode(&dst, &raw[0]);
    ax25_address_encode(&src, &raw[7]);
    raw[14] = ax25_frame_build_rr_control(5, false);

    ax25_frame_t frame;
    esp_err_t err = ax25_frame_parse(raw, sizeof(raw), &frame);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(AX25_FRAME_S, frame.type);
    TEST_ASSERT_EQUAL_UINT8(5, ax25_frame_extract_nr(frame.control));
}

TEST_CASE("AX25Frame: parse rejects frame too short", "[ax25_frame]")
{
    /* Minimum valid frame is 15 bytes (7 dest + 7 src + 1 control). */
    uint8_t raw[14] = {0};
    ax25_frame_t frame;
    esp_err_t err = ax25_frame_parse(raw, sizeof(raw), &frame);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, err);
}

TEST_CASE("AX25Frame: parse rejects NULL inputs", "[ax25_frame]")
{
    uint8_t raw[20] = {0};
    ax25_frame_t frame;
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ax25_frame_parse(NULL, 20, &frame));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ax25_frame_parse(raw, 20, NULL));
}

TEST_CASE("AX25Frame: max payload (256 bytes) round-trip", "[ax25_frame]")
{
    ensure_pool();

    ax25_frame_t original;
    ax25_frame_init(&original);
    ax25_address_from_string("DEST-0", &original.destination);
    ax25_address_from_string("SRC-0",  &original.source);
    original.type        = AX25_FRAME_UI;
    original.control     = AX25_CTRL_UI;
    original.pid         = AX25_PID_NONE;
    original.payload_len = AX25_MAX_INFO_LEN;
    for (int i = 0; i < AX25_MAX_INFO_LEN; i++) {
        original.payload[i] = (uint8_t)i;
    }

    ax25_buffer_t *buf = ax25_buffer_alloc(&test_pool, 500);
    TEST_ASSERT_NOT_NULL(buf);

    esp_err_t err = ax25_frame_build(&original, buf);
    TEST_ASSERT_EQUAL(ESP_OK, err);

    ax25_frame_t decoded;
    err = ax25_frame_parse(buf->data, buf->len, &decoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT32(AX25_MAX_INFO_LEN, decoded.payload_len);
    TEST_ASSERT_EQUAL_MEMORY(original.payload, decoded.payload, AX25_MAX_INFO_LEN);

    ax25_buffer_free(&test_pool, buf);
}

// ---------------------------------------------------------------------------
// C/R bit encoding and decoding (is_command field)
// ---------------------------------------------------------------------------

TEST_CASE("AX25Frame: Build command frame sets C bit in destination SSID", "[ax25_frame]")
{
    ensure_pool();

    ax25_frame_t frame;
    ax25_frame_init(&frame);
    ax25_address_from_string("DEST-0", &frame.destination);
    ax25_address_from_string("SRC-0",  &frame.source);
    frame.type       = AX25_FRAME_U;
    frame.control    = AX25_CTRL_SABM | AX25_CTRL_PF_BIT;
    frame.is_command = true;

    ax25_buffer_t *buf = ax25_buffer_alloc(&test_pool, 32);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_build(&frame, buf));

    /* Destination SSID is at byte index 6; bit 7 must be set for command frames. */
    TEST_ASSERT_TRUE(buf->data[6] & 0x80);
    /* Source SSID is at byte index 13; bit 7 must NOT be set for command frames. */
    TEST_ASSERT_FALSE(buf->data[13] & 0x80);

    ax25_buffer_free(&test_pool, buf);
}

TEST_CASE("AX25Frame: Build response frame sets R bit in source SSID", "[ax25_frame]")
{
    ensure_pool();

    ax25_frame_t frame;
    ax25_frame_init(&frame);
    ax25_address_from_string("DEST-0", &frame.destination);
    ax25_address_from_string("SRC-0",  &frame.source);
    frame.type       = AX25_FRAME_U;
    frame.control    = AX25_CTRL_UA | AX25_CTRL_PF_BIT;
    frame.is_command = false;   /* response */

    ax25_buffer_t *buf = ax25_buffer_alloc(&test_pool, 32);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_build(&frame, buf));

    /* Source SSID is at byte index 13; bit 7 must be set for response frames. */
    TEST_ASSERT_TRUE(buf->data[13] & 0x80);
    /* Destination SSID is at byte index 6; bit 7 must NOT be set for response frames. */
    TEST_ASSERT_FALSE(buf->data[6] & 0x80);

    ax25_buffer_free(&test_pool, buf);
}

TEST_CASE("AX25Frame: Parse sets is_command from destination SSID C bit", "[ax25_frame]")
{
    /* Build raw frames manually to control the C/R bit independently of ax25_frame_build. */
    uint8_t raw[15];
    ax25_address_t dst, src;
    ax25_address_from_string("DEST-0", &dst);
    ax25_address_from_string("SRC-0",  &src);
    src.is_last = true;

    /* Command frame: C bit in destination SSID (byte 6). */
    memset(raw, 0, sizeof(raw));
    ax25_address_encode(&dst, &raw[0]);
    ax25_address_encode(&src, &raw[7]);
    raw[6]  |= 0x80;  /* set C bit */
    raw[14]  = AX25_CTRL_SABM | AX25_CTRL_PF_BIT;

    ax25_frame_t frame;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_parse(raw, sizeof(raw), &frame));
    TEST_ASSERT_TRUE(frame.is_command);

    /* Response frame: R bit in source SSID (byte 13), C bit clear. */
    memset(raw, 0, sizeof(raw));
    ax25_address_encode(&dst, &raw[0]);
    ax25_address_encode(&src, &raw[7]);
    raw[13] |= 0x80;  /* set R bit */
    raw[14]  = AX25_CTRL_UA | AX25_CTRL_PF_BIT;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_parse(raw, sizeof(raw), &frame));
    TEST_ASSERT_FALSE(frame.is_command);
}

TEST_CASE("AX25Frame: C/R bit round-trip for command and response frames", "[ax25_frame]")
{
    ensure_pool();

    ax25_frame_t orig, decoded;

    /* Command round-trip */
    ax25_frame_init(&orig);
    ax25_address_from_string("DEST-0", &orig.destination);
    ax25_address_from_string("SRC-0",  &orig.source);
    orig.type       = AX25_FRAME_U;
    orig.control    = AX25_CTRL_SABM | AX25_CTRL_PF_BIT;
    orig.is_command = true;

    ax25_buffer_t *buf = ax25_buffer_alloc(&test_pool, 32);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_build(&orig, buf));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_parse(buf->data, buf->len, &decoded));
    TEST_ASSERT_TRUE(decoded.is_command);
    ax25_buffer_free(&test_pool, buf);

    /* Response round-trip */
    ax25_frame_init(&orig);
    ax25_address_from_string("DEST-0", &orig.destination);
    ax25_address_from_string("SRC-0",  &orig.source);
    orig.type       = AX25_FRAME_U;
    orig.control    = AX25_CTRL_UA | AX25_CTRL_PF_BIT;
    orig.is_command = false;

    buf = ax25_buffer_alloc(&test_pool, 32);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_build(&orig, buf));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_parse(buf->data, buf->len, &decoded));
    TEST_ASSERT_FALSE(decoded.is_command);
    ax25_buffer_free(&test_pool, buf);
}
