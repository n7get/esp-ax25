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
 * @file test_ax25_agwpe.c
 * @brief Unit tests for AGWPE encoder and decoder
 */

#include "unity.h"
#include "ax25_agwpe.h"
#include "ax25_frame.h"
#include "ax25_address.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

static uint8_t enc_buf[AGWPE_MAX_FRAME_SIZE];
static size_t enc_len;

/* Callback context for decoder tests */
typedef struct {
    int           call_count;
    agwpe_frame_t last_frame;
} agwpe_rx_ctx_t;

static void agwpe_rx_cb(const agwpe_frame_t *frame, void *user_data)
{
    agwpe_rx_ctx_t *ctx = (agwpe_rx_ctx_t *)user_data;
    ctx->call_count++;
    memcpy(&ctx->last_frame, frame, sizeof(agwpe_frame_t));
}

static void write_le32_test(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
    buf[2] = (uint8_t)((value >> 16) & 0xFF);
    buf[3] = (uint8_t)((value >> 24) & 0xFF);
}

// ---------------------------------------------------------------------------
// Initialization Tests
// ---------------------------------------------------------------------------

TEST_CASE("AGWPE: frame init clears structure", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    memset(&frame, 0xFF, sizeof(frame));
    
    agwpe_frame_init(&frame);
    
    TEST_ASSERT_EQUAL_UINT8(0, frame.header.port);
    TEST_ASSERT_EQUAL_UINT8(0, frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(0, frame.header.data_len);
}

TEST_CASE("AGWPE: decoder init", "[ax25_agwpe]")
{
    agwpe_rx_ctx_t ctx = {0};
    agwpe_decoder_t dec;
    
    agwpe_decoder_init(&dec, agwpe_rx_cb, &ctx);
    
    TEST_ASSERT_EQUAL(AGWPE_STATE_HEADER, dec.state);
    TEST_ASSERT_EQUAL_size_t(0, dec.bytes_received);
    TEST_ASSERT_EQUAL_PTR(agwpe_rx_cb, dec.callback);
}

TEST_CASE("AGWPE: NULL args handled safely", "[ax25_agwpe]")
{
    /* NULL frame - must not crash */
    agwpe_frame_init(NULL);
    agwpe_decoder_init(NULL, NULL, NULL);
    agwpe_decoder_reset(NULL);
    agwpe_decoder_process_byte(NULL, 0);
    agwpe_decoder_process_bytes(NULL, NULL, 0);
    
    /* NULL callback */
    agwpe_decoder_t dec;
    agwpe_decoder_init(&dec, NULL, NULL);
    /* Feed a complete frame - should not crash */
    uint8_t dummy[AGWPE_HEADER_SIZE] = {0};
    agwpe_decoder_process_bytes(&dec, dummy, sizeof(dummy));
}

// ---------------------------------------------------------------------------
// Encoder Tests
// ---------------------------------------------------------------------------

TEST_CASE("AGWPE: encode basic frame", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    
    frame.header.port = 1;
    frame.header.data_kind = 'R';
    
    enc_len = agwpe_frame_encode(&frame, enc_buf, sizeof(enc_buf));
    
    TEST_ASSERT_EQUAL_size_t(AGWPE_HEADER_SIZE, enc_len);
    TEST_ASSERT_EQUAL_UINT8(1, enc_buf[0]);   /* port */
    TEST_ASSERT_EQUAL_UINT8('R', enc_buf[4]); /* data_kind */
}

TEST_CASE("AGWPE: encode frame with data", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    
    frame.header.port = 2;
    frame.header.data_kind = 'D';
    frame.header.pid = 0xF0;
    agwpe_set_callsign(frame.header.call_from, "N0CALL");
    agwpe_set_callsign(frame.header.call_to, "DEST");
    
    const uint8_t payload[] = "Hello, World!";
    memcpy(frame.data, payload, sizeof(payload));
    frame.header.data_len = sizeof(payload);
    
    enc_len = agwpe_frame_encode(&frame, enc_buf, sizeof(enc_buf));
    
    TEST_ASSERT_EQUAL_size_t(AGWPE_HEADER_SIZE + sizeof(payload), enc_len);
    TEST_ASSERT_EQUAL_UINT8(2, enc_buf[0]);
    TEST_ASSERT_EQUAL_UINT8('D', enc_buf[4]);
    TEST_ASSERT_EQUAL_UINT8(0xF0, enc_buf[6]);
    
    /* Check callsigns */
    TEST_ASSERT_EQUAL_STRING_LEN("N0CALL", (char *)&enc_buf[8], 6);
    TEST_ASSERT_EQUAL_STRING_LEN("DEST", (char *)&enc_buf[18], 4);
    
    /* Check data length (little-endian at offset 28) */
    uint32_t data_len = enc_buf[28] | (enc_buf[29] << 8) | 
                        (enc_buf[30] << 16) | (enc_buf[31] << 24);
    TEST_ASSERT_EQUAL_UINT32(sizeof(payload), data_len);
    
    /* Check payload */
    TEST_ASSERT_EQUAL_MEMORY(payload, &enc_buf[AGWPE_HEADER_SIZE], sizeof(payload));
}

TEST_CASE("AGWPE: encode returns 0 on NULL args", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    
    TEST_ASSERT_EQUAL_size_t(0, agwpe_frame_encode(NULL, enc_buf, sizeof(enc_buf)));
    TEST_ASSERT_EQUAL_size_t(0, agwpe_frame_encode(&frame, NULL, 0));
}

TEST_CASE("AGWPE: encode returns 0 on buffer too small", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    frame.header.data_kind = 'R';
    
    /* Buffer smaller than header */
    TEST_ASSERT_EQUAL_size_t(0, agwpe_frame_encode(&frame, enc_buf, 10));
}

// ---------------------------------------------------------------------------
// Decoder Tests
// ---------------------------------------------------------------------------

TEST_CASE("AGWPE: decode basic frame", "[ax25_agwpe]")
{
    agwpe_rx_ctx_t ctx = {0};
    agwpe_decoder_t dec;
    agwpe_decoder_init(&dec, agwpe_rx_cb, &ctx);
    
    /* Build a simple header-only frame */
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    frame.header.port = 3;
    frame.header.data_kind = 'G';
    
    enc_len = agwpe_frame_encode(&frame, enc_buf, sizeof(enc_buf));
    agwpe_decoder_process_bytes(&dec, enc_buf, enc_len);
    
    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
    TEST_ASSERT_EQUAL_UINT8(3, ctx.last_frame.header.port);
    TEST_ASSERT_EQUAL_UINT8('G', ctx.last_frame.header.data_kind);
}

TEST_CASE("AGWPE: decode frame with data payload", "[ax25_agwpe]")
{
    agwpe_rx_ctx_t ctx = {0};
    agwpe_decoder_t dec;
    agwpe_decoder_init(&dec, agwpe_rx_cb, &ctx);
    
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    frame.header.port = 0;
    frame.header.data_kind = 'D';
    frame.header.pid = 0xF0;
    agwpe_set_callsign(frame.header.call_from, "TEST-5");
    agwpe_set_callsign(frame.header.call_to, "APRS");
    
    const uint8_t payload[] = {0x11, 0x22, 0x33, 0x44, 0x55};
    memcpy(frame.data, payload, sizeof(payload));
    frame.header.data_len = sizeof(payload);
    
    enc_len = agwpe_frame_encode(&frame, enc_buf, sizeof(enc_buf));
    agwpe_decoder_process_bytes(&dec, enc_buf, enc_len);
    
    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
    TEST_ASSERT_EQUAL_UINT8('D', ctx.last_frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(sizeof(payload), ctx.last_frame.header.data_len);
    TEST_ASSERT_EQUAL_MEMORY(payload, ctx.last_frame.data, sizeof(payload));
    
    char call_from[12];
    agwpe_get_callsign(ctx.last_frame.header.call_from, call_from);
    TEST_ASSERT_EQUAL_STRING("TEST-5", call_from);
}

TEST_CASE("AGWPE: decoder byte-by-byte", "[ax25_agwpe]")
{
    agwpe_rx_ctx_t ctx = {0};
    agwpe_decoder_t dec;
    agwpe_decoder_init(&dec, agwpe_rx_cb, &ctx);
    
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    frame.header.data_kind = 'R';
    
    enc_len = agwpe_frame_encode(&frame, enc_buf, sizeof(enc_buf));
    
    for (size_t i = 0; i < enc_len; i++) {
        agwpe_decoder_process_byte(&dec, enc_buf[i]);
    }
    
    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
}

TEST_CASE("AGWPE: decoder reset clears partial state", "[ax25_agwpe]")
{
    agwpe_rx_ctx_t ctx = {0};
    agwpe_decoder_t dec;
    agwpe_decoder_init(&dec, agwpe_rx_cb, &ctx);
    
    /* Feed partial header */
    uint8_t partial[20] = {0};
    partial[4] = 'R';
    agwpe_decoder_process_bytes(&dec, partial, sizeof(partial));
    
    TEST_ASSERT_EQUAL_INT(0, ctx.call_count);
    TEST_ASSERT_GREATER_THAN(0, dec.bytes_received);
    
    /* Reset */
    agwpe_decoder_reset(&dec);
    TEST_ASSERT_EQUAL(AGWPE_STATE_HEADER, dec.state);
    TEST_ASSERT_EQUAL_size_t(0, dec.bytes_received);
    
    /* Feed a complete frame */
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    frame.header.data_kind = 'G';
    enc_len = agwpe_frame_encode(&frame, enc_buf, sizeof(enc_buf));
    agwpe_decoder_process_bytes(&dec, enc_buf, enc_len);
    
    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
}

TEST_CASE("AGWPE: decode multiple sequential frames", "[ax25_agwpe]")
{
    agwpe_rx_ctx_t ctx = {0};
    agwpe_decoder_t dec;
    agwpe_decoder_init(&dec, agwpe_rx_cb, &ctx);
    
    /* Build two frames */
    agwpe_frame_t frame1, frame2;
    agwpe_frame_init(&frame1);
    agwpe_frame_init(&frame2);
    frame1.header.data_kind = 'R';
    frame2.header.data_kind = 'G';
    
    uint8_t buf[AGWPE_MAX_FRAME_SIZE * 2];
    size_t len1 = agwpe_frame_encode(&frame1, buf, sizeof(buf));
    size_t len2 = agwpe_frame_encode(&frame2, &buf[len1], sizeof(buf) - len1);
    
    agwpe_decoder_process_bytes(&dec, buf, len1 + len2);
    
    TEST_ASSERT_EQUAL_INT(2, ctx.call_count);
    TEST_ASSERT_EQUAL_UINT8('G', ctx.last_frame.header.data_kind);
}

TEST_CASE("AGWPE: decoder clamps oversized declared payload", "[ax25_agwpe]")
{
    agwpe_rx_ctx_t ctx = {0};
    agwpe_decoder_t dec;
    agwpe_decoder_init(&dec, agwpe_rx_cb, &ctx);

    uint8_t header[AGWPE_HEADER_SIZE] = {0};
    header[0] = 4;
    header[4] = 'K';
    write_le32_test(&header[28], AGWPE_MAX_DATA_LEN + 32u);

    agwpe_decoder_process_bytes(&dec, header, sizeof(header));
    TEST_ASSERT_EQUAL(AGWPE_STATE_DATA, dec.state);
    TEST_ASSERT_EQUAL_UINT32(AGWPE_MAX_DATA_LEN, dec.frame.header.data_len);

    memset(enc_buf, 0xAB, AGWPE_MAX_DATA_LEN);
    agwpe_decoder_process_bytes(&dec, enc_buf, AGWPE_MAX_DATA_LEN);

    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
    TEST_ASSERT_EQUAL_UINT8('K', ctx.last_frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(AGWPE_MAX_DATA_LEN, ctx.last_frame.header.data_len);
    TEST_ASSERT_EQUAL(AGWPE_STATE_HEADER, dec.state);
}

// ---------------------------------------------------------------------------
// Encode/Decode Round-Trip Tests
// ---------------------------------------------------------------------------

TEST_CASE("AGWPE: encode/decode round-trip", "[ax25_agwpe]")
{
    agwpe_frame_t original, decoded;
    agwpe_frame_init(&original);
    
    original.header.port = 5;
    original.header.data_kind = 'D';
    original.header.pid = 0xF0;
    original.header.user = 0x12345678;
    agwpe_set_callsign(original.header.call_from, "N0CALL-7");
    agwpe_set_callsign(original.header.call_to, "APRS-0");
    
    const char *msg = "Test message for round-trip";
    memcpy(original.data, msg, strlen(msg));
    original.header.data_len = (uint32_t)strlen(msg);
    
    enc_len = agwpe_frame_encode(&original, enc_buf, sizeof(enc_buf));
    TEST_ASSERT_GREATER_THAN(0, enc_len);
    
    esp_err_t err = agwpe_frame_decode(enc_buf, enc_len, &decoded);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    TEST_ASSERT_EQUAL_UINT8(original.header.port, decoded.header.port);
    TEST_ASSERT_EQUAL_UINT8(original.header.data_kind, decoded.header.data_kind);
    TEST_ASSERT_EQUAL_UINT8(original.header.pid, decoded.header.pid);
    TEST_ASSERT_EQUAL_UINT32(original.header.user, decoded.header.user);
    TEST_ASSERT_EQUAL_UINT32(original.header.data_len, decoded.header.data_len);
    TEST_ASSERT_EQUAL_MEMORY(original.header.call_from, decoded.header.call_from, AGWPE_CALLSIGN_LEN);
    TEST_ASSERT_EQUAL_MEMORY(original.header.call_to, decoded.header.call_to, AGWPE_CALLSIGN_LEN);
    TEST_ASSERT_EQUAL_MEMORY(original.data, decoded.data, original.header.data_len);
}

// ---------------------------------------------------------------------------
// Frame Builder Tests
// ---------------------------------------------------------------------------

TEST_CASE("AGWPE: build version request", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_build_version_req(&frame);
    
    TEST_ASSERT_EQUAL_UINT8('R', frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(0, frame.header.data_len);
}

TEST_CASE("AGWPE: build port info request", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_build_port_info_req(&frame);
    
    TEST_ASSERT_EQUAL_UINT8('G', frame.header.data_kind);
}

TEST_CASE("AGWPE: build port cap request", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_build_port_cap_req(2, &frame);
    
    TEST_ASSERT_EQUAL_UINT8('g', frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT8(2, frame.header.port);
}

TEST_CASE("AGWPE: build register callsign", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_register_call(0, "N0CALL-5", &frame);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('X', frame.header.data_kind);
    
    char call[12];
    agwpe_get_callsign(frame.header.call_from, call);
    TEST_ASSERT_EQUAL_STRING("N0CALL-5", call);
}

TEST_CASE("AGWPE: build connect request", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_connect_req(0, "N0CALL", "DEST", &frame);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('C', frame.header.data_kind);
    
    char from[12], to[12];
    agwpe_get_callsign(frame.header.call_from, from);
    agwpe_get_callsign(frame.header.call_to, to);
    TEST_ASSERT_EQUAL_STRING("N0CALL", from);
    TEST_ASSERT_EQUAL_STRING("DEST", to);
}

TEST_CASE("AGWPE: build connect via digipeaters", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    const char *digis[] = {"RELAY1", "RELAY2"};
    esp_err_t err = agwpe_build_connect_via_req(0, "SRC", "DST", digis, 2, &frame);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('v', frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT8(2, frame.data[0]); /* num digis */
    TEST_ASSERT_TRUE(frame.header.data_len > 1);
}

TEST_CASE("AGWPE: build disconnect request", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_disconnect_req(1, "SRC", "DST", &frame);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('d', frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT8(1, frame.header.port);
}

TEST_CASE("AGWPE: build send data", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    const uint8_t data[] = "Test data";
    esp_err_t err = agwpe_build_send_data(0, "SRC", "DST", data, sizeof(data), 0xF0, &frame);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('D', frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT8(0xF0, frame.header.pid);
    TEST_ASSERT_EQUAL_UINT32(sizeof(data), frame.header.data_len);
    TEST_ASSERT_EQUAL_MEMORY(data, frame.data, sizeof(data));
}

TEST_CASE("AGWPE: build send unproto", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    const uint8_t data[] = "APRS packet";
    esp_err_t err = agwpe_build_send_unproto(0, "N0CALL-1", "APRS", data, sizeof(data), 0xF0, &frame);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('M', frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(sizeof(data), frame.header.data_len);
}

TEST_CASE("AGWPE: build send unproto via", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    const char *digis[] = {"WIDE1-1", "WIDE2-1"};
    const uint8_t data[] = "VIA packet";
    esp_err_t err = agwpe_build_send_unproto_via(0, "N0CALL", "APRS", digis, 2, 
                                                   data, sizeof(data), 0xF0, &frame);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('V', frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT8(2, frame.data[0]); /* num digis */
}

TEST_CASE("AGWPE: build connect via rejects oversized path", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    char long_digi[AGWPE_MAX_DATA_LEN + 2];
    memset(long_digi, 'A', sizeof(long_digi) - 1);
    long_digi[sizeof(long_digi) - 1] = '\0';
    const char *digis[] = {long_digi};

    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM,
                      agwpe_build_connect_via_req(0, "N0CALL", "DEST",
                                                  digis, 1, &frame));
}

TEST_CASE("AGWPE: build send unproto via rejects payload overflow", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    const char *digis[] = {"WIDE1-1"};
    uint8_t payload[AGWPE_MAX_DATA_LEN];
    memset(payload, 0x5A, sizeof(payload));

    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM,
                      agwpe_build_send_unproto_via(0, "N0CALL", "APRS",
                                                   digis, 1,
                                                   payload, sizeof(payload),
                                                   0xF0, &frame));
}

TEST_CASE("AGWPE: build enable monitor", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_build_enable_monitor(&frame);
    
    TEST_ASSERT_EQUAL_UINT8('m', frame.header.data_kind);
}

TEST_CASE("AGWPE: build enable raw", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_build_enable_raw(&frame);
    
    TEST_ASSERT_EQUAL_UINT8('k', frame.header.data_kind);
}

TEST_CASE("AGWPE: build outstanding request", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_outstanding_req(0, "SRC", "DST", &frame);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('Y', frame.header.data_kind);
}

TEST_CASE("AGWPE: build heard request", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_build_heard_req(3, &frame);
    
    TEST_ASSERT_EQUAL_UINT8('H', frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT8(3, frame.header.port);
}

// ---------------------------------------------------------------------------
// AX.25 Conversion Tests
// ---------------------------------------------------------------------------

TEST_CASE("AGWPE: AX.25 UI frame to AGWPE unproto", "[ax25_agwpe]")
{
    ax25_frame_t ax25;
    ax25_frame_init(&ax25);
    ax25_address_from_string("APRS-0", &ax25.destination);
    ax25_address_from_string("N0CALL-5", &ax25.source);
    ax25.type = AX25_FRAME_UI;
    ax25.control = AX25_CTRL_UI;
    ax25.pid = 0xF0;
    const char *info = "!3518.00N/10623.00W-PHG2360";
    memcpy(ax25.payload, info, strlen(info));
    ax25.payload_len = strlen(info);
    
    agwpe_frame_t agwpe;
    esp_err_t err = ax25_to_agwpe_unproto(&ax25, 0, &agwpe);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('M', agwpe.header.data_kind);
    TEST_ASSERT_EQUAL_UINT8(0xF0, agwpe.header.pid);
    
    char from[12], to[12];
    agwpe_get_callsign(agwpe.header.call_from, from);
    agwpe_get_callsign(agwpe.header.call_to, to);
    TEST_ASSERT_EQUAL_STRING("N0CALL-5", from);
    TEST_ASSERT_EQUAL_STRING("APRS", to);
    
    TEST_ASSERT_EQUAL_UINT32(strlen(info), agwpe.header.data_len);
    TEST_ASSERT_EQUAL_MEMORY(info, agwpe.data, strlen(info));
}

TEST_CASE("AGWPE: AX.25 UI frame with digipeaters to AGWPE unproto via", "[ax25_agwpe]")
{
    ax25_frame_t ax25;
    ax25_frame_init(&ax25);
    ax25_address_from_string("APRS", &ax25.destination);
    ax25_address_from_string("N0CALL", &ax25.source);
    ax25_address_from_string("WIDE1-1", &ax25.digipeaters[0]);
    ax25_address_from_string("WIDE2-1", &ax25.digipeaters[1]);
    ax25.num_digipeaters = 2;
    ax25.type = AX25_FRAME_UI;
    ax25.control = AX25_CTRL_UI;
    ax25.pid = 0xF0;
    const char *info = "Test";
    memcpy(ax25.payload, info, strlen(info));
    ax25.payload_len = strlen(info);
    
    agwpe_frame_t agwpe;
    esp_err_t err = ax25_to_agwpe_unproto(&ax25, 1, &agwpe);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('V', agwpe.header.data_kind);
    TEST_ASSERT_EQUAL_UINT8(1, agwpe.header.port);
    TEST_ASSERT_EQUAL_UINT8(2, agwpe.data[0]); /* num digis */
}

TEST_CASE("AGWPE: AX.25 frame to AGWPE raw", "[ax25_agwpe]")
{
    ax25_frame_t ax25;
    ax25_frame_init(&ax25);
    ax25_address_from_string("DEST-0", &ax25.destination);
    ax25_address_from_string("SRC-1", &ax25.source);
    ax25.type = AX25_FRAME_UI;
    ax25.control = AX25_CTRL_UI;
    ax25.pid = 0xF0;
    const char *info = "Raw test";
    memcpy(ax25.payload, info, strlen(info));
    ax25.payload_len = strlen(info);
    
    agwpe_frame_t agwpe;
    esp_err_t err = ax25_to_agwpe_raw(&ax25, 2, &agwpe);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8('K', agwpe.header.data_kind);
    TEST_ASSERT_EQUAL_UINT8(2, agwpe.header.port);
    
    /* Data should contain encoded AX.25 frame */
    /* Minimum: 7 (dest) + 7 (src) + 1 (ctrl) + 1 (pid) + payload = 16 + strlen(info) */
    TEST_ASSERT_TRUE(agwpe.header.data_len >= 16 + strlen(info));
}

TEST_CASE("AGWPE: AGWPE raw frame to AX.25", "[ax25_agwpe]")
{
    /* First create an AX.25 frame and convert to AGWPE raw */
    ax25_frame_t original;
    ax25_frame_init(&original);
    ax25_address_from_string("DEST-0", &original.destination);
    ax25_address_from_string("SRC-3", &original.source);
    original.type = AX25_FRAME_UI;
    original.control = AX25_CTRL_UI;
    original.pid = 0xF0;
    const char *info = "Round trip";
    memcpy(original.payload, info, strlen(info));
    original.payload_len = strlen(info);
    
    agwpe_frame_t agwpe;
    esp_err_t err = ax25_to_agwpe_raw(&original, 0, &agwpe);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Change kind to 'T' (received raw) for conversion back */
    agwpe.header.data_kind = 'T';
    
    /* Convert back to AX.25 */
    ax25_frame_t decoded;
    err = agwpe_raw_to_ax25(&agwpe, &decoded);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING(original.destination.callsign, decoded.destination.callsign);
    TEST_ASSERT_EQUAL_STRING(original.source.callsign, decoded.source.callsign);
    TEST_ASSERT_EQUAL_UINT8(original.source.ssid, decoded.source.ssid);
    TEST_ASSERT_EQUAL_UINT8(original.pid, decoded.pid);
    TEST_ASSERT_EQUAL_size_t(original.payload_len, decoded.payload_len);
    TEST_ASSERT_EQUAL_MEMORY(original.payload, decoded.payload, original.payload_len);
}

TEST_CASE("AGWPE: monitored frame converts to AX.25 with payload clamp", "[ax25_agwpe]")
{
    agwpe_frame_t agwpe;
    agwpe_frame_init(&agwpe);
    agwpe.header.data_kind = 'U';
    agwpe.header.pid = 0xF0;
    agwpe_set_callsign(agwpe.header.call_from, "REMOTE-1");
    agwpe_set_callsign(agwpe.header.call_to, "LOCAL");
    agwpe.header.data_len = AGWPE_MAX_DATA_LEN;
    memset(agwpe.data, 'Q', sizeof(agwpe.data));

    ax25_frame_t ax25;
    esp_err_t err = agwpe_monitored_to_ax25(&agwpe, &ax25);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(AX25_FRAME_UI, ax25.type);
    TEST_ASSERT_EQUAL_UINT8(AX25_CTRL_UI, ax25.control);
    TEST_ASSERT_EQUAL_UINT8(0xF0, ax25.pid);
    TEST_ASSERT_EQUAL_STRING("REMOTE", ax25.source.callsign);
    TEST_ASSERT_EQUAL_UINT8(1, ax25.source.ssid);
    TEST_ASSERT_EQUAL_STRING("LOCAL", ax25.destination.callsign);
    TEST_ASSERT_EQUAL_UINT32(AX25_MAX_INFO_LEN, ax25.payload_len);
    TEST_ASSERT_EQUAL_MEMORY(agwpe.data, ax25.payload, AX25_MAX_INFO_LEN);
}

TEST_CASE("AGWPE: monitored_to_ax25 rejects unsupported kind", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    frame.header.data_kind = 'R';
    agwpe_set_callsign(frame.header.call_from, "SRC");
    agwpe_set_callsign(frame.header.call_to, "DST");

    ax25_frame_t ax25;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, agwpe_monitored_to_ax25(&frame, &ax25));
}

// ---------------------------------------------------------------------------
// Response Parsing Tests
// ---------------------------------------------------------------------------

TEST_CASE("AGWPE: parse version response", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    frame.header.data_kind = 'R';
    frame.header.data_len = 4;
    /* Version 2000.0 in little-endian */
    frame.data[0] = 0xD0; /* 2000 & 0xFF */
    frame.data[1] = 0x07; /* 2000 >> 8 */
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    
    agwpe_version_t version;
    esp_err_t err = agwpe_parse_version_resp(&frame, &version);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT16(2000, version.major);
    TEST_ASSERT_EQUAL_UINT16(0, version.minor);
}

TEST_CASE("AGWPE: parse port caps", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    frame.header.data_kind = 'g';
    frame.header.data_len = 8;
    frame.data[0] = 12;  /* on_air_baud (1200 indicator) */
    frame.data[1] = 50;  /* traffic_level */
    frame.data[2] = 30;  /* tx_delay */
    frame.data[3] = 5;   /* tx_tail */
    frame.data[4] = 63;  /* persist */
    frame.data[5] = 10;  /* slot_time */
    frame.data[6] = 4;   /* max_frame */
    frame.data[7] = 2;   /* active_conns */
    
    agwpe_port_caps_t caps;
    esp_err_t err = agwpe_parse_port_caps(&frame, &caps);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT8(12, caps.on_air_baud);
    TEST_ASSERT_EQUAL_UINT8(50, caps.traffic_level);
    TEST_ASSERT_EQUAL_UINT8(30, caps.tx_delay);
    TEST_ASSERT_EQUAL_UINT8(63, caps.persist);
    TEST_ASSERT_EQUAL_UINT8(4, caps.max_frame);
}

TEST_CASE("AGWPE: parse outstanding response", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    frame.header.data_kind = 'y';
    frame.header.data_len = 4;
    /* Outstanding = 5 in little-endian */
    frame.data[0] = 5;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    
    uint32_t outstanding;
    esp_err_t err = agwpe_parse_outstanding_resp(&frame, &outstanding);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT32(5, outstanding);
}

// ---------------------------------------------------------------------------
// Utility Function Tests
// ---------------------------------------------------------------------------

TEST_CASE("AGWPE: kind to string", "[ax25_agwpe]")
{
    TEST_ASSERT_EQUAL_STRING("Version", agwpe_kind_to_string('R'));
    TEST_ASSERT_EQUAL_STRING("Port Info", agwpe_kind_to_string('G'));
    TEST_ASSERT_EQUAL_STRING("Connect Request", agwpe_kind_to_string('C'));
    TEST_ASSERT_EQUAL_STRING("Data", agwpe_kind_to_string('D'));
    TEST_ASSERT_EQUAL_STRING("Unproto", agwpe_kind_to_string('M'));
    TEST_ASSERT_EQUAL_STRING("Raw Frame", agwpe_kind_to_string('T'));
    TEST_ASSERT_EQUAL_STRING("Unknown", agwpe_kind_to_string('?'));
}

TEST_CASE("AGWPE: callsign set/get", "[ax25_agwpe]")
{
    char buf[AGWPE_CALLSIGN_LEN];
    char result[AGWPE_CALLSIGN_LEN + 1];
    
    /* Normal callsign */
    agwpe_set_callsign(buf, "N0CALL-5");
    agwpe_get_callsign(buf, result);
    TEST_ASSERT_EQUAL_STRING("N0CALL-5", result);
    
    /* Long callsign gets truncated */
    agwpe_set_callsign(buf, "VERYLONGCALL");
    agwpe_get_callsign(buf, result);
    TEST_ASSERT_TRUE(strlen(result) <= AGWPE_CALLSIGN_LEN - 1);
    
    /* Empty/NULL callsign */
    agwpe_set_callsign(buf, NULL);
    agwpe_get_callsign(buf, result);
    TEST_ASSERT_EQUAL_STRING("", result);
    
    /* NULL handling */
    agwpe_set_callsign(NULL, "TEST");
    agwpe_get_callsign(NULL, result);
    TEST_ASSERT_EQUAL_STRING("", result);
}

TEST_CASE("AGWPE: is_request", "[ax25_agwpe]")
{
    TEST_ASSERT_TRUE(agwpe_is_request('R'));
    TEST_ASSERT_TRUE(agwpe_is_request('G'));
    TEST_ASSERT_TRUE(agwpe_is_request('C'));
    TEST_ASSERT_TRUE(agwpe_is_request('D'));
    TEST_ASSERT_TRUE(agwpe_is_request('M'));
    TEST_ASSERT_TRUE(agwpe_is_request('K'));
    TEST_ASSERT_TRUE(agwpe_is_request('m'));
    TEST_ASSERT_FALSE(agwpe_is_request('U'));
    TEST_ASSERT_FALSE(agwpe_is_request('T'));
    TEST_ASSERT_FALSE(agwpe_is_request('c'));
}

TEST_CASE("AGWPE: is_monitored", "[ax25_agwpe]")
{
    TEST_ASSERT_TRUE(agwpe_is_monitored('U'));
    TEST_ASSERT_TRUE(agwpe_is_monitored('S'));
    TEST_ASSERT_TRUE(agwpe_is_monitored('I'));
    TEST_ASSERT_TRUE(agwpe_is_monitored('T'));
    TEST_ASSERT_FALSE(agwpe_is_monitored('R'));
    TEST_ASSERT_FALSE(agwpe_is_monitored('M'));
    TEST_ASSERT_FALSE(agwpe_is_monitored('D'));
}

// ---------------------------------------------------------------------------
// Error Handling Tests
// ---------------------------------------------------------------------------

TEST_CASE("AGWPE: frame decode rejects too short data", "[ax25_agwpe]")
{
    uint8_t short_data[10] = {0};
    agwpe_frame_t frame;
    
    esp_err_t err = agwpe_frame_decode(short_data, sizeof(short_data), &frame);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, err);
}

TEST_CASE("AGWPE: frame decode rejects NULL args", "[ax25_agwpe]")
{
    uint8_t data[AGWPE_HEADER_SIZE] = {0};
    agwpe_frame_t frame;
    
    TEST_ASSERT_NOT_EQUAL(ESP_OK, agwpe_frame_decode(NULL, AGWPE_HEADER_SIZE, &frame));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, agwpe_frame_decode(data, AGWPE_HEADER_SIZE, NULL));
}

TEST_CASE("AGWPE: raw_to_ax25 rejects wrong frame type", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    agwpe_frame_init(&frame);
    frame.header.data_kind = 'M';  /* Not 'T' */
    
    ax25_frame_t ax25;
    esp_err_t err = agwpe_raw_to_ax25(&frame, &ax25);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, err);
}

TEST_CASE("AGWPE: builder functions reject NULL args", "[ax25_agwpe]")
{
    agwpe_frame_t frame;
    
    TEST_ASSERT_NOT_EQUAL(ESP_OK, agwpe_build_register_call(0, NULL, &frame));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, agwpe_build_register_call(0, "TEST", NULL));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, agwpe_build_connect_req(0, NULL, "DST", &frame));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, agwpe_build_connect_req(0, "SRC", NULL, &frame));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, agwpe_build_send_data(0, NULL, "DST", NULL, 0, 0, &frame));
}

TEST_CASE("AGWPE: conversion rejects NULL args", "[ax25_agwpe]")
{
    ax25_frame_t ax25;
    agwpe_frame_t agwpe;
    
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ax25_to_agwpe_raw(NULL, 0, &agwpe));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ax25_to_agwpe_raw(&ax25, 0, NULL));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ax25_to_agwpe_unproto(NULL, 0, &agwpe));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, agwpe_raw_to_ax25(NULL, &ax25));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, agwpe_raw_to_ax25(&agwpe, NULL));
    TEST_ASSERT_NOT_EQUAL(ESP_OK, agwpe_monitored_to_ax25(NULL, &ax25));
}
