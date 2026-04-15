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
 * @file test_ax25_kiss.c
 * @brief Unit tests for AX.25 KISS encoder and decoder
 */

#include "unity.h"
#include "ax25_kiss.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/* Static buffer for encoder output. */
static uint8_t enc_buf[AX25_KISS_MAX_ENCODED_SIZE];
static size_t  enc_len;

/* Callback context for decoder tests. */
typedef struct {
    int     call_count;
    uint8_t last_port;
    uint8_t last_cmd;
    uint8_t last_data[AX25_MAX_INFO_LEN];
    size_t  last_len;
} kiss_rx_ctx_t;

static void kiss_rx_cb(uint8_t port, uint8_t command,
                       const uint8_t *data, size_t len,
                       void *user_data)
{
    kiss_rx_ctx_t *ctx = (kiss_rx_ctx_t *)user_data;
    ctx->call_count++;
    ctx->last_port = port;
    ctx->last_cmd  = command;
    size_t n = (len < sizeof(ctx->last_data)) ? len : sizeof(ctx->last_data);
    memcpy(ctx->last_data, data, n);
    ctx->last_len = n;
}

// ---------------------------------------------------------------------------
// Encoder tests
// ---------------------------------------------------------------------------

TEST_CASE("KISS encode: basic structure", "[ax25_kiss]")
{
    const uint8_t payload[] = {0x01, 0x02, 0x03};
    enc_len = ax25_kiss_encode(0, 0, payload, sizeof(payload),
                               enc_buf, sizeof(enc_buf));

    TEST_ASSERT_GREATER_THAN(0, enc_len);

    /* Must start and end with FEND (0xC0). */
    TEST_ASSERT_EQUAL_UINT8(AX25_KISS_FEND, enc_buf[0]);
    TEST_ASSERT_EQUAL_UINT8(AX25_KISS_FEND, enc_buf[enc_len - 1]);

    /* Byte after the opening FEND is the port/command byte. */
    TEST_ASSERT_EQUAL_UINT8(0x00, enc_buf[1]);   /* port=0, cmd=0 */

    /* Total minimal length: FEND + port_cmd + 3 data bytes + FEND = 6. */
    TEST_ASSERT_EQUAL_size_t(6, enc_len);
}

TEST_CASE("KISS encode: port and command fields", "[ax25_kiss]")
{
    const uint8_t payload[] = {0xAB};

    /* port=3, cmd=1  =>  port_cmd = (3 << 4) | 1 = 0x31 */
    enc_len = ax25_kiss_encode(3, 1, payload, sizeof(payload),
                               enc_buf, sizeof(enc_buf));
    TEST_ASSERT_EQUAL_UINT8(0x31, enc_buf[1]);

    /* port=15, cmd=0  =>  port_cmd = 0xF0 — FEND-equivalent must be escaped */
    enc_len = ax25_kiss_encode(15, 0, payload, sizeof(payload),
                               enc_buf, sizeof(enc_buf));
    /* 0xF0 is not a special KISS byte, so it passes through unescaped. */
    TEST_ASSERT_EQUAL_UINT8(0xF0, enc_buf[1]);
}

TEST_CASE("KISS encode: FEND in payload is escaped", "[ax25_kiss]")
{
    const uint8_t payload[] = {0xAA, AX25_KISS_FEND, 0xBB};
    enc_len = ax25_kiss_encode(0, 0, payload, sizeof(payload),
                               enc_buf, sizeof(enc_buf));

    /* Locate the data section (skip opening FEND and port/cmd byte). */
    const uint8_t *d = enc_buf + 2;
    size_t dlen      = enc_len - 3;   /* exclude opening FEND, port_cmd, closing FEND */

    /* Expect: 0xAA, FESC, TFEND, 0xBB */
    TEST_ASSERT_EQUAL_size_t(4, dlen);
    TEST_ASSERT_EQUAL_UINT8(0xAA,            d[0]);
    TEST_ASSERT_EQUAL_UINT8(AX25_KISS_FESC,  d[1]);
    TEST_ASSERT_EQUAL_UINT8(AX25_KISS_TFEND, d[2]);
    TEST_ASSERT_EQUAL_UINT8(0xBB,            d[3]);
}

TEST_CASE("KISS encode: FESC in payload is escaped", "[ax25_kiss]")
{
    const uint8_t payload[] = {AX25_KISS_FESC, 0x42};
    enc_len = ax25_kiss_encode(0, 0, payload, sizeof(payload),
                               enc_buf, sizeof(enc_buf));

    const uint8_t *d = enc_buf + 2;
    size_t dlen      = enc_len - 3;

    /* Expect: FESC, TFESC, 0x42 */
    TEST_ASSERT_EQUAL_size_t(3, dlen);
    TEST_ASSERT_EQUAL_UINT8(AX25_KISS_FESC,  d[0]);
    TEST_ASSERT_EQUAL_UINT8(AX25_KISS_TFESC, d[1]);
    TEST_ASSERT_EQUAL_UINT8(0x42,            d[2]);
}

TEST_CASE("KISS encode: NULL data returns 0", "[ax25_kiss]")
{
    size_t written = ax25_kiss_encode(0, 0, NULL, 4, enc_buf, sizeof(enc_buf));
    TEST_ASSERT_EQUAL_size_t(0, written);
}

TEST_CASE("KISS encode: NULL output buffer returns 0", "[ax25_kiss]")
{
    const uint8_t payload[] = {0x01};
    size_t written = ax25_kiss_encode(0, 0, payload, sizeof(payload), NULL, 0);
    TEST_ASSERT_EQUAL_size_t(0, written);
}

TEST_CASE("KISS encode: empty payload", "[ax25_kiss]")
{
    const uint8_t dummy = 0;
    /* len = 0 means no data bytes */
    enc_len = ax25_kiss_encode(0, 0, &dummy, 0, enc_buf, sizeof(enc_buf));
    /* FEND + port_cmd + FEND = 3 bytes */
    TEST_ASSERT_EQUAL_size_t(3, enc_len);
    TEST_ASSERT_EQUAL_UINT8(AX25_KISS_FEND, enc_buf[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00,           enc_buf[1]);
    TEST_ASSERT_EQUAL_UINT8(AX25_KISS_FEND, enc_buf[2]);
}

// ---------------------------------------------------------------------------
// Decoder tests
// ---------------------------------------------------------------------------

TEST_CASE("KISS decoder: init", "[ax25_kiss]")
{
    kiss_rx_ctx_t ctx = {0};
    ax25_kiss_decoder_t dec;

    ax25_kiss_decoder_init(&dec, kiss_rx_cb, &ctx);
    TEST_ASSERT_EQUAL(KISS_STATE_WAIT_FEND, dec.state);
}

TEST_CASE("KISS decoder: NULL args are handled safely", "[ax25_kiss]")
{
    /* NULL decoder — must not crash. */
    ax25_kiss_decoder_init(NULL, kiss_rx_cb, NULL);
    ax25_kiss_decoder_process_byte(NULL, 0x00);
    ax25_kiss_decoder_reset(NULL);

    /* NULL callback — init must not crash; frame must not be delivered. */
    ax25_kiss_decoder_t dec;
    ax25_kiss_decoder_init(&dec, NULL, NULL);
    const uint8_t frame[] = {AX25_KISS_FEND, 0x00, 0x42, AX25_KISS_FEND};
    ax25_kiss_decoder_process_bytes(&dec, frame, sizeof(frame));
    /* If we reach here without crashing, the test passes. */
}

TEST_CASE("KISS decoder: complete frame decoded", "[ax25_kiss]")
{
    kiss_rx_ctx_t ctx = {0};
    ax25_kiss_decoder_t dec;
    ax25_kiss_decoder_init(&dec, kiss_rx_cb, &ctx);

    /* Build a simple KISS frame in a local buffer. */
    const uint8_t payload[] = {0x11, 0x22, 0x33};
    enc_len = ax25_kiss_encode(0, 0, payload, sizeof(payload),
                               enc_buf, sizeof(enc_buf));

    ax25_kiss_decoder_process_bytes(&dec, enc_buf, enc_len);

    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
    TEST_ASSERT_EQUAL_size_t(3, ctx.last_len);
    TEST_ASSERT_EQUAL_MEMORY(payload, ctx.last_data, 3);
}

TEST_CASE("KISS decoder: port and command extracted", "[ax25_kiss]")
{
    kiss_rx_ctx_t ctx = {0};
    ax25_kiss_decoder_t dec;
    ax25_kiss_decoder_init(&dec, kiss_rx_cb, &ctx);

    const uint8_t payload[] = {0xAB};
    enc_len = ax25_kiss_encode(5, 2, payload, sizeof(payload),
                               enc_buf, sizeof(enc_buf));

    ax25_kiss_decoder_process_bytes(&dec, enc_buf, enc_len);

    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
    TEST_ASSERT_EQUAL_UINT8(5, ctx.last_port);
    TEST_ASSERT_EQUAL_UINT8(2, ctx.last_cmd);
}

TEST_CASE("KISS decoder: extra leading FENDs ignored", "[ax25_kiss]")
{
    kiss_rx_ctx_t ctx = {0};
    ax25_kiss_decoder_t dec;
    ax25_kiss_decoder_init(&dec, kiss_rx_cb, &ctx);

    /* Extra FENDs before the frame must not confuse the decoder. */
    const uint8_t stream[] = {
        AX25_KISS_FEND, AX25_KISS_FEND,  /* spurious FENDs */
        AX25_KISS_FEND,                   /* start of real frame */
        0x00,                             /* port=0, cmd=0 */
        0x42,                             /* payload byte */
        AX25_KISS_FEND                    /* end of frame */
    };
    ax25_kiss_decoder_process_bytes(&dec, stream, sizeof(stream));

    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
    TEST_ASSERT_EQUAL_size_t(1, ctx.last_len);
    TEST_ASSERT_EQUAL_UINT8(0x42, ctx.last_data[0]);
}

TEST_CASE("KISS decoder: FESC TFEND decoded as 0xC0", "[ax25_kiss]")
{
    kiss_rx_ctx_t ctx = {0};
    ax25_kiss_decoder_t dec;
    ax25_kiss_decoder_init(&dec, kiss_rx_cb, &ctx);

    /* Frame containing 0xC0 (FEND), encoded as FESC TFEND. */
    const uint8_t stream[] = {
        AX25_KISS_FEND,   /* start */
        0x00,             /* port/cmd */
        AX25_KISS_FESC, AX25_KISS_TFEND,  /* escaped 0xC0 */
        AX25_KISS_FEND    /* end */
    };
    ax25_kiss_decoder_process_bytes(&dec, stream, sizeof(stream));

    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
    TEST_ASSERT_EQUAL_size_t(1, ctx.last_len);
    TEST_ASSERT_EQUAL_UINT8(AX25_KISS_FEND, ctx.last_data[0]);
}

TEST_CASE("KISS decoder: FESC TFESC decoded as 0xDB", "[ax25_kiss]")
{
    kiss_rx_ctx_t ctx = {0};
    ax25_kiss_decoder_t dec;
    ax25_kiss_decoder_init(&dec, kiss_rx_cb, &ctx);

    const uint8_t stream[] = {
        AX25_KISS_FEND,
        0x00,
        AX25_KISS_FESC, AX25_KISS_TFESC,
        AX25_KISS_FEND
    };
    ax25_kiss_decoder_process_bytes(&dec, stream, sizeof(stream));

    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
    TEST_ASSERT_EQUAL_size_t(1, ctx.last_len);
    TEST_ASSERT_EQUAL_UINT8(AX25_KISS_FESC, ctx.last_data[0]);
}

TEST_CASE("KISS decoder: multiple sequential frames", "[ax25_kiss]")
{
    kiss_rx_ctx_t ctx = {0};
    ax25_kiss_decoder_t dec;
    ax25_kiss_decoder_init(&dec, kiss_rx_cb, &ctx);

    /* Two concatenated frames. */
    const uint8_t stream[] = {
        AX25_KISS_FEND, 0x00, 0x01, AX25_KISS_FEND,
        AX25_KISS_FEND, 0x00, 0x02, AX25_KISS_FEND,
    };
    ax25_kiss_decoder_process_bytes(&dec, stream, sizeof(stream));

    TEST_ASSERT_EQUAL_INT(2, ctx.call_count);
}

TEST_CASE("KISS decoder: reset clears partial state", "[ax25_kiss]")
{
    kiss_rx_ctx_t ctx = {0};
    ax25_kiss_decoder_t dec;
    ax25_kiss_decoder_init(&dec, kiss_rx_cb, &ctx);

    /* Start a frame but don't close it. */
    const uint8_t partial[] = {AX25_KISS_FEND, 0x00, 0xDE, 0xAD};
    ax25_kiss_decoder_process_bytes(&dec, partial, sizeof(partial));

    TEST_ASSERT_EQUAL_INT(0, ctx.call_count);

    /* Reset must discard the partial frame. */
    ax25_kiss_decoder_reset(&dec);
    TEST_ASSERT_EQUAL(KISS_STATE_WAIT_FEND, dec.state);

    /* A new, complete frame must be delivered normally. */
    const uint8_t good[] = {AX25_KISS_FEND, 0x00, 0x55, AX25_KISS_FEND};
    ax25_kiss_decoder_process_bytes(&dec, good, sizeof(good));
    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
}

TEST_CASE("KISS encode/decode: round-trip with special bytes", "[ax25_kiss]")
{
    kiss_rx_ctx_t ctx = {0};
    ax25_kiss_decoder_t dec;
    ax25_kiss_decoder_init(&dec, kiss_rx_cb, &ctx);

    /* Payload that contains both special bytes. */
    const uint8_t original[] = {
        0x00, AX25_KISS_FEND, 0x01, AX25_KISS_FESC, 0x02, 0xFF
    };

    enc_len = ax25_kiss_encode(2, 0, original, sizeof(original),
                               enc_buf, sizeof(enc_buf));
    ax25_kiss_decoder_process_bytes(&dec, enc_buf, enc_len);

    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
    TEST_ASSERT_EQUAL_size_t(sizeof(original), ctx.last_len);
    TEST_ASSERT_EQUAL_MEMORY(original, ctx.last_data, sizeof(original));
}

TEST_CASE("KISS decoder: process_byte (single-byte API) produces same result", "[ax25_kiss]")
{
    kiss_rx_ctx_t ctx = {0};
    ax25_kiss_decoder_t dec;
    ax25_kiss_decoder_init(&dec, kiss_rx_cb, &ctx);

    /* Encode a simple two-byte payload. */
    const uint8_t payload[] = {0xAA, 0xBB};
    enc_len = ax25_kiss_encode(0, 0, payload, sizeof(payload),
                               enc_buf, sizeof(enc_buf));

    /* Feed bytes one at a time using the single-byte API. */
    for (size_t i = 0; i < enc_len; i++) {
        ax25_kiss_decoder_process_byte(&dec, enc_buf[i]);
    }

    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);
    TEST_ASSERT_EQUAL_size_t(sizeof(payload), ctx.last_len);
    TEST_ASSERT_EQUAL_UINT8(0xAA, ctx.last_data[0]);
    TEST_ASSERT_EQUAL_UINT8(0xBB, ctx.last_data[1]);
}
