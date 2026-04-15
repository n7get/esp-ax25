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
 * @file test_ax25_print.c
 * @brief Unit tests for AX.25 pretty-print utilities
 *
 * ax25_print_frame and ax25_print_buffer write to ESP_LOGI and produce no
 * testable return value, so these tests are smoke tests: they verify that
 * the functions don't crash, and that the pure string-mapping helpers
 * (ax25_frame_type_str, ax25_conn_state_str) return the correct strings for
 * all defined enum values.
 */

#include "unity.h"
#include "ax25_print.h"
#include "ax25_frame.h"
#include "ax25_buffer.h"
#include "ax25_address.h"
#include "ax25_types.h"
#include <string.h>

// ---------------------------------------------------------------------------
// ax25_frame_type_str
// ---------------------------------------------------------------------------

TEST_CASE("AX25Print: frame_type_str returns non-NULL non-empty string for every type", "[ax25_print]")
{
    const ax25_frame_type_t types[] = {
        AX25_FRAME_UI, AX25_FRAME_I, AX25_FRAME_S, AX25_FRAME_U, AX25_FRAME_UNKNOWN
    };
    for (size_t i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
        const char *s = ax25_frame_type_str(types[i]);
        TEST_ASSERT_NOT_NULL(s);
        TEST_ASSERT_GREATER_THAN(0, (int)strlen(s));
    }
}

TEST_CASE("AX25Print: frame_type_str returns distinct strings for distinct types", "[ax25_print]")
{
    TEST_ASSERT_NOT_EQUAL(0, strcmp(ax25_frame_type_str(AX25_FRAME_UI),
                                    ax25_frame_type_str(AX25_FRAME_I)));
    TEST_ASSERT_NOT_EQUAL(0, strcmp(ax25_frame_type_str(AX25_FRAME_S),
                                    ax25_frame_type_str(AX25_FRAME_U)));
    TEST_ASSERT_NOT_EQUAL(0, strcmp(ax25_frame_type_str(AX25_FRAME_I),
                                    ax25_frame_type_str(AX25_FRAME_S)));
}

// ---------------------------------------------------------------------------
// ax25_conn_state_str
// ---------------------------------------------------------------------------

TEST_CASE("AX25Print: conn_state_str returns non-NULL non-empty string for every state", "[ax25_print]")
{
    const ax25_conn_state_t states[] = {
        AX25_CONN_STATE_DISCONNECTED,
        AX25_CONN_STATE_AWAITING_CONNECTION,
        AX25_CONN_STATE_CONNECTED,
        AX25_CONN_STATE_AWAITING_RELEASE,
        AX25_CONN_STATE_TIMER_RECOVERY,
    };
    for (size_t i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
        const char *s = ax25_conn_state_str(states[i]);
        TEST_ASSERT_NOT_NULL(s);
        TEST_ASSERT_GREATER_THAN(0, (int)strlen(s));
    }
}

TEST_CASE("AX25Print: conn_state_str returns distinct strings for distinct states", "[ax25_print]")
{
    TEST_ASSERT_NOT_EQUAL(0, strcmp(ax25_conn_state_str(AX25_CONN_STATE_DISCONNECTED),
                                    ax25_conn_state_str(AX25_CONN_STATE_CONNECTED)));
    TEST_ASSERT_NOT_EQUAL(0, strcmp(ax25_conn_state_str(AX25_CONN_STATE_AWAITING_CONNECTION),
                                    ax25_conn_state_str(AX25_CONN_STATE_AWAITING_RELEASE)));
}

// ---------------------------------------------------------------------------
// ax25_print_frame — smoke tests (output goes to log; no assertion on text)
// ---------------------------------------------------------------------------

TEST_CASE("AX25Print: print_frame with NULL frame does not crash", "[ax25_print]")
{
    ax25_print_frame("TEST", NULL);
    /* Reaching this line means the null-guard worked. */
}

TEST_CASE("AX25Print: print_frame with UI frame does not crash", "[ax25_print]")
{
    ax25_address_t src, dst;
    ax25_address_from_string("N0CALL-0", &src);
    ax25_address_from_string("APRS-0",   &dst);

    ax25_frame_t f;
    ax25_frame_init(&f);
    ax25_address_copy(&f.source,      &src);
    ax25_address_copy(&f.destination, &dst);
    f.type      = AX25_FRAME_UI;
    f.control   = AX25_CTRL_UI;
    f.pid       = AX25_PID_NONE;
    const char *info = "Hello World";
    memcpy(f.payload, info, strlen(info));
    f.payload_len = strlen(info);

    ax25_print_frame(AX25_PRINT_LABEL_RX, &f);
}

// ---------------------------------------------------------------------------
// ax25_print_buffer — smoke tests
// ---------------------------------------------------------------------------

TEST_CASE("AX25Print: print_buffer with NULL data does not crash", "[ax25_print]")
{
    ax25_print_buffer("TEST", NULL, 0);
}

TEST_CASE("AX25Print: print_buffer with a valid encoded frame does not crash", "[ax25_print]")
{
    ax25_address_t src, dst;
    ax25_address_from_string("N0CALL-0", &src);
    ax25_address_from_string("NOCALL-0", &dst);

    ax25_frame_t f;
    ax25_frame_init(&f);
    ax25_address_copy(&f.source,      &src);
    ax25_address_copy(&f.destination, &dst);
    f.type    = AX25_FRAME_UI;
    f.control = AX25_CTRL_UI;
    f.pid     = AX25_PID_NONE;

    ax25_buffer_t raw;
    memset(&raw, 0, sizeof(raw));
    esp_err_t err = ax25_frame_build(&f, &raw);
    TEST_ASSERT_EQUAL(ESP_OK, err);

    ax25_print_buffer(AX25_PRINT_LABEL_TX, raw.data, raw.len);
}
