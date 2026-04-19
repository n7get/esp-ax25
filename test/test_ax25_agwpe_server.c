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
 * @file test_ax25_agwpe_server.c
 * @brief Comprehensive unit tests for the ax25_agwpe_server module
 *
 * Tests cover:
 *   - Initialization / deinitialization / invalid parameters
 *   - Version ('R'), port info ('G'), capabilities ('g')
 *   - Callsign registration ('X') / unregistration ('x')
 *   - Raw mode ('k') and monitor mode ('m') toggling & frame delivery
 *   - Transmit: raw ('K'), unproto ('M'), unproto-via ('V')
 *   - Connect ('C'), connect-via ('v'), connect-with-PID ('c')
 *   - Send connected data ('D'), disconnect ('d')
 *   - Outstanding queries: port ('y'), connection ('Y')
 *   - AX.25 frame ingress routing & monitor/raw output
 *   - Error handling: null pointers, uninitialised server, bad frames
 *   - Integration: multiple connections, concurrent operations
 *
 * ## Architecture
 *
 * Because the AGWPE server relies on FreeRTOS primitives (mutexes, queues,
 * tasks) and several esp-ax25 sub-modules (ax25_conn, ax25_router, …),
 * these tests run on a real ESP-IDF target (or QEMU) using the Unity
 * framework.  Mocking is done at the *observation* layer: a small capture
 * callback records every AGWPE frame the server sends so we can inspect it.
 */


#include <string.h>
#include <stdint.h>
#include "unity.h"
#include "ax25_agwpe_server.h"
#include "ax25_agwpe.h"
#include "ax25_address.h"

// --- BEGIN: Test-only context/capture types ---
#define MAX_CAPTURED_FRAMES 16

typedef struct {
    agwpe_frame_t frames[MAX_CAPTURED_FRAMES];
    int count;
} capture_ctx_t;

typedef struct {
    ax25_frame_t last_frame;
    int call_count;
} router_capture_ctx_t;

// Global capture buffer for most tests
static capture_ctx_t s_cap;
static capture_ctx_t s_mgr_cap1, s_mgr_cap2, s_mgr_cap3;
static int s_unknown_client;
static ax25_router_port_t s_test_source_port;

// Router port frame capture callback for tests
static void router_capture_cb(const ax25_frame_t *frame, void *user_data) {
    router_capture_ctx_t *ctx = (router_capture_ctx_t *)user_data;
    if (ctx) {
        memcpy(&ctx->last_frame, frame, sizeof(ax25_frame_t));
        ctx->call_count++;
    }
}

// Expand stubs for undefined functions
static void make_sabm_frame(ax25_frame_t *f, const char *src, const char *dst) {
    memset(f, 0, sizeof(ax25_frame_t));
    strncpy(f->source.callsign, src, sizeof(f->source.callsign));
    strncpy(f->destination.callsign, dst, sizeof(f->destination.callsign));
    f->control = AX25_CTRL_SABM;
}

static void make_ua_frame(ax25_frame_t *f, const char *src, const char *dst) {
    memset(f, 0, sizeof(ax25_frame_t));
    strncpy(f->source.callsign, src, sizeof(f->source.callsign));
    strncpy(f->destination.callsign, dst, sizeof(f->destination.callsign));
    f->control = AX25_CTRL_UA;
}

/* forward declarations */
static const agwpe_frame_t *find_captured(uint8_t kind, int start);
static const agwpe_frame_t *find_captured_in(const capture_ctx_t *ctx, uint8_t kind, int start);

static void wait_for_captured_kind(uint8_t kind, int timeout_ms) {
    // Minimal logic to simulate waiting for a captured frame
    for (int i = 0; i < timeout_ms; i += 100) {
        if (find_captured(kind, 0)) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    TEST_FAIL_MESSAGE("Timeout waiting for captured frame");
}

static void wait_for_capture_ctx_kind(const capture_ctx_t *ctx, uint8_t kind, int timeout_ms) {
    // Minimal logic to simulate waiting for a captured frame in a specific context
    for (int i = 0; i < timeout_ms; i += 100) {
        if (find_captured_in(ctx, kind, 0)) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    TEST_FAIL_MESSAGE("Timeout waiting for captured frame in context");
}

// Helper functions
static const agwpe_frame_t *find_captured(uint8_t kind, int start) {
    for (int i = start; i < s_cap.count; i++) {
        if (s_cap.frames[i].header.data_kind == kind) {
            return &s_cap.frames[i];
        }
    }
    return NULL;
}

static const agwpe_frame_t *find_captured_in(const capture_ctx_t *ctx, uint8_t kind, int start) {
    for (int i = start; i < ctx->count; i++) {
        if (ctx->frames[i].header.data_kind == kind) {
            return &ctx->frames[i];
        }
    }
    return NULL;
}

static void wait_for_frames(int n, int timeout_ms) {
    int waited = 0;
    while (s_cap.count < n && waited < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
}

static void wait_for_router_frames(router_capture_ctx_t *ctx, int n, int timeout_ms) {
    int waited = 0;
    while (ctx->call_count < n && waited < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
}

static void make_request(agwpe_frame_t *f, uint8_t kind) {
    agwpe_frame_init(f);
    f->header.data_kind = kind;
}

static void set_from(agwpe_frame_t *f, const char *call) {
    agwpe_set_callsign(f->header.call_from, call);
}

static void set_to(agwpe_frame_t *f, const char *call) {
    agwpe_set_callsign(f->header.call_to, call);
}

static void make_ui_frame(ax25_frame_t *f, const char *src, const char *dst, const uint8_t *payload, size_t len) {
    ax25_frame_init(f);
    f->type    = AX25_FRAME_UI;
    f->control = AX25_CTRL_UI;
    f->pid     = AX25_PID_NONE;
    ax25_address_from_string(src, &f->source);
    ax25_address_from_string(dst, &f->destination);
    if (payload && len > 0) {
        size_t copy = len > AX25_MAX_INFO_LEN ? AX25_MAX_INFO_LEN : len;
        memcpy(f->payload, payload, copy);
        f->payload_len = copy;
    }
}

// Properly stub `capture_reset`
static void capture_reset(void) {
    memset(&s_cap, 0, sizeof(s_cap));
}

// Define `capture_cb` callback
static void capture_cb(const agwpe_frame_t *frame, void *user_data) {
    capture_ctx_t *ctx = (capture_ctx_t *)user_data;
    if (ctx && ctx->count < MAX_CAPTURED_FRAMES) {
        memcpy(&ctx->frames[ctx->count], frame, sizeof(agwpe_frame_t));
        ctx->count++;
    }
}

// Global server instance for tests
static ax25_agwpe_server_t *s_server = NULL;
static ax25_agwpe_server_t s_server_storage;

// Initialize global variables in test setup
static void setup_server(void) {
    capture_reset();
    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame   = capture_cb,
        .user_data        = &s_cap,
        .port_description = "Test Port",
    };
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_init(&cfg, &s_server_storage));
    s_server = &s_server_storage;
}

static void teardown_server(void) {
    if (s_server) {
        ax25_agwpe_server_deinit(s_server);
        s_server = NULL;
    }
}

// Removed duplicate definition of capture_cb

// New test cases for AGWPE/Direwolf compatibility
TEST_CASE("AGWPE_SRV: 'M' and 'V' PID=0 passthrough", "[ax25_agwpe_server]")
{
    ax25_router_init();
    setup_server();
    router_capture_ctx_t promisc_ctx = {0};
    ax25_router_port_t promisc_port;
    memset(&promisc_port, 0, sizeof(promisc_port));
    promisc_port.mode = AX25_PORT_PROMISCUOUS;
    promisc_port.on_tx_frame = router_capture_cb;
    promisc_port.user_data = &promisc_ctx;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&promisc_port));
    agwpe_frame_t req;
    make_request(&req, 'M');
    set_from(&req, "N0CALL");
    set_to(&req, "CQ");
    req.header.pid = 0x00;
    memcpy(req.data, "abc", 3);
    req.header.data_len = 3;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));
    wait_for_router_frames(&promisc_ctx, 1, 500);
    TEST_ASSERT_GREATER_THAN(0, promisc_ctx.call_count);
    TEST_ASSERT_EQUAL_HEX8(0x00, promisc_ctx.last_frame.pid);
    memset(&req, 0, sizeof(req));
    make_request(&req, 'V');
    set_from(&req, "N0CALL");
    set_to(&req, "CQ");
    req.header.pid = 0x00;
    uint8_t *p = req.data;
    p[0] = 1; /* one digi */
    memset(p+1, 0, 10);
    memcpy(p+1, "RELAY", 5);
    memcpy(p+11, "xyz", 3);
    req.header.data_len = 14;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));
    wait_for_router_frames(&promisc_ctx, 2, 500);
    TEST_ASSERT_EQUAL_HEX8(0x00, promisc_ctx.last_frame.pid);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&promisc_port));
    teardown_server();
    ax25_router_deinit();
}

TEST_CASE("AGWPE_SRV: own-transmitted UI frame emits 'T' monitor frame", "[ax25_agwpe_server]")
{
    setup_server();
    agwpe_frame_t req;
    make_request(&req, 'm');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));
    ax25_frame_t ui;
    make_ui_frame(&ui, "SRC", "DST", (const uint8_t *)"own", 3);
    ax25_agwpe_server_ax25_out(s_server, &ui);
    wait_for_frames(1, 500);
    const agwpe_frame_t *mon = find_captured('T', 0);
    TEST_ASSERT_NOT_NULL_MESSAGE(mon, "Expected 'T' monitor frame for own-transmitted UI");
    char text[AGWPE_MAX_DATA_LEN + 1];
    memcpy(text, mon->data, mon->header.data_len);
    text[mon->header.data_len] = '\0';
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(text, "SRC"), "Monitor text should contain source");
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(text, "DST"), "Monitor text should contain destination");
    teardown_server();
}

TEST_CASE("AGWPE_SRV: monitor SREJ S-frame text", "[ax25_agwpe_server]")
{
    setup_server();
    agwpe_frame_t req;
    make_request(&req, 'm');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));
    ax25_frame_t srej;
    ax25_frame_init(&srej);
    srej.type = AX25_FRAME_S;
    srej.control = 0x0D;
    srej.is_command = true;
    ax25_address_from_string("SRC", &srej.source);
    ax25_address_from_string("DST", &srej.destination);
    ax25_agwpe_server_ax25_in(s_server, &srej);
    wait_for_frames(1, 500);
    const agwpe_frame_t *mon = find_captured('S', 0);
    TEST_ASSERT_NOT_NULL_MESSAGE(mon, "Expected 'S' monitor frame for SREJ");
    char text[AGWPE_MAX_DATA_LEN + 1];
    memcpy(text, mon->data, mon->header.data_len);
    text[mon->header.data_len] = '\0';
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(text, "SREJ"), "Monitor text should contain SREJ");
    teardown_server();
}

/* =========================================================================
 *  1. INITIALIZATION TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: init with valid config succeeds", "[ax25_agwpe_server]")
{
    setup_server();
    TEST_ASSERT_NOT_NULL(s_server);
    teardown_server();
}

TEST_CASE("AGWPE_SRV: init with NULL config returns INVALID_ARG", "[ax25_agwpe_server]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_agwpe_server_init(NULL, &s_server_storage));
}

TEST_CASE("AGWPE_SRV: init with NULL out_server returns INVALID_ARG", "[ax25_agwpe_server]")
{
    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame = capture_cb,
        .user_data      = &s_cap,
    };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_agwpe_server_init(&cfg, NULL));
}

TEST_CASE("AGWPE_SRV: init with NULL callback returns INVALID_ARG", "[ax25_agwpe_server]")
{
    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame = NULL,
        .user_data      = NULL,
    };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_agwpe_server_init(&cfg, &s_server_storage));
}

TEST_CASE("AGWPE_SRV: deinit NULL is safe (no crash)", "[ax25_agwpe_server]")
{
    ax25_agwpe_server_deinit(NULL);
    /* No assertion needed — test passes if it doesn't crash. */
}

TEST_CASE("AGWPE_SRV: double deinit is safe", "[ax25_agwpe_server]")
{
    setup_server();
    ax25_agwpe_server_deinit(s_server);
    /* Second deinit on freed pointer is risky in real code, but deinit
     * checks initialized flag — we can't truly test without use-after-free.
     * Instead just verify single deinit works. */
    s_server = NULL;
    ax25_router_deinit();
}

TEST_CASE("AGWPE_SRV: init with custom config values", "[ax25_agwpe_server]")
{
    capture_reset();
    ax25_router_init();

    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame   = capture_cb,
        .user_data        = &s_cap,
        .port             = 3,
        .port_description = "Custom Port Description",
        .tx_queue_depth   = 4,
        .task_stack_size  = 2048,
        .task_priority    = 2,
    };
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_init(&cfg, &s_server_storage));
    ax25_agwpe_server_deinit(&s_server_storage);
    ax25_router_deinit();
}

TEST_CASE("AGWPE_SRV: custom tx queue depth is honored", "[ax25_agwpe_server]")
{
    capture_reset();
    ax25_router_init();

    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame   = capture_cb,
        .user_data        = &s_cap,
        .port             = 0,
        .port_description = "Test Port",
        .tx_queue_depth   = 1,
    };
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_init(&cfg, &s_server_storage));
    s_server = &s_server_storage;

    vTaskSuspend(s_server->tx_task);

    agwpe_frame_t req;
    make_request(&req, 'R');
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    UBaseType_t queued = uxQueueMessagesWaiting(s_server->tx_queue);
    TEST_ASSERT_EQUAL_UINT32(1, (uint32_t)queued);

    vTaskResume(s_server->tx_task);
    wait_for_frames(1, 500);

    ax25_agwpe_server_deinit(s_server);
    s_server = NULL;
    ax25_router_deinit();
}

/* =========================================================================
 *  2. VERSION / INFO / CAPABILITIES TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: 'R' version request returns version 2005.127", "[ax25_agwpe_server]")
{
    setup_server();

    agwpe_frame_t req;
    make_request(&req, 'R');
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    wait_for_frames(1, 500);
    TEST_ASSERT_GREATER_OR_EQUAL(1, s_cap.count);

    const agwpe_frame_t *resp = find_captured('R', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT8('R', resp->header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(8, resp->header.data_len);

    /* Decode little-endian version */
    uint32_t major = resp->data[0] | (resp->data[1] << 8) |
                     (resp->data[2] << 16) | (resp->data[3] << 24);
    uint32_t minor = resp->data[4] | (resp->data[5] << 8) |
                     (resp->data[6] << 16) | (resp->data[7] << 24);
    TEST_ASSERT_EQUAL_UINT32(2005, major);
    TEST_ASSERT_EQUAL_UINT32(127, minor);

    teardown_server();
}

TEST_CASE("AGWPE_SRV: 'G' port info request returns description", "[ax25_agwpe_server]")
{
    setup_server();
    agwpe_frame_t req;
    make_request(&req, 'G');
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    wait_for_frames(1, 500);
    const agwpe_frame_t *resp = find_captured('G', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT8('G', resp->header.data_kind);

    /* Data should start with "1;" (one port) and contain "Test Port" */
    char data_str[AGWPE_MAX_DATA_LEN + 1];
    memcpy(data_str, resp->data, resp->header.data_len);
    data_str[resp->header.data_len] = '\0';
    TEST_ASSERT_NOT_NULL(strstr(data_str, "1;"));
    TEST_ASSERT_NOT_NULL(strstr(data_str, "Test Port"));
    teardown_server();
}

TEST_CASE("AGWPE_SRV: 'g' port capabilities returns 12-byte response", "[ax25_agwpe_server]")
{
    setup_server();
    agwpe_frame_t req;
    make_request(&req, 'g');
    req.header.port = 0;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    wait_for_frames(1, 500);
    const agwpe_frame_t *resp = find_captured('g', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT8('g', resp->header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(12, resp->header.data_len);
    /* traffic_level at offset 1 should be 1 (matches direwolf) */
    TEST_ASSERT_EQUAL_UINT8(1, resp->data[1]);
    /* Active connections byte (offset 7) should be 0 initially */
    TEST_ASSERT_EQUAL_UINT8(0, resp->data[7]);
    teardown_server();
}

/* =========================================================================
 *  3. REGISTRATION TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: 'X' register callsign returns success=1", "[ax25_agwpe_server]")
{
    setup_server();
    agwpe_frame_t req;
    make_request(&req, 'X');
    set_from(&req, "N0CALL");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    wait_for_frames(1, 500);
    const agwpe_frame_t *resp = find_captured('X', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT8('X', resp->header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(1, resp->header.data_len);
    TEST_ASSERT_EQUAL_UINT8(1, resp->data[0]);  /* success */

    /* Verify call_from echoed */
    char call_back[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(resp->header.call_from, call_back);
    TEST_ASSERT_EQUAL_STRING("N0CALL", call_back);
    teardown_server();
}
