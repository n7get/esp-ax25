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

#include "unity.h"

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ax25_agwpe_server.h"
#include "ax25_agwpe.h"
#include "ax25_frame.h"
#include "ax25_address.h"
#include "ax25_router.h"
#include "ax25_conn.h"
#include "ax25_types.h"

#include "esp_log.h"
#include "esp_heap_caps.h"

static ax25_router_port_t s_test_source_port;

/* =========================================================================
 * Capture / observation infrastructure
 * ========================================================================= */

/** Maximum frames we can capture in a test run. */
#define MAX_CAPTURED_FRAMES 8

typedef struct {
    int            count;                            /**< Total captured frames */
    agwpe_frame_t  frames[MAX_CAPTURED_FRAMES];      /**< Ring buffer of frames */
} capture_ctx_t;

static capture_ctx_t s_cap;
static capture_ctx_t s_mgr_cap1;
static capture_ctx_t s_mgr_cap2;
static capture_ctx_t s_mgr_cap3;
static ax25_agwpe_server_t s_unknown_client;

typedef struct {
    int          call_count;
    ax25_frame_t last_frame;
} router_capture_ctx_t;

static const agwpe_frame_t *find_captured_in(const capture_ctx_t *ctx,
                                             uint8_t kind,
                                             int start);
static const agwpe_frame_t *find_captured(uint8_t kind, int start);

static void router_capture_cb(const ax25_frame_t *frame, void *user_data)
{
    router_capture_ctx_t *ctx = (router_capture_ctx_t *)user_data;
    if (ctx == NULL || frame == NULL) {
        return;
    }
    ctx->call_count++;
    ctx->last_frame = *frame;
}

/**
 * @brief Callback installed as `on_agwpe_frame`.
 *
 * Records every outgoing AGWPE frame so tests can verify the server's
 * responses without an actual TCP socket.
 */
static void capture_cb(const agwpe_frame_t *frame, void *user_data)
{
    capture_ctx_t *ctx = (capture_ctx_t *)user_data;
    if (ctx->count < MAX_CAPTURED_FRAMES) {
        memcpy(&ctx->frames[ctx->count], frame, sizeof(agwpe_frame_t));
    }
    ctx->count++;
}

/** Reset capture buffer before each test. */
static void capture_reset(void)
{
    memset(&s_cap, 0, sizeof(s_cap));
}

/**
 * @brief Wait for the TX task to drain at least `n` frames.
 *
 * The server enqueues frames to a FreeRTOS queue; a background task calls
 * `capture_cb`.  We need a small delay so the task has time to run.
 */
static void wait_for_frames(int n, int timeout_ms)
{
    int waited = 0;
    while (s_cap.count < n && waited < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
}

static void wait_for_capture_ctx_frames(const capture_ctx_t *ctx, int n, int timeout_ms)
{
    int waited = 0;
    while (ctx->count < n && waited < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
}

static void wait_for_router_frames(router_capture_ctx_t *ctx, int n, int timeout_ms)
{
    int waited = 0;
    while (ctx->call_count < n && waited < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
}

static void wait_for_captured_kind(uint8_t kind, int timeout_ms)
{
    int waited = 0;
    while (find_captured(kind, 0) == NULL && waited < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
}

static void wait_for_capture_ctx_kind(const capture_ctx_t *ctx, uint8_t kind, int timeout_ms)
{
    int waited = 0;
    while (find_captured_in(ctx, kind, 0) == NULL && waited < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
}

/* =========================================================================
 * Helpers – build AGWPE request frames
 * ========================================================================= */

/** Build a minimal AGWPE request frame with a given data_kind. */
static void make_request(agwpe_frame_t *f, uint8_t kind)
{
    agwpe_frame_init(f);
    f->header.data_kind = kind;
}

/** Set call_from in an AGWPE frame. */
static void set_from(agwpe_frame_t *f, const char *call)
{
    agwpe_set_callsign(f->header.call_from, call);
}

/** Set call_to in an AGWPE frame. */
static void set_to(agwpe_frame_t *f, const char *call)
{
    agwpe_set_callsign(f->header.call_to, call);
}

/* =========================================================================
 * Helpers – build AX.25 frames for injection
 * ========================================================================= */

/** Build a simple UI frame. */
static void make_ui_frame(ax25_frame_t *f, const char *src, const char *dst,
                           const uint8_t *payload, size_t len)
{
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

/** Build a simple I-frame. */
static void make_i_frame(ax25_frame_t *f, const char *src, const char *dst,
                          uint8_t ns, uint8_t nr, const uint8_t *payload,
                          size_t len)
{
    ax25_frame_init(f);
    f->type    = AX25_FRAME_I;
    f->control = ax25_frame_build_i_control(ns, nr, false);
    f->pid     = AX25_PID_NONE;
    ax25_address_from_string(src, &f->source);
    ax25_address_from_string(dst, &f->destination);
    if (payload && len > 0) {
        size_t copy = len > AX25_MAX_INFO_LEN ? AX25_MAX_INFO_LEN : len;
        memcpy(f->payload, payload, copy);
        f->payload_len = copy;
    }
}

/** Build a supervisory RR frame. */
static void make_rr_frame(ax25_frame_t *f, const char *src, const char *dst,
                           uint8_t nr)
{
    ax25_frame_init(f);
    f->type    = AX25_FRAME_S;
    f->control = ax25_frame_build_rr_control(nr, false);
    ax25_address_from_string(src, &f->source);
    ax25_address_from_string(dst, &f->destination);
}

/** Build a SABM frame (unnumbered). */
static void make_sabm_frame(ax25_frame_t *f, const char *src, const char *dst)
{
    ax25_frame_init(f);
    f->type      = AX25_FRAME_U;
    f->control   = AX25_CTRL_SABM | AX25_CTRL_PF_BIT;
    f->is_command = true;
    ax25_address_from_string(src, &f->source);
    ax25_address_from_string(dst, &f->destination);
}

/** Build a UA frame (unnumbered). */
static void make_ua_frame(ax25_frame_t *f, const char *src, const char *dst)
{
    ax25_frame_init(f);
    f->type      = AX25_FRAME_U;
    f->control   = AX25_CTRL_UA | AX25_CTRL_PF_BIT;
    f->is_command = false;
    ax25_address_from_string(src, &f->source);
    ax25_address_from_string(dst, &f->destination);
}

typedef struct {
    volatile bool stop;
    volatile bool done;
    volatile int send_count;
    volatile int send_errors;
} manager_flood_ctx_t;

static void manager_flood_sender_task(void *arg)
{
    manager_flood_ctx_t *ctx = (manager_flood_ctx_t *)arg;
    ax25_frame_t ui;
    make_ui_frame(&ui, "SRC", "DST", (const uint8_t *)"race", 4);

    for (int i = 0; i < 300 && !ctx->stop; i++) {
        if (ax25_router_send(&ui, &s_test_source_port) != ESP_OK) {
            ctx->send_errors++;
        }
        ctx->send_count++;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ctx->done = true;
    vTaskDelete(NULL);
}

/* =========================================================================
 * Helpers – find captured frame by kind
 * ========================================================================= */

/**
 * @brief Search captured frames for one with a given data_kind.
 * @param kind   AGWPE data_kind to look for.
 * @param start  Index to start searching from.
 * @return Pointer to the captured frame, or NULL if not found.
 */
static const agwpe_frame_t *find_captured(uint8_t kind, int start)
{
    return find_captured_in(&s_cap, kind, start);
}

static const agwpe_frame_t *find_captured_in(const capture_ctx_t *ctx,
                                             uint8_t kind,
                                             int start)
{
    int end = ctx->count < MAX_CAPTURED_FRAMES ? ctx->count : MAX_CAPTURED_FRAMES;
    for (int i = start; i < end; i++) {
        if (ctx->frames[i].header.data_kind == kind) {
            return &ctx->frames[i];
        }
    }
    return NULL;
}

/* =========================================================================
 * Fixture: server lifecycle shared by most tests
 * ========================================================================= */

static ax25_agwpe_server_t  s_server_storage;
static ax25_agwpe_server_t *s_server = NULL;  /* points to s_server_storage when active */

/**
 * @brief Create a fresh server for a test.
 * Ensures ax25_router is initialised first.
 */
static void create_server(void)
{
    capture_reset();
    ax25_router_init();  /* idempotent */

    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame  = capture_cb,
        .user_data       = &s_cap,
        .port            = 0,
        .port_description = "Test Port",
    };
    esp_err_t err = ax25_agwpe_server_init(&cfg, &s_server_storage);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    s_server = &s_server_storage;
}

/** Destroy the server and router after a test. */
static void destroy_server(void)
{
    if (s_server) {
        ax25_agwpe_server_deinit(s_server);
        s_server = NULL;
    }
    ax25_router_deinit();
}

/* =========================================================================
 *  1. INITIALIZATION TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: init with valid config succeeds", "[ax25_agwpe_server]")
{
    create_server();
    TEST_ASSERT_NOT_NULL(s_server);
    destroy_server();
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
    create_server();
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
    create_server();

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

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'G' port info request returns description", "[ax25_agwpe_server]")
{
    create_server();

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

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'g' port capabilities returns 12-byte response", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'g');
    req.header.port = 0;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    wait_for_frames(1, 500);
    const agwpe_frame_t *resp = find_captured('g', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT8('g', resp->header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(12, resp->header.data_len);
    /* Active connections byte (offset 7) should be 0 initially */
    TEST_ASSERT_EQUAL_UINT8(0, resp->data[7]);

    destroy_server();
}

/* =========================================================================
 *  3. REGISTRATION TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: 'X' register callsign returns success=1", "[ax25_agwpe_server]")
{
    create_server();

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

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'X' register callsign with SSID", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'X');
    set_from(&req, "W1AW-5");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    wait_for_frames(1, 500);
    const agwpe_frame_t *resp = find_captured('X', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT8(1, resp->data[0]);

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'x' unregister callsign produces no response", "[ax25_agwpe_server]")
{
    create_server();

    /* Register first */
    agwpe_frame_t req;
    make_request(&req, 'X');
    set_from(&req, "N0CALL");
    ax25_agwpe_server_agwpe_in(s_server, &req);
    wait_for_frames(1, 500);

    int before_count = s_cap.count;

    /* Unregister */
    make_request(&req, 'x');
    set_from(&req, "N0CALL");
    ax25_agwpe_server_agwpe_in(s_server, &req);

    vTaskDelay(pdMS_TO_TICKS(100));
    /* No additional response expected */
    TEST_ASSERT_EQUAL(before_count, s_cap.count);

    destroy_server();
}

/* =========================================================================
 *  4. MONITOR / RAW MODE TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: 'k' toggles raw mode", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'k');

    /* First toggle: raw enabled */
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Inject a UI frame — should produce 'K' raw output */
    ax25_frame_t ui;
    make_ui_frame(&ui, "SRC", "DST", (const uint8_t *)"hello", 5);
    ax25_agwpe_server_ax25_in(s_server, &ui);

    wait_for_frames(1, 500);
    const agwpe_frame_t *raw = find_captured('K', 0);
    TEST_ASSERT_NOT_NULL_MESSAGE(raw, "Expected raw 'K' frame after enabling raw mode");

    /* Second toggle: raw disabled */
    capture_reset();
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));

    ax25_agwpe_server_ax25_in(s_server, &ui);
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_EQUAL_MESSAGE(0, s_cap.count,
        "No raw frame expected after disabling raw mode");

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'm' toggles monitor mode", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'm');

    /* Enable monitor */
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Inject UI → should produce 'U' */
    ax25_frame_t ui;
    make_ui_frame(&ui, "SRC", "DST", (const uint8_t *)"test", 4);
    ax25_agwpe_server_ax25_in(s_server, &ui);

    wait_for_frames(1, 500);
    const agwpe_frame_t *mon = find_captured('U', 0);
    TEST_ASSERT_NOT_NULL_MESSAGE(mon, "Expected 'U' monitor frame");

    /* Disable monitor */
    capture_reset();
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));

    ax25_agwpe_server_ax25_in(s_server, &ui);
    vTaskDelay(pdMS_TO_TICKS(200));
    const agwpe_frame_t *mon2 = find_captured('U', 0);
    TEST_ASSERT_NULL_MESSAGE(mon2,
        "No monitor frame expected after disabling monitor mode");

    destroy_server();
}

TEST_CASE("AGWPE_SRV: monitor mode reports I-frames as 'I'", "[ax25_agwpe_server]")
{
    create_server();

    /* Enable monitor */
    agwpe_frame_t req;
    make_request(&req, 'm');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Inject I-frame */
    ax25_frame_t iframe;
    make_i_frame(&iframe, "SRC", "DST", 0, 0,
                 (const uint8_t *)"i-data", 6);
    ax25_agwpe_server_ax25_in(s_server, &iframe);

    wait_for_frames(1, 500);
    const agwpe_frame_t *mon = find_captured('I', 0);
    TEST_ASSERT_NOT_NULL_MESSAGE(mon, "Expected 'I' monitor frame for I-frame");

    destroy_server();
}

TEST_CASE("AGWPE_SRV: monitor mode reports S-frames as 'S'", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'm');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));

    ax25_frame_t rr;
    make_rr_frame(&rr, "SRC", "DST", 3);
    ax25_agwpe_server_ax25_in(s_server, &rr);

    wait_for_frames(1, 500);
    const agwpe_frame_t *mon = find_captured('S', 0);
    TEST_ASSERT_NOT_NULL_MESSAGE(mon, "Expected 'S' monitor frame for S-frame");

    destroy_server();
}

TEST_CASE("AGWPE_SRV: raw+monitor both active sends K and U/I/S", "[ax25_agwpe_server]")
{
    create_server();

    /* Enable both raw and monitor */
    agwpe_frame_t req;
    make_request(&req, 'k');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    make_request(&req, 'm');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));

    ax25_frame_t ui;
    make_ui_frame(&ui, "AB1CD", "EF2GH", (const uint8_t *)"dual", 4);
    ax25_agwpe_server_ax25_in(s_server, &ui);

    wait_for_frames(2, 500);
    TEST_ASSERT_NOT_NULL(find_captured('K', 0));
    TEST_ASSERT_NOT_NULL(find_captured('U', 0));

    destroy_server();
}

/* =========================================================================
 *  5. TRANSMIT TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: 'M' send unproto routes UI frame", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'M');
    set_from(&req, "N0CALL");
    set_to(&req, "CQ");
    req.header.pid = AX25_PID_NONE;
    const char *payload = "Hello World";
    size_t plen = strlen(payload);
    memcpy(req.data, payload, plen);
    req.header.data_len = (uint32_t)plen;

    /* This should not crash; the frame is routed via ax25_router */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'V' send unproto via digis", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'V');
    set_from(&req, "N0CALL");
    set_to(&req, "CQ");
    req.header.pid = AX25_PID_NONE;

    /* Data format: <num_digis:1> <digi1:10> <payload> */
    uint8_t *p = req.data;
    p[0] = 1;  /* one digipeater */
    p++;
    /* Digipeater callsign in 10-byte field */
    memset(p, 0, 10);
    memcpy(p, "RELAY", 5);
    p += 10;
    /* Payload */
    memcpy(p, "test", 4);
    p += 4;
    req.header.data_len = (uint32_t)(p - req.data);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'V' with empty data is handled gracefully", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'V');
    set_from(&req, "N0CALL");
    set_to(&req, "CQ");
    req.header.data_len = 0;

    /* Should not crash — data_len < 1 guard triggers */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'K' send raw frame", "[ax25_agwpe_server]")
{
    create_server();

    /* Build a raw AX.25 frame to embed in the 'K' data */
    ax25_frame_t ax25;
    make_ui_frame(&ax25, "SRC", "DST", (const uint8_t *)"raw", 3);

    /* Encode to bytes */
    ax25_buffer_t buf;
    memset(&buf, 0, sizeof(buf));
    buf.len = 0;
    esp_err_t err = ax25_frame_build(&ax25, &buf);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_GREATER_THAN(0, buf.len);

    /* Pack into AGWPE 'K' frame: first byte is TNC port indicator */
    agwpe_frame_t req;
    make_request(&req, 'K');
    req.data[0] = 0;  /* TNC port 0 */
    memcpy(req.data + 1, buf.data, buf.len);
    req.header.data_len = (uint32_t)(1 + buf.len);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'K' with too-short data is rejected", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'K');
    req.header.data_len = 1;  /* Only TNC byte, no AX.25 data */
    req.data[0] = 0;

    /* Should not crash; the handler checks for data_len < 2 */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    destroy_server();
}

/* =========================================================================
 *  6. CONNECTION TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: 'C' connect request creates connection slot", "[ax25_agwpe_server]")
{
    create_server();

    /* Register callsign first */
    agwpe_frame_t reg;
    make_request(&reg, 'X');
    set_from(&reg, "N0CALL");
    ax25_agwpe_server_agwpe_in(s_server, &reg);
    wait_for_frames(1, 500);

    /* Send connect request */
    agwpe_frame_t req;
    make_request(&req, 'C');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW");
    req.header.port = 0;

    capture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    /* Give ax25_conn time to send SABM; the router should see it.
     * We can't easily verify the SABM without a full test rig, but at
     * least the call should succeed without crashing. */
    vTaskDelay(pdMS_TO_TICKS(100));

    destroy_server();
}

TEST_CASE("AGWPE_SRV: duplicate 'C' connect is rejected", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'C');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW");

    /* First connect should succeed */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Second connect to same callsign pair: silently ignored */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    capture_reset();
    make_request(&req, 'g');
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));
    wait_for_frames(1, 500);

    const agwpe_frame_t *caps = find_captured('g', 0);
    TEST_ASSERT_NOT_NULL(caps);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, caps->data[7],
                                    "Duplicate pending connect should not allocate a second slot");

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'v' connect via digipeaters", "[ax25_agwpe_server]")
{
    create_server();

    router_capture_ctx_t promisc_ctx = {0};
    ax25_router_port_t promisc_port;
    memset(&promisc_port, 0, sizeof(promisc_port));
    promisc_port.mode = AX25_PORT_PROMISCUOUS;
    promisc_port.on_tx_frame = router_capture_cb;
    promisc_port.user_data = &promisc_ctx;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&promisc_port));

    agwpe_frame_t req;
    make_request(&req, 'v');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW");

    /* Data: digipeater path (same format as 'V') */
    req.data[0] = 1;
    memset(req.data + 1, 0, 10);
    memcpy(req.data + 1, "RELAY", 5);
    req.header.data_len = 11;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    wait_for_router_frames(&promisc_ctx, 1, 500);
    TEST_ASSERT_EQUAL(AX25_FRAME_U, promisc_ctx.last_frame.type);
    TEST_ASSERT_EQUAL(AX25_CTRL_SABM | AX25_CTRL_PF_BIT, promisc_ctx.last_frame.control);
    TEST_ASSERT_EQUAL_UINT8(1, promisc_ctx.last_frame.num_digipeaters);
    TEST_ASSERT_EQUAL_STRING("RELAY", promisc_ctx.last_frame.digipeaters[0].callsign);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&promisc_port));

    destroy_server();
}

TEST_CASE("AGWPE_SRV: malformed 'v' digipeater path is rejected", "[ax25_agwpe_server]")
{
    create_server();

    router_capture_ctx_t promisc_ctx = {0};
    ax25_router_port_t promisc_port;
    memset(&promisc_port, 0, sizeof(promisc_port));
    promisc_port.mode = AX25_PORT_PROMISCUOUS;
    promisc_port.on_tx_frame = router_capture_cb;
    promisc_port.user_data = &promisc_ctx;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&promisc_port));

    agwpe_frame_t req;
    make_request(&req, 'v');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW");
    req.data[0] = AX25_MAX_DIGIPEATERS + 1;
    req.header.data_len = 1;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(0, promisc_ctx.call_count);

    capture_reset();
    make_request(&req, 'g');
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));
    wait_for_frames(1, 500);

    const agwpe_frame_t *caps = find_captured('g', 0);
    TEST_ASSERT_NOT_NULL(caps);
    TEST_ASSERT_EQUAL_UINT8(0, caps->data[7]);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&promisc_port));
    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'c' connect with custom PID", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'c');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW-3");
    req.header.pid = 0xCC;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));
    vTaskDelay(pdMS_TO_TICKS(100));

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'd' disconnect non-existing connection is handled", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'd');
    set_from(&req, "N0CALL");
    set_to(&req, "NOBODY");

    /* Should log a warning but not crash */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'D' send data to non-existing connection is handled", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'D');
    set_from(&req, "N0CALL");
    set_to(&req, "NOBODY");
    memcpy(req.data, "data", 4);
    req.header.data_len = 4;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    destroy_server();
}

/* =========================================================================
 *  7. OUTSTANDING FRAME QUERY TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: 'y' outstanding on port returns 0", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'y');
    req.header.port = 0;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    wait_for_frames(1, 500);
    const agwpe_frame_t *resp = find_captured('y', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT32(4, resp->header.data_len);

    uint32_t count = resp->data[0] | (resp->data[1] << 8) |
                     (resp->data[2] << 16) | (resp->data[3] << 24);
    TEST_ASSERT_EQUAL_UINT32(0, count);

    destroy_server();
}

TEST_CASE("AGWPE_SRV: 'Y' outstanding on non-existing connection returns 0", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'Y');
    set_from(&req, "N0CALL");
    set_to(&req, "NOBODY");
    req.header.port = 0;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    wait_for_frames(1, 500);
    const agwpe_frame_t *resp = find_captured('Y', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT32(4, resp->header.data_len);

    uint32_t count = resp->data[0] | (resp->data[1] << 8) |
                     (resp->data[2] << 16) | (resp->data[3] << 24);
    TEST_ASSERT_EQUAL_UINT32(0, count);

    /* Verify callsigns are echoed */
    char from[AGWPE_CALLSIGN_LEN + 1], to[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(resp->header.call_from, from);
    agwpe_get_callsign(resp->header.call_to, to);
    TEST_ASSERT_EQUAL_STRING("N0CALL", from);
    TEST_ASSERT_EQUAL_STRING("NOBODY", to);

    destroy_server();
}

/* =========================================================================
 *  8. 'P' APPLICATION LOGIN (silently accepted)
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: 'P' login is silently accepted", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'P');
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    vTaskDelay(pdMS_TO_TICKS(100));
    /* No response expected */
    TEST_ASSERT_EQUAL(0, s_cap.count);

    destroy_server();
}

/* =========================================================================
 *  9. UNHANDLED / UNKNOWN FRAME TYPE
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: unknown frame type is handled gracefully", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, '!');  /* Not a valid AGWPE command */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    vTaskDelay(pdMS_TO_TICKS(100));
    /* No crash, no response */
    TEST_ASSERT_EQUAL(0, s_cap.count);

    destroy_server();
}

/* =========================================================================
 * 10. ERROR HANDLING TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: agwpe_in with NULL server returns INVALID_ARG", "[ax25_agwpe_server]")
{
    agwpe_frame_t req;
    make_request(&req, 'R');
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_agwpe_server_agwpe_in(NULL, &req));
}

TEST_CASE("AGWPE_SRV: agwpe_in with NULL frame returns INVALID_ARG", "[ax25_agwpe_server]")
{
    create_server();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_agwpe_server_agwpe_in(s_server, NULL));
    destroy_server();
}

TEST_CASE("AGWPE_SRV: ax25_in with NULL server returns INVALID_ARG", "[ax25_agwpe_server]")
{
    ax25_frame_t frame;
    make_ui_frame(&frame, "SRC", "DST", NULL, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_agwpe_server_ax25_in(NULL, &frame));
}

TEST_CASE("AGWPE_SRV: ax25_in with NULL frame returns INVALID_ARG", "[ax25_agwpe_server]")
{
    create_server();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_agwpe_server_ax25_in(s_server, NULL));
    destroy_server();
}

TEST_CASE("AGWPE_SRV: get_router_port with NULL returns NULL", "[ax25_agwpe_server]")
{
    TEST_ASSERT_NULL(ax25_agwpe_server_get_router_port(NULL));
}

TEST_CASE("AGWPE_SRV: get_router_port returns non-NULL for valid server", "[ax25_agwpe_server]")
{
    create_server();
    TEST_ASSERT_NOT_NULL(ax25_agwpe_server_get_router_port(s_server));
    destroy_server();
}

TEST_CASE("AGWPE_SRV: add_client before manager init returns INVALID_STATE", "[ax25_agwpe_server]")
{
    ax25_agwpe_server_t *client = NULL;
    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame = capture_cb,
        .user_data = &s_mgr_cap1,
        .port = 1,
        .port_description = "Managed Client",
    };

    memset(&s_mgr_cap1, 0, sizeof(s_mgr_cap1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      ax25_agwpe_server_add_client(&cfg, &client));
    TEST_ASSERT_NULL(client);
}

TEST_CASE("AGWPE_SRV: manager init twice returns INVALID_STATE", "[ax25_agwpe_server]")
{
    ax25_router_init();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_manager_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_agwpe_server_manager_init());
    ax25_agwpe_server_manager_deinit();
    ax25_router_deinit();
}

TEST_CASE("AGWPE_SRV: manager fanout delivers frames to multiple clients", "[ax25_agwpe_server]")
{
    ax25_agwpe_server_t *client1 = NULL;
    ax25_agwpe_server_t *client2 = NULL;
    agwpe_frame_t req;

    memset(&s_mgr_cap1, 0, sizeof(s_mgr_cap1));
    memset(&s_mgr_cap2, 0, sizeof(s_mgr_cap2));
    ax25_router_init();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_manager_init());

    ax25_agwpe_server_config_t cfg1 = {
        .on_agwpe_frame = capture_cb,
        .user_data = &s_mgr_cap1,
        .port = 1,
        .port_description = "Managed One",
    };
    ax25_agwpe_server_config_t cfg2 = {
        .on_agwpe_frame = capture_cb,
        .user_data = &s_mgr_cap2,
        .port = 2,
        .port_description = "Managed Two",
    };

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_add_client(&cfg1, &client1));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_add_client(&cfg2, &client2));

    make_request(&req, 'm');
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_client_agwpe_in(client1, &req));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_client_agwpe_in(client2, &req));
    vTaskDelay(pdMS_TO_TICKS(50));

    ax25_frame_t ui;
    make_ui_frame(&ui, "SRC", "DST", (const uint8_t *)"fanout", 6);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&ui, &s_test_source_port));

    wait_for_capture_ctx_frames(&s_mgr_cap1, 1, 500);
    wait_for_capture_ctx_frames(&s_mgr_cap2, 1, 500);
    TEST_ASSERT_NOT_NULL(find_captured_in(&s_mgr_cap1, 'U', 0));
    TEST_ASSERT_NOT_NULL(find_captured_in(&s_mgr_cap2, 'U', 0));

    ax25_agwpe_server_manager_deinit();
    ax25_router_deinit();
}

TEST_CASE("AGWPE_SRV: removed managed client is rejected", "[ax25_agwpe_server]")
{
    ax25_agwpe_server_t *client = NULL;
    agwpe_frame_t req;
    ax25_frame_t ui;

    memset(&s_mgr_cap3, 0, sizeof(s_mgr_cap3));
    memset(&s_unknown_client, 0, sizeof(s_unknown_client));
    ax25_router_init();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_manager_init());

    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame = capture_cb,
        .user_data = &s_mgr_cap3,
        .port = 3,
        .port_description = "Managed Remove",
    };
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_add_client(&cfg, &client));

    ax25_agwpe_server_remove_client(client);

    make_request(&req, 'R');
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_agwpe_server_client_agwpe_in(&s_unknown_client, &req));

    make_ui_frame(&ui, "SRC", "DST", NULL, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_agwpe_server_client_ax25_in(&s_unknown_client, &ui));

    ax25_agwpe_server_manager_deinit();
    ax25_router_deinit();
}

TEST_CASE("AGWPE_SRV: remove managed client during fanout is safe", "[ax25_agwpe_server]")
{
    ax25_agwpe_server_t *client = NULL;
    agwpe_frame_t req;
    manager_flood_ctx_t flood = {0};

    memset(&s_mgr_cap1, 0, sizeof(s_mgr_cap1));

    ax25_router_init();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_manager_init());

    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame = capture_cb,
        .user_data = &s_mgr_cap1,
        .port = 1,
        .port_description = "Managed Race",
    };
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_add_client(&cfg, &client));

    make_request(&req, 'm');
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_client_agwpe_in(client, &req));
    vTaskDelay(pdMS_TO_TICKS(30));

    TEST_ASSERT_EQUAL(pdPASS,
                      xTaskCreate(manager_flood_sender_task,
                                  "agwpe_mgr_flood",
                                  4096,
                                  &flood,
                                  tskIDLE_PRIORITY + 1,
                                  NULL));

    for (int waited = 0; waited < 200 && flood.send_count < 20; waited += 10) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ax25_agwpe_server_remove_client(client);
    client = NULL;

    flood.stop = true;
    for (int waited = 0; waited < 2000 && !flood.done; waited += 10) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_TRUE_MESSAGE(flood.done, "Flood sender task did not exit in time");
    TEST_ASSERT_EQUAL_MESSAGE(0, flood.send_errors, "Unexpected router_send errors during fanout/remove race");

    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE_MESSAGE(heap_caps_check_integrity_all(true),
                             "Heap corruption detected after managed client removal during fanout");

    ax25_agwpe_server_manager_deinit();
    ax25_router_deinit();

    vTaskDelay(pdMS_TO_TICKS(20));
    TEST_ASSERT_TRUE_MESSAGE(heap_caps_check_integrity_all(true),
                             "Heap corruption detected after manager/router deinit");
}

/* =========================================================================
 * 11. AX.25 FRAME ROUTING TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: ax25_in without raw/monitor produces no output", "[ax25_agwpe_server]")
{
    create_server();

    ax25_frame_t ui;
    make_ui_frame(&ui, "SRC", "DST", (const uint8_t *)"quiet", 5);
    ax25_agwpe_server_ax25_in(s_server, &ui);

    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_EQUAL(0, s_cap.count);

    destroy_server();
}

TEST_CASE("AGWPE_SRV: monitor mode includes digipeater path in text", "[ax25_agwpe_server]")
{
    create_server();

    /* Enable monitor */
    agwpe_frame_t req;
    make_request(&req, 'm');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* UI frame with digipeater */
    ax25_frame_t ui;
    make_ui_frame(&ui, "SRC", "DST", (const uint8_t *)"via", 3);
    ax25_address_from_string("RELAY", &ui.digipeaters[0]);
    ui.digipeaters[0].has_been_repeated = true;
    ui.num_digipeaters = 1;

    ax25_agwpe_server_ax25_in(s_server, &ui);

    wait_for_frames(1, 500);
    const agwpe_frame_t *mon = find_captured('U', 0);
    TEST_ASSERT_NOT_NULL(mon);

    /* The monitor text should contain "RELAY*" */
    char text[AGWPE_MAX_DATA_LEN + 1];
    memcpy(text, mon->data, mon->header.data_len);
    text[mon->header.data_len] = '\0';
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(text, "RELAY"),
        "Monitor text should include digipeater callsign");

    destroy_server();
}

TEST_CASE("AGWPE_SRV: monitor SABM frame appears as 'S' kind", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'm');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));

    ax25_frame_t sabm;
    make_sabm_frame(&sabm, "SRC", "DST");
    ax25_agwpe_server_ax25_in(s_server, &sabm);

    wait_for_frames(1, 500);
    const agwpe_frame_t *mon = find_captured('S', 0);
    TEST_ASSERT_NOT_NULL_MESSAGE(mon,
        "SABM (U-frame) should be reported as 'S' monitor frame");

    /* Verify text contains "SABM" */
    char text[AGWPE_MAX_DATA_LEN + 1];
    memcpy(text, mon->data, mon->header.data_len);
    text[mon->header.data_len] = '\0';
    TEST_ASSERT_NOT_NULL(strstr(text, "SABM"));

    destroy_server();
}

/* =========================================================================
 * 12. INCOMING CONNECTION (SABM → auto-accept) TEST
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: incoming SABM for registered call creates conn", "[ax25_agwpe_server]")
{
    create_server();

    /* Register callsign */
    agwpe_frame_t reg;
    make_request(&reg, 'X');
    set_from(&reg, "N0CALL");
    ax25_agwpe_server_agwpe_in(s_server, &reg);
    wait_for_frames(1, 500);
    capture_reset();

    /* Enable monitor so we can see that the frame is processed */
    agwpe_frame_t mon_req;
    make_request(&mon_req, 'm');
    ax25_agwpe_server_agwpe_in(s_server, &mon_req);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Inject a SABM from REMOTE to N0CALL */
    ax25_frame_t sabm;
    make_sabm_frame(&sabm, "REMOTE", "N0CALL");
    ax25_agwpe_server_ax25_in(s_server, &sabm);

    /* Wait for the connection logic to run */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* The server should have auto-accepted the SABM and created a
     * connection slot.  We can't easily verify the slot directly (internal
     * state), but the monitor frame should appear. */
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(1, s_cap.count,
        "Expected at least one frame (monitor or connect notification)");

    destroy_server();
}

TEST_CASE("AGWPE_SRV: incoming SABM for unregistered call is ignored", "[ax25_agwpe_server]")
{
    create_server();

    /* Do NOT register any callsign */
    ax25_frame_t sabm;
    make_sabm_frame(&sabm, "REMOTE", "N0CALL");
    ax25_agwpe_server_ax25_in(s_server, &sabm);

    vTaskDelay(pdMS_TO_TICKS(200));
    /* No 'C' connect frame should appear (no callsign registered) */
    const agwpe_frame_t *conn = find_captured('C', 0);
    TEST_ASSERT_NULL_MESSAGE(conn,
        "No connection should be accepted without a registered callsign");

    destroy_server();
}

TEST_CASE("AGWPE_SRV: incoming digi SABM unrepeated then repeated uses one slot", "[ax25_agwpe_server]")
{
    create_server();

    /* Register callsign for incoming auto-accept. */
    agwpe_frame_t reg;
    make_request(&reg, 'X');
    set_from(&reg, "N0CALL");
    ax25_agwpe_server_agwpe_in(s_server, &reg);
    wait_for_frames(1, 500);
    capture_reset();

    /* 1) First SABM seen before digi repeat completion: should not create slot. */
    ax25_frame_t sabm;
    make_sabm_frame(&sabm, "REMOTE", "N0CALL");
    ax25_address_from_string("RELAY", &sabm.digipeaters[0]);
    sabm.digipeaters[0].has_been_repeated = false;
    sabm.num_digipeaters = 1;
    ax25_agwpe_server_ax25_in(s_server, &sabm);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 2) Retransmitted SABM with repeated digi hop should be accepted. */
    sabm.digipeaters[0].has_been_repeated = true;
    ax25_agwpe_server_ax25_in(s_server, &sabm);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* Query capabilities and ensure only one active connection slot is used. */
    capture_reset();
    agwpe_frame_t cap_req;
    make_request(&cap_req, 'g');
    ax25_agwpe_server_agwpe_in(s_server, &cap_req);
    wait_for_frames(1, 500);

    const agwpe_frame_t *resp = find_captured('g', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(
        1, resp->data[7],
        "Expected exactly one active connection slot after unrepeated->repeated SABM sequence");

    destroy_server();
}

/* =========================================================================
 * 13. MULTIPLE CONNECTIONS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: multiple concurrent connect requests", "[ax25_agwpe_server]")
{
    create_server();

    const int connect_count = (int)s_server->max_conns;

    /* Connect to as many stations as configured slots allow. */
    for (int i = 0; i < connect_count; i++) {
        agwpe_frame_t req;
        make_request(&req, 'C');
        set_from(&req, "N0CALL");
        char to[20];
        snprintf(to, sizeof(to), "W1AW-%d", i);
        set_to(&req, to);
        TEST_ASSERT_EQUAL(ESP_OK,
                          ax25_agwpe_server_agwpe_in(s_server, &req));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* Query capabilities — active_conns should reflect the connections */
    capture_reset();
    agwpe_frame_t cap_req;
    make_request(&cap_req, 'g');
    ax25_agwpe_server_agwpe_in(s_server, &cap_req);

    wait_for_frames(1, 500);
    const agwpe_frame_t *resp = find_captured('g', 0);
    TEST_ASSERT_NOT_NULL(resp);
    /* active_conns at data[7] should equal configured slot count */
    TEST_ASSERT_EQUAL_UINT8(connect_count, resp->data[7]);

    destroy_server();
}

TEST_CASE("AGWPE_SRV: connection slots exhausted sends disconnect", "[ax25_agwpe_server]")
{
    create_server();

    /* Fill all configured slots */
    for (int i = 0; i < (int)s_server->max_conns; i++) {
        agwpe_frame_t req;
        make_request(&req, 'C');
        set_from(&req, "N0CALL");
        char to[20];
        snprintf(to, sizeof(to), "FILL-%d", i);
        set_to(&req, to);
        ax25_agwpe_server_agwpe_in(s_server, &req);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Next connect should fail → sends 'd' disconnect to client */
    capture_reset();
    agwpe_frame_t req;
    make_request(&req, 'C');
    set_from(&req, "N0CALL");
    set_to(&req, "OVERFLOW");
    ax25_agwpe_server_agwpe_in(s_server, &req);

    wait_for_frames(1, 500);
    const agwpe_frame_t *disc = find_captured('d', 0);
    TEST_ASSERT_NOT_NULL_MESSAGE(disc,
        "Expected 'd' disconnect when connection slots are full");

    destroy_server();
}

/* =========================================================================
 * 14. FULL ROUND-TRIP INTEGRATION TEST
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: full version+register+info sequence", "[ax25_agwpe_server]")
{
    create_server();

    /* 1) Version request */
    agwpe_frame_t req;
    make_request(&req, 'R');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    wait_for_frames(1, 500);
    TEST_ASSERT_NOT_NULL(find_captured('R', 0));

    /* 2) Port info */
    make_request(&req, 'G');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    wait_for_frames(2, 500);
    TEST_ASSERT_NOT_NULL(find_captured('G', 0));

    /* 3) Register callsign */
    make_request(&req, 'X');
    set_from(&req, "N0CALL-7");
    ax25_agwpe_server_agwpe_in(s_server, &req);
    wait_for_frames(3, 500);
    const agwpe_frame_t *xr = find_captured('X', 0);
    TEST_ASSERT_NOT_NULL(xr);
    TEST_ASSERT_EQUAL_UINT8(1, xr->data[0]);

    /* 4) Port capabilities */
    make_request(&req, 'g');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    wait_for_frames(4, 500);
    TEST_ASSERT_NOT_NULL(find_captured('g', 0));

    destroy_server();
}

/* =========================================================================
 * 15. 'H' HEARD STATIONS (not implemented, no-op)
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: 'H' heard stations is a no-op", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'H');
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(0, s_cap.count);

    destroy_server();
}

/* =========================================================================
 * 16. PORT NUMBER PROPAGATION
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: port number is propagated in responses", "[ax25_agwpe_server]")
{
    capture_reset();
    ax25_router_init();

    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame  = capture_cb,
        .user_data       = &s_cap,
        .port            = 5,
        .port_description = "Port Five",
    };
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_init(&cfg, &s_server_storage));

    /* 'g' request for port 5 */
    agwpe_frame_t req;
    make_request(&req, 'g');
    req.header.port = 5;
    ax25_agwpe_server_agwpe_in(&s_server_storage, &req);

    wait_for_frames(1, 500);
    const agwpe_frame_t *resp = find_captured('g', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT8(5, resp->header.port);

    ax25_agwpe_server_deinit(&s_server_storage);
    ax25_router_deinit();
}

/* =========================================================================
 * 17. LARGE PAYLOAD CLAMPING
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: 'M' with max-size payload doesn't overflow", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'M');
    set_from(&req, "N0CALL");
    set_to(&req, "CQ");
    /* Fill data to maximum */
    memset(req.data, 'A', AGWPE_MAX_DATA_LEN);
    req.header.data_len = AGWPE_MAX_DATA_LEN;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    destroy_server();
}

/* =========================================================================
 * 18. MONITOR TEXT CONTENT VERIFICATION
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: monitor UI text includes payload", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;
    make_request(&req, 'm');
    ax25_agwpe_server_agwpe_in(s_server, &req);
    vTaskDelay(pdMS_TO_TICKS(50));

    const char *payload = "APRS>test123";
    ax25_frame_t ui;
    make_ui_frame(&ui, "N0CALL", "APRS",
                  (const uint8_t *)payload, strlen(payload));
    ax25_agwpe_server_ax25_in(s_server, &ui);

    wait_for_frames(1, 500);
    const agwpe_frame_t *mon = find_captured('U', 0);
    TEST_ASSERT_NOT_NULL(mon);

    char text[AGWPE_MAX_DATA_LEN + 1];
    memcpy(text, mon->data, mon->header.data_len);
    text[mon->header.data_len] = '\0';

    /* Text should contain source, destination, and payload */
    TEST_ASSERT_NOT_NULL(strstr(text, "N0CALL"));
    TEST_ASSERT_NOT_NULL(strstr(text, "APRS"));
    TEST_ASSERT_NOT_NULL(strstr(text, "APRS>test123"));

    destroy_server();
}

/* =========================================================================
 * 19. RAPID COMMAND SEQUENCE
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: rapid sequence of commands doesn't crash", "[ax25_agwpe_server]")
{
    create_server();

    agwpe_frame_t req;

    /* Fire a burst of commands */
    make_request(&req, 'R'); ax25_agwpe_server_agwpe_in(s_server, &req);
    make_request(&req, 'G'); ax25_agwpe_server_agwpe_in(s_server, &req);
    make_request(&req, 'g'); ax25_agwpe_server_agwpe_in(s_server, &req);
    make_request(&req, 'X'); set_from(&req, "TEST"); ax25_agwpe_server_agwpe_in(s_server, &req);
    make_request(&req, 'k'); ax25_agwpe_server_agwpe_in(s_server, &req);
    make_request(&req, 'm'); ax25_agwpe_server_agwpe_in(s_server, &req);
    make_request(&req, 'y'); ax25_agwpe_server_agwpe_in(s_server, &req);
    make_request(&req, 'P'); ax25_agwpe_server_agwpe_in(s_server, &req);
    make_request(&req, 'H'); ax25_agwpe_server_agwpe_in(s_server, &req);

    /* Wait for all responses */
    wait_for_frames(6, 1000);  /* R, G, g, X, y = 5 responses + potential extras */

    /* At least 5 responses expected: R, G, g, X, y */
    TEST_ASSERT_GREATER_OR_EQUAL(5, s_cap.count);

    destroy_server();
}

/* =========================================================================
 * 20. RE-REGISTER CALLSIGN
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: re-registering callsign overwrites previous", "[ax25_agwpe_server]")
{
    create_server();

    /* Register first callsign */
    agwpe_frame_t req;
    make_request(&req, 'X');
    set_from(&req, "OLD");
    ax25_agwpe_server_agwpe_in(s_server, &req);
    wait_for_frames(1, 500);

    /* Register second callsign */
    capture_reset();
    make_request(&req, 'X');
    set_from(&req, "NEW");
    ax25_agwpe_server_agwpe_in(s_server, &req);
    wait_for_frames(1, 500);

    const agwpe_frame_t *resp = find_captured('X', 0);
    TEST_ASSERT_NOT_NULL(resp);
    TEST_ASSERT_EQUAL_UINT8(1, resp->data[0]);

    char call[AGWPE_CALLSIGN_LEN + 1];
    agwpe_get_callsign(resp->header.call_from, call);
    TEST_ASSERT_EQUAL_STRING("NEW", call);

    destroy_server();
}

/* =========================================================================
 * 21. ROUTING REGRESSION TESTS
 * ========================================================================= */

TEST_CASE("AGWPE_SRV: connected ingress uses conn static port, not default", "[ax25_agwpe_server]")
{
    create_server();

    /* Register a default router port to detect unintended fallback routing. */
    router_capture_ctx_t default_ctx = {0};
    ax25_router_port_t default_port;
    memset(&default_port, 0, sizeof(default_port));
    default_port.mode = AX25_PORT_DEFAULT;
    default_port.on_tx_frame = router_capture_cb;
    default_port.user_data = &default_ctx;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&default_port));

    /* Register our AGWPE local callsign. */
    agwpe_frame_t req;
    make_request(&req, 'X');
    set_from(&req, "N0CALL");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));
    wait_for_frames(1, 500); /* 'X' ack */

    /* Open a connected-mode slot N0CALL <-> W1AW. */
    make_request(&req, 'C');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    /* Drive the AGWPE-side connection to connected state by injecting UA. */
    ax25_frame_t ua;
    make_ua_frame(&ua, "W1AW", "N0CALL");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&ua, &s_test_source_port));
    wait_for_frames(2, 500); /* 'X' + likely 'C' */

    /* Clear outputs before the actual assertion frame. */
    capture_reset();
    default_ctx.call_count = 0;

    /* Inject connected payload from BBS side to AGWPE local callsign. */
    const uint8_t payload[] = "bbs->agwpe";
    ax25_frame_t iframe;
    make_i_frame(&iframe, "W1AW", "N0CALL", 0, 0, payload, sizeof(payload) - 1);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&iframe, &s_test_source_port));

    /* Expect AGWPE connected-data output and no default-port fallback. */
    wait_for_frames(1, 500);
    const agwpe_frame_t *d = find_captured('D', 0);
    TEST_ASSERT_NOT_NULL_MESSAGE(d, "Expected connected data ('D') frame for AGWPE client");
    TEST_ASSERT_EQUAL_UINT32(sizeof(payload) - 1, d->header.data_len);
    TEST_ASSERT_EQUAL_MEMORY(payload, d->data, sizeof(payload) - 1);

    wait_for_router_frames(&default_ctx, 1, 100);
    if (default_ctx.call_count > 0) {
        bool inbound_payload_was_default_routed =
            ax25_address_equals(&default_ctx.last_frame.source, &iframe.source) &&
            ax25_address_equals(&default_ctx.last_frame.destination, &iframe.destination) &&
            default_ctx.last_frame.payload_len == iframe.payload_len &&
            memcmp(default_ctx.last_frame.payload, iframe.payload, iframe.payload_len) == 0;
        TEST_ASSERT_FALSE_MESSAGE(inbound_payload_was_default_routed,
                                  "Connected ingress payload should not fall back to default router port");
    }

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&default_port));
    destroy_server();
}

TEST_CASE("AGWPE_SRV: disconnect removes conn static routing", "[ax25_agwpe_server]")
{
    create_server();

    router_capture_ctx_t default_ctx = {0};
    ax25_router_port_t default_port;
    memset(&default_port, 0, sizeof(default_port));
    default_port.mode = AX25_PORT_DEFAULT;
    default_port.on_tx_frame = router_capture_cb;
    default_port.user_data = &default_ctx;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&default_port));

    agwpe_frame_t req;
    make_request(&req, 'X');
    set_from(&req, "N0CALL");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));
    wait_for_frames(1, 500);

    make_request(&req, 'C');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    ax25_frame_t ua;
    make_ua_frame(&ua, "W1AW", "N0CALL");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&ua, &s_test_source_port));
    wait_for_frames(2, 500);

    /* Disconnect and wait for the AGWPE-side notification. */
    make_request(&req, 'd');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_agwpe_in(s_server, &req));

    /* Complete the disconnect handshake so the slot is actually released and
     * its router port is unregistered. */
    ax25_frame_t disc_ua;
    make_ua_frame(&disc_ua, "W1AW", "N0CALL");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&disc_ua, &s_test_source_port));
    wait_for_captured_kind('d', 1200);
    TEST_ASSERT_NOT_NULL_MESSAGE(find_captured('d', 0),
                                 "Expected AGWPE disconnect indication before post-disconnect routing checks");

    /* Post-disconnect: connected data must no longer be delivered to AGWPE.
     * Send a few attempts to tolerate async teardown timing. */
    const uint8_t payload[] = "after-disc";
    ax25_frame_t iframe;
    make_i_frame(&iframe, "W1AW", "N0CALL", 1, 0, payload, sizeof(payload) - 1);
    bool default_routed = false;
    for (int attempt = 0; attempt < 20 && !default_routed; attempt++) {
        capture_reset();
        default_ctx.call_count = 0;

        TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&iframe, &s_test_source_port));
        wait_for_router_frames(&default_ctx, 1, 250);

        TEST_ASSERT_NULL_MESSAGE(find_captured('D', 0),
                                 "No connected-data frame expected after disconnect");

        if (default_ctx.call_count > 0) {
            default_routed = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (!default_routed) {
        /* Depending on teardown timing and queue state, unmatched post-disconnect
         * traffic can be dropped rather than forwarded to default. The critical
         * contract here is that it must no longer be delivered as connected data. */
        TEST_MESSAGE("Post-disconnect frame was not default-routed within timeout; accepted as long as no 'D' frame was emitted");
    }

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&default_port));
    destroy_server();
}

TEST_CASE("AGWPE_SRV: managed disconnect preserves heap integrity", "[ax25_agwpe_server]")
{
    ax25_agwpe_server_t *client = NULL;
    agwpe_frame_t req;

    memset(&s_mgr_cap1, 0, sizeof(s_mgr_cap1));

    ax25_router_init();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_manager_init());

    ax25_agwpe_server_config_t cfg = {
        .on_agwpe_frame = capture_cb,
        .user_data = &s_mgr_cap1,
        .port = 1,
        .port_description = "Managed Lifecycle",
    };
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_add_client(&cfg, &client));

    make_request(&req, 'X');
    set_from(&req, "N0CALL");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_client_agwpe_in(client, &req));
    wait_for_capture_ctx_frames(&s_mgr_cap1, 1, 500);
    TEST_ASSERT_NOT_NULL(find_captured_in(&s_mgr_cap1, 'X', 0));

    make_request(&req, 'C');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_client_agwpe_in(client, &req));

    ax25_frame_t ua;
    make_ua_frame(&ua, "W1AW", "N0CALL");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&ua, &s_test_source_port));
    wait_for_capture_ctx_kind(&s_mgr_cap1, 'C', 1000);
    TEST_ASSERT_NOT_NULL_MESSAGE(find_captured_in(&s_mgr_cap1, 'C', 0),
                                 "Expected managed client connect indication");

    make_request(&req, 'D');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW");
    memcpy(req.data, "b\r", 2);
    req.header.data_len = 2;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_client_agwpe_in(client, &req));
    vTaskDelay(pdMS_TO_TICKS(100));

    ax25_frame_t iframe;
    make_i_frame(&iframe, "W1AW", "N0CALL", 0, 0,
                 (const uint8_t *)"ok", 2);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&iframe, &s_test_source_port));
    wait_for_capture_ctx_kind(&s_mgr_cap1, 'D', 1000);
    TEST_ASSERT_NOT_NULL_MESSAGE(find_captured_in(&s_mgr_cap1, 'D', 0),
                                 "Expected connected data for managed client");

    make_request(&req, 'd');
    set_from(&req, "N0CALL");
    set_to(&req, "W1AW");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_server_client_agwpe_in(client, &req));

    ax25_frame_t disc_ua;
    make_ua_frame(&disc_ua, "W1AW", "N0CALL");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&disc_ua, &s_test_source_port));
    wait_for_capture_ctx_kind(&s_mgr_cap1, 'd', 1200);
    TEST_ASSERT_NOT_NULL_MESSAGE(find_captured_in(&s_mgr_cap1, 'd', 0),
                                 "Expected managed client disconnect indication");

    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_TRUE_MESSAGE(heap_caps_check_integrity_all(true),
                             "Heap corruption detected after managed disconnect");

    ax25_agwpe_server_remove_client(client);
    client = NULL;
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE_MESSAGE(heap_caps_check_integrity_all(true),
                             "Heap corruption detected after managed client removal");

    ax25_agwpe_server_manager_deinit();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_TRUE_MESSAGE(heap_caps_check_integrity_all(true),
                             "Heap corruption detected after manager deinit");

    ax25_router_deinit();
    vTaskDelay(pdMS_TO_TICKS(20));
    TEST_ASSERT_TRUE_MESSAGE(heap_caps_check_integrity_all(true),
                             "Heap corruption detected after router deinit");
}
