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
 * @file test_ax25_router.c
 * @brief Unit tests for ax25_router
 *
 * Tests cover:
 *  - Lifecycle (init / deinit idempotency)
 *  - Port registration and removal validation
 *  - Forwarding rules: destination match, promiscuous, default, digipeater
 *  - Counter accuracy (frames_sent, bytes_sent)
 *  - Multi-port delivery
 *  - Thread-safety stress test
 */

#include "unity.h"
#include "ax25_router.h"
#include "ax25_address.h"
#include "ax25_frame.h"
#include "ax25_types.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static ax25_router_port_t s_test_source_port;

#define ROUTER_WAIT_MS 1500
#define ROUTER_NO_CALL_WAIT_MS 150

/* -------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------- */

/** Build a minimal UI frame addressed from @p src_str to @p dst_str. */
static ax25_frame_t make_ui_frame(const char *dst_str, const char *src_str)
{
    ax25_frame_t f;
    ax25_frame_init(&f);
    ax25_address_from_string(dst_str, &f.destination);
    ax25_address_from_string(src_str, &f.source);
    f.type    = AX25_FRAME_UI;
    f.control = AX25_CTRL_UI;
    f.pid     = AX25_PID_NONE;
    return f;
}

/** Add a digipeater entry to @p frame.  @p repeated sets has_been_repeated. */
static void add_digipeater(ax25_frame_t *frame, const char *call, bool repeated)
{
    ax25_address_t addr;
    ax25_address_from_string(call, &addr);
    addr.has_been_repeated = repeated;
    frame->digipeaters[frame->num_digipeaters++] = addr;
}

/**
 * Compute the expected byte count for a frame the same way the router does.
 * Used to verify bytes_sent counter values.
 */
static uint32_t expected_bytes(const ax25_frame_t *f)
{
    uint32_t addr = (uint32_t)AX25_ADDRESS_LEN * (2u + f->num_digipeaters);
    uint32_t oh   = 1u;
    if (f->type == AX25_FRAME_I || f->type == AX25_FRAME_UI) {
        oh += 1u;
    }
    return addr + oh + (uint32_t)f->payload_len;
}

/**
 * @brief Callback delivery context.
 *
 * Counts invocations and captures the last received frame.
 */
typedef struct {
    volatile int call_count;
    ax25_frame_t last_frame;
} cb_ctx_t;

typedef struct {
    volatile int call_count;
    SemaphoreHandle_t entered;
    SemaphoreHandle_t release;
    int blocks_remaining;
} blocking_cb_ctx_t;

typedef struct {
    volatile int call_count;
    ax25_router_port_t *port;
} self_remove_ctx_t;

/** Callback that counts calls and captures the delivered frame. */
static void capture_cb(const ax25_frame_t *frame, void *user_data)
{
    cb_ctx_t *ctx = (cb_ctx_t *)user_data;
    ctx->call_count++;
    ctx->last_frame = *frame;
}

static void blocking_capture_cb(const ax25_frame_t *frame, void *user_data)
{
    blocking_cb_ctx_t *ctx = (blocking_cb_ctx_t *)user_data;
    ctx->call_count++;
    (void)frame;
    if (ctx->blocks_remaining > 0) {
        ctx->blocks_remaining--;
        xSemaphoreGive(ctx->entered);
        xSemaphoreTake(ctx->release, portMAX_DELAY);
    }
}

static void self_remove_cb(const ax25_frame_t *frame, void *user_data)
{
    self_remove_ctx_t *ctx = (self_remove_ctx_t *)user_data;
    (void)frame;
    ctx->call_count++;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(ctx->port));
}

/** Helper: zero a port struct and wire up the capture callback. */
static void port_init(ax25_router_port_t *port, const char *dest_str, cb_ctx_t *ctx)
{
    memset(port, 0, sizeof(*port));
    ax25_address_from_string(dest_str, &port->destination);
    port->on_tx_frame  = capture_cb;
    port->user_data = ctx;
}

static void wait_for_call_count(cb_ctx_t *ctx, int expected)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ROUTER_WAIT_MS);
    while (xTaskGetTickCount() < deadline) {
        if (ctx->call_count == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL_INT(expected, ctx->call_count);
}

static void wait_for_no_calls(cb_ctx_t *ctx)
{
    vTaskDelay(pdMS_TO_TICKS(ROUTER_NO_CALL_WAIT_MS));
    TEST_ASSERT_EQUAL_INT(0, ctx->call_count);
}

static void wait_for_frames_sent(ax25_router_port_t *port, uint32_t expected)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ROUTER_WAIT_MS);
    while (xTaskGetTickCount() < deadline) {
        if (port->frames_sent == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL_UINT32(expected, port->frames_sent);
}

/* -------------------------------------------------------------------------
 * Test setup / teardown
 * ---------------------------------------------------------------------- */

static void router_test_setup(void)
{
    /* A prior test assertion can bypass teardown; force clean baseline. */
    ax25_router_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_init());
}

static void router_test_teardown(void)
{
    ax25_router_deinit();
}

/* -------------------------------------------------------------------------
 * Lifecycle tests
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: init succeeds", "[ax25_router]")
{
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_init());
    ax25_router_deinit();
}

TEST_CASE("Router: init is idempotent", "[ax25_router]")
{
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_init());
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_init()); /* second call is no-op */
    ax25_router_deinit();
}

TEST_CASE("Router: deinit on uninitialised router is a no-op", "[ax25_router]")
{
    /* Should not crash or assert. */
    ax25_router_deinit();
}

TEST_CASE("Router: deinit then re-init works", "[ax25_router]")
{
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_init());
    ax25_router_deinit();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_init());
    ax25_router_deinit();
}

/* -------------------------------------------------------------------------
 * register_port validation
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: register_port rejects NULL port", "[ax25_router]")
{
    router_test_setup();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_router_register_port(NULL));
    router_test_teardown();
}

TEST_CASE("Router: register_port rejects NULL callback", "[ax25_router]")
{
    router_test_setup();
    ax25_router_port_t port = {0};
    ax25_address_from_string("N0CALL-1", &port.destination);
    port.on_tx_frame = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_router_register_port(&port));
    router_test_teardown();
}

TEST_CASE("Router: register_port fails when router not initialised", "[ax25_router]")
{
    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "N0CALL-1", &ctx);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_router_register_port(&port));
}

TEST_CASE("Router: register_port rejects duplicate registration", "[ax25_router]")
{
    router_test_setup();
    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "N0CALL-1", &ctx);
    TEST_ASSERT_EQUAL(ESP_OK,              ax25_router_register_port(&port));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_router_register_port(&port));
    router_test_teardown();
}

TEST_CASE("Router: register_port zeroes counters", "[ax25_router]")
{
    router_test_setup();
    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "N0CALL-1", &ctx);
    /* Pre-populate counters to verify they are zeroed. */
    port.frames_sent = 99;
    port.bytes_sent  = 999;
    port.frames_dropped = 42;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&port));
    TEST_ASSERT_EQUAL_UINT32(0, port.frames_sent);
    TEST_ASSERT_EQUAL_UINT32(0, port.bytes_sent);
    TEST_ASSERT_EQUAL_UINT32(0, port.frames_dropped);
    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * remove_port validation
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: remove_port rejects NULL port", "[ax25_router]")
{
    router_test_setup();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_router_remove_port(NULL));
    router_test_teardown();
}

TEST_CASE("Router: remove_port fails when router not initialised", "[ax25_router]")
{
    ax25_router_port_t port = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_router_remove_port(&port));
}

TEST_CASE("Router: remove_port returns NOT_FOUND for unregistered port", "[ax25_router]")
{
    router_test_setup();
    ax25_router_port_t port = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, ax25_router_remove_port(&port));
    router_test_teardown();
}

TEST_CASE("Router: remove_port succeeds for registered port", "[ax25_router]")
{
    router_test_setup();
    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "N0CALL-1", &ctx);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&port));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&port));
    router_test_teardown();
}

TEST_CASE("Router: slot is reusable across repeated register/remove cycles", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "N0CALL-1", &ctx);

    /* Regression test for long-lifecycle reuse: repeated remove/register
     * must not exhaust internal router slots over time. */
    for (int i = 0; i < 32; i++) {
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ax25_router_register_port(&port),
                                  "register_port failed during reuse loop");
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ax25_router_remove_port(&port),
                                  "remove_port failed during reuse loop");
    }

    router_test_teardown();
}

TEST_CASE("Router: mixed transient ports remain reusable with baseline ports", "[ax25_router]")
{
    router_test_setup();

    /* Keep a realistic baseline similar to long-lived apps. */
    cb_ctx_t base_default_ctx = {0};
    cb_ctx_t base_static_ctx  = {0};
    cb_ctx_t base_promisc_ctx = {0};

    ax25_router_port_t base_default;
    ax25_router_port_t base_static;
    ax25_router_port_t base_promisc;
    cb_ctx_t *transient_promisc_ctxs = calloc(24, sizeof(*transient_promisc_ctxs));
    cb_ctx_t *transient_dynamic_ctxs = calloc(24, sizeof(*transient_dynamic_ctxs));

    TEST_ASSERT_NOT_NULL(transient_promisc_ctxs);
    TEST_ASSERT_NOT_NULL(transient_dynamic_ctxs);

    port_init(&base_default, "DFLT-0", &base_default_ctx);
    base_default.mode = AX25_PORT_DEFAULT;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&base_default));

    port_init(&base_static, "BASE-1", &base_static_ctx);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&base_static));

    port_init(&base_promisc, "PROMS-0", &base_promisc_ctx);
    base_promisc.mode = AX25_PORT_PROMISCUOUS;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&base_promisc));

    for (int i = 0; i < 24; i++) {
        char src_call[16];
        snprintf(src_call, sizeof(src_call), "SRC%02d-0", i);

        cb_ctx_t *transient_promisc_ctx = &transient_promisc_ctxs[i];
        cb_ctx_t *transient_dynamic_ctx = &transient_dynamic_ctxs[i];
        ax25_router_port_t transient_promisc;
        ax25_router_port_t transient_dynamic;

        port_init(&transient_promisc, "TPRM-0", transient_promisc_ctx);
        transient_promisc.mode = AX25_PORT_PROMISCUOUS;
        esp_err_t promisc_reg = ESP_FAIL;
        for (int attempt = 0; attempt < 20; attempt++) {
            promisc_reg = ax25_router_register_port(&transient_promisc);
            if (promisc_reg == ESP_OK) {
                break;
            }
            if (promisc_reg != ESP_ERR_NO_MEM) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, promisc_reg,
                                  "transient promiscuous register failed");

        memset(&transient_dynamic, 0, sizeof(transient_dynamic));
        transient_dynamic.mode = AX25_PORT_DYNAMIC;
        transient_dynamic.on_tx_frame = capture_cb;
        transient_dynamic.user_data = transient_dynamic_ctx;
        esp_err_t dyn_reg = ESP_FAIL;
        for (int attempt = 0; attempt < 20; attempt++) {
            dyn_reg = ax25_router_register_port(&transient_dynamic);
            if (dyn_reg == ESP_OK) {
                break;
            }
            if (dyn_reg != ESP_ERR_NO_MEM) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, dyn_reg,
                                  "transient dynamic register failed");

        /* Bind dynamic destination by sending from it once. */
        ax25_frame_t bind = make_ui_frame("NOBODY-0", src_call);
        TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&bind, &transient_dynamic));

        /* Once bound, it should behave like a static destination port. */
        ax25_frame_t inbound = make_ui_frame(src_call, "PEER-0");
        TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&inbound, &s_test_source_port));
        wait_for_call_count(transient_dynamic_ctx, 1);

        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                                  ax25_router_remove_port(&transient_dynamic),
                                  "transient dynamic remove failed");
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                                  ax25_router_remove_port(&transient_promisc),
                                  "transient promiscuous remove failed");
    }

    /* Baseline ports should still function after repeated churn. */
    base_default_ctx.call_count = 0;
    base_static_ctx.call_count = 0;
    base_promisc_ctx.call_count = 0;

    ax25_frame_t to_static = make_ui_frame("BASE-1", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&to_static, &s_test_source_port));
    wait_for_call_count(&base_static_ctx, 1);

    ax25_frame_t to_default = make_ui_frame("NOHIT-0", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&to_default, &s_test_source_port));
    wait_for_call_count(&base_default_ctx, 1);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&base_promisc));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&base_static));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&base_default));

    free(transient_promisc_ctxs);
    free(transient_dynamic_ctxs);

    router_test_teardown();
}

TEST_CASE("Router: double remove fails with NOT_FOUND", "[ax25_router]")
{
    router_test_setup();
    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "N0CALL-1", &ctx);
    esp_err_t reg = ESP_FAIL;
    for (int attempt = 0; attempt < 20; attempt++) {
        reg = ax25_router_register_port(&port);
        if (reg == ESP_OK) {
            break;
        }
        if (reg != ESP_ERR_NO_MEM) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL(ESP_OK,            reg);
    TEST_ASSERT_EQUAL(ESP_OK,            ax25_router_remove_port(&port));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, ax25_router_remove_port(&port));
    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * on_tx_frame validation
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: on_tx_frame rejects NULL frame", "[ax25_router]")
{
    router_test_setup();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_router_send(NULL, &s_test_source_port));
    router_test_teardown();
}

TEST_CASE("Router: on_tx_frame rejects NULL source port", "[ax25_router]")
{
    router_test_setup();
    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_router_send(&f, NULL));
    router_test_teardown();
}

TEST_CASE("Router: on_tx_frame fails when router not initialised", "[ax25_router]")
{
    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_router_send(&f, &s_test_source_port));
}

TEST_CASE("Router: on_tx_frame with no ports is a no-op", "[ax25_router]")
{
    router_test_setup();
    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));
    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * Destination-match forwarding
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: frame delivered to matching normal port", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "DEST-0", &ctx);
    ax25_router_register_port(&port);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));
    wait_for_call_count(&ctx, 1);

    router_test_teardown();
}

TEST_CASE("Router: frame not delivered to mismatched normal port", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "OTHER-0", &ctx);
    ax25_router_register_port(&port);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));
    wait_for_no_calls(&ctx);

    router_test_teardown();
}

TEST_CASE("Router: SSID is part of destination comparison", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "DEST-1", &ctx);  /* SSID 1 */
    ax25_router_register_port(&port);

    /* Frame destination has SSID 0 — must NOT match. */
    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    ax25_router_send(&f, &s_test_source_port);
    wait_for_no_calls(&ctx);

    /* Frame destination has SSID 1 — must match. */
    f = make_ui_frame("DEST-1", "SRC-0");
    ax25_router_send(&f, &s_test_source_port);
    wait_for_call_count(&ctx, 1);

    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * Promiscuous port
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: promiscuous port receives all frames", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t promisc;
    port_init(&promisc, "PROMS-0", &ctx);
    promisc.mode = AX25_PORT_PROMISCUOUS;
    ax25_router_register_port(&promisc);

    /* Three frames with different destinations. */
    ax25_frame_t f1 = make_ui_frame("ADDR1-0", "SRC-0");
    ax25_frame_t f2 = make_ui_frame("ADDR2-0", "SRC-0");
    ax25_frame_t f3 = make_ui_frame("ADDR3-0", "SRC-0");
    ax25_router_send(&f1, &s_test_source_port);
    ax25_router_send(&f2, &s_test_source_port);
    ax25_router_send(&f3, &s_test_source_port);

    wait_for_call_count(&ctx, 3);

    router_test_teardown();
}

TEST_CASE("Router: promiscuous port does not block default port", "[ax25_router]")
{
    router_test_setup();

    /* Promiscuous port. */
    cb_ctx_t ctx_p = {0};
    ax25_router_port_t promisc;
    port_init(&promisc, "PROMS-0", &ctx_p);
    promisc.mode = AX25_PORT_PROMISCUOUS;
    ax25_router_register_port(&promisc);

    /* Default port for unmatched frames. */
    cb_ctx_t ctx_d = {0};
    ax25_router_port_t dflt;
    port_init(&dflt, "DFLT-0", &ctx_d);
    dflt.mode = AX25_PORT_DEFAULT;
    ax25_router_register_port(&dflt);

    /* Frame with an address no normal port listens to. */
    ax25_frame_t f = make_ui_frame("NOBODY-0", "SRC-0");
    ax25_router_send(&f, &s_test_source_port);

    /* Promiscuous always gets it; default gets it because no normal port matched. */
    wait_for_call_count(&ctx_p, 1);
    wait_for_call_count(&ctx_d, 1);

    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * Default port
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: default port receives frame when no normal port matches", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t dflt;
    port_init(&dflt, "DFLT-0", &ctx);
    dflt.mode = AX25_PORT_DEFAULT;
    ax25_router_register_port(&dflt);

    ax25_frame_t f = make_ui_frame("NOBODY-0", "SRC-0");
    ax25_router_send(&f, &s_test_source_port);
    wait_for_call_count(&ctx, 1);

    router_test_teardown();
}

TEST_CASE("Router: default port not used when normal port matches", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx_n = {0};
    ax25_router_port_t normal;
    port_init(&normal, "DEST-0", &ctx_n);
    ax25_router_register_port(&normal);

    cb_ctx_t ctx_d = {0};
    ax25_router_port_t dflt;
    port_init(&dflt, "DFLT-0", &ctx_d);
    dflt.mode = AX25_PORT_DEFAULT;
    ax25_router_register_port(&dflt);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    ax25_router_send(&f, &s_test_source_port);

    wait_for_call_count(&ctx_n, 1);  /* normal port received the frame */
    wait_for_no_calls(&ctx_d);       /* default port must NOT have received it */

    router_test_teardown();
}

TEST_CASE("Router: second default port registration is rejected", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx1 = {0}, ctx2 = {0};
    ax25_router_port_t d1, d2;
    port_init(&d1, "DEF1-0", &ctx1); d1.mode = AX25_PORT_DEFAULT;
    port_init(&d2, "DEF2-0", &ctx2); d2.mode = AX25_PORT_DEFAULT;
    TEST_ASSERT_EQUAL(ESP_OK,               ax25_router_register_port(&d1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_router_register_port(&d2));

    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * Digipeater port
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: digipeater port matches next un-repeated digi", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t digi;
    port_init(&digi, "RELAY-1", &ctx);
    digi.mode = AX25_PORT_DIGIPEATER;
    ax25_router_register_port(&digi);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    add_digipeater(&f, "RELAY-1", /*repeated=*/false); /* next digi */
    ax25_router_send(&f, &s_test_source_port);

    wait_for_call_count(&ctx, 1);
    TEST_ASSERT_TRUE(ctx.last_frame.digipeaters[0].has_been_repeated);

    router_test_teardown();
}

TEST_CASE("Router: digipeater port ignores already-repeated digi", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t digi;
    port_init(&digi, "RELAY-1", &ctx);
    digi.mode = AX25_PORT_DIGIPEATER;
    ax25_router_register_port(&digi);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    add_digipeater(&f, "RELAY-1", /*repeated=*/true); /* already repeated */
    ax25_router_send(&f, &s_test_source_port);

    wait_for_no_calls(&ctx);

    router_test_teardown();
}

TEST_CASE("Router: digipeater port uses first un-repeated digi when multiple present", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx_a = {0}, ctx_b = {0};
    ax25_router_port_t digi_a, digi_b;
    port_init(&digi_a, "DIGA-1", &ctx_a); digi_a.mode = AX25_PORT_DIGIPEATER;
    port_init(&digi_b, "DIGB-1", &ctx_b); digi_b.mode = AX25_PORT_DIGIPEATER;
    ax25_router_register_port(&digi_a);
    ax25_router_register_port(&digi_b);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    add_digipeater(&f, "DIGA-1", /*repeated=*/true);  /* already done */
    add_digipeater(&f, "DIGB-1", /*repeated=*/false); /* next */
    ax25_router_send(&f, &s_test_source_port);

    wait_for_no_calls(&ctx_a);       /* repeated — skip */
    wait_for_call_count(&ctx_b, 1);  /* next in path — deliver */

    router_test_teardown();
}

TEST_CASE("Router: digipeater port receives nothing for frame with no digipeaters", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t digi;
    port_init(&digi, "RELAY-1", &ctx);
    digi.mode = AX25_PORT_DIGIPEATER;
    ax25_router_register_port(&digi);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    /* No digipeaters added. */
    ax25_router_send(&f, &s_test_source_port);

    wait_for_no_calls(&ctx);

    router_test_teardown();
}

TEST_CASE("Router: digipeater port receives nothing when all digis already repeated", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t digi;
    port_init(&digi, "RELAY-1", &ctx);
    digi.mode = AX25_PORT_DIGIPEATER;
    ax25_router_register_port(&digi);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    add_digipeater(&f, "RELAY-1", /*repeated=*/true);
    ax25_router_send(&f, &s_test_source_port);
    wait_for_no_calls(&ctx);

    router_test_teardown();
}

TEST_CASE("Router: unresolved digipeater blocks destination and falls back to default", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t static_ctx = {0};
    cb_ctx_t default_ctx = {0};
    ax25_router_port_t normal;
    ax25_router_port_t dflt;

    port_init(&normal, "DEST-0", &static_ctx);
    port_init(&dflt, "DFLT-0", &default_ctx);
    dflt.mode = AX25_PORT_DEFAULT;
    ax25_router_register_port(&normal);
    ax25_router_register_port(&dflt);

    ax25_frame_t frame = make_ui_frame("DEST-0", "SRC-0");
    add_digipeater(&frame, "RELAY-1", false);
    ax25_router_send(&frame, &s_test_source_port);

    wait_for_no_calls(&static_ctx);
    wait_for_call_count(&default_ctx, 1);

    router_test_teardown();
}

TEST_CASE("Router: unresolved digipeater matching port suppresses default fallback", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t digi_ctx = {0};
    cb_ctx_t default_ctx = {0};
    ax25_router_port_t digi;
    ax25_router_port_t dflt;

    port_init(&digi, "RELAY-1", &digi_ctx);
    digi.mode = AX25_PORT_DIGIPEATER;
    port_init(&dflt, "DFLT-0", &default_ctx);
    dflt.mode = AX25_PORT_DEFAULT;
    ax25_router_register_port(&digi);
    ax25_router_register_port(&dflt);

    ax25_frame_t frame = make_ui_frame("DEST-0", "SRC-0");
    add_digipeater(&frame, "RELAY-1", false);
    ax25_router_send(&frame, &s_test_source_port);

    wait_for_call_count(&digi_ctx, 1);
    wait_for_no_calls(&default_ctx);

    router_test_teardown();
}

TEST_CASE("Router: promiscuous sees original while digipeater sees H-bit marked copy", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t promisc_ctx = {0};
    cb_ctx_t digi_ctx = {0};
    ax25_router_port_t promisc;
    ax25_router_port_t digi;

    port_init(&promisc, "PROM-0", &promisc_ctx);
    promisc.mode = AX25_PORT_PROMISCUOUS;
    port_init(&digi, "RELAY-1", &digi_ctx);
    digi.mode = AX25_PORT_DIGIPEATER;
    ax25_router_register_port(&promisc);
    ax25_router_register_port(&digi);

    ax25_frame_t frame = make_ui_frame("DEST-0", "SRC-0");
    add_digipeater(&frame, "RELAY-1", false);
    ax25_router_send(&frame, &s_test_source_port);

    wait_for_call_count(&promisc_ctx, 1);
    wait_for_call_count(&digi_ctx, 1);

    TEST_ASSERT_FALSE(promisc_ctx.last_frame.digipeaters[0].has_been_repeated);
    TEST_ASSERT_TRUE(digi_ctx.last_frame.digipeaters[0].has_been_repeated);

    router_test_teardown();
}

TEST_CASE("Router: destination receives frame after all digipeaters repeated", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "DEST-0", &ctx);
    ax25_router_register_port(&port);

    ax25_frame_t frame = make_ui_frame("DEST-0", "SRC-0");
    add_digipeater(&frame, "RELAY1-1", true);
    add_digipeater(&frame, "RELAY2-2", true);
    ax25_router_send(&frame, &s_test_source_port);

    wait_for_call_count(&ctx, 1);

    router_test_teardown();
}

TEST_CASE("Router: next hop can be eighth digipeater", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t digi;
    port_init(&digi, "R8-8", &ctx);
    digi.mode = AX25_PORT_DIGIPEATER;
    ax25_router_register_port(&digi);

    ax25_frame_t frame = make_ui_frame("DEST-0", "SRC-0");
    for (int index = 1; index <= 8; index++) {
        char call[10];
        snprintf(call, sizeof(call), "R%d-%d", index, index);
        add_digipeater(&frame, call, index != 8);
    }

    ax25_router_send(&frame, &s_test_source_port);
    wait_for_call_count(&ctx, 1);
    TEST_ASSERT_TRUE(ctx.last_frame.digipeaters[7].has_been_repeated);

    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * Counter accuracy
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: frames_sent counter increments on delivery", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "DEST-0", &ctx);
    ax25_router_register_port(&port);

    for (int i = 0; i < 5; i++) {
        ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
        ax25_router_send(&f, &s_test_source_port);
    }

    wait_for_frames_sent(&port, 5);

    router_test_teardown();
}

TEST_CASE("Router: bytes_sent reflects frame wire size", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "DEST-0", &ctx);
    ax25_router_register_port(&port);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    /* Add a small payload to make the byte count non-trivial. */
    const char *data = "HELLO";
    memcpy(f.payload, data, 5);
    f.payload_len = 5;

    ax25_router_send(&f, &s_test_source_port);

    wait_for_frames_sent(&port, 1);
    TEST_ASSERT_EQUAL_UINT32(expected_bytes(&f), port.bytes_sent);

    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * Multi-port delivery
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: frame goes to static and promiscuous port simultaneously", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx1 = {0}, ctx2 = {0}, ctx3 = {0};
    ax25_router_port_t p1, p2, p3;
    port_init(&p1, "DEST-0",  &ctx1);
    port_init(&p2, "PROMS-0", &ctx2);
    p2.mode = AX25_PORT_PROMISCUOUS;
    port_init(&p3, "OTHER-0", &ctx3);

    ax25_router_register_port(&p1);
    ax25_router_register_port(&p2);
    ax25_router_register_port(&p3);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    ax25_router_send(&f, &s_test_source_port);

    wait_for_call_count(&ctx1, 1);  /* static match */
    wait_for_call_count(&ctx2, 1);  /* promiscuous always receives */
    wait_for_no_calls(&ctx3);       /* wrong destination */

    router_test_teardown();
}

TEST_CASE("Router: promiscuous port alongside matching normal port both receive", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx_n = {0}, ctx_p = {0};
    ax25_router_port_t normal, promisc;
    port_init(&normal, "DEST-0",  &ctx_n);
    port_init(&promisc, "PROMS-0", &ctx_p);
    promisc.mode = AX25_PORT_PROMISCUOUS;

    ax25_router_register_port(&normal);
    ax25_router_register_port(&promisc);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    ax25_router_send(&f, &s_test_source_port);

    wait_for_call_count(&ctx_n, 1);
    wait_for_call_count(&ctx_p, 1);

    router_test_teardown();
}

TEST_CASE("Router: frame content is preserved through routing", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "DEST-0", &ctx);
    ax25_router_register_port(&port);

    ax25_frame_t tx = make_ui_frame("DEST-0", "SRC-7");
    const char *payload = "TESTPAYLOAD";
    memcpy(tx.payload, payload, 11);
    tx.payload_len = 11;

    ax25_router_send(&tx, &s_test_source_port);

    wait_for_call_count(&ctx, 1);
    TEST_ASSERT_TRUE(ax25_address_equals(&tx.destination, &ctx.last_frame.destination));
    TEST_ASSERT_TRUE(ax25_address_equals(&tx.source,      &ctx.last_frame.source));
    TEST_ASSERT_EQUAL_UINT8(tx.type,        ctx.last_frame.type);
    TEST_ASSERT_EQUAL_UINT8(tx.control,     ctx.last_frame.control);
    TEST_ASSERT_EQUAL_UINT8(tx.pid,         ctx.last_frame.pid);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)tx.payload_len, (uint32_t)ctx.last_frame.payload_len);
    TEST_ASSERT_EQUAL_MEMORY(tx.payload,    ctx.last_frame.payload, tx.payload_len);

    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * Remove-port behavioural tests
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: removed port no longer receives frames", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx = {0};
    ax25_router_port_t port;
    port_init(&port, "DEST-0", &ctx);
    ax25_router_register_port(&port);

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    ax25_router_send(&f, &s_test_source_port);
    wait_for_call_count(&ctx, 1);

    /* Remove and verify no further delivery. */
    ax25_router_remove_port(&port);
    ctx.call_count = 0;

    ax25_router_send(&f, &s_test_source_port);
    wait_for_no_calls(&ctx);

    router_test_teardown();
}

TEST_CASE("Router: port can remove itself from callback", "[ax25_router]")
{
    router_test_setup();

    self_remove_ctx_t ctx = {0};
    ax25_router_port_t port;
    memset(&port, 0, sizeof(port));
    ax25_address_from_string("DEST-0", &port.destination);
    port.on_tx_frame = self_remove_cb;
    port.user_data = &ctx;
    ctx.port = &port;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&port));

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (xTaskGetTickCount() < deadline && ctx.call_count == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL_INT(1, ctx.call_count);

    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, ax25_router_remove_port(&port));

    ctx.call_count = 0;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL_INT(0, ctx.call_count);

    router_test_teardown();
}

TEST_CASE("Router: queue overflow increments frames_dropped", "[ax25_router]")
{
    router_test_setup();

    blocking_cb_ctx_t ctx = {0};
    ctx.entered = xSemaphoreCreateBinary();
    ctx.release = xSemaphoreCreateBinary();
    ctx.blocks_remaining = 1;
    TEST_ASSERT_NOT_NULL(ctx.entered);
    TEST_ASSERT_NOT_NULL(ctx.release);

    ax25_router_port_t port;
    memset(&port, 0, sizeof(port));
    ax25_address_from_string("DEST-0", &port.destination);
    port.on_tx_frame = blocking_capture_cb;
    port.user_data = &ctx;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&port));

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.entered, pdMS_TO_TICKS(500)));

    uint32_t initial_drops = port.frames_dropped;
    for (int i = 0; i < 16; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));
    }

    TEST_ASSERT_GREATER_THAN_UINT32(initial_drops, port.frames_dropped);

    xSemaphoreGive(ctx.release);
    vTaskDelay(pdMS_TO_TICKS(100));

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&port));
    vSemaphoreDelete(ctx.entered);
    vSemaphoreDelete(ctx.release);

    router_test_teardown();
}

TEST_CASE("Router: queue overflow recovers after callback unblocks", "[ax25_router]")
{
    router_test_setup();

    blocking_cb_ctx_t ctx = {0};
    ctx.entered = xSemaphoreCreateBinary();
    ctx.release = xSemaphoreCreateBinary();
    ctx.blocks_remaining = 1;
    TEST_ASSERT_NOT_NULL(ctx.entered);
    TEST_ASSERT_NOT_NULL(ctx.release);

    ax25_router_port_t port;
    memset(&port, 0, sizeof(port));
    ax25_address_from_string("DEST-0", &port.destination);
    port.on_tx_frame = blocking_capture_cb;
    port.user_data = &ctx;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&port));

    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.entered, pdMS_TO_TICKS(500)));

    for (int i = 0; i < 16; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));
    }
    uint32_t drops_after_overflow = port.frames_dropped;
    TEST_ASSERT_GREATER_THAN_UINT32(0, drops_after_overflow);

    xSemaphoreGive(ctx.release);

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (xTaskGetTickCount() < deadline && port.frames_sent < 2) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_GREATER_THAN_UINT32(1, port.frames_sent);

    uint32_t frames_sent_before_followup = port.frames_sent;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));

    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (xTaskGetTickCount() < deadline && port.frames_sent == frames_sent_before_followup) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_GREATER_THAN_UINT32(frames_sent_before_followup, port.frames_sent);
    TEST_ASSERT_EQUAL_UINT32(drops_after_overflow, port.frames_dropped);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&port));
    vSemaphoreDelete(ctx.entered);
    vSemaphoreDelete(ctx.release);

    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * Thread-safety stress test
 * ---------------------------------------------------------------------- */

#define STRESS_TASKS      4
#define STRESS_FRAMES     50
#define STRESS_TASK_STACK 3072
#define STRESS_WAIT_MS    3000

typedef struct {
    SemaphoreHandle_t start_gate;
    SemaphoreHandle_t done_gate;
    int               task_index;
} stress_task_arg_t;

static void stress_cb(const ax25_frame_t *frame, void *user_data)
{
    (void)frame;
    int *count = (int *)user_data;
    (*count)++;
}

static void stress_task(void *arg)
{
    stress_task_arg_t *a = (stress_task_arg_t *)arg;

    /* Wait for all tasks to be ready before starting. */
    xSemaphoreTake(a->start_gate, portMAX_DELAY);

    int frame_count = 0;
    ax25_router_port_t port = {0};
    char dst[16];
    snprintf(dst, sizeof(dst), "TASK%d-0", a->task_index);
    ax25_address_from_string(dst, &port.destination);
    port.on_tx_frame  = stress_cb;
    port.user_data = &frame_count;

    esp_err_t reg_err = ax25_router_register_port(&port);

    if (reg_err == ESP_OK) {
        for (int i = 0; i < STRESS_FRAMES; i++) {
            ax25_frame_t f = make_ui_frame(dst, "SRC-0");
            ax25_router_send(&f, &s_test_source_port);
        }
        ax25_router_remove_port(&port);
    }

    xSemaphoreGive(a->done_gate);
    vTaskDelete(NULL);
}

TEST_CASE("Router: concurrent register/route/remove is thread-safe", "[ax25_router]")
{
    router_test_setup();

    SemaphoreHandle_t start_gate = xSemaphoreCreateCounting(STRESS_TASKS, 0);
    SemaphoreHandle_t done_gate  = xSemaphoreCreateCounting(STRESS_TASKS, 0);
    int created_tasks = 0;
    int completed_tasks = 0;
    TEST_ASSERT_NOT_NULL(start_gate);
    TEST_ASSERT_NOT_NULL(done_gate);

    stress_task_arg_t args[STRESS_TASKS];
    for (int i = 0; i < STRESS_TASKS; i++) {
        args[i].start_gate = start_gate;
        args[i].done_gate  = done_gate;
        args[i].task_index = i;
        if (xTaskCreate(stress_task,
                        "stress",
                        STRESS_TASK_STACK,
                        &args[i],
                        5,
                        NULL) == pdPASS) {
            created_tasks++;
        } else {
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT(STRESS_TASKS, created_tasks);

    /* Release all tasks simultaneously. */
    for (int i = 0; i < created_tasks; i++) {
        xSemaphoreGive(start_gate);
    }

    /* Wait for all tasks to finish. */
    for (int i = 0; i < created_tasks; i++) {
        if (xSemaphoreTake(done_gate, pdMS_TO_TICKS(STRESS_WAIT_MS)) == pdTRUE) {
            completed_tasks++;
        } else {
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT(created_tasks, completed_tasks);

    vSemaphoreDelete(start_gate);
    vSemaphoreDelete(done_gate);

    router_test_teardown();
    /* If we reached here without a crash or assertion failure, the test passes. */
}

/* -------------------------------------------------------------------------
 * Registration constraint tests
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: duplicate static destination rejected", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx1 = {0}, ctx2 = {0};
    ax25_router_port_t p1, p2;
    port_init(&p1, "DEST-0", &ctx1);
    port_init(&p2, "DEST-0", &ctx2);  /* same destination */
    TEST_ASSERT_EQUAL(ESP_OK,               ax25_router_register_port(&p1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_router_register_port(&p2));

    router_test_teardown();
}

TEST_CASE("Router: duplicate static and digipeater destination rejected", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx1 = {0}, ctx2 = {0};
    ax25_router_port_t p1, p2;
    port_init(&p1, "DEST-0", &ctx1);
    port_init(&p2, "DEST-0", &ctx2);
    p2.mode = AX25_PORT_DIGIPEATER;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&p1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_router_register_port(&p2));

    router_test_teardown();
}

TEST_CASE("Router: duplicate digipeater destination rejected", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx1 = {0}, ctx2 = {0};
    ax25_router_port_t p1, p2;
    port_init(&p1, "RELAY-1", &ctx1);
    port_init(&p2, "RELAY-1", &ctx2);
    p1.mode = AX25_PORT_DIGIPEATER;
    p2.mode = AX25_PORT_DIGIPEATER;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&p1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_router_register_port(&p2));

    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * Source-port skip
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: source port does not receive its own frame", "[ax25_router]")
{
    router_test_setup();

    /* p1: static port on DEST-0 — will be used as source_port. */
    cb_ctx_t ctx1 = {0};
    ax25_router_port_t p1;
    port_init(&p1, "DEST-0", &ctx1);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&p1));

    /* p2: promiscuous — always receives, regardless of source. */
    cb_ctx_t ctx2 = {0};
    ax25_router_port_t p2;
    port_init(&p2, "PROMS-0", &ctx2);
    p2.mode = AX25_PORT_PROMISCUOUS;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&p2));

    /* Send with p1 as source — p1 must be skipped, p2 must receive. */
    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &p1));

    wait_for_call_count(&ctx2, 1);
    wait_for_no_calls(&ctx1);

    router_test_teardown();
}

/* -------------------------------------------------------------------------
 * Dynamic port
 * ---------------------------------------------------------------------- */

TEST_CASE("Router: unbound dynamic port does not receive inbound frames", "[ax25_router]")
{
    router_test_setup();

    /* An unbound dynamic port. */
    cb_ctx_t ctx_d = {0};
    ax25_router_port_t dyn_port;
    memset(&dyn_port, 0, sizeof(dyn_port));
    dyn_port.mode      = AX25_PORT_DYNAMIC;
    dyn_port.on_tx_frame  = capture_cb;
    dyn_port.user_data = &ctx_d;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&dyn_port));

    /* Route a frame from a non-dynamic source — the unbound dynamic port must
     * NOT fire, even though no static port matches. Unlike AX25_PORT_DEFAULT,
     * an unbound dynamic port is only activated by outbound routing. */
    ax25_frame_t f = make_ui_frame("NOBODY-0", "SRC-A");
    ax25_router_send(&f, &s_test_source_port);
    wait_for_no_calls(&ctx_d);

    router_test_teardown();
}

TEST_CASE("Router: dynamic port binds destination to frame source on first send", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx_d = {0};
    ax25_router_port_t dyn_port;
    memset(&dyn_port, 0, sizeof(dyn_port));
    dyn_port.mode      = AX25_PORT_DYNAMIC;
    dyn_port.on_tx_frame  = capture_cb;
    dyn_port.user_data = &ctx_d;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&dyn_port));

    /* Route with dyn_port as source_port: binding must be set to frame->source. */
    ax25_frame_t f = make_ui_frame("NOBODY-0", "SRC-7");
    ax25_router_send(&f, &dyn_port);

    ax25_address_t expected_dest;
    ax25_address_from_string("SRC-7", &expected_dest);
    TEST_ASSERT_TRUE(ax25_address_equals(&dyn_port.destination, &expected_dest));

    /* A second send-as-source must not change the binding. */
    f = make_ui_frame("NOBODY-0", "CHANGED-0");
    ax25_router_send(&f, &dyn_port);
    TEST_ASSERT_TRUE(ax25_address_equals(&dyn_port.destination, &expected_dest));

    router_test_teardown();
}

TEST_CASE("Router: bound dynamic port acts like static port", "[ax25_router]")
{
    router_test_setup();

    cb_ctx_t ctx_d = {0};
    ax25_router_port_t dyn_port;
    memset(&dyn_port, 0, sizeof(dyn_port));
    dyn_port.mode      = AX25_PORT_DYNAMIC;
    dyn_port.on_tx_frame  = capture_cb;
    dyn_port.user_data = &ctx_d;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&dyn_port));

    /* Trigger binding: route a frame with dyn_port as source, frame from SRC-3. */
    ax25_frame_t f = make_ui_frame("NOBODY-0", "SRC-3");
    ax25_router_send(&f, &dyn_port);

    ax25_address_t expected_dest;
    ax25_address_from_string("SRC-3", &expected_dest);
    TEST_ASSERT_TRUE(ax25_address_equals(&dyn_port.destination, &expected_dest));

    /* Now bound to SRC-3. Frame addressed to SRC-3 must deliver. */
    f = make_ui_frame("SRC-3", "OTHER-0");
    ax25_router_send(&f, &s_test_source_port);
    wait_for_call_count(&ctx_d, 1);
    ctx_d.call_count = 0;

    /* Frame addressed to a different callsign must NOT deliver. */
    f = make_ui_frame("OTHER-0", "OTHER-0");
    ax25_router_send(&f, &s_test_source_port);
    wait_for_no_calls(&ctx_d);

    router_test_teardown();
}

TEST_CASE("Router: self-removed slot is reusable by a different port", "[ax25_router]")
{
    router_test_setup();

    self_remove_ctx_t self_ctx = {0};
    cb_ctx_t new_ctx = {0};
    ax25_router_port_t self_port;
    ax25_router_port_t new_port;
    ax25_frame_t f = make_ui_frame("DEST-0", "SRC-0");

    memset(&self_port, 0, sizeof(self_port));
    ax25_address_from_string("DEST-0", &self_port.destination);
    self_port.on_tx_frame = self_remove_cb;
    self_port.user_data = &self_ctx;
    self_ctx.port = &self_port;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&self_port));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (xTaskGetTickCount() < deadline && self_ctx.call_count == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL_INT(1, self_ctx.call_count);

    port_init(&new_port, "DEST-0", &new_ctx);
    esp_err_t reg = ESP_FAIL;
    for (int attempt = 0; attempt < 20; attempt++) {
        reg = ax25_router_register_port(&new_port);
        if (reg == ESP_OK) {
            break;
        }
        if (reg != ESP_ERR_NO_MEM) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL(ESP_OK, reg);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&f, &s_test_source_port));
    wait_for_call_count(&new_ctx, 1);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&new_port));

    router_test_teardown();
}
