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
 * @file test_ax25_conn.c
 * @brief Comprehensive unit tests for AX.25 connected mode session handler
 */

#include "unity.h"
#include "ax25_conn.h"
#include "ax25_conn_dispatcher.h"
#include "ax25_address.h"
#include "ax25_frame.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "TEST_CONN";

#define TEST_CONN_WAIT_MS 1500

/*******************************************************************************
 * Test Infrastructure
 ******************************************************************************/

/* Test state tracking */
typedef struct {
    /* Callback counters */
    volatile int connect_count;
    volatile int disconnect_count;
    volatile int link_reset_count;
    volatile int error_count;
    volatile int final_count;
    volatile int data_count;
    volatile int frame_count;

    /* Last callback data */
    bool last_connect_local;
    esp_err_t last_error_code;
    char last_error_msg[128];
    uint8_t last_data[256];
    size_t last_data_len;
    ax25_frame_t last_frame;

    /* Frame history for verification */
    ax25_frame_t frame_history[16];
    volatile int frame_history_count;

    /* Synchronization */
    SemaphoreHandle_t sem;
    SemaphoreHandle_t callback_enter_sem;
    SemaphoreHandle_t callback_release_sem;
    bool refuse_connection;

    /* Re-entrant callback checks */
    ax25_conn_t* callback_ctx;
    esp_err_t callback_result;
    ax25_conn_state_t callback_state;
    volatile int callback_reenter_count;
} test_state_t;

static test_state_t s_test_state;

typedef struct {
    ax25_conn_t* ctx;
    ax25_conn_dispatch_action_t first_action;
    ax25_conn_dispatch_action_t second_action;
    volatile int enqueue_count;
    volatile int enqueue_error_count;
} dispatch_timer_enqueue_state_t;

static dispatch_timer_enqueue_state_t s_dispatch_timer_enqueue_state;

static void reset_test_state(void) {
    memset(&s_test_state, 0, sizeof(s_test_state));
    if (!s_test_state.sem) {
        s_test_state.sem = xSemaphoreCreateBinary();
    }
    if (!s_test_state.callback_enter_sem) {
        s_test_state.callback_enter_sem = xSemaphoreCreateBinary();
    }
    if (!s_test_state.callback_release_sem) {
        s_test_state.callback_release_sem = xSemaphoreCreateBinary();
    }
}

static void cleanup_test_state(void) {
    if (s_test_state.sem) {
        vSemaphoreDelete(s_test_state.sem);
        s_test_state.sem = NULL;
    }
    if (s_test_state.callback_enter_sem) {
        vSemaphoreDelete(s_test_state.callback_enter_sem);
        s_test_state.callback_enter_sem = NULL;
    }
    if (s_test_state.callback_release_sem) {
        vSemaphoreDelete(s_test_state.callback_release_sem);
        s_test_state.callback_release_sem = NULL;
    }
}

static void wait_for_counter_value(const volatile int* counter, int expected) {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(TEST_CONN_WAIT_MS);

    while (xTaskGetTickCount() < deadline) {
        if (*counter == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_ASSERT_EQUAL_INT(expected, *counter);
}

static void wait_for_counter_at_least(const volatile int* counter, int minimum) {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(TEST_CONN_WAIT_MS);

    while (xTaskGetTickCount() < deadline) {
        if (*counter >= minimum) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_ASSERT_TRUE(*counter >= minimum);
}

static void wait_for_no_counter_change(const volatile int* counter, int expected) {
    vTaskDelay(pdMS_TO_TICKS(150));
    TEST_ASSERT_EQUAL_INT(expected, *counter);
}

static void wait_for_frame_count(int expected) {
    wait_for_counter_value(&s_test_state.frame_count, expected);
}

static void wait_for_frame_count_at_least(int minimum) {
    wait_for_counter_at_least(&s_test_state.frame_count, minimum);
}

static void wait_for_connect_count(int expected) {
    wait_for_counter_value(&s_test_state.connect_count, expected);
}

static void wait_for_disconnect_count(int expected) {
    wait_for_counter_value(&s_test_state.disconnect_count, expected);
}

static void wait_for_error_count(int expected) {
    wait_for_counter_value(&s_test_state.error_count, expected);
}

static void wait_for_final_count(int expected) {
    wait_for_counter_value(&s_test_state.final_count, expected);
}

static void wait_for_data_count(int expected) {
    wait_for_counter_value(&s_test_state.data_count, expected);
}

static void wait_for_callback_reenter_count(int expected) {
    wait_for_counter_value(&s_test_state.callback_reenter_count, expected);
}

/* Callbacks */
static void test_on_connect(ax25_address_t remote_addr, bool is_local_initiated, void* user_data) {
    (void)remote_addr;
    test_state_t* state = (test_state_t*)user_data;
    state->connect_count++;
    state->last_connect_local = is_local_initiated;
    ESP_LOGI(TAG, "on_connect: local=%d", is_local_initiated);
}

static void test_on_disconnect(void* user_data) {
    test_state_t* state = (test_state_t*)user_data;
    state->disconnect_count++;
    ESP_LOGI(TAG, "on_disconnect");
    if (state->sem) {
        xSemaphoreGive(state->sem);
    }
}

static void test_on_link_reset(void* user_data) {
    test_state_t* state = (test_state_t*)user_data;
    state->link_reset_count++;
    ESP_LOGI(TAG, "on_link_reset");
}

static void test_on_error(const ax25_conn_error_t* error, void* user_data) {
    test_state_t* state = (test_state_t*)user_data;
    state->error_count++;
    state->last_error_code = error->code;
    strncpy(state->last_error_msg, error->message, sizeof(state->last_error_msg) - 1);
    ESP_LOGW(TAG, "on_error: %s (code=%d retry=%d)", error->message, error->code, error->retry_count);
}

static void test_on_final(void* user_data) {
    test_state_t* state = (test_state_t*)user_data;
    state->final_count++;
    ESP_LOGI(TAG, "on_final");
}

static void test_on_data(const uint8_t* data, size_t len, void* user_data) {
    test_state_t* state = (test_state_t*)user_data;
    state->data_count++;
    if (len <= sizeof(state->last_data)) {
        memcpy(state->last_data, data, len);
        state->last_data_len = len;
    }
    ESP_LOGI(TAG, "on_data: len=%zu", len);
}

static void test_on_frame(const ax25_frame_t* frame, void* user_data) {
    test_state_t* state = (test_state_t*)user_data;
    state->frame_count++;
    state->last_frame = *frame;
    /* Store in history */
    if (state->frame_history_count < 16) {
        state->frame_history[state->frame_history_count] = *frame;
        state->frame_history_count++;
    }
    ESP_LOGI(TAG, "on_tx_frame");
}

static void test_on_frame_blocking(const ax25_frame_t* frame, void* user_data) {
    test_state_t* state = (test_state_t*)user_data;

    test_on_frame(frame, user_data);

    if (state->callback_enter_sem) {
        xSemaphoreGive(state->callback_enter_sem);
    }

    if (state->callback_release_sem) {
        xSemaphoreTake(state->callback_release_sem, pdMS_TO_TICKS(1000));
    }
}

static void test_on_connect_send_data(ax25_address_t remote_addr,
                                      bool is_local_initiated,
                                      void* user_data) {
    (void)remote_addr;
    test_state_t* state = (test_state_t*)user_data;
    static const uint8_t payload[] = {'O', 'K'};

    state->connect_count++;
    state->last_connect_local = is_local_initiated;

    if (is_local_initiated && state->callback_ctx != NULL) {
        state->callback_reenter_count++;
        state->callback_result = ax25_conn_send_data(state->callback_ctx,
                                                     payload,
                                                     sizeof(payload));
    }
}

static void test_on_data_get_state(const uint8_t* data, size_t len, void* user_data) {
    test_state_t* state = (test_state_t*)user_data;

    test_on_data(data, len, user_data);

    if (state->callback_ctx != NULL) {
        state->callback_reenter_count++;
        state->callback_state = ax25_conn_get_state(state->callback_ctx);
        state->callback_result = ESP_OK;
    }
}

static void release_callback_task(void* arg) {
    test_state_t* state = (test_state_t*)arg;

    vTaskDelay(pdMS_TO_TICKS(50));
    if (state->callback_release_sem) {
        xSemaphoreGive(state->callback_release_sem);
    }

    vTaskDelete(NULL);
}

static void test_dispatch_enqueue_timer_cb(void* arg) {
    dispatch_timer_enqueue_state_t* state = (dispatch_timer_enqueue_state_t*)arg;

    if (ax25_conn_dispatcher_enqueue(state->ctx, &state->first_action) == ESP_OK) {
        state->enqueue_count++;
    } else {
        state->enqueue_error_count++;
    }

    if (ax25_conn_dispatcher_enqueue(state->ctx, &state->second_action) == ESP_OK) {
        state->enqueue_count++;
    } else {
        state->enqueue_error_count++;
    }
}

/* Helper to create test callbacks */
static ax25_conn_callbacks_t make_test_callbacks(void) {
    return (ax25_conn_callbacks_t) {
        .on_connect = test_on_connect,
        .on_disconnect = test_on_disconnect,
        .on_link_reset = test_on_link_reset,
        .on_error = test_on_error,
        .on_final = test_on_final,
        .on_data = test_on_data,
        .on_tx_frame = test_on_frame
    };
}

/* Helper to create test config with short timeouts for testing */
static ax25_conn_config_t make_test_config(void) {
    return (ax25_conn_config_t) {
        .t1_ms = 100,       /* Short T1 for tests */
        .t2_ms = 50,        /* Short T2 for tests */
        .t3_ms = 500,       /* Short T3 for tests */
        .n2_retries = 3,    /* Fewer retries for tests */
        .window_size = 4,
    };
}

/* Helper to build a test frame struct */
static ax25_frame_t build_test_frame(const char* dst, const char* src,
                                      uint8_t control, bool is_command,
                                      const uint8_t* payload, size_t payload_len) {
    ax25_frame_t frame;
    ax25_frame_init(&frame);

    ax25_address_from_string(dst, &frame.destination);
    ax25_address_from_string(src, &frame.source);
    frame.is_command = is_command;
    frame.control = control;
    frame.type = ax25_frame_identify_type(control);

    if (payload && payload_len > 0 &&
        (frame.type == AX25_FRAME_I || frame.type == AX25_FRAME_UI)) {
        frame.pid = AX25_PID_NONE;
        memcpy(frame.payload, payload, payload_len);
        frame.payload_len = payload_len;
    }

    return frame;
}

static ax25_address_t make_address(const char* addr_str) {
    ax25_address_t addr;
    ax25_address_from_string(addr_str, &addr);
    return addr;
}

static void assert_digipeater_path(const ax25_frame_t* frame,
                                   const ax25_address_t* expected,
                                   uint8_t count) {
    TEST_ASSERT_EQUAL_UINT8(count, frame->num_digipeaters);
    for (uint8_t index = 0; index < count; index++) {
        TEST_ASSERT_TRUE(ax25_address_equals(&frame->digipeaters[index], &expected[index]));
        TEST_ASSERT_FALSE(frame->digipeaters[index].has_been_repeated);
    }
}

/*******************************************************************************
 * Basic Initialization Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: init and deinit", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);

    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    esp_err_t err = ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(ctx.initialized);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));

    ax25_conn_deinit(&ctx);
    TEST_ASSERT_FALSE(ctx.initialized);

    cleanup_test_state();
}

TEST_CASE("AX25Conn: init rejects NULL params", "[ax25_conn]")
{
    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_init(NULL, &local, &cbs, NULL, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_init(&ctx, NULL, &cbs, NULL, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_init(&ctx, &local, NULL, NULL, NULL));
}

TEST_CASE("AX25Conn: init requires on_tx_frame and on_data callbacks", "[ax25_conn]")
{
    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);

    /* Missing on_tx_frame */
    ax25_conn_callbacks_t cbs1 = {
        .on_data = test_on_data,
        .on_tx_frame = NULL
    };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_init(&ctx, &local, &cbs1, NULL, NULL));

    /* Missing on_data */
    ax25_conn_callbacks_t cbs2 = {
        .on_data = NULL,
        .on_tx_frame = test_on_frame
    };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_init(&ctx, &local, &cbs2, NULL, NULL));
}

TEST_CASE("AX25Conn: uses default config when NULL", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();

    esp_err_t err = ax25_conn_init(&ctx, &local, &cbs, &s_test_state, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, err);

    /* Check that defaults are applied */
    TEST_ASSERT_EQUAL(CONFIG_AX25_CONN_T1_MS, ctx.config.t1_ms);
    TEST_ASSERT_EQUAL(CONFIG_AX25_CONN_N2_RETRIES, ctx.config.n2_retries);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Connection Establishment Tests (Local Initiated)
 ******************************************************************************/

TEST_CASE("AX25Conn: connect sends SABM", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    esp_err_t err = ax25_conn_connect(&ctx, &remote);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_CONNECTION, ax25_conn_get_state(&ctx));
    wait_for_frame_count(1);

    /* Verify SABM was sent */
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_U, sent.type);
    TEST_ASSERT_EQUAL(AX25_CTRL_SABM | AX25_CTRL_PF_BIT, sent.control);
    TEST_ASSERT_TRUE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: connect via digipeater sends SABM path", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_address_t digipeaters[2] = {
        make_address("RELAY1-1"),
        make_address("RELAY2-2"),
    };
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    esp_err_t err = ax25_conn_connect(&ctx, &remote, digipeaters, 2);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_CONNECTION, ax25_conn_get_state(&ctx));
    wait_for_frame_count(1);

    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_CTRL_SABM | AX25_CTRL_PF_BIT, sent.control);
    assert_digipeater_path(&sent, digipeaters, 2);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: connect via max digipeaters preserves all hops", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_address_t digipeaters[AX25_MAX_DIGIPEATERS];
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    const char* path[AX25_MAX_DIGIPEATERS] = {
        "R1-1", "R2-2", "R3-3", "R4-4", "R5-5", "R6-6", "R7-7", "R8-8"
    };

    for (uint8_t index = 0; index < AX25_MAX_DIGIPEATERS; index++) {
        digipeaters[index] = make_address(path[index]);
    }

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_conn_connect(&ctx, &remote, digipeaters, AX25_MAX_DIGIPEATERS));
    wait_for_frame_count(1);
    assert_digipeater_path(&s_test_state.last_frame, digipeaters, AX25_MAX_DIGIPEATERS);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: connect rejects more than max digipeaters", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_address_t digipeaters[AX25_MAX_DIGIPEATERS + 1];
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    for (uint8_t index = 0; index < AX25_MAX_DIGIPEATERS + 1; index++) {
        char path[10];
        snprintf(path, sizeof(path), "R%u", (unsigned)(index + 1));
        digipeaters[index] = make_address(path);
    }

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_conn_connect(&ctx, &remote, digipeaters, AX25_MAX_DIGIPEATERS + 1));
    TEST_ASSERT_EQUAL(0, s_test_state.frame_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: initial T1 seeds from digipeater count", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_address_t digipeaters[2] = {
        make_address("RELAY1-1"),
        make_address("RELAY2-2"),
    };
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 1000;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_connect(&ctx, &remote, digipeaters, 2));

    TEST_ASSERT_EQUAL_UINT32(cfg.t1_ms, ctx.t1_current_ms);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: handshake keeps fixed T1 after data ACK", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_address_t digipeaters[1] = { make_address("RELAY-1") };
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_connect(&ctx, &remote, digipeaters, 1));

    uint32_t seeded_floor = ctx.t1_current_ms;

    ax25_frame_t ua = build_test_frame("N0CALL", "N0CALL-1",
                                       AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ua.num_digipeaters = 1;
    ua.digipeaters[0] = digipeaters[0];
    ua.digipeaters[0].has_been_repeated = true;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_on_frame(&ctx, &ua));
    TEST_ASSERT_EQUAL_UINT32(seeded_floor, ctx.t1_current_ms);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"A", 1));
    ax25_frame_t rr = build_test_frame("N0CALL", "N0CALL-1",
                                       ax25_frame_build_rr_control(1, false), false, NULL, 0);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_on_frame(&ctx, &rr));

    TEST_ASSERT_EQUAL_UINT32(seeded_floor, ctx.t1_current_ms);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: T1 timeout keeps fixed retry interval", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 50;
    cfg.n2_retries = 2;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_connect(&ctx, &remote));

    uint32_t before_timeout_t1 = ctx.t1_current_ms;
    vTaskDelay(pdMS_TO_TICKS(80));

    TEST_ASSERT_EQUAL_UINT32(before_timeout_t1, ctx.t1_current_ms);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: fixed T1 remains stable across ACK samples", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 1800;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_connect(&ctx, &remote));

    ax25_frame_t ua = build_test_frame("N0CALL", "N0CALL-1",
                                       AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua);

    uint32_t t1_after_ua = ctx.t1_current_ms;

    /* Sample 2: send one I-frame and ACK with RR N(R)=1 */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"A", 1));
    ax25_frame_t rr1 = build_test_frame("N0CALL", "N0CALL-1",
                                        ax25_frame_build_rr_control(1, false), false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rr1);
    uint32_t t1_after_rr1 = ctx.t1_current_ms;

    /* Sample 3: repeat acknowledgement path. */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"B", 1));
    ax25_frame_t rr2 = build_test_frame("N0CALL", "N0CALL-1",
                                        ax25_frame_build_rr_control(2, false), false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rr2);
    uint32_t t1_after_rr2 = ctx.t1_current_ms;

    TEST_ASSERT_EQUAL_UINT32(cfg.t1_ms, t1_after_ua);
    TEST_ASSERT_EQUAL_UINT32(cfg.t1_ms, t1_after_rr1);
    TEST_ASSERT_EQUAL_UINT32(cfg.t1_ms, t1_after_rr2);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: fixed T1 ignores RTT sample extremes", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 3000;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_connect(&ctx, &remote));

    /* Establish connection and ensure T1 remains fixed across traffic. */
    ax25_frame_t ua = build_test_frame("N0CALL", "N0CALL-1",
                                       AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"X", 1));
    ax25_frame_t rr_fast = build_test_frame("N0CALL", "N0CALL-1",
                                            ax25_frame_build_rr_control(1, false), false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rr_fast);
    TEST_ASSERT_EQUAL_UINT32(cfg.t1_ms, ctx.t1_current_ms);

    /* Simulate another ACK cycle and keep fixed T1. */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"Y", 1));
    ax25_frame_t rr_slow = build_test_frame("N0CALL", "N0CALL-1",
                                            ax25_frame_build_rr_control(2, false), false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rr_slow);
    TEST_ASSERT_EQUAL_UINT32(cfg.t1_ms, ctx.t1_current_ms);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: UA response completes connection", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    /* Simulate UA response with F bit */
    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);

    ax25_conn_on_frame(&ctx, &ua_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_TRUE(ax25_conn_is_connected(&ctx));
    wait_for_connect_count(1);
    TEST_ASSERT_TRUE(s_test_state.last_connect_local);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: UA with unresolved hops leaves fixed T1 unchanged", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_address_t digipeaters[1] = { make_address("RELAY-1") };
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_connect(&ctx, &remote, digipeaters, 1));

    uint32_t t1_before = ctx.t1_current_ms;
    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                             AX25_CTRL_UA | AX25_CTRL_PF_BIT,
                                             false,
                                             NULL,
                                             0);
    ua_frame.num_digipeaters = 1;
    ua_frame.digipeaters[0] = digipeaters[0];
    ua_frame.digipeaters[0].has_been_repeated = false;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_on_frame(&ctx, &ua_frame));

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_CONNECTION, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL_UINT32(t1_before, ctx.t1_current_ms);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: DM response refuses connection", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    /* Simulate DM response */
    ax25_frame_t dm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_DM | AX25_CTRL_PF_BIT, false, NULL, 0);

    ax25_conn_on_frame(&ctx, &dm_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    wait_for_error_count(1);
    wait_for_no_counter_change(&s_test_state.connect_count, 0);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: connect rejected when not disconnected", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    /* Try to connect again while awaiting */
    esp_err_t err = ax25_conn_connect(&ctx, &remote);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: connect rejects NULL digipeater array with nonzero count", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_conn_connect(&ctx, &remote, NULL, 1));

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Connection Establishment Tests (Remote Initiated)
 ******************************************************************************/

TEST_CASE("AX25Conn: incoming SABM establishes connection", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Receive SABM from remote */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);

    ax25_conn_on_frame(&ctx, &sabm_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    wait_for_connect_count(1);
    TEST_ASSERT_FALSE(s_test_state.last_connect_local);
    wait_for_frame_count(1);

    /* Verify UA was sent */
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_U, sent.type);
    TEST_ASSERT_EQUAL(AX25_CTRL_UA | AX25_CTRL_PF_BIT, sent.control);
    TEST_ASSERT_FALSE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: incoming SABM via digipeaters reuses normalized path", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                               AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    sabm_frame.digipeaters[0] = make_address("RELAY1-1");
    sabm_frame.digipeaters[0].has_been_repeated = true;
    sabm_frame.digipeaters[1] = make_address("RELAY2-2");
    sabm_frame.digipeaters[1].has_been_repeated = true;
    sabm_frame.num_digipeaters = 2;

    ax25_conn_on_frame(&ctx, &sabm_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    wait_for_frame_count(1);
    TEST_ASSERT_EQUAL_UINT8(2, s_test_state.last_frame.num_digipeaters);
    TEST_ASSERT_FALSE(s_test_state.last_frame.digipeaters[0].has_been_repeated);
    TEST_ASSERT_FALSE(s_test_state.last_frame.digipeaters[1].has_been_repeated);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/* Callback for refuse test - stores ctx pointer */
static ax25_conn_t* s_refuse_ctx = NULL;

static void test_on_connect_refuse(ax25_address_t remote_addr, bool is_local_initiated, void* user_data) {
    (void)remote_addr;
    test_state_t* state = (test_state_t*)user_data;
    state->connect_count++;
    state->last_connect_local = is_local_initiated;
    if (state->refuse_connection && s_refuse_ctx) {
        ax25_conn_refuse(s_refuse_ctx, NULL, 0);
    }
}

TEST_CASE("AX25Conn: refuse incoming connection", "[ax25_conn]")
{
    reset_test_state();
    s_test_state.refuse_connection = true;

    ax25_conn_t ctx;
    s_refuse_ctx = &ctx;

    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);

    /* Custom on_connect that refuses */
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    cbs.on_connect = test_on_connect_refuse;

    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Receive SABM from remote */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);

    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Should remain disconnected and send DM */
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    wait_for_connect_count(1);
    wait_for_frame_count(1);

    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_CTRL_DM | AX25_CTRL_PF_BIT, sent.control);

    ax25_conn_deinit(&ctx);
    s_refuse_ctx = NULL;
    cleanup_test_state();
}

TEST_CASE("AX25Conn: refuse flag is cleared for later incoming connection", "[ax25_conn]")
{
    reset_test_state();
    s_test_state.refuse_connection = true;

    ax25_conn_t ctx;
    s_refuse_ctx = &ctx;

    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);

    ax25_conn_callbacks_t cbs = make_test_callbacks();
    cbs.on_connect = test_on_connect_refuse;

    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);

    ax25_conn_on_frame(&ctx, &sabm_frame);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(AX25_CTRL_DM | AX25_CTRL_PF_BIT, s_test_state.last_frame.control);

    s_test_state.refuse_connection = false;
    s_test_state.frame_count = 0;

    ax25_conn_on_frame(&ctx, &sabm_frame);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    wait_for_frame_count(1);
    wait_for_connect_count(2);
    TEST_ASSERT_EQUAL(AX25_CTRL_UA | AX25_CTRL_PF_BIT, s_test_state.last_frame.control);

    ax25_conn_deinit(&ctx);
    s_refuse_ctx = NULL;
    cleanup_test_state();
}

TEST_CASE("AX25Conn: local on_connect can re-enter send_data", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    cbs.on_connect = test_on_connect_send_data;
    s_test_state.callback_ctx = &ctx;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_connect(&ctx, &remote));
    wait_for_frame_count(1);

    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                             AX25_CTRL_UA | AX25_CTRL_PF_BIT,
                                             false,
                                             NULL,
                                             0);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_on_frame(&ctx, &ua_frame));
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    wait_for_connect_count(1);
    wait_for_callback_reenter_count(1);
    TEST_ASSERT_EQUAL(ESP_OK, s_test_state.callback_result);
    wait_for_frame_count(2);
    TEST_ASSERT_EQUAL(AX25_FRAME_I, s_test_state.frame_history[1].type);
    TEST_ASSERT_EQUAL_UINT8(2, s_test_state.frame_history[1].payload_len);
    TEST_ASSERT_EQUAL_UINT8('O', s_test_state.frame_history[1].payload[0]);
    TEST_ASSERT_EQUAL_UINT8('K', s_test_state.frame_history[1].payload[1]);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: deinit quiesces queued dispatcher callbacks", "[ax25_conn]")
{
    reset_test_state();
    memset(&s_dispatch_timer_enqueue_state, 0, sizeof(s_dispatch_timer_enqueue_state));

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    esp_timer_handle_t enqueue_timer = NULL;
    esp_timer_create_args_t timer_args = {
        .callback = test_dispatch_enqueue_timer_cb,
        .arg = &s_dispatch_timer_enqueue_state,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "conn_disp_test",
    };

    cbs.on_tx_frame = test_on_frame_blocking;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg));

    s_dispatch_timer_enqueue_state.ctx = &ctx;

    s_dispatch_timer_enqueue_state.first_action.type = AX25_CONN_DISPATCH_ACTION_FRAME;
    s_dispatch_timer_enqueue_state.first_action.payload.frame = build_test_frame("N0CALL-1",
                                                                                 "N0CALL",
                                                                                 ax25_frame_build_rr_control(0, false),
                                                                                 true,
                                                                                 NULL,
                                                                                 0);
    s_dispatch_timer_enqueue_state.second_action = s_dispatch_timer_enqueue_state.first_action;

    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_create(&timer_args, &enqueue_timer));
    TEST_ASSERT_EQUAL(ESP_OK, esp_timer_start_once(enqueue_timer, 1000));
    TEST_ASSERT_EQUAL(pdTRUE,
                      xSemaphoreTake(s_test_state.callback_enter_sem, pdMS_TO_TICKS(TEST_CONN_WAIT_MS)));
    wait_for_counter_value(&s_dispatch_timer_enqueue_state.enqueue_count, 2);
    TEST_ASSERT_EQUAL(0, s_dispatch_timer_enqueue_state.enqueue_error_count);

    TEST_ASSERT_EQUAL(pdPASS,
                      xTaskCreate(release_callback_task,
                                  "conn_cb_rel",
                                  2048,
                                  &s_test_state,
                                  5,
                                  NULL));

    ax25_conn_deinit(&ctx);
    esp_timer_stop(enqueue_timer);
    esp_timer_delete(enqueue_timer);
    vTaskDelay(pdMS_TO_TICKS(20));

    wait_for_counter_value(&s_test_state.frame_count, 1);
    cleanup_test_state();
}

/*******************************************************************************
 * Data Transfer Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: on_data can re-enter get_state", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    const uint8_t payload[] = {'H', 'i'};

    cbs.on_data = test_on_data_get_state;
    s_test_state.callback_ctx = &ctx;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                               AX25_CTRL_SABM | AX25_CTRL_PF_BIT,
                                               true,
                                               NULL,
                                               0);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_on_frame(&ctx, &sabm_frame));
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));

    ax25_frame_t iframe = build_test_frame("N0CALL", "N0CALL-1",
                                           ax25_frame_build_i_control(0, 0, false),
                                           true,
                                           payload,
                                           sizeof(payload));

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_on_frame(&ctx, &iframe));
    wait_for_data_count(1);
    wait_for_callback_reenter_count(1);
    TEST_ASSERT_EQUAL(ESP_OK, s_test_state.callback_result);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, s_test_state.callback_state);
    TEST_ASSERT_EQUAL_UINT8(sizeof(payload), s_test_state.last_data_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, s_test_state.last_data, sizeof(payload));

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: send data queues I frame", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    /* Complete connection */
    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    /* Reset frame count */
    s_test_state.frame_count = 0;

    /* Send data */
    const char* test_data = "Hello, AX.25!";
    esp_err_t err = ax25_conn_send_data(&ctx, (const uint8_t*)test_data, strlen(test_data));
    TEST_ASSERT_EQUAL(ESP_OK, err);
    wait_for_frame_count(1);

    /* Verify I frame was sent */
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_I, sent.type);
    TEST_ASSERT_EQUAL(strlen(test_data), sent.payload_len);
    TEST_ASSERT_EQUAL_MEMORY(test_data, sent.payload, sent.payload_len);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: connected data frame keeps configured digipeater path", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_address_t digipeaters[1] = { make_address("RELAY-1") };
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote, digipeaters, 1);

    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                             AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    s_test_state.frame_count = 0;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"X", 1));
    wait_for_frame_count(1);

    TEST_ASSERT_EQUAL(1, s_test_state.last_frame.num_digipeaters);
    TEST_ASSERT_TRUE(ax25_address_equals(&s_test_state.last_frame.digipeaters[0], &digipeaters[0]));

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: receive I frame delivers data", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection via incoming SABM */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Receive I frame with data */
    const char* test_data = "Test message";
    uint8_t i_ctrl = ax25_frame_build_i_control(0, 0, false);  /* N(S)=0, N(R)=0 */

    ax25_frame_t iframe;
    ax25_frame_init(&iframe);
    ax25_address_from_string("N0CALL", &iframe.destination);
    ax25_address_from_string("N0CALL-1", &iframe.source);
    iframe.is_command = true;
    iframe.type = AX25_FRAME_I;
    iframe.control = i_ctrl;
    iframe.pid = AX25_PID_NONE;
    memcpy(iframe.payload, test_data, strlen(test_data));
    iframe.payload_len = strlen(test_data);

    ax25_conn_on_frame(&ctx, &iframe);

    wait_for_data_count(1);
    TEST_ASSERT_EQUAL(strlen(test_data), s_test_state.last_data_len);
    TEST_ASSERT_EQUAL_MEMORY(test_data, s_test_state.last_data, s_test_state.last_data_len);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: send data rejects when not connected", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    const char* test_data = "Hello";
    esp_err_t err = ax25_conn_send_data(&ctx, (const uint8_t*)test_data, strlen(test_data));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: sequence numbers increment correctly", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.window_size = 7;  /* Allow more outstanding frames */

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    /* Complete connection */
    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    s_test_state.frame_count = 0;
    s_test_state.frame_history_count = 0;

    /* Send multiple frames */
    for (int i = 0; i < 4; i++) {
        char data[16];
        snprintf(data, sizeof(data), "MSG%d", i);
        ax25_conn_send_data(&ctx, (const uint8_t*)data, strlen(data));
    }

    wait_for_frame_count_at_least(4);

    /* Verify sequence numbers */
    for (int i = 0; i < 4; i++) {
        ax25_frame_t sent = s_test_state.frame_history[i];
        uint8_t ns = ax25_frame_extract_ns(sent.control);
        TEST_ASSERT_EQUAL(i, ns);
    }

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Disconnection Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: shutdown sends DISC", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    /* Complete connection */
    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    s_test_state.frame_count = 0;

    /* Initiate shutdown */
    esp_err_t err = ax25_conn_shutdown(&ctx);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_RELEASE, ax25_conn_get_state(&ctx));

    /* Verify DISC was sent */
    wait_for_frame_count(1);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_CTRL_DISC | AX25_CTRL_PF_BIT, sent.control);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: UA response to DISC completes disconnection", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    /* Complete connection */
    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    ax25_conn_shutdown(&ctx);

    /* Send UA response to DISC */
    ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                               AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    wait_for_disconnect_count(1);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: remote DISC disconnects", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    s_test_state.frame_count = 0;

    /* Receive DISC from remote */
    ax25_frame_t disc_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_DISC | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &disc_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    wait_for_disconnect_count(1);

    /* Verify UA was sent */
    wait_for_frame_count(1);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_CTRL_UA | AX25_CTRL_PF_BIT, sent.control);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: remote DISC dispatches disconnect only once", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    ax25_frame_t disc_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_DISC | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &disc_frame);

    wait_for_disconnect_count(1);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_shutdown(&ctx));
    wait_for_disconnect_count(1);

    ax25_frame_t dm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_DM | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &dm_frame);
    wait_for_disconnect_count(1);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: shutdown while awaiting release is idempotent", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_shutdown(&ctx));
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_RELEASE, ax25_conn_get_state(&ctx));
    int frame_count_after_first_shutdown = s_test_state.frame_count;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_shutdown(&ctx));
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_RELEASE, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(frame_count_after_first_shutdown, s_test_state.frame_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: DISC while awaiting release sends UA and disconnects", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_shutdown(&ctx));
    s_test_state.frame_count = 0;

    ax25_frame_t disc_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_DISC | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &disc_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    wait_for_disconnect_count(1);
    wait_for_final_count(1);
    wait_for_frame_count(1);
    TEST_ASSERT_EQUAL(AX25_CTRL_UA | AX25_CTRL_PF_BIT, s_test_state.last_frame.control);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Timeout Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: T1 timeout retransmits SABM", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 50;  /* Very short for testing */
    cfg.n2_retries = 3;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    wait_for_frame_count(1);

    /* Wait for T1 timeout and retransmit */
    vTaskDelay(pdMS_TO_TICKS(100));

    wait_for_frame_count_at_least(2);

    /* All transmitted frames should be SABM */
    for (int i = 0; i < s_test_state.frame_history_count; i++) {
        ax25_frame_t sent = s_test_state.frame_history[i];
        TEST_ASSERT_EQUAL(AX25_CTRL_SABM | AX25_CTRL_PF_BIT, sent.control);
    }

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: N2 exceeded gives up connection", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 30;
    cfg.n2_retries = 2;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    /* Wait for N2 retries to be exhausted */
    vTaskDelay(pdMS_TO_TICKS(200));

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    wait_for_error_count(1);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, s_test_state.last_error_code);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: T3 timeout polls remote", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t3_ms = 50;  /* Very short for testing */
    cfg.t1_ms = 200;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    int initial_frames = s_test_state.frame_count;

    /* Wait for T3 timeout */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Should have sent RR poll and entered timer recovery */
    wait_for_frame_count_at_least(initial_frames + 1);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_TIMER_RECOVERY, ax25_conn_get_state(&ctx));

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Supervisory Frame Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: RR acknowledges frames", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    /* Complete connection */
    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    /* Send data (queues I frame with N(S)=0) */
    ax25_conn_send_data(&ctx, (const uint8_t*)"test", 4);

    TEST_ASSERT_EQUAL(1, ctx.vs);  /* V(S) should be 1 */
    TEST_ASSERT_EQUAL(0, ctx.va);  /* V(A) should be 0 (unacked) */

    /* Receive RR with N(R)=1 acknowledging our frame */
    uint8_t rr_ctrl = ax25_frame_build_rr_control(1, false);
    ax25_frame_t rr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      rr_ctrl, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rr_frame);

    TEST_ASSERT_EQUAL(1, ctx.va);  /* V(A) should now be 1 */

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: partial RR ack restarts T1", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 80;
    cfg.t3_ms = 500;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    ax25_conn_send_data(&ctx, (const uint8_t*)"msg0", 4);
    ax25_conn_send_data(&ctx, (const uint8_t*)"msg1", 4);
    TEST_ASSERT_EQUAL(2, ctx.vs);
    TEST_ASSERT_EQUAL(0, ctx.va);

    vTaskDelay(pdMS_TO_TICKS(30));

    uint8_t rr_ctrl = ax25_frame_build_rr_control(1, false);
    ax25_frame_t rr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      rr_ctrl, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rr_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(1, ctx.va);

    int frames_before_wait = s_test_state.frame_count;
    vTaskDelay(pdMS_TO_TICKS(60));

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(frames_before_wait, s_test_state.frame_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: RNR sets peer busy", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    TEST_ASSERT_FALSE(ctx.peer_busy);

    /* Receive RNR */
    uint8_t rnr_ctrl = ax25_frame_build_rnr_control(0, false);
    ax25_frame_t rnr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                       rnr_ctrl, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rnr_frame);

    TEST_ASSERT_TRUE(ctx.peer_busy);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: peer busy clears on RR and queued send resumes", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"A", 1));
    int frames_after_first_send = s_test_state.frame_count;

    uint8_t rnr_ctrl = ax25_frame_build_rnr_control(0, false);
    ax25_frame_t rnr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                       rnr_ctrl, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rnr_frame);
    TEST_ASSERT_TRUE(ctx.peer_busy);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"B", 1));
    TEST_ASSERT_EQUAL(frames_after_first_send, s_test_state.frame_count);

    uint8_t rr_ctrl = ax25_frame_build_rr_control(1, false);
    ax25_frame_t rr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      rr_ctrl, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rr_frame);

    TEST_ASSERT_FALSE(ctx.peer_busy);
    wait_for_frame_count_at_least(frames_after_first_send + 1);
    TEST_ASSERT_EQUAL(AX25_FRAME_I, s_test_state.last_frame.type);
    TEST_ASSERT_EQUAL_UINT32(1, s_test_state.last_frame.payload_len);
    TEST_ASSERT_EQUAL_UINT8('B', s_test_state.last_frame.payload[0]);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: REJ triggers retransmission", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.window_size = 4;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    /* Send two frames */
    ax25_conn_send_data(&ctx, (const uint8_t*)"msg0", 4);
    ax25_conn_send_data(&ctx, (const uint8_t*)"msg1", 4);

    int frames_before = s_test_state.frame_count;

    /* Receive REJ with N(R)=0 (reject both frames) */
    uint8_t rej_ctrl = ax25_frame_build_rej_control(0, false);
    ax25_frame_t rej_frame = build_test_frame("N0CALL", "N0CALL-1",
                                       rej_ctrl, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rej_frame);

    /* Should retransmit from N(R)=0 */
    TEST_ASSERT_TRUE(s_test_state.frame_count >= frames_before);
    TEST_ASSERT_EQUAL(2, ctx.vs);  /* V(S) stays at 2; frames retransmitted by unmarking transmitted flag */
    TEST_ASSERT_EQUAL(0, ctx.va);  /* N(R)=0 acknowledges no queued I-frames */

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Error Handling Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: FRMR received triggers link reset", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    s_test_state.frame_count = 0;

    /* Receive FRMR */
    ax25_frame_t frmr;
    ax25_frame_init(&frmr);
    ax25_address_from_string("N0CALL", &frmr.destination);
    ax25_address_from_string("N0CALL-1", &frmr.source);
    frmr.is_command = false;
    frmr.type = AX25_FRAME_U;
    frmr.control = AX25_CTRL_FRMR;
    frmr.payload[0] = 0;
    frmr.payload[1] = 0;
    frmr.payload[2] = 0x01;  /* W bit set */
    frmr.payload_len = 3;

    ax25_conn_on_frame(&ctx, &frmr);

    /* Should trigger error and attempt link reset */
    wait_for_error_count(1);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_CONNECTION, ax25_conn_get_state(&ctx));

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: DM while connected triggers disconnect", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Receive unexpected DM */
    ax25_frame_t dm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_DM | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &dm_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    wait_for_error_count(1);
    wait_for_disconnect_count(1);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: invalid N(R) starts active recovery", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    s_test_state.frame_count = 0;

    /* V(S) and V(A) are both 0, so N(R)=5 is invalid. */
    uint8_t rr_ctrl = ax25_frame_build_rr_control(5, false);
    ax25_frame_t rr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      rr_ctrl, false, NULL, 0);

    esp_err_t ret = ax25_conn_on_frame(&ctx, &rr_frame);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, ret);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_CONNECTION, ax25_conn_get_state(&ctx));
    wait_for_error_count(1);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, s_test_state.last_error_code);

    /* Active recovery should restart the link with SABM rather than only sending FRMR. */
    wait_for_frame_count(1);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_CTRL_SABM | AX25_CTRL_PF_BIT, sent.control);
    TEST_ASSERT_TRUE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Frame Filtering Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: ignores frames not addressed to us", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Send frame addressed to different station */
    ax25_frame_t sabm_frame = build_test_frame("NOCALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);

    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Should remain disconnected */
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(0, s_test_state.frame_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: ignores frames with unresolved digipeater path", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    sabm_frame.num_digipeaters = 1;
    sabm_frame.digipeaters[0] = make_address("RELAY-1");
    sabm_frame.digipeaters[0].has_been_repeated = false;

    ax25_conn_on_frame(&ctx, &sabm_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(0, s_test_state.connect_count);
    TEST_ASSERT_EQUAL(0, s_test_state.frame_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: ignores UI frames", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Send UI frame */
    ax25_frame_t ui;
    ax25_frame_init(&ui);
    ax25_address_from_string("N0CALL", &ui.destination);
    ax25_address_from_string("N0CALL-1", &ui.source);
    ui.is_command = true;
    ui.type = AX25_FRAME_UI;
    ui.control = AX25_CTRL_UI;
    ui.pid = AX25_PID_NONE;
    const char* msg = "UI message";
    memcpy(ui.payload, msg, strlen(msg));
    ui.payload_len = strlen(msg);

    ax25_conn_on_frame(&ctx, &ui);

    /* Should remain disconnected, no data callback */
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(0, s_test_state.data_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: unsupported SABME is ignored in disconnected state", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* SABME is unsupported by this implementation; keep disconnected. */
    ax25_frame_t sabme_frame = build_test_frame("N0CALL", "N0CALL-1",
                                         0x6F | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabme_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(0, s_test_state.connect_count);
    TEST_ASSERT_EQUAL(0, s_test_state.frame_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Out of Sequence Frame Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: out of sequence I frame triggers REJ", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    s_test_state.frame_count = 0;

    /* Send I frame with N(S)=1 when we expect N(S)=0 */
    uint8_t i_ctrl = ax25_frame_build_i_control(1, 0, false);

    ax25_frame_t iframe;
    ax25_frame_init(&iframe);
    ax25_address_from_string("N0CALL", &iframe.destination);
    ax25_address_from_string("N0CALL-1", &iframe.source);
    iframe.is_command = true;
    iframe.type = AX25_FRAME_I;
    iframe.control = i_ctrl;
    iframe.pid = AX25_PID_NONE;
    iframe.payload[0] = 'X';
    iframe.payload_len = 1;

    ax25_conn_on_frame(&ctx, &iframe);

    /* Should send REJ */
    wait_for_frame_count(1);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_S, sent.type);
    TEST_ASSERT_EQUAL(AX25_CTRL_REJ_MASK, sent.control & 0x0F);

    /* Data should not be delivered */
    TEST_ASSERT_EQUAL(0, s_test_state.data_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Poll/Final Bit Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: responds to RR poll with RR final", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    s_test_state.frame_count = 0;

    /* Send RR command with P bit */
    uint8_t rr_ctrl = ax25_frame_build_rr_control(0, true);
    ax25_frame_t rr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      rr_ctrl, true, NULL, 0);  /* command */
    ax25_conn_on_frame(&ctx, &rr_frame);

    /* Should respond with RR response with F bit */
    wait_for_frame_count(1);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_S, sent.type);
    TEST_ASSERT_EQUAL(AX25_CTRL_RR_MASK, sent.control & 0x0F);
    TEST_ASSERT_TRUE(ax25_frame_has_pf(sent.control));
    TEST_ASSERT_FALSE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Timer Recovery Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: RR response with F exits timer recovery", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 30;
    cfg.t3_ms = 500;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Queue outstanding I-frames so timer-recovery exit must trigger retransmit path. */
    ax25_conn_send_data(&ctx, (const uint8_t*)"msg0", 4);
    ax25_conn_send_data(&ctx, (const uint8_t*)"msg1", 4);
    TEST_ASSERT_EQUAL(2, ctx.vs);

    /* Outstanding data uses T1; wait for timeout to enter timer recovery. */
    vTaskDelay(pdMS_TO_TICKS(80));
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_TIMER_RECOVERY, ax25_conn_get_state(&ctx));

    /* Send RR response with F bit and N(R)=1 (ack first frame, keep one outstanding). */
    uint8_t rr_ctrl = ax25_frame_build_rr_control(1, true);
    ax25_frame_t rr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      rr_ctrl, false, NULL, 0);  /* response */
    ax25_conn_on_frame(&ctx, &rr_frame);

    /* Should return to connected state */
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(1, ctx.va);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: timer recovery success resets retry counter", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 30;
    cfg.t3_ms = 500;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);
    ax25_conn_send_data(&ctx, (const uint8_t*)"msg0", 4);

    int waited_ms = 0;
    while (ax25_conn_get_state(&ctx) != AX25_CONN_STATE_TIMER_RECOVERY && waited_ms < 200) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
    }
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_TIMER_RECOVERY, ax25_conn_get_state(&ctx));

    waited_ms = 0;
    while (ctx.retry_count == 0 && waited_ms < 50) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
    }
    TEST_ASSERT_GREATER_THAN_UINT32(0, (uint32_t)ctx.retry_count);

    uint8_t rr_ctrl = ax25_frame_build_rr_control(1, true);
    ax25_frame_t rr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      rr_ctrl, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rr_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(0, ctx.retry_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: RR command with P in timer recovery does not retransmit I", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 30;
    cfg.t3_ms = 500;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    ax25_conn_send_data(&ctx, (const uint8_t*)"msg0", 4);

    vTaskDelay(pdMS_TO_TICKS(80));
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_TIMER_RECOVERY, ax25_conn_get_state(&ctx));

    int frames_before = s_test_state.frame_count;

    uint8_t rr_ctrl = ax25_frame_build_rr_control(0, true);
    ax25_frame_t rr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      rr_ctrl, true, NULL, 0);  /* command */
    ax25_conn_on_frame(&ctx, &rr_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_TIMER_RECOVERY, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(frames_before + 1, s_test_state.frame_count);

    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_S, sent.type);
    TEST_ASSERT_EQUAL(AX25_CTRL_RR_MASK, sent.control & 0x0F);
    TEST_ASSERT_TRUE(ax25_frame_has_pf(sent.control));
    TEST_ASSERT_FALSE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: timer recovery N2 timeout sends DM", "[ax25_conn][timeout]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t1_ms = 30;
    cfg.t3_ms = 500;
    cfg.n2_retries = 1;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    ax25_conn_send_data(&ctx, (const uint8_t*)"msg0", 4);

    vTaskDelay(pdMS_TO_TICKS(120));

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(1, s_test_state.error_count);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, s_test_state.last_error_code);

    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_CTRL_DM | AX25_CTRL_PF_BIT, sent.control);
    TEST_ASSERT_FALSE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Connection Collision Test
 ******************************************************************************/

TEST_CASE("AX25Conn: SABM collision handled", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* We initiate connection */
    ax25_conn_connect(&ctx, &remote);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_CONNECTION, ax25_conn_get_state(&ctx));

    /* Remote also sends SABM (collision) */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Should accept their SABM and become connected */
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    wait_for_connect_count(1);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: DISC while awaiting connection sends DM", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_CONNECTION, ax25_conn_get_state(&ctx));

    ax25_frame_t disc_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_DISC | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &disc_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_AWAITING_CONNECTION, ax25_conn_get_state(&ctx));

    wait_for_frame_count(2);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_CTRL_DM | AX25_CTRL_PF_BIT, sent.control);
    TEST_ASSERT_FALSE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Link Reset Test
 ******************************************************************************/

TEST_CASE("AX25Conn: SABM while connected resets link", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Modify state to simulate activity */
    ctx.vs = 3;
    ctx.vr = 2;

    s_test_state.frame_count = 0;

    /* Receive another SABM (link reset) */
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Should remain connected but state vars reset */
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(0, ctx.vs);
    TEST_ASSERT_EQUAL(0, ctx.vr);

    /* Should have sent UA */
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_CTRL_UA | AX25_CTRL_PF_BIT, sent.control);
    TEST_ASSERT_EQUAL(1, s_test_state.connect_count);
    TEST_ASSERT_EQUAL(0, s_test_state.disconnect_count);
    TEST_ASSERT_EQUAL(1, s_test_state.link_reset_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Edge Cases
 ******************************************************************************/

TEST_CASE("AX25Conn: shutdown when already disconnected", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    esp_err_t err = ax25_conn_shutdown(&ctx);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(0, s_test_state.frame_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: empty data rejected", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Try to send empty data */
    esp_err_t err = ax25_conn_send_data(&ctx, NULL, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);

    err = ax25_conn_send_data(&ctx, (const uint8_t*)"test", 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: oversized data rejected", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Try to send oversized data */
    uint8_t big_data[AX25_MAX_INFO_LEN + 10];
    memset(big_data, 'X', sizeof(big_data));

    esp_err_t err = ax25_conn_send_data(&ctx, big_data, sizeof(big_data));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, err);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: get remote addr", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote_out;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* No remote set yet */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_conn_get_remote_addr(&ctx, &remote_out));

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Now should work */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_get_remote_addr(&ctx, &remote_out));
    TEST_ASSERT_TRUE(ax25_address_equals(&remote_out, &ctx.remote_addr));

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: get remote addr rejects uninitialized and deinitialized ctx", "[ax25_conn]")
{
    ax25_conn_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ax25_address_t remote_out;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_get_remote_addr(&ctx, &remote_out));

    reset_test_state();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg));
    ax25_conn_deinit(&ctx);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_get_remote_addr(&ctx, &remote_out));

    cleanup_test_state();
}

TEST_CASE("AX25Conn: get path returns direct link with zero digipeaters", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_conn_path_t path;
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_conn_get_path(&ctx, &path));

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_connect(&ctx, &remote));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_get_path(&ctx, &path));
    TEST_ASSERT_EQUAL_UINT8(0, path.num_digipeaters);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: get path returns configured via chain normalized", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local = make_address("N0CALL");
    ax25_address_t remote = make_address("N0CALL-1");
    ax25_address_t digipeaters[2] = {
        make_address("RELAY1-1"),
        make_address("RELAY2-2"),
    };
    ax25_conn_path_t path;
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    digipeaters[0].has_been_repeated = true;
    digipeaters[1].has_been_repeated = true;

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_connect(&ctx, &remote, digipeaters, 2));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_get_path(&ctx, &path));

    TEST_ASSERT_EQUAL_UINT8(2, path.num_digipeaters);
    TEST_ASSERT_TRUE(ax25_address_equals(&path.digipeaters[0], &digipeaters[0]));
    TEST_ASSERT_TRUE(ax25_address_equals(&path.digipeaters[1], &digipeaters[1]));
    TEST_ASSERT_FALSE(path.digipeaters[0].has_been_repeated);
    TEST_ASSERT_FALSE(path.digipeaters[1].has_been_repeated);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: get path rejects uninitialized and deinitialized ctx", "[ax25_conn]")
{
    ax25_conn_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ax25_conn_path_t path;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_get_path(&ctx, &path));

    reset_test_state();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg));
    ax25_conn_deinit(&ctx);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_get_path(&ctx, &path));

    cleanup_test_state();
}

TEST_CASE("AX25Conn: ignores frames from non-session source", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    s_test_state.frame_count = 0;
    s_test_state.data_count = 0;

    ax25_frame_t wrong_source = build_test_frame("N0CALL", "N0CALL-2",
                                                 ax25_frame_build_rr_control(0, false),
                                                 false,
                                                 NULL,
                                                 0);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_on_frame(&ctx, &wrong_source));

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(0, s_test_state.frame_count);
    TEST_ASSERT_EQUAL(0, s_test_state.data_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: malformed frame rejected", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* NULL frame pointer must be rejected */
    esp_err_t err = ax25_conn_on_frame(&ctx, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: on_tx_frame rejects uninitialized and deinitialized ctx", "[ax25_conn]")
{
    ax25_conn_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ax25_frame_t frame = build_test_frame("N0CALL", "N0CALL-1",
                                          AX25_CTRL_SABM | AX25_CTRL_PF_BIT,
                                          true,
                                          NULL,
                                          0);
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_on_frame(&ctx, &frame));

    reset_test_state();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg));
    ax25_conn_deinit(&ctx);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_on_frame(&ctx, &frame));

    cleanup_test_state();
}

TEST_CASE("AX25Conn: shutdown rejects uninitialized and deinitialized ctx", "[ax25_conn]")
{
    ax25_conn_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_shutdown(&ctx));

    reset_test_state();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg));
    ax25_conn_deinit(&ctx);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_shutdown(&ctx));

    cleanup_test_state();
}

TEST_CASE("AX25Conn: refuse rejects uninitialized and deinitialized ctx", "[ax25_conn]")
{
    ax25_conn_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_refuse(&ctx, NULL, 0));

    reset_test_state();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg));
    ax25_conn_deinit(&ctx);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_conn_refuse(&ctx, NULL, 0));

    cleanup_test_state();
}

/*******************************************************************************
 * Window Size Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: window size limits outstanding frames", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local, remote;
    ax25_address_from_string("N0CALL", &local);
    ax25_address_from_string("N0CALL-1", &remote);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.window_size = 2;  /* Small window for testing */

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);
    ax25_conn_connect(&ctx, &remote);

    /* Complete connection */
    ax25_frame_t ua_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      AX25_CTRL_UA | AX25_CTRL_PF_BIT, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &ua_frame);

    /* Send window_size frames */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"A", 1));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"B", 1));

    /* Third should fail - window full */
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, ax25_conn_send_data(&ctx, (const uint8_t*)"C", 1));

    /* Acknowledge first frame */
    uint8_t rr_ctrl = ax25_frame_build_rr_control(1, false);
    ax25_frame_t rr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      rr_ctrl, false, NULL, 0);
    ax25_conn_on_frame(&ctx, &rr_frame);

    /* Now should be able to send again */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_send_data(&ctx, (const uint8_t*)"C", 1));

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: init normalizes invalid window size", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx_low;
    ax25_conn_t ctx_high;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg_low = make_test_config();
    ax25_conn_config_t cfg_high = make_test_config();

    cfg_low.window_size = 0;
    cfg_high.window_size = 8;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_init(&ctx_low, &local, &cbs, &s_test_state, &cfg_low));
    TEST_ASSERT_EQUAL(4, ctx_low.config.window_size);
    ax25_conn_deinit(&ctx_low);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_init(&ctx_high, &local, &cbs, &s_test_state, &cfg_high));
    TEST_ASSERT_EQUAL(4, ctx_high.config.window_size);
    ax25_conn_deinit(&ctx_high);

    cleanup_test_state();
}

TEST_CASE("AX25Conn: disconnect clears remote session metadata", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_t remote_out;
    ax25_conn_path_t path;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                               AX25_CTRL_SABM | AX25_CTRL_PF_BIT,
                                               true,
                                               NULL,
                                               0);
    sabm_frame.num_digipeaters = 1;
    sabm_frame.digipeaters[0] = make_address("RELAY-1");
    sabm_frame.digipeaters[0].has_been_repeated = true;
    ax25_conn_on_frame(&ctx, &sabm_frame);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_get_remote_addr(&ctx, &remote_out));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_conn_get_path(&ctx, &path));

    ax25_frame_t disc_frame = build_test_frame("N0CALL", "N0CALL-1",
                                               AX25_CTRL_DISC | AX25_CTRL_PF_BIT,
                                               true,
                                               NULL,
                                               0);
    ax25_conn_on_frame(&ctx, &disc_frame);

    TEST_ASSERT_EQUAL(AX25_CONN_STATE_DISCONNECTED, ax25_conn_get_state(&ctx));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_conn_get_remote_addr(&ctx, &remote_out));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_conn_get_path(&ctx, &path));

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

/*******************************************************************************
 * Local Busy / RNR Flow Control Tests
 ******************************************************************************/

TEST_CASE("AX25Conn: set_busy sends RNR immediately when connected", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection via incoming SABM */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);
    TEST_ASSERT_EQUAL(AX25_CONN_STATE_CONNECTED, ax25_conn_get_state(&ctx));

    s_test_state.frame_count = 0;

    /* Signal busy */
    esp_err_t err = ax25_conn_set_busy(&ctx, true);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(ctx.local_busy);

    /* Should have sent RNR immediately */
    TEST_ASSERT_TRUE(s_test_state.frame_count >= 1);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_S, sent.type);
    TEST_ASSERT_EQUAL(AX25_CTRL_RNR_MASK, sent.control & 0x0F);
    TEST_ASSERT_FALSE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: clearing busy sends RR immediately when connected", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Enter busy state */
    ax25_conn_set_busy(&ctx, true);
    s_test_state.frame_count = 0;

    /* Clear busy — should send RR */
    ax25_conn_set_busy(&ctx, false);
    TEST_ASSERT_FALSE(ctx.local_busy);

    TEST_ASSERT_TRUE(s_test_state.frame_count >= 1);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_S, sent.type);
    TEST_ASSERT_EQUAL(AX25_CTRL_RR_MASK, sent.control & 0x0F);
    TEST_ASSERT_FALSE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: set_busy when disconnected sends no frame", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    esp_err_t err = ax25_conn_set_busy(&ctx, true);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(0, s_test_state.frame_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: set_busy rejects NULL ctx", "[ax25_conn]")
{
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ax25_conn_set_busy(NULL, true));
}

TEST_CASE("AX25Conn: I-frame poll with P bit sends RNR when locally busy", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Go locally busy */
    ax25_conn_set_busy(&ctx, true);
    s_test_state.frame_count = 0;

    /* Receive I frame with P bit set */
    uint8_t i_ctrl = ax25_frame_build_i_control(0, 0, true);  /* P=1 */
    ax25_frame_t iframe;
    ax25_frame_init(&iframe);
    ax25_address_from_string("N0CALL", &iframe.destination);
    ax25_address_from_string("N0CALL-1", &iframe.source);
    iframe.is_command = true;
    iframe.type = AX25_FRAME_I;
    iframe.control = i_ctrl;
    iframe.pid = AX25_PID_NONE;
    iframe.payload[0] = 'X';
    iframe.payload_len = 1;

    ax25_conn_on_frame(&ctx, &iframe);

    /* Should respond with an S-frame (RNR preferred while busy). */
    TEST_ASSERT_TRUE(s_test_state.frame_count >= 1);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_S, sent.type);
    uint8_t s_type = sent.control & 0x0F;
    TEST_ASSERT_TRUE(s_type == AX25_CTRL_RNR_MASK || s_type == AX25_CTRL_RR_MASK);
    TEST_ASSERT_TRUE(ax25_frame_has_pf(sent.control));
    TEST_ASSERT_FALSE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: T2 delayed ack sends RNR when locally busy", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();
    cfg.t2_ms = 30;  /* Very short T2 for testing */

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    /* Go locally busy - this will cancel any pending T2 ack */
    ax25_conn_set_busy(&ctx, true);

    /* Receive I frame with P=0 while locally busy - arms T2 */
    uint8_t i_ctrl = ax25_frame_build_i_control(0, 0, false);  /* N(S)=0, P=0 */
    ax25_frame_t iframe;
    ax25_frame_init(&iframe);
    ax25_address_from_string("N0CALL", &iframe.destination);
    ax25_address_from_string("N0CALL-1", &iframe.source);
    iframe.is_command = true;
    iframe.type = AX25_FRAME_I;
    iframe.control = i_ctrl;
    iframe.pid = AX25_PID_NONE;
    iframe.payload[0] = 'X';
    iframe.payload_len = 1;
    ax25_conn_on_frame(&ctx, &iframe);

    /* T2 should be armed and ack_pending set */
    TEST_ASSERT_TRUE(ctx.ack_pending);

    s_test_state.frame_count = 0;

    /* Wait for T2 to fire */
    vTaskDelay(pdMS_TO_TICKS(80));

    /* T2 fired — should have sent an S-frame acknowledgement. */
    TEST_ASSERT_TRUE(s_test_state.frame_count >= 1);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_S, sent.type);
    uint8_t s_type = sent.control & 0x0F;
    TEST_ASSERT_TRUE(s_type == AX25_CTRL_RNR_MASK || s_type == AX25_CTRL_RR_MASK);
    TEST_ASSERT_FALSE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: set_busy idempotent, no extra frames on repeated call", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    ax25_conn_set_busy(&ctx, true);
    s_test_state.frame_count = 0;

    /* Calling set_busy(true) again should not send another frame */
    ax25_conn_set_busy(&ctx, true);
    TEST_ASSERT_EQUAL(0, s_test_state.frame_count);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}

TEST_CASE("AX25Conn: RR poll response is RNR when locally busy", "[ax25_conn]")
{
    reset_test_state();

    ax25_conn_t ctx;
    ax25_address_t local;
    ax25_address_from_string("N0CALL", &local);
    ax25_conn_callbacks_t cbs = make_test_callbacks();
    ax25_conn_config_t cfg = make_test_config();

    ax25_conn_init(&ctx, &local, &cbs, &s_test_state, &cfg);

    /* Establish connection */
    ax25_frame_t sabm_frame = build_test_frame("N0CALL", "N0CALL-1",
                                        AX25_CTRL_SABM | AX25_CTRL_PF_BIT, true, NULL, 0);
    ax25_conn_on_frame(&ctx, &sabm_frame);

    ax25_conn_set_busy(&ctx, true);
    s_test_state.frame_count = 0;

    /* Peer sends RR command with P bit — we must respond with F bit */
    uint8_t rr_ctrl = ax25_frame_build_rr_control(0, true);  /* P=1 */
    ax25_frame_t rr_frame = build_test_frame("N0CALL", "N0CALL-1",
                                      rr_ctrl, true, NULL, 0);  /* command */
    ax25_conn_on_frame(&ctx, &rr_frame);

    /* Response should be RNR (not RR) with F bit */
    TEST_ASSERT_EQUAL(1, s_test_state.frame_count);
    ax25_frame_t sent = s_test_state.last_frame;
    TEST_ASSERT_EQUAL(AX25_FRAME_S, sent.type);
    TEST_ASSERT_EQUAL(AX25_CTRL_RNR_MASK, sent.control & 0x0F);
    TEST_ASSERT_TRUE(ax25_frame_has_pf(sent.control));
    TEST_ASSERT_FALSE(sent.is_command);

    ax25_conn_deinit(&ctx);
    cleanup_test_state();
}
