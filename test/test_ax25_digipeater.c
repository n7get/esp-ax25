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

#include "unity.h"

#include <string.h>

#include "nvs_flash.h"

#include "ax25_address.h"
#include "ax25_config.h"
#include "ax25_digipeater.h"
#include "ax25_frame.h"
#include "ax25_router.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    int call_count;
    ax25_frame_t last_frame;
} tx_ctx_t;

static tx_ctx_t s_tx;
static ax25_router_port_t s_test_source_port;

static esp_err_t tx_capture_cb(const ax25_frame_t *frame, void *user_data)
{
    tx_ctx_t *ctx = (tx_ctx_t *)user_data;
    if (ctx == NULL || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ctx->call_count++;
    ctx->last_frame = *frame;
    return ESP_OK;
}

static void wait_for_call_count(int expected)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (xTaskGetTickCount() < deadline) {
        if (s_tx.call_count == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL_INT(expected, s_tx.call_count);
}

static void config_set_str_or_fail(const char *parameter, const char *value)
{
    char err[96] = {0};
    esp_err_t rc = ax25_cfg_set(parameter, value, err, sizeof(err));
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, rc, err);
}

static void test_setup(void)
{
    memset(&s_tx, 0, sizeof(s_tx));
    memset(&s_test_source_port, 0, sizeof(s_test_source_port));

    /* A prior suite may leave singletons initialized; normalize baseline. */
    ax25_digipeater_deinit();
    ax25_router_deinit();
    ax25_cfg_deinit();

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_erase());
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_init());
    } else {
        TEST_ASSERT_EQUAL(ESP_OK, nvs_err);
    }

    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_init(NULL, 0));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_init());
}

static void test_teardown(void)
{
    ax25_digipeater_deinit();
    ax25_router_deinit();
    ax25_cfg_deinit();
}

TEST_CASE("Digipeater: empty digi.callsign keeps module disabled", "[ax25_digipeater]")
{
    test_setup();

    config_set_str_or_fail("digi.callsign", "");

    const ax25_digipeater_config_t cfg = {
        .on_transmit = tx_capture_cb,
        .user_data = &s_tx,
    };
    TEST_ASSERT_EQUAL(ESP_OK, ax25_digipeater_init(&cfg));
    TEST_ASSERT_TRUE(ax25_digipeater_is_initialized());

    ax25_frame_t frame;
    ax25_frame_init(&frame);
    ax25_address_from_string("DEST-0", &frame.destination);
    ax25_address_from_string("SRC-0", &frame.source);
    frame.type = AX25_FRAME_UI;
    frame.control = AX25_CTRL_UI;
    frame.pid = AX25_PID_NONE;
    ax25_address_from_string("RELAY-1", &frame.digipeaters[0]);
    frame.num_digipeaters = 1;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&frame, &s_test_source_port));
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL_INT(0, s_tx.call_count);

    test_teardown();
}

TEST_CASE("Digipeater: relays when first unrepeated path matches digi.callsign", "[ax25_digipeater]")
{
    test_setup();

    config_set_str_or_fail("digi.callsign", "RELAY-1");

    const ax25_digipeater_config_t cfg = {
        .on_transmit = tx_capture_cb,
        .user_data = &s_tx,
    };
    TEST_ASSERT_EQUAL(ESP_OK, ax25_digipeater_init(&cfg));

    ax25_frame_t frame;
    ax25_frame_init(&frame);
    ax25_address_from_string("DEST-0", &frame.destination);
    ax25_address_from_string("SRC-0", &frame.source);
    frame.type = AX25_FRAME_UI;
    frame.control = AX25_CTRL_UI;
    frame.pid = AX25_PID_NONE;
    ax25_address_from_string("RELAY-1", &frame.digipeaters[0]);
    frame.digipeaters[0].has_been_repeated = false;
    frame.num_digipeaters = 1;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&frame, &s_test_source_port));
    wait_for_call_count(1);
    TEST_ASSERT_TRUE(s_tx.last_frame.digipeaters[0].has_been_repeated);

    test_teardown();
}

TEST_CASE("Digipeater: ignores frame when first unrepeated hop does not match", "[ax25_digipeater]")
{
    test_setup();

    config_set_str_or_fail("digi.callsign", "RELAY-2");

    const ax25_digipeater_config_t cfg = {
        .on_transmit = tx_capture_cb,
        .user_data = &s_tx,
    };
    TEST_ASSERT_EQUAL(ESP_OK, ax25_digipeater_init(&cfg));

    ax25_frame_t frame;
    ax25_frame_init(&frame);
    ax25_address_from_string("DEST-0", &frame.destination);
    ax25_address_from_string("SRC-0", &frame.source);
    frame.type = AX25_FRAME_UI;
    frame.control = AX25_CTRL_UI;
    frame.pid = AX25_PID_NONE;

    ax25_address_from_string("RELAY-1", &frame.digipeaters[0]);
    frame.digipeaters[0].has_been_repeated = false;
    ax25_address_from_string("RELAY-2", &frame.digipeaters[1]);
    frame.digipeaters[1].has_been_repeated = false;
    frame.num_digipeaters = 2;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&frame, &s_test_source_port));
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL_INT(0, s_tx.call_count);

    test_teardown();
}

TEST_CASE("Digipeater: invalid digi.callsign is rejected", "[ax25_digipeater]")
{
    test_setup();

    config_set_str_or_fail("digi.callsign", "BAD!CALL");

    const ax25_digipeater_config_t cfg = {
        .on_transmit = tx_capture_cb,
        .user_data = &s_tx,
    };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_digipeater_init(&cfg));
    TEST_ASSERT_FALSE(ax25_digipeater_is_initialized());

    test_teardown();
}

