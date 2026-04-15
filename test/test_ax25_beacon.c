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
#include "ax25_beacon.h"
#include "ax25_config.h"
#include "ax25_frame.h"
#include "ax25_router.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    int call_count;
    ax25_frame_t last_frame;
} beacon_capture_t;

static beacon_capture_t s_capture;

static void beacon_capture_cb(const ax25_frame_t *frame, void *user_data)
{
    beacon_capture_t *capture = (beacon_capture_t *)user_data;
    if (capture == NULL || frame == NULL) {
        return;
    }

    capture->call_count++;
    capture->last_frame = *frame;
}

static void wait_for_capture_count(int expected)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (xTaskGetTickCount() < deadline) {
        if (s_capture.call_count == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL_INT(expected, s_capture.call_count);
}

static void ensure_nvs_ready(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_erase());
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_init());
    } else {
        TEST_ASSERT_EQUAL(ESP_OK, err);
    }
}

static void beacon_test_setup(void)
{
    memset(&s_capture, 0, sizeof(s_capture));

    ax25_beacon_deinit();
    ax25_router_deinit();
    ax25_cfg_deinit();

    ensure_nvs_ready();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_init(NULL, 0));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_init());
}

static void beacon_test_teardown(void)
{
    ax25_beacon_deinit();
    ax25_router_deinit();
    ax25_cfg_deinit();
}

static void config_set_or_fail(const char *parameter, const char *value)
{
    char err[96] = {0};
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                              ax25_cfg_set(parameter, value, err, sizeof(err)),
                              err);
}

TEST_CASE("Beacon: trigger sends configured frame with unescaped text", "[ax25_beacon]")
{
    ax25_router_port_t capture_port = {
        .mode = AX25_PORT_PROMISCUOUS,
        .on_tx_frame = beacon_capture_cb,
        .user_data = &s_capture,
    };
    ax25_address_t digi1;
    ax25_address_t digi2;
    char src_buf[16] = {0};
    char dst_buf[16] = {0};
    static const uint8_t expected_payload[] = "Line1\r\n!\\done";

    beacon_test_setup();

    config_set_or_fail("beacon.every", "1");
    config_set_or_fail("beacon.source", "N0CALL-1");
    config_set_or_fail("beacon.destination", "APRS");
    config_set_or_fail("beacon.via", " WIDE1-1 , TOOLONG7 , WIDE2-2 ");
    config_set_or_fail("beacon.text", "Line1\\r\\n\\x21\\\\done");

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&capture_port));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_beacon_init());
    TEST_ASSERT_TRUE(ax25_beacon_is_initialized());
    TEST_ASSERT_EQUAL(ESP_OK, ax25_beacon_trigger());

    wait_for_capture_count(1);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_address_from_string("WIDE1-1", &digi1));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_address_from_string("WIDE2-2", &digi2));

    TEST_ASSERT_EQUAL(ESP_OK, ax25_address_to_string(&s_capture.last_frame.source,
                                                     src_buf,
                                                     sizeof(src_buf)));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_address_to_string(&s_capture.last_frame.destination,
                                                     dst_buf,
                                                     sizeof(dst_buf)));
    TEST_ASSERT_EQUAL_STRING("N0CALL-1", src_buf);
    TEST_ASSERT_EQUAL_STRING("APRS", dst_buf);
    TEST_ASSERT_EQUAL_UINT8(AX25_CTRL_UI, s_capture.last_frame.control);
    TEST_ASSERT_EQUAL_UINT8(AX25_PID_NONE, s_capture.last_frame.pid);
    TEST_ASSERT_EQUAL_UINT8(2, s_capture.last_frame.num_digipeaters);
    TEST_ASSERT_TRUE(ax25_address_equals(&s_capture.last_frame.digipeaters[0], &digi1));
    TEST_ASSERT_TRUE(ax25_address_equals(&s_capture.last_frame.digipeaters[1], &digi2));
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_payload) - 1, s_capture.last_frame.payload_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_payload,
                                  s_capture.last_frame.payload,
                                  sizeof(expected_payload) - 1);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&capture_port));
    beacon_test_teardown();
}

TEST_CASE("Beacon: disabled period initializes but manual trigger is rejected", "[ax25_beacon]")
{
    beacon_test_setup();

    config_set_or_fail("beacon.every", "0");
    config_set_or_fail("beacon.source", "N0CALL");
    config_set_or_fail("beacon.destination", "APRS");

    TEST_ASSERT_EQUAL(ESP_OK, ax25_beacon_init());
    TEST_ASSERT_TRUE(ax25_beacon_is_initialized());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_beacon_trigger());

    beacon_test_teardown();
}

TEST_CASE("Beacon: invalid destination is rejected without routing a frame", "[ax25_beacon]")
{
    ax25_router_port_t capture_port = {
        .mode = AX25_PORT_PROMISCUOUS,
        .on_tx_frame = beacon_capture_cb,
        .user_data = &s_capture,
    };

    beacon_test_setup();

    config_set_or_fail("beacon.every", "1");
    config_set_or_fail("beacon.source", "N0CALL");
    config_set_or_fail("beacon.destination", "BAD!CALL");
    config_set_or_fail("beacon.text", "test");

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_register_port(&capture_port));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_beacon_init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_beacon_trigger());
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_EQUAL_INT(0, s_capture.call_count);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_remove_port(&capture_port));
    beacon_test_teardown();
}