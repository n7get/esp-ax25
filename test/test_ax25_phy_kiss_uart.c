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
#include "ax25_frame.h"
#include "ax25_phy_kiss_uart.h"

typedef struct {
    int call_count;
    ax25_frame_t last_frame;
} uart_frame_capture_t;

static uart_frame_capture_t s_uart_capture;

static void on_uart_frame(const ax25_frame_t *frame, void *user_data)
{
    uart_frame_capture_t *capture = (uart_frame_capture_t *)user_data;
    if (capture == NULL || frame == NULL) {
        return;
    }

    capture->call_count++;
    capture->last_frame = *frame;
}

static void uart_rx_tap_noop(const uint8_t *data, size_t len, void *user_data)
{
    (void)data;
    (void)len;
    (void)user_data;
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

static void uart_phy_test_setup(void)
{
    memset(&s_uart_capture, 0, sizeof(s_uart_capture));
    ax25_cfg_deinit();
    ensure_nvs_ready();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_init(NULL, 0));
}

static void uart_phy_test_teardown(void)
{
    ax25_cfg_deinit();
}

static void config_set_or_fail(const char *parameter, const char *value)
{
    char err[96] = {0};
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                              ax25_cfg_set(parameter, value, err, sizeof(err)),
                              err);
}

static void make_ui_frame(ax25_frame_t *frame, const char *src, const char *dst,
                          const uint8_t *payload, size_t payload_len)
{
    ax25_frame_init(frame);
    frame->type = AX25_FRAME_UI;
    frame->control = AX25_CTRL_UI;
    frame->pid = AX25_PID_NONE;
    ax25_address_from_string(src, &frame->source);
    ax25_address_from_string(dst, &frame->destination);
    if (payload != NULL && payload_len > 0) {
        memcpy(frame->payload, payload, payload_len);
        frame->payload_len = payload_len;
    }
}

TEST_CASE("PHY UART: init rejects invalid arguments", "[ax25_phy_kiss_uart]")
{
    ax25_phy_kiss_uart_t phy = {0};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_uart_init(NULL, NULL, &phy));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_uart_init(on_uart_frame, NULL, NULL));
}

TEST_CASE("PHY UART: send and raw write reject invalid arguments", "[ax25_phy_kiss_uart]")
{
    ax25_phy_kiss_uart_t phy = {0};
    ax25_frame_t frame;

    make_ui_frame(&frame, "SRC", "DST", (const uint8_t *)"x", 1);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_uart_send(NULL, &phy));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_uart_send(&frame, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_uart_write_raw(NULL, 1, &phy));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_uart_write_raw((const uint8_t *)"x", 0, &phy));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_uart_write_raw((const uint8_t *)"x", 1, NULL));
}

TEST_CASE("PHY UART: init supports send raw write and RX tap lifecycle", "[ax25_phy_kiss_uart]")
{
    ax25_phy_kiss_uart_t phy = {0};
    ax25_frame_t frame;

    uart_phy_test_setup();
    config_set_or_fail("uart.no", "2");
    config_set_or_fail("uart.baud", "19200");
    config_set_or_fail("uart.tx_pin", "17");
    config_set_or_fail("uart.rx_pin", "18");

    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_uart_init(on_uart_frame, &s_uart_capture, &phy));
    TEST_ASSERT_TRUE(phy.running);
    TEST_ASSERT_NOT_NULL(phy.uart_rx_task_handle);
    TEST_ASSERT_NOT_NULL(phy.uart_tx_task_handle);
    TEST_ASSERT_NOT_NULL(phy.uart_tx_queue);
    TEST_ASSERT_NOT_NULL(phy.tx_mutex);

    ax25_phy_kiss_uart_set_rx_tap(uart_rx_tap_noop, &phy, &phy);
    TEST_ASSERT_EQUAL_PTR(uart_rx_tap_noop, phy.rx_tap);
    TEST_ASSERT_EQUAL_PTR(&phy, phy.rx_tap_user_data);

    ax25_phy_kiss_uart_clear_rx_tap(&phy);
    TEST_ASSERT_NULL(phy.rx_tap);
    TEST_ASSERT_NULL(phy.rx_tap_user_data);

    make_ui_frame(&frame, "SRC-1", "DST-1", (const uint8_t *)"uart", 4);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_uart_write_raw((const uint8_t *)"+++", 3, &phy));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_uart_send(&frame, &phy));
    TEST_ASSERT_TRUE(uxQueueMessagesWaiting(phy.uart_tx_queue) <= 1);

    ax25_phy_kiss_uart_deinit(&phy);
    TEST_ASSERT_FALSE(phy.running);
    TEST_ASSERT_NULL(phy.uart_rx_task_handle);
    TEST_ASSERT_NULL(phy.uart_tx_task_handle);
    TEST_ASSERT_NULL(phy.uart_tx_queue);
    TEST_ASSERT_NULL(phy.tx_mutex);

    uart_phy_test_teardown();
}

TEST_CASE("PHY UART: send reports invalid state after deinit", "[ax25_phy_kiss_uart]")
{
    ax25_phy_kiss_uart_t phy = {0};
    ax25_frame_t frame;

    uart_phy_test_setup();
    config_set_or_fail("uart.no", "2");
    config_set_or_fail("uart.baud", "19200");
    config_set_or_fail("uart.tx_pin", "17");
    config_set_or_fail("uart.rx_pin", "18");

    make_ui_frame(&frame, "SRC-1", "DST-1", (const uint8_t *)"uart", 4);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_uart_init(on_uart_frame, &s_uart_capture, &phy));
    ax25_phy_kiss_uart_deinit(&phy);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_phy_kiss_uart_send(&frame, &phy));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      ax25_phy_kiss_uart_write_raw((const uint8_t *)"+++", 3, &phy));

    uart_phy_test_teardown();
}