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

#include "nvs_flash.h"

#include "ax25_config.h"
#include "ax25_uart.h"

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

static void reset_config_state(void)
{
    ax25_cfg_deinit();
    ensure_nvs_ready();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_init(NULL, 0));
}

static void config_set_or_fail(const char *parameter, const char *value)
{
    char err[96] = {0};
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                              ax25_cfg_set(parameter, value, err, sizeof(err)),
                              err);
}

TEST_CASE("UART: init rejects null output pointer", "[ax25_uart]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_uart_init(NULL));
}

TEST_CASE("UART: init applies configured port and baud", "[ax25_uart]")
{
    reset_config_state();

    config_set_or_fail("uart.no", "2");
    config_set_or_fail("uart.baud", "19200");
    config_set_or_fail("uart.tx_pin", "17");
    config_set_or_fail("uart.rx_pin", "18");

    uart_port_t port = UART_NUM_0;
    uint32_t baud_rate = 0;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_uart_init(&port));
    TEST_ASSERT_EQUAL(UART_NUM_2, port);
    TEST_ASSERT_EQUAL(ESP_OK, uart_get_baudrate(port, &baud_rate));
    TEST_ASSERT_EQUAL_UINT32(19200, baud_rate);

    ax25_uart_deinit(port);
    ax25_cfg_deinit();
}