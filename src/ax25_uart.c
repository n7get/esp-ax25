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
 * @file ax25_uart.c
 * @brief UART hardware initialisation helper
 */

#include "esp_log.h"
#include "ax25_config.h"
#include "ax25_uart.h"

static const char *TAG = "AX25_UART";

#define AX25_UART_RX_BUF_SIZE 2048

esp_err_t ax25_uart_init(uart_port_t *uart_num_out)
{
    if (!uart_num_out) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!ax25_cfg_is_initialized()) {
        ESP_LOGW(TAG, "ax25_config not initialized; using defaults/fallbacks for uart.*");
    }

    int uart_num  = ax25_cfg_get_int_global("uart.no",     1);
    int baud_rate = ax25_cfg_get_int_global("uart.baud",   9600);
    int tx_pin    = ax25_cfg_get_int_global("uart.tx_pin", 6);
    int rx_pin    = ax25_cfg_get_int_global("uart.rx_pin", 7);

    ESP_LOGI(TAG, "Init: UART%d %d baud, TX=%d RX=%d", uart_num, baud_rate, tx_pin, rx_pin);

    if (uart_num < (int)UART_NUM_0 || uart_num >= (int)UART_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid uart.no=%d", uart_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (baud_rate <= 0 || tx_pin < 0 || rx_pin < 0) {
        ESP_LOGE(TAG, "Invalid UART config: baud=%d tx_pin=%d rx_pin=%d",
                 baud_rate, tx_pin, rx_pin);
        return ESP_ERR_INVALID_ARG;
    }

    const uart_config_t uart_cfg = {
        .baud_rate           = baud_rate,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    uart_port_t port = (uart_port_t)uart_num;

    esp_err_t err = uart_driver_install(port, AX25_UART_RX_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_param_config(port, &uart_cfg);
    if (err != ESP_OK) {
        uart_driver_delete(port);
        return err;
    }

    err = uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        uart_driver_delete(port);
        return err;
    }

    *uart_num_out = port;
    return ESP_OK;
}

void ax25_uart_deinit(uart_port_t uart_num)
{
    uart_driver_delete(uart_num);
}
