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
 * @file ax25_uart.h
 * @brief UART hardware initialisation helper
 *
 * Reads UART parameters from ax25_config, installs the ESP-IDF UART driver,
 * and configures the selected port's baud rate and pins.
 *
 * Configuration keys (with defaults):
 * - uart.no     (default 1)
 * - uart.baud   (default 9600)
 * - uart.tx_pin (default 6)
 * - uart.rx_pin (default 7)
 */

#ifndef AX25_UART_H
#define AX25_UART_H

#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise and configure a UART port from ax25_config.
 *
 * Reads uart.no, uart.baud, uart.tx_pin, and uart.rx_pin from ax25_config,
 * then installs the UART driver with a 2048-byte RX ring buffer and configures
 * baud rate and I/O pins.
 *
 * @param[out] uart_num_out  Receives the installed UART port number on success.
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if @p uart_num_out is NULL or any config value is out of range
 * @return Any ESP_ERR_* propagated from the UART driver
 */
esp_err_t ax25_uart_init(uart_port_t *uart_num_out);

/**
 * @brief Uninstall the UART driver for the given port.
 *
 * @param uart_num  Port previously initialised by ax25_uart_init().
 */
void ax25_uart_deinit(uart_port_t uart_num);

#ifdef __cplusplus
}
#endif

#endif /* AX25_UART_H */
