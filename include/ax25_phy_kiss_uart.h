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
 * @file ax25_phy_kiss_uart.h
 * @brief UART KISS physical-layer driver
 *
 * Wraps a UART port and KISS framing into a simple physical-layer context.
 * Received KISS frames are parsed and delivered to a caller-supplied callback.
 * Outgoing AX.25 frames are KISS-encoded by ax25_phy_kiss_uart_send() and
 * queued for transmission from a dedicated TX task. Raw writes used by
 * console-mode passthrough remain synchronous.
 *
 * Typical usage
 * -------------
 * @code
 *   static ax25_phy_kiss_uart_t phy;
 *
 *   static void on_rx_frame(const ax25_frame_t *frame, void *user_data) {
 *       ax25_router_send(frame, (ax25_router_port_t *)user_data);
 *   }
 *
 *   ax25_phy_kiss_uart_init(on_rx_frame, &phy_port, &phy);
 *
 *   // Send a frame:
 *   ax25_phy_kiss_uart_send(&frame, &phy);
 *
 *   // Shutdown:
 *   ax25_phy_kiss_uart_deinit(&phy);
 * @endcode
 */

#ifndef AX25_PHY_KISS_UART_H
#define AX25_PHY_KISS_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ax25_kiss.h"
#include "ax25_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Context
 ******************************************************************************/

/**
 * @brief Raw RX tap callback
 *
 * When registered, this callback receives every byte read from the UART
 * instead of the KISS decoder.  Used by console mode to relay TNC output to
 * the connected station.
 *
 * @param data      Pointer to received bytes
 * @param len       Number of bytes
 * @param user_data Caller-supplied opaque pointer
 */
typedef void (*ax25_phy_uart_rx_tap_t)(const uint8_t *data, size_t len, void *user_data);

/**
 * @brief Runtime context for a UART KISS physical-layer instance
 *
 * Allocate as a static or long-lived variable and pass its address to
 * ax25_phy_kiss_uart_init().  The structure must remain valid (not moved or
 * freed) until ax25_phy_kiss_uart_deinit() returns.
 *
 * All other fields are managed by the driver; treat as opaque.
 */
typedef struct {
    uart_port_t              uart_num;
    ax25_kiss_decoder_t      kiss_decoder;
    volatile bool            running;
    TaskHandle_t             uart_rx_task_handle;
    TaskHandle_t             uart_tx_task_handle;
    QueueHandle_t            uart_tx_queue;
    SemaphoreHandle_t        tx_mutex;
    ax25_on_frame_t          on_rx_frame;
    void                    *user_data;
    ax25_phy_uart_rx_tap_t   rx_tap;
    void                    *rx_tap_user_data;
} ax25_phy_kiss_uart_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Initialise the UART KISS driver
 *
 * Installs the UART driver, configures pins and baud rate from ax25_config,
 * initialises the KISS decoder, and spawns RX/TX tasks. Each received frame
 * is delivered to @p on_rx_frame, while outgoing AX.25 frames are queued to the
 * TX task for UART transmission.
 *
 * UART hardware setup is delegated to ax25_uart_init(); see ax25_uart.h for
 * the configuration keys read.
 *
 * @param on_rx_frame  Receive callback (must not be NULL)
 * @param user_data Passed as-is to @p on_rx_frame
 * @param ctx     Caller-allocated context (must remain valid until deinit)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if @p ctx or @p on_rx_frame is NULL
 * @return ESP_ERR_NO_MEM if the RX/TX task or TX queue could not be created
 * @return Any ESP_ERR_* returned by the UART driver
 */
esp_err_t ax25_phy_kiss_uart_init(ax25_on_frame_t on_rx_frame, void *user_data, ax25_phy_kiss_uart_t *ctx);

/**
 * @brief Stop the RX/TX tasks and release the UART driver
 *
 * Signals the RX/TX tasks to exit, waits briefly for them to do so, then deletes
 * the UART driver. Safe to call on a zero-initialised or
 * already-deinitialised context.
 *
 * @param ctx  Context previously passed to ax25_phy_kiss_uart_init()
 */
void ax25_phy_kiss_uart_deinit(ax25_phy_kiss_uart_t *ctx);

/**
 * @brief Send @p frame via the UART
 *
 * Builds the AX.25 byte string, wraps it in a KISS frame, and queues it for
 * transmission by the TX task.
 *
 * @param frame  AX.25 frame to transmit
 * @param ctx    Initialised UART KISS context
 * @return ESP_OK if the frame was successfully queued
 * @return ESP_ERR_INVALID_ARG if @p ctx or @p frame is NULL
 * @return ESP_ERR_INVALID_STATE if the driver is not initialized
 * @return ESP_ERR_NO_MEM if the TX queue is full
 * @return ESP_FAIL if frame build/encode/write fails
 */
esp_err_t ax25_phy_kiss_uart_send(const ax25_frame_t *frame, ax25_phy_kiss_uart_t *ctx);

/**
 * @brief Write raw bytes directly to the UART
 *
 * Used in console (terminal) mode to forward user input to the TNC without
 * KISS framing.
 *
 * @param data  Bytes to write
 * @param len   Number of bytes
 * @param ctx   Initialised UART KISS context
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if any argument is NULL / len is zero
 * @return ESP_FAIL on write error
 */
esp_err_t ax25_phy_kiss_uart_write_raw(const uint8_t *data, size_t len, ax25_phy_kiss_uart_t *ctx);

/**
 * @brief Register a raw RX tap
 *
 * While a tap is registered all bytes received from the UART are delivered
 * to @p tap instead of to the KISS decoder.  Call
 * ax25_phy_kiss_uart_clear_rx_tap() to remove the tap and restore normal KISS
 * decoding.
 *
 * @param tap       Callback to receive raw bytes (must not be NULL)
 * @param user_data Passed as-is to @p tap
 * @param ctx       Initialised UART KISS context
 */
void ax25_phy_kiss_uart_set_rx_tap(ax25_phy_uart_rx_tap_t tap, void *user_data, ax25_phy_kiss_uart_t *ctx);

/**
 * @brief Remove the raw RX tap and restore KISS decoding
 *
 * Safe to call even when no tap is currently registered.
 *
 * @param ctx  Initialised UART KISS context
 */
void ax25_phy_kiss_uart_clear_rx_tap(ax25_phy_kiss_uart_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* AX25_PHY_KISS_UART_H */
