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
 * @file ax25_agwpe_client.h
 * @brief TCP AGWPE client transport
 *
 * Wraps a TCP connection to a remote AGWPE server. The client owns the socket
 * lifecycle, reconnect behavior, inbound byte-stream decoding, and outbound
 * frame queueing. Each successfully decoded AGWPE frame is delivered to the
 * application via the configured frame callback.
 *
 * This module is transport- and protocol-oriented only. It does not integrate
 * with the local AX.25 router or manage local connected-mode state.
 */

#ifndef AX25_AGWPE_CLIENT_H
#define AX25_AGWPE_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "ax25_agwpe.h"
#include "ax25_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parameters for ax25_agwpe_client_init().
 *
 * Zero-valued optional timing and task fields fall back to sensible defaults.
 */
typedef struct {
    /** Hostname or dotted-decimal IP address of the AGWPE server. */
    const char *host;

    /** TCP port number of the AGWPE server. */
    uint16_t port;

    /** Callback invoked for each fully decoded inbound AGWPE frame. */
    agwpe_frame_cb_t on_rx_frame;

    /** Opaque pointer forwarded to every on_rx_frame callback. */
    void *user_data;

    /** Timeout for connect() in milliseconds. 0 -> default. */
    uint32_t connect_timeout_ms;

    /** Delay between reconnect attempts in milliseconds. 0 -> default. */
    uint32_t reconnect_delay_ms;

    /** RX task stack size in bytes. 0 -> default. */
    uint32_t rx_task_stack_size;

    /** RX task priority. 0 -> default. */
    UBaseType_t rx_task_priority;

    /** TX task stack size in bytes. 0 -> default. */
    uint32_t tx_task_stack_size;

    /** TX task priority. 0 -> default. */
    UBaseType_t tx_task_priority;

    /** Maximum number of encoded AGWPE frames buffered for TX. 0 -> default. */
    uint32_t tx_queue_depth;
} ax25_agwpe_client_config_t;

/**
 * @brief Runtime context for a TCP AGWPE client instance.
 *
 * Allocate as a static or long-lived variable and pass its address to
 * ax25_agwpe_client_init(). Treat all fields as implementation-private.
 */
typedef struct {
    const char *host;
    uint16_t port;
    agwpe_frame_cb_t on_rx_frame;
    void *user_data;
    uint32_t connect_timeout_ms;
    uint32_t reconnect_delay_ms;
    agwpe_decoder_t decoder;
    volatile bool running;
    TaskHandle_t rx_task_handle;
    TaskHandle_t tx_task_handle;
    int sock;
    SemaphoreHandle_t send_mutex;
    QueueHandle_t tx_queue;
} ax25_agwpe_client_t;

esp_err_t ax25_agwpe_client_init(const ax25_agwpe_client_config_t *config,
                                 ax25_agwpe_client_t *ctx);

void ax25_agwpe_client_deinit(ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_send_frame(const agwpe_frame_t *frame,
                                       ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_request_version(ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_request_port_info(ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_request_port_caps(uint8_t port,
                                              ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_register_callsign(uint8_t port,
                                              const char *callsign,
                                              ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_unregister_callsign(uint8_t port,
                                                const char *callsign,
                                                ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_enable_monitor(ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_enable_raw(ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_send_raw(const ax25_frame_t *frame,
                                     uint8_t port,
                                     ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_send_unproto(const ax25_frame_t *frame,
                                         uint8_t port,
                                         ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_connect(uint8_t port,
                                    const char *from_call,
                                    const char *to_call,
                                    ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_connect_via(uint8_t port,
                                        const char *from_call,
                                        const char *to_call,
                                        const char *digipeaters[],
                                        uint8_t num_digis,
                                        ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_disconnect(uint8_t port,
                                       const char *from_call,
                                       const char *to_call,
                                       ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_send_connected(uint8_t port,
                                           const char *from_call,
                                           const char *to_call,
                                           const uint8_t *data,
                                           size_t data_len,
                                           uint8_t pid,
                                           ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_send_unproto_data(uint8_t port,
                                              const char *from_call,
                                              const char *to_call,
                                              const uint8_t *data,
                                              size_t data_len,
                                              uint8_t pid,
                                              ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_send_unproto_via_data(uint8_t port,
                                                  const char *from_call,
                                                  const char *to_call,
                                                  const char *digipeaters[],
                                                  uint8_t num_digis,
                                                  const uint8_t *data,
                                                  size_t data_len,
                                                  uint8_t pid,
                                                  ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_query_outstanding(uint8_t port,
                                              const char *from_call,
                                              const char *to_call,
                                              ax25_agwpe_client_t *ctx);

esp_err_t ax25_agwpe_client_request_heard(uint8_t port,
                                          ax25_agwpe_client_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* AX25_AGWPE_CLIENT_H */