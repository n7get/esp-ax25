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
 * @file ax25_phy_kiss_tcp_client.h
 * @brief TCP KISS client physical-layer driver
 *
 * Wraps a TCP connection to a KISS-over-TCP soundmodem (e.g. Direwolf) into a
 * simple physical-layer context.  Each received KISS frame is decoded and the
 * resulting AX.25 frame is delivered to the application via the @c frame_cb
 * callback supplied in the configuration.  Outgoing frames are serialized and
 * queued by ax25_phy_kiss_tcp_client_send() for transmission from a dedicated
 * TX task.
 *
 * The driver spawns RX and TX tasks.  The RX task owns the socket lifecycle:
 * it resolves the host, connects, receives, and reconnects automatically on
 * disconnection for as long as the driver is running.  The TX task drains a
 * bounded queue of encoded KISS frames and writes them to the active socket.
 *
 * Typical usage
 * -------------
 * @code
 *   static ax25_phy_kiss_tcp_client_t phy;
 *
 *   static void my_frame_cb(const ax25_frame_t *frame, void *user_data) {
 *       ax25_print_frame("RX", frame);
 *   }
 *
 *   const ax25_phy_kiss_tcp_client_config_t cfg = {
 *       .host            = "192.168.1.10",
 *       .port            = 8001,
 *       .on_rx_frame        = my_frame_cb,
 *       .user_data = NULL,
 *   };
 *
 *   ax25_phy_kiss_tcp_client_init(&cfg, &phy);
 *
 *   // Send a frame:
 *   ax25_phy_kiss_tcp_client_send(&frame, &phy);
 *
 *   // Shutdown:
 *   ax25_phy_kiss_tcp_client_deinit(&phy);
 * @endcode
 */

#ifndef AX25_PHY_KISS_TCP_CLIENT_H
#define AX25_PHY_KISS_TCP_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ax25_kiss.h"
#include "ax25_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * @brief Parameters for ax25_phy_kiss_tcp_client_init()
 *
 * Zero-valued optional fields fall back to the defaults noted in the field
 * comments.
 */
typedef struct {
    /** Hostname or dotted-decimal IP address of the KISS soundmodem.
     *  The string must remain valid for the lifetime of the driver. */
    const char  *host;

    /** TCP port number of the KISS soundmodem (e.g. 8001). */
    uint16_t     port;

    /** Callback invoked for each successfully decoded RX frame.  Must not be
     *  NULL. */
    ax25_on_frame_t  on_rx_frame;

    /** Opaque pointer forwarded to every @c on_rx_frame invocation. */
    void        *user_data;

    /** Timeout for the TCP connect() call in milliseconds.  0 → 6000. */
    uint32_t     connect_timeout_ms;

    /** Delay between reconnection attempts in milliseconds.  0 → 5000. */
    uint32_t     reconnect_delay_ms;

    /** RX task stack depth in bytes.  0 → 4096. */
    uint32_t     rx_task_stack_size;

    /** RX task FreeRTOS priority.  0 → 5. */
    UBaseType_t  rx_task_priority;

    /** TX task stack depth in bytes.  0 → 4096. */
    uint32_t     tx_task_stack_size;

    /** TX task FreeRTOS priority.  0 → 5. */
    UBaseType_t  tx_task_priority;

    /** Maximum number of encoded KISS frames buffered for TX.  0 → 8. */
    uint32_t     tx_queue_depth;
} ax25_phy_kiss_tcp_client_config_t;

/*******************************************************************************
 * Context
 ******************************************************************************/

/**
 * @brief Runtime context for a TCP KISS client physical-layer instance
 *
 * Allocate as a static or long-lived variable and pass its address to
 * ax25_phy_kiss_tcp_client_init().  The structure must remain valid (not moved
 * or freed) until ax25_phy_kiss_tcp_client_deinit() returns.
 *
 * All fields are managed by the driver; treat as opaque.
 */
typedef struct {
    const char                    *host;
    uint16_t                       port;
    ax25_on_frame_t   on_rx_frame;
    void                          *user_data;
    uint32_t                       connect_timeout_ms;
    uint32_t                       reconnect_delay_ms;
    ax25_kiss_decoder_t            kiss_decoder;
    volatile bool                  running;
    TaskHandle_t                   rx_task_handle;
    TaskHandle_t                   tx_task_handle;
    /** Active socket fd, or -1 when not connected.  Protected by send_mutex. */
    int                            sock;
    SemaphoreHandle_t              send_mutex;
    QueueHandle_t                  tx_queue;
} ax25_phy_kiss_tcp_client_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Initialise the TCP KISS client driver
 *
 * Resolves @p config->host, spawns RX/TX tasks, and initialises the KISS
 * decoder.  Each received KISS frame is decoded and delivered to
 * @p config->on_rx_frame.  Outgoing frames are queued for transmission by the TX
 * task.
 *
 * @param config  Host, port, callback, and task parameters
 * @param ctx     Caller-allocated context (must remain valid until deinit)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if @p ctx, @p config, @p config->host, or
 *         @p config->on_rx_frame is NULL, or @p config->port is 0
 * @return ESP_ERR_NO_MEM if OS objects could not be created
 */
esp_err_t ax25_phy_kiss_tcp_client_init(const ax25_phy_kiss_tcp_client_config_t *config,
                                        ax25_phy_kiss_tcp_client_t *ctx);

/**
 * @brief Stop the RX/TX tasks and release all resources
 *
 * Signals the RX/TX tasks to stop, shuts down the active socket (if any) to
 * unblock any in-progress recv()/send(), and waits for both tasks to exit.
 * Safe to call on a zero-initialised or already-deinitialised context.
 *
 * @param ctx  Context previously passed to ax25_phy_kiss_tcp_client_init()
 */
void ax25_phy_kiss_tcp_client_deinit(ax25_phy_kiss_tcp_client_t *ctx);

/**
 * @brief Send @p frame via the TCP connection
 *
 * Builds the AX.25 byte string, wraps it in a KISS frame, and queues it for
 * transmission by the TX task.  Returns ESP_ERR_INVALID_STATE if the socket is
 * not currently connected.
 *
 * @param frame  AX.25 frame to transmit
 * @param ctx    Initialised TCP KISS client context
 * @return ESP_OK if the frame was successfully queued
 * @return ESP_ERR_INVALID_ARG if @p ctx or @p frame is NULL
 * @return ESP_ERR_INVALID_STATE if not currently connected
 * @return ESP_ERR_NO_MEM if the TX queue is full
 * @return ESP_FAIL if frame build, encode, or socket write fails
 */
esp_err_t ax25_phy_kiss_tcp_client_send(const ax25_frame_t *frame,
                                        ax25_phy_kiss_tcp_client_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* AX25_PHY_KISS_TCP_CLIENT_H */
