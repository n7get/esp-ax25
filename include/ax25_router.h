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
 * @file ax25_router.h
 * @brief AX.25 frame router - forwards received frames to registered ports
 *
 * The router is a singleton: there is exactly one global router per
 * application.  Call ax25_router_init() once at startup before registering
 * any ports or routing frames.  All public functions are thread-safe.
 *
 * Port modes
 * ----------
 * Each port has exactly one mode:
 *
 *  - **AX25_PORT_STATIC** — receives frames whose destination address equals
 *    the port's registered destination.  No two static or digipeater ports
 *    may share the same destination address.
 *  - **AX25_PORT_DIGIPEATER** — receives frames where the next un-repeated
 *    digipeater address equals the port's destination.  No two static or
 *    digipeater ports may share the same destination address.
 *  - **AX25_PORT_PROMISCUOUS** — receives every frame unconditionally.
 *  - **AX25_PORT_DEFAULT** — receives frames only when no static or
 *    digipeater port was eligible.  At most one default port may be
 *    registered at a time.
 *  - **AX25_PORT_DYNAMIC** — does not receive inbound traffic while unbound.
 *    It binds when used as the source_port for an outbound call to
 *    ax25_router_send(), recording that frame's source address as its
 *    destination. Once bound it subsequently acts like AX25_PORT_STATIC.
 *
 * Forwarding rules
 * ----------------
 * When ax25_router_send() is called the router walks its port list:
 *
 *  1. Promiscuous ports receive the frame unconditionally (but never the
 *     source port that originated the frame).
 *  2. Static / digipeater / bound-dynamic ports receive the frame when their
 *     destination matches.  If any such port is eligible, the frame is NOT
 *     forwarded to default or unbound-dynamic ports.
 *  3. If no static/digipeater/bound-dynamic port was eligible, default ports
 *     receive the frame. Unbound dynamic ports are skipped until they bind
 *     via outbound routing as source_port.
 *
 * A port never receives a frame that it originated (i.e. the port pointer
 * passed as @p source_port to ax25_router_send() is always skipped).
 *
 * Frames are delivered asynchronously: each registered port owns an internal
 * queue and worker task.  ax25_router_send() enqueues matching frames and
 * returns; each port task invokes that port's callback in its own task
 * context.
 *
 * Callback requirements
 * ---------------------
 * The on_tx_frame callback is invoked from its port's dedicated router task.
 * Callbacks must:
 *  - Complete bounded work synchronously and return promptly.
 *  - Not call any blocking I/O directly (socket send, UART write, etc.).
 *  - Not acquire locks that are also held across calls to ax25_router_send()
 *    to avoid deadlock.
 *  - May call async-safe downstream APIs such as PHY transport send functions
 *    that queue internally and return without blocking.
 *
 * The router guarantees that, once ax25_router_remove_port() returns, the
 * callback will not be invoked again for that port.
 */

#ifndef AX25_ROUTER_H
#define AX25_ROUTER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "ax25_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Port mode
 ******************************************************************************/

/**
 * @brief Port operating mode
 *
 * Exactly one mode must be chosen per port.  See the file-level documentation
 * for a full description of each mode's forwarding behaviour.
 */
typedef enum {
    /** Receives frames whose destination equals the port's destination. */
    AX25_PORT_STATIC      = 0,

    /** Receives frames when no static or digipeater port matched. */
    AX25_PORT_DEFAULT     = 1,

    /** Receives every frame regardless of address. */
    AX25_PORT_PROMISCUOUS = 2,

    /** Receives frames where the next un-repeated digipeater address
     *  equals the port's destination. */
    AX25_PORT_DIGIPEATER  = 3,

    /** Unbound: does not receive inbound traffic. After first outbound use as
     *  source_port, binds destination = frame->source, then acts like
     *  AX25_PORT_STATIC. */
    AX25_PORT_DYNAMIC     = 4,
} ax25_router_port_mode_t;

/*******************************************************************************
 * Port structure
 ******************************************************************************/

/**
 * @brief Per-port state and statistics
 *
 * The application allocates this structure (typically as a static/global)
 * and passes its address to ax25_router_register_port().  The router stores
 * the pointer internally; the application must keep the structure alive until
 * ax25_router_remove_port() is called.
 *
 * All counter fields are updated by the router and are safe to read from any
 * task (they are written under the router mutex).
 */
typedef struct {
    /** Destination address this port listens on.
     *  For AX25_PORT_DYNAMIC, this field is written by the router on first
     *  outbound bind and must be zeroed before registration. */
    ax25_address_t destination;

    /** Port operating mode. */
    ax25_router_port_mode_t mode;

    /** Callback invoked to deliver each eligible frame.  Must not be NULL. */
    ax25_on_frame_t on_tx_frame;

    /** Opaque pointer forwarded to every callback invocation. */
    void *user_data;

    /*--- Statistics (maintained by the router) ---*/

    /** Number of frames for which the callback was invoked. */
    uint32_t frames_sent;

    /** Cumulative sum of frame payload+header byte counts for frames that
     *  were delivered via the callback. */
    uint32_t bytes_sent;

    /** Number of frames dropped because this port's internal queue was full. */
    uint32_t frames_dropped;
} ax25_router_port_t;

/*******************************************************************************
 * Lifecycle
 ******************************************************************************/

/**
 * @brief Initialise the global AX.25 router
 *
 * Must be called once before any other ax25_router_*() function.  Calling
 * init on an already-initialised router is a no-op and returns ESP_OK.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if the mutex or port list could not be allocated
 */
esp_err_t ax25_router_init(void);

/**
 * @brief Deinitialise the global AX.25 router
 *
 * Removes all ports and frees internal resources.  The router may be
 * re-initialised with ax25_router_init() afterwards.
 */
void ax25_router_deinit(void);

/*******************************************************************************
 * Port management
 ******************************************************************************/

/**
 * @brief Register a port with the router
 *
 * The application fills in @p port->mode, @p port->destination (where
 * applicable), @p port->on_tx_frame, and @p port->user_data before calling
 * this function.  The counter fields are zeroed by this function.  The
 * router stores the pointer @p port directly; the caller must not free or
 * move the structure until ax25_router_remove_port() is called.
 *
 * Constraints enforced at registration time:
 *  - @p port may only be registered once (same pointer).
 *  - At most one AX25_PORT_DEFAULT port may be registered.
 *  - No two AX25_PORT_STATIC or AX25_PORT_DIGIPEATER ports may share the
 *    same destination address.
 *
 * @param port  Pointer to the caller-allocated port structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG   if @p port is NULL or its callback is NULL
 * @return ESP_ERR_INVALID_STATE if @p port is already registered, or a
 *                               constraint above would be violated, or the
 *                               router has not been initialised
 */
esp_err_t ax25_router_register_port(ax25_router_port_t *port);

/**
 * @brief Remove a previously registered port
 *
 * After this call returns the router will no longer deliver frames to
 * @p port.  The application owns the port structure after removal.
 *
 * @param port  The port that was previously registered
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG   if @p port is NULL
 * @return ESP_ERR_NOT_FOUND     if @p port is not currently registered
 * @return ESP_ERR_INVALID_STATE if the router has not been initialised
 */
esp_err_t ax25_router_remove_port(ax25_router_port_t *port);

/**
 * @brief Log a one-line summary of current router slot usage
 *
 * When CONFIG_AX25_ROUTER_LOG_PORT_STATUS is enabled this logs the current
 * number of active, retired, and free router slots along with the configured
 * maximum. When disabled this function is a no-op.
 *
 * @param reason Optional short reason string included in the log line
 */
void ax25_router_log_port_summary(const char *reason);

/*******************************************************************************
 * Frame delivery
 ******************************************************************************/

/**
 * @brief Route a frame to all eligible registered ports
 *
 * Applies the forwarding rules documented at the top of this file.
 *
 * @p source_port identifies the port that originated @p frame.  The router
 * will never deliver the frame back to that port.  This parameter is required
 * and must not be NULL.
 *
 * This function may be called from any task context but must not be called
 * from an ISR.
 *
 * @param frame        The frame to route
 * @param source_port  Port the frame arrived on (required, must not be NULL)
 * @return ESP_OK on success (even if no port was eligible)
 * @return ESP_ERR_INVALID_ARG   if @p frame is NULL or @p source_port is NULL
 * @return ESP_ERR_INVALID_STATE if the router has not been initialised
 */
esp_err_t ax25_router_send(const ax25_frame_t *frame, ax25_router_port_t *source_port);

#ifdef __cplusplus
}
#endif

#endif /* AX25_ROUTER_H */
