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
 * @file ax25_log_tcp_server.h
 * @brief ESP log fanout: console sink + TCP log server
 *
 * Installs a custom vprintf hook via esp_log_set_vprintf() that fans every
 * formatted log line out to a list of sinks.  A console sink is always
 * active.  Each TCP client that connects gets its own sink; when it
 * disconnects the sink is removed.
 *
 * Thread / ISR safety
 * -------------------
 * The vprintf hook is protected by a FreeRTOS spinlock (portMUX_TYPE) so
 * it is safe to call from any context including ISRs and the timer task.
 * The sink list itself is only modified from normal task context under the
 * same lock.
 *
 * Limits
 * ------
 * The maximum number of concurrent TCP clients is configured at runtime
 * by ax25_config parameter net.log.max_conns.
 *
 * Wire format
 * -----------
 * Each log line is sent to TCP clients as a plain UTF-8 string terminated
 * by '\\n' — identical to what is printed on the console.  No framing is
 * added; connect with netcat or any line-oriented tool.
 *
 * Usage
 * -----
 * @code
 * ax25_log_tcp_server_t log_srv;
 * ESP_ERROR_CHECK(ax25_log_tcp_server_init(&log_srv));
 * @endcode
 */

#ifndef AX25_LOG_TCP_SERVER_H
#define AX25_LOG_TCP_SERVER_H

#include <stdbool.h>
#include <stdint.h>

#include "ax25_phy_tcp_server.h"
#include "esp_err.h"
#include "esp_log_write.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Server context — allocate statically, pass to init/deinit.
 *
 * Do not access fields directly.
 */
struct ax25_log_tcp_server {
    uint16_t                port;
    uint32_t                client_task_stack_size;
    UBaseType_t             client_task_priority;
    uint16_t                queue_depth;
    vprintf_like_t          prev_vprintf;   /**< restored on deinit */
    ax25_phy_tcp_server_t   tcp_server;
};
typedef struct ax25_log_tcp_server ax25_log_tcp_server_t;

/**
 * @brief Initialise the log TCP server and install the vprintf hook.
 *
 * - Registers a console sink so existing log output is preserved.
 * - Installs log_vprintf() via esp_log_set_vprintf().
 * - Starts a TCP listener using ax25_config key net.log.port.
 *
 * Must be called from a task context (not from an ISR).
 *
 * Reads these settings from ax25_config (with fallback defaults):
 * - net.log.port (8300)
 * - net.log.conn_task_stack (4096)
 * - net.log.conn_task_priority (5)
 * - net.log.queue_depth (32)
 * - net.log.max_conns (2)
 *
 * @param ctx     Caller-allocated context to initialise
 * @return ESP_OK on success
 */
esp_err_t ax25_log_tcp_server_init(ax25_log_tcp_server_t *ctx);

/**
 * @brief Deinitialise the log TCP server and restore the previous vprintf hook.
 *
 * Closes all active client connections, drains their send queues, removes
 * the console sink, and calls esp_log_set_vprintf() with the function that
 * was active before ax25_log_tcp_server_init() was called.
 *
 * @param ctx  Context previously initialised with ax25_log_tcp_server_init()
 */
void ax25_log_tcp_server_deinit(ax25_log_tcp_server_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* AX25_LOG_TCP_SERVER_H */
