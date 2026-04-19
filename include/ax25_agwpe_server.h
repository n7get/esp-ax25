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
 * @file ax25_agwpe_server.h
 * @brief AGWPE server for esp-ax25 — bridges one AGWPE TCP client to AX.25
 *
 * Each ax25_agwpe_server_t instance manages a single AGWPE client connection.
 * Multiple instances can coexist so that several AGWPE clients can be served
 * simultaneously.
 *
 * ## Architecture
 *
 *  ┌─────────────┐   AGWPE frames   ┌───────────────────┐   AX.25 frames
 *  │ AGWPE client│ ◄──────────────► │ agwpe_server      │ ◄──────────────►
 *  │ (TCP)       │                  │                   │
 *  └─────────────┘                  │  ┌──────────┐     │   ax25_router
 *                                   │  │ ax25_conn│ ... │ ◄──────────────►
 *                                   │  └──────────┘     │
 *                                   └───────────────────┘
 *
 * ### I/O paths
 *
 * **AGWPE client → server** (`ax25_agwpe_server_agwpe_in`)
 *   The network layer calls this with decoded AGWPE frames from the client.
 *   The server processes command frames (R, G, g, X, x, k, m, C, v, c, D,
 *   d, V, M, K, Y, y, P) and routes the resulting AX.25 traffic via
 *   ax25_router_send() and ax25_conn.
 *
 * **AX.25 router → server** (`ax25_agwpe_server_ax25_in`)
 *   The router (or any source) calls this when an AX.25 frame arrives that
 *   this server should see.  The server formats monitoring/raw/connected-mode
 *   AGWPE response frames and enqueues them for sending to the client.
 *
 * **Server → AGWPE client** (`on_agwpe_frame_cb`)
 *   A FreeRTOS task drains the outgoing queue and calls this user-supplied
 *   callback for each AGWPE frame that must be sent to the TCP client.
 *
 * **Server → AX.25 router** (via ax25_router_send / ax25_conn on_tx_frame)
 *   Outbound AX.25 traffic is sent directly through the router or through
 *   the ax25_conn on_tx_frame callback which in turn calls ax25_router_send.
 *
 * ### Thread safety
 *
 * All public functions are thread-safe.  Internal state is protected by a
 * FreeRTOS mutex.  The outgoing AGWPE queue is a FreeRTOS queue (inherently
 * thread-safe).
 */

#ifndef AX25_AGWPE_SERVER_H
#define AX25_AGWPE_SERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "ax25_agwpe.h"
#include "ax25_frame.h"
#include "ax25_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "ax25_conn.h"
#include "ax25_router.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Forward declarations / opaque handle
 ******************************************************************************/

/**
 * @brief Opaque handle to an AGWPE server instance.
 *
 * Caller provides storage (typically static).  Initialised by
 * ax25_agwpe_server_init(); cleaned up by ax25_agwpe_server_deinit().
 */
typedef struct ax25_agwpe_server ax25_agwpe_server_t;
/*******************************************************************************
 * Callback types
 ******************************************************************************/

/**
 * @brief Callback invoked to send an AGWPE frame to the connected client.
 *
 * Called from the server's internal FreeRTOS task context.  The callee
 * should encode the frame with agwpe_frame_encode() and write the bytes
 * to the network socket.  The frame pointer is valid only for the duration
 * of the call.
 *
 * @param frame     The AGWPE frame to send to the client.
 * @param user_data Opaque pointer supplied in the configuration.
 */
typedef void (*ax25_agwpe_server_on_frame_cb_t)(const agwpe_frame_t *frame,
                                                 void *user_data);

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/** Depth of the outgoing AGWPE frame queue.
 *  Configurable via menuconfig: AX25_AGWPE_SERVER_TX_QUEUE_DEPTH */
#ifdef CONFIG_AX25_AGWPE_SERVER_TX_QUEUE_DEPTH
#define AX25_AGWPE_SERVER_TX_QUEUE_DEPTH  CONFIG_AX25_AGWPE_SERVER_TX_QUEUE_DEPTH
#elif !defined(AX25_AGWPE_SERVER_TX_QUEUE_DEPTH)
#define AX25_AGWPE_SERVER_TX_QUEUE_DEPTH  16
#endif

/** Stack size for the outgoing-frame sender task (bytes).
 *  Configurable via menuconfig: AX25_AGWPE_SERVER_TASK_STACK_SIZE */
#ifdef CONFIG_AX25_AGWPE_SERVER_TASK_STACK_SIZE
#define AX25_AGWPE_SERVER_TASK_STACK_SIZE  CONFIG_AX25_AGWPE_SERVER_TASK_STACK_SIZE
#elif !defined(AX25_AGWPE_SERVER_TASK_STACK_SIZE)
#define AX25_AGWPE_SERVER_TASK_STACK_SIZE  4096
#endif

/** Priority for the outgoing-frame sender task.
 *  Configurable via menuconfig: AX25_AGWPE_SERVER_TASK_PRIORITY */
#ifdef CONFIG_AX25_AGWPE_SERVER_TASK_PRIORITY
#define AX25_AGWPE_SERVER_TASK_PRIORITY  CONFIG_AX25_AGWPE_SERVER_TASK_PRIORITY
#elif !defined(AX25_AGWPE_SERVER_TASK_PRIORITY)
#define AX25_AGWPE_SERVER_TASK_PRIORITY  5
#endif

/** Number of pre-allocated monitor output buffers.
 *  Configurable via menuconfig: AX25_AGWPE_SERVER_MONITOR_POOL_SIZE */
#ifdef CONFIG_AX25_AGWPE_SERVER_MONITOR_POOL_SIZE
#define AX25_AGWPE_SERVER_MONITOR_POOL_SIZE  CONFIG_AX25_AGWPE_SERVER_MONITOR_POOL_SIZE
#elif !defined(AX25_AGWPE_SERVER_MONITOR_POOL_SIZE)
#define AX25_AGWPE_SERVER_MONITOR_POOL_SIZE  4
#endif

/** Maximum number of AGWPE clients managed by singleton manager.
 *  Configurable via menuconfig: AX25_AGWPE_SERVER_MAX_CLIENTS */
#ifdef CONFIG_AX25_AGWPE_SERVER_MAX_CLIENTS
#define AX25_AGWPE_SERVER_MAX_CLIENTS  CONFIG_AX25_AGWPE_SERVER_MAX_CLIENTS
#elif !defined(AX25_AGWPE_SERVER_MAX_CLIENTS)
#define AX25_AGWPE_SERVER_MAX_CLIENTS  8
#endif

/**
 * @brief Configuration supplied to ax25_agwpe_server_init().
 */
typedef struct {
    /** Callback for sending AGWPE frames to the client (required). */
    ax25_agwpe_server_on_frame_cb_t on_agwpe_frame;

    /** User-data pointer forwarded to on_agwpe_frame. */
    void *user_data;

    /** Radio port number reported to the AGWPE client (default 0). */
    uint8_t port;

    /** Human-readable port description shown in 'G' replies.
     *  If NULL, a default string is used. */
    const char *port_description;

    /** Depth of the outgoing AGWPE frame queue.
     *  0 selects the compile-time default. Values above the compile-time
     *  maximum are capped. */
    uint16_t tx_queue_depth;

    /** Stack size for the TX task (bytes).
     *  0 selects the compile-time default. Values above the compile-time
     *  maximum are capped because the backing stack storage is static. */
    uint32_t task_stack_size;

    /** Priority for the TX task.  0 selects the default. */
    UBaseType_t task_priority;
} ax25_agwpe_server_config_t;

/**
 * @brief Tracks one AX.25 connected-mode session (internal).
 *
 * generation is incremented whenever a slot is freed. Callback contexts carry
 * a copy of this generation so stale callbacks from a prior slot lifetime can
 * be rejected safely after teardown/reuse.
 */
typedef struct {
    bool        in_use;
    bool        conn_needs_cleanup;
    uint32_t    generation;
    ax25_conn_t conn;
    char        local_call[AGWPE_CALLSIGN_LEN];
    char        remote_call[AGWPE_CALLSIGN_LEN];
    uint8_t     pid;
    uint8_t     port;
    ax25_router_port_t conn_router_port;
    bool               conn_router_port_registered;
} agwpe_conn_slot_t;

/**
 * @brief Context for ax25_conn callbacks (internal).
 *
 * This context is stable storage parallel to the slot array. The generation
 * field must match slot->generation at callback entry; otherwise the callback
 * is treated as stale and ignored.
 */
typedef struct {
    ax25_agwpe_server_t *server;
    agwpe_conn_slot_t   *slot;
    uint32_t             generation;
} conn_cb_ctx_t;

/**
 * @brief One monitor output buffer (internal).
 */
typedef struct {
    agwpe_frame_t frame;
    bool          in_use;
} monitor_buf_t;

/**
 * @brief Internal state of one AGWPE server instance (full definition).
 *
 * Caller-provided storage (typically static).  All internal resources
 * (tasks, queues, semaphores) use static FreeRTOS variants.
 */
struct ax25_agwpe_server {
    ax25_agwpe_server_on_frame_cb_t on_agwpe_frame;
    void           *user_data;
    uint8_t         port;
    char            port_desc[64];

    QueueHandle_t     tx_queue;
    StaticQueue_t     tx_queue_buf;
    uint8_t          *tx_queue_storage;
    bool              tx_queue_storage_psram;

    TaskHandle_t      tx_task;
    StaticTask_t      tx_task_tcb;
    StackType_t       tx_task_stack[AX25_AGWPE_SERVER_TASK_STACK_SIZE / sizeof(StackType_t)];

    SemaphoreHandle_t mutex;
    StaticSemaphore_t mutex_buf;

    bool            raw_enabled;
    bool            monitor_enabled;

    bool            callsign_registered;
    char            registered_call[AGWPE_CALLSIGN_LEN];

    size_t           max_conns;
    agwpe_conn_slot_t *conns;
    conn_cb_ctx_t    *cb_contexts;
    bool              cb_contexts_psram;

    monitor_buf_t    *monitor_pool;
    bool              monitor_pool_psram;
    SemaphoreHandle_t monitor_pool_sem;
    StaticSemaphore_t monitor_pool_sem_buf;

    ax25_router_port_t router_port;
    bool               router_port_registered;

    bool initialized;
};

/*******************************************************************************
 * Lifecycle
 ******************************************************************************/

/**
 * @brief Initialise an AGWPE server instance using caller-provided storage.
 *
 * The caller must supply a pointer to a pre-allocated (typically static)
 * ax25_agwpe_server_t. The implementation uses static FreeRTOS control
 * objects and task stacks, while some internal data buffers may be placed
 * in PSRAM when available (with SRAM fallback).
 *
 * Creates the outgoing-frame queue and sender task, and registers a
 * promiscuous port with ax25_router so that monitoring frames can be
 * forwarded to the client (when enabled).
 *
 * @param config      Configuration (must remain valid only for the duration
 *                    of this call; the server copies what it needs).
 * @param out_server  Pointer to caller-owned server storage to initialise.
 * @return ESP_OK on success;
 *         ESP_ERR_INVALID_ARG if required fields are missing;
 *         ESP_ERR_NO_MEM on resource creation failure.
 */
esp_err_t ax25_agwpe_server_init(const ax25_agwpe_server_config_t *config,
                                  ax25_agwpe_server_t *out_server);

/**
 * @brief Shut down and clean up an AGWPE server instance.
 *
 * Disconnects all active AX.25 connections, unregisters the router port,
 * and stops the sender task.  The server storage is NOT freed — the caller
 * owns the memory.
 *
 * @param server  Server handle.  NULL is silently ignored.
 */
void ax25_agwpe_server_deinit(ax25_agwpe_server_t *server);

/*******************************************************************************
 * Frame ingress
 ******************************************************************************/

/**
 * @brief Feed an AGWPE frame received from the client into the server.
 *
 * Call this whenever the network layer has assembled a complete AGWPE frame
 * from the TCP socket.  The server processes the command and may enqueue
 * response frames and/or generate AX.25 traffic.
 *
 * Thread-safe; may be called from any task.
 *
 * @param server  Server handle.
 * @param frame   The decoded AGWPE frame from the client.
 * @return ESP_OK on success;
 *         ESP_ERR_INVALID_ARG if server or frame is NULL.
 */
esp_err_t ax25_agwpe_server_agwpe_in(ax25_agwpe_server_t *server,
                                      const agwpe_frame_t *frame);

/**
 * @brief Feed an AX.25 frame (from the router or elsewhere) into the server.
 *
 * The server will format appropriate AGWPE monitoring / raw / connected-data
 * frames and enqueue them for the client (if the client has enabled the
 * corresponding mode).
 *
 * Thread-safe; may be called from any task (typically the router worker task).
 *
 * @param server  Server handle.
 * @param frame   The decoded AX.25 frame.
 * @return ESP_OK on success;
 *         ESP_ERR_INVALID_ARG if server or frame is NULL.
 */
esp_err_t ax25_agwpe_server_ax25_in(ax25_agwpe_server_t *server,
                                     const ax25_frame_t *frame);

/**
 * @brief Notify the server that the local station just transmitted an AX.25 frame.
 *
 * If monitor mode is enabled and the frame is a UI frame, a 'T' (own-transmitted)
 * AGWPE monitor frame is enqueued for the client.  This mirrors the direwolf
 * behaviour of emitting 'T' frames for locally-generated UI traffic.
 *
 * Thread-safe; may be called from any task.
 *
 * @param server  Server handle.
 * @param frame   The AX.25 frame that was just transmitted.
 * @return ESP_OK on success;
 *         ESP_ERR_INVALID_ARG if server or frame is NULL.
 */
esp_err_t ax25_agwpe_server_ax25_out(ax25_agwpe_server_t *server,
                                      const ax25_frame_t *frame);

/*******************************************************************************
 * Queries
 ******************************************************************************/

/**
 * @brief Get the router port registered by this server instance.
 *
 * Useful if the caller needs to pass the port pointer as source_port to
 * ax25_router_send() for frames originating from this server.
 *
 * @param server  Server handle.
 * @return Pointer to the internal ax25_router_port_t, or NULL if not
 *         initialised.
 */
struct ax25_router_port_t *ax25_agwpe_server_get_router_port(
    ax25_agwpe_server_t *server);

/*******************************************************************************
 * Singleton manager API
 ******************************************************************************/

/**
 * @brief Initialise the singleton AGWPE manager.
 *
 * Registers a single promiscuous router port used to demux AX.25 frames to
 * all attached AGWPE clients.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if already initialised
 * @return ESP_ERR_NO_MEM if mutex creation fails
 */
esp_err_t ax25_agwpe_server_manager_init(void);

/**
 * @brief Deinitialise the singleton AGWPE manager and all attached clients.
 */
void ax25_agwpe_server_manager_deinit(void);

/**
 * @brief Add a managed AGWPE client.
 *
 * Creates one AGWPE client core and attaches it to the singleton manager.
 * The manager uses one shared promiscuous router port for RX demux.
 *
 * @param config       Per-client configuration. on_agwpe_frame is required.
 * @param out_client   Receives allocated client handle on success.
 * @return ESP_OK on success
 */
esp_err_t ax25_agwpe_server_add_client(const ax25_agwpe_server_config_t *config,
                                        ax25_agwpe_server_t **out_client);

/**
 * @brief Remove a managed AGWPE client.
 *
 * @param client  Client handle returned by ax25_agwpe_server_add_client().
 */
void ax25_agwpe_server_remove_client(ax25_agwpe_server_t *client);

/**
 * @brief Ingest AGWPE frame for a managed AGWPE client.
 */
esp_err_t ax25_agwpe_server_client_agwpe_in(ax25_agwpe_server_t *client,
                                             const agwpe_frame_t *frame);

/**
 * @brief Ingest AX.25 frame for a managed AGWPE client.
 */
esp_err_t ax25_agwpe_server_client_ax25_in(ax25_agwpe_server_t *client,
                                            const ax25_frame_t *frame);

/**
 * @brief Notify a managed AGWPE client that the local station transmitted a frame.
 *
 * Mirrors ax25_agwpe_server_ax25_out() for the singleton-manager client path.
 * If monitor mode is enabled and the frame is a UI frame, a 'T' monitor frame
 * is enqueued for the client.
 */
esp_err_t ax25_agwpe_server_client_ax25_out(ax25_agwpe_server_t *client,
                                             const ax25_frame_t *frame);

/**
 * @brief Get router source port marker for a managed AGWPE client.
 */
struct ax25_router_port_t *ax25_agwpe_server_get_client_router_port(
    ax25_agwpe_server_t *client);

#ifdef __cplusplus
}
#endif

#endif /* AX25_AGWPE_SERVER_H */
