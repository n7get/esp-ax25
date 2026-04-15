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
 * @file ax25_conn.h
 * @brief AX.25 v2.0 connected mode session handler
 *
 * Implements a single AX.25 connected mode session with proper state machine,
 * timers, retries, and error handling per the AX.25 v2.0 specification.
 * Thread-safe implementation using mutexes for API protection.
 * 
 * Ignores UI frames - this component is for connected mode only.
 * Handles only one connection at a time.
 */

#ifndef AX25_CONN_H
#define AX25_CONN_H

#include "ax25_types.h"
#include "ax25_frame.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration Defaults (overridable via Kconfig)
 ******************************************************************************/

#ifndef CONFIG_AX25_CONN_T1_MS
#define CONFIG_AX25_CONN_T1_MS 10000
#endif

#ifndef CONFIG_AX25_CONN_T2_MS
#define CONFIG_AX25_CONN_T2_MS 1000
#endif

#ifndef CONFIG_AX25_CONN_T3_MS
#define CONFIG_AX25_CONN_T3_MS 180000
#endif

#ifndef CONFIG_AX25_CONN_N2_RETRIES
#define CONFIG_AX25_CONN_N2_RETRIES 10
#endif

#ifndef CONFIG_AX25_CONN_WINDOW_SIZE
#define CONFIG_AX25_CONN_WINDOW_SIZE 4
#endif

/*******************************************************************************
 * Types and Enumerations
 ******************************************************************************/

/**
 * @brief Connection state for ax25_conn
 */
typedef enum {
    AX25_CONN_STATE_DISCONNECTED,           /**< No connection, idle */
    AX25_CONN_STATE_AWAITING_CONNECTION,    /**< SABM sent, waiting for UA */
    AX25_CONN_STATE_AWAITING_RELEASE,       /**< DISC sent, waiting for UA/DM */
    AX25_CONN_STATE_CONNECTED,              /**< Connection established */
    AX25_CONN_STATE_TIMER_RECOVERY,         /**< T1 expired, in recovery mode */
} ax25_conn_state_t;

/**
 * @brief Error information structure for on_error callback
 */
typedef struct {
    esp_err_t code;         /**< Error code */
    const char* message;    /**< Human-readable error message */
    uint8_t retry_count;    /**< Current retry count when error occurred */
} ax25_conn_error_t;

/**
 * @brief Callback: connection established
 * @param remote_addr Address of the remote station
 * @param is_local_initiated true if we initiated the connection (sent SABM)
 * @param user_data User context pointer
 * @note Can call ax25_conn_refuse() from this callback to terminate
 */
typedef void (*ax25_conn_on_connect_fn_t)(ax25_address_t remote_addr, bool is_local_initiated, void* user_data);

/**
 * @brief Callback: connection disconnected
 * @param user_data User context pointer
 */
typedef void (*ax25_conn_on_disconnect_fn_t)(void* user_data);

/**
 * @brief Callback: in-session link reset detected (optional)
 * @param user_data User context pointer
 * @note Triggered when SABM is received while already connected.
 */
typedef void (*ax25_conn_on_link_reset_fn_t)(void* user_data);

/**
 * @brief Callback: error occurred
 * @param error Error information
 * @param user_data User context pointer
 */
typedef void (*ax25_conn_on_error_fn_t)(const ax25_conn_error_t* error, void* user_data);

/**
 * @brief Callback: final cleanup about to happen
 * @param user_data User context pointer
 * @note Called just before ax25_conn terminates/resets
 */
typedef void (*ax25_conn_on_final_fn_t)(void* user_data);

/**
 * @brief Callback: data received from remote (required)
 * @param data Pointer to received data
 * @param len Length of received data
 * @param user_data User context pointer
 */
typedef void (*ax25_conn_on_data_fn_t)(const uint8_t* data, size_t len, void* user_data);

/**
 * @brief Callback: frame to be transmitted (required)
 * @param frame Parsed frame to transmit
 * @param user_data User context pointer
 */
typedef void (*ax25_conn_on_frame_fn_t)(const ax25_frame_t* frame, void* user_data);

/**
 * @brief Callbacks structure for ax25_conn
 */
typedef struct {
    ax25_conn_on_connect_fn_t on_connect;       /**< Connection established (optional) */
    ax25_conn_on_disconnect_fn_t on_disconnect; /**< Disconnected (optional) */
    ax25_conn_on_link_reset_fn_t on_link_reset; /**< Link reset while connected (optional) */
    ax25_conn_on_error_fn_t on_error;           /**< Error occurred (optional) */
    ax25_conn_on_final_fn_t on_final;           /**< Final cleanup (optional) */
    ax25_conn_on_data_fn_t on_data;             /**< Data received (required) */
    ax25_conn_on_frame_fn_t on_tx_frame;           /**< Frame to transmit (required) */
} ax25_conn_callbacks_t;

/**
 * @brief Configuration structure for ax25_conn
 */
typedef struct {
    uint32_t t1_ms;         /**< T1: Acknowledgement timeout (ms) */
    uint32_t t2_ms;         /**< T2: Response delay timeout (ms) */
    uint32_t t3_ms;         /**< T3: Inactive link timeout (ms) */
    uint8_t n2_retries;     /**< N2: Maximum retry count */
    uint8_t window_size;    /**< k: Maximum outstanding I frames (1-7) */
} ax25_conn_config_t;

/**
 * @brief Optional digipeater path for connected-mode frames.
 */
typedef struct {
    ax25_address_t digipeaters[AX25_MAX_DIGIPEATERS];
    uint8_t num_digipeaters;
} ax25_conn_path_t;

/**
 * @brief Default configuration
 */
#define AX25_CONN_CONFIG_DEFAULT() {                    \
    .t1_ms = CONFIG_AX25_CONN_T1_MS,                   \
    .t2_ms = CONFIG_AX25_CONN_T2_MS,                   \
    .t3_ms = CONFIG_AX25_CONN_T3_MS,                   \
    .n2_retries = CONFIG_AX25_CONN_N2_RETRIES,         \
    .window_size = CONFIG_AX25_CONN_WINDOW_SIZE,       \
}

/**
 * @brief Pending I-frame for transmit/retransmit queue
 */
typedef struct {
    uint8_t data[AX25_MAX_INFO_LEN];    /**< I-frame payload */
    size_t len;                          /**< Payload length */
    uint8_t ns;                          /**< Send sequence number */
    bool in_use;                         /**< Slot in use */
    bool transmitted;                    /**< Frame has been sent at least once */
} ax25_conn_pending_frame_t;

/**
 * @brief ax25_conn context structure
 */
typedef struct {
    /* Configuration */
    ax25_conn_config_t config;

    /* Addresses */
    ax25_address_t local_addr;
    ax25_address_t remote_addr;
    bool remote_addr_set;
    ax25_conn_path_t path;

    /* Callbacks */
    ax25_conn_callbacks_t callbacks;
    void* user_data;

    /* State */
    ax25_conn_state_t state;
    bool is_local_initiated;
    bool refuse_pending;            /**< Set in on_connect to refuse connection */

    /* Sequence numbers (modulo 8) */
    uint8_t vs;     /**< V(S): Send state variable */
    uint8_t vr;     /**< V(R): Receive state variable */
    uint8_t va;     /**< V(A): Acknowledge state variable */

    /* Flow control */
    bool peer_busy;         /**< Remote sent RNR */
    bool local_busy;        /**< We're busy - sends RNR to peer when set */
    bool rej_sent;          /**< REJ has been sent, waiting for retransmit */
    bool ack_pending;       /**< Need to send acknowledgement */

    /* Retry tracking */
    uint8_t retry_count;

    /* Active T1 timeout */
    uint32_t t1_current_ms;      /**< Active T1 timeout currently used */

    /* Deferred callbacks */
    bool disconnect_pending;   /**< on_disconnect should be dispatched unlocked */
    void* deferred_pool[2];    /**< Internal reusable deferred dispatch storage */
    uint8_t deferred_pool_in_use_mask;
    uint32_t dispatcher_generation;
    uint16_t dispatcher_pending_count;
    uint8_t dispatcher_executing_count;
    bool dispatcher_shutdown;
    SemaphoreHandle_t dispatcher_quiesced_sem;

    /* Pending frames for transmission */
    ax25_conn_pending_frame_t tx_queue[8];  /**< Unacknowledged frames */

    /* Timers */
    esp_timer_handle_t timer_t1;
    esp_timer_handle_t timer_t2;
    esp_timer_handle_t timer_t3;

    /* Thread safety */
    SemaphoreHandle_t mutex;

    /* Initialization flag */
    bool initialized;
    uint32_t magic_cookie;  /**< Validation cookie to detect use-after-free */
} ax25_conn_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initialize ax25_conn
 *
 * @param ctx Pointer to ax25_conn context (caller allocates)
 * @param local_addr Local station address
 * @param callbacks Callback functions (on_data and on_tx_frame are required)
 * @param user_data User context pointer passed to all callbacks
 * @param config Configuration (NULL for defaults)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ax25_conn_init(ax25_conn_t* ctx,
                         const ax25_address_t* local_addr,
                         const ax25_conn_callbacks_t* callbacks,
                         void* user_data,
                         const ax25_conn_config_t* config);

/**
 * @brief Deinitialize ax25_conn
 *
 * Stops all timers and releases resources.
 *
 * @param ctx Pointer to ax25_conn context
 */
void ax25_conn_deinit(ax25_conn_t* ctx);

/**
 * @brief Initiate connection to remote station
 *
 * Sends SABM and transitions to AWAITING_CONNECTION state.
 * Thread-safe.
 *
 * @param ctx Pointer to ax25_conn context
 * @param remote_addr Remote station address
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not disconnected
 */
esp_err_t ax25_conn_connect_direct(ax25_conn_t* ctx, const ax25_address_t* remote_addr);

/**
 * @brief Initiate connection to remote station through optional digipeaters.
 *
 * Sends SABM and transitions to AWAITING_CONNECTION state.
 * Thread-safe.
 *
 * @param ctx Pointer to ax25_conn context
 * @param remote_addr Remote station address
 * @param digipeaters Optional digipeater array (NULL for direct connect)
 * @param num_digipeaters Number of digipeaters in @p digipeaters (0-8)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not disconnected
 */
esp_err_t ax25_conn_connect_via(ax25_conn_t* ctx,
                                const ax25_address_t* remote_addr,
                                const ax25_address_t* digipeaters,
                                uint8_t num_digipeaters);

#define AX25_CONN_CONNECT_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define ax25_conn_connect(...) \
    AX25_CONN_CONNECT_SELECT(__VA_ARGS__, \
                             ax25_conn_connect_via, \
                             ax25_conn_connect_via, \
                             ax25_conn_connect_direct)(__VA_ARGS__)

/**
 * @brief Process received frame from remote
 *
 * Call this when a frame is received from the physical layer.
 * Thread-safe.
 *
 * @param ctx Pointer to ax25_conn context
 * @param frame Parsed frame (UI frames are silently ignored)
 * @return ESP_OK on success, error code if frame ignored
 */
esp_err_t ax25_conn_on_frame(ax25_conn_t* ctx, const ax25_frame_t* frame);

/**
 * @brief Send data to remote application
 *
 * Queues data for transmission in I-frames. Thread-safe.
 *
 * @param ctx Pointer to ax25_conn context
 * @param data Pointer to data to send
 * @param len Length of data
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not connected,
 *         ESP_ERR_NO_MEM if send buffer full
 */
esp_err_t ax25_conn_send_data(ax25_conn_t* ctx, const uint8_t* data, size_t len);

/**
 * @brief Graceful shutdown (disconnect)
 *
 * Sends DISC and transitions to AWAITING_RELEASE state.
 * Thread-safe.
 *
 * @param ctx Pointer to ax25_conn context
 * @return ESP_OK on success
 */
esp_err_t ax25_conn_shutdown(ax25_conn_t* ctx);

/**
 * @brief Refuse incoming connection
 *
 * Call from on_connect callback to refuse an incoming connection.
 * Sends DM response.
 *
 * @param ctx Pointer to ax25_conn context
 * @param reason Optional reason data to include (may be NULL)
 * @param len Length of reason data
 * @return ESP_OK on success
 */
esp_err_t ax25_conn_refuse(ax25_conn_t* ctx, const uint8_t* reason, size_t len);

/**
 * @brief Get current connection state
 *
 * Thread-safe.
 *
 * @param ctx Pointer to ax25_conn context
 * @return Current state
 */
ax25_conn_state_t ax25_conn_get_state(ax25_conn_t* ctx);

/**
 * @brief Check if connected
 *
 * Thread-safe.
 *
 * @param ctx Pointer to ax25_conn context
 * @return true if in CONNECTED or TIMER_RECOVERY state
 */
bool ax25_conn_is_connected(ax25_conn_t* ctx);

/**
 * @brief Get remote address
 *
 * @param ctx Pointer to ax25_conn context
 * @param addr Output address structure
 * @return ESP_OK if remote address is set
 */
esp_err_t ax25_conn_get_remote_addr(ax25_conn_t* ctx, ax25_address_t* addr);

/**
 * @brief Get active normalized digipeater path for the current session.
 *
 * Returns the path currently associated with the connection context.
 * For direct links this returns num_digipeaters=0.
 *
 * @param ctx Pointer to ax25_conn context
 * @param path Output path structure
 * @return ESP_OK if a remote/session path is available
 */
esp_err_t ax25_conn_get_path(ax25_conn_t* ctx, ax25_conn_path_t* path);

/**
 * @brief Signal local busy / not-busy state
 *
 * When busy is true, sends RNR to the peer immediately (if connected) so it
 * stops sending I frames.  Any acknowledgement responses while locally busy
 * also use RNR instead of RR, keeping the peer informed.
 *
 * When busy is false, sends RR to the peer to indicate we can accept frames
 * again.
 *
 * Has no effect when called while disconnected.  Thread-safe.
 *
 * @param ctx  Pointer to ax25_conn context
 * @param busy true to enter busy state, false to clear it
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if ctx is NULL/uninitialized
 */
esp_err_t ax25_conn_set_busy(ax25_conn_t* ctx, bool busy);

#ifdef __cplusplus
}
#endif

#endif /* AX25_CONN_H */
