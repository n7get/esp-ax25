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
 * @file ax25_conn.c
 * @brief Implementation of AX.25 v2.0 connected mode session handler
 */

#include "ax25_conn.h"
#include "ax25_conn_dispatcher.h"
#include "ax25_address.h"
#include "ax25_frame.h"
#include "ax25_print.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char* TAG = "AX25_CONN";

#ifdef CONFIG_AX25_CONN_LOG_FRAMES
#define AX25_CONN_LOG_FRAMES CONFIG_AX25_CONN_LOG_FRAMES
#else
#define AX25_CONN_LOG_FRAMES 0
#endif

#ifdef CONFIG_AX25_CONN_LOG_DROPPED_FRAMES
#define AX25_CONN_LOG_DROPPED_FRAMES CONFIG_AX25_CONN_LOG_DROPPED_FRAMES
#else
#define AX25_CONN_LOG_DROPPED_FRAMES 0
#endif

/*******************************************************************************
 * Helper Macros
 ******************************************************************************/

#define MOD8(x) ((x) & 0x07)
#define LOCK(ctx) xSemaphoreTake((ctx)->mutex, portMAX_DELAY)
#define UNLOCK(ctx) xSemaphoreGive((ctx)->mutex)
#define AX25_CONN_MAX_DEFERRED_ACTIONS 12
#define AX25_CONN_DEFERRED_POOL_SIZE 2

typedef ax25_conn_dispatch_action_t ax25_conn_deferred_action_t;

typedef struct {
    size_t count;
    ax25_conn_deferred_action_t actions[AX25_CONN_MAX_DEFERRED_ACTIONS];
} ax25_conn_deferred_t;

/*******************************************************************************
 * Forward Declarations
 ******************************************************************************/

static void timer_t1_callback(void* arg);
static void timer_t2_callback(void* arg);
static void timer_t3_callback(void* arg);
static void start_timer_t1(ax25_conn_t* ctx);
static void stop_timer_t1(ax25_conn_t* ctx);
static void start_timer_t2(ax25_conn_t* ctx);
static void stop_timer_t2(ax25_conn_t* ctx);
static void start_timer_t3(ax25_conn_t* ctx);
static void stop_timer_t3(ax25_conn_t* ctx);
static void stop_all_timers(ax25_conn_t* ctx);
static void reset_state_vars(ax25_conn_t* ctx);
static void enter_disconnected_state(ax25_conn_t* ctx, ax25_conn_deferred_t* deferred);
static esp_err_t send_frame_internal(ax25_conn_t* ctx,
                                     const ax25_frame_t* frame,
                                     ax25_conn_deferred_t* deferred);
static esp_err_t set_conn_path(ax25_conn_t* ctx,
                               const ax25_address_t* digipeaters,
                               uint8_t num_digipeaters);
static void set_conn_path_from_frame(ax25_conn_t* ctx, const ax25_frame_t* frame);
static void apply_conn_path(ax25_conn_t* ctx, ax25_frame_t* frame);
static esp_err_t send_sabm(ax25_conn_t* ctx, bool poll, ax25_conn_deferred_t* deferred);
static esp_err_t send_disc(ax25_conn_t* ctx, bool poll, ax25_conn_deferred_t* deferred);
static esp_err_t send_ua(ax25_conn_t* ctx, bool final, ax25_conn_deferred_t* deferred);
static esp_err_t send_dm(ax25_conn_t* ctx, bool final, ax25_conn_deferred_t* deferred);
static esp_err_t send_rr(ax25_conn_t* ctx,
                         bool pf,
                         bool is_command,
                         ax25_conn_deferred_t* deferred);
static esp_err_t send_rnr(ax25_conn_t* ctx,
                          bool pf,
                          bool is_command,
                          ax25_conn_deferred_t* deferred);
static esp_err_t send_ack(ax25_conn_t* ctx,
                          bool pf,
                          bool is_command,
                          ax25_conn_deferred_t* deferred);
static esp_err_t send_rej(ax25_conn_t* ctx,
                          bool pf,
                          bool is_command,
                          ax25_conn_deferred_t* deferred);
static esp_err_t send_pending_i_frames(ax25_conn_t* ctx, ax25_conn_deferred_t* deferred);
static esp_err_t queue_i_frame(ax25_conn_t* ctx, const uint8_t* data, size_t len);
static void handle_received_nr(ax25_conn_t* ctx, uint8_t nr, const ax25_frame_t* frame);
static bool is_valid_nr(ax25_conn_t* ctx, uint8_t nr);
static esp_err_t start_nr_error_recovery(ax25_conn_t* ctx,
                                         const char* msg,
                                         ax25_conn_deferred_t* deferred);
static bool defer_action(ax25_conn_deferred_t* deferred, ax25_conn_deferred_action_t action);
static ax25_conn_deferred_t* alloc_deferred_storage(void);
static void free_deferred_storage(ax25_conn_deferred_t* deferred);
static ax25_conn_deferred_t* acquire_deferred(ax25_conn_t* ctx);
static void release_deferred(ax25_conn_t* ctx, ax25_conn_deferred_t* deferred);
static void defer_on_error(ax25_conn_t* ctx,
                           esp_err_t code,
                           const char* msg,
                           ax25_conn_deferred_t* deferred);
static void defer_on_disconnect(ax25_conn_deferred_t* deferred);
static void defer_on_link_reset(ax25_conn_deferred_t* deferred);
static void drain_deferred(ax25_conn_t* ctx, ax25_conn_deferred_t* deferred);
static void invoke_deferred_action(ax25_conn_t* ctx,
                                   const ax25_conn_deferred_action_t* action);
static void invoke_on_connect_unlocked(ax25_conn_t* ctx, bool is_local);
static void defer_on_final(ax25_conn_deferred_t* deferred);
static void defer_on_data(const uint8_t* data, size_t len, ax25_conn_deferred_t* deferred);
static void clear_tx_queue(ax25_conn_t* ctx);
static int outstanding_frames(ax25_conn_t* ctx);
static bool running_on_esp_timer_task(void);
static void log_conn_frame(const char* label, const ax25_frame_t* frame);
static void log_conn_drop(ax25_conn_t* ctx,
                          const ax25_frame_t* frame,
                          const char* reason);

static void *ax25_conn_alloc_prefer_psram(size_t size)
{
#if CONFIG_SPIRAM
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr != NULL) {
        memset(ptr, 0, size);
        return ptr;
    }
#endif

    return calloc(1, size);
}

/* State handlers */
static esp_err_t handle_frame_disconnected(ax25_conn_t* ctx,
                                           const ax25_frame_t* frame,
                                           ax25_conn_deferred_t* deferred);
static esp_err_t handle_frame_awaiting_connection(ax25_conn_t* ctx,
                                                  const ax25_frame_t* frame,
                                                  ax25_conn_deferred_t* deferred);
static esp_err_t handle_frame_awaiting_release(ax25_conn_t* ctx,
                                               const ax25_frame_t* frame,
                                               ax25_conn_deferred_t* deferred);
static esp_err_t handle_frame_connected(ax25_conn_t* ctx,
                                        const ax25_frame_t* frame,
                                        ax25_conn_deferred_t* deferred);
static esp_err_t handle_frame_timer_recovery(ax25_conn_t* ctx,
                                             const ax25_frame_t* frame,
                                             ax25_conn_deferred_t* deferred);

/*******************************************************************************
 * Timer Callbacks (run from timer task context)
 ******************************************************************************/

/**
 * @brief Handle T1 expiry for retransmit and recovery state transitions.
 * @param arg Connection context supplied when the timer was created.
 */
static void timer_t1_callback(void* arg) {
    ax25_conn_t* ctx = (ax25_conn_t*)arg;
    ax25_conn_deferred_t* deferred = NULL;
    
    if (!ctx || ctx->magic_cookie != 0xDEADBEEF || !ctx->initialized) return;
    
    LOCK(ctx);
    if (!ctx->initialized || ctx->magic_cookie != 0xDEADBEEF) {
        UNLOCK(ctx);
        return;
    }

    deferred = acquire_deferred(ctx);
    if (deferred == NULL) {
        UNLOCK(ctx);
        return;
    }
    
    ESP_LOGD(TAG, "T1 timeout, state=%d, retry=%d/%d", 
             ctx->state, ctx->retry_count, ctx->config.n2_retries);

    switch (ctx->state) {
        case AX25_CONN_STATE_AWAITING_CONNECTION:
            if (ctx->retry_count >= ctx->config.n2_retries) {
                /* N2 exceeded, give up */
                defer_on_error(ctx, ESP_ERR_TIMEOUT, "Connection timeout (N2 exceeded)", deferred);
                enter_disconnected_state(ctx, deferred);
            } else {
                ctx->retry_count++;
                send_sabm(ctx, true, deferred);
                start_timer_t1(ctx);
            }
            break;
            
        case AX25_CONN_STATE_AWAITING_RELEASE:
            if (ctx->retry_count >= ctx->config.n2_retries) {
                /* N2 exceeded, force disconnect */
                defer_on_error(ctx, ESP_ERR_TIMEOUT, "Disconnect timeout (N2 exceeded)", deferred);
                enter_disconnected_state(ctx, deferred);
            } else {
                ctx->retry_count++;
                send_disc(ctx, true, deferred);
                start_timer_t1(ctx);
            }
            break;
            
        case AX25_CONN_STATE_CONNECTED:
            /* Enter timer recovery mode */
            ctx->state = AX25_CONN_STATE_TIMER_RECOVERY;
            ctx->retry_count = 0;
            /* Fall through */
            
        case AX25_CONN_STATE_TIMER_RECOVERY:
            if (ctx->retry_count >= ctx->config.n2_retries) {
                /* N2 exceeded, connection lost */
                defer_on_error(ctx, ESP_ERR_TIMEOUT, "Link failure (N2 exceeded)", deferred);
                send_dm(ctx, true, deferred);
                enter_disconnected_state(ctx, deferred);
            } else {
                ctx->retry_count++;
                /* Send RR with P bit to poll peer */
                send_rr(ctx, true, true, deferred);
                start_timer_t1(ctx);
            }
            break;
            
        default:
            break;
    }
    
    UNLOCK(ctx);
    drain_deferred(ctx, deferred);
        release_deferred(ctx, deferred);
}

/**
 * @brief Log a transmitted or received frame when frame logging is enabled.
 * @param label Log label to use for the frame dump.
 * @param frame Frame to print.
 */
static void log_conn_frame(const char* label, const ax25_frame_t* frame) {
#if AX25_CONN_LOG_FRAMES
    if (frame != NULL) {
        ax25_print_frame(label != NULL ? label : "ax25_conn", frame);
    }
#else
    (void)label;
    (void)frame;
#endif
}

/**
 * @brief Log why a frame or send attempt was dropped when enabled.
 * @param ctx Connection state used for diagnostic context.
 * @param frame Optional frame associated with the drop.
 * @param reason Human-readable drop reason.
 */
static void log_conn_drop(ax25_conn_t* ctx,
                          const ax25_frame_t* frame,
                          const char* reason) {
#if AX25_CONN_LOG_DROPPED_FRAMES
    ESP_LOGW(TAG,
             "Dropping frame: %s (state=%d remote_set=%d)",
             reason != NULL ? reason : "unspecified",
             ctx != NULL ? (int)ctx->state : -1,
             ctx != NULL ? (int)ctx->remote_addr_set : 0);
    if (frame != NULL) {
        ax25_print_frame("ax25_conn_drop", frame);
    }
#else
    (void)ctx;
    (void)frame;
    (void)reason;
#endif
}

/**
 * @brief Seed the active T1 timeout for the current routed path.
 * @param ctx Connection whose active T1 value is being set.
 * @param num_digipeaters Current digipeater count for the path.
 */
static void seed_t1_for_path(ax25_conn_t* ctx, uint8_t num_digipeaters) {
    if (ctx == NULL) {
        return;
    }

    (void)num_digipeaters;
    ctx->t1_current_ms = ctx->config.t1_ms;
}

/**
 * @brief Handle T2 expiry and send a delayed acknowledgement if needed.
 * @param arg Connection context supplied when the timer was created.
 */
static void timer_t2_callback(void* arg) {
    ax25_conn_t* ctx = (ax25_conn_t*)arg;
    ax25_conn_deferred_t* deferred = NULL;
    
    if (!ctx || ctx->magic_cookie != 0xDEADBEEF || !ctx->initialized) return;
    
    LOCK(ctx);
    if (!ctx->initialized || ctx->magic_cookie != 0xDEADBEEF) {
        UNLOCK(ctx);
        return;
    }

    deferred = acquire_deferred(ctx);
    if (deferred == NULL) {
        UNLOCK(ctx);
        return;
    }
    
    ESP_LOGD(TAG, "T2 timeout, ack_pending=%d", ctx->ack_pending);
    
    if (ctx->state == AX25_CONN_STATE_CONNECTED || 
        ctx->state == AX25_CONN_STATE_TIMER_RECOVERY) {
        if (ctx->ack_pending) {
            /* Send acknowledgement */
            send_ack(ctx, false, false, deferred);
            ctx->ack_pending = false;
        }
    }
    
    UNLOCK(ctx);
    drain_deferred(ctx, deferred);
        release_deferred(ctx, deferred);
}

/**
 * @brief Handle T3 expiry by polling an inactive peer.
 * @param arg Connection context supplied when the timer was created.
 */
static void timer_t3_callback(void* arg) {
    ax25_conn_t* ctx = (ax25_conn_t*)arg;
    ax25_conn_deferred_t* deferred = NULL;
    
    if (!ctx || ctx->magic_cookie != 0xDEADBEEF || !ctx->initialized) return;
    
    LOCK(ctx);
    if (!ctx->initialized || ctx->magic_cookie != 0xDEADBEEF) {
        UNLOCK(ctx);
        return;
    }

    deferred = acquire_deferred(ctx);
    if (deferred == NULL) {
        UNLOCK(ctx);
        return;
    }
    
    ESP_LOGD(TAG, "T3 timeout (inactive link)");
    
    if (ctx->state == AX25_CONN_STATE_CONNECTED) {
        /* Poll peer to check if still alive */
        send_rr(ctx, true, true, deferred);
        ctx->state = AX25_CONN_STATE_TIMER_RECOVERY;
        ctx->retry_count = 0;
        start_timer_t1(ctx);
    }
    
    UNLOCK(ctx);
        drain_deferred(ctx, deferred);
        release_deferred(ctx, deferred);
}

/*******************************************************************************
 * Timer Management
 ******************************************************************************/

/**
 * @brief Arm or re-arm the T1 acknowledgement timer.
 * @param ctx Connection whose T1 timer should be started.
 */
static void start_timer_t1(ax25_conn_t* ctx) {
    if (ctx->timer_t1) {
        uint32_t timeout_ms = ctx->t1_current_ms > 0 ? ctx->t1_current_ms : ctx->config.t1_ms;
        esp_timer_stop(ctx->timer_t1);
        esp_timer_start_once(ctx->timer_t1, (uint64_t)timeout_ms * 1000ULL);
    }
}

/**
 * @brief Stop the T1 acknowledgement timer.
 * @param ctx Connection whose T1 timer should be stopped.
 */
static void stop_timer_t1(ax25_conn_t* ctx) {
    if (ctx->timer_t1) {
        esp_timer_stop(ctx->timer_t1);
    }
}

/**
 * @brief Arm or re-arm the T2 delayed-ack timer.
 * @param ctx Connection whose T2 timer should be started.
 */
static void start_timer_t2(ax25_conn_t* ctx) {
    if (ctx->timer_t2) {
        esp_timer_stop(ctx->timer_t2);
        esp_timer_start_once(ctx->timer_t2, ctx->config.t2_ms * 1000ULL);
    }
}

/**
 * @brief Stop the T2 delayed-ack timer.
 * @param ctx Connection whose T2 timer should be stopped.
 */
static void stop_timer_t2(ax25_conn_t* ctx) {
    if (ctx->timer_t2) {
        esp_timer_stop(ctx->timer_t2);
    }
}

/**
 * @brief Arm or re-arm the T3 idle-link timer.
 * @param ctx Connection whose T3 timer should be started.
 */
static void start_timer_t3(ax25_conn_t* ctx) {
    if (ctx->timer_t3) {
        esp_timer_stop(ctx->timer_t3);
        esp_timer_start_once(ctx->timer_t3, ctx->config.t3_ms * 1000ULL);
    }
}

/**
 * @brief Stop the T3 idle-link timer.
 * @param ctx Connection whose T3 timer should be stopped.
 */
static void stop_timer_t3(ax25_conn_t* ctx) {
    if (ctx->timer_t3) {
        esp_timer_stop(ctx->timer_t3);
    }
}

/**
 * @brief Stop all protocol timers owned by the connection.
 * @param ctx Connection whose timers should be stopped.
 */
static void stop_all_timers(ax25_conn_t* ctx) {
    stop_timer_t1(ctx);
    stop_timer_t2(ctx);
    stop_timer_t3(ctx);
}

/*******************************************************************************
 * State Management
 ******************************************************************************/

/**
 * @brief Reset sequence numbers, retry state, and flow-control flags.
 * @param ctx Connection whose session state variables should be reset.
 */
static void reset_state_vars(ax25_conn_t* ctx) {
    ctx->vs = 0;
    ctx->vr = 0;
    ctx->va = 0;
    ctx->peer_busy = false;
    ctx->local_busy = false;
    ctx->rej_sent = false;
    ctx->ack_pending = false;
    ctx->retry_count = 0;
    clear_tx_queue(ctx);
}

/**
 * @brief Store a normalized digipeater path for future connected-mode frames.
 * @param ctx Connection whose path should be updated.
 * @param digipeaters Digipeater array to copy from.
 * @param num_digipeaters Number of digipeaters in the array.
 * @return ESP_OK on success or ESP_ERR_INVALID_SIZE if the path is too long.
 */
static esp_err_t set_conn_path(ax25_conn_t* ctx,
                               const ax25_address_t* digipeaters,
                               uint8_t num_digipeaters) {
    if (num_digipeaters > AX25_MAX_DIGIPEATERS) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(&ctx->path, 0, sizeof(ctx->path));
    ctx->path.num_digipeaters = num_digipeaters;

    for (uint8_t i = 0; i < num_digipeaters; i++) {
        ctx->path.digipeaters[i] = digipeaters[i];
        ctx->path.digipeaters[i].has_been_repeated = false;
        ctx->path.digipeaters[i].is_last = false;
    }

    return ESP_OK;
}

/**
 * @brief Copy the digipeater path from an incoming frame into the connection.
 * @param ctx Connection whose path should be updated.
 * @param frame Incoming frame that supplies the path.
 */
static void set_conn_path_from_frame(ax25_conn_t* ctx, const ax25_frame_t* frame) {
    (void)set_conn_path(ctx, frame->digipeaters, frame->num_digipeaters);
}

/**
 * @brief Apply the stored digipeater path to an outgoing frame.
 * @param ctx Connection that owns the path.
 * @param frame Outgoing frame to update.
 */
static void apply_conn_path(ax25_conn_t* ctx, ax25_frame_t* frame) {
    frame->num_digipeaters = ctx->path.num_digipeaters;
    for (uint8_t i = 0; i < ctx->path.num_digipeaters; i++) {
        frame->digipeaters[i] = ctx->path.digipeaters[i];
    }
}

/**
 * @brief Move the connection back to the disconnected baseline state.
 * @param ctx Connection to reset.
 * @param deferred Deferred-callback buffer that records final notifications.
 */
static void enter_disconnected_state(ax25_conn_t* ctx, ax25_conn_deferred_t* deferred) {
    bool was_connected = (ctx->state == AX25_CONN_STATE_CONNECTED ||
                          ctx->state == AX25_CONN_STATE_TIMER_RECOVERY ||
                          ctx->state == AX25_CONN_STATE_AWAITING_RELEASE ||
                          ctx->state == AX25_CONN_STATE_AWAITING_CONNECTION);
    
    stop_all_timers(ctx);
    reset_state_vars(ctx);
    ctx->state = AX25_CONN_STATE_DISCONNECTED;
    ctx->remote_addr_set = false;
    memset(&ctx->path, 0, sizeof(ctx->path));
    ctx->t1_current_ms = ctx->config.t1_ms;
    
    defer_on_final(deferred);
    
    if (was_connected) {
        defer_on_disconnect(deferred);
    }
}

/**
 * @brief Clear all pending outbound I-frame queue slots.
 * @param ctx Connection whose transmit queue should be cleared.
 */
static void clear_tx_queue(ax25_conn_t* ctx) {
    for (int i = 0; i < 8; i++) {
        ctx->tx_queue[i].in_use = false;
        ctx->tx_queue[i].transmitted = false;
        ctx->tx_queue[i].len = 0;
    }
}

/**
 * @brief Return the number of unacknowledged outstanding I frames.
 * @param ctx Connection whose send window occupancy is queried.
 * @return Count of outstanding frames in modulo-8 sequence space.
 */
static int outstanding_frames(ax25_conn_t* ctx) {
    return MOD8(ctx->vs - ctx->va);
}

/*******************************************************************************
 * Deferred Callback Recording and Dispatch
 ******************************************************************************/

/**
 * @brief Append one deferred callback action to the local batch buffer.
 * @param deferred Deferred action buffer to append to.
 * @param action Action to record.
 * @return true if recorded, false if the buffer is full or unavailable.
 */
static bool defer_action(ax25_conn_deferred_t* deferred, ax25_conn_deferred_action_t action) {
    if (deferred == NULL || deferred->count >= AX25_CONN_MAX_DEFERRED_ACTIONS) {
        return false;
    }

    deferred->actions[deferred->count++] = action;
    return true;
}

/**
 * @brief Allocate overflow storage for deferred callback batching.
 * @return Newly allocated deferred buffer, or NULL on allocation failure.
 */
static ax25_conn_deferred_t* alloc_deferred_storage(void) {
    ax25_conn_deferred_t* deferred = (ax25_conn_deferred_t*)ax25_conn_alloc_prefer_psram(sizeof(*deferred));
    if (deferred == NULL) {
        ESP_LOGE(TAG, "Failed to allocate deferred action queue");
    }
    return deferred;
}

/**
 * @brief Free dynamically allocated deferred callback storage.
 * @param deferred Deferred buffer to free.
 */
static void free_deferred_storage(ax25_conn_deferred_t* deferred) {
    free(deferred);
}

/**
 * @brief Obtain a reusable deferred callback buffer for one operation.
 * @param ctx Connection whose local deferred pool should be used.
 * @return Deferred buffer, using heap overflow allocation if the pool is busy.
 */
static ax25_conn_deferred_t* acquire_deferred(ax25_conn_t* ctx) {
    if (ctx == NULL) {
        return NULL;
    }

    for (uint8_t i = 0; i < AX25_CONN_DEFERRED_POOL_SIZE; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        if ((ctx->deferred_pool_in_use_mask & bit) == 0 && ctx->deferred_pool[i] != NULL) {
            ctx->deferred_pool_in_use_mask |= bit;
            memset(ctx->deferred_pool[i], 0, sizeof(ax25_conn_deferred_t));
            return (ax25_conn_deferred_t*)ctx->deferred_pool[i];
        }
    }

    ESP_LOGW(TAG, "Deferred pool exhausted, using overflow allocation");
    return alloc_deferred_storage();
}

/**
 * @brief Return a deferred buffer to the pool or free overflow storage.
 * @param ctx Connection that owns the pooled deferred buffers.
 * @param deferred Deferred buffer to release.
 */
static void release_deferred(ax25_conn_t* ctx, ax25_conn_deferred_t* deferred) {
    if (ctx == NULL || deferred == NULL) {
        return;
    }

    if (!ctx->initialized || ctx->magic_cookie != 0xDEADBEEF) {
        return;
    }

    for (uint8_t i = 0; i < AX25_CONN_DEFERRED_POOL_SIZE; i++) {
        if (ctx->deferred_pool[i] == deferred) {
            ctx->deferred_pool_in_use_mask &= (uint8_t)~(1u << i);
            return;
        }
    }

    free_deferred_storage(deferred);
}

/**
 * @brief Record an error callback action for later dispatch.
 * @param ctx Connection that generated the error.
 * @param code Error code to report.
 * @param msg Human-readable error message.
 * @param deferred Deferred buffer receiving the action.
 */
static void defer_on_error(ax25_conn_t* ctx,
                           esp_err_t code,
                           const char* msg,
                           ax25_conn_deferred_t* deferred) {
    ax25_conn_deferred_action_t action = {
        .type = AX25_CONN_DISPATCH_ACTION_ERROR,
        .payload.error = {
            .code = code,
            .message = msg,
            .retry_count = ctx->retry_count,
        },
    };

    if (!defer_action(deferred, action)) {
        ESP_LOGE(TAG, "Deferred action queue full while recording error callback");
    }

    ESP_LOGW(TAG, "Error: %s (code=%d, retry=%d)", msg, code, ctx->retry_count);
}

/**
 * @brief Record a disconnect callback action.
 * @param deferred Deferred buffer receiving the action.
 */
static void defer_on_disconnect(ax25_conn_deferred_t* deferred) {
    ax25_conn_deferred_action_t action = {
        .type = AX25_CONN_DISPATCH_ACTION_DISCONNECT,
    };

    if (!defer_action(deferred, action)) {
        ESP_LOGE(TAG, "Deferred action queue full while recording disconnect callback");
    }
}

/**
 * @brief Record a link-reset callback action.
 * @param deferred Deferred buffer receiving the action.
 */
static void defer_on_link_reset(ax25_conn_deferred_t* deferred) {
    ax25_conn_deferred_action_t action = {
        .type = AX25_CONN_DISPATCH_ACTION_LINK_RESET,
    };

    if (!defer_action(deferred, action)) {
        ESP_LOGE(TAG, "Deferred action queue full while recording link-reset callback");
    }
}

/**
 * @brief Invoke `on_connect` without holding the connection mutex.
 * @param ctx Connection whose connect callback should be run.
 * @param is_local Whether the local side initiated the session.
 */
static void invoke_on_connect_unlocked(ax25_conn_t* ctx, bool is_local) {
    if (ctx == NULL) {
        return;
    }

    ax25_address_t remote_addr = ctx->remote_addr;
    ax25_conn_on_connect_fn_t callback = ctx->callbacks.on_connect;
    void* user_data = ctx->user_data;

    ctx->is_local_initiated = is_local;
    ctx->refuse_pending = false;

    UNLOCK(ctx);
    if (callback != NULL) {
        callback(remote_addr, is_local, user_data);
    }
    LOCK(ctx);
}

/**
 * @brief Record a final-cleanup callback action.
 * @param deferred Deferred buffer receiving the action.
 */
static void defer_on_final(ax25_conn_deferred_t* deferred) {
    ax25_conn_deferred_action_t action = {
        .type = AX25_CONN_DISPATCH_ACTION_FINAL,
    };

    if (!defer_action(deferred, action)) {
        ESP_LOGE(TAG, "Deferred action queue full while recording final callback");
    }
}

/**
 * @brief Record an incoming-data callback action.
 * @param data Payload bytes to copy into the deferred action.
 * @param len Payload length in bytes.
 * @param deferred Deferred buffer receiving the action.
 */
static void defer_on_data(const uint8_t* data, size_t len, ax25_conn_deferred_t* deferred) {
    ax25_conn_deferred_action_t action = {
        .type = AX25_CONN_DISPATCH_ACTION_DATA,
    };

    if (len > sizeof(action.payload.data.data)) {
        ESP_LOGE(TAG, "Deferred data payload too large: %zu", len);
        return;
    }

    action.payload.data.len = len;
    if (len > 0) {
        memcpy(action.payload.data.data, data, len);
    }

    if (!defer_action(deferred, action)) {
        ESP_LOGE(TAG, "Deferred action queue full while recording data callback");
    }
}

/**
 * @brief Invoke one deferred callback action immediately on the current task.
 * @param ctx Connection that owns the callback table.
 * @param action Deferred action to execute.
 */
static void invoke_deferred_action(ax25_conn_t* ctx,
                                   const ax25_conn_deferred_action_t* action) {
    if (ctx == NULL || action == NULL) {
        return;
    }

    switch (action->type) {
        case AX25_CONN_DISPATCH_ACTION_ERROR:
            if (ctx->callbacks.on_error) {
                ctx->callbacks.on_error(&action->payload.error, ctx->user_data);
            }
            break;

        case AX25_CONN_DISPATCH_ACTION_DISCONNECT:
            if (ctx->callbacks.on_disconnect) {
                ctx->callbacks.on_disconnect(ctx->user_data);
            }
            break;

        case AX25_CONN_DISPATCH_ACTION_LINK_RESET:
            if (ctx->callbacks.on_link_reset) {
                ctx->callbacks.on_link_reset(ctx->user_data);
            }
            break;

        case AX25_CONN_DISPATCH_ACTION_CONNECT:
            if (ctx->callbacks.on_connect) {
                ctx->callbacks.on_connect(action->payload.connect.remote_addr,
                                          action->payload.connect.is_local,
                                          ctx->user_data);
            }
            break;

        case AX25_CONN_DISPATCH_ACTION_FINAL:
            if (ctx->callbacks.on_final) {
                ctx->callbacks.on_final(ctx->user_data);
            }
            break;

        case AX25_CONN_DISPATCH_ACTION_DATA:
            if (ctx->callbacks.on_data) {
                ctx->callbacks.on_data(action->payload.data.data,
                                       action->payload.data.len,
                                       ctx->user_data);
            }
            break;

        case AX25_CONN_DISPATCH_ACTION_FRAME:
            if (ctx->callbacks.on_tx_frame) {
                ctx->callbacks.on_tx_frame(&action->payload.frame, ctx->user_data);
            }
            break;
    }
}

/**
 * @brief Check whether the current code is running on the ESP timer task.
 * @return true when the current FreeRTOS task name is `esp_timer`.
 */
static bool running_on_esp_timer_task(void) {
    const char* task_name = pcTaskGetName(NULL);

    return task_name != NULL && strcmp(task_name, "esp_timer") == 0;
}

/**
 * @brief Flush recorded deferred actions after leaving the critical section.
 * @param ctx Connection that owns the deferred actions.
 * @param deferred Batch of recorded actions to dispatch.
 */
static void drain_deferred(ax25_conn_t* ctx, ax25_conn_deferred_t* deferred) {
    if (ctx == NULL || deferred == NULL) {
        return;
    }

    bool use_shared_dispatcher = running_on_esp_timer_task();

    if (!use_shared_dispatcher) {
        size_t action_count = deferred->count;

        if (action_count == 0) {
            return;
        }

        ax25_conn_deferred_action_t* action_snapshot = calloc(action_count, sizeof(*action_snapshot));
        if (action_snapshot == NULL) {
            for (size_t i = 0; i < action_count; i++) {
                invoke_deferred_action(ctx, &deferred->actions[i]);
                if (!ctx->initialized || ctx->magic_cookie != 0xDEADBEEF) {
                    break;
                }
            }
            deferred->count = 0;
            return;
        }

        memcpy(action_snapshot, deferred->actions, action_count * sizeof(*action_snapshot));
        deferred->count = 0;

        for (size_t i = 0; i < action_count; i++) {
            invoke_deferred_action(ctx, &action_snapshot[i]);
            if (!ctx->initialized || ctx->magic_cookie != 0xDEADBEEF) {
                break;
            }
        }

        free(action_snapshot);
        return;
    }

    for (size_t i = 0; i < deferred->count; i++) {
        ax25_conn_deferred_action_t* action = &deferred->actions[i];

        esp_err_t err = ax25_conn_dispatcher_enqueue(ctx, action);
        if (err == ESP_ERR_NO_MEM) {
            invoke_deferred_action(ctx, action);
        }
    }

    deferred->count = 0;
}

/*******************************************************************************
 * Frame Transmission
 ******************************************************************************/

/**
 * @brief Record one outbound frame for later delivery through `on_tx_frame`.
 * @param ctx Connection producing the frame.
 * @param frame Outgoing frame to copy.
 * @param deferred Deferred buffer receiving the frame action.
 * @return ESP_OK on success or ESP_ERR_NO_MEM if the deferred buffer is full.
 */
static esp_err_t send_frame_internal(ax25_conn_t* ctx,
                                     const ax25_frame_t* frame,
                                     ax25_conn_deferred_t* deferred) {
    ax25_conn_deferred_action_t action = {
        .type = AX25_CONN_DISPATCH_ACTION_FRAME,
    };

    log_conn_frame("ax25_conn_tx", frame);

    action.payload.frame = *frame;
    if (!defer_action(deferred, action)) {
        ESP_LOGE(TAG, "Deferred action queue full while recording transmit frame");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief Build and queue a SABM command frame.
 * @param ctx Connection sending the frame.
 * @param poll Whether to set the P bit.
 * @param deferred Deferred buffer receiving the outbound frame action.
 * @return Result from the deferred transmit path.
 */
static esp_err_t send_sabm(ax25_conn_t* ctx, bool poll, ax25_conn_deferred_t* deferred) {
    ax25_frame_t frame;
    ax25_frame_init(&frame);
    
    frame.destination = ctx->remote_addr;
    frame.source = ctx->local_addr;
    apply_conn_path(ctx, &frame);
    frame.is_command = true;
    frame.type = AX25_FRAME_U;
    frame.control = AX25_CTRL_SABM | (poll ? AX25_CTRL_PF_BIT : 0);
    
    ESP_LOGD(TAG, "Sending SABM (P=%d)", poll);
        return send_frame_internal(ctx, &frame, deferred);
}

/**
 * @brief Build and queue a DISC command frame.
 * @param ctx Connection sending the frame.
 * @param poll Whether to set the P bit.
 * @param deferred Deferred buffer receiving the outbound frame action.
 * @return Result from the deferred transmit path.
 */
    static esp_err_t send_disc(ax25_conn_t* ctx, bool poll, ax25_conn_deferred_t* deferred) {
    ax25_frame_t frame;
    ax25_frame_init(&frame);
    
    frame.destination = ctx->remote_addr;
    frame.source = ctx->local_addr;
    apply_conn_path(ctx, &frame);
    frame.is_command = true;
    frame.type = AX25_FRAME_U;
    frame.control = AX25_CTRL_DISC | (poll ? AX25_CTRL_PF_BIT : 0);
    
    ESP_LOGD(TAG, "Sending DISC (P=%d)", poll);
        return send_frame_internal(ctx, &frame, deferred);
}

/**
 * @brief Build and queue a UA response frame.
 * @param ctx Connection sending the frame.
 * @param final Whether to set the F bit.
 * @param deferred Deferred buffer receiving the outbound frame action.
 * @return Result from the deferred transmit path.
 */
    static esp_err_t send_ua(ax25_conn_t* ctx, bool final, ax25_conn_deferred_t* deferred) {
    ax25_frame_t frame;
    ax25_frame_init(&frame);
    
    frame.destination = ctx->remote_addr;
    frame.source = ctx->local_addr;
    apply_conn_path(ctx, &frame);
    frame.is_command = false;
    frame.type = AX25_FRAME_U;
    frame.control = AX25_CTRL_UA | (final ? AX25_CTRL_PF_BIT : 0);
    
    ESP_LOGD(TAG, "Sending UA (F=%d)", final);
        return send_frame_internal(ctx, &frame, deferred);
}

/**
 * @brief Build and queue a DM response frame.
 * @param ctx Connection sending the frame.
 * @param final Whether to set the F bit.
 * @param deferred Deferred buffer receiving the outbound frame action.
 * @return Result from the deferred transmit path.
 */
    static esp_err_t send_dm(ax25_conn_t* ctx, bool final, ax25_conn_deferred_t* deferred) {
    ax25_frame_t frame;
    ax25_frame_init(&frame);
    
    frame.destination = ctx->remote_addr;
    frame.source = ctx->local_addr;
    apply_conn_path(ctx, &frame);
    frame.is_command = false;
    frame.type = AX25_FRAME_U;
    frame.control = AX25_CTRL_DM | (final ? AX25_CTRL_PF_BIT : 0);
    
    ESP_LOGD(TAG, "Sending DM (F=%d)", final);
        return send_frame_internal(ctx, &frame, deferred);
}

/**
 * @brief Build and queue an RR supervisory frame.
 * @param ctx Connection sending the frame.
 * @param pf Whether to set the P/F bit.
 * @param is_command Whether the frame is sent as a command.
 * @param deferred Deferred buffer receiving the outbound frame action.
 * @return Result from the deferred transmit path.
 */
    static esp_err_t send_rr(ax25_conn_t* ctx,
                             bool pf,
                             bool is_command,
                             ax25_conn_deferred_t* deferred) {
    ax25_frame_t frame;
    ax25_frame_init(&frame);
    
    frame.destination = ctx->remote_addr;
    frame.source = ctx->local_addr;
    apply_conn_path(ctx, &frame);
    frame.is_command = is_command;
    frame.type = AX25_FRAME_S;
    frame.control = ax25_frame_build_rr_control(ctx->vr, pf);
    
    ESP_LOGD(TAG, "Sending RR N(R)=%d P/F=%d cmd=%d", ctx->vr, pf, is_command);
        return send_frame_internal(ctx, &frame, deferred);
}

/**
 * @brief Build and queue an RNR supervisory frame.
 * @param ctx Connection sending the frame.
 * @param pf Whether to set the P/F bit.
 * @param is_command Whether the frame is sent as a command.
 * @param deferred Deferred buffer receiving the outbound frame action.
 * @return Result from the deferred transmit path.
 */
    static esp_err_t send_rnr(ax25_conn_t* ctx,
                              bool pf,
                              bool is_command,
                              ax25_conn_deferred_t* deferred) {
    ax25_frame_t frame;
    ax25_frame_init(&frame);
    
    frame.destination = ctx->remote_addr;
    frame.source = ctx->local_addr;
    apply_conn_path(ctx, &frame);
    frame.is_command = is_command;
    frame.type = AX25_FRAME_S;
    frame.control = ax25_frame_build_rnr_control(ctx->vr, pf);
    
    ESP_LOGD(TAG, "Sending RNR N(R)=%d P/F=%d cmd=%d", ctx->vr, pf, is_command);
        return send_frame_internal(ctx, &frame, deferred);
}

/**
 * @brief Send either RR or RNR depending on local-busy state.
 * @param ctx Connection sending the acknowledgement.
 * @param pf Whether to set the P/F bit.
 * @param is_command Whether the frame is sent as a command.
 * @param deferred Deferred buffer receiving the outbound frame action.
 * @return Result from the deferred transmit path.
 */
    static esp_err_t send_ack(ax25_conn_t* ctx,
                              bool pf,
                              bool is_command,
                              ax25_conn_deferred_t* deferred) {
    if (ctx->local_busy) {
            return send_rnr(ctx, pf, is_command, deferred);
    }
        return send_rr(ctx, pf, is_command, deferred);
}

/**
 * @brief Build and queue a REJ supervisory frame.
 * @param ctx Connection sending the frame.
 * @param pf Whether to set the P/F bit.
 * @param is_command Whether the frame is sent as a command.
 * @param deferred Deferred buffer receiving the outbound frame action.
 * @return Result from the deferred transmit path.
 */
    static esp_err_t send_rej(ax25_conn_t* ctx,
                              bool pf,
                              bool is_command,
                              ax25_conn_deferred_t* deferred) {
    ax25_frame_t frame;
    ax25_frame_init(&frame);
    
    frame.destination = ctx->remote_addr;
    frame.source = ctx->local_addr;
    apply_conn_path(ctx, &frame);
    frame.is_command = is_command;
    frame.type = AX25_FRAME_S;
    frame.control = ax25_frame_build_rej_control(ctx->vr, pf);
    
    ESP_LOGD(TAG, "Sending REJ N(R)=%d P/F=%d cmd=%d", ctx->vr, pf, is_command);
    ctx->rej_sent = true;
        return send_frame_internal(ctx, &frame, deferred);
}

/**
 * @brief Build and queue one I frame from queued payload data.
 * @param ctx Connection sending the frame.
 * @param ns Send sequence number for the frame.
 * @param data Payload bytes to transmit.
 * @param len Payload length in bytes.
 * @param deferred Deferred buffer receiving the outbound frame action.
 * @return Result from the deferred transmit path.
 */
    static esp_err_t send_i_frame(ax25_conn_t* ctx,
                                  uint8_t ns,
                                  const uint8_t* data,
                                  size_t len,
                                  ax25_conn_deferred_t* deferred) {
    ax25_frame_t frame;
    ax25_frame_init(&frame);
    
    frame.destination = ctx->remote_addr;
    frame.source = ctx->local_addr;
    apply_conn_path(ctx, &frame);
    frame.is_command = true;
    frame.type = AX25_FRAME_I;
    frame.control = ax25_frame_build_i_control(ns, ctx->vr, false);
    frame.pid = AX25_PID_NONE;
    
    if (len > 0 && len <= AX25_MAX_INFO_LEN) {
        memcpy(frame.payload, data, len);
        frame.payload_len = len;
    }
    
    ESP_LOGD(TAG, "Sending I N(S)=%d N(R)=%d len=%zu", ns, ctx->vr, len);
    ctx->ack_pending = false;  /* Piggyback ack in I frame */
        return send_frame_internal(ctx, &frame, deferred);
}

/**
 * @brief Send every queued I frame that is eligible for first transmit or retry.
 * @param ctx Connection whose transmit queue should be drained.
 * @param deferred Deferred buffer receiving outbound frame actions.
 * @return ESP_OK on success or the first transmit error encountered.
 */
    static esp_err_t send_pending_i_frames(ax25_conn_t* ctx, ax25_conn_deferred_t* deferred) {
    if (ctx->peer_busy) {
        return ESP_OK;  /* Don't send if peer is busy */
    }

    /* Send only frames that have not yet been transmitted */
    for (uint8_t seq = ctx->va; seq != ctx->vs; seq = MOD8(seq + 1)) {
        ax25_conn_pending_frame_t* pf = &ctx->tx_queue[seq];
        if (pf->in_use && !pf->transmitted) {
            esp_err_t ret = send_i_frame(ctx, seq, pf->data, pf->len, deferred);
            if (ret != ESP_OK) {
                return ret;
            }
            pf->transmitted = true;
            start_timer_t1(ctx);
            stop_timer_t3(ctx);
        }
    }

    return ESP_OK;
}

/**
 * @brief Queue payload bytes into the modulo-8 transmit window.
 * @param ctx Connection whose pending I-frame queue is being updated.
 * @param data Payload bytes to copy.
 * @param len Payload length in bytes.
 * @return ESP_OK on success or ESP_ERR_NO_MEM if the send window is full.
 */
static esp_err_t queue_i_frame(ax25_conn_t* ctx, const uint8_t* data, size_t len) {
    if (outstanding_frames(ctx) >= ctx->config.window_size) {
        log_conn_drop(ctx, NULL, "tx window full");
        return ESP_ERR_NO_MEM;
    }
    
    ax25_conn_pending_frame_t* pf = &ctx->tx_queue[ctx->vs];
    if (pf->in_use) {
        log_conn_drop(ctx, NULL, "tx queue slot already in use");
        return ESP_ERR_NO_MEM;
    }
    
    memcpy(pf->data, data, len);
    pf->len = len;
    pf->ns = ctx->vs;
    pf->in_use = true;
    pf->transmitted = false;
    
    ctx->vs = MOD8(ctx->vs + 1);
    
    return ESP_OK;
}

/*******************************************************************************
 * N(R) Handling
 ******************************************************************************/

/**
 * @brief Check whether an incoming N(R) falls within the valid outstanding range.
 * @param ctx Connection whose sequence state is being checked.
 * @param nr Incoming acknowledgement number.
 * @return true if `nr` is valid for the current modulo-8 send window.
 */
static bool is_valid_nr(ax25_conn_t* ctx, uint8_t nr) {
    /* N(R) is valid if it acknowledges frames between V(A) and V(S) (inclusive) */
    /* V(A) <= N(R) <= V(S) in modulo 8 arithmetic */
    uint8_t va = ctx->va;
    uint8_t vs = ctx->vs;
    
    if (va <= vs) {
        return (nr >= va && nr <= vs);
    } else {
        return (nr >= va || nr <= vs);
    }
}

/**
 * @brief Apply an incoming N(R) and retire any newly acknowledged frames.
 * @param ctx Connection whose send window should be advanced.
 * @param nr Acknowledgement number extracted from the incoming frame.
 * @param frame Frame that carried the acknowledgement.
 */
static void handle_received_nr(ax25_conn_t* ctx, uint8_t nr, const ax25_frame_t* frame) {
    bool ack_progress = (ctx->va != nr);

    if (!is_valid_nr(ctx, nr)) {
        ESP_LOGW(TAG, "Invalid N(R)=%d (va=%d vs=%d)", nr, ctx->va, ctx->vs);
        return;
    }
    
    /* Acknowledge frames from V(A) up to (but not including) N(R) */
    while (ctx->va != nr) {
        ax25_conn_pending_frame_t* pf = &ctx->tx_queue[ctx->va];
        pf->in_use = false;
        ctx->va = MOD8(ctx->va + 1);
    }
    (void)frame;

    if (ctx->state == AX25_CONN_STATE_CONNECTED) {
        if (ctx->va == ctx->vs) {
            stop_timer_t1(ctx);
            start_timer_t3(ctx);
        } else if (ack_progress) {
            stop_timer_t3(ctx);
            start_timer_t1(ctx);
        }
    }
}

/**
 * @brief Enter active recovery after detecting an invalid acknowledgement number.
 * @param ctx Connection entering recovery.
 * @param msg Error message reported to the application.
 * @param deferred Deferred buffer receiving the error and recovery frames.
 * @return ESP_ERR_INVALID_RESPONSE.
 */
static esp_err_t start_nr_error_recovery(ax25_conn_t* ctx,
                                         const char* msg,
                                         ax25_conn_deferred_t* deferred) {
    stop_all_timers(ctx);
    defer_on_error(ctx, ESP_ERR_INVALID_RESPONSE, msg, deferred);
    reset_state_vars(ctx);
    send_sabm(ctx, true, deferred);
    ctx->state = AX25_CONN_STATE_AWAITING_CONNECTION;
    start_timer_t1(ctx);
    return ESP_ERR_INVALID_RESPONSE;
}

/*******************************************************************************
 * State Handlers
 ******************************************************************************/

/**
 * @brief Handle an incoming frame while no session is established.
 * @param ctx Connection processing the frame.
 * @param frame Incoming frame.
 * @param deferred Deferred buffer receiving any resulting callbacks or TX work.
 * @return ESP_OK for handled or ignored frames.
 */
static esp_err_t handle_frame_disconnected(ax25_conn_t* ctx,
                                           const ax25_frame_t* frame,
                                           ax25_conn_deferred_t* deferred) {
    bool pf = ax25_frame_has_pf(frame->control);
    
    if (frame->type == AX25_FRAME_U) {
        uint8_t u_type = frame->control & 0xEF;  /* Mask out P/F */
        
        if (u_type == AX25_CTRL_SABM) {
            /* Incoming connection request */
            ctx->remote_addr = frame->source;
            ctx->remote_addr_set = true;
            set_conn_path_from_frame(ctx, frame);
            seed_t1_for_path(ctx, frame->num_digipeaters);
            
            /* Notify application without holding the conn mutex. */
            invoke_on_connect_unlocked(ctx, false);
            
            if (ctx->refuse_pending) {
                /* Application refused connection */
                send_dm(ctx, pf, deferred);
                enter_disconnected_state(ctx, deferred);
                return ESP_OK;
            }
            
            /* Accept connection */
            reset_state_vars(ctx);
            send_ua(ctx, pf, deferred);
            ctx->state = AX25_CONN_STATE_CONNECTED;
            start_timer_t3(ctx);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_DISC) {
            /* DISC while disconnected, respond with DM */
            send_dm(ctx, pf, deferred);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_UA || u_type == AX25_CTRL_DM) {
            /* Ignore UA/DM in disconnected state */
            return ESP_OK;
        }
    }
    
    /* Ignore other frames in disconnected state */
    return ESP_OK;
}

/**
 * @brief Handle an incoming frame while awaiting connection establishment.
 * @param ctx Connection processing the frame.
 * @param frame Incoming frame.
 * @param deferred Deferred buffer receiving any resulting callbacks or TX work.
 * @return ESP_OK for handled or ignored frames.
 */
static esp_err_t handle_frame_awaiting_connection(ax25_conn_t* ctx,
                                                  const ax25_frame_t* frame,
                                                  ax25_conn_deferred_t* deferred) {
    bool pf = ax25_frame_has_pf(frame->control);
    
    if (frame->type == AX25_FRAME_U) {
        uint8_t u_type = frame->control & 0xEF;
        
        if (u_type == AX25_CTRL_UA) {
            if (pf) {
                /* Connection accepted */
                stop_timer_t1(ctx);
                reset_state_vars(ctx);
                ctx->state = AX25_CONN_STATE_CONNECTED;
                start_timer_t3(ctx);
                invoke_on_connect_unlocked(ctx, true);
                return ESP_OK;
            }
        }
        else if (u_type == AX25_CTRL_DM) {
            /* Connection refused */
            stop_timer_t1(ctx);
            defer_on_error(ctx, ESP_ERR_NOT_ALLOWED, "Connection refused (DM)", deferred);
            enter_disconnected_state(ctx, deferred);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_SABM) {
            /* Collision: both sides trying to connect */
            /* Accept their SABM */
            set_conn_path_from_frame(ctx, frame);
            seed_t1_for_path(ctx, frame->num_digipeaters);
            reset_state_vars(ctx);
            send_ua(ctx, pf, deferred);
            stop_timer_t1(ctx);
            ctx->state = AX25_CONN_STATE_CONNECTED;
            start_timer_t3(ctx);
            invoke_on_connect_unlocked(ctx, true);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_DISC) {
            /* DISC while awaiting connection is rejected with DM. */
            send_dm(ctx, pf, deferred);
            return ESP_OK;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Handle an incoming frame while awaiting disconnect completion.
 * @param ctx Connection processing the frame.
 * @param frame Incoming frame.
 * @param deferred Deferred buffer receiving any resulting callbacks or TX work.
 * @return ESP_OK for handled or ignored frames.
 */
static esp_err_t handle_frame_awaiting_release(ax25_conn_t* ctx,
                                               const ax25_frame_t* frame,
                                               ax25_conn_deferred_t* deferred) {
    bool pf = ax25_frame_has_pf(frame->control);
    
    if (frame->type == AX25_FRAME_U) {
        uint8_t u_type = frame->control & 0xEF;
        
        if (u_type == AX25_CTRL_UA || u_type == AX25_CTRL_DM) {
            /* Disconnection confirmed */
            stop_timer_t1(ctx);
            enter_disconnected_state(ctx, deferred);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_SABM) {
            /* Remote trying to reconnect while we're disconnecting */
            send_dm(ctx, pf, deferred);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_DISC) {
            /* Both sides disconnecting */
            send_ua(ctx, pf, deferred);
            stop_timer_t1(ctx);
            enter_disconnected_state(ctx, deferred);
            return ESP_OK;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Handle an incoming frame during the normal connected state.
 * @param ctx Connection processing the frame.
 * @param frame Incoming frame.
 * @param deferred Deferred buffer receiving callbacks and outbound responses.
 * @return ESP_OK on normal handling or an error from active recovery.
 */
static esp_err_t handle_frame_connected(ax25_conn_t* ctx,
                                        const ax25_frame_t* frame,
                                        ax25_conn_deferred_t* deferred) {
    bool pf = ax25_frame_has_pf(frame->control);
    
    /* Handle U frames */
    if (frame->type == AX25_FRAME_U) {
        uint8_t u_type = frame->control & 0xEF;
        
        if (u_type == AX25_CTRL_SABM) {
            /* Link reset while connected (application callback is reset-specific) */
            defer_on_link_reset(deferred);
            set_conn_path_from_frame(ctx, frame);
            seed_t1_for_path(ctx, frame->num_digipeaters);
            reset_state_vars(ctx);
            stop_timer_t1(ctx);
            send_ua(ctx, pf, deferred);
            start_timer_t3(ctx);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_DISC) {
            /* Disconnection request */
            send_ua(ctx, pf, deferred);
            enter_disconnected_state(ctx, deferred);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_DM) {
            /* Remote reports disconnected */
            defer_on_error(ctx, ESP_ERR_INVALID_STATE, "Received DM while connected", deferred);
            enter_disconnected_state(ctx, deferred);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_FRMR) {
            /* Frame reject */
            defer_on_error(ctx, ESP_ERR_INVALID_RESPONSE, "Received FRMR", deferred);
            /* Reset link */
            reset_state_vars(ctx);
            send_sabm(ctx, true, deferred);
            ctx->state = AX25_CONN_STATE_AWAITING_CONNECTION;
            start_timer_t1(ctx);
            return ESP_OK;
        }
        return ESP_OK;
    }
    
    /* Handle S frames */
    if (frame->type == AX25_FRAME_S) {
        uint8_t s_type = frame->control & 0x0F;
        uint8_t nr = ax25_frame_extract_nr(frame->control);
        
        if (!is_valid_nr(ctx, nr)) {
            return start_nr_error_recovery(ctx, "Invalid N(R) while connected", deferred);
        }
        
        handle_received_nr(ctx, nr, frame);
        
        if (s_type == AX25_CTRL_RR_MASK) {
            /* RR */
            ctx->peer_busy = false;
            if (pf && frame->is_command) {
                send_ack(ctx, true, false, deferred);
            }
            send_pending_i_frames(ctx, deferred);
        }
        else if (s_type == AX25_CTRL_RNR_MASK) {
            /* RNR */
            ctx->peer_busy = true;
            if (pf && frame->is_command) {
                send_ack(ctx, true, false, deferred);
            }
        }
        else if (s_type == AX25_CTRL_REJ_MASK) {
            /* REJ - retransmit from N(R): mark frames as untransmitted */
            ctx->peer_busy = false;
            if (pf && frame->is_command) {
                send_ack(ctx, true, false, deferred);
            }
            /* Mark frames from N(R) onwards as untransmitted so they get resent */
            for (uint8_t seq = nr; seq != ctx->vs; seq = MOD8(seq + 1)) {
                ctx->tx_queue[seq].transmitted = false;
            }
            send_pending_i_frames(ctx, deferred);
        }
        
        return ESP_OK;
    }
    
    /* Handle I frames */
    if (frame->type == AX25_FRAME_I) {
        uint8_t ns = ax25_frame_extract_ns(frame->control);
        uint8_t nr = ax25_frame_extract_nr(frame->control);
        
        /* Check N(R) validity */
        if (!is_valid_nr(ctx, nr)) {
            return start_nr_error_recovery(ctx,
                                           "Invalid N(R) in I frame while connected",
                                           deferred);
        }
        
        handle_received_nr(ctx, nr, frame);
        ctx->peer_busy = false;
        
        /* Check N(S) */
        if (ns == ctx->vr) {
            if (ctx->local_busy) {
                /* In local-busy mode, do not consume payload or advance V(R). */
                if (pf) {
                    send_ack(ctx, true, false, deferred);
                } else {
                    ctx->ack_pending = true;
                    start_timer_t2(ctx);
                }
                start_timer_t3(ctx);
                send_pending_i_frames(ctx, deferred);
                return ESP_OK;
            }

            /* Expected frame */
            ctx->vr = MOD8(ctx->vr + 1);
            ctx->rej_sent = false;
            
            /* Deliver data to application */
            if (frame->payload_len > 0) {
                defer_on_data(frame->payload, frame->payload_len, deferred);
            }
            
            /* Send acknowledgement */
            if (pf) {
                send_ack(ctx, true, false, deferred);
            } else {
                ctx->ack_pending = true;
                start_timer_t2(ctx);
            }
            
            start_timer_t3(ctx);
        }
        else {
            /* Out of sequence */
            if (!ctx->rej_sent) {
                send_rej(ctx, pf, false, deferred);
            } else if (pf) {
                send_ack(ctx, true, false, deferred);
            }
        }
        
        /* Try to send pending I frames */
        send_pending_i_frames(ctx, deferred);
        
        return ESP_OK;
    }
    
    return ESP_OK;
}

/**
 * @brief Handle an incoming frame while T1 recovery is in progress.
 * @param ctx Connection processing the frame.
 * @param frame Incoming frame.
 * @param deferred Deferred buffer receiving callbacks and outbound responses.
 * @return ESP_OK on normal handling or an error from active recovery.
 */
static esp_err_t handle_frame_timer_recovery(ax25_conn_t* ctx,
                                             const ax25_frame_t* frame,
                                             ax25_conn_deferred_t* deferred) {
    bool pf = ax25_frame_has_pf(frame->control);
    
    /* Handle U frames */
    if (frame->type == AX25_FRAME_U) {
        uint8_t u_type = frame->control & 0xEF;
        
        if (u_type == AX25_CTRL_SABM) {
            set_conn_path_from_frame(ctx, frame);
            seed_t1_for_path(ctx, frame->num_digipeaters);
            reset_state_vars(ctx);
            stop_timer_t1(ctx);
            send_ua(ctx, pf, deferred);
            ctx->state = AX25_CONN_STATE_CONNECTED;
            start_timer_t3(ctx);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_DISC) {
            send_ua(ctx, pf, deferred);
            enter_disconnected_state(ctx, deferred);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_DM) {
            defer_on_error(ctx, ESP_ERR_INVALID_STATE, "Received DM in timer recovery", deferred);
            enter_disconnected_state(ctx, deferred);
            return ESP_OK;
        }
        else if (u_type == AX25_CTRL_FRMR) {
            defer_on_error(ctx,
                           ESP_ERR_INVALID_RESPONSE,
                           "Received FRMR in timer recovery",
                           deferred);
            reset_state_vars(ctx);
            send_sabm(ctx, true, deferred);
            ctx->state = AX25_CONN_STATE_AWAITING_CONNECTION;
            start_timer_t1(ctx);
            return ESP_OK;
        }
        return ESP_OK;
    }
    
    /* Handle S frames */
    if (frame->type == AX25_FRAME_S) {
        uint8_t s_type = frame->control & 0x0F;
        uint8_t nr = ax25_frame_extract_nr(frame->control);
        bool response_final = (pf && !frame->is_command);
        bool retransmit_outstanding = false;
        
        if (!is_valid_nr(ctx, nr)) {
            return start_nr_error_recovery(ctx, "Invalid N(R) in timer recovery", deferred);
        }
        
        handle_received_nr(ctx, nr, frame);
        
        if (response_final) {
            /* Response with F bit concludes the recovery exchange. */
            stop_timer_t1(ctx);
            ctx->retry_count = 0;
            ctx->state = AX25_CONN_STATE_CONNECTED;
            if (ctx->va != ctx->vs) {
                for (uint8_t seq = ctx->va; seq != ctx->vs; seq = MOD8(seq + 1)) {
                    ctx->tx_queue[seq].transmitted = false;
                }
                retransmit_outstanding = true;
            }
        }
        
        if (s_type == AX25_CTRL_RR_MASK) {
            ctx->peer_busy = false;
            if (pf && frame->is_command) {
                send_ack(ctx, true, false, deferred);
            }
        }
        else if (s_type == AX25_CTRL_RNR_MASK) {
            ctx->peer_busy = true;
            if (pf && frame->is_command) {
                send_ack(ctx, true, false, deferred);
            }
        }
        else if (s_type == AX25_CTRL_REJ_MASK) {
            ctx->peer_busy = false;
            if (pf && frame->is_command) {
                send_ack(ctx, true, false, deferred);
            }
            /* Mark frames from N(R) onwards as untransmitted so they get resent */
            for (uint8_t seq = nr; seq != ctx->vs; seq = MOD8(seq + 1)) {
                ctx->tx_queue[seq].transmitted = false;
            }
            retransmit_outstanding = (ctx->va != ctx->vs);
        }
        
        if (ctx->state == AX25_CONN_STATE_CONNECTED) {
            if (ctx->va == ctx->vs) {
                stop_timer_t1(ctx);
                start_timer_t3(ctx);
            } else {
                stop_timer_t3(ctx);
                if (retransmit_outstanding) {
                    send_pending_i_frames(ctx, deferred);
                }
                start_timer_t1(ctx);
            }
        } else if (retransmit_outstanding) {
            send_pending_i_frames(ctx, deferred);
        }

        return ESP_OK;
    }
    
    /* Handle I frames */
    if (frame->type == AX25_FRAME_I) {
        uint8_t ns = ax25_frame_extract_ns(frame->control);
        uint8_t nr = ax25_frame_extract_nr(frame->control);
        
        if (!is_valid_nr(ctx, nr)) {
            return start_nr_error_recovery(ctx,
                                           "Invalid N(R) in I frame during timer recovery",
                                           deferred);
        }
        
        handle_received_nr(ctx, nr, frame);
        ctx->peer_busy = false;
        
        if (ns == ctx->vr) {
            if (ctx->local_busy) {
                /* In local-busy mode, do not consume payload or advance V(R). */
                if (pf) {
                    send_ack(ctx, true, false, deferred);
                } else {
                    ctx->ack_pending = true;
                    start_timer_t2(ctx);
                }
                return ESP_OK;
            }

            ctx->vr = MOD8(ctx->vr + 1);
            ctx->rej_sent = false;
            
            if (frame->payload_len > 0) {
                defer_on_data(frame->payload, frame->payload_len, deferred);
            }
            
            if (pf) {
                send_ack(ctx, true, false, deferred);
            } else {
                ctx->ack_pending = true;
                start_timer_t2(ctx);
            }
        }
        else {
            if (!ctx->rej_sent) {
                send_rej(ctx, pf, false, deferred);
            } else if (pf) {
                send_ack(ctx, true, false, deferred);
            }
        }

        return ESP_OK;
    }
    
    return ESP_OK;
}

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

/**
 * @brief Initialize one connected-mode AX.25 session context.
 * @param ctx Caller-allocated connection context.
 * @param local_addr Local AX.25 source address.
 * @param callbacks Callback table used for data, frame output, and events.
 * @param user_data Opaque pointer passed back to callbacks.
 * @param config Optional configuration, or NULL for defaults.
 * @return ESP_OK on success, or an error if required callbacks or resources are missing.
 */
esp_err_t ax25_conn_init(ax25_conn_t* ctx,
                         const ax25_address_t* local_addr,
                         const ax25_conn_callbacks_t* callbacks,
                         void* user_data,
                         const ax25_conn_config_t* config) {
    if (!ctx || !local_addr || !callbacks) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!callbacks->on_tx_frame || !callbacks->on_data) {
        ESP_LOGE(TAG, "on_tx_frame and on_data callbacks are required");
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(ctx, 0, sizeof(ax25_conn_t));
    
    /* Apply configuration */
    if (config) {
        ctx->config = *config;
    } else {
        ctx->config = (ax25_conn_config_t)AX25_CONN_CONFIG_DEFAULT();
    }
    
    /* Validate window size */
    if (ctx->config.window_size < 1 || ctx->config.window_size > 7) {
        ctx->config.window_size = 4;
    }
    
    /* Store addresses and callbacks */
    ctx->local_addr = *local_addr;
    ctx->callbacks = *callbacks;
    ctx->user_data = user_data;
    
    /* Initialize state */
    ctx->state = AX25_CONN_STATE_DISCONNECTED;
    reset_state_vars(ctx);
    ctx->t1_current_ms = ctx->config.t1_ms;

    esp_err_t ret;

    /* Create mutex */
    ctx->mutex = xSemaphoreCreateMutex();
    if (!ctx->mutex) {
        return ESP_ERR_NO_MEM;
    }

    ctx->dispatcher_quiesced_sem = xSemaphoreCreateBinary();
    if (!ctx->dispatcher_quiesced_sem) {
        vSemaphoreDelete(ctx->mutex);
        ctx->mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    ret = ax25_conn_dispatcher_init_once();
    if (ret != ESP_OK) {
        vSemaphoreDelete(ctx->dispatcher_quiesced_sem);
        ctx->dispatcher_quiesced_sem = NULL;
        vSemaphoreDelete(ctx->mutex);
        ctx->mutex = NULL;
        return ret;
    }

    ctx->dispatcher_generation = 1;
    ctx->dispatcher_shutdown = false;

    for (uint8_t i = 0; i < AX25_CONN_DEFERRED_POOL_SIZE; i++) {
        ctx->deferred_pool[i] = alloc_deferred_storage();
        if (ctx->deferred_pool[i] == NULL) {
            for (uint8_t j = 0; j < i; j++) {
                free_deferred_storage((ax25_conn_deferred_t*)ctx->deferred_pool[j]);
                ctx->deferred_pool[j] = NULL;
            }
            vSemaphoreDelete(ctx->dispatcher_quiesced_sem);
            ctx->dispatcher_quiesced_sem = NULL;
            vSemaphoreDelete(ctx->mutex);
            ctx->mutex = NULL;
            return ESP_ERR_NO_MEM;
        }
    }

    /* Create timers */
    esp_timer_create_args_t t1_args = {
        .callback = timer_t1_callback,
        .arg = ctx,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ax25_t1"
    };
    ret = esp_timer_create(&t1_args, &ctx->timer_t1);
    if (ret != ESP_OK) {
        for (uint8_t i = 0; i < AX25_CONN_DEFERRED_POOL_SIZE; i++) {
            free_deferred_storage((ax25_conn_deferred_t*)ctx->deferred_pool[i]);
            ctx->deferred_pool[i] = NULL;
        }
        vSemaphoreDelete(ctx->dispatcher_quiesced_sem);
        ctx->dispatcher_quiesced_sem = NULL;
        vSemaphoreDelete(ctx->mutex);
        return ret;
    }

    esp_timer_create_args_t t2_args = {
        .callback = timer_t2_callback,
        .arg = ctx,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ax25_t2"
    };
    ret = esp_timer_create(&t2_args, &ctx->timer_t2);
    if (ret != ESP_OK) {
        esp_timer_delete(ctx->timer_t1);
        for (uint8_t i = 0; i < AX25_CONN_DEFERRED_POOL_SIZE; i++) {
            free_deferred_storage((ax25_conn_deferred_t*)ctx->deferred_pool[i]);
            ctx->deferred_pool[i] = NULL;
        }
        vSemaphoreDelete(ctx->dispatcher_quiesced_sem);
        ctx->dispatcher_quiesced_sem = NULL;
        vSemaphoreDelete(ctx->mutex);
        return ret;
    }

    esp_timer_create_args_t t3_args = {
        .callback = timer_t3_callback,
        .arg = ctx,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ax25_t3"
    };
    ret = esp_timer_create(&t3_args, &ctx->timer_t3);
    if (ret != ESP_OK) {
        esp_timer_delete(ctx->timer_t2);
        esp_timer_delete(ctx->timer_t1);
        for (uint8_t i = 0; i < AX25_CONN_DEFERRED_POOL_SIZE; i++) {
            free_deferred_storage((ax25_conn_deferred_t*)ctx->deferred_pool[i]);
            ctx->deferred_pool[i] = NULL;
        }
        vSemaphoreDelete(ctx->dispatcher_quiesced_sem);
        ctx->dispatcher_quiesced_sem = NULL;
        vSemaphoreDelete(ctx->mutex);
        return ret;
    }
    
    ctx->initialized = true;
    ctx->magic_cookie = 0xDEADBEEF;
    ESP_LOGI(TAG, "Initialized (T1=%lums current_t1=%lums T2=%lums T3=%lums N2=%d k=%d)",
             (unsigned long)ctx->config.t1_ms, (unsigned long)ctx->t1_current_ms,
             (unsigned long)ctx->config.t2_ms,
             (unsigned long)ctx->config.t3_ms, ctx->config.n2_retries,
             ctx->config.window_size);
    
    return ESP_OK;
}

/**
 * @brief Deinitialize a connection context and release all owned resources.
 * @param ctx Connection context previously initialized by `ax25_conn_init()`.
 */
void ax25_conn_deinit(ax25_conn_t* ctx) {
    if (!ctx || !ctx->initialized) {
        return;
    }

    /* Serialize deinit with any in-flight callback that already passed pre-checks. */
    LOCK(ctx);
    ctx->initialized = false;
    ctx->magic_cookie = 0;
    UNLOCK(ctx);

    ax25_conn_dispatcher_quiesce(ctx);
    
    /* Stop and delete timers */
    if (ctx->timer_t1) {
        esp_timer_stop(ctx->timer_t1);
        esp_timer_delete(ctx->timer_t1);
        ctx->timer_t1 = NULL;
    }
    if (ctx->timer_t2) {
        esp_timer_stop(ctx->timer_t2);
        esp_timer_delete(ctx->timer_t2);
        ctx->timer_t2 = NULL;
    }
    if (ctx->timer_t3) {
        esp_timer_stop(ctx->timer_t3);
        esp_timer_delete(ctx->timer_t3);
        ctx->timer_t3 = NULL;
    }
    
    /* Delete mutex */
    if (ctx->mutex) {
        vSemaphoreDelete(ctx->mutex);
        ctx->mutex = NULL;
    }

    for (uint8_t i = 0; i < AX25_CONN_DEFERRED_POOL_SIZE; i++) {
        free_deferred_storage((ax25_conn_deferred_t*)ctx->deferred_pool[i]);
        ctx->deferred_pool[i] = NULL;
    }
    ctx->deferred_pool_in_use_mask = 0;
    if (ctx->dispatcher_quiesced_sem) {
        vSemaphoreDelete(ctx->dispatcher_quiesced_sem);
        ctx->dispatcher_quiesced_sem = NULL;
    }

    ESP_LOGI(TAG, "Deinitialized");
}

/**
 * @brief Begin a direct connection with no digipeater path.
 * @param ctx Connection to start.
 * @param remote_addr Remote station address.
 * @return Result from `ax25_conn_connect_via()`.
 */
esp_err_t ax25_conn_connect_direct(ax25_conn_t* ctx, const ax25_address_t* remote_addr) {
    return ax25_conn_connect_via(ctx, remote_addr, NULL, 0);
}

/**
 * @brief Begin a connection using an optional digipeater path.
 * @param ctx Connection to start.
 * @param remote_addr Remote station address.
 * @param digipeaters Optional digipeater path array.
 * @param num_digipeaters Number of digipeaters in the path.
 * @return ESP_OK if SABM was queued, or an error if state or inputs are invalid.
 */
esp_err_t ax25_conn_connect_via(ax25_conn_t* ctx,
                                const ax25_address_t* remote_addr,
                                const ax25_address_t* digipeaters,
                                uint8_t num_digipeaters) {
    ax25_conn_deferred_t* deferred = NULL;

    if (!ctx || !ctx->initialized || !remote_addr) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((digipeaters == NULL && num_digipeaters != 0) ||
        num_digipeaters > AX25_MAX_DIGIPEATERS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    LOCK(ctx);
    deferred = acquire_deferred(ctx);
    if (deferred == NULL) {
        UNLOCK(ctx);
        return ESP_ERR_NO_MEM;
    }
    
    if (ctx->state != AX25_CONN_STATE_DISCONNECTED) {
        UNLOCK(ctx);
        drain_deferred(ctx, deferred);
        release_deferred(ctx, deferred);
        return ESP_ERR_INVALID_STATE;
    }
    
    ctx->remote_addr = *remote_addr;
    ctx->remote_addr_set = true;
    ctx->is_local_initiated = true;
    ctx->retry_count = 0;

    esp_err_t path_err = set_conn_path(ctx, digipeaters, num_digipeaters);
    if (path_err != ESP_OK) {
        UNLOCK(ctx);
        drain_deferred(ctx, deferred);
        release_deferred(ctx, deferred);
        return path_err;
    }
    
    reset_state_vars(ctx);
    seed_t1_for_path(ctx, num_digipeaters);
    
    esp_err_t ret = send_sabm(ctx, true, deferred);
    if (ret == ESP_OK) {
        ctx->state = AX25_CONN_STATE_AWAITING_CONNECTION;
        start_timer_t1(ctx);
    }
    
    UNLOCK(ctx);
    drain_deferred(ctx, deferred);
        release_deferred(ctx, deferred);
    return ret;
}

/**
 * @brief Feed one incoming AX.25 frame into the connected-mode state machine.
 * @param ctx Connection processing the frame.
 * @param frame Incoming AX.25 frame.
 * @return ESP_OK for handled or ignored frames, or an error on invalid state.
 */
esp_err_t ax25_conn_on_frame(ax25_conn_t* ctx, const ax25_frame_t* frame) {
        ax25_conn_deferred_t* deferred = NULL;

    if (!ctx || !ctx->initialized || !frame) {
        return ESP_ERR_INVALID_ARG;
    }

    log_conn_frame("ax25_conn_rx", frame);

    /* Ignore UI frames */
    if (frame->type == AX25_FRAME_UI) {
        log_conn_drop(ctx, frame, "ui frame ignored by connected mode");
        return ESP_OK;
    }

    /* Check if frame is for us */
    if (!ax25_address_equals(&frame->destination, &ctx->local_addr)) {
        log_conn_drop(ctx, frame, "destination does not match local callsign");
        return ESP_OK;
    }

    /* Connected-mode processing only accepts frames whose digipeater path is
     * fully completed. Frames still in transit should be ignored here. */
    for (uint8_t i = 0; i < frame->num_digipeaters; i++) {
        if (!frame->digipeaters[i].has_been_repeated) {
            log_conn_drop(ctx, frame, "digipeater path not fully repeated");
            return ESP_OK;
        }
    }

    LOCK(ctx);
    deferred = acquire_deferred(ctx);
    if (deferred == NULL) {
        UNLOCK(ctx);
        return ESP_ERR_NO_MEM;
    }

    /* If we have a remote address set, verify source matches */
    if (ctx->remote_addr_set && ctx->state != AX25_CONN_STATE_DISCONNECTED) {
        if (!ax25_address_equals(&frame->source, &ctx->remote_addr)) {
            log_conn_drop(ctx, frame, "source does not match active remote station");
            UNLOCK(ctx);
            drain_deferred(ctx, deferred);
            release_deferred(ctx, deferred);
            return ESP_OK;  /* Ignore frames from other stations */
        }
    }

    /* Handle frame based on state */
    esp_err_t ret;
    switch (ctx->state) {
        case AX25_CONN_STATE_DISCONNECTED:
            ret = handle_frame_disconnected(ctx, frame, deferred);
            break;
        case AX25_CONN_STATE_AWAITING_CONNECTION:
            ret = handle_frame_awaiting_connection(ctx, frame, deferred);
            break;
        case AX25_CONN_STATE_AWAITING_RELEASE:
            ret = handle_frame_awaiting_release(ctx, frame, deferred);
            break;
        case AX25_CONN_STATE_CONNECTED:
            ret = handle_frame_connected(ctx, frame, deferred);
            break;
        case AX25_CONN_STATE_TIMER_RECOVERY:
            ret = handle_frame_timer_recovery(ctx, frame, deferred);
            break;
        default:
            ret = ESP_ERR_INVALID_STATE;
            break;
    }

    UNLOCK(ctx);
    drain_deferred(ctx, deferred);
    release_deferred(ctx, deferred);
    return ret;
}

/**
 * @brief Queue user payload for connected-mode transmission.
 * @param ctx Connection sending the payload.
 * @param data Payload bytes to send.
 * @param len Payload length in bytes.
 * @return ESP_OK on success, or an error if the session is not writable.
 */
esp_err_t ax25_conn_send_data(ax25_conn_t* ctx, const uint8_t* data, size_t len) {
    ax25_conn_deferred_t* deferred = NULL;

    if (!ctx || !ctx->initialized || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (len > AX25_MAX_INFO_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    LOCK(ctx);
    deferred = acquire_deferred(ctx);
    if (deferred == NULL) {
        UNLOCK(ctx);
        return ESP_ERR_NO_MEM;
    }
    
    if (ctx->state != AX25_CONN_STATE_CONNECTED &&
        ctx->state != AX25_CONN_STATE_TIMER_RECOVERY) {
        UNLOCK(ctx);
        drain_deferred(ctx, deferred);
        release_deferred(ctx, deferred);
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = queue_i_frame(ctx, data, len);
    if (ret == ESP_OK) {
        ret = send_pending_i_frames(ctx, deferred);
    }
    
    UNLOCK(ctx);
    drain_deferred(ctx, deferred);
        release_deferred(ctx, deferred);
    return ret;
}

/**
 * @brief Begin orderly disconnect of an established or pending session.
 * @param ctx Connection to shut down.
 * @return ESP_OK if shutdown is complete or DISC was queued, or an error on invalid input.
 */
esp_err_t ax25_conn_shutdown(ax25_conn_t* ctx) {
        ax25_conn_deferred_t* deferred = NULL;

    if (!ctx || !ctx->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    LOCK(ctx);
        deferred = acquire_deferred(ctx);
        if (deferred == NULL) {
            UNLOCK(ctx);
            return ESP_ERR_NO_MEM;
        }
    
    if (ctx->state == AX25_CONN_STATE_DISCONNECTED) {
        UNLOCK(ctx);
        drain_deferred(ctx, deferred);
            release_deferred(ctx, deferred);
        return ESP_OK;
    }
    
    if (ctx->state == AX25_CONN_STATE_AWAITING_RELEASE) {
        UNLOCK(ctx);
        drain_deferred(ctx, deferred);
            release_deferred(ctx, deferred);
        return ESP_OK;  /* Already shutting down */
    }
    
    stop_all_timers(ctx);
    ctx->retry_count = 0;
    
    esp_err_t ret = send_disc(ctx, true, deferred);
    if (ret == ESP_OK) {
        ctx->state = AX25_CONN_STATE_AWAITING_RELEASE;
        start_timer_t1(ctx);
    }
    
    UNLOCK(ctx);
    drain_deferred(ctx, deferred);
        release_deferred(ctx, deferred);
    return ret;
}

/**
 * @brief Mark the current inbound connection attempt for refusal.
 * @param ctx Connection currently invoking `on_connect`.
 * @param reason Optional refusal payload, currently unused.
 * @param len Length of the optional refusal payload.
 * @return ESP_OK on success or ESP_ERR_INVALID_ARG if the context is invalid.
 */
esp_err_t ax25_conn_refuse(ax25_conn_t* ctx, const uint8_t* reason, size_t len) {
    (void)reason;
    (void)len;
    
    if (!ctx || !ctx->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    /* This should be called from on_connect callback */
    ctx->refuse_pending = true;
    return ESP_OK;
}

/**
 * @brief Return the current connection state.
 * @param ctx Connection to query.
 * @return Current state, or `AX25_CONN_STATE_DISCONNECTED` for invalid contexts.
 */
ax25_conn_state_t ax25_conn_get_state(ax25_conn_t* ctx) {
    if (!ctx || !ctx->initialized) {
        return AX25_CONN_STATE_DISCONNECTED;
    }
    
    LOCK(ctx);
    ax25_conn_state_t state = ctx->state;
    UNLOCK(ctx);
    
    return state;
}

/**
 * @brief Check whether the connection is currently usable for connected-mode I/O.
 * @param ctx Connection to query.
 * @return true when connected or in timer recovery.
 */
bool ax25_conn_is_connected(ax25_conn_t* ctx) {
    ax25_conn_state_t state = ax25_conn_get_state(ctx);
    return (state == AX25_CONN_STATE_CONNECTED || 
            state == AX25_CONN_STATE_TIMER_RECOVERY);
}

    /**
     * @brief Copy out the current remote station address.
     * @param ctx Connection to query.
     * @param addr Output address destination.
     * @return ESP_OK on success, or an error if no remote is set.
     */
esp_err_t ax25_conn_get_remote_addr(ax25_conn_t* ctx, ax25_address_t* addr) {
    if (!ctx || !ctx->initialized || !addr) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK(ctx);

    if (!ctx->remote_addr_set) {
        UNLOCK(ctx);
        return ESP_ERR_INVALID_STATE;
    }

    *addr = ctx->remote_addr;
    UNLOCK(ctx);

    return ESP_OK;
}

/**
 * @brief Copy out the current connected-mode digipeater path.
 * @param ctx Connection to query.
 * @param path Output path destination.
 * @return ESP_OK on success, or an error if no remote session is set.
 */
esp_err_t ax25_conn_get_path(ax25_conn_t* ctx, ax25_conn_path_t* path) {
    if (!ctx || !ctx->initialized || !path) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK(ctx);

    if (!ctx->remote_addr_set) {
        UNLOCK(ctx);
        return ESP_ERR_INVALID_STATE;
    }

    *path = ctx->path;
    UNLOCK(ctx);

    return ESP_OK;
}

/**
 * @brief Enter or clear local-busy mode for the session.
 * @param ctx Connection to update.
 * @param busy New local-busy state.
 * @return ESP_OK on success, or an error if the context is invalid.
 */
esp_err_t ax25_conn_set_busy(ax25_conn_t* ctx, bool busy) {
    ax25_conn_deferred_t* deferred = NULL;

    if (!ctx || !ctx->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK(ctx);
    deferred = acquire_deferred(ctx);
    if (deferred == NULL) {
        UNLOCK(ctx);
        return ESP_ERR_NO_MEM;
    }

    if (ctx->local_busy != busy) {
        ctx->local_busy = busy;

        bool in_connected_state = (ctx->state == AX25_CONN_STATE_CONNECTED ||
                                   ctx->state == AX25_CONN_STATE_TIMER_RECOVERY);
        if (in_connected_state) {
            if (busy) {
                /* Immediately notify peer we are busy; cancel any pending T2 ack */
                ctx->ack_pending = false;
                stop_timer_t2(ctx);
                send_rnr(ctx, false, false, deferred);
            } else {
                /* Notify peer that we are ready to receive again */
                send_rr(ctx, false, false, deferred);
            }
        }
    }

    UNLOCK(ctx);
    drain_deferred(ctx, deferred);
    release_deferred(ctx, deferred);
    return ESP_OK;
}
