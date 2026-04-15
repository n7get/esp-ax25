#include "ax25_conn_dispatcher.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <stdlib.h>
#include <string.h>

#if CONFIG_SPIRAM
static const char* TAG = "AX25_CONN_DISP";
#endif

#if CONFIG_SPIRAM
#define AX25_CONN_DISPATCHER_DEPTH 128
#else
#define AX25_CONN_DISPATCHER_DEPTH 64
#endif

#define AX25_CONN_DISPATCHER_STACK_SIZE 4096
#define AX25_CONN_DISPATCHER_PRIORITY 9

typedef struct {
    bool in_use;
    bool cancelled;
    bool executing;
    ax25_conn_t* ctx;
    uint32_t generation;
    ax25_conn_dispatch_action_t action;
} ax25_conn_dispatch_item_t;

static QueueHandle_t s_dispatch_queue;
static TaskHandle_t s_dispatch_task;
static SemaphoreHandle_t s_dispatch_mutex;
static StaticSemaphore_t s_dispatch_mutex_buffer;
static portMUX_TYPE s_dispatch_init_lock = portMUX_INITIALIZER_UNLOCKED;
static ax25_conn_dispatch_item_t* s_dispatch_pool;
static bool s_dispatcher_initialized;

/**
 * @brief Return the shared dispatcher mutex, creating it on first use.
 *
 * The mutex is created under a small critical section so dispatcher startup
 * remains race-free across multiple callers.
 *
 * @return Shared dispatcher mutex, or NULL if creation fails.
 */
static SemaphoreHandle_t get_dispatch_mutex(void) {
    taskENTER_CRITICAL(&s_dispatch_init_lock);
    if (s_dispatch_mutex == NULL) {
        s_dispatch_mutex = xSemaphoreCreateMutexStatic(&s_dispatch_mutex_buffer);
    }
    taskEXIT_CRITICAL(&s_dispatch_init_lock);
    return s_dispatch_mutex;
}

/**
 * @brief Allocate the shared pool of queued dispatcher items.
 *
 * On PSRAM-capable targets this prefers PSRAM to keep callback buffering out
 * of scarce internal RAM, then falls back to normal heap allocation.
 *
 * @return Pointer to the allocated pool, or NULL on allocation failure.
 */
static ax25_conn_dispatch_item_t* alloc_dispatch_pool(void) {
#if CONFIG_SPIRAM
    ax25_conn_dispatch_item_t* pool = heap_caps_calloc(AX25_CONN_DISPATCHER_DEPTH,
                                                       sizeof(*pool),
                                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (pool != NULL) {
        return pool;
    }

    ESP_LOGW(TAG, "PSRAM pool allocation failed, falling back to internal RAM");
#endif

    return calloc(AX25_CONN_DISPATCHER_DEPTH, sizeof(*s_dispatch_pool));
}

/**
 * @brief Free the shared dispatcher item pool.
 *
 * Resets the global pool pointer after releasing its storage.
 */
static void free_dispatch_pool(void) {
    free(s_dispatch_pool);
    s_dispatch_pool = NULL;
}

/**
 * @brief Signal that a connection has no outstanding dispatcher work.
 *
 * When teardown is waiting for queued and in-flight callbacks to finish, this
 * gives the connection's quiesce semaphore once the counters reach zero.
 *
 * @param ctx Connection whose dispatcher state should be checked.
 */
static void maybe_signal_quiesced(ax25_conn_t* ctx) {
    if (ctx != NULL && ctx->dispatcher_shutdown &&
        ctx->dispatcher_pending_count == 0 &&
        ctx->dispatcher_executing_count == 0 &&
        ctx->dispatcher_quiesced_sem != NULL) {
        xSemaphoreGive(ctx->dispatcher_quiesced_sem);
    }
}

/**
 * @brief Execute one callback action against a connection.
 *
 * This is the final dispatch step after a queued action has been validated for
 * generation and shutdown state.
 *
 * @param ctx Connection that owns the callback table.
 * @param action Deferred callback action to invoke.
 */
static void invoke_action(ax25_conn_t* ctx, const ax25_conn_dispatch_action_t* action) {
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
 * @brief Dispatcher worker task.
 *
 * Drains queued action indices, validates each queued item against current
 * connection lifetime state, invokes eligible callbacks, and updates pending
 * and executing counters for shutdown coordination.
 *
 * @param arg Unused FreeRTOS task argument.
 */
static void dispatch_task(void* arg) {
    (void)arg;

    for (;;) {
        uint16_t index = 0;
        if (xQueueReceive(s_dispatch_queue, &index, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (index >= AX25_CONN_DISPATCHER_DEPTH) {
            continue;
        }

        ax25_conn_t* ctx = NULL;
        ax25_conn_dispatch_item_t* item = NULL;
        bool should_invoke = false;

        xSemaphoreTake(s_dispatch_mutex, portMAX_DELAY);

        item = &s_dispatch_pool[index];
        if (!item->in_use) {
            xSemaphoreGive(s_dispatch_mutex);
            continue;
        }

        if (item->cancelled || item->ctx == NULL ||
            item->generation != item->ctx->dispatcher_generation ||
            item->ctx->dispatcher_shutdown) {
            if (!item->cancelled && item->ctx != NULL && item->ctx->dispatcher_pending_count > 0) {
                item->ctx->dispatcher_pending_count--;
                maybe_signal_quiesced(item->ctx);
            }
            item->in_use = false;
            item->executing = false;
            item->cancelled = false;
            item->ctx = NULL;
            xSemaphoreGive(s_dispatch_mutex);
            continue;
        }

        ctx = item->ctx;
        item->executing = true;
        if (ctx->dispatcher_pending_count > 0) {
            ctx->dispatcher_pending_count--;
        }
        ctx->dispatcher_executing_count++;
        should_invoke = true;

        xSemaphoreGive(s_dispatch_mutex);

        if (should_invoke) {
            invoke_action(ctx, &item->action);
        }

        xSemaphoreTake(s_dispatch_mutex, portMAX_DELAY);
        if (item != NULL) {
            item->executing = false;
            item->in_use = false;
            item->cancelled = false;
            item->ctx = NULL;
        }
        if (ctx != NULL && ctx->dispatcher_executing_count > 0) {
            ctx->dispatcher_executing_count--;
            maybe_signal_quiesced(ctx);
        }
        xSemaphoreGive(s_dispatch_mutex);
    }
}

/**
 * @brief Initialize the shared connection callback dispatcher.
 *
 * This function is idempotent. The first successful call creates the shared
 * mutex, action pool, queue, and worker task used by all `ax25_conn`
 * instances.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_NO_MEM if any dispatcher resource cannot be created.
 */
esp_err_t ax25_conn_dispatcher_init_once(void) {
    SemaphoreHandle_t mutex = get_dispatch_mutex();
    if (mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(mutex, portMAX_DELAY);

    if (s_dispatcher_initialized) {
        xSemaphoreGive(mutex);
        return ESP_OK;
    }

    s_dispatch_pool = alloc_dispatch_pool();
    if (s_dispatch_pool == NULL) {
        xSemaphoreGive(mutex);
        return ESP_ERR_NO_MEM;
    }

    s_dispatch_queue = xQueueCreate(AX25_CONN_DISPATCHER_DEPTH, sizeof(uint16_t));
    if (s_dispatch_queue == NULL) {
        free_dispatch_pool();
        xSemaphoreGive(mutex);
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(dispatch_task,
                    "ax25_conn_cb",
                    AX25_CONN_DISPATCHER_STACK_SIZE,
                    NULL,
                    AX25_CONN_DISPATCHER_PRIORITY,
                    &s_dispatch_task) != pdPASS) {
        vQueueDelete(s_dispatch_queue);
        s_dispatch_queue = NULL;
        free_dispatch_pool();
        xSemaphoreGive(mutex);
        return ESP_ERR_NO_MEM;
    }

    s_dispatcher_initialized = true;
    xSemaphoreGive(mutex);
    return ESP_OK;
}

/**
 * @brief Queue one deferred callback action for a connection.
 *
 * The action is copied into a free dispatcher slot and a slot index is posted
 * to the shared worker queue. The connection generation is captured so stale
 * work can be rejected after teardown or reinitialization.
 *
 * @param ctx Connection that owns the queued callback.
 * @param action Deferred callback payload to copy into the dispatcher.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if `ctx` or `action` is NULL.
 * @return ESP_ERR_INVALID_STATE if the connection is already shutting down.
 * @return ESP_ERR_NO_MEM if the dispatcher cannot initialize or no free slot
 *         or queue space is available.
 */
esp_err_t ax25_conn_dispatcher_enqueue(ax25_conn_t* ctx,
                                       const ax25_conn_dispatch_action_t* action) {
    if (ctx == NULL || action == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ax25_conn_dispatcher_init_once();
    if (err != ESP_OK) {
        return err;
    }

    xSemaphoreTake(s_dispatch_mutex, portMAX_DELAY);

    if (ctx->dispatcher_shutdown) {
        xSemaphoreGive(s_dispatch_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    int slot = -1;
    for (int i = 0; i < AX25_CONN_DISPATCHER_DEPTH; i++) {
        if (!s_dispatch_pool[i].in_use) {
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        xSemaphoreGive(s_dispatch_mutex);
        return ESP_ERR_NO_MEM;
    }

    s_dispatch_pool[slot].in_use = true;
    s_dispatch_pool[slot].cancelled = false;
    s_dispatch_pool[slot].executing = false;
    s_dispatch_pool[slot].ctx = ctx;
    s_dispatch_pool[slot].generation = ctx->dispatcher_generation;
    s_dispatch_pool[slot].action = *action;
    ctx->dispatcher_pending_count++;

    uint16_t index = (uint16_t)slot;
    BaseType_t queued = xQueueSend(s_dispatch_queue, &index, 0);
    if (queued != pdTRUE) {
        s_dispatch_pool[slot].in_use = false;
        s_dispatch_pool[slot].ctx = NULL;
        ctx->dispatcher_pending_count--;
        xSemaphoreGive(s_dispatch_mutex);
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(s_dispatch_mutex);
    return ESP_OK;
}

/**
 * @brief Stop future callback delivery and wait for outstanding work to drain.
 *
 * Marks the connection as shutting down, cancels any queued-but-not-yet-
 * executing items for that connection, and waits for pending or active
 * callbacks to finish before returning.
 *
 * @param ctx Connection being torn down.
 */
void ax25_conn_dispatcher_quiesce(ax25_conn_t* ctx) {
    if (ctx == NULL || !s_dispatcher_initialized || s_dispatch_mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_dispatch_mutex, portMAX_DELAY);

    ctx->dispatcher_shutdown = true;
    ctx->dispatcher_generation++;

    for (int i = 0; i < AX25_CONN_DISPATCHER_DEPTH; i++) {
        ax25_conn_dispatch_item_t* item = &s_dispatch_pool[i];
        if (item->in_use && !item->executing && !item->cancelled && item->ctx == ctx) {
            item->cancelled = true;
            if (ctx->dispatcher_pending_count > 0) {
                ctx->dispatcher_pending_count--;
            }
        }
    }

    bool wait_needed = (ctx->dispatcher_pending_count != 0 ||
                        ctx->dispatcher_executing_count != 0);

    if (!wait_needed) {
        xSemaphoreGive(s_dispatch_mutex);
        return;
    }

    if (ctx->dispatcher_quiesced_sem != NULL) {
        xSemaphoreTake(ctx->dispatcher_quiesced_sem, 0);
    }

    xSemaphoreGive(s_dispatch_mutex);

    if (ctx->dispatcher_quiesced_sem != NULL) {
        xSemaphoreTake(ctx->dispatcher_quiesced_sem, portMAX_DELAY);
    }
}
