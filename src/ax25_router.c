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
 * @file ax25_router.c
 * @brief AX.25 frame router implementation
 *
 * Port contexts are stored in a fixed-size array dimensioned by Kconfig.
 * Queue backing storage prefers PSRAM when available, with SRAM fallback.
 */

#include "ax25_router.h"
#include "ax25_address.h"
#include "ax25_frame.h"
#include "ax25_print.h"
#include "ax25_config.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ax25_router";

#ifdef CONFIG_AX25_ROUTER_LOG_FRAMES
#define AX25_ROUTER_LOG_FRAMES CONFIG_AX25_ROUTER_LOG_FRAMES
#else
#define AX25_ROUTER_LOG_FRAMES 0
#endif

#ifdef CONFIG_AX25_ROUTER_LOG_PORT_STATUS
#define AX25_ROUTER_LOG_PORT_STATUS CONFIG_AX25_ROUTER_LOG_PORT_STATUS
#else
#define AX25_ROUTER_LOG_PORT_STATUS 0
#endif

/* Runtime defaults (used when the ax25_config singleton is not installed
 * or the parameter is not in the schema). */
#define ROUTER_MAX_PORTS_DEFAULT    CONFIG_AX25_ROUTER_MAX_PORTS
#define ROUTER_QUEUE_DEPTH_DEFAULT  8

#ifdef CONFIG_AX25_ROUTER_PORT_TASK_STACK_SIZE
#define ROUTER_PORT_TASK_STACK      CONFIG_AX25_ROUTER_PORT_TASK_STACK_SIZE
#else
#define ROUTER_PORT_TASK_STACK      4096
#endif

#ifdef CONFIG_AX25_ROUTER_PORT_TASK_PRIORITY
#define ROUTER_PORT_TASK_PRIORITY   CONFIG_AX25_ROUTER_PORT_TASK_PRIORITY
#else
#define ROUTER_PORT_TASK_PRIORITY   5
#endif

typedef struct {
    ax25_router_port_t *port;           /* NULL when slot is free */
    QueueHandle_t       tx_queue;
    TaskHandle_t        tx_task;
    volatile bool       running;
    volatile bool       stopped;
    bool                retired;

    /* Queue uses static FreeRTOS storage with externally managed backing bytes. */
    StaticQueue_t       queue_buf;
    uint8_t            *queue_storage_ptr;
    bool                queue_storage_psram;
} ax25_router_port_ctx_t;

typedef struct {
    ax25_router_port_ctx_t  *slots;        /* heap-allocated; count = max_ports */
    size_t                   max_ports;
    size_t                   queue_depth;
    uint8_t                 *queue_storage;      /* flat slab: max_ports * queue_depth * frame_size */
    bool                     queue_storage_psram;
    SemaphoreHandle_t        mutex;
    StaticSemaphore_t        mutex_buf;
    bool                     initialized;
} ax25_router_t;

static ax25_router_t s_router;

static void *router_alloc_prefer_psram(size_t size, bool *used_psram)
{
    if (used_psram != NULL) {
        *used_psram = false;
    }

#if defined(CONFIG_SPIRAM) && CONFIG_SPIRAM
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr != NULL) {
        if (used_psram != NULL) {
            *used_psram = true;
        }
        return ptr;
    }
#endif

    return malloc(size);
}

static esp_err_t router_alloc_queue_storage(ax25_router_port_ctx_t *ctx)
{
    /* Storage is pre-allocated in the slab at ax25_router_init; just zero it. */
    if (ctx->queue_storage_ptr == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(ctx->queue_storage_ptr, 0, s_router.queue_depth * sizeof(ax25_frame_t));
    return ESP_OK;
}

static void router_free_queue_storage(ax25_router_port_ctx_t *ctx)
{
    (void)ctx;   /* storage is owned by the s_router.queue_storage slab */
}

/*******************************************************************************
 * Helpers
 ******************************************************************************/

__attribute__((unused)) static const char *port_mode_str(ax25_router_port_mode_t mode);
__attribute__((unused)) static void port_dest_to_str(const ax25_router_port_t *port,
                                                     char *buf,
                                                     size_t buf_len);

#if AX25_ROUTER_LOG_PORT_STATUS
static void log_router_slot_inventory_locked(const char *reason)
{
    int active = 0;
    int retired = 0;
    int free_slots = 0;

    for (int i = 0; i < (int)s_router.max_ports; i++) {
        const ax25_router_port_ctx_t *ctx = &s_router.slots[i];

        if (ctx->port != NULL) {
            active++;
        } else if (ctx->retired) {
            retired++;
        } else {
            free_slots++;
        }
    }

    ESP_LOGW(TAG,
             "%s: router slots active=%d retired=%d free=%d max=%d",
             reason != NULL ? reason : "router slots",
             active, retired, free_slots, (int)s_router.max_ports);

    for (int i = 0; i < (int)s_router.max_ports; i++) {
        const ax25_router_port_ctx_t *ctx = &s_router.slots[i];
        char dest[16] = "-";
        const char *mode = "free";
        UBaseType_t queued = 0;

        if (ctx->tx_queue != NULL) {
            queued = uxQueueMessagesWaiting(ctx->tx_queue);
        }

        if (ctx->port != NULL) {
            port_dest_to_str(ctx->port, dest, sizeof(dest));
            mode = port_mode_str(ctx->port->mode);
        } else if (ctx->retired) {
            snprintf(dest, sizeof(dest), "<retired>");
            mode = "retired";
        }

        ESP_LOGW(TAG,
                 "slot[%d]: mode=%s dest=%s port=%p queue=%p queued=%u task=%p running=%d stopped=%d retired=%d",
                 i,
                 mode,
                 dest,
                 (void *)ctx->port,
                 (void *)ctx->tx_queue,
                 (unsigned)queued,
                 (void *)ctx->tx_task,
                 ctx->running,
                 ctx->stopped,
                 ctx->retired);
    }
}
#endif

static ax25_router_port_ctx_t *alloc_port_slot(void)
{
    for (int i = 0; i < (int)s_router.max_ports; i++) {
        if (s_router.slots[i].port == NULL && !s_router.slots[i].retired) {
            return &s_router.slots[i];
        }
    }

    for (int i = 0; i < (int)s_router.max_ports; i++) {
        if (s_router.slots[i].port == NULL &&
            s_router.slots[i].retired &&
            s_router.slots[i].tx_task == NULL &&
            !s_router.slots[i].running) {
            if (s_router.slots[i].tx_queue != NULL) {
                vQueueDelete(s_router.slots[i].tx_queue);
                s_router.slots[i].tx_queue = NULL;
            }
            router_free_queue_storage(&s_router.slots[i]);
#if AX25_ROUTER_LOG_PORT_STATUS
            ESP_LOGI(TAG, "port status: reclaiming retired slot=%d", i);
#endif
            s_router.slots[i].retired = false;
            s_router.slots[i].stopped = false;
            return &s_router.slots[i];
        }
    }
    return NULL;
}

static ax25_router_port_ctx_t *find_port_slot(const ax25_router_port_t *port)
{
    for (int i = 0; i < (int)s_router.max_ports; i++) {
        if (s_router.slots[i].port == port) {
            return &s_router.slots[i];
        }
    }
    return NULL;
}

/**
 * @brief Return the next un-repeated digipeater address in the frame, or NULL
 *        if there are no digipeaters remaining to be repeated.
 */
static const ax25_address_t *next_digipeater(const ax25_frame_t *frame)
{
    for (uint8_t i = 0; i < frame->num_digipeaters; i++) {
        if (!frame->digipeaters[i].has_been_repeated) {
            return &frame->digipeaters[i];
        }
    }
    return NULL;
}

static int next_digipeater_index(const ax25_frame_t *frame)
{
    for (uint8_t i = 0; i < frame->num_digipeaters; i++) {
        if (!frame->digipeaters[i].has_been_repeated) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Compute the wire size of a frame in bytes.
 */
static uint32_t frame_byte_count(const ax25_frame_t *frame)
{
    uint32_t addr_bytes = (uint32_t)AX25_ADDRESS_LEN * (2u + frame->num_digipeaters);
    uint32_t overhead = 1u; /* control */
    if (frame->type == AX25_FRAME_I || frame->type == AX25_FRAME_UI) {
        overhead += 1u; /* PID byte */
    }
    return addr_bytes + overhead + (uint32_t)frame->payload_len;
}

/**
 * @brief Return true if a PORT_DYNAMIC port has been bound to a destination.
 */
static bool port_dynamic_is_bound(const ax25_router_port_t *port)
{
    return port->destination.callsign[0] != '\0';
}

__attribute__((unused)) static const char *port_mode_str(ax25_router_port_mode_t mode)
{
    switch (mode) {
        case AX25_PORT_STATIC: return "static";
        case AX25_PORT_DEFAULT: return "default";
        case AX25_PORT_PROMISCUOUS: return "promisc";
        case AX25_PORT_DIGIPEATER: return "digipeat";
        case AX25_PORT_DYNAMIC: return "dynamic";
        default: return "unknown";
    }
}

__attribute__((unused)) static void port_dest_to_str(const ax25_router_port_t *port, char *buf, size_t buf_len)
{
    if (port == NULL || buf == NULL || buf_len == 0) {
        return;
    }

    if (port->mode == AX25_PORT_PROMISCUOUS || port->mode == AX25_PORT_DEFAULT) {
        snprintf(buf, buf_len, "-");
        return;
    }

    if (port->mode == AX25_PORT_DYNAMIC && !port_dynamic_is_bound(port)) {
        snprintf(buf, buf_len, "<unbound>");
        return;
    }

    ax25_address_to_string(&port->destination, buf, buf_len);
}

static void router_port_tx_task(void *arg)
{
    ax25_router_port_ctx_t *ctx = (ax25_router_port_ctx_t *)arg;
    ESP_LOGI(TAG,
             "port_task: start slot=%ld ctx=%p queue=%p port=%p running=%d",
             (long)(ctx - &s_router.slots[0]),
             (void *)ctx,
             (void *)ctx->tx_queue,
             (void *)ctx->port,
             ctx->running);

    ax25_frame_t *frame = (ax25_frame_t *)heap_caps_calloc(
        1,
        sizeof(*frame),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    bool warned_low_stack = false;

    if (frame == NULL) {
        ESP_LOGE(TAG, "port_task: failed to allocate frame buffer for slot=%ld",
                 (long)(ctx - &s_router.slots[0]));
        xSemaphoreTake(s_router.mutex, portMAX_DELAY);
        ctx->running = false;
        ctx->stopped = true;
        ctx->tx_task = NULL;
        xSemaphoreGive(s_router.mutex);
        vTaskDelete(NULL);
        return;
    }

    while (ctx->running) {
        if (!warned_low_stack) {
            UBaseType_t stack_words = uxTaskGetStackHighWaterMark(NULL);
            if (stack_words > 0 && stack_words < 128) {
                ESP_LOGW(TAG,
                         "Low stack watermark in router port task slot=%ld: %u words",
                         (long)(ctx - &s_router.slots[0]),
                         (unsigned)stack_words);
                warned_low_stack = true;
            }
        }

        if (xQueueReceive(ctx->tx_queue, frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            ax25_router_port_t *port = ctx->port;
            if (port == NULL) {
                continue;
            }
            port->on_tx_frame(frame, port->user_data);
            port->frames_sent++;
            port->bytes_sent += frame_byte_count(frame);
        }
    }

    free(frame);

    xSemaphoreTake(s_router.mutex, portMAX_DELAY);
    ctx->stopped = true;
    ctx->tx_task = NULL;
    xSemaphoreGive(s_router.mutex);
    vTaskDelete(NULL);
}

static void port_ctx_teardown(ax25_router_port_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    TaskHandle_t self = xTaskGetCurrentTaskHandle();

#if AX25_ROUTER_LOG_PORT_STATUS
    {
        char dest[16] = "-";

        if (ctx->port != NULL) {
            port_dest_to_str(ctx->port, dest, sizeof(dest));
            ESP_LOGI(TAG,
                     "port status: teardown requested slot=%ld mode=%s dest=%s port=%p queue=%p task=%p",
                     (long)(ctx - &s_router.slots[0]),
                     port_mode_str(ctx->port->mode),
                     dest,
                     (void *)ctx->port,
                     (void *)ctx->tx_queue,
                     (void *)ctx->tx_task);
        } else {
            ESP_LOGI(TAG,
                     "port status: teardown requested slot=%ld with no active port queue=%p task=%p retired=%d",
                     (long)(ctx - &s_router.slots[0]),
                     (void *)ctx->tx_queue,
                     (void *)ctx->tx_task,
                     ctx->retired);
        }
    }
#endif

    ctx->running = false;

    /* Self-teardown happens when a callback removes its own dynamic port.
     * Do not wait for this task to stop here; let router_port_tx_task() exit naturally
     * and perform final queue/storage cleanup on its way out. */
    if (ctx->tx_task == self) {
        ctx->port = NULL;
        ctx->retired = true;
#if AX25_ROUTER_LOG_PORT_STATUS
        ESP_LOGI(TAG,
                 "port status: self-teardown deferred slot=%ld task=%p",
                 (long)(ctx - &s_router.slots[0]),
                 (void *)ctx->tx_task);
#endif
        return;
    }

    if (ctx->tx_task != NULL) {
        xTaskAbortDelay(ctx->tx_task);
    }

    for (int i = 0; i < 250 && ctx->tx_task != NULL && !ctx->stopped; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (ctx->tx_task != NULL) {
        ESP_LOGW(TAG, "teardown: task did not stop in time, leaving slot retired (%p)",
                 (void *)ctx->tx_task);
        /* Detach the port pointer immediately so future registrations of the
         * same port object are not rejected as duplicates while this slot is
         * retired waiting for task reclamation. */
        ctx->port = NULL;
        ctx->retired = true;
#if AX25_ROUTER_LOG_PORT_STATUS
        ESP_LOGW(TAG,
                 "port status: slot=%ld transitioned to retired while waiting for task cleanup",
                 (long)(ctx - &s_router.slots[0]));
#endif
        return;
    }

    if (ctx->tx_queue) {
        vQueueDelete(ctx->tx_queue);
        ctx->tx_queue = NULL;
    }

    router_free_queue_storage(ctx);

    /* Clear the task handle now that the task has exited and cleanup is complete.
    * This is done here (not in router_port_tx_task) to ensure the slot cannot be reused
     * until the task has fully terminated. */
    ctx->tx_task = NULL;
    ctx->stopped = false;

    /* Clean teardown complete; keep the slot retired so the allocator
     * prefers never-used/free slots before reusing one that was just
     * torn down in a tight register/remove/register sequence. */
    ctx->port = NULL;
    ctx->retired = true;
#if AX25_ROUTER_LOG_PORT_STATUS
    ESP_LOGI(TAG,
             "port status: slot=%ld transitioned to retired after clean teardown",
             (long)(ctx - &s_router.slots[0]));
#endif
}

/**
 * @brief Return true if @p port is eligible to receive @p frame in the first
 *        pass (promiscuous, static, digipeater, and bound-dynamic ports).
 */
static bool port_is_eligible(const ax25_router_port_t *port,
                             const ax25_frame_t *frame,
                             const ax25_address_t *next_digi)
{
    switch (port->mode) {
    case AX25_PORT_PROMISCUOUS:
        return true;

    case AX25_PORT_DEFAULT:
        return false;

    case AX25_PORT_DIGIPEATER:
        return (next_digi != NULL) && ax25_address_equals(&port->destination, next_digi);

    case AX25_PORT_STATIC:
        return (next_digi == NULL) && ax25_address_equals(&port->destination, &frame->destination);

    case AX25_PORT_DYNAMIC:
        if (!port_dynamic_is_bound(port)) {
            return false;
        }
        return (next_digi == NULL) && ax25_address_equals(&port->destination, &frame->destination);
    }

    return false;
}

/** Enqueue @p frame to @p ctx, incrementing frames_dropped on overflow. */
static void enqueue_tx_frame(ax25_router_port_ctx_t *ctx, const ax25_frame_t *frame)
{
    if (xQueueSend(ctx->tx_queue, frame, 0) != pdTRUE) {
        ctx->port->frames_dropped++;
        ESP_LOGW(TAG, "send: port queue full, dropping frame");
    }
}

static void enqueue_tx_frame_for_port(ax25_router_port_ctx_t *ctx,
                                      const ax25_frame_t *frame,
                                      int next_digi_idx)
{
    if (ctx->port != NULL &&
        ctx->port->mode == AX25_PORT_DIGIPEATER &&
        next_digi_idx >= 0 &&
        next_digi_idx < frame->num_digipeaters) {
        ax25_frame_t forwarded = *frame;
        forwarded.digipeaters[next_digi_idx].has_been_repeated = true;
        enqueue_tx_frame(ctx, &forwarded);
        return;
    }

    enqueue_tx_frame(ctx, frame);
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

esp_err_t ax25_router_init(void)
{
    if (s_router.initialized) {
        return ESP_OK;
    }

    memset(&s_router, 0, sizeof(s_router));

    s_router.max_ports   = (size_t)ax25_cfg_get_int_global("ax25.router.max_ports",
                                                            ROUTER_MAX_PORTS_DEFAULT);
    s_router.queue_depth = (size_t)ax25_cfg_get_int_global("ax25.router.queue_depth",
                                                            ROUTER_QUEUE_DEPTH_DEFAULT);
    if (s_router.max_ports   < 1) { s_router.max_ports   = 1; }
    if (s_router.queue_depth < 1) { s_router.queue_depth = 1; }

    s_router.slots = calloc(s_router.max_ports, sizeof(ax25_router_port_ctx_t));
    if (s_router.slots == NULL) {
        ESP_LOGE(TAG, "init: failed to allocate port slots");
        return ESP_ERR_NO_MEM;
    }

    /* Allocate all queue backing storage as a single slab to avoid per-port
     * alloc/free churn when ports are registered and removed at runtime.   */
    bool q_psram = false;
    s_router.queue_storage = (uint8_t *)router_alloc_prefer_psram(
        s_router.max_ports * s_router.queue_depth * sizeof(ax25_frame_t), &q_psram);
    if (s_router.queue_storage == NULL) {
        ESP_LOGE(TAG, "init: failed to allocate queue storage");
        free(s_router.slots);
        s_router.slots = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_router.queue_storage_psram = q_psram;
    for (size_t i = 0; i < s_router.max_ports; i++) {
        s_router.slots[i].queue_storage_ptr =
            s_router.queue_storage + i * s_router.queue_depth * sizeof(ax25_frame_t);
        s_router.slots[i].queue_storage_psram = q_psram;
    }

    s_router.mutex = xSemaphoreCreateMutexStatic(&s_router.mutex_buf);
    if (s_router.mutex == NULL) {
        free(s_router.queue_storage);
        s_router.queue_storage = NULL;
        free(s_router.slots);
        s_router.slots = NULL;
        ESP_LOGE(TAG, "failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    s_router.initialized = true;
    return ESP_OK;
}

void ax25_router_deinit(void)
{
    if (!s_router.initialized) {
        return;
    }

    /* Tear down all active port contexts */
    for (int i = 0; i < (int)s_router.max_ports; i++) {
        if (s_router.slots[i].port != NULL ||
            s_router.slots[i].tx_task != NULL ||
            s_router.slots[i].tx_queue != NULL) {
            port_ctx_teardown(&s_router.slots[i]);
        }
    }

    /* Give the idle task time to finish reclaiming any deleted static tasks
     * before we zero their backing storage for the next init cycle. */
    vTaskDelay(pdMS_TO_TICKS(10));

    s_router.initialized = false;
    vSemaphoreDelete(s_router.mutex);
    free(s_router.slots);
    free(s_router.queue_storage);
    memset(&s_router, 0, sizeof(s_router));   /* zeros slots, queue_storage, max_ports, queue_depth */
}

esp_err_t ax25_router_register_port(ax25_router_port_t *port)
{
    char requested_dest[16] = "-";

    if (port == NULL) {
        ESP_LOGE(TAG, "register_port: port is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (port->on_tx_frame == NULL) {
        ESP_LOGE(TAG, "register_port: port callback is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_router.initialized) {
        ESP_LOGE(TAG, "register_port: router not initialised");
        return ESP_ERR_INVALID_STATE;
    }

    port_dest_to_str(port, requested_dest, sizeof(requested_dest));

#if AX25_ROUTER_LOG_PORT_STATUS
    ESP_LOGI(TAG, "register_port: request mode=%s dest=%s port=%p",
             port_mode_str(port->mode), requested_dest, (void *)port);
#endif

    xSemaphoreTake(s_router.mutex, portMAX_DELAY);

    for (int i = 0; i < (int)s_router.max_ports; i++) {
        ax25_router_port_ctx_t *existing = &s_router.slots[i];
        if (existing->port == NULL) {
            continue;
        }
        ax25_router_port_t *ep = existing->port;

        /* Reject duplicate pointer registration. */
        if (ep == port) {
            xSemaphoreGive(s_router.mutex);
            ESP_LOGE(TAG, "register_port: port already registered mode=%s dest=%s port=%p",
                     port_mode_str(port->mode), requested_dest, (void *)port);
            return ESP_ERR_INVALID_STATE;
        }

        /* At most one default port. */
        if (port->mode == AX25_PORT_DEFAULT && ep->mode == AX25_PORT_DEFAULT) {
            char existing_dest[16] = "-";
            port_dest_to_str(ep, existing_dest, sizeof(existing_dest));
            xSemaphoreGive(s_router.mutex);
            ESP_LOGE(TAG,
                     "register_port: default port already registered existing_port=%p existing_dest=%s requested_port=%p",
                     (void *)ep, existing_dest, (void *)port);
            return ESP_ERR_INVALID_STATE;
        }

        /* No duplicate destinations among static and digipeater ports. */
        if ((port->mode == AX25_PORT_STATIC || port->mode == AX25_PORT_DIGIPEATER) &&
            (ep->mode   == AX25_PORT_STATIC || ep->mode   == AX25_PORT_DIGIPEATER) &&
            ax25_address_equals(&port->destination, &ep->destination)) {
            char existing_dest[16] = "-";
            port_dest_to_str(ep, existing_dest, sizeof(existing_dest));
            xSemaphoreGive(s_router.mutex);
            ESP_LOGE(TAG,
                     "register_port: duplicate destination address dest=%s existing_port=%p requested_port=%p",
                     existing_dest, (void *)ep, (void *)port);
            return ESP_ERR_INVALID_STATE;
        }
    }

    ax25_router_port_ctx_t *ctx = alloc_port_slot();
    if (ctx == NULL) {
#if AX25_ROUTER_LOG_PORT_STATUS
        log_router_slot_inventory_locked("register_port: no free port slots");
#endif
        xSemaphoreGive(s_router.mutex);
        ESP_LOGE(TAG, "register_port: no free port slots for mode=%s dest=%s port=%p (max %d)",
         port_mode_str(port->mode), requested_dest, (void *)port, (int)s_router.max_ports);
        return ESP_ERR_NO_MEM;
    }

    /* Preserve the pre-allocated storage pointer across the slot reset. */
    uint8_t *saved_storage = ctx->queue_storage_ptr;
    memset(ctx, 0, sizeof(*ctx));
    ctx->queue_storage_ptr   = saved_storage;
    ctx->queue_storage_psram = s_router.queue_storage_psram;
    ctx->port = port;

    /* Zero queue contents for clean re-use. */
    if (router_alloc_queue_storage(ctx) != ESP_OK) {
        ctx->port = NULL;
        xSemaphoreGive(s_router.mutex);
        return ESP_ERR_NO_MEM;
    }

    ctx->tx_queue = xQueueCreateStatic((UBaseType_t)s_router.queue_depth,
                                       sizeof(ax25_frame_t),
                                       ctx->queue_storage_ptr,
                                       &ctx->queue_buf);
    if (ctx->tx_queue == NULL) {
        ctx->port = NULL;
        xSemaphoreGive(s_router.mutex);
        return ESP_ERR_NO_MEM;
    }

    ctx->running = true;
    if (xTaskCreate(router_port_tx_task,
                    "ax25_r_port",
                    ROUTER_PORT_TASK_STACK,
                    ctx,
                    ROUTER_PORT_TASK_PRIORITY,
                    &ctx->tx_task) != pdPASS) {
        ctx->tx_task = NULL;
        vQueueDelete(ctx->tx_queue);
        ctx->tx_queue = NULL;
        router_free_queue_storage(ctx);
        ctx->port = NULL;
        xSemaphoreGive(s_router.mutex);
        return ESP_ERR_NO_MEM;
    }

    ctx->port = port;

    /* Zero the counters so stats always reflect activity since registration. */
    port->frames_sent    = 0;
    port->bytes_sent     = 0;
    port->frames_dropped = 0;

#if AX25_ROUTER_LOG_PORT_STATUS
    ESP_LOGI(TAG, "register_port: assigned slot=%ld ctx=%p mode=%s dest=%s port=%p queue=%p task=%p storage=%s",
             (long)(ctx - &s_router.slots[0]),
             (void *)ctx,
             port_mode_str(port->mode), requested_dest, (void *)port,
             (void *)ctx->tx_queue, (void *)ctx->tx_task,
             ctx->queue_storage_psram ? "PSRAM" : "SRAM");
#endif

    xSemaphoreGive(s_router.mutex);
    return ESP_OK;
}

esp_err_t ax25_router_remove_port(ax25_router_port_t *port)
{
    char dest[16] = "-";

    if (port == NULL) {
        ESP_LOGE(TAG, "remove_port: port is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_router.initialized) {
        ESP_LOGE(TAG, "remove_port: router not initialised");
        return ESP_ERR_INVALID_STATE;
    }

    port_dest_to_str(port, dest, sizeof(dest));

#if AX25_ROUTER_LOG_PORT_STATUS
    ESP_LOGI(TAG, "remove_port: request mode=%s dest=%s port=%p",
             port_mode_str(port->mode), dest, (void *)port);
#endif

    xSemaphoreTake(s_router.mutex, portMAX_DELAY);

    ax25_router_port_ctx_t *ctx = find_port_slot(port);

#if AX25_ROUTER_LOG_PORT_STATUS
    if (ctx != NULL) {
        ESP_LOGI(TAG, "remove_port: found slot=%ld queue=%p task=%p running=%d stopped=%d retired=%d",
                 (long)(ctx - &s_router.slots[0]),
                 (void *)ctx->tx_queue,
                 (void *)ctx->tx_task,
                 ctx->running,
                 ctx->stopped,
                 ctx->retired);
    } else {
        log_router_slot_inventory_locked("remove_port: port not found");
    }
#endif

    xSemaphoreGive(s_router.mutex);

    if (ctx == NULL) {
        ESP_LOGE(TAG, "remove_port: port not found");
        return ESP_ERR_NOT_FOUND;
    }

    port_ctx_teardown(ctx);

    return ESP_OK;
}

void ax25_router_log_port_summary(const char *reason)
{
#if AX25_ROUTER_LOG_PORT_STATUS
    int active = 0;
    int retired = 0;
    int free_slots = 0;

    if (!s_router.initialized) {
        ESP_LOGW(TAG, "%s: router not initialised",
                 reason != NULL ? reason : "router summary");
        return;
    }

    xSemaphoreTake(s_router.mutex, portMAX_DELAY);

    for (int i = 0; i < (int)s_router.max_ports; i++) {
        const ax25_router_port_ctx_t *ctx = &s_router.slots[i];

        if (ctx->port != NULL) {
            active++;
        } else if (ctx->retired) {
            retired++;
        } else {
            free_slots++;
        }
    }

    xSemaphoreGive(s_router.mutex);

    ESP_LOGI(TAG, "%s: router slots active=%d retired=%d free=%d max=%d",
             reason != NULL ? reason : "router summary",
             active, retired, free_slots, (int)s_router.max_ports);
#else
    (void)reason;
#endif
}

esp_err_t ax25_router_send(const ax25_frame_t *frame, ax25_router_port_t *source_port)
{
    if (frame == NULL) {
        ESP_LOGE(TAG, "on_tx_frame: frame is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (source_port == NULL) {
        ESP_LOGE(TAG, "on_tx_frame: source_port is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_router.initialized) {
        ESP_LOGE(TAG, "on_tx_frame: router not initialised");
        return ESP_ERR_INVALID_STATE;
    }

    const ax25_address_t *next_digi = next_digipeater(frame);
    int next_digi_idx = next_digipeater_index(frame);

#if AX25_ROUTER_LOG_FRAMES
    {
        char src[16] = {0};
        char dst[16] = {0};
        char src_port_buf[16] = {0};

        ax25_address_to_string(&frame->source, src, sizeof(src));
        ax25_address_to_string(&frame->destination, dst, sizeof(dst));
        port_dest_to_str(source_port, src_port_buf, sizeof(src_port_buf));
        ESP_LOGI(TAG, "route: from=%s to=%s type=%s src_port=%s(%s)",
                 src, dst, ax25_frame_type_str(frame->type),
                 port_mode_str(source_port->mode), src_port_buf);
    }
#endif

    xSemaphoreTake(s_router.mutex, portMAX_DELAY);

    /* Bind unbound dynamic source port on first send */
    if (source_port != NULL &&
        source_port->mode == AX25_PORT_DYNAMIC &&
        !port_dynamic_is_bound(source_port)) {
        ax25_address_copy(&source_port->destination, &frame->source);
#if AX25_ROUTER_LOG_PORT_STATUS
        {
            char bound_dest[16] = {0};
            port_dest_to_str(source_port, bound_dest, sizeof(bound_dest));
            ESP_LOGI(TAG,
                     "port status: dynamic port bound dest=%s port=%p source_frame=%s",
                     bound_dest,
                     (void *)source_port,
                     bound_dest);
        }
#endif
    }

    bool normal_port_matched = false;

    /* First pass — promiscuous, static, digipeater, and bound-dynamic ports. */
    for (int i = 0; i < (int)s_router.max_ports; i++) {
        ax25_router_port_ctx_t *ctx  = &s_router.slots[i];
        ax25_router_port_t     *port = ctx->port;
        if (port == NULL || port == source_port) {
            continue;
        }

        if (!port_is_eligible(port, frame, next_digi)) {
            continue;
        }

        if (port->mode == AX25_PORT_STATIC    ||
            port->mode == AX25_PORT_DIGIPEATER ||
            (port->mode == AX25_PORT_DYNAMIC && port_dynamic_is_bound(port))) {
            normal_port_matched = true;
        }

#if AX25_ROUTER_LOG_FRAMES
        {
            char dest[16] = {0};
            port_dest_to_str(port, dest, sizeof(dest));
            ESP_LOGI(TAG, "route: queue -> mode=%s dest=%s port=%p",
                     port_mode_str(port->mode), dest, (void *)port);
        }
#endif
        enqueue_tx_frame_for_port(ctx, frame, next_digi_idx);
    }

    /* Second pass — default ports fire when no addressed port matched. */
    if (!normal_port_matched) {
        for (int i = 0; i < (int)s_router.max_ports; i++) {
            ax25_router_port_ctx_t *ctx  = &s_router.slots[i];
            ax25_router_port_t     *port = ctx->port;
            if (port == NULL || port == source_port) {
                continue;
            }

            if (port->mode == AX25_PORT_DEFAULT) {
#if AX25_ROUTER_LOG_FRAMES
                {
                    char dest[16] = {0};
                    port_dest_to_str(port, dest, sizeof(dest));
                    ESP_LOGI(TAG, "route: no addressed match, queue default -> dest=%s port=%p",
                             dest, (void *)port);
                }
#endif
                enqueue_tx_frame_for_port(ctx, frame, next_digi_idx);
            }
        }
    }

#if AX25_ROUTER_LOG_FRAMES
    if (!normal_port_matched) {
        ESP_LOGI(TAG, "route: no addressed port matched");
    }
#endif

    xSemaphoreGive(s_router.mutex);
    return ESP_OK;
}
