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
 * @file ax25_log_tcp_server.c
 * @brief ESP log fanout: console sink + TCP log server
 *
 * Architecture
 * ------------
 * A global sink table (s_sinks[]) holds up to
 * (1 + net.log.max_conns) entries at runtime:
 *   slot 0  — always the console (uart_vprintf)
 *   slots 1..N — one per connected TCP client
 *
 * Each sink owns a queue of `char *` pointers.  log_vprintf() formats the
 * log line into a heap string and posts the pointer (non-blocking) to every
 * active sink queue.  If a queue is full the pointer is not posted to that
 * sink and that client/console copy is dropped.  The heap string is freed
 * by the last sink that dequeues it — protected by a refcount.
 *
 * Spinlock (portMUX_TYPE) protects the sink table and the log_vprintf
 * refcount so it is safe from any context (timer, ISR, normal task).
 *
 * Each TCP client gets a lightweight sender task that drains its queue and
 * write()s the string to the socket.  On disconnect the task drains the
 * queue, frees any remaining strings, and removes the sink before exiting.
 */

#include "ax25_log_tcp_server.h"

#include <errno.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_log_write.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "ax25_config.h"
#include "ax25_phy_tcp_server.h"

#define LOG_SINK_CONSOLE_IDX 0

/* Default queue depth for each sink */
#define LOG_SINK_QUEUE_DEFAULT_DEPTH 32

/* -------------------------------------------------------------------------
 * Per-item reference count helper
 *
 * Each heap string is reference-counted so multiple sinks can share the
 * same allocation and the last consumer frees it.
 * ---------------------------------------------------------------------- */

typedef struct {
    char   *str;      /* heap-allocated formatted log line (includes '\n') */
    int     refcnt;   /* number of sinks that still need to consume this */
} log_item_t;

/* Items are passed through queues as (log_item_t*) pointers so that the
 * queue element size is always sizeof(void*). */

/* -------------------------------------------------------------------------
 * Sink table
 * ---------------------------------------------------------------------- */

typedef struct {
    bool          active;      /* slot in use?                         */
    QueueHandle_t queue;       /* queue of (log_item_t*) pointers      */
    TaskHandle_t  task;        /* sender task handle (NULL for console) */
    ax25_phy_tcp_server_conn_t *tcp_conn; /* owning TCP connection */
} log_sink_t;

/* Protected by s_mux */
static portMUX_TYPE  s_mux     = portMUX_INITIALIZER_UNLOCKED;
static log_sink_t   *s_sinks   = NULL;
static size_t        s_sink_total = 0;
static size_t        s_active_sink_count = 0;

/* -------------------------------------------------------------------------
 * log_item_t pool — eliminates per-message malloc(sizeof(log_item_t)).
 * Protected by s_mux.  Falls back to heap when exhausted.
 * ---------------------------------------------------------------------- */
#define LOG_ITEM_POOL_SIZE  64
static log_item_t s_item_pool[LOG_ITEM_POOL_SIZE];
static uint8_t    s_item_pool_stack[LOG_ITEM_POOL_SIZE];
static int        s_item_pool_top;   /* 0 = empty; LOG_ITEM_POOL_SIZE = full */

/* -------------------------------------------------------------------------
 * Back-pointer to the single server context (set on init, cleared on deinit)
 * ---------------------------------------------------------------------- */
static ax25_log_tcp_server_t *s_server = NULL;   /* set by init */

/* -------------------------------------------------------------------------
 * Spinlock-safe helpers
 * ---------------------------------------------------------------------- */

/* Find a free (non-console) sink slot.  Must be called with s_mux held. */
static int find_free_sink_locked(void)
{
    for (size_t i = 1; i < s_sink_total; i++) {
        if (!s_sinks[i].active) {
            return (int)i;
        }
    }
    return -1;
}

static void set_sink_active_locked(int idx, bool active)
{
    if (idx < 0 || (size_t)idx >= s_sink_total) {
        return;
    }

    if (s_sinks[idx].active == active) {
        return;
    }

    s_sinks[idx].active = active;
    if (active) {
        s_active_sink_count++;
    } else if (s_active_sink_count > 0) {
        s_active_sink_count--;
    }
}

/* Release a log_item_t.  Decrements refcnt; frees if it hits zero.
 * May be called with or without the spinlock — refcnt is decremented inside
 * the lock to keep atomicity.                                             */
static void item_release(log_item_t *item)
{
    if (item == NULL) {
        return;
    }

    bool do_free = false;
    portENTER_CRITICAL(&s_mux);
    item->refcnt--;
    if (item->refcnt <= 0) {
        do_free = true;
    }
    portEXIT_CRITICAL(&s_mux);

    if (do_free) {
        free(item->str);
        /* Return wrapper to pool if it came from there, otherwise heap-free. */
        if (item >= s_item_pool && item < s_item_pool + LOG_ITEM_POOL_SIZE) {
            portENTER_CRITICAL(&s_mux);
            if (s_item_pool_top < LOG_ITEM_POOL_SIZE) {
                s_item_pool_stack[s_item_pool_top++] = (uint8_t)(item - s_item_pool);
            }
            portEXIT_CRITICAL(&s_mux);
        } else {
            free(item);
        }
    }
}

/* -------------------------------------------------------------------------
 * log_vprintf — installed via esp_log_set_vprintf()
 *
 * Called from any context (including ISRs).  We must not block.
 * ---------------------------------------------------------------------- */
static int log_vprintf(const char *fmt, va_list args)
{
    /* Format the string onto the heap. vasprintf is not ISR-safe on all
     * platforms; however esp_log_set_vprintf is documented to be called from
     * task context on ESP-IDF, so heap allocation is safe here.            */
    char *str = NULL;
    int n = vasprintf(&str, fmt, args);
    if (n < 0 || str == NULL) {
        /* malloc failed — bail out silently */
        return 0;
    }

    /* Allocate item wrapper — try the pool first to avoid heap churn. */
    log_item_t *item = NULL;
    portENTER_CRITICAL(&s_mux);
    if (s_item_pool_top > 0) {
        item = &s_item_pool[s_item_pool_stack[--s_item_pool_top]];
    }
    portEXIT_CRITICAL(&s_mux);
    if (item == NULL) {
        item = (log_item_t *)malloc(sizeof(log_item_t));
        if (item == NULL) {
            free(str);
            return n;
        }
    }
    item->str = str;
    item->refcnt = 0;

    portENTER_CRITICAL(&s_mux);

    int refs = (int)s_active_sink_count;

    if (refs == 0) {
        portEXIT_CRITICAL(&s_mux);
        free(str);
        free(item);
        return n;
    }

    item->refcnt = refs;

    /* Post to each active queue — non-blocking.  Drop on full queue. */
    for (size_t i = 0; i < s_sink_total; i++) {
        if (!s_sinks[i].active) {
            continue;
        }
        /* xQueueSendFromISR is safe here from normal task context too */
        BaseType_t sent = xQueueSendFromISR(s_sinks[i].queue, &item, NULL);
        if (sent != pdTRUE) {
            /* Queue full — this sink won't handle this item */
            item->refcnt--;
        }
    }

    /* If every queue was full refcnt may have reached 0 already */
    bool do_free = (item->refcnt <= 0);
    portEXIT_CRITICAL(&s_mux);

    if (do_free) {
        free(str);
        if (item >= s_item_pool && item < s_item_pool + LOG_ITEM_POOL_SIZE) {
            portENTER_CRITICAL(&s_mux);
            if (s_item_pool_top < LOG_ITEM_POOL_SIZE) {
                s_item_pool_stack[s_item_pool_top++] = (uint8_t)(item - s_item_pool);
            }
            portEXIT_CRITICAL(&s_mux);
        } else {
            free(item);
        }
    }

    return n;
}

/* -------------------------------------------------------------------------
 * Console sender task — runs forever; slot 0
 * ---------------------------------------------------------------------- */
static void console_task(void *arg)
{
    (void)arg;
    log_item_t *item;
    for (;;) {
        if (xQueueReceive(s_sinks[LOG_SINK_CONSOLE_IDX].queue,
                          &item, portMAX_DELAY) == pdTRUE) {
            /* Use the original vprintf to write to UART without recursion. */
            fputs(item->str, stdout);
            item_release(item);
        }
    }
}

/* -------------------------------------------------------------------------
 * Per-TCP-client sender task
 * ---------------------------------------------------------------------- */
static void client_send_task(void *arg)
{
    int idx = (int)(intptr_t)arg;
    QueueHandle_t queue = s_sinks[idx].queue;

    log_item_t *item;
    for (;;) {
        if (xQueueReceive(queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (item == NULL) {
            /* Sentinel: disconnect requested */
            break;
        }

        /* Write to socket — ignore errors; if write fails the TCP task
         * will detect the disconnect and call on_tcp_disconnected.     */
        size_t len = strlen(item->str);
        ax25_phy_tcp_server_conn_t *conn = s_sinks[idx].tcp_conn;
        if (conn != NULL && conn->sock >= 0 && len > 0) {
            send(conn->sock, item->str, len, MSG_NOSIGNAL);
        }
        item_release(item);
    }

    /* Drain any remaining items */
    while (xQueueReceive(queue, &item, 0) == pdTRUE) {
        if (item != NULL) {
            item_release(item);
        }
    }

    vQueueDelete(queue);

    /* Remove sink from table */
    portENTER_CRITICAL(&s_mux);
    set_sink_active_locked(idx, false);
    s_sinks[idx].queue = NULL;
    s_sinks[idx].tcp_conn = NULL;
    s_sinks[idx].task = NULL;
    portEXIT_CRITICAL(&s_mux);

    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * ax25_phy_tcp_server callbacks
 * ---------------------------------------------------------------------- */
static void on_tcp_connected(ax25_phy_tcp_server_conn_t *tcp_conn)
{
    ax25_log_tcp_server_t *srv = s_server;
    if (srv == NULL) {
        return;
    }

    /* Find a free sink slot */
    portENTER_CRITICAL(&s_mux);
    int idx = find_free_sink_locked();
    portEXIT_CRITICAL(&s_mux);

    if (idx < 0) {
        /* This should not happen: ax25_phy_tcp_server already limits clients */
        fputs("ax25_log_tcp_server: no free sink slot\n", stdout);
        return;
    }

    /* Create the queue (static allocation at this layer would require
     * knowing the depth at compile time; dynamic allocation is fine because
     * this path runs from a normal task.)                                  */
    QueueHandle_t q = xQueueCreate(srv->queue_depth, sizeof(log_item_t *));
    if (q == NULL) {
        fputs("ax25_log_tcp_server: queue alloc failed\n", stdout);
        return;
    }

    /* Register the sink BEFORE creating the task so no log lines are missed */
    portENTER_CRITICAL(&s_mux);
    s_sinks[idx].queue  = q;
    s_sinks[idx].tcp_conn = tcp_conn;
    set_sink_active_locked(idx, true);
    s_sinks[idx].task   = NULL;   /* filled in below */
    portEXIT_CRITICAL(&s_mux);

    TaskHandle_t task;
    char name[20];
    snprintf(name, sizeof(name), "log_tcp_%d", idx);
    if (xTaskCreate(client_send_task, name,
                    srv->client_task_stack_size,
                    (void *)(intptr_t)idx,
                    srv->client_task_priority,
                    &task) != pdPASS) {
        portENTER_CRITICAL(&s_mux);
        s_sinks[idx].queue = NULL;
        s_sinks[idx].tcp_conn = NULL;
        set_sink_active_locked(idx, false);
        portEXIT_CRITICAL(&s_mux);
        vQueueDelete(q);
        fputs("ax25_log_tcp_server: task create failed\n", stdout);
        return;
    }

    portENTER_CRITICAL(&s_mux);
    s_sinks[idx].task = task;
    portEXIT_CRITICAL(&s_mux);

    ax25_phy_tcp_server_conn_set_user_data(tcp_conn, (void *)(intptr_t)idx);

    /* Direct printf here (not ESP_LOGI) to avoid recursive log_vprintf call
     * from ISR context; the log macro is fine from a normal task but a
     * plain fputs is safer and avoids a redundant fanout of this banner.   */
    fputs("ax25_log_tcp_server: client connected\n", stdout);
}

static void on_tcp_disconnected(ax25_phy_tcp_server_conn_t *tcp_conn)
{
    int idx = (int)(intptr_t)ax25_phy_tcp_server_conn_get_user_data(tcp_conn);
    if (idx < 1 || (size_t)idx >= s_sink_total) {
        return;
    }

    /* Mark inactive so log_vprintf stops posting here immediately */
    portENTER_CRITICAL(&s_mux);
    set_sink_active_locked(idx, false);
    portEXIT_CRITICAL(&s_mux);

    /* Send NULL sentinel to unblock the sender task */
    log_item_t *sentinel = NULL;
    xQueueSend(s_sinks[idx].queue, &sentinel, portMAX_DELAY);

    /* The sender task will drain, free ctx, and call vTaskDelete itself */
    fputs("ax25_log_tcp_server: client disconnected\n", stdout);
}

static void on_tcp_data(ax25_phy_tcp_server_conn_t *tcp_conn,
                        const uint8_t *data,
                        size_t len)
{
    /* All input from remote clients is discarded */
    (void)tcp_conn;
    (void)data;
    (void)len;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t ax25_log_tcp_server_init(ax25_log_tcp_server_t *ctx)
{
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_server != NULL) {
        return ESP_ERR_INVALID_STATE;   /* already initialised */
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->port = (uint16_t)ax25_cfg_get_int_global("net.log.port", 8300);
    ctx->client_task_stack_size = (uint32_t)ax25_cfg_get_int_global("net.log.conn_task_stack", 4096);
    ctx->client_task_priority = (UBaseType_t)ax25_cfg_get_int_global("net.log.conn_task_priority", 5);
    ctx->queue_depth = (uint16_t)ax25_cfg_get_int_global("net.log.queue_depth",
                                                          LOG_SINK_QUEUE_DEFAULT_DEPTH);

    size_t max_clients = (size_t)ax25_cfg_get_int_global("net.log.max_conns", 2);
    if (max_clients < 1) {
        max_clients = 1;
    }
    s_sink_total = 1 + max_clients;
    s_sinks = (log_sink_t *)calloc(s_sink_total, sizeof(log_sink_t));
    if (s_sinks == NULL) {
        s_sink_total = 0;
        s_active_sink_count = 0;
        return ESP_ERR_NO_MEM;
    }

    /* Initialise sink table */
    memset(s_sinks, 0, s_sink_total * sizeof(log_sink_t));

    /* Initialise the log_item_t pool.  Until this point s_item_pool_top == 0
     * so the pool is safely inactive.                                      */
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < LOG_ITEM_POOL_SIZE; i++) {
        s_item_pool_stack[i] = (uint8_t)i;
    }
    s_item_pool_top = LOG_ITEM_POOL_SIZE;
    portEXIT_CRITICAL(&s_mux);

    /* Console sink — slot 0 */
    QueueHandle_t console_q = xQueueCreate(ctx->queue_depth,
                                           sizeof(log_item_t *));
    if (console_q == NULL) {
        free(s_sinks);
        s_sinks = NULL;
        s_sink_total = 0;
        s_active_sink_count = 0;
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_mux);
    s_sinks[LOG_SINK_CONSOLE_IDX].queue  = console_q;
    set_sink_active_locked(LOG_SINK_CONSOLE_IDX, true);
    portEXIT_CRITICAL(&s_mux);

    TaskHandle_t console_task_handle;
    if (xTaskCreate(console_task, "log_console",
                    ctx->client_task_stack_size,
                    NULL,
                    ctx->client_task_priority,
                    &console_task_handle) != pdPASS) {
        vQueueDelete(console_q);
        portENTER_CRITICAL(&s_mux);
        set_sink_active_locked(LOG_SINK_CONSOLE_IDX, false);
        portEXIT_CRITICAL(&s_mux);
        free(s_sinks);
        s_sinks = NULL;
        s_sink_total = 0;
        s_active_sink_count = 0;
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_mux);
    s_sinks[LOG_SINK_CONSOLE_IDX].task = console_task_handle;
    portEXIT_CRITICAL(&s_mux);

    /* Install our vprintf hook and save the previous one */
    ctx->prev_vprintf = esp_log_set_vprintf(log_vprintf);
    s_server = ctx;

    /* Start TCP listener */
    esp_err_t err = ax25_phy_tcp_server_init("net.log.port",
                                             on_tcp_connected,
                                             on_tcp_disconnected,
                                             on_tcp_data,
                                             ctx,
                                             &ctx->tcp_server);
    if (err != ESP_OK) {
        /* Restore previous vprintf, tear down console sink */
        esp_log_set_vprintf(ctx->prev_vprintf);
        s_server = NULL;

        portENTER_CRITICAL(&s_mux);
        set_sink_active_locked(LOG_SINK_CONSOLE_IDX, false);
        portEXIT_CRITICAL(&s_mux);

        vTaskDelete(console_task_handle);
        vQueueDelete(console_q);
        free(s_sinks);
        s_sinks = NULL;
        s_sink_total = 0;
        s_active_sink_count = 0;
        return err;
    }

    /* Use fputs to avoid recursion during the banner message */
    fputs("ax25_log_tcp_server: log TCP server started\n", stdout);
    return ESP_OK;
}

void ax25_log_tcp_server_deinit(ax25_log_tcp_server_t *ctx)
{
    if (ctx == NULL || s_server == NULL) {
        return;
    }

    /* Restore the previous vprintf hook first so we stop fanout */
    esp_log_set_vprintf(ctx->prev_vprintf);
    s_server = NULL;

    /* Stop TCP server — triggers on_tcp_disconnected for active clients */
    ax25_phy_tcp_server_deinit(&ctx->tcp_server);

    /* Wait for all client sender tasks to exit */
    for (size_t i = 1; i < s_sink_total; i++) {
        for (int j = 0; j < 60; j++) {
            portENTER_CRITICAL(&s_mux);
            bool has_task = (s_sinks[i].task != NULL);
            portEXIT_CRITICAL(&s_mux);
            if (!has_task) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        /* Force-delete any task that didn't exit in time */
        portENTER_CRITICAL(&s_mux);
        TaskHandle_t t = s_sinks[i].task;
        s_sinks[i].task   = NULL;
        s_sinks[i].tcp_conn = NULL;
        set_sink_active_locked((int)i, false);
        portEXIT_CRITICAL(&s_mux);
        if (t != NULL) {
            vTaskDelete(t);
        }
        if (s_sinks[i].queue != NULL) {
            /* Drain remaining items */
            log_item_t *item;
            while (xQueueReceive(s_sinks[i].queue, &item, 0) == pdTRUE) {
                if (item != NULL) {
                    item_release(item);
                }
            }
            vQueueDelete(s_sinks[i].queue);
            s_sinks[i].queue = NULL;
        }
    }

    /* Tear down console sink */
    portENTER_CRITICAL(&s_mux);
    TaskHandle_t console_t = s_sinks[LOG_SINK_CONSOLE_IDX].task;
    s_sinks[LOG_SINK_CONSOLE_IDX].task   = NULL;
    set_sink_active_locked(LOG_SINK_CONSOLE_IDX, false);
    portEXIT_CRITICAL(&s_mux);

    if (console_t != NULL) {
        vTaskDelete(console_t);
    }
    if (s_sinks[LOG_SINK_CONSOLE_IDX].queue != NULL) {
        log_item_t *item;
        while (xQueueReceive(s_sinks[LOG_SINK_CONSOLE_IDX].queue,
                             &item, 0) == pdTRUE) {
            if (item != NULL) {
                item_release(item);
            }
        }
        vQueueDelete(s_sinks[LOG_SINK_CONSOLE_IDX].queue);
        s_sinks[LOG_SINK_CONSOLE_IDX].queue = NULL;
    }

    free(s_sinks);
    s_sinks = NULL;
    s_sink_total = 0;
    s_active_sink_count = 0;

    /* Prevent pool allocations after deinit. Items still in flight that try
     * to return to the pool will hit the bounds check and be silently skipped
     * (the pool entries are static so no leak occurs).                     */
    portENTER_CRITICAL(&s_mux);
    s_item_pool_top = 0;
    portEXIT_CRITICAL(&s_mux);
}
