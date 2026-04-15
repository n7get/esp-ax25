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
 * @file test_ax25_stress.c
 * @brief AX.25 Stack Stress Test (Unity test suite integration)
 *
 * Runs CONFIG_STRESS_TOTAL_CONNECTIONS AX.25 connected-mode sessions through
 * an in-memory loopback, CONFIG_STRESS_SIMULTANEOUS_CONNECTIONS at a time.
 *
 * Each session:
 *   1. Initiates a connection (SABM / UA handshake).
 *   2. Sends a random payload (CONFIG_STRESS_MIN_DATA_SIZE to
 *      CONFIG_STRESS_MAX_DATA_SIZE bytes) as I-frames.
 *   3. Receives the echo from the server side and verifies correctness.
 *   4. Disconnects cleanly.
 *
 * Architecture
 * ============
 * Each slot owns a permanently-allocated pair of ax25_conn_t objects
 * (conn_a = initiator, conn_b = echo server) initialised once per test run
 * and reconnected for every new session.
 *
 *   conn_a.on_tx_frame  ──►  loopback_queue  ──►  ax25_conn_on_frame(conn_b)
 *   conn_b.on_tx_frame  ──►  loopback_queue  ──►  ax25_conn_on_frame(conn_a)
 *
 * One loopback_task drains the queue and calls ax25_conn_on_frame.
 * The supervisor loop runs inline in the TEST_CASE function.
 * Callbacks only set flags / copy data — they never call back into the
 * connection (which would deadlock on the per-connection mutex).
 *
 * Enable via menuconfig: AX.25 Test Configuration -> Enable stress test.
 */

#include "sdkconfig.h"

#if CONFIG_STRESS_TEST_ENABLE

#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_random.h"

#include "ax25_conn.h"
#include "ax25_address.h"
#include "ax25_frame.h"

static const char *TAG = "STRESS";

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------

/** Loopback queue depth — one full window per slot is sufficient; the loopback
 *  task runs at higher priority and drains the queue promptly.  Keep this
 *  small to avoid consuming large heap in the Unity environment. */
#define LOOPBACK_QUEUE_DEPTH    (CONFIG_STRESS_SIMULTANEOUS_CONNECTIONS * 8)

/** T1 retransmit timeout (ms) — shorter than default to speed up recovery. */
#define CONN_T1_MS              2000

/** T3 idle link timeout (ms) — long enough to never fire during a session. */
#define CONN_T3_MS              300000

/** Max retries before giving up on a connection. */
#define CONN_MAX_RETRIES        5

/** Per-slot watchdog (ms).  If a slot does not complete in this time it is
 *  marked as an error so the test can continue. */
#define SLOT_TIMEOUT_MS         30000

/** Supervisor polling period (ms). */
#define SUPERVISOR_PERIOD_MS    10

/** Log a progress line every this many completed connections. */
#define LOG_INTERVAL            50

// ---------------------------------------------------------------------------
// Loopback
// ---------------------------------------------------------------------------

typedef struct {
    ax25_conn_t *dest;
    ax25_frame_t frame;
} loopback_msg_t;

static QueueHandle_t    g_loopback_queue;
static StaticQueue_t    g_loopback_queue_buf;
static uint8_t          g_loopback_queue_storage[LOOPBACK_QUEUE_DEPTH * sizeof(loopback_msg_t)];

static void loopback_task(void *arg)
{
    loopback_msg_t msg;
    while (1) {
        if (xQueueReceive(g_loopback_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ax25_conn_on_frame(msg.dest, &msg.frame);
        }
    }
}

// ---------------------------------------------------------------------------
// Slot
// ---------------------------------------------------------------------------

typedef enum {
    SLOT_IDLE,
    SLOT_CONNECTING,
    SLOT_SENDING,
    SLOT_WAITING_ECHO,
    SLOT_DISCONNECTING,
    SLOT_DONE,
    SLOT_ERROR,
} slot_state_t;

typedef struct {
    int  index;

    ax25_conn_t    conn_a;
    ax25_conn_t    conn_b;
    ax25_address_t remote_addr_a;   /* remote address for conn_a (= addr_b) */

    slot_state_t state;
    SemaphoreHandle_t  mutex;
    StaticSemaphore_t  mutex_buf;

    uint8_t  tx_data[CONFIG_STRESS_MAX_DATA_SIZE];
    size_t   tx_len;
    size_t   tx_sent;

    uint8_t  echo_buf[CONFIG_STRESS_MAX_DATA_SIZE];
    size_t   echo_len;
    size_t   echo_sent;

    uint8_t  rx_data[CONFIG_STRESS_MAX_DATA_SIZE];
    size_t   rx_len;

    volatile bool a_connected;
    volatile bool b_connected;
    volatile bool a_done;

    uint32_t conn_number;
    bool     data_ok;
    uint32_t start_tick;
} conn_slot_t;

static conn_slot_t g_slots[CONFIG_STRESS_SIMULTANEOUS_CONNECTIONS];

// ---------------------------------------------------------------------------
// Global statistics
// ---------------------------------------------------------------------------

static uint32_t g_ok;
static uint32_t g_fail;
static uint32_t g_data_errors;
static uint64_t g_bytes_ok;
static uint32_t g_next_conn_num;

// ---------------------------------------------------------------------------
// Frame loopback callbacks (one pair per slot direction)
// ---------------------------------------------------------------------------

static void on_a_frame(const ax25_frame_t *frame, void *ud)
{
    conn_slot_t *s = ud;
    loopback_msg_t msg;
    msg.dest = &s->conn_b;
    msg.frame = *frame;
    if (xQueueSend(g_loopback_queue, &msg, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGW(TAG, "Loopback queue full — dropping frame (a→b)");
    }
}

static void on_b_frame(const ax25_frame_t *frame, void *ud)
{
    conn_slot_t *s = ud;
    loopback_msg_t msg;
    msg.dest = &s->conn_a;
    msg.frame = *frame;
    if (xQueueSend(g_loopback_queue, &msg, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGW(TAG, "Loopback queue full — dropping frame (b→a)");
    }
}

// ---------------------------------------------------------------------------
// Application callbacks
// ---------------------------------------------------------------------------

static void on_a_connected(ax25_address_t remote_addr, bool is_local_initiated, void *ud)
{
    (void)remote_addr;
    conn_slot_t *s = ud;
    (void)is_local_initiated;
    s->a_connected = true;
}

static void on_b_connected(ax25_address_t remote_addr, bool is_local_initiated, void *ud)
{
    (void)remote_addr;
    conn_slot_t *s = ud;
    (void)is_local_initiated;
    s->b_connected = true;
}

static void on_a_disconnected(void *ud)
{
    conn_slot_t *s = ud;
    s->a_done = true;
}

static void on_a_data(const uint8_t *data, size_t len, void *ud)
{
    conn_slot_t *s = ud;
    if (xSemaphoreTake(s->mutex, portMAX_DELAY) == pdTRUE) {
        size_t space = CONFIG_STRESS_MAX_DATA_SIZE - s->rx_len;
        size_t n = (len < space) ? len : space;
        memcpy(s->rx_data + s->rx_len, data, n);
        s->rx_len += n;
        xSemaphoreGive(s->mutex);
    }
}

static void on_b_data(const uint8_t *data, size_t len, void *ud)
{
    conn_slot_t *s = ud;
    if (xSemaphoreTake(s->mutex, portMAX_DELAY) == pdTRUE) {
        size_t space = CONFIG_STRESS_MAX_DATA_SIZE - s->echo_len;
        size_t n = (len < space) ? len : space;
        memcpy(s->echo_buf + s->echo_len, data, n);
        s->echo_len += n;
        xSemaphoreGive(s->mutex);
    }
}

// ---------------------------------------------------------------------------
// Slot helpers
// ---------------------------------------------------------------------------

static void slot_start(conn_slot_t *s, uint32_t conn_number)
{
    s->a_connected = false;
    s->b_connected = false;
    s->a_done      = false;
    s->tx_sent     = 0;
    s->rx_len      = 0;
    s->echo_len    = 0;
    s->echo_sent   = 0;
    s->data_ok     = false;
    s->conn_number = conn_number;
    s->start_tick  = xTaskGetTickCount();

    uint32_t rng   = esp_random();
    size_t   range = (size_t)(CONFIG_STRESS_MAX_DATA_SIZE - CONFIG_STRESS_MIN_DATA_SIZE) + 1;
    s->tx_len      = CONFIG_STRESS_MIN_DATA_SIZE + (rng % range);

    size_t words = s->tx_len / sizeof(uint32_t);
    uint32_t *wp = (uint32_t *)(void *)s->tx_data;
    for (size_t i = 0; i < words; i++) {
        wp[i] = esp_random();
    }
    for (size_t i = words * sizeof(uint32_t); i < s->tx_len; i++) {
        s->tx_data[i] = (uint8_t)esp_random();
    }

    s->state = SLOT_CONNECTING;
    ax25_conn_connect(&s->conn_a, &s->remote_addr_a);
}

static bool slot_drive_echo(conn_slot_t *s)
{
    size_t echo_avail = s->echo_sent;
    if (xSemaphoreTake(s->mutex, 0) == pdTRUE) {
        echo_avail = s->echo_len;
        xSemaphoreGive(s->mutex);
    }

    while (s->echo_sent < echo_avail) {
        size_t rem   = echo_avail - s->echo_sent;
        size_t chunk = (rem < AX25_MAX_INFO_LEN) ? rem : AX25_MAX_INFO_LEN;
        esp_err_t err = ax25_conn_send_data(&s->conn_b,
                                             s->echo_buf + s->echo_sent,
                                             chunk);
        if (err == ESP_OK) {
            s->echo_sent += chunk;
        } else {
            break;
        }
    }

    return (s->echo_sent < s->tx_len);
}

// ---------------------------------------------------------------------------
// Progress logging
// ---------------------------------------------------------------------------

static void log_progress(void)
{
    uint32_t total = g_ok + g_fail;
    if (total == 0 || (total % LOG_INTERVAL) == 0) {
        ESP_LOGI(TAG, "Progress %4lu/%d  ok=%-4lu fail=%-4lu data_err=%-4lu bytes=%llu",
                 (unsigned long)total,
                 CONFIG_STRESS_TOTAL_CONNECTIONS,
                 (unsigned long)g_ok,
                 (unsigned long)g_fail,
                 (unsigned long)g_data_errors,
                 (unsigned long long)g_bytes_ok);
    }
}

// ---------------------------------------------------------------------------
// Test case
// ---------------------------------------------------------------------------

TEST_CASE("AX25: stress test - connected sessions with echo verification", "[ax25_stress]")
{
    /* Reset global state in case the test is re-run. */
    g_ok             = 0;
    g_fail           = 0;
    g_data_errors    = 0;
    g_bytes_ok       = 0;
    g_next_conn_num  = 0;

    ESP_LOGI(TAG, "Stress test starting");
    ESP_LOGI(TAG, "  Total connections    : %d", CONFIG_STRESS_TOTAL_CONNECTIONS);
    ESP_LOGI(TAG, "  Simultaneous         : %d", CONFIG_STRESS_SIMULTANEOUS_CONNECTIONS);
    ESP_LOGI(TAG, "  Min payload (bytes)  : %d", CONFIG_STRESS_MIN_DATA_SIZE);
    ESP_LOGI(TAG, "  Max payload (bytes)  : %d", CONFIG_STRESS_MAX_DATA_SIZE);

    /* Loopback queue — static */
    g_loopback_queue = xQueueCreateStatic(LOOPBACK_QUEUE_DEPTH,
                                          sizeof(loopback_msg_t),
                                          g_loopback_queue_storage,
                                          &g_loopback_queue_buf);
    TEST_ASSERT_NOT_NULL(g_loopback_queue);

    /* Connection configuration shared by every slot. */
    ax25_conn_config_t conn_cfg = AX25_CONN_CONFIG_DEFAULT();
    conn_cfg.t1_ms        = CONN_T1_MS;
    conn_cfg.t3_ms        = CONN_T3_MS;
    conn_cfg.n2_retries   = CONN_MAX_RETRIES;

    /* Initialise all slots. */
    for (int i = 0; i < CONFIG_STRESS_SIMULTANEOUS_CONNECTIONS; i++) {
        conn_slot_t *s = &g_slots[i];

        s->index    = i;
        s->state    = SLOT_IDLE;
        s->mutex    = xSemaphoreCreateMutexStatic(&s->mutex_buf);
        TEST_ASSERT_NOT_NULL(s->mutex);
        /* tx_data, rx_data, echo_buf are now embedded arrays — no malloc. */

        char cs_a[8], cs_b[8];
        snprintf(cs_a, sizeof(cs_a), "CLI%03d", i + 1);
        snprintf(cs_b, sizeof(cs_b), "SRV%03d", i + 1);

        ax25_address_t addr_a, addr_b;
        ax25_address_from_string(cs_a, &addr_a);
        ax25_address_from_string(cs_b, &addr_b);

        /* Store remote address for conn_a so slot_start can pass it to connect */
        s->remote_addr_a = addr_b;

        ax25_conn_callbacks_t cb_a = {
            .on_connect    = on_a_connected,
            .on_disconnect = on_a_disconnected,
            .on_data       = on_a_data,
            .on_tx_frame      = on_a_frame,
        };
        esp_err_t err = ax25_conn_init(&s->conn_a, &addr_a, &cb_a, s, &conn_cfg);
        TEST_ASSERT_EQUAL_INT(ESP_OK, err);

        ax25_conn_callbacks_t cb_b = {
            .on_connect = on_b_connected,
            .on_data    = on_b_data,
            .on_tx_frame   = on_b_frame,
        };
        err = ax25_conn_init(&s->conn_b, &addr_b, &cb_b, s, &conn_cfg);
        TEST_ASSERT_EQUAL_INT(ESP_OK, err);
    }

    /* Start the loopback task — static storage. */
    static StaticTask_t lb_tcb;
    static StackType_t  lb_stack[4096];
    TaskHandle_t lb_handle = xTaskCreateStatic(loopback_task, "ax25_lb",
                                               4096, NULL, 15,
                                               lb_stack, &lb_tcb);
    TEST_ASSERT_NOT_NULL(lb_handle);

    /* Start the initial batch of connections. */
    for (int i = 0;
         i < CONFIG_STRESS_SIMULTANEOUS_CONNECTIONS &&
         g_next_conn_num < (uint32_t)CONFIG_STRESS_TOTAL_CONNECTIONS;
         i++) {
        slot_start(&g_slots[i], g_next_conn_num++);
    }

    /* Supervisor loop — runs inline (no separate task needed). */
    while (1) {
        uint32_t now    = xTaskGetTickCount();
        int      active = 0;

        for (int i = 0; i < CONFIG_STRESS_SIMULTANEOUS_CONNECTIONS; i++) {
            conn_slot_t *s = &g_slots[i];

            switch (s->state) {

            case SLOT_IDLE:
                break;

            case SLOT_CONNECTING:
                active++;
                if (s->a_connected && s->b_connected) {
                    s->state   = SLOT_SENDING;
                    s->tx_sent = 0;
                } else if ((now - s->start_tick) > pdMS_TO_TICKS(SLOT_TIMEOUT_MS)) {
                    ESP_LOGE(TAG, "[%d] #%lu connect timeout",
                             i, (unsigned long)s->conn_number);
                    s->state = SLOT_ERROR;
                }
                break;

            case SLOT_SENDING:
                active++;
                if ((now - s->start_tick) > pdMS_TO_TICKS(SLOT_TIMEOUT_MS)) {
                    ESP_LOGE(TAG, "[%d] #%lu send timeout (tx=%zu/%zu)",
                             i, (unsigned long)s->conn_number,
                             s->tx_sent, s->tx_len);
                    s->state = SLOT_ERROR;
                    break;
                }
                if (s->tx_sent < s->tx_len) {
                    size_t rem   = s->tx_len - s->tx_sent;
                    size_t chunk = (rem < AX25_MAX_INFO_LEN) ? rem : AX25_MAX_INFO_LEN;
                    if (ax25_conn_send_data(&s->conn_a,
                                            s->tx_data + s->tx_sent,
                                            chunk) == ESP_OK) {
                        s->tx_sent += chunk;
                    }
                }
                slot_drive_echo(s);
                if (s->tx_sent >= s->tx_len) {
                    s->state = SLOT_WAITING_ECHO;
                }
                break;

            case SLOT_WAITING_ECHO:
                active++;
                if ((now - s->start_tick) > pdMS_TO_TICKS(SLOT_TIMEOUT_MS)) {
                    ESP_LOGE(TAG, "[%d] #%lu echo timeout (rx=%zu/%zu)",
                             i, (unsigned long)s->conn_number,
                             s->rx_len, s->tx_len);
                    s->state = SLOT_ERROR;
                    break;
                }
                slot_drive_echo(s);
                {
                    size_t rx_now = 0;
                    if (xSemaphoreTake(s->mutex, 0) == pdTRUE) {
                        rx_now = s->rx_len;
                        xSemaphoreGive(s->mutex);
                    } else {
                        break;
                    }
                    if (rx_now >= s->tx_len && s->echo_sent >= s->tx_len) {
                        s->data_ok = (memcmp(s->rx_data, s->tx_data, s->tx_len) == 0);
                        if (!s->data_ok) {
                            ESP_LOGE(TAG, "[%d] #%lu DATA MISMATCH len=%zu",
                                     i, (unsigned long)s->conn_number, s->tx_len);
                        }
                        s->state = SLOT_DISCONNECTING;
                        ax25_conn_shutdown(&s->conn_a);
                    }
                }
                break;

            case SLOT_DISCONNECTING:
                active++;
                if (s->a_done) {
                    s->state = SLOT_DONE;
                } else if ((now - s->start_tick) > pdMS_TO_TICKS(SLOT_TIMEOUT_MS)) {
                    ESP_LOGE(TAG, "[%d] #%lu disconnect timeout",
                             i, (unsigned long)s->conn_number);
                    s->state = SLOT_ERROR;
                }
                break;

            case SLOT_DONE:
                g_ok++;
                g_bytes_ok += s->tx_len;
                if (!s->data_ok) {
                    g_data_errors++;
                }
                log_progress();
                s->state = SLOT_IDLE;
                if (g_next_conn_num < (uint32_t)CONFIG_STRESS_TOTAL_CONNECTIONS) {
                    slot_start(s, g_next_conn_num++);
                    active++;
                }
                break;

            case SLOT_ERROR:
                g_fail++;
                log_progress();
                s->state = SLOT_IDLE;
                if (g_next_conn_num < (uint32_t)CONFIG_STRESS_TOTAL_CONNECTIONS) {
                    slot_start(s, g_next_conn_num++);
                    active++;
                }
                break;
            }
        }

        if (active == 0 && g_next_conn_num >= (uint32_t)CONFIG_STRESS_TOTAL_CONNECTIONS) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_PERIOD_MS));
    }

    /* Summary */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Stress test COMPLETE");
    ESP_LOGI(TAG, "  Total attempted : %lu", (unsigned long)(g_ok + g_fail));
    ESP_LOGI(TAG, "  Passed          : %lu", (unsigned long)g_ok);
    ESP_LOGI(TAG, "  Failed          : %lu", (unsigned long)g_fail);
    ESP_LOGI(TAG, "  Data mismatches : %lu", (unsigned long)g_data_errors);
    ESP_LOGI(TAG, "  Bytes echoed    : %llu", (unsigned long long)g_bytes_ok);
    ESP_LOGI(TAG, "========================================");

    /* Stop the loopback task — queue is drained by this point. */
    vTaskDelete(lb_handle);

    /* Clean up slot resources (static storage — no free needed for buffers). */
    for (int i = 0; i < CONFIG_STRESS_SIMULTANEOUS_CONNECTIONS; i++) {
        conn_slot_t *s = &g_slots[i];
        ax25_conn_deinit(&s->conn_a);
        ax25_conn_deinit(&s->conn_b);
        vSemaphoreDelete(s->mutex);
        s->mutex = NULL;
    }

    /* Static queue — vQueueDelete is still valid for bookkeeping. */
    vQueueDelete(g_loopback_queue);
    g_loopback_queue = NULL;

    /* Assertions — any failure or data error counts as a test failure. */
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, g_fail,        "connections failed");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, g_data_errors, "data integrity errors");
}

#endif /* CONFIG_STRESS_TEST_ENABLE */
