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
 * @file test_ax25_hardening.c
 * @brief Memory-fragmentation hardening tests for the ESP-AX25 stack.
 *
 * Validates that all static-storage refactoring is correct:
 *   - Buffer pools use embedded static arrays (no heap)
 *   - AGWPE server uses caller-provided storage
 *   - Router slots are statically allocated
 *   - FreeRTOS primitives use static creation variants
 *   - Pool exhaustion and boundary conditions are handled gracefully
 *
 * These tests do NOT require CONFIG_STRESS_TEST_ENABLE; they are always built.
 */

#include "unity.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "ax25_buffer.h"
#include "ax25_router.h"
#include "ax25_agwpe_server.h"
#include "ax25_address.h"
#include "ax25_frame.h"
#include "ax25_types.h"

/* -------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------- */

/** Heap-free-bytes snapshot.  On ESP-IDF this uses esp_get_free_heap_size();
 *  on the host (Linux Unity runner) we stub it to 0 so the test still
 *  compiles but the heap-delta assertion is skipped. */
#ifdef CONFIG_IDF_TARGET
#include "esp_system.h"
static size_t heap_free(void) { return esp_get_free_heap_size(); }
#else
static size_t heap_free(void) { return 0; }
#endif

/* =========================================================================
 * 1. Buffer pool — static array, no heap
 * ======================================================================= */

TEST_CASE("Hardening: buffer pool uses embedded static storage", "[hardening]")
{
    ax25_buffer_pool_t pool;
    size_t before = heap_free();

    esp_err_t err = ax25_buffer_pool_init(&pool, 4);
    TEST_ASSERT_EQUAL(ESP_OK, err);

    size_t after = heap_free();

    /* The pool should NOT have called malloc; heap delta ≈ 0.
     * Allow a small tolerance (256 B) for FreeRTOS bookkeeping in the
     * static semaphore paths on some ports.  On Linux stubs heap_free()
     * returns 0, so skip the check. */
    if (before > 0) {
        TEST_ASSERT_MESSAGE(before - after < 256,
                            "Buffer pool init consumed unexpected heap");
    }

    /* Verify pool_size and that buffers pointer points inside the struct. */
    TEST_ASSERT_EQUAL_size_t(4, pool.pool_size);
    TEST_ASSERT_NOT_NULL(pool.buffers);

    /* The buffers array must live inside pool (static embed). */
    uintptr_t pool_start = (uintptr_t)&pool;
    uintptr_t pool_end   = pool_start + sizeof(pool);
    uintptr_t buf_addr   = (uintptr_t)pool.buffers;
    TEST_ASSERT_MESSAGE(buf_addr >= pool_start && buf_addr < pool_end,
                        "pool.buffers must point to the embedded static array");

    ax25_buffer_pool_deinit(&pool);
}

TEST_CASE("Hardening: buffer pool rejects size > AX25_BUFFER_POOL_MAX_SIZE",
          "[hardening]")
{
    ax25_buffer_pool_t pool;
    /* AX25_BUFFER_POOL_MAX_SIZE defaults to 16; asking for 64 must fail. */
    esp_err_t err = ax25_buffer_pool_init(&pool, AX25_BUFFER_POOL_MAX_SIZE + 1);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, err);
}

TEST_CASE("Hardening: buffer pool alloc-exhaust-free cycle", "[hardening]")
{
    ax25_buffer_pool_t pool;
    const size_t SZ = 4;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_buffer_pool_init(&pool, SZ));

    ax25_buffer_t *bufs[4];

    /* Exhaust all buffers. */
    for (size_t i = 0; i < SZ; i++) {
        bufs[i] = ax25_buffer_alloc(&pool, 50 /*timeout ms*/);
        TEST_ASSERT_NOT_NULL(bufs[i]);
    }

    /* Next alloc must fail (timeout). */
    ax25_buffer_t *overflow = ax25_buffer_alloc(&pool, 50);
    TEST_ASSERT_NULL(overflow);

    /* Free one and re-alloc must succeed. */
    ax25_buffer_free(&pool, bufs[0]);
    bufs[0] = ax25_buffer_alloc(&pool, 50);
    TEST_ASSERT_NOT_NULL(bufs[0]);

    /* Clean up. */
    for (size_t i = 0; i < SZ; i++) {
        ax25_buffer_free(&pool, bufs[i]);
    }
    ax25_buffer_pool_deinit(&pool);
}

/* =========================================================================
 * 2. Buffer pool boundary — exact max
 * ======================================================================= */

TEST_CASE("Hardening: buffer pool at exact max capacity", "[hardening]")
{
    ax25_buffer_pool_t pool;
    esp_err_t err = ax25_buffer_pool_init(&pool, AX25_BUFFER_POOL_MAX_SIZE);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_size_t(AX25_BUFFER_POOL_MAX_SIZE, pool.pool_size);

    /* Allocate all, verify they are distinct. */
    ax25_buffer_t *bufs[AX25_BUFFER_POOL_MAX_SIZE];
    for (size_t i = 0; i < AX25_BUFFER_POOL_MAX_SIZE; i++) {
        bufs[i] = ax25_buffer_alloc(&pool, 50);
        TEST_ASSERT_NOT_NULL(bufs[i]);
        for (size_t j = 0; j < i; j++) {
            TEST_ASSERT_NOT_EQUAL(bufs[j], bufs[i]);
        }
    }

    for (size_t i = 0; i < AX25_BUFFER_POOL_MAX_SIZE; i++) {
        ax25_buffer_free(&pool, bufs[i]);
    }
    ax25_buffer_pool_deinit(&pool);
}

/* =========================================================================
 * 3. AGWPE server — caller-provided storage
 * ======================================================================= */

TEST_CASE("Hardening: AGWPE server uses caller-provided struct", "[hardening]")
{
    /* The server struct lives on our stack — no heap allocation. */
    ax25_agwpe_server_t srv;
    memset(&srv, 0xAA, sizeof(srv));   /* poison to detect partial init */

    ax25_agwpe_server_config_t cfg = {
        .port          = 0,        /* won't actually bind */
        .max_clients   = 1,
        .local_address = {{0}},
    };
    ax25_address_from_string("N0CALL", &cfg.local_address);

    /* Init should succeed and write into our storage. */
    esp_err_t err = ax25_agwpe_server_init(&cfg, &srv);
    /* Accept ESP_OK or ESP_FAIL (socket bind may fail in test env);
     * the point is it wrote into *srv without heap-allocating a new one. */
    (void)err;

    /* Verify the struct was touched (poison overwritten). */
    uint8_t poison[sizeof(srv)];
    memset(poison, 0xAA, sizeof(poison));
    TEST_ASSERT_MESSAGE(memcmp(&srv, poison, sizeof(srv)) != 0,
                        "Server struct was not written to by init");

    ax25_agwpe_server_deinit(&srv);
}

/* =========================================================================
 * 4. Router — static slot array
 * ======================================================================= */

/** Dummy send callback for router port registration. */
static esp_err_t dummy_send(const ax25_frame_t *f, void *ud)
{
    (void)f; (void)ud;
    return ESP_OK;
}

TEST_CASE("Hardening: router uses static port slots", "[hardening]")
{
    ax25_router_init();

    size_t before = heap_free();

    ax25_address_t addr;
    ax25_address_from_string("TEST-1", &addr);

    ax25_router_port_t port = {
        .name      = "tst",
        .send      = dummy_send,
        .user_data = NULL,
    };

    esp_err_t err = ax25_router_add_port(&port);
    TEST_ASSERT_EQUAL(ESP_OK, err);

    size_t after = heap_free();

    /* Adding a port should not allocate heap (static slots). */
    if (before > 0) {
        TEST_ASSERT_MESSAGE(before - after < 512,
                            "Router add_port consumed unexpected heap");
    }

    ax25_router_remove_port(&port);
    ax25_router_deinit();
}

TEST_CASE("Hardening: router rejects more than ROUTER_MAX_PORTS", "[hardening]")
{
    ax25_router_init();

    ax25_router_port_t ports[CONFIG_AX25_ROUTER_MAX_PORTS + 1];
    char names[CONFIG_AX25_ROUTER_MAX_PORTS + 1][8];

    /* Fill all slots. */
    for (int i = 0; i < CONFIG_AX25_ROUTER_MAX_PORTS; i++) {
        snprintf(names[i], sizeof(names[i]), "p%d", i);
        memset(&ports[i], 0, sizeof(ports[i]));
        ports[i].name      = names[i];
        ports[i].send      = dummy_send;
        ports[i].user_data = NULL;
        TEST_ASSERT_EQUAL(ESP_OK, ax25_router_add_port(&ports[i]));
    }

    /* One more must fail. */
    int last = CONFIG_AX25_ROUTER_MAX_PORTS;
    snprintf(names[last], sizeof(names[last]), "pX");
    memset(&ports[last], 0, sizeof(ports[last]));
    ports[last].name      = names[last];
    ports[last].send      = dummy_send;
    ports[last].user_data = NULL;
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ax25_router_add_port(&ports[last]));

    /* Clean up. */
    for (int i = 0; i < CONFIG_AX25_ROUTER_MAX_PORTS; i++) {
        ax25_router_remove_port(&ports[i]);
    }
    ax25_router_deinit();
}

/* =========================================================================
 * 5. Static FreeRTOS primitives — mutex round-trip
 * ======================================================================= */

TEST_CASE("Hardening: static mutex create / take / give", "[hardening]")
{
    StaticSemaphore_t buf;
    SemaphoreHandle_t mtx = xSemaphoreCreateMutexStatic(&buf);
    TEST_ASSERT_NOT_NULL(mtx);

    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(mtx, 0));
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreGive(mtx));

    /* Static semaphores do not need vSemaphoreDelete, but it's safe. */
    vSemaphoreDelete(mtx);
}

TEST_CASE("Hardening: static counting semaphore create / signal", "[hardening]")
{
    StaticSemaphore_t buf;
    SemaphoreHandle_t sem = xSemaphoreCreateCountingStatic(4, 4, &buf);
    TEST_ASSERT_NOT_NULL(sem);

    /* Take all 4. */
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(sem, 0));
    }
    /* 5th take must fail. */
    TEST_ASSERT_EQUAL(pdFALSE, xSemaphoreTake(sem, 0));

    /* Give back one, take again succeeds. */
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreGive(sem));
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(sem, 0));

    vSemaphoreDelete(sem);
}

/* =========================================================================
 * 6. Rapid buffer alloc/free — no fragmentation
 * ======================================================================= */

TEST_CASE("Hardening: rapid buffer alloc/free cycle (no fragmentation)",
          "[hardening]")
{
    ax25_buffer_pool_t pool;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_buffer_pool_init(&pool, 4));

    /* Run 200 alloc-use-free cycles; if the pool leaked or fragmented,
     * later allocs would fail. */
    for (int round = 0; round < 200; round++) {
        ax25_buffer_t *b = ax25_buffer_alloc(&pool, 50);
        TEST_ASSERT_NOT_NULL(b);

        /* Write some data to prove the buffer is usable. */
        memset(b->data, (uint8_t)round, 64);
        b->len = 64;

        ax25_buffer_free(&pool, b);
    }

    ax25_buffer_pool_deinit(&pool);
}

/* =========================================================================
 * 7. Multi-threaded buffer pool stress
 * ======================================================================= */

#define MT_POOL_SIZE   4
#define MT_ITERATIONS  100
#define MT_NUM_TASKS   3

typedef struct {
    ax25_buffer_pool_t *pool;
    int                 task_id;
    volatile bool       done;
    int                 ok_count;
} mt_ctx_t;

static void mt_alloc_free_task(void *arg)
{
    mt_ctx_t *ctx = arg;
    int ok = 0;

    for (int i = 0; i < MT_ITERATIONS; i++) {
        ax25_buffer_t *b = ax25_buffer_alloc(ctx->pool, 200 /*ms*/);
        if (b) {
            /* Touch the buffer to detect corruption. */
            memset(b->data, (uint8_t)(ctx->task_id ^ i), 32);
            b->len = 32;
            vTaskDelay(1);   /* yield to other tasks */
            ax25_buffer_free(ctx->pool, b);
            ok++;
        }
    }

    ctx->ok_count = ok;
    ctx->done = true;
    vTaskDelete(NULL);
}

TEST_CASE("Hardening: multi-threaded buffer pool stress", "[hardening]")
{
    ax25_buffer_pool_t pool;
    TEST_ASSERT_EQUAL(ESP_OK, ax25_buffer_pool_init(&pool, MT_POOL_SIZE));

    static mt_ctx_t ctxs[MT_NUM_TASKS];
    static StaticTask_t tcbs[MT_NUM_TASKS];
    static StackType_t stacks[MT_NUM_TASKS][2048];

    for (int i = 0; i < MT_NUM_TASKS; i++) {
        ctxs[i].pool     = &pool;
        ctxs[i].task_id  = i;
        ctxs[i].done     = false;
        ctxs[i].ok_count = 0;

        TaskHandle_t h = xTaskCreateStatic(mt_alloc_free_task,
                                           "mt_buf",
                                           2048,
                                           &ctxs[i],
                                           5,
                                           stacks[i],
                                           &tcbs[i]);
        TEST_ASSERT_NOT_NULL(h);
    }

    /* Wait for all tasks to finish (max 30 s). */
    for (int wait = 0; wait < 3000; wait++) {
        bool all_done = true;
        for (int i = 0; i < MT_NUM_TASKS; i++) {
            if (!ctxs[i].done) { all_done = false; break; }
        }
        if (all_done) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    int total_ok = 0;
    for (int i = 0; i < MT_NUM_TASKS; i++) {
        TEST_ASSERT_TRUE_MESSAGE(ctxs[i].done, "Task did not finish in time");
        total_ok += ctxs[i].ok_count;
    }

    /* With 3 tasks contending on 4 buffers, most allocs should succeed. */
    TEST_ASSERT_GREATER_THAN_INT(MT_NUM_TASKS * MT_ITERATIONS / 2, total_ok);

    ax25_buffer_pool_deinit(&pool);
}
