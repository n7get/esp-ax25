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
 * @file test_ax25_buffer.c
 * @brief Unit tests for AX.25 buffer pool
 */

#include "unity.h"
#include "ax25_buffer.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Pool lifecycle
// ---------------------------------------------------------------------------

TEST_CASE("AX25Buffer: pool init and deinit", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    esp_err_t err = ax25_buffer_pool_init(&pool, 4);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_NOT_NULL(pool.buffers);
    TEST_ASSERT_EQUAL_size_t(4, pool.pool_size);
    TEST_ASSERT_NOT_NULL(pool.mutex);
    TEST_ASSERT_NOT_NULL(pool.available);

    ax25_buffer_pool_deinit(&pool);
    TEST_ASSERT_EQUAL_size_t(0, pool.pool_size);
    TEST_ASSERT_NULL(pool.mutex);
    TEST_ASSERT_NULL(pool.available);
}

TEST_CASE("AX25Buffer: pool init rejects NULL and zero size", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_buffer_pool_init(NULL, 4));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_buffer_pool_init(&pool, 0));
}

// ---------------------------------------------------------------------------
// Alloc / free / available count
// ---------------------------------------------------------------------------

TEST_CASE("AX25Buffer: alloc returns non-NULL buffer", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 4);

    ax25_buffer_t *buf = ax25_buffer_alloc(&pool, 100);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL_size_t(0, buf->len);    /* fresh buffer starts empty */

    ax25_buffer_free(&pool, buf);
    ax25_buffer_pool_deinit(&pool);
}

TEST_CASE("AX25Buffer: available count decrements on alloc", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 4);

    TEST_ASSERT_EQUAL_size_t(4, ax25_buffer_pool_available(&pool));
    ax25_buffer_t *b1 = ax25_buffer_alloc(&pool, 100);
    TEST_ASSERT_EQUAL_size_t(3, ax25_buffer_pool_available(&pool));
    ax25_buffer_t *b2 = ax25_buffer_alloc(&pool, 100);
    TEST_ASSERT_EQUAL_size_t(2, ax25_buffer_pool_available(&pool));

    ax25_buffer_free(&pool, b1);
    ax25_buffer_free(&pool, b2);
    ax25_buffer_pool_deinit(&pool);
}

TEST_CASE("AX25Buffer: free returns buffer to pool", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 2);

    ax25_buffer_t *buf = ax25_buffer_alloc(&pool, 100);
    TEST_ASSERT_EQUAL_size_t(1, ax25_buffer_pool_available(&pool));

    ax25_buffer_free(&pool, buf);
    TEST_ASSERT_EQUAL_size_t(2, ax25_buffer_pool_available(&pool));

    ax25_buffer_pool_deinit(&pool);
}

TEST_CASE("AX25Buffer: pool exhaustion returns NULL immediately", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 2);

    ax25_buffer_t *b1 = ax25_buffer_alloc(&pool, 100);
    ax25_buffer_t *b2 = ax25_buffer_alloc(&pool, 100);
    TEST_ASSERT_NOT_NULL(b1);
    TEST_ASSERT_NOT_NULL(b2);
    TEST_ASSERT_EQUAL_size_t(0, ax25_buffer_pool_available(&pool));

    /* timeout_ms = 0: must return NULL without blocking. */
    ax25_buffer_t *b3 = ax25_buffer_alloc(&pool, 0);
    TEST_ASSERT_NULL(b3);

    ax25_buffer_free(&pool, b1);
    ax25_buffer_free(&pool, b2);
    ax25_buffer_pool_deinit(&pool);
}

TEST_CASE("AX25Buffer: double free is safe", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 4);

    ax25_buffer_t *buf = ax25_buffer_alloc(&pool, 100);
    ax25_buffer_free(&pool, buf);
    TEST_ASSERT_EQUAL_size_t(4, ax25_buffer_pool_available(&pool));

    /* Second free must not crash and must not change available count. */
    ax25_buffer_free(&pool, buf);
    TEST_ASSERT_EQUAL_size_t(4, ax25_buffer_pool_available(&pool));

    ax25_buffer_pool_deinit(&pool);
}

// ---------------------------------------------------------------------------
// buffer_copy
// ---------------------------------------------------------------------------

TEST_CASE("AX25Buffer: buffer_copy success", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 2);

    ax25_buffer_t *buf = ax25_buffer_alloc(&pool, 100);
    const uint8_t src[] = {0xDE, 0xAD, 0xBE, 0xEF};
    esp_err_t err = ax25_buffer_copy(buf, src, sizeof(src));
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_size_t(4, buf->len);
    TEST_ASSERT_EQUAL_MEMORY(src, buf->data, 4);

    ax25_buffer_free(&pool, buf);
    ax25_buffer_pool_deinit(&pool);
}

TEST_CASE("AX25Buffer: buffer_copy rejects oversized data", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 2);

    ax25_buffer_t *buf = ax25_buffer_alloc(&pool, 100);
    /* Allocate a large dummy source on the heap to avoid stack overflow. */
    uint8_t *big = malloc(AX25_BUFFER_SIZE + 1);
    TEST_ASSERT_NOT_NULL(big);
    memset(big, 0xAA, AX25_BUFFER_SIZE + 1);

    esp_err_t err = ax25_buffer_copy(buf, big, AX25_BUFFER_SIZE + 1);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, err);

    free(big);
    ax25_buffer_free(&pool, buf);
    ax25_buffer_pool_deinit(&pool);
}

// ---------------------------------------------------------------------------
// buffer_append
// ---------------------------------------------------------------------------

TEST_CASE("AX25Buffer: buffer_append success", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 2);
    ax25_buffer_t *buf = ax25_buffer_alloc(&pool, 100);

    const uint8_t a[] = {0x01, 0x02};
    const uint8_t b[] = {0x03, 0x04};
    TEST_ASSERT_EQUAL(ESP_OK, ax25_buffer_append(buf, a, 2));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_buffer_append(buf, b, 2));
    TEST_ASSERT_EQUAL_size_t(4, buf->len);

    const uint8_t expected[] = {0x01, 0x02, 0x03, 0x04};
    TEST_ASSERT_EQUAL_MEMORY(expected, buf->data, 4);

    ax25_buffer_free(&pool, buf);
    ax25_buffer_pool_deinit(&pool);
}

TEST_CASE("AX25Buffer: buffer_append rejects overflow", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 2);
    ax25_buffer_t *buf = ax25_buffer_alloc(&pool, 100);

    /* Fill the buffer to capacity. */
    uint8_t *fill = malloc(AX25_BUFFER_SIZE);
    TEST_ASSERT_NOT_NULL(fill);
    memset(fill, 0x00, AX25_BUFFER_SIZE);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_buffer_copy(buf, fill, AX25_BUFFER_SIZE));
    free(fill);

    /* Any further append must fail. */
    const uint8_t extra = 0xFF;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, ax25_buffer_append(buf, &extra, 1));

    ax25_buffer_free(&pool, buf);
    ax25_buffer_pool_deinit(&pool);
}

// ---------------------------------------------------------------------------
// buffer_append_byte
// ---------------------------------------------------------------------------

TEST_CASE("AX25Buffer: buffer_append_byte boundary", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 2);
    ax25_buffer_t *buf = ax25_buffer_alloc(&pool, 100);

    /* Fill to one byte short of full. */
    uint8_t *fill = malloc(AX25_BUFFER_SIZE - 1);
    TEST_ASSERT_NOT_NULL(fill);
    memset(fill, 0x00, AX25_BUFFER_SIZE - 1);
    ax25_buffer_copy(buf, fill, AX25_BUFFER_SIZE - 1);
    free(fill);

    /* One more byte must succeed. */
    TEST_ASSERT_EQUAL(ESP_OK, ax25_buffer_append_byte(buf, 0xFF));
    TEST_ASSERT_EQUAL_size_t(AX25_BUFFER_SIZE, buf->len);

    /* Now full — next byte must fail. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, ax25_buffer_append_byte(buf, 0xAA));

    ax25_buffer_free(&pool, buf);
    ax25_buffer_pool_deinit(&pool);
}

// ---------------------------------------------------------------------------
// buffer_reset
// ---------------------------------------------------------------------------

TEST_CASE("AX25Buffer: buffer_reset clears length", "[ax25_buffer]")
{
    ax25_buffer_pool_t pool;
    ax25_buffer_pool_init(&pool, 2);
    ax25_buffer_t *buf = ax25_buffer_alloc(&pool, 100);

    const uint8_t data[] = {0x01, 0x02, 0x03};
    ax25_buffer_copy(buf, data, sizeof(data));
    TEST_ASSERT_EQUAL_size_t(3, buf->len);

    ax25_buffer_reset(buf);
    TEST_ASSERT_EQUAL_size_t(0, buf->len);

    /* Data can be rewritten after reset. */
    const uint8_t data2[] = {0xAA};
    ax25_buffer_copy(buf, data2, sizeof(data2));
    TEST_ASSERT_EQUAL_size_t(1, buf->len);
    TEST_ASSERT_EQUAL_UINT8(0xAA, buf->data[0]);

    ax25_buffer_free(&pool, buf);
    ax25_buffer_pool_deinit(&pool);
}
