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
 * @file ax25_buffer.h
 * @brief Fixed-size buffer pool for AX.25 frames
 * 
 * Implements a thread-safe buffer pool using fixed-size buffers to reduce
 * memory fragmentation from malloc/free operations.
 */

#ifndef AX25_BUFFER_H
#define AX25_BUFFER_H

#include "ax25_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Buffer structure for AX.25 frames
 */
typedef struct {
    uint8_t data[AX25_BUFFER_SIZE];     /**< Fixed-size data buffer */
    size_t len;                          /**< Current data length */
    bool in_use;                         /**< Whether buffer is currently allocated */
} ax25_buffer_t;

/** Compile-time max pool capacity (Kconfig-backed). */
#ifdef CONFIG_AX25_BUFFER_POOL_MAX_SIZE
#define AX25_BUFFER_POOL_MAX_SIZE  CONFIG_AX25_BUFFER_POOL_MAX_SIZE
#else
#define AX25_BUFFER_POOL_MAX_SIZE  16
#endif

/**
 * @brief Buffer pool structure
 *
 * All storage is embedded — no heap allocation is performed.
 */
typedef struct {
    ax25_buffer_t     buffers[AX25_BUFFER_POOL_MAX_SIZE]; /**< Static buffer array */
    size_t            pool_size;          /**< Active pool size (<= max)   */
    SemaphoreHandle_t mutex;              /**< Mutex for thread safety     */
    SemaphoreHandle_t available;          /**< Counting semaphore          */
    StaticSemaphore_t mutex_buf;          /**< Static mutex storage        */
    StaticSemaphore_t avail_buf;          /**< Static counting-sem storage */
} ax25_buffer_pool_t;

/**
 * @brief Initialize a buffer pool
 * 
 * @param pool Pointer to buffer pool structure
 * @param pool_size Number of buffers to allocate
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ax25_buffer_pool_init(ax25_buffer_pool_t* pool, size_t pool_size);

/**
 * @brief Deinitialize a buffer pool
 * 
 * @param pool Pointer to buffer pool structure
 */
void ax25_buffer_pool_deinit(ax25_buffer_pool_t* pool);

/**
 * @brief Allocate a buffer from the pool
 * 
 * Thread-safe. Blocks until a buffer is available or timeout occurs.
 * 
 * @param pool Pointer to buffer pool
 * @param timeout_ms Maximum time to wait for a buffer (portMAX_DELAY for infinite)
 * @return Pointer to allocated buffer, or NULL if timeout
 */
ax25_buffer_t* ax25_buffer_alloc(ax25_buffer_pool_t* pool, uint32_t timeout_ms);

/**
 * @brief Return a buffer to the pool
 * 
 * Thread-safe.
 * 
 * @param pool Pointer to buffer pool
 * @param buffer Buffer to return
 */
void ax25_buffer_free(ax25_buffer_pool_t* pool, ax25_buffer_t* buffer);

/**
 * @brief Get number of available buffers
 * 
 * Thread-safe.
 * 
 * @param pool Pointer to buffer pool
 * @return Number of available buffers
 */
size_t ax25_buffer_pool_available(ax25_buffer_pool_t* pool);

/**
 * @brief Reset buffer contents
 * 
 * @param buffer Buffer to reset
 */
static inline void ax25_buffer_reset(ax25_buffer_t* buffer) {
    if (buffer) {
        buffer->len = 0;
    }
}

/**
 * @brief Copy data to buffer
 * 
 * @param buffer Destination buffer
 * @param data Source data
 * @param len Length of data to copy
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE if data too large
 */
esp_err_t ax25_buffer_copy(ax25_buffer_t* buffer, const uint8_t* data, size_t len);

/**
 * @brief Append data to buffer
 * 
 * @param buffer Destination buffer
 * @param data Source data
 * @param len Length of data to append
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE if insufficient space
 */
esp_err_t ax25_buffer_append(ax25_buffer_t* buffer, const uint8_t* data, size_t len);

/**
 * @brief Append single byte to buffer
 * 
 * @param buffer Destination buffer
 * @param byte Byte to append
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE if buffer full
 */
esp_err_t ax25_buffer_append_byte(ax25_buffer_t* buffer, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* AX25_BUFFER_H */
