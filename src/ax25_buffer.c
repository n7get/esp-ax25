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
 * @file ax25_buffer.c
 * @brief Implementation of fixed-size buffer pool for AX.25 frames
 *
 * Memory-hardened: all storage is embedded in the pool struct — no heap
 * allocation.  FreeRTOS primitives use the static variants.
 */

#include "ax25_buffer.h"
#include <string.h>
#include "esp_log.h"

static const char* TAG = "AX25_BUF";

esp_err_t ax25_buffer_pool_init(ax25_buffer_pool_t* pool, size_t pool_size) {
    if (!pool || pool_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (pool_size > AX25_BUFFER_POOL_MAX_SIZE) {
        ESP_LOGE(TAG, "Requested pool_size %zu exceeds compile-time max %d",
                 pool_size, AX25_BUFFER_POOL_MAX_SIZE);
        return ESP_ERR_INVALID_ARG;
    }

    memset(pool, 0, sizeof(*pool));
    pool->pool_size = pool_size;

    /* All buffers start as not-in-use (zeroed by memset) */

    /* Create mutex (static) */
    pool->mutex = xSemaphoreCreateMutexStatic(&pool->mutex_buf);
    if (!pool->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Create counting semaphore (static) */
    pool->available = xSemaphoreCreateCountingStatic(pool_size, pool_size,
                                                      &pool->avail_buf);
    if (!pool->available) {
        ESP_LOGE(TAG, "Failed to create counting semaphore");
        vSemaphoreDelete(pool->mutex);
        pool->mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Buffer pool initialized with %zu buffers of %d bytes each",
             pool_size, AX25_BUFFER_SIZE);
    return ESP_OK;
}

void ax25_buffer_pool_deinit(ax25_buffer_pool_t* pool) {
    if (!pool) {
        return;
    }

    if (pool->mutex) {
        vSemaphoreDelete(pool->mutex);
        pool->mutex = NULL;
    }

    if (pool->available) {
        vSemaphoreDelete(pool->available);
        pool->available = NULL;
    }

    /* Static storage — no free needed */
    pool->pool_size = 0;
    ESP_LOGI(TAG, "Buffer pool deinitialized");
}

ax25_buffer_t* ax25_buffer_alloc(ax25_buffer_pool_t* pool, uint32_t timeout_ms) {
    if (!pool || pool->pool_size == 0) {
        return NULL;
    }

    /* Wait for an available buffer */
    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? 
                       portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    
    if (xSemaphoreTake(pool->available, ticks) != pdTRUE) {
        ESP_LOGW(TAG, "Buffer pool exhausted, timeout after %lu ms", (unsigned long)timeout_ms);
        return NULL;
    }

    /* Lock the pool to find a free buffer */
    if (xSemaphoreTake(pool->mutex, portMAX_DELAY) != pdTRUE) {
        xSemaphoreGive(pool->available);
        return NULL;
    }

    ax25_buffer_t* buffer = NULL;
    for (size_t i = 0; i < pool->pool_size; i++) {
        if (!pool->buffers[i].in_use) {
            pool->buffers[i].in_use = true;
            pool->buffers[i].len = 0;
            buffer = &pool->buffers[i];
            break;
        }
    }

    xSemaphoreGive(pool->mutex);

    if (!buffer) {
        /* This shouldn't happen if semaphores are working correctly */
        ESP_LOGE(TAG, "Buffer allocation inconsistency");
        xSemaphoreGive(pool->available);
    }

    return buffer;
}

void ax25_buffer_free(ax25_buffer_pool_t* pool, ax25_buffer_t* buffer) {
    if (!pool || !buffer) {
        return;
    }

    /* Verify buffer belongs to this pool */
    ptrdiff_t index = buffer - pool->buffers;
    if (index < 0 || (size_t)index >= pool->pool_size) {
        ESP_LOGE(TAG, "Attempted to free buffer not belonging to pool");
        return;
    }

    if (xSemaphoreTake(pool->mutex, portMAX_DELAY) == pdTRUE) {
        if (buffer->in_use) {
            buffer->in_use = false;
            buffer->len = 0;
            xSemaphoreGive(pool->mutex);
            xSemaphoreGive(pool->available);
        } else {
            ESP_LOGW(TAG, "Double free detected");
            xSemaphoreGive(pool->mutex);
        }
    }
}

size_t ax25_buffer_pool_available(ax25_buffer_pool_t* pool) {
    if (!pool || !pool->available) {
        return 0;
    }
    return (size_t)uxSemaphoreGetCount(pool->available);
}

esp_err_t ax25_buffer_copy(ax25_buffer_t* buffer, const uint8_t* data, size_t len) {
    if (!buffer || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    if (len > AX25_BUFFER_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(buffer->data, data, len);
    buffer->len = len;
    return ESP_OK;
}

esp_err_t ax25_buffer_append(ax25_buffer_t* buffer, const uint8_t* data, size_t len) {
    if (!buffer || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    if (buffer->len + len > AX25_BUFFER_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(buffer->data + buffer->len, data, len);
    buffer->len += len;
    return ESP_OK;
}

esp_err_t ax25_buffer_append_byte(ax25_buffer_t* buffer, uint8_t byte) {
    if (!buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    if (buffer->len >= AX25_BUFFER_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    buffer->data[buffer->len++] = byte;
    return ESP_OK;
}
