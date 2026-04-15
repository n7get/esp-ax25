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
 * @file test_main.c
 * @brief Main entry point for AX.25 unit tests
 */

#include "sdkconfig.h"
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "TEST";

static int64_t s_test_start_us;

static void log_psram_status(void)
{
#if CONFIG_SPIRAM
    size_t total_bytes = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t free_bytes = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    if (total_bytes == 0) {
        ESP_LOGW(TAG, "PSRAM enabled in sdkconfig but not detected at runtime");
        return;
    }

    ESP_LOGI(TAG, "PSRAM detected: %u bytes total, %u bytes free",
             (unsigned)total_bytes,
             (unsigned)free_bytes);
#else
    ESP_LOGW(TAG, "PSRAM disabled in sdkconfig");
#endif
}

/* Called by Unity before each TEST_CASE. */
void setUp(void)
{
    s_test_start_us = esp_timer_get_time();
}

/* Called by Unity after each TEST_CASE. */
void tearDown(void)
{
    int64_t elapsed_us = esp_timer_get_time() - s_test_start_us;
    ESP_LOGI(TAG, "  took %lld us", elapsed_us);
}

void app_main(void)
{
    /* Small delay for serial to initialize. */
    vTaskDelay(pdMS_TO_TICKS(2000));

    log_psram_status();

    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
