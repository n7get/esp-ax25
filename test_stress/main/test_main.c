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
 * @brief Main entry point for AX.25 stress tests
 */

#include "sdkconfig.h"
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "TEST_STRESS";

static int64_t s_test_start_us;

void setUp(void)
{
    s_test_start_us = esp_timer_get_time();
}

void tearDown(void)
{
    int64_t elapsed_us = esp_timer_get_time() - s_test_start_us;
    ESP_LOGI(TAG, "  took %lld us", elapsed_us);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));

    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
