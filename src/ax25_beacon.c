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
 * @file ax25_beacon.c
 * @brief AX.25 periodic beacon transmission implementation
 */

#include "ax25_beacon.h"

#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "ax25_address.h"
#include "ax25_frame.h"
#include "ax25_router.h"
#include "ax25_config.h"

static const char *TAG = "AX25_BEACON";

/*******************************************************************************
 * Module Context
 ******************************************************************************/
#define AX25_BEACON_WORKER_STACK_BYTES 6144
#define AX25_BEACON_WORKER_PRIORITY    4


typedef struct {
    bool initialized;
    TimerHandle_t timer;
    TaskHandle_t worker_task;
    StaticTask_t worker_task_tcb;
    StackType_t worker_task_stack[AX25_BEACON_WORKER_STACK_BYTES / sizeof(StackType_t)];
    bool worker_running;
    ax25_router_port_t source_port;
    bool source_port_registered;
} ax25_beacon_ctx_t;

static ax25_beacon_ctx_t s_ctx = {0};

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return -1;
}

static bool beacon_source_is_configured(void)
{
    char src_str[AX25_MAX_CALLSIGN_LEN + 5];
    ax25_cfg_get_str("beacon.source", src_str, sizeof(src_str));
    return src_str[0] != '\0';
}

static void beacon_source_port_on_frame(const ax25_frame_t *frame, void *user_data)
{
    (void)frame;
    (void)user_data;
}

/*******************************************************************************
 * Escape Sequence Processing
 ******************************************************************************/

/**
 * @brief Expand escape sequences in input string
 *
 * Processes escape sequences like \r (CR), \n (LF), \t (TAB), \\ (backslash),
 * and \xHH (hex byte).
 * Invalid escapes are treated as literal characters.
 *
 * @param src Source string (may contain escapes)
 * @param dst Output buffer
 * @param dst_len Maximum output length (including null terminator)
 * @return Number of characters written to dst (excluding null terminator)
 */
static size_t unescape_text(const char *src, char *dst, size_t dst_len)
{
    if (src == NULL || dst == NULL || dst_len == 0) {
        if (dst != NULL && dst_len > 0) {
            dst[0] = '\0';
        }
        return 0;
    }

    size_t out_pos = 0;

    for (const char *p = src; *p != '\0' && out_pos < dst_len - 1; p++) {
        if (*p == '\\' && *(p + 1) != '\0') {
            p++;
            switch (*p) {
                case 'r':
                    dst[out_pos++] = '\r';
                    break;
                case 'n':
                    dst[out_pos++] = '\n';
                    break;
                case 't':
                    dst[out_pos++] = '\t';
                    break;
                case '\\':
                    dst[out_pos++] = '\\';
                    break;
                case 'x': {
                    int hi = hex_nibble(*(p + 1));
                    int lo = hex_nibble(*(p + 2));
                    if (hi >= 0 && lo >= 0) {
                        dst[out_pos++] = (char)((hi << 4) | lo);
                        p += 2;
                    } else {
                        // Invalid hex escape: treat as literals
                        if (out_pos < dst_len - 3) {
                            dst[out_pos++] = '\\';
                            dst[out_pos++] = 'x';
                        } else {
                            dst[out_pos++] = '\\';
                        }
                    }
                    break;
                }
                default:
                    // Invalid escape: treat backslash + char as literals
                    if (out_pos < dst_len - 2) {
                        dst[out_pos++] = '\\';
                        dst[out_pos++] = *p;
                    } else {
                        dst[out_pos++] = '\\';
                    }
                    break;
            }
        } else {
            dst[out_pos++] = *p;
        }
    }

    dst[out_pos] = '\0';
    return out_pos;
}

/*******************************************************************************
 * Beacon Frame Transmission
 ******************************************************************************/

/**
 * @brief Build and send a beacon frame
 *
 * Reads beacon configuration parameters, builds a UI frame, and sends it
 * through the router. The frame follows the same router rules as any other frame.
 *
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t beacon_send_frame(void)
{
    ax25_frame_t frame;
    char src_str[AX25_MAX_CALLSIGN_LEN + 5];
    char dst_str[AX25_MAX_CALLSIGN_LEN + 5];
    char via_str[256];
    char text_raw[AX25_MAX_INFO_LEN];
    char text_unescaped[AX25_MAX_INFO_LEN];

    // Read configuration
    ax25_cfg_get_str("beacon.source", src_str, sizeof(src_str));
    ax25_cfg_get_str("beacon.destination", dst_str, sizeof(dst_str));
    ax25_cfg_get_str("beacon.via", via_str, sizeof(via_str));
    ax25_cfg_get_str("beacon.text", text_raw, sizeof(text_raw));

    // Validate source
    if (src_str[0] == '\0') {
        ESP_LOGI(TAG, "Beacon disabled (beacon.source empty), skipping transmission");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate destination
    if (dst_str[0] == '\0') {
        ESP_LOGW(TAG, "beacon.destination is empty, skipping transmission");
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize frame
    ax25_frame_init(&frame);
    frame.type = AX25_FRAME_UI;
    frame.control = AX25_CTRL_UI;
    frame.pid = AX25_PID_NONE;
    frame.is_command = true;

    // Parse source address
    if (ax25_address_from_string(src_str, &frame.source) != ESP_OK) {
        ESP_LOGW(TAG, "Invalid beacon.source: %s", src_str);
        return ESP_ERR_INVALID_ARG;
    }

    // Parse destination address
    if (ax25_address_from_string(dst_str, &frame.destination) != ESP_OK) {
        ESP_LOGW(TAG, "Invalid beacon.destination: %s", dst_str);
        return ESP_ERR_INVALID_ARG;
    }

    // Parse digipeater path (via)
    if (via_str[0] != '\0') {
        // Split by comma and parse each digipeater
        char via_copy[256];
        strlcpy(via_copy, via_str, sizeof(via_copy));
        
        char *saveptr = NULL;
        char *token = strtok_r(via_copy, ",", &saveptr);
        uint8_t digi_count = 0;

        while (token != NULL && digi_count < AX25_MAX_DIGIPEATERS) {
            // Trim whitespace
            while (*token == ' ') token++;
            char *end = token + strlen(token) - 1;
            while (end > token && *end == ' ') {
                *end = '\0';
                end--;
            }

            if (*token != '\0') {
                if (ax25_address_from_string(token, &frame.digipeaters[digi_count]) != ESP_OK) {
                    ESP_LOGW(TAG, "Invalid digipeater address: %s", token);
                    // Skip invalid digipeater and continue
                } else {
                    digi_count++;
                }
            }

            token = strtok_r(NULL, ",", &saveptr);
        }

        frame.num_digipeaters = digi_count;
    } else {
        frame.num_digipeaters = 0;
    }

    // Unescape and copy text payload
    size_t text_len = unescape_text(text_raw, text_unescaped, sizeof(text_unescaped));
    if (text_len > AX25_MAX_INFO_LEN) {
        text_len = AX25_MAX_INFO_LEN;
    }
    memcpy(frame.payload, text_unescaped, text_len);
    frame.payload_len = text_len;

    // Send through router (frames follow same router rules)
    esp_err_t err = ax25_router_send(&frame, &s_ctx.source_port);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send beacon frame: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Beacon sent: %s -> %s", src_str, dst_str);
    return ESP_OK;
}

/*******************************************************************************
 * Timer Callback
 ******************************************************************************/

static void beacon_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (s_ctx.worker_task != NULL) {
        xTaskNotifyGive(s_ctx.worker_task);
    }
}

static void beacon_worker_task(void *arg)
{
    (void)arg;

    while (s_ctx.worker_running) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!s_ctx.worker_running) {
            break;
        }

        // Read period from config and reschedule timer
        int period_min = ax25_cfg_get_int("beacon.every");
        if (period_min == 0) {
            ESP_LOGI(TAG, "Beacon disabled (beacon.every=0), stopping timer");
            if (s_ctx.timer != NULL) {
                xTimerStop(s_ctx.timer, 0);
            }
            continue;
        }
        if (period_min < 0) {
            period_min = 5;  // Default to 5 minutes if invalid
        }
        if (period_min > 10080) {
            period_min = 10080;  // Cap at 7 days
        }

        uint32_t period_ms = (uint32_t)period_min * 60 * 1000U;

        if (!beacon_source_is_configured()) {
            ESP_LOGD(TAG, "Beacon disabled (beacon.source empty), skipping timer send");
            if (s_ctx.timer != NULL) {
                xTimerChangePeriod(s_ctx.timer, pdMS_TO_TICKS(period_ms), 0);
            }
            continue;
        }

        // Send beacon
        beacon_send_frame();

        // Reschedule timer with potentially new period
        if (s_ctx.timer != NULL) {
            xTimerChangePeriod(s_ctx.timer, pdMS_TO_TICKS(period_ms), 0);
        }
    }

    vTaskDelete(NULL);
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

esp_err_t ax25_beacon_init(void)
{
    if (s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Check if config module is initialized
    if (!ax25_cfg_is_initialized()) {
        ESP_LOGE(TAG, "ax25_config not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read initial period from config
    int period_min = ax25_cfg_get_int("beacon.every");

    memset(&s_ctx.source_port, 0, sizeof(s_ctx.source_port));
    s_ctx.source_port.mode = AX25_PORT_DYNAMIC;
    s_ctx.source_port.on_tx_frame = beacon_source_port_on_frame;
    s_ctx.source_port.user_data = NULL;
    esp_err_t port_err = ax25_router_register_port(&s_ctx.source_port);
    if (port_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register beacon source port: %s", esp_err_to_name(port_err));
        return port_err;
    }
    s_ctx.source_port_registered = true;

    if (period_min == 0) {
        s_ctx.initialized = true;
        ESP_LOGI(TAG, "Beacon module initialized (disabled: beacon.every=0)");
        return ESP_OK;
    }
    if (period_min < 0) {
        period_min = 5;  // Default to 5 minutes if invalid
    }
    if (period_min > 10080) {
        period_min = 10080;  // Cap at 7 days
    }

    if (!beacon_source_is_configured()) {
        ESP_LOGI(TAG, "Beacon source is empty; beacon sends are disabled until configured");
    }

    s_ctx.worker_running = true;

    s_ctx.worker_task = xTaskCreateStatic(beacon_worker_task,
                                          "ax25_beacon_w",
                                          AX25_BEACON_WORKER_STACK_BYTES / sizeof(StackType_t),
                                          NULL,
                                          AX25_BEACON_WORKER_PRIORITY,
                                          s_ctx.worker_task_stack,
                                          &s_ctx.worker_task_tcb);
    if (s_ctx.worker_task == NULL) {
        s_ctx.worker_running = false;
        if (s_ctx.source_port_registered) {
            ax25_router_remove_port(&s_ctx.source_port);
            s_ctx.source_port_registered = false;
        }
        ESP_LOGE(TAG, "Failed to create beacon worker task");
        return ESP_ERR_NO_MEM;
    }

    uint32_t period_ms = (uint32_t)period_min * 60 * 1000U;

    // Create FreeRTOS timer
    s_ctx.timer = xTimerCreate(
        "ax25_beacon",
        pdMS_TO_TICKS(period_ms),
        pdTRUE,  // Auto-reload
        NULL,    // Reserved
        beacon_timer_cb
    );

    if (s_ctx.timer == NULL) {
        ESP_LOGE(TAG, "Failed to create beacon timer");
        s_ctx.worker_running = false;
        if (s_ctx.worker_task != NULL) {
            xTaskNotifyGive(s_ctx.worker_task);
            vTaskDelay(pdMS_TO_TICKS(10));
            s_ctx.worker_task = NULL;
        }
        if (s_ctx.source_port_registered) {
            ax25_router_remove_port(&s_ctx.source_port);
            s_ctx.source_port_registered = false;
        }
        return ESP_ERR_NO_MEM;
    }

    // Start timer
    if (xTimerStart(s_ctx.timer, 0) != pdPASS) {
        xTimerDelete(s_ctx.timer, 0);
        s_ctx.timer = NULL;
        s_ctx.worker_running = false;
        if (s_ctx.worker_task != NULL) {
            xTaskNotifyGive(s_ctx.worker_task);
            vTaskDelay(pdMS_TO_TICKS(10));
            s_ctx.worker_task = NULL;
        }
        if (s_ctx.source_port_registered) {
            ax25_router_remove_port(&s_ctx.source_port);
            s_ctx.source_port_registered = false;
        }
        ESP_LOGE(TAG, "Failed to start beacon timer");
        return ESP_FAIL;
    }

    s_ctx.initialized = true;
    ESP_LOGI(TAG, "Beacon module initialized (period: %d min)", period_min);
    return ESP_OK;
}

void ax25_beacon_deinit(void)
{
    if (s_ctx.timer != NULL) {
        xTimerStop(s_ctx.timer, 0);
        xTimerDelete(s_ctx.timer, 0);
        s_ctx.timer = NULL;
    }

    if (s_ctx.worker_task != NULL) {
        s_ctx.worker_running = false;
        xTaskNotifyGive(s_ctx.worker_task);
        /* Give worker task a chance to observe stop flag and exit. */
        vTaskDelay(pdMS_TO_TICKS(10));
        s_ctx.worker_task = NULL;
    }


    if (s_ctx.source_port_registered) {
        ax25_router_remove_port(&s_ctx.source_port);
        s_ctx.source_port_registered = false;
    }

    s_ctx.initialized = false;
    ESP_LOGI(TAG, "Beacon module deinitialized");
}

bool ax25_beacon_is_initialized(void)
{
    return s_ctx.initialized;
}

esp_err_t ax25_beacon_trigger(void)
{
    if (!s_ctx.initialized) {
        ESP_LOGW(TAG, "Beacon module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ax25_cfg_get_int("beacon.every") == 0) {
        ESP_LOGI(TAG, "Beacon disabled (beacon.every=0), skipping manual trigger");
        return ESP_ERR_INVALID_STATE;
    }

    if (!beacon_source_is_configured()) {
        ESP_LOGI(TAG, "Beacon disabled (beacon.source empty), skipping manual trigger");
        return ESP_ERR_INVALID_STATE;
    }

    return beacon_send_frame();
}
