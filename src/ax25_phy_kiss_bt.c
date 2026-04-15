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
 * @file ax25_phy_kiss_bt.c
 * @brief Bluetooth Classic SPP KISS physical-layer driver
 *
 * Implements KISS protocol over Bluetooth Classic SPP (Serial Port Profile).
 */

#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "ax25_buffer.h"
#include "ax25_frame.h"
#include "ax25_print.h"
#include "ax25_phy_kiss_bt.h"

static const char *TAG = "AX25_PHY_BT";

/* -------------------------------------------------------------------------
 * Configuration from Kconfig
 * ---------------------------------------------------------------------- */

#ifdef CONFIG_AX25_PHY_BT_LOG_FRAMES
#define AX25_PHY_BT_LOG_FRAMES CONFIG_AX25_PHY_BT_LOG_FRAMES
#else
#define AX25_PHY_BT_LOG_FRAMES 0
#endif

#ifdef CONFIG_AX25_PHY_BT_WRITE_RETRIES
#define AX25_PHY_BT_WRITE_RETRIES CONFIG_AX25_PHY_BT_WRITE_RETRIES
#else
#define AX25_PHY_BT_WRITE_RETRIES 3
#endif

#ifdef CONFIG_AX25_PHY_BT_WRITE_BACKOFF_MS
#define AX25_PHY_BT_WRITE_BACKOFF_MS CONFIG_AX25_PHY_BT_WRITE_BACKOFF_MS
#else
#define AX25_PHY_BT_WRITE_BACKOFF_MS 10
#endif

/* -------------------------------------------------------------------------
 * SPP Server Configuration
 * ---------------------------------------------------------------------- */

#define SPP_SERVER_NAME "ESP_AX25_SPP_SERVER"
#define SPP_SERVICE_NAME "ESP_AX25_SPP"

/* Global context pointer for callbacks (ESP-IDF SPP uses global callbacks) */
static ax25_phy_kiss_bt_t *g_bt_ctx = NULL;

/* -------------------------------------------------------------------------
 * Private helpers
 * ---------------------------------------------------------------------- */

/**
 * Called by the KISS decoder for each complete KISS frame.
 * Parses the AX.25 content and delivers it to the user callback.
 */
static void kiss_frame_cb(uint8_t port_num, uint8_t command, const uint8_t *data, size_t len, void *user_data)
{
    if (command != 0) {
        return; /* ignore non-data commands */
    }

    ax25_frame_t frame;
    if (ax25_frame_parse(data, len, &frame) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to parse incoming frame (%zu bytes)", len);
        return;
    }

#if AX25_PHY_BT_LOG_FRAMES
    ax25_print_frame("bt_recv", &frame);
#endif

    ax25_phy_kiss_bt_t *phy = (ax25_phy_kiss_bt_t *)user_data;
    if (phy && phy->on_rx_frame) {
        phy->on_rx_frame(&frame, phy->user_data);
    }
}

/**
 * Update LED state based on connection status.
 */
static void update_led(ax25_phy_kiss_bt_t *ctx)
{
    if (ctx->led_gpio == GPIO_NUM_NC) {
        return;
    }

    switch (ctx->state) {
        case AX25_PHY_BT_STATE_CONNECTED:
            /* Solid on when connected */
            gpio_set_level(ctx->led_gpio, 1);
            if (ctx->led_timer_handle) {
                xTimerStop(ctx->led_timer_handle, 0);
            }
            break;

        case AX25_PHY_BT_STATE_PAIRING:
            /* Blinking handled by timer */
            if (ctx->led_timer_handle) {
                xTimerStart(ctx->led_timer_handle, 0);
            }
            break;

        case AX25_PHY_BT_STATE_IDLE:
        default:
            /* Off when idle */
            gpio_set_level(ctx->led_gpio, 0);
            if (ctx->led_timer_handle) {
                xTimerStop(ctx->led_timer_handle, 0);
            }
            break;
    }
}

/**
 * LED timer callback for blinking in pairing mode.
 */
static void led_timer_callback(TimerHandle_t xTimer)
{
    ax25_phy_kiss_bt_t *ctx = (ax25_phy_kiss_bt_t *)pvTimerGetTimerID(xTimer);
    if (ctx && ctx->state == AX25_PHY_BT_STATE_PAIRING) {
        ctx->led_state = !ctx->led_state;
        gpio_set_level(ctx->led_gpio, ctx->led_state ? 1 : 0);
    }
}

/**
 * GAP callback for Bluetooth events.
 */
static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    ax25_phy_kiss_bt_t *ctx = g_bt_ctx;

    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
            } else {
                ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
            }
            break;

        case ESP_BT_GAP_PIN_REQ_EVT:
            ESP_LOGI(TAG, "PIN request");
            if (ctx && ctx->require_pin) {
                esp_bt_pin_code_t pin;
                memset(pin, 0, sizeof(pin));
                int pin_len = strlen(ctx->pin_code);
                if (pin_len > ESP_BT_PIN_CODE_LEN) {
                    pin_len = ESP_BT_PIN_CODE_LEN;
                }
                memcpy(pin, ctx->pin_code, pin_len);
                esp_bt_gap_pin_reply(param->pin_req.bda, true, pin_len, pin);
            }
            break;

#if (CONFIG_EXAMPLE_SSP_ENABLED == true) || defined(CONFIG_BT_SSP_ENABLED)
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(TAG, "SSP confirm request for %06lu", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;

        case ESP_BT_GAP_KEY_NOTIF_EVT:
            ESP_LOGI(TAG, "SSP passkey notify: %06lu", param->key_notif.passkey);
            break;

        case ESP_BT_GAP_KEY_REQ_EVT:
            ESP_LOGI(TAG, "SSP passkey request");
            break;
#endif

        case ESP_BT_GAP_MODE_CHG_EVT:
            ESP_LOGD(TAG, "Mode change: %d", param->mode_chg.mode);
            break;

        default:
            ESP_LOGD(TAG, "GAP event: %d", event);
            break;
    }
}

/**
 * SPP callback for connection and data events.
 */
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    ax25_phy_kiss_bt_t *ctx = g_bt_ctx;

    switch (event) {
        case ESP_SPP_INIT_EVT:
            if (param->init.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAG, "SPP initialized");
                /* Set device name */
                esp_bt_dev_set_device_name(ctx ? ctx->device_name : "ESP-AX25 KISS TNC");
                /* Connectable but not discoverable: previously-paired remotes
                 * can reconnect without a button press; new pairings still
                 * require holding the button to enter discoverable mode. */
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "SPP init failed: %d", param->init.status);
            }
            break;

        case ESP_SPP_DISCOVERY_COMP_EVT:
            ESP_LOGD(TAG, "SPP discovery complete");
            break;

        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(TAG, "SPP client connected");
            break;

        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "SPP connection closed");
            if (ctx && ctx->spp_handle == param->close.handle) {
                ctx->spp_handle = 0;
                ctx->state = AX25_PHY_BT_STATE_IDLE;
                update_led(ctx);
                /* Restore page scan so already-paired remotes can reconnect.
                 * NON_CONNECTABLE is set in SRV_OPEN_EVT to block new scans
                 * during a session; we must undo that here or the Windows
                 * COM port open will fail on every subsequent attempt. */
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            }
            break;

        case ESP_SPP_START_EVT:
            if (param->start.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAG, "SPP server started, handle: %lu", (unsigned long)param->start.handle);
            } else {
                ESP_LOGE(TAG, "SPP server start failed: %d", param->start.status);
            }
            break;

        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGD(TAG, "SPP client init");
            break;

        case ESP_SPP_DATA_IND_EVT:
            /* Data received from connected client */
            if (ctx && param->data_ind.len > 0) {
                ESP_LOGD(TAG, "SPP data received: %d bytes", param->data_ind.len);
                ax25_kiss_decoder_process_bytes(&ctx->kiss_decoder,
                                                 param->data_ind.data,
                                                 param->data_ind.len);
            }
            break;

        case ESP_SPP_CONG_EVT:
            ESP_LOGD(TAG, "SPP congestion: %s", param->cong.cong ? "congested" : "clear");
            break;

        case ESP_SPP_WRITE_EVT:
            ESP_LOGD(TAG, "SPP write complete: %d bytes, status: %d",
                     param->write.len, param->write.status);
            break;

        case ESP_SPP_SRV_OPEN_EVT:
            /* Server connection opened (client connected to us) */
            ESP_LOGI(TAG, "SPP server connection opened, handle: %lu",
                     (unsigned long)param->srv_open.handle);
            if (ctx) {
                ctx->spp_handle = param->srv_open.handle;
                ctx->state = AX25_PHY_BT_STATE_CONNECTED;
                /* Stop discoverable mode once connected */
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                update_led(ctx);
            }
            break;

        case ESP_SPP_SRV_STOP_EVT:
            ESP_LOGI(TAG, "SPP server stopped");
            break;

        case ESP_SPP_UNINIT_EVT:
            ESP_LOGI(TAG, "SPP uninitialized");
            break;

        default:
            ESP_LOGD(TAG, "SPP event: %d", event);
            break;
    }
}

/**
 * Button monitoring task.
 * Detects 5-second hold to enter pairing mode.
 */
static void button_task(void *arg)
{
    ax25_phy_kiss_bt_t *ctx = (ax25_phy_kiss_bt_t *)arg;
    TickType_t press_start = 0;
    bool was_pressed = false;

    while (ctx->running) {
        int level = gpio_get_level(ctx->button_gpio);
        bool is_pressed = (level == 0); /* Active low button */

        if (is_pressed && !was_pressed) {
            /* Button just pressed */
            press_start = xTaskGetTickCount();
        } else if (is_pressed && was_pressed) {
            /* Button held */
            TickType_t held_time = (xTaskGetTickCount() - press_start) * portTICK_PERIOD_MS;
            if (held_time >= AX25_PHY_BT_BUTTON_HOLD_MS) {
                /* Enter pairing mode */
                if (ctx->state != AX25_PHY_BT_STATE_PAIRING &&
                    ctx->state != AX25_PHY_BT_STATE_CONNECTED) {
                    ESP_LOGI(TAG, "Button held for 5 seconds - entering pairing mode");
                    ax25_phy_kiss_bt_start_pairing(ctx);
                    /* Reset press tracking to avoid repeated triggers */
                    press_start = xTaskGetTickCount();
                }
            }
        } else if (!is_pressed && was_pressed) {
            /* Button released */
            press_start = 0;
        }

        was_pressed = is_pressed;
        vTaskDelay(pdMS_TO_TICKS(50)); /* Poll every 50ms */
    }

    ctx->button_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * Initialize GPIO for button and LED.
 */
static esp_err_t init_gpio(ax25_phy_kiss_bt_t *ctx)
{
    esp_err_t err;

    /* Configure button GPIO (input with pull-up) */
    if (ctx->button_gpio != GPIO_NUM_NC) {
        gpio_config_t btn_config = {
            .pin_bit_mask = (1ULL << ctx->button_gpio),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&btn_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure button GPIO: %s", esp_err_to_name(err));
            return err;
        }
    }

    /* Configure LED GPIO (output) */
    if (ctx->led_gpio != GPIO_NUM_NC) {
        gpio_config_t led_config = {
            .pin_bit_mask = (1ULL << ctx->led_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&led_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LED GPIO: %s", esp_err_to_name(err));
            return err;
        }
        gpio_set_level(ctx->led_gpio, 0); /* Start with LED off */
    }

    return ESP_OK;
}

/**
 * Initialize Bluetooth controller and Bluedroid stack.
 */
static esp_err_t init_bluetooth(ax25_phy_kiss_bt_t *ctx)
{
    esp_err_t err;

    /* Release BLE memory when both BT+BLE are compiled in (BTDM mode).
     * In BR_EDR_ONLY mode BLE memory was never allocated; calling
     * mem_release for it would corrupt memory. */
#if CONFIG_BTDM_CTRL_MODE_BTDM
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
#endif

    /* Initialize BT controller */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(err));
        esp_bt_controller_deinit();
        return err;
    }

    /* Initialize Bluedroid stack */
    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(err));
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return err;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(err));
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return err;
    }

    /* Register GAP callback */
    err = esp_bt_gap_register_callback(esp_bt_gap_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Register SPP callback */
    err = esp_spp_register_callback(esp_spp_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPP callback register failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Configure SPP */
    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0, /* Use default */
    };
    err = esp_spp_enhanced_init(&spp_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPP init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Set PIN code security */
    if (ctx->require_pin) {
        esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
        esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE; /* No I/O capability, use PIN */
        esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

        esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
        esp_bt_pin_code_t pin_code;
        memset(pin_code, 0, sizeof(pin_code));
        int pin_len = strlen(ctx->pin_code);
        if (pin_len > ESP_BT_PIN_CODE_LEN) {
            pin_len = ESP_BT_PIN_CODE_LEN;
        }
        memcpy(pin_code, ctx->pin_code, pin_len);
        esp_bt_gap_set_pin(pin_type, pin_len, pin_code);
        ESP_LOGI(TAG, "PIN authentication enabled");
    } else {
        /* Open connection - no PIN required (SSP with just works) */
        esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
        esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
        esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

        esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
        esp_bt_gap_set_pin(pin_type, 0, NULL);
        ESP_LOGI(TAG, "Open connection mode (no PIN required)");
    }

    /* Start SPP server */
    esp_spp_sec_t sec_mask = ctx->require_pin ?
        (ESP_SPP_SEC_AUTHENTICATE | ESP_SPP_SEC_ENCRYPT) : ESP_SPP_SEC_NONE;
    esp_spp_role_t role = ESP_SPP_ROLE_SLAVE;

    err = esp_spp_start_srv(sec_mask, role, 0, SPP_SERVER_NAME);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPP server start failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Bluetooth initialized, device name: %s", ctx->device_name);
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t ax25_phy_kiss_bt_init(const ax25_phy_kiss_bt_config_t *config, ax25_phy_kiss_bt_t *ctx)
{
    if (!ctx || !config || !config->on_rx_frame) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize context */
    memset(ctx, 0, sizeof(ax25_phy_kiss_bt_t));

    /* Copy device name */
    const char *name = config->device_name;
    if (!name || strlen(name) == 0) {
#ifdef CONFIG_AX25_PHY_BT_DEVICE_NAME
        name = CONFIG_AX25_PHY_BT_DEVICE_NAME;
#else
        name = "ESP-AX25 KISS TNC";
#endif
    }
    strncpy(ctx->device_name, name, AX25_PHY_BT_MAX_NAME_LEN);
    ctx->device_name[AX25_PHY_BT_MAX_NAME_LEN] = '\0';

    /* Configure GPIO pins with defaults from Kconfig or function args */
#ifdef CONFIG_AX25_PHY_BT_BUTTON_GPIO
    ctx->button_gpio = config->button_gpio ? config->button_gpio : CONFIG_AX25_PHY_BT_BUTTON_GPIO;
#else
    ctx->button_gpio = config->button_gpio ? config->button_gpio : GPIO_NUM_7;
#endif

#ifdef CONFIG_AX25_PHY_BT_LED_GPIO
    ctx->led_gpio = config->led_gpio ? config->led_gpio : CONFIG_AX25_PHY_BT_LED_GPIO;
#else
    ctx->led_gpio = config->led_gpio ? config->led_gpio : GPIO_NUM_3;
#endif

    /* Configure PIN settings */
#ifdef CONFIG_AX25_PHY_BT_REQUIRE_PIN
    ctx->require_pin = config->require_pin || CONFIG_AX25_PHY_BT_REQUIRE_PIN;
#else
    ctx->require_pin = config->require_pin;
#endif

    const char *pin = config->pin_code;
    if (!pin || strlen(pin) == 0) {
#ifdef CONFIG_AX25_PHY_BT_PIN_CODE
        pin = CONFIG_AX25_PHY_BT_PIN_CODE;
#else
        pin = "1234";
#endif
    }
    strncpy(ctx->pin_code, pin, AX25_PHY_BT_MAX_PIN_LEN);
    ctx->pin_code[AX25_PHY_BT_MAX_PIN_LEN] = '\0';

    ctx->on_rx_frame = config->on_rx_frame;
    ctx->user_data = config->user_data;
    ctx->state = AX25_PHY_BT_STATE_IDLE;
    ctx->spp_handle = 0;
    ctx->running = false;

    /* Set global context for callbacks */
    g_bt_ctx = ctx;

    /* Initialize KISS decoder */
    ax25_kiss_decoder_init(&ctx->kiss_decoder, kiss_frame_cb, ctx);

    /* Initialize GPIO */
    esp_err_t err = init_gpio(ctx);
    if (err != ESP_OK) {
        return err;
    }

    /* Create LED timer */
    ctx->led_timer_handle = xTimerCreate("led_timer",
                                          pdMS_TO_TICKS(AX25_PHY_BT_LED_BLINK_MS),
                                          pdTRUE, /* Auto-reload */
                                          ctx,
                                          led_timer_callback);
    if (!ctx->led_timer_handle) {
        ESP_LOGE(TAG, "Failed to create LED timer");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize Bluetooth */
    err = init_bluetooth(ctx);
    if (err != ESP_OK) {
        xTimerDelete(ctx->led_timer_handle, 0);
        return err;
    }

    ctx->running = true;

    /* Start button monitoring task */
    uint32_t stack_size = config->rx_task_stack_size ? config->rx_task_stack_size : 4096;
    UBaseType_t priority = config->rx_task_priority ? config->rx_task_priority : 5;

    if (ctx->button_gpio != GPIO_NUM_NC) {
        if (xTaskCreate(button_task, "bt_button_task", stack_size, ctx, priority,
                        &ctx->button_task_handle) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create button task");
            ctx->running = false;
            esp_spp_deinit();
            esp_bluedroid_disable();
            esp_bluedroid_deinit();
            esp_bt_controller_disable();
            esp_bt_controller_deinit();
            xTimerDelete(ctx->led_timer_handle, 0);
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_LOGI(TAG, "Bluetooth SPP KISS driver initialized");
    ESP_LOGI(TAG, "Button GPIO: %d, LED GPIO: %d", ctx->button_gpio, ctx->led_gpio);
    ESP_LOGI(TAG, "Hold button for 5 seconds to enter pairing mode");

    return ESP_OK;
}

void ax25_phy_kiss_bt_deinit(ax25_phy_kiss_bt_t *ctx)
{
    if (!ctx || !ctx->running) {
        return;
    }

    ctx->running = false;

    /* Wait for tasks to exit */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Stop and delete LED timer */
    if (ctx->led_timer_handle) {
        xTimerStop(ctx->led_timer_handle, 0);
        xTimerDelete(ctx->led_timer_handle, 0);
        ctx->led_timer_handle = NULL;
    }

    /* Turn off LED */
    if (ctx->led_gpio != GPIO_NUM_NC) {
        gpio_set_level(ctx->led_gpio, 0);
    }

    /* Shutdown Bluetooth */
    esp_spp_deinit();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    g_bt_ctx = NULL;

    ESP_LOGI(TAG, "Bluetooth SPP KISS driver deinitialized");
}

esp_err_t ax25_phy_kiss_bt_send(const ax25_frame_t *frame, ax25_phy_kiss_bt_t *ctx)
{
    if (!ctx || !frame) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ctx->state != AX25_PHY_BT_STATE_CONNECTED || ctx->spp_handle == 0) {
        ESP_LOGW(TAG, "Cannot send: no SPP connection");
        return ESP_ERR_INVALID_STATE;
    }

#if AX25_PHY_BT_LOG_FRAMES
    ax25_print_frame("bt_send", frame);
#endif

    /* Build raw AX.25 frame */
    ax25_buffer_t raw = {0};
    if (ax25_frame_build(frame, &raw) != ESP_OK) {
        ESP_LOGE(TAG, "ax25_frame_build failed");
        return ESP_FAIL;
    }

    /* KISS encode */
    uint8_t encoded[AX25_KISS_MAX_ENCODED_SIZE];
    size_t encoded_len = ax25_kiss_encode(0, 0, raw.data, raw.len, encoded, sizeof(encoded));
    if (encoded_len == 0) {
        ESP_LOGE(TAG, "ax25_kiss_encode failed");
        return ESP_FAIL;
    }

    /* Send via SPP */
    esp_err_t err = esp_spp_write(ctx->spp_handle, encoded_len, encoded);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_spp_write failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    return ESP_OK;
}

ax25_phy_kiss_bt_state_t ax25_phy_kiss_bt_get_state(const ax25_phy_kiss_bt_t *ctx)
{
    if (!ctx) {
        return AX25_PHY_BT_STATE_IDLE;
    }
    return ctx->state;
}

esp_err_t ax25_phy_kiss_bt_start_pairing(ax25_phy_kiss_bt_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ctx->state == AX25_PHY_BT_STATE_CONNECTED) {
        ESP_LOGW(TAG, "Already connected, disconnect first to enter pairing mode");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Entering pairing/discoverable mode for device: %s", ctx->device_name);

    /* Make device discoverable and connectable */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    ctx->state = AX25_PHY_BT_STATE_PAIRING;
    update_led(ctx);

    return ESP_OK;
}

esp_err_t ax25_phy_kiss_bt_stop_pairing(ax25_phy_kiss_bt_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ctx->state == AX25_PHY_BT_STATE_CONNECTED) {
        /* Don't change if connected */
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Exiting pairing mode");

    /* Make device non-discoverable */
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    ctx->state = AX25_PHY_BT_STATE_IDLE;
    update_led(ctx);

    return ESP_OK;
}
