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
 * @file ax25_phy_kiss_bt.h
 * @brief Bluetooth Classic SPP KISS physical-layer driver
 *
 * Implements KISS protocol over Bluetooth Classic SPP (Serial Port Profile),
 * allowing a PC to connect via Bluetooth as a virtual COM port.
 *
 * Features:
 * - Bluetooth Classic SPP for KISS TNC communication
 * - Button-activated pairing mode (hold for 5 seconds)
 * - LED status indicator (blinking=pairing, solid=connected)
 * - Configurable PIN authentication (optional)
 *
 * Typical usage
 * -------------
 * @code
 *   static ax25_phy_kiss_bt_t phy;
 *
 *   static void on_rx_frame(const ax25_frame_t *frame, void *user_data) {
 *       ax25_router_send(frame, (ax25_router_port_t *)user_data);
 *   }
 *
 *   const ax25_phy_kiss_bt_config_t cfg = {
 *       .device_name        = "ESP-AX25 KISS TNC",
 *       .button_gpio        = GPIO_NUM_7,
 *       .led_gpio           = GPIO_NUM_3,
 *       .require_pin        = false,
 *       .pin_code           = "1234",
 *       .on_rx_frame           = on_rx_frame,
 *       .user_data          = &phy_port,
 *   };
 *
 *   ax25_phy_kiss_bt_init(&cfg, &phy);
 *
 *   // Send a frame:
 *   ax25_phy_kiss_bt_send(&frame, &phy);
 *
 *   // Shutdown:
 *   ax25_phy_kiss_bt_deinit(&phy);
 * @endcode
 */

#ifndef AX25_PHY_KISS_BT_H
#define AX25_PHY_KISS_BT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "ax25_kiss.h"
#include "ax25_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/** Maximum length of Bluetooth device name */
#define AX25_PHY_BT_MAX_NAME_LEN    32

/** Maximum length of PIN code */
#define AX25_PHY_BT_MAX_PIN_LEN     16

/** Button hold time for pairing mode (milliseconds) */
#define AX25_PHY_BT_BUTTON_HOLD_MS  5000

/** LED blink period in pairing mode (milliseconds) */
#define AX25_PHY_BT_LED_BLINK_MS    1000

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * @brief Connection state for Bluetooth SPP
 */
typedef enum {
    AX25_PHY_BT_STATE_IDLE,         /**< Not initialized or disconnected */
    AX25_PHY_BT_STATE_PAIRING,      /**< In discoverable/pairing mode */
    AX25_PHY_BT_STATE_CONNECTED,    /**< SPP connection established */
} ax25_phy_kiss_bt_state_t;

/**
 * @brief Configuration parameters for ax25_phy_kiss_bt_init()
 *
 * Zero-valued optional fields fall back to the defaults noted in the field
 * comments.
 */
typedef struct {
    /** Bluetooth device name (max 32 chars). NULL → "ESP-AX25 KISS TNC" */
    const char  *device_name;

    /** GPIO pin for pairing button. 0 → GPIO_NUM_7 */
    gpio_num_t   button_gpio;

    /** GPIO pin for status LED. 0 → GPIO_NUM_3 */
    gpio_num_t   led_gpio;

    /** Require PIN for Bluetooth pairing. false → open connection */
    bool         require_pin;

    /** PIN code for pairing (if require_pin is true). NULL → "1234" */
    const char  *pin_code;

    /** RX task stack depth in bytes. 0 → 4096 */
    uint32_t     rx_task_stack_size;

    /** RX task FreeRTOS priority. 0 → 5 */
    UBaseType_t  rx_task_priority;

    /** Called for each received frame. Must not be NULL. */
    ax25_on_frame_t  on_rx_frame;

    /** Passed as-is to @p on_rx_frame. */
    void         *user_data;
} ax25_phy_kiss_bt_config_t;

/*******************************************************************************
 * Context
 ******************************************************************************/

/**
 * @brief Runtime context for a Bluetooth SPP KISS physical-layer instance
 *
 * Allocate as a static or long-lived variable and pass its address to
 * ax25_phy_kiss_bt_init(). The structure must remain valid (not moved or
 * freed) until ax25_phy_kiss_bt_deinit() returns.
 *
 * All fields are managed by the driver; treat as opaque.
 */
typedef struct {
    char                    device_name[AX25_PHY_BT_MAX_NAME_LEN + 1];
    char                    pin_code[AX25_PHY_BT_MAX_PIN_LEN + 1];
    gpio_num_t              button_gpio;
    gpio_num_t              led_gpio;
    bool                    require_pin;
    ax25_kiss_decoder_t     kiss_decoder;
    volatile bool           running;
    ax25_phy_kiss_bt_state_t state;
    uint32_t                spp_handle;
    TaskHandle_t            rx_task_handle;
    TaskHandle_t            button_task_handle;
    TimerHandle_t           led_timer_handle;
    bool                    led_state;
    ax25_on_frame_t         on_rx_frame;
    void                    *user_data;
} ax25_phy_kiss_bt_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Initialize the Bluetooth SPP KISS driver
 *
 * Initializes Bluetooth Classic, configures SPP, sets up GPIO for button
 * and LED, initializes the KISS decoder, and spawns tasks for RX and
 * button monitoring.
 *
 * @param config  Bluetooth settings, GPIO pins, and receive callback
 * @param ctx     Caller-allocated context (must remain valid until deinit)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if @p ctx, @p config, or @p config->on_rx_frame is NULL
 * @return ESP_ERR_NO_MEM if tasks could not be created
 * @return Any ESP_ERR_* returned by Bluetooth initialization
 */
esp_err_t ax25_phy_kiss_bt_init(const ax25_phy_kiss_bt_config_t *config, ax25_phy_kiss_bt_t *ctx);

/**
 * @brief Stop tasks and release Bluetooth resources
 *
 * Signals tasks to exit, waits briefly for them to do so, then shuts down
 * Bluetooth. Safe to call on a zero-initialized or already-deinitialized
 * context.
 *
 * @param ctx  Context previously passed to ax25_phy_kiss_bt_init()
 */
void ax25_phy_kiss_bt_deinit(ax25_phy_kiss_bt_t *ctx);

/**
 * @brief Send @p frame via Bluetooth SPP
 *
 * Builds the AX.25 byte string, wraps it in a KISS frame, and writes it
 * to the connected SPP client.
 *
 * @param frame  AX.25 frame to transmit
 * @param ctx    Initialized Bluetooth SPP KISS context
 * @return ESP_OK if the frame was successfully written
 * @return ESP_ERR_INVALID_ARG if @p ctx or @p frame is NULL
 * @return ESP_ERR_INVALID_STATE if no SPP connection is active
 * @return ESP_FAIL if frame build/encode/write fails
 */
esp_err_t ax25_phy_kiss_bt_send(const ax25_frame_t *frame, ax25_phy_kiss_bt_t *ctx);

/**
 * @brief Get current connection state
 *
 * @param ctx  Initialized Bluetooth SPP KISS context
 * @return Current connection state
 */
ax25_phy_kiss_bt_state_t ax25_phy_kiss_bt_get_state(const ax25_phy_kiss_bt_t *ctx);

/**
 * @brief Manually enter pairing/discoverable mode
 *
 * Makes the device discoverable for Bluetooth pairing. Normally triggered
 * by holding the button for 5 seconds.
 *
 * @param ctx  Initialized Bluetooth SPP KISS context
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if @p ctx is NULL
 * @return ESP_ERR_INVALID_STATE if already in pairing mode
 */
esp_err_t ax25_phy_kiss_bt_start_pairing(ax25_phy_kiss_bt_t *ctx);

/**
 * @brief Exit pairing/discoverable mode
 *
 * Makes the device non-discoverable.
 *
 * @param ctx  Initialized Bluetooth SPP KISS context
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if @p ctx is NULL
 */
esp_err_t ax25_phy_kiss_bt_stop_pairing(ax25_phy_kiss_bt_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* AX25_PHY_KISS_BT_H */
