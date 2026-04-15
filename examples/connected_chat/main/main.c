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
 * @file main.c
 * @brief Connected Chat Example — connection-oriented AX.25 interactive console
 *
 * Demonstrates ax25_conn for both outgoing and incoming connections.
 * An interactive console accepts commands while the ax25_conn state machine
 * runs entirely through callbacks.
 *
 * Architecture
 * ============
 * app_main sets up the router, PHY, ax25_conn, and the REPL, then
 * returns.  Console commands call the ax25_conn API directly from the
 * REPL task.  Received data arrives via on_data and is printed inline.
 *
 * Incoming connections are handled automatically: when ax25_conn is in
 * DISCONNECTED state and receives a SABM, it accepts it and calls
 * on_connect with is_local_initiated=false.
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_console.h"

#include "ax25_conn.h"
#include "ax25_address.h"
#include "ax25_frame.h"
#include "ax25_router.h"
#include "ax25_phy_kiss_uart.h"
#include "nvs_flash.h"
#include "ax25_config.h"

static const char *TAG = "CONNECTED_CHAT";

/*******************************************************************************
 * Application context
 ******************************************************************************/

typedef struct {
    ax25_conn_t        conn;
    ax25_router_port_t app_port;
    ax25_address_t     local_addr;
} chat_ctx_t;

/*******************************************************************************
 * PHY callbacks
 ******************************************************************************/

/* Frame received from the TNC: inject into the router from phy_port. */
static void phy_frame_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_router_send(frame, (ax25_router_port_t *)user_data);
}

/* Router delivers a frame to the PHY port: write it to the TNC. */
static void phy_port_output_cb(const ax25_frame_t *frame, void *user_data)
{
    ax25_phy_kiss_uart_send(frame, (ax25_phy_kiss_uart_t *)user_data);
}

/*******************************************************************************
 * App router port -> conn
 ******************************************************************************/

/**
 * The router delivers an ax25_frame_t to our app port.  Pass it directly
 * to ax25_conn for protocol processing.
 */
static void app_port_on_frame(const ax25_frame_t *frame, void *user_data)
{
    ax25_conn_t *conn = (ax25_conn_t *)user_data;
    ax25_conn_on_frame(conn, frame);
}

/*******************************************************************************
 * ax25_conn callbacks
 ******************************************************************************/

static void on_connect(ax25_address_t remote_addr, bool is_local_initiated,
                       void *user_data)
{
    char remote_str[12];
    ax25_address_to_string(&remote_addr, remote_str, sizeof(remote_str));
    if (is_local_initiated) {
        printf("\n*** Connected to %s ***\n", remote_str);
    } else {
        printf("\n*** Incoming connection from %s ***\n", remote_str);
    }
    printf("Commands: send <msg>   disconnect   status\n\n");
}

static void on_disconnect(void *user_data)
{
    printf("\n*** Disconnected ***\n");
}

static void on_error(const ax25_conn_error_t *error, void *user_data)
{
    printf("\n*** AX.25 error: %s (code %d) ***\n",
           error->message ? error->message : "", error->code);
}

static void on_data(const uint8_t *data, size_t len, void *user_data)
{
    printf("[REMOTE]: %.*s\n", (int)len, (const char *)data);
}

/**
 * ax25_conn wants to transmit a frame: route it through the router,
 * using app_port as the source so the frame is not looped back.
 */
static void on_tx_frame(const ax25_frame_t *frame, void *user_data)
{
    chat_ctx_t *ctx = (chat_ctx_t *)user_data;
    ax25_router_send(frame, &ctx->app_port);
}

/*******************************************************************************
 * Helper
 ******************************************************************************/

static const char *state_str(ax25_conn_state_t s)
{
    switch (s) {
        case AX25_CONN_STATE_DISCONNECTED:        return "DISCONNECTED";
        case AX25_CONN_STATE_AWAITING_CONNECTION: return "AWAITING_CONNECTION";
        case AX25_CONN_STATE_AWAITING_RELEASE:    return "AWAITING_RELEASE";
        case AX25_CONN_STATE_CONNECTED:           return "CONNECTED";
        case AX25_CONN_STATE_TIMER_RECOVERY:      return "TIMER_RECOVERY";
        default:                                  return "UNKNOWN";
    }
}

/*******************************************************************************
 * Console commands
 ******************************************************************************/

static int cmd_connect(void *ctx_ptr, int argc, char **argv)
{
    chat_ctx_t *ctx = (chat_ctx_t *)ctx_ptr;

    if (argc != 2) {
        printf("Usage: connect <CALLSIGN-SSID>\n");
        printf("Example: connect GB7BBS-0\n");
        return 1;
    }

    ax25_conn_state_t state = ax25_conn_get_state(&ctx->conn);
    if (state != AX25_CONN_STATE_DISCONNECTED) {
        printf("Error: already %s\n", state_str(state));
        return 1;
    }

    ax25_address_t remote;
    if (ax25_address_from_string(argv[1], &remote) != ESP_OK) {
        printf("Error: invalid callsign format\n");
        return 1;
    }

    printf("Connecting to %s...\n", argv[1]);
    if (ax25_conn_connect(&ctx->conn, &remote) != ESP_OK) {
        printf("Error: connect failed\n");
        return 1;
    }

    return 0;
}

static int cmd_disconnect(void *ctx_ptr, int argc, char **argv)
{
    (void)argc;
    (void)argv;
    chat_ctx_t *ctx = (chat_ctx_t *)ctx_ptr;

    if (!ax25_conn_is_connected(&ctx->conn)) {
        printf("Not connected\n");
        return 1;
    }

    printf("Disconnecting...\n");
    ax25_conn_shutdown(&ctx->conn);
    return 0;
}

static int cmd_send(void *ctx_ptr, int argc, char **argv)
{
    chat_ctx_t *ctx = (chat_ctx_t *)ctx_ptr;

    if (!ax25_conn_is_connected(&ctx->conn)) {
        printf("Not connected\n");
        return 1;
    }

    if (argc < 2) {
        printf("Usage: send <message>\n");
        return 1;
    }

    char message[256];
    message[0] = '\0';
    for (int i = 1; i < argc; i++) {
        if (i > 1) strcat(message, " ");
        strcat(message, argv[i]);
    }
    strcat(message, "\r");

    esp_err_t err = ax25_conn_send_data(&ctx->conn,
                                        (const uint8_t *)message,
                                        strlen(message));
    if (err != ESP_OK) {
        printf("Error: send failed (%d)\n", err);
        return 1;
    }

    printf("[LOCAL]: %s\n", message);
    return 0;
}

static int cmd_status(void *ctx_ptr, int argc, char **argv)
{
    (void)argc;
    (void)argv;
    chat_ctx_t *ctx = (chat_ctx_t *)ctx_ptr;

    char local_str[12];
    ax25_address_to_string(&ctx->local_addr, local_str, sizeof(local_str));
    printf("Local: %s\n", local_str);
    printf("State: %s\n", state_str(ax25_conn_get_state(&ctx->conn)));

    ax25_address_t remote;
    if (ax25_conn_get_remote_addr(&ctx->conn, &remote) == ESP_OK) {
        char remote_str[12];
        ax25_address_to_string(&remote, remote_str, sizeof(remote_str));
        printf("Remote: %s\n", remote_str);
    }

    return 0;
}

static void register_console_commands(chat_ctx_t *ctx)
{
    const esp_console_cmd_t cmds[] = {
        {
            .command        = "connect",
            .help           = "Connect to remote station (usage: connect CALLSIGN)",
            .hint           = NULL,
            .func_w_context = cmd_connect,
            .context        = ctx,
        },
        {
            .command        = "disconnect",
            .help           = "Disconnect from remote station",

            .hint           = NULL,
            .func_w_context = cmd_disconnect,
            .context        = ctx,
        },
        {
            .command        = "send",
            .help           = "Send message (usage: send <message>)",
            .hint           = NULL,
            .func_w_context = cmd_send,
            .context        = ctx,
        },
        {
            .command        = "status",
            .help           = "Show connection status",
            .hint           = NULL,
            .func_w_context = cmd_status,
            .context        = ctx,
        },
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}

static void init_console(chat_ctx_t *ctx)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "ax25> ";
    repl_config.max_cmdline_length = 256;

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t hw_config =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
#else
#error "connected_chat requires an ESP console backend"
#endif

    register_console_commands(ctx);

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

static esp_err_t configure_uart_keys_from_kconfig(void)
{
    char value[16];
    char err_msg[96] = {0};

    snprintf(value, sizeof(value), "%d", CONFIG_CONNECTED_CHAT_UART_NUM);
    if (ax25_cfg_set("uart.no", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.no: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_CONNECTED_CHAT_TNC_BAUD);
    if (ax25_cfg_set("uart.baud", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.baud: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_CONNECTED_CHAT_TNC_TX_PIN);
    if (ax25_cfg_set("uart.tx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.tx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    snprintf(value, sizeof(value), "%d", CONFIG_CONNECTED_CHAT_TNC_RX_PIN);
    if (ax25_cfg_set("uart.rx_pin", value, err_msg, sizeof(err_msg)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart.rx_pin: %s", err_msg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/*******************************************************************************
 * app_main
 ******************************************************************************/

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-AX25 Connected Chat");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(ax25_cfg_init(NULL, 0));
    ESP_ERROR_CHECK(configure_uart_keys_from_kconfig());

    static chat_ctx_t ctx;

    if (ax25_address_from_string(CONFIG_CONNECTED_CHAT_LOCAL_CALLSIGN,
                                 &ctx.local_addr) != ESP_OK) {
        ESP_LOGE(TAG, "Invalid local callsign: %s",
                 CONFIG_CONNECTED_CHAT_LOCAL_CALLSIGN);
        return;
    }

    if (ax25_router_init() != ESP_OK) {
        ESP_LOGE(TAG, "Router init failed");
        return;
    }

    /*
     * App port: static port matching our local address.  The router delivers
     * incoming frames here; app_port_on_frame passes them to ax25_conn.
     * user_data is &ctx.conn so frame delivery does not require going through
     * the full chat_ctx_t.
     */
    ctx.app_port.destination = ctx.local_addr;
    ctx.app_port.mode        = AX25_PORT_STATIC;
    ctx.app_port.on_tx_frame    = app_port_on_frame;
    ctx.app_port.user_data   = &ctx.conn;
    if (ax25_router_register_port(&ctx.app_port) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register app port");
        ax25_router_deinit();
        return;
    }

    /* Initialize the UART KISS PHY. */
    static ax25_phy_kiss_uart_t phy;
    static ax25_router_port_t   phy_port;
    if (ax25_phy_kiss_uart_init(phy_frame_cb, &phy_port, &phy) != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed");
        ax25_router_remove_port(&ctx.app_port);
        ax25_router_deinit();
        return;
    }

    /* PHY port: default port that forwards frames to the TNC. */
    phy_port.mode      = AX25_PORT_DEFAULT;
    phy_port.on_tx_frame  = phy_port_output_cb;
    phy_port.user_data = &phy;
    if (ax25_router_register_port(&phy_port) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register PHY port");
        ax25_phy_kiss_uart_deinit(&phy);
        ax25_router_remove_port(&ctx.app_port);
        ax25_router_deinit();
        return;
    }

    /* Initialize ax25_conn.  ctx is the user_data for all callbacks. */
    const ax25_conn_callbacks_t cbs = {
        .on_connect    = on_connect,
        .on_disconnect = on_disconnect,
        .on_error      = on_error,
        .on_data       = on_data,
        .on_tx_frame      = on_tx_frame,
    };
    if (ax25_conn_init(&ctx.conn, &ctx.local_addr, &cbs, &ctx, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "ax25_conn_init failed");
        ax25_phy_kiss_uart_deinit(&phy);
        ax25_router_remove_port(&phy_port);
        ax25_router_remove_port(&ctx.app_port);
        ax25_router_deinit();
        return;
    }

    ESP_LOGI(TAG, "Ready — local: %s", CONFIG_CONNECTED_CHAT_LOCAL_CALLSIGN);

    printf("\n");
    printf("=====================================\n");
    printf("  ESP-AX25 Connected Chat\n");
    printf("=====================================\n");
    printf("Commands:\n");
    printf("  connect <CALLSIGN>  - Connect to remote station\n");
    printf("  disconnect          - Disconnect\n");
    printf("  send <message>      - Send a message\n");
    printf("  status              - Show connection status\n");
    printf("  help                - List all commands\n");
    printf("\n");

    init_console(&ctx);
}
