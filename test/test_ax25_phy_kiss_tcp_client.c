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

#include "unity.h"

#include <string.h>

#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "ax25_address.h"
#include "ax25_buffer.h"
#include "ax25_frame.h"
#include "ax25_kiss.h"
#include "ax25_phy_kiss_tcp_client.h"

#define SERVER_LISTENING_BIT BIT0
#define SERVER_ACCEPTED_BIT  BIT1
#define SERVER_DONE_BIT      BIT2
#define SERVER_ERROR_BIT     BIT3

typedef enum {
    TCP_SERVER_SEND_BYTES = 0,
    TCP_SERVER_CAPTURE_BYTES,
    TCP_SERVER_WAIT_FOR_DISCONNECT,
} tcp_server_mode_t;

typedef struct {
    volatile int call_count;
    ax25_frame_t last_frame;
} rx_capture_t;

typedef struct {
    uint16_t port;
    tcp_server_mode_t mode;
    EventGroupHandle_t events;
    TaskHandle_t task_handle;
    int listen_sock;
    int client_sock;
    uint8_t io_buf[AX25_KISS_MAX_ENCODED_SIZE * 2];
    size_t io_len;
    volatile size_t recv_len;
} tcp_server_ctx_t;

static rx_capture_t s_rx_capture;

static void ensure_tcpip_stack_ready(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_erase());
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_init());
    } else {
        TEST_ASSERT_EQUAL(ESP_OK, err);
    }

    err = esp_netif_init();
    TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_INVALID_STATE);

    err = esp_event_loop_create_default();
    TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_INVALID_STATE);
}

static void make_ui_frame(ax25_frame_t *frame, const char *src, const char *dst,
                          const uint8_t *payload, size_t payload_len)
{
    ax25_frame_init(frame);
    frame->type = AX25_FRAME_UI;
    frame->control = AX25_CTRL_UI;
    frame->pid = AX25_PID_NONE;
    ax25_address_from_string(src, &frame->source);
    ax25_address_from_string(dst, &frame->destination);
    if (payload != NULL && payload_len > 0) {
        size_t copy_len = payload_len > AX25_MAX_INFO_LEN ? AX25_MAX_INFO_LEN : payload_len;
        memcpy(frame->payload, payload, copy_len);
        frame->payload_len = copy_len;
    }
}

static void on_frame_capture(const ax25_frame_t *frame, void *user_data)
{
    rx_capture_t *capture = (rx_capture_t *)user_data;
    if (capture == NULL || frame == NULL) {
        return;
    }

    capture->call_count++;
    capture->last_frame = *frame;
}

static void wait_for_frame_count(int expected)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < deadline) {
        if (s_rx_capture.call_count == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL_INT(expected, s_rx_capture.call_count);
}

static void wait_for_socket_connected(const ax25_phy_kiss_tcp_client_t *ctx)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < deadline) {
        if (ctx->sock >= 0) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_GREATER_OR_EQUAL_INT(0, ctx->sock);
}

static void tcp_server_task(void *arg)
{
    tcp_server_ctx_t *ctx = (tcp_server_ctx_t *)arg;
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(ctx->port),
    };
    int one = 1;

    ctx->listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (ctx->listen_sock < 0) {
        xEventGroupSetBits(ctx->events, SERVER_ERROR_BIT | SERVER_DONE_BIT);
        ctx->task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    setsockopt(ctx->listen_sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (bind(ctx->listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(ctx->listen_sock, 1) != 0) {
        close(ctx->listen_sock);
        ctx->listen_sock = -1;
        xEventGroupSetBits(ctx->events, SERVER_ERROR_BIT | SERVER_DONE_BIT);
        ctx->task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    xEventGroupSetBits(ctx->events, SERVER_LISTENING_BIT);

    ctx->client_sock = accept(ctx->listen_sock, NULL, NULL);
    if (ctx->client_sock < 0) {
        close(ctx->listen_sock);
        ctx->listen_sock = -1;
        xEventGroupSetBits(ctx->events, SERVER_ERROR_BIT | SERVER_DONE_BIT);
        ctx->task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    xEventGroupSetBits(ctx->events, SERVER_ACCEPTED_BIT);

    if (ctx->mode == TCP_SERVER_SEND_BYTES) {
        size_t offset = 0;
        while (offset < ctx->io_len) {
            ssize_t written = send(ctx->client_sock,
                                   ctx->io_buf + offset,
                                   ctx->io_len - offset,
                                   0);
            if (written <= 0) {
                xEventGroupSetBits(ctx->events, SERVER_ERROR_BIT);
                break;
            }
            offset += (size_t)written;
        }
        shutdown(ctx->client_sock, SHUT_WR);
    } else if (ctx->mode == TCP_SERVER_CAPTURE_BYTES) {
        struct timeval recv_timeout = { .tv_sec = 2, .tv_usec = 0 };
        setsockopt(ctx->client_sock,
                   SOL_SOCKET,
                   SO_RCVTIMEO,
                   &recv_timeout,
                   sizeof(recv_timeout));
        ssize_t recv_len = recv(ctx->client_sock, ctx->io_buf, sizeof(ctx->io_buf), 0);
        if (recv_len <= 0) {
            xEventGroupSetBits(ctx->events, SERVER_ERROR_BIT);
        } else {
            ctx->recv_len = (size_t)recv_len;
        }
    } else {
        struct timeval recv_timeout = { .tv_sec = 2, .tv_usec = 0 };
        setsockopt(ctx->client_sock,
                   SOL_SOCKET,
                   SO_RCVTIMEO,
                   &recv_timeout,
                   sizeof(recv_timeout));
        ssize_t recv_len = recv(ctx->client_sock, ctx->io_buf, sizeof(ctx->io_buf), 0);
        if (recv_len > 0) {
            ctx->recv_len = (size_t)recv_len;
        } else if (recv_len < 0) {
            xEventGroupSetBits(ctx->events, SERVER_ERROR_BIT);
        }
    }

    close(ctx->client_sock);
    close(ctx->listen_sock);
    ctx->client_sock = -1;
    ctx->listen_sock = -1;
    xEventGroupSetBits(ctx->events, SERVER_DONE_BIT);
    ctx->task_handle = NULL;
    vTaskDelete(NULL);
}

static void start_tcp_server(tcp_server_ctx_t *ctx, tcp_server_mode_t mode, uint16_t port)
{
    ctx->port = port;
    ctx->mode = mode;
    ctx->listen_sock = -1;
    ctx->client_sock = -1;
    ctx->recv_len = 0;
    ctx->events = xEventGroupCreate();
    TEST_ASSERT_NOT_NULL(ctx->events);
    TEST_ASSERT_EQUAL(pdPASS,
                      xTaskCreate(tcp_server_task,
                                  "tcp_test_srv",
                                  4096,
                                  ctx,
                                  tskIDLE_PRIORITY + 1,
                                  &ctx->task_handle));

    EventBits_t bits = xEventGroupWaitBits(ctx->events,
                                           SERVER_LISTENING_BIT | SERVER_ERROR_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_LISTENING_BIT) != 0);
    TEST_ASSERT_FALSE((bits & SERVER_ERROR_BIT) != 0);
}

static void stop_tcp_server(tcp_server_ctx_t *ctx)
{
    if (ctx->events != NULL) {
        EventBits_t bits = xEventGroupWaitBits(ctx->events,
                                               SERVER_DONE_BIT,
                                               pdFALSE,
                                               pdTRUE,
                                               pdMS_TO_TICKS(3000));
        TEST_ASSERT_TRUE((bits & SERVER_DONE_BIT) != 0);
        TEST_ASSERT_FALSE((bits & SERVER_ERROR_BIT) != 0);
        vEventGroupDelete(ctx->events);
        ctx->events = NULL;
    }

    if (ctx->client_sock >= 0) {
        close(ctx->client_sock);
        ctx->client_sock = -1;
    }
    if (ctx->listen_sock >= 0) {
        close(ctx->listen_sock);
        ctx->listen_sock = -1;
    }
}

static void wait_for_queue_messages(const ax25_phy_kiss_tcp_client_t *ctx, UBaseType_t expected)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < deadline) {
        if (ctx->tx_queue != NULL && uxQueueMessagesWaiting(ctx->tx_queue) == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_ASSERT_NOT_NULL(ctx->tx_queue);
    TEST_ASSERT_EQUAL_UINT32(expected, uxQueueMessagesWaiting(ctx->tx_queue));
}

TEST_CASE("PHY TCP client: init rejects invalid arguments", "[ax25_phy_kiss_tcp_client]")
{
    ax25_phy_kiss_tcp_client_t ctx = {0};
    ax25_phy_kiss_tcp_client_config_t config = {
        .host = "127.0.0.1",
        .port = 8001,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
    };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_tcp_client_init(NULL, &ctx));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_tcp_client_init(&config, NULL));

    config.host = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_tcp_client_init(&config, &ctx));

    config.host = "127.0.0.1";
    config.port = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_tcp_client_init(&config, &ctx));

    config.port = 8001;
    config.on_rx_frame = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_tcp_client_init(&config, &ctx));
}

TEST_CASE("PHY TCP client: send reports disconnected state", "[ax25_phy_kiss_tcp_client]")
{
    ax25_phy_kiss_tcp_client_t ctx = {0};
    ax25_frame_t frame;

    make_ui_frame(&frame, "SRC-1", "DEST-1", (const uint8_t *)"ping", 4);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_tcp_client_send(NULL, &ctx));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_phy_kiss_tcp_client_send(&frame, NULL));

    ctx.send_mutex = xSemaphoreCreateMutex();
    TEST_ASSERT_NOT_NULL(ctx.send_mutex);
    ctx.sock = -1;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_phy_kiss_tcp_client_send(&frame, &ctx));

    vSemaphoreDelete(ctx.send_mutex);
}

TEST_CASE("PHY TCP client: loopback receive ignores non-data KISS frames", "[ax25_phy_kiss_tcp_client]")
{
    const uint16_t port = 18121;
    ax25_frame_t expected;
    ax25_buffer_t raw = {0};
    tcp_server_ctx_t server;
    ax25_phy_kiss_tcp_client_t client = {0};
    ax25_phy_kiss_tcp_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
    };
    char src_buf[16] = {0};
    char dst_buf[16] = {0};

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));

    make_ui_frame(&expected, "N0CALL-1", "APRS", (const uint8_t *)"hello", 5);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_build(&expected, &raw));

    size_t non_data_len = ax25_kiss_encode(0,
                                           1,
                                           raw.data,
                                           raw.len,
                                           server.io_buf,
                                           sizeof(server.io_buf));
    TEST_ASSERT_GREATER_THAN(0, (int)non_data_len);

    size_t data_len = ax25_kiss_encode(0,
                                       0,
                                       raw.data,
                                       raw.len,
                                       server.io_buf + non_data_len,
                                       sizeof(server.io_buf) - non_data_len);
    TEST_ASSERT_GREATER_THAN(0, (int)data_len);

    server.io_len = non_data_len + data_len;
    start_tcp_server(&server, TCP_SERVER_SEND_BYTES, port);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_tcp_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);

    wait_for_frame_count(1);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_address_to_string(&s_rx_capture.last_frame.source,
                                                     src_buf,
                                                     sizeof(src_buf)));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_address_to_string(&s_rx_capture.last_frame.destination,
                                                     dst_buf,
                                                     sizeof(dst_buf)));
    TEST_ASSERT_EQUAL_STRING("N0CALL-1", src_buf);
    TEST_ASSERT_EQUAL_STRING("APRS", dst_buf);
    TEST_ASSERT_EQUAL_UINT8(AX25_CTRL_UI, s_rx_capture.last_frame.control);
    TEST_ASSERT_EQUAL_UINT8(5, s_rx_capture.last_frame.payload_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY((const uint8_t *)"hello",
                                  s_rx_capture.last_frame.payload,
                                  5);

    ax25_phy_kiss_tcp_client_deinit(&client);
    stop_tcp_server(&server);
}

TEST_CASE("PHY TCP client: loopback send writes encoded KISS frame", "[ax25_phy_kiss_tcp_client]")
{
    const uint16_t port = 18122;
    ax25_frame_t frame;
    ax25_buffer_t raw = {0};
    uint8_t expected_encoded[AX25_KISS_MAX_ENCODED_SIZE] = {0};
    tcp_server_ctx_t server;
    ax25_phy_kiss_tcp_client_t client = {0};
    ax25_phy_kiss_tcp_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
    };

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));

    make_ui_frame(&frame, "SRC-2", "DEST-2", (const uint8_t *)"tx", 2);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_build(&frame, &raw));
    size_t expected_len = ax25_kiss_encode(0,
                                           0,
                                           raw.data,
                                           raw.len,
                                           expected_encoded,
                                           sizeof(expected_encoded));
    TEST_ASSERT_GREATER_THAN(0, (int)expected_len);

    start_tcp_server(&server, TCP_SERVER_CAPTURE_BYTES, port);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_tcp_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);
    wait_for_socket_connected(&client);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_tcp_client_send(&frame, &client));

    bits = xEventGroupWaitBits(server.events,
                               SERVER_DONE_BIT,
                               pdFALSE,
                               pdTRUE,
                               pdMS_TO_TICKS(3000));
    TEST_ASSERT_TRUE((bits & SERVER_DONE_BIT) != 0);
    TEST_ASSERT_FALSE((bits & SERVER_ERROR_BIT) != 0);
    TEST_ASSERT_EQUAL_UINT32(expected_len, server.recv_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_encoded, server.io_buf, expected_len);

    ax25_phy_kiss_tcp_client_deinit(&client);
    stop_tcp_server(&server);
}

TEST_CASE("PHY TCP client: send reports tx queue full when saturated", "[ax25_phy_kiss_tcp_client]")
{
    const uint16_t port = 18123;
    ax25_frame_t frame;
    tcp_server_ctx_t server;
    ax25_phy_kiss_tcp_client_t client = {0};
    ax25_phy_kiss_tcp_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
        .tx_queue_depth = 1,
    };

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));

    make_ui_frame(&frame, "SRC-3", "DEST-3", (const uint8_t *)"full", 4);

    start_tcp_server(&server, TCP_SERVER_CAPTURE_BYTES, port);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_tcp_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);
    wait_for_socket_connected(&client);

    vTaskSuspend(client.tx_task_handle);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_tcp_client_send(&frame, &client));
    wait_for_queue_messages(&client, 1);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, ax25_phy_kiss_tcp_client_send(&frame, &client));

    vTaskResume(client.tx_task_handle);

    bits = xEventGroupWaitBits(server.events,
                               SERVER_DONE_BIT,
                               pdFALSE,
                               pdTRUE,
                               pdMS_TO_TICKS(3000));
    TEST_ASSERT_TRUE((bits & SERVER_DONE_BIT) != 0);
    TEST_ASSERT_FALSE((bits & SERVER_ERROR_BIT) != 0);
    TEST_ASSERT_GREATER_THAN_UINT32(0, server.recv_len);

    ax25_phy_kiss_tcp_client_deinit(&client);
    stop_tcp_server(&server);
}

TEST_CASE("PHY TCP client: deinit drops queued tx without hanging", "[ax25_phy_kiss_tcp_client]")
{
    const uint16_t port = 18124;
    ax25_frame_t frame;
    tcp_server_ctx_t server;
    ax25_phy_kiss_tcp_client_t client = {0};
    ax25_phy_kiss_tcp_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
        .tx_queue_depth = 1,
    };

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));

    make_ui_frame(&frame, "SRC-4", "DEST-4", (const uint8_t *)"drop", 4);

    start_tcp_server(&server, TCP_SERVER_WAIT_FOR_DISCONNECT, port);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_tcp_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);
    wait_for_socket_connected(&client);

    vTaskSuspend(client.tx_task_handle);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_phy_kiss_tcp_client_send(&frame, &client));
    wait_for_queue_messages(&client, 1);

    ax25_phy_kiss_tcp_client_deinit(&client);

    TEST_ASSERT_NULL(client.tx_queue);
    TEST_ASSERT_NULL(client.send_mutex);
    TEST_ASSERT_NULL(client.tx_task_handle);
    TEST_ASSERT_NULL(client.rx_task_handle);

    stop_tcp_server(&server);
}