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
#include "ax25_agwpe.h"
#include "ax25_agwpe_client.h"
#include "ax25_frame.h"

#define SERVER_LISTENING_BIT BIT0
#define SERVER_ACCEPTED_BIT  BIT1
#define SERVER_DONE_BIT      BIT2
#define SERVER_ERROR_BIT     BIT3

typedef enum {
    AGWPE_SERVER_SEND_BYTES = 0,
    AGWPE_SERVER_CAPTURE_BYTES,
    AGWPE_SERVER_WAIT_FOR_DISCONNECT,
} agwpe_server_mode_t;

typedef struct {
    volatile int call_count;
    agwpe_frame_t last_frame;
} agwpe_rx_capture_t;

typedef struct {
    uint16_t port;
    agwpe_server_mode_t mode;
    EventGroupHandle_t events;
    TaskHandle_t task_handle;
    int listen_sock;
    int client_sock;
    uint8_t io_buf[AGWPE_MAX_FRAME_SIZE * 2];
    size_t io_len;
    volatile size_t recv_len;
} agwpe_server_ctx_t;

static agwpe_rx_capture_t s_rx_capture;

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

static void on_frame_capture(const agwpe_frame_t *frame, void *user_data)
{
    agwpe_rx_capture_t *capture = (agwpe_rx_capture_t *)user_data;
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

static void wait_for_socket_connected(const ax25_agwpe_client_t *ctx)
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

static void wait_for_queue_messages(const ax25_agwpe_client_t *ctx, UBaseType_t expected)
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

static void agwpe_server_task(void *arg)
{
    agwpe_server_ctx_t *ctx = (agwpe_server_ctx_t *)arg;
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

    if (ctx->mode == AGWPE_SERVER_SEND_BYTES) {
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
    } else if (ctx->mode == AGWPE_SERVER_CAPTURE_BYTES) {
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

static void start_server(agwpe_server_ctx_t *ctx, agwpe_server_mode_t mode, uint16_t port)
{
    ctx->port = port;
    ctx->mode = mode;
    ctx->listen_sock = -1;
    ctx->client_sock = -1;
    ctx->recv_len = 0;
    ctx->events = xEventGroupCreate();
    TEST_ASSERT_NOT_NULL(ctx->events);
    TEST_ASSERT_EQUAL(pdPASS,
                      xTaskCreate(agwpe_server_task,
                                  "agwpe_srv",
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

static void stop_server(agwpe_server_ctx_t *ctx)
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

TEST_CASE("AGWPE client: init rejects invalid arguments", "[ax25_agwpe_client]")
{
    ax25_agwpe_client_t ctx = {0};
    ax25_agwpe_client_config_t config = {
        .host = "127.0.0.1",
        .port = 8000,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
    };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_agwpe_client_init(NULL, &ctx));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_agwpe_client_init(&config, NULL));

    config.host = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_agwpe_client_init(&config, &ctx));

    config.host = "127.0.0.1";
    config.port = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_agwpe_client_init(&config, &ctx));

    config.port = 8000;
    config.on_rx_frame = NULL;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_agwpe_client_init(&config, &ctx));
}

TEST_CASE("AGWPE client: send reports disconnected state", "[ax25_agwpe_client]")
{
    ax25_agwpe_client_t ctx = {0};
    agwpe_frame_t frame;

    agwpe_build_version_req(&frame);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_agwpe_client_send_frame(NULL, &ctx));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_agwpe_client_send_frame(&frame, NULL));

    ctx.send_mutex = xSemaphoreCreateMutex();
    TEST_ASSERT_NOT_NULL(ctx.send_mutex);
    ctx.tx_queue = xQueueCreate(1, sizeof(agwpe_server_ctx_t));
    TEST_ASSERT_NOT_NULL(ctx.tx_queue);
    ctx.sock = -1;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_agwpe_client_send_frame(&frame, &ctx));

    vQueueDelete(ctx.tx_queue);
    vSemaphoreDelete(ctx.send_mutex);
}

TEST_CASE("AGWPE client: loopback receive decodes AGWPE frame", "[ax25_agwpe_client]")
{
    const uint16_t port = 18131;
    agwpe_server_ctx_t server;
    ax25_agwpe_client_t client = {0};
    ax25_agwpe_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
    };
    agwpe_frame_t frame;

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));

    agwpe_build_version_req(&frame);
    frame.header.data_len = 4;
    frame.data[0] = 0xD5;
    frame.data[1] = 0x07;
    frame.data[2] = 0x7F;
    frame.data[3] = 0x00;

    server.io_len = agwpe_frame_encode(&frame, server.io_buf, sizeof(server.io_buf));
    TEST_ASSERT_GREATER_THAN(0, (int)server.io_len);

    start_server(&server, AGWPE_SERVER_SEND_BYTES, port);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);

    wait_for_frame_count(1);
    TEST_ASSERT_EQUAL_UINT8(AGWPE_KIND_VERSION_RESP, s_rx_capture.last_frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(4, s_rx_capture.last_frame.header.data_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame.data, s_rx_capture.last_frame.data, 4);

    ax25_agwpe_client_deinit(&client);
    stop_server(&server);
}

TEST_CASE("AGWPE client: request_version sends encoded AGWPE frame", "[ax25_agwpe_client]")
{
    const uint16_t port = 18132;
    agwpe_server_ctx_t server;
    ax25_agwpe_client_t client = {0};
    ax25_agwpe_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
    };
    agwpe_frame_t expected_frame;
    uint8_t expected_encoded[AGWPE_MAX_FRAME_SIZE] = {0};
    size_t expected_len;

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));

    agwpe_build_version_req(&expected_frame);
    expected_len = agwpe_frame_encode(&expected_frame, expected_encoded, sizeof(expected_encoded));
    TEST_ASSERT_GREATER_THAN(0, (int)expected_len);

    start_server(&server, AGWPE_SERVER_CAPTURE_BYTES, port);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);
    wait_for_socket_connected(&client);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_request_version(&client));

    bits = xEventGroupWaitBits(server.events,
                               SERVER_DONE_BIT,
                               pdFALSE,
                               pdTRUE,
                               pdMS_TO_TICKS(3000));
    TEST_ASSERT_TRUE((bits & SERVER_DONE_BIT) != 0);
    TEST_ASSERT_FALSE((bits & SERVER_ERROR_BIT) != 0);
    TEST_ASSERT_EQUAL_UINT32(expected_len, server.recv_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_encoded, server.io_buf, expected_len);

    ax25_agwpe_client_deinit(&client);
    stop_server(&server);
}

TEST_CASE("AGWPE client: request_port_info sends encoded AGWPE frame", "[ax25_agwpe_client]")
{
    const uint16_t port = 18134;
    agwpe_server_ctx_t server;
    ax25_agwpe_client_t client = {0};
    ax25_agwpe_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
    };
    agwpe_frame_t expected_frame;
    uint8_t expected_encoded[AGWPE_MAX_FRAME_SIZE] = {0};
    size_t expected_len;

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));

    agwpe_build_port_info_req(&expected_frame);
    expected_len = agwpe_frame_encode(&expected_frame, expected_encoded, sizeof(expected_encoded));
    TEST_ASSERT_GREATER_THAN(0, (int)expected_len);

    start_server(&server, AGWPE_SERVER_CAPTURE_BYTES, port);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);
    wait_for_socket_connected(&client);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_request_port_info(&client));

    bits = xEventGroupWaitBits(server.events,
                               SERVER_DONE_BIT,
                               pdFALSE,
                               pdTRUE,
                               pdMS_TO_TICKS(3000));
    TEST_ASSERT_TRUE((bits & SERVER_DONE_BIT) != 0);
    TEST_ASSERT_FALSE((bits & SERVER_ERROR_BIT) != 0);
    TEST_ASSERT_EQUAL_UINT32(expected_len, server.recv_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_encoded, server.io_buf, expected_len);

    ax25_agwpe_client_deinit(&client);
    stop_server(&server);
}

TEST_CASE("AGWPE client: send_unproto encodes AGWPE UI frame", "[ax25_agwpe_client]")
{
    const uint16_t port = 18135;
    agwpe_server_ctx_t server;
    ax25_agwpe_client_t client = {0};
    ax25_agwpe_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
    };
    ax25_frame_t ui_frame;
    agwpe_frame_t expected_frame;
    uint8_t expected_encoded[AGWPE_MAX_FRAME_SIZE] = {0};
    size_t expected_len;

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));

    make_ui_frame(&ui_frame, "N0CALL-1", "APRS", (const uint8_t *)"beacon", 6);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_to_agwpe_unproto(&ui_frame, 2, &expected_frame));
    expected_len = agwpe_frame_encode(&expected_frame, expected_encoded, sizeof(expected_encoded));
    TEST_ASSERT_GREATER_THAN(0, (int)expected_len);

    start_server(&server, AGWPE_SERVER_CAPTURE_BYTES, port);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);
    wait_for_socket_connected(&client);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_send_unproto(&ui_frame, 2, &client));

    bits = xEventGroupWaitBits(server.events,
                               SERVER_DONE_BIT,
                               pdFALSE,
                               pdTRUE,
                               pdMS_TO_TICKS(3000));
    TEST_ASSERT_TRUE((bits & SERVER_DONE_BIT) != 0);
    TEST_ASSERT_FALSE((bits & SERVER_ERROR_BIT) != 0);
    TEST_ASSERT_EQUAL_UINT32(expected_len, server.recv_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_encoded, server.io_buf, expected_len);

    ax25_agwpe_client_deinit(&client);
    stop_server(&server);
}

TEST_CASE("AGWPE client: send_raw encodes AGWPE raw frame", "[ax25_agwpe_client]")
{
    const uint16_t port = 18136;
    agwpe_server_ctx_t server;
    ax25_agwpe_client_t client = {0};
    ax25_agwpe_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
    };
    ax25_frame_t raw_frame;
    agwpe_frame_t expected_frame;
    uint8_t expected_encoded[AGWPE_MAX_FRAME_SIZE] = {0};
    size_t expected_len;

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));

    make_ui_frame(&raw_frame, "N0CALL-2", "TEST-1", (const uint8_t *)"raw test", 8);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_to_agwpe_raw(&raw_frame, 1, &expected_frame));
    expected_len = agwpe_frame_encode(&expected_frame, expected_encoded, sizeof(expected_encoded));
    TEST_ASSERT_GREATER_THAN(0, (int)expected_len);

    start_server(&server, AGWPE_SERVER_CAPTURE_BYTES, port);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);
    wait_for_socket_connected(&client);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_send_raw(&raw_frame, 1, &client));

    bits = xEventGroupWaitBits(server.events,
                               SERVER_DONE_BIT,
                               pdFALSE,
                               pdTRUE,
                               pdMS_TO_TICKS(3000));
    TEST_ASSERT_TRUE((bits & SERVER_DONE_BIT) != 0);
    TEST_ASSERT_FALSE((bits & SERVER_ERROR_BIT) != 0);
    TEST_ASSERT_EQUAL_UINT32(expected_len, server.recv_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_encoded, server.io_buf, expected_len);

    ax25_agwpe_client_deinit(&client);
    stop_server(&server);
}

TEST_CASE("AGWPE client: receive callback fires for connected data", "[ax25_agwpe_client]")
{
    const uint16_t port = 18137;
    agwpe_server_ctx_t server;
    ax25_agwpe_client_t client = {0};
    ax25_agwpe_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
    };
    agwpe_frame_t frame;

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));

    agwpe_frame_init(&frame);
    frame.header.port = 1;
    frame.header.data_kind = AGWPE_KIND_RECV_DATA;
    frame.header.pid = AX25_PID_TEXT;
    memcpy(frame.data, "Hello", 5);
    frame.header.data_len = 5;

    server.io_len = agwpe_frame_encode(&frame, server.io_buf, sizeof(server.io_buf));
    TEST_ASSERT_GREATER_THAN(0, (int)server.io_len);

    start_server(&server, AGWPE_SERVER_SEND_BYTES, port);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);

    wait_for_frame_count(1);
    TEST_ASSERT_EQUAL_UINT8(AGWPE_KIND_RECV_DATA, s_rx_capture.last_frame.header.data_kind);
    TEST_ASSERT_EQUAL_UINT32(5, s_rx_capture.last_frame.header.data_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY((const uint8_t *)"Hello", s_rx_capture.last_frame.data, 5);

    ax25_agwpe_client_deinit(&client);
    stop_server(&server);
}

TEST_CASE("AGWPE client: send reports tx queue full when saturated", "[ax25_agwpe_client]")
{
    const uint16_t port = 18133;
    agwpe_server_ctx_t server;
    ax25_agwpe_client_t client = {0};
    ax25_agwpe_client_config_t config = {
        .host = "127.0.0.1",
        .port = port,
        .on_rx_frame = on_frame_capture,
        .user_data = &s_rx_capture,
        .connect_timeout_ms = 500,
        .reconnect_delay_ms = 100,
        .rx_task_stack_size = 4096,
        .tx_queue_depth = 1,
    };
    agwpe_frame_t frame;

    ensure_tcpip_stack_ready();
    memset(&s_rx_capture, 0, sizeof(s_rx_capture));
    memset(&server, 0, sizeof(server));
    agwpe_build_version_req(&frame);

    start_server(&server, AGWPE_SERVER_CAPTURE_BYTES, port);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_init(&config, &client));

    EventBits_t bits = xEventGroupWaitBits(server.events,
                                           SERVER_ACCEPTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(2000));
    TEST_ASSERT_TRUE((bits & SERVER_ACCEPTED_BIT) != 0);
    wait_for_socket_connected(&client);

    vTaskSuspend(client.tx_task_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_agwpe_client_send_frame(&frame, &client));
    wait_for_queue_messages(&client, 1);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, ax25_agwpe_client_send_frame(&frame, &client));
    vTaskResume(client.tx_task_handle);

    bits = xEventGroupWaitBits(server.events,
                               SERVER_DONE_BIT,
                               pdFALSE,
                               pdTRUE,
                               pdMS_TO_TICKS(3000));
    TEST_ASSERT_TRUE((bits & SERVER_DONE_BIT) != 0);
    TEST_ASSERT_FALSE((bits & SERVER_ERROR_BIT) != 0);
    TEST_ASSERT_GREATER_THAN_UINT32(0, server.recv_len);

    ax25_agwpe_client_deinit(&client);
    stop_server(&server);
}