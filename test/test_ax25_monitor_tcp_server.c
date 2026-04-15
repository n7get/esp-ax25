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
#include "freertos/task.h"

#include "ax25_address.h"
#include "ax25_buffer.h"
#include "ax25_config.h"
#include "ax25_frame.h"
#include "ax25_kiss.h"
#include "ax25_monitor_tcp_server.h"
#include "ax25_router.h"

typedef struct {
    volatile int connected_count;
    volatile int disconnected_count;
    ax25_monitor_tcp_server_conn_t *last_conn;
} monitor_events_t;

static monitor_events_t s_monitor_events;

static void on_monitor_connected(ax25_monitor_tcp_server_conn_t *conn)
{
    s_monitor_events.connected_count++;
    s_monitor_events.last_conn = conn;
}

static void on_monitor_disconnected(ax25_monitor_tcp_server_conn_t *conn)
{
    s_monitor_events.disconnected_count++;
    s_monitor_events.last_conn = conn;
}

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

static void monitor_test_setup(void)
{
    memset(&s_monitor_events, 0, sizeof(s_monitor_events));

    ax25_router_deinit();
    ax25_cfg_deinit();

    ensure_tcpip_stack_ready();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_init(NULL, 0));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_init());
}

static void monitor_test_teardown(void)
{
    ax25_router_deinit();
    ax25_cfg_deinit();
}

static void config_set_or_fail(const char *parameter, const char *value)
{
    char err[96] = {0};
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                              ax25_cfg_set(parameter, value, err, sizeof(err)),
                              err);
}

static void wait_for_monitor_count(volatile int *counter, int expected)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < deadline) {
        if (*counter == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL_INT(expected, *counter);
}


static int connect_client(uint16_t port)
{
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    TEST_ASSERT_NOT_EQUAL(-1, sock);

    struct timeval recv_timeout = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        .sin_port = htons(port),
    };

    TEST_ASSERT_EQUAL(0, connect(sock, (struct sockaddr *)&addr, sizeof(addr)));
    return sock;
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
        memcpy(frame->payload, payload, payload_len);
        frame->payload_len = payload_len;
    }
}

TEST_CASE("Monitor TCP server: init rejects null context", "[ax25_monitor_tcp_server]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_monitor_tcp_server_init(NULL, NULL, NULL, NULL));
}

TEST_CASE("Monitor TCP server: client receives routed KISS frame and disconnect callback fires",
          "[ax25_monitor_tcp_server]")
{
    const uint16_t port = 18231;
    int client_sock = -1;
    ax25_monitor_tcp_server_t server = {0};
    ax25_frame_t frame;
    ax25_buffer_t raw = {0};
    uint8_t expected_encoded[AX25_KISS_MAX_ENCODED_SIZE] = {0};
    uint8_t recv_buf[AX25_KISS_MAX_ENCODED_SIZE] = {0};
    ax25_router_port_t source_port = {0};

    monitor_test_setup();
    config_set_or_fail("net.monitor.port", "18231");

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_monitor_tcp_server_init(on_monitor_connected,
                                                   on_monitor_disconnected,
                                                   NULL,
                                                   &server));

    client_sock = connect_client(port);
    wait_for_monitor_count(&s_monitor_events.connected_count, 1);

    TEST_ASSERT_NOT_NULL(s_monitor_events.last_conn);
    TEST_ASSERT_TRUE(s_monitor_events.last_conn->in_use);
    TEST_ASSERT_EQUAL(AX25_PORT_PROMISCUOUS, s_monitor_events.last_conn->router_port.mode);

    TEST_ASSERT_EQUAL(7, send(client_sock, "ignored", 7, 0));

    make_ui_frame(&frame, "SRC-1", "DEST-1", (const uint8_t *)"mon", 3);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_build(&frame, &raw));
    size_t expected_len = ax25_kiss_encode(0,
                                           0,
                                           raw.data,
                                           raw.len,
                                           expected_encoded,
                                           sizeof(expected_encoded));
    TEST_ASSERT_GREATER_THAN(0, (int)expected_len);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_router_send(&frame, &source_port));

    ssize_t recv_len = recv(client_sock, recv_buf, sizeof(recv_buf), 0);
    TEST_ASSERT_EQUAL_INT((int)expected_len, (int)recv_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_encoded, recv_buf, expected_len);

    close(client_sock);
    client_sock = -1;
    wait_for_monitor_count(&s_monitor_events.disconnected_count, 1);

    ax25_monitor_tcp_server_deinit(&server);
    monitor_test_teardown();
}

TEST_CASE("Monitor TCP server: deinit while connected shuts down cleanly", "[ax25_monitor_tcp_server]")
{
    ax25_monitor_tcp_server_t server = {0};
    int client_sock = -1;

    monitor_test_setup();
    config_set_or_fail("net.monitor.port", "18232");

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_monitor_tcp_server_init(on_monitor_connected,
                                                   on_monitor_disconnected,
                                                   NULL,
                                                   &server));

    client_sock = connect_client(18232);
    wait_for_monitor_count(&s_monitor_events.connected_count, 1);

    ax25_monitor_tcp_server_deinit(&server);
    vTaskDelay(pdMS_TO_TICKS(50));
    close(client_sock);

    monitor_test_teardown();
}
