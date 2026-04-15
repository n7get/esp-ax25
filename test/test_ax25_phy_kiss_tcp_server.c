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
#include "ax25_frame.h"
#include "ax25_kiss.h"
#include "ax25_config.h"
#include "ax25_kiss_tcp_server.h"

static void on_connected_noop(ax25_kiss_tcp_server_conn_t *conn)
{
    (void)conn;
}

static void on_disconnected_noop(ax25_kiss_tcp_server_conn_t *conn)
{
    (void)conn;
}

static void on_frame_noop(ax25_kiss_tcp_server_conn_t *conn,
                          const ax25_frame_t *frame)
{
    (void)conn;
    (void)frame;
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

    err = ax25_cfg_init(NULL, 0);
    TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_INVALID_STATE);
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

static void wait_for_kiss_conn(const ax25_kiss_tcp_server_t *srv, int expected)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < deadline) {
        int active = 0;
        for (size_t i = 0; i < srv->tcp_server.max_clients; i++) {
            if (srv->conns[i].in_use) {
                active++;
            }
        }
        if (active == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    int active = 0;
    for (size_t i = 0; i < srv->tcp_server.max_clients; i++) {
        if (srv->conns[i].in_use) {
            active++;
        }
    }
    TEST_ASSERT_EQUAL_INT(expected, active);
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

TEST_CASE("TCP server: multiple instances can run on different ports", "[ax25_kiss_tcp_server]")
{
    ensure_tcpip_stack_ready();

    ax25_kiss_tcp_server_t srv1 = {0};
    ax25_kiss_tcp_server_t srv2 = {0};

    /* Set port for srv1 */
    char err_msg[96] = {0};
    char value[16] = {0};
    snprintf(value, sizeof(value), "%d", 18001);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_set("net.kiss.port", value, err_msg, sizeof(err_msg)));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_kiss_tcp_server_init(on_connected_noop,
                                                         on_disconnected_noop,
                                                         on_frame_noop,
                                                         NULL,
                                                         &srv1));

    /* Set port for srv2 */
    snprintf(value, sizeof(value), "%d", 18002);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_set("net.kiss.port", value, err_msg, sizeof(err_msg)));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_kiss_tcp_server_init(on_connected_noop,
                                                         on_disconnected_noop,
                                                         on_frame_noop,
                                                         NULL,
                                                         &srv2));

    TEST_ASSERT_TRUE(srv1.running);
    TEST_ASSERT_TRUE(srv2.running);
    TEST_ASSERT_NOT_EQUAL(-1, srv1.listen_sock);
    TEST_ASSERT_NOT_EQUAL(-1, srv2.listen_sock);

    ax25_kiss_tcp_server_deinit(&srv2);
    ax25_kiss_tcp_server_deinit(&srv1);
}

TEST_CASE("TCP server: same port conflict is isolated per instance", "[ax25_kiss_tcp_server]")
{
    ensure_tcpip_stack_ready();

    ax25_kiss_tcp_server_t srv1 = {0};
    ax25_kiss_tcp_server_t srv2 = {0};

    char err_msg[96] = {0};
    char value[16] = {0};
    snprintf(value, sizeof(value), "%d", 18011);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_set("net.kiss.port", value, err_msg, sizeof(err_msg)));

    TEST_ASSERT_EQUAL(ESP_OK, ax25_kiss_tcp_server_init(on_connected_noop,
                                                         on_disconnected_noop,
                                                         on_frame_noop,
                                                         NULL,
                                                         &srv1));

    esp_err_t second_err = ax25_kiss_tcp_server_init(on_connected_noop,
                                                      on_disconnected_noop,
                                                      on_frame_noop,
                                                      NULL,
                                                      &srv2);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, second_err);

    ax25_kiss_tcp_server_deinit(&srv2);
    ax25_kiss_tcp_server_deinit(&srv1);
}

TEST_CASE("KISS TCP server: queued send reaches client", "[ax25_kiss_tcp_server]")
{
    ensure_tcpip_stack_ready();

    ax25_kiss_tcp_server_t srv = {0};
    ax25_frame_t frame;
    ax25_buffer_t raw = {0};
    uint8_t expected_encoded[AX25_KISS_MAX_ENCODED_SIZE] = {0};
    uint8_t recv_buf[AX25_KISS_MAX_ENCODED_SIZE] = {0};
    char err_msg[96] = {0};
    int client_sock = -1;

    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_set("net.kiss.port", "18021", err_msg, sizeof(err_msg)));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_kiss_tcp_server_init(on_connected_noop,
                                                         on_disconnected_noop,
                                                         on_frame_noop,
                                                         NULL,
                                                         &srv));

    client_sock = connect_client(18021);
    wait_for_kiss_conn(&srv, 1);

    make_ui_frame(&frame, "SRC-1", "DEST-1", (const uint8_t *)"kiss", 4);
    TEST_ASSERT_EQUAL(ESP_OK, ax25_frame_build(&frame, &raw));
    size_t expected_len = ax25_kiss_encode(0, 0, raw.data, raw.len,
                                           expected_encoded, sizeof(expected_encoded));
    TEST_ASSERT_GREATER_THAN(0, (int)expected_len);

    TEST_ASSERT_EQUAL(ESP_OK, ax25_kiss_tcp_server_conn_send(&srv.conns[0], &frame));

    ssize_t recv_len = recv(client_sock, recv_buf, sizeof(recv_buf), 0);
    TEST_ASSERT_EQUAL_INT((int)expected_len, (int)recv_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_encoded, recv_buf, expected_len);

    close(client_sock);
    ax25_kiss_tcp_server_deinit(&srv);
}

