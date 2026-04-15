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

#include "ax25_config.h"
#include "ax25_phy_tcp_server.h"

typedef struct {
    volatile int connected_count;
    volatile int disconnected_count;
    volatile int data_count;
    ax25_phy_tcp_server_conn_t *last_conn;
    uint8_t last_data[64];
    size_t last_len;
} tcp_server_capture_t;

static tcp_server_capture_t s_tcp_capture;

static void on_tcp_connected(ax25_phy_tcp_server_conn_t *conn)
{
    s_tcp_capture.connected_count++;
    s_tcp_capture.last_conn = conn;
}

static void on_tcp_disconnected(ax25_phy_tcp_server_conn_t *conn)
{
    s_tcp_capture.disconnected_count++;
    s_tcp_capture.last_conn = conn;
}

static void on_tcp_data(ax25_phy_tcp_server_conn_t *conn,
                        const uint8_t *data,
                        size_t len)
{
    (void)conn;
    s_tcp_capture.data_count++;
    s_tcp_capture.last_len = len > sizeof(s_tcp_capture.last_data)
        ? sizeof(s_tcp_capture.last_data)
        : len;
    memcpy(s_tcp_capture.last_data, data, s_tcp_capture.last_len);
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

static void tcp_server_test_setup(void)
{
    memset(&s_tcp_capture, 0, sizeof(s_tcp_capture));
    ax25_cfg_deinit();
    ensure_tcpip_stack_ready();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_init(NULL, 0));
}

static void tcp_server_test_teardown(void)
{
    ax25_cfg_deinit();
}

static void config_set_or_fail(const char *parameter, const char *value)
{
    char err[96] = {0};
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                              ax25_cfg_set(parameter, value, err, sizeof(err)),
                              err);
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

static void wait_for_counter(volatile int *counter, int expected)
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

static void wait_for_queue_messages(const ax25_phy_tcp_server_conn_t *conn, UBaseType_t expected)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < deadline) {
        if (conn->tcp_tx_queue != NULL && uxQueueMessagesWaiting(conn->tcp_tx_queue) == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_ASSERT_NOT_NULL(conn->tcp_tx_queue);
    TEST_ASSERT_EQUAL_UINT32(expected, uxQueueMessagesWaiting(conn->tcp_tx_queue));
}

static void wait_for_free_queue_messages(const ax25_phy_tcp_server_conn_t *conn,
                                         UBaseType_t expected)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < deadline) {
        if (conn->tcp_tx_free_queue != NULL &&
            uxQueueMessagesWaiting(conn->tcp_tx_free_queue) == expected) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_ASSERT_NOT_NULL(conn->tcp_tx_free_queue);
    TEST_ASSERT_EQUAL_UINT32(expected, uxQueueMessagesWaiting(conn->tcp_tx_free_queue));
}

TEST_CASE("PHY TCP server: init rejects invalid arguments", "[ax25_phy_tcp_server]")
{
    ax25_phy_tcp_server_t server = {0};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_phy_tcp_server_init(NULL,
                                               on_tcp_connected,
                                               on_tcp_disconnected,
                                               on_tcp_data,
                                               NULL,
                                               &server));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_phy_tcp_server_init("net.kiss.port",
                                               on_tcp_connected,
                                               on_tcp_disconnected,
                                               NULL,
                                               NULL,
                                               &server));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_phy_tcp_server_init("net.kiss.port",
                                               on_tcp_connected,
                                               on_tcp_disconnected,
                                               on_tcp_data,
                                               NULL,
                                               NULL));
}

TEST_CASE("PHY TCP server: loopback client receives sent data and callbacks fire",
          "[ax25_phy_tcp_server]")
{
    ax25_phy_tcp_server_t server = {0};
    int client_sock = -1;
    static const uint8_t payload[] = "hello over tcp";
    uint8_t recv_buf[32] = {0};

    tcp_server_test_setup();
    config_set_or_fail("net.kiss.port", "18311");
    config_set_or_fail("net.kiss.max_conns", "2");

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_phy_tcp_server_init("net.kiss.port",
                                               on_tcp_connected,
                                               on_tcp_disconnected,
                                               on_tcp_data,
                                               NULL,
                                               &server));

    client_sock = connect_client(18311);
    wait_for_counter(&s_tcp_capture.connected_count, 1);

    TEST_ASSERT_NOT_NULL(s_tcp_capture.last_conn);
    ax25_phy_tcp_server_conn_set_user_data(s_tcp_capture.last_conn, (void *)0x1234);
    TEST_ASSERT_EQUAL_PTR((void *)0x1234,
                          ax25_phy_tcp_server_conn_get_user_data(s_tcp_capture.last_conn));

    TEST_ASSERT_EQUAL(4, send(client_sock, "ping", 4, 0));
    wait_for_counter(&s_tcp_capture.data_count, 1);
    TEST_ASSERT_EQUAL_UINT32(4, s_tcp_capture.last_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY((const uint8_t *)"ping", s_tcp_capture.last_data, 4);

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_phy_tcp_server_conn_send(s_tcp_capture.last_conn,
                                                    payload,
                                                    sizeof(payload) - 1));
    ssize_t recv_len = recv(client_sock, recv_buf, sizeof(recv_buf), 0);
    TEST_ASSERT_EQUAL_INT((int)(sizeof(payload) - 1), (int)recv_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, recv_buf, sizeof(payload) - 1);

    close(client_sock);
    client_sock = -1;
    wait_for_counter(&s_tcp_capture.disconnected_count, 1);

    ax25_phy_tcp_server_deinit(&server);
    tcp_server_test_teardown();
}

TEST_CASE("PHY TCP server: max clients are enforced", "[ax25_phy_tcp_server]")
{
    ax25_phy_tcp_server_t server = {0};
    int first_sock = -1;
    int second_sock = -1;
    uint8_t recv_byte = 0;

    tcp_server_test_setup();
    config_set_or_fail("net.kiss.port", "18312");
    config_set_or_fail("net.kiss.max_conns", "1");

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_phy_tcp_server_init("net.kiss.port",
                                               on_tcp_connected,
                                               on_tcp_disconnected,
                                               on_tcp_data,
                                               NULL,
                                               &server));

    first_sock = connect_client(18312);
    wait_for_counter(&s_tcp_capture.connected_count, 1);

    second_sock = connect_client(18312);
    vTaskDelay(pdMS_TO_TICKS(100));

    ssize_t recv_len = recv(second_sock, &recv_byte, 1, 0);
    TEST_ASSERT_EQUAL_INT(0, (int)recv_len);

    close(second_sock);
    close(first_sock);
    wait_for_counter(&s_tcp_capture.disconnected_count, 1);

    ax25_phy_tcp_server_deinit(&server);
    tcp_server_test_teardown();
}

TEST_CASE("PHY TCP server: send reports tx queue full when saturated", "[ax25_phy_tcp_server]")
{
    ax25_phy_tcp_server_t server = {0};
    int client_sock = -1;
    static const uint8_t payload[] = "queued server payload";

    tcp_server_test_setup();
    config_set_or_fail("net.kiss.port", "18313");
    config_set_or_fail("net.kiss.max_conns", "1");
    config_set_or_fail("net.kiss.tx_queue_depth", "1");

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_phy_tcp_server_init("net.kiss.port",
                                               on_tcp_connected,
                                               on_tcp_disconnected,
                                               on_tcp_data,
                                               NULL,
                                               &server));

    client_sock = connect_client(18313);
    wait_for_counter(&s_tcp_capture.connected_count, 1);

    TEST_ASSERT_NOT_NULL(s_tcp_capture.last_conn);
    TEST_ASSERT_NOT_NULL(s_tcp_capture.last_conn->tcp_tx_task_handle);
    vTaskSuspend(s_tcp_capture.last_conn->tcp_tx_task_handle);

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_phy_tcp_server_conn_send(s_tcp_capture.last_conn,
                                                    payload,
                                                    sizeof(payload) - 1));
    wait_for_queue_messages(s_tcp_capture.last_conn, 1);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM,
                      ax25_phy_tcp_server_conn_send(s_tcp_capture.last_conn,
                                                    payload,
                                                    sizeof(payload) - 1));

    vTaskResume(s_tcp_capture.last_conn->tcp_tx_task_handle);

    uint8_t recv_buf[64] = {0};
    ssize_t recv_len = recv(client_sock, recv_buf, sizeof(recv_buf), 0);
    TEST_ASSERT_EQUAL_INT((int)(sizeof(payload) - 1), (int)recv_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, recv_buf, sizeof(payload) - 1);

    close(client_sock);
    client_sock = -1;
    wait_for_counter(&s_tcp_capture.disconnected_count, 1);

    ax25_phy_tcp_server_deinit(&server);
    tcp_server_test_teardown();
}

TEST_CASE("PHY TCP server: tx pool items are reused after delivery", "[ax25_phy_tcp_server]")
{
    ax25_phy_tcp_server_t server = {0};
    int client_sock = -1;
    static const uint8_t payload[] = "reused pool payload";
    uint8_t recv_buf[64] = {0};

    tcp_server_test_setup();
    config_set_or_fail("net.kiss.port", "18315");
    config_set_or_fail("net.kiss.max_conns", "1");
    config_set_or_fail("net.kiss.tx_queue_depth", "2");

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_phy_tcp_server_init("net.kiss.port",
                                               on_tcp_connected,
                                               on_tcp_disconnected,
                                               on_tcp_data,
                                               NULL,
                                               &server));

    client_sock = connect_client(18315);
    wait_for_counter(&s_tcp_capture.connected_count, 1);

    TEST_ASSERT_NOT_NULL(s_tcp_capture.last_conn);
    wait_for_free_queue_messages(s_tcp_capture.last_conn, 2);

    TEST_ASSERT_NOT_NULL(s_tcp_capture.last_conn->tcp_tx_task_handle);
    vTaskSuspend(s_tcp_capture.last_conn->tcp_tx_task_handle);

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_phy_tcp_server_conn_send(s_tcp_capture.last_conn,
                                                    payload,
                                                    sizeof(payload) - 1));
    wait_for_queue_messages(s_tcp_capture.last_conn, 1);
    wait_for_free_queue_messages(s_tcp_capture.last_conn, 1);

    vTaskResume(s_tcp_capture.last_conn->tcp_tx_task_handle);

    ssize_t recv_len = recv(client_sock, recv_buf, sizeof(recv_buf), 0);
    TEST_ASSERT_EQUAL_INT((int)(sizeof(payload) - 1), (int)recv_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, recv_buf, sizeof(payload) - 1);

    wait_for_queue_messages(s_tcp_capture.last_conn, 0);
    wait_for_free_queue_messages(s_tcp_capture.last_conn, 2);

    close(client_sock);
    client_sock = -1;
    wait_for_counter(&s_tcp_capture.disconnected_count, 1);

    ax25_phy_tcp_server_deinit(&server);
    tcp_server_test_teardown();
}

TEST_CASE("PHY TCP server: oversized payload is rejected before queueing", "[ax25_phy_tcp_server]")
{
    ax25_phy_tcp_server_t server = {0};
    int client_sock = -1;
    uint8_t oversized_payload[1024] = {0};

    tcp_server_test_setup();
    config_set_or_fail("net.kiss.port", "18316");
    config_set_or_fail("net.kiss.max_conns", "1");
    config_set_or_fail("net.kiss.tx_queue_depth", "2");

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_phy_tcp_server_init("net.kiss.port",
                                               on_tcp_connected,
                                               on_tcp_disconnected,
                                               on_tcp_data,
                                               NULL,
                                               &server));

    client_sock = connect_client(18316);
    wait_for_counter(&s_tcp_capture.connected_count, 1);

    TEST_ASSERT_NOT_NULL(s_tcp_capture.last_conn);
    wait_for_free_queue_messages(s_tcp_capture.last_conn, 2);
    wait_for_queue_messages(s_tcp_capture.last_conn, 0);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE,
                      ax25_phy_tcp_server_conn_send(s_tcp_capture.last_conn,
                                                    oversized_payload,
                                                    sizeof(oversized_payload)));

    wait_for_free_queue_messages(s_tcp_capture.last_conn, 2);
    wait_for_queue_messages(s_tcp_capture.last_conn, 0);

    close(client_sock);
    client_sock = -1;
    wait_for_counter(&s_tcp_capture.disconnected_count, 1);

    ax25_phy_tcp_server_deinit(&server);
    tcp_server_test_teardown();
}

TEST_CASE("PHY TCP server: deinit drops queued tx without hanging", "[ax25_phy_tcp_server]")
{
    ax25_phy_tcp_server_t server = {0};
    int client_sock = -1;
    static const uint8_t payload[] = "queued then deinit";

    tcp_server_test_setup();
    config_set_or_fail("net.kiss.port", "18314");
    config_set_or_fail("net.kiss.max_conns", "1");
    config_set_or_fail("net.kiss.tx_queue_depth", "1");

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_phy_tcp_server_init("net.kiss.port",
                                               on_tcp_connected,
                                               on_tcp_disconnected,
                                               on_tcp_data,
                                               NULL,
                                               &server));

    client_sock = connect_client(18314);
    wait_for_counter(&s_tcp_capture.connected_count, 1);

    TEST_ASSERT_NOT_NULL(s_tcp_capture.last_conn);
    TEST_ASSERT_NOT_NULL(s_tcp_capture.last_conn->tcp_tx_task_handle);
    vTaskSuspend(s_tcp_capture.last_conn->tcp_tx_task_handle);

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_phy_tcp_server_conn_send(s_tcp_capture.last_conn,
                                                    payload,
                                                    sizeof(payload) - 1));
    wait_for_queue_messages(s_tcp_capture.last_conn, 1);

    ax25_phy_tcp_server_deinit(&server);

    uint8_t recv_buf[64] = {0};
    ssize_t recv_len = recv(client_sock, recv_buf, sizeof(recv_buf), 0);
    TEST_ASSERT_TRUE(recv_len == 0 || recv_len < 0);

    close(client_sock);
    client_sock = -1;
    tcp_server_test_teardown();
}