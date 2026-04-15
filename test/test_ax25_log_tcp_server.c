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

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ax25_config.h"
#include "ax25_log_tcp_server.h"

static const char *TAG = "TEST_LOG_TCP";

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

static void log_server_test_setup(void)
{
    ax25_cfg_deinit();
    ensure_tcpip_stack_ready();
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_init(NULL, 0));
}

static void log_server_test_teardown(void)
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

static void drain_socket(int sock)
{
    uint8_t buf[128];
    while (recv(sock, buf, sizeof(buf), 0) > 0) {
    }
}

static void recv_until_contains(int sock, const char *needle)
{
    char aggregate[512] = {0};
    size_t used = 0;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < deadline) {
        ssize_t len = recv(sock, aggregate + used, sizeof(aggregate) - used - 1, 0);
        if (len > 0) {
            used += (size_t)len;
            aggregate[used] = '\0';
            if (strstr(aggregate, needle) != NULL) {
                return;
            }
            if (used >= sizeof(aggregate) - 2) {
                break;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    TEST_FAIL_MESSAGE("Expected log substring was not received");
}

TEST_CASE("Log TCP server: init rejects null context and double init is blocked",
          "[ax25_log_tcp_server]")
{
    ax25_log_tcp_server_t server = {0};

    log_server_test_setup();
    config_set_or_fail("net.log.port", "18321");

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ax25_log_tcp_server_init(NULL));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_log_tcp_server_init(&server));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ax25_log_tcp_server_init(&server));

    ax25_log_tcp_server_deinit(&server);
    log_server_test_teardown();
}

TEST_CASE("Log TCP server: fanout delivers log lines to multiple clients",
          "[ax25_log_tcp_server]")
{
    ax25_log_tcp_server_t server = {0};
    int client_a = -1;
    int client_b = -1;

    log_server_test_setup();
    config_set_or_fail("net.log.port", "18322");
    config_set_or_fail("net.log.max_conns", "2");
    config_set_or_fail("net.log.queue_depth", "4");

    TEST_ASSERT_EQUAL(ESP_OK, ax25_log_tcp_server_init(&server));

    client_a = connect_client(18322);
    client_b = connect_client(18322);
    vTaskDelay(pdMS_TO_TICKS(100));
    drain_socket(client_a);
    drain_socket(client_b);

    ESP_LOGI(TAG, "fanout-marker-18322");

    recv_until_contains(client_a, "fanout-marker-18322");
    recv_until_contains(client_b, "fanout-marker-18322");

    close(client_a);
    close(client_b);

    ax25_log_tcp_server_deinit(&server);
    log_server_test_teardown();
}

TEST_CASE("Log TCP server: deinit while client connected restores clean shutdown",
          "[ax25_log_tcp_server]")
{
    ax25_log_tcp_server_t server = {0};
    int client = -1;

    log_server_test_setup();
    config_set_or_fail("net.log.port", "18323");

    TEST_ASSERT_EQUAL(ESP_OK, ax25_log_tcp_server_init(&server));
    client = connect_client(18323);
    vTaskDelay(pdMS_TO_TICKS(100));

    ax25_log_tcp_server_deinit(&server);
    close(client);

    log_server_test_teardown();
}

TEST_CASE("Log TCP server: client reconnect reuses sink slot and still receives logs",
          "[ax25_log_tcp_server]")
{
    ax25_log_tcp_server_t server = {0};
    int client_a = -1;
    int client_b = -1;

    log_server_test_setup();
    config_set_or_fail("net.log.port", "18324");
    config_set_or_fail("net.log.max_conns", "1");
    config_set_or_fail("net.log.queue_depth", "4");

    TEST_ASSERT_EQUAL(ESP_OK, ax25_log_tcp_server_init(&server));

    client_a = connect_client(18324);
    vTaskDelay(pdMS_TO_TICKS(100));
    drain_socket(client_a);

    close(client_a);
    client_a = -1;
    vTaskDelay(pdMS_TO_TICKS(150));

    client_b = connect_client(18324);
    vTaskDelay(pdMS_TO_TICKS(100));
    drain_socket(client_b);

    ESP_LOGI(TAG, "reconnect-marker-18324");

    recv_until_contains(client_b, "reconnect-marker-18324");

    close(client_b);
    client_b = -1;

    ax25_log_tcp_server_deinit(&server);
    log_server_test_teardown();
}

TEST_CASE("Log TCP server: repeated reconnect cycles exercise sink slot reuse",
          "[ax25_log_tcp_server]")
{
    ax25_log_tcp_server_t server = {0};
    int client = -1;
    char marker[64];

    log_server_test_setup();
    config_set_or_fail("net.log.port", "18325");
    config_set_or_fail("net.log.max_conns", "1");
    config_set_or_fail("net.log.queue_depth", "4");

    TEST_ASSERT_EQUAL(ESP_OK, ax25_log_tcp_server_init(&server));

    for (int cycle = 0; cycle < 4; cycle++) {
        client = connect_client(18325);
        vTaskDelay(pdMS_TO_TICKS(100));
        drain_socket(client);

        snprintf(marker, sizeof(marker), "cycle-%d-marker-18325", cycle);
        ESP_LOGI(TAG, "%s", marker);

        recv_until_contains(client, marker);

        close(client);
        client = -1;
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    ax25_log_tcp_server_deinit(&server);
    log_server_test_teardown();
}