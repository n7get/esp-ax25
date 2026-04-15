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
 * @file ax25_agwpe_client.c
 * @brief TCP AGWPE client transport implementation
 */

#include "ax25_agwpe_client.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include "esp_log.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

static const char *TAG = "AX25_AGWPE_CLI";

#define AX25_AGWPE_CLIENT_DEFAULT_CONNECT_TIMEOUT_MS 6000u
#define AX25_AGWPE_CLIENT_DEFAULT_RECONNECT_DELAY_MS 5000u
#define AX25_AGWPE_CLIENT_DEFAULT_TASK_STACK_SIZE 4096u
#define AX25_AGWPE_CLIENT_DEFAULT_TASK_PRIORITY 5u
#define AX25_AGWPE_CLIENT_DEFAULT_TX_QUEUE_DEPTH 8u

typedef struct {
    size_t len;
    uint8_t data[AGWPE_MAX_FRAME_SIZE];
} ax25_agwpe_client_tx_item_t;

static int connect_with_timeout(int sock, const struct sockaddr *addr,
                                socklen_t addrlen, int timeout_ms)
{
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0) {
        return -1;
    }
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
        return -1;
    }

    int rc = connect(sock, addr, addrlen);
    if (rc == 0) {
        (void)fcntl(sock, F_SETFL, flags);
        return 0;
    }
    if (errno != EINPROGRESS) {
        (void)fcntl(sock, F_SETFL, flags);
        return -1;
    }

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(sock, &write_fds);

    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };

    rc = select(sock + 1, NULL, &write_fds, NULL, &tv);
    if (rc <= 0) {
        errno = (rc == 0) ? ETIMEDOUT : errno;
        (void)fcntl(sock, F_SETFL, flags);
        return -1;
    }

    int so_error = 0;
    socklen_t so_error_len = sizeof(so_error);
    if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &so_error_len) < 0) {
        (void)fcntl(sock, F_SETFL, flags);
        return -1;
    }

    (void)fcntl(sock, F_SETFL, flags);
    if (so_error != 0) {
        errno = so_error;
        return -1;
    }

    return 0;
}

static esp_err_t send_built_frame(ax25_agwpe_client_t *ctx,
                                  const agwpe_frame_t *frame)
{
    return ax25_agwpe_client_send_frame(frame, ctx);
}

static void tcp_tx_task(void *arg)
{
    ax25_agwpe_client_t *ctx = (ax25_agwpe_client_t *)arg;
    ax25_agwpe_client_tx_item_t item;

    while (ctx->running) {
        if (xQueueReceive(ctx->tx_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        if (!ctx->running) {
            break;
        }

        xSemaphoreTake(ctx->send_mutex, portMAX_DELAY);

        if (ctx->sock < 0) {
            xSemaphoreGive(ctx->send_mutex);
            ESP_LOGW(TAG, "tx drop: not connected");
            continue;
        }

        size_t offset = 0;
        while (ctx->running && offset < item.len) {
            ssize_t written = send(ctx->sock,
                                   item.data + offset,
                                   item.len - offset,
                                   MSG_NOSIGNAL);
            if (written > 0) {
                offset += (size_t)written;
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            ESP_LOGE(TAG, "send failed: %s", strerror(errno));
            break;
        }

        xSemaphoreGive(ctx->send_mutex);
    }

    ctx->tx_task_handle = NULL;
    vTaskDelete(NULL);
}

static void tcp_rx_task(void *arg)
{
    ax25_agwpe_client_t *ctx = (ax25_agwpe_client_t *)arg;
    uint8_t rx_buf[256];

    while (ctx->running) {
        struct addrinfo hints = {
            .ai_family = AF_INET,
            .ai_socktype = SOCK_STREAM,
        };
        struct addrinfo *res = NULL;
        char port_str[8];
        snprintf(port_str, sizeof(port_str), "%u", (unsigned)ctx->port);

        if (getaddrinfo(ctx->host, port_str, &hints, &res) != 0 || res == NULL) {
            ESP_LOGW(TAG, "DNS lookup failed for %s", ctx->host);
            goto reconnect_delay;
        }

        int sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (sock < 0) {
            ESP_LOGW(TAG, "Socket create failed: %s", strerror(errno));
            freeaddrinfo(res);
            goto reconnect_delay;
        }

        struct timeval recv_tv = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &recv_tv, sizeof(recv_tv));

        struct timeval send_tv = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &send_tv, sizeof(send_tv));

        ESP_LOGI(TAG, "Connecting to %s:%u", ctx->host, (unsigned)ctx->port);
        if (connect_with_timeout(sock, res->ai_addr, res->ai_addrlen,
                                 (int)ctx->connect_timeout_ms) != 0) {
            ESP_LOGW(TAG, "Connect to %s:%u failed: %s",
                     ctx->host, (unsigned)ctx->port, strerror(errno));
            close(sock);
            freeaddrinfo(res);
            goto reconnect_delay;
        }
        freeaddrinfo(res);

        ESP_LOGI(TAG, "Connected to %s:%u", ctx->host, (unsigned)ctx->port);

        xSemaphoreTake(ctx->send_mutex, portMAX_DELAY);
        ctx->sock = sock;
        xSemaphoreGive(ctx->send_mutex);

        agwpe_decoder_reset(&ctx->decoder);

        while (ctx->running) {
            ssize_t len = recv(sock, rx_buf, sizeof(rx_buf), 0);
            if (len > 0) {
                agwpe_decoder_process_bytes(&ctx->decoder, rx_buf, (size_t)len);
                continue;
            }
            if (len == 0) {
                ESP_LOGW(TAG, "Connection closed by %s:%u",
                         ctx->host, (unsigned)ctx->port);
                break;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            if (ctx->running) {
                ESP_LOGW(TAG, "recv error: %s", strerror(errno));
            }
            break;
        }

        xSemaphoreTake(ctx->send_mutex, portMAX_DELAY);
        ctx->sock = -1;
        xSemaphoreGive(ctx->send_mutex);

        close(sock);

        if (ctx->running) {
            ESP_LOGW(TAG, "Disconnected from %s:%u, reconnecting...",
                     ctx->host, (unsigned)ctx->port);
        }

reconnect_delay:
        {
            TickType_t deadline = xTaskGetTickCount() +
                                  pdMS_TO_TICKS(ctx->reconnect_delay_ms);
            while (ctx->running &&
                   (TickType_t)(deadline - xTaskGetTickCount()) < 0x80000000u) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }

    ctx->rx_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t ax25_agwpe_client_init(const ax25_agwpe_client_config_t *config,
                                 ax25_agwpe_client_t *ctx)
{
    if (ctx == NULL || config == NULL || config->host == NULL ||
        config->port == 0 || config->on_rx_frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(ctx, 0, sizeof(*ctx));

    ctx->host = config->host;
    ctx->port = config->port;
    ctx->on_rx_frame = config->on_rx_frame;
    ctx->user_data = config->user_data;
    ctx->connect_timeout_ms = config->connect_timeout_ms
                                  ? config->connect_timeout_ms
                                  : AX25_AGWPE_CLIENT_DEFAULT_CONNECT_TIMEOUT_MS;
    ctx->reconnect_delay_ms = config->reconnect_delay_ms
                                  ? config->reconnect_delay_ms
                                  : AX25_AGWPE_CLIENT_DEFAULT_RECONNECT_DELAY_MS;
    ctx->sock = -1;

    uint32_t rx_stk = config->rx_task_stack_size
                          ? config->rx_task_stack_size
                          : AX25_AGWPE_CLIENT_DEFAULT_TASK_STACK_SIZE;
    UBaseType_t rx_prio = config->rx_task_priority
                              ? config->rx_task_priority
                              : AX25_AGWPE_CLIENT_DEFAULT_TASK_PRIORITY;
    uint32_t tx_stk = config->tx_task_stack_size
                          ? config->tx_task_stack_size
                          : AX25_AGWPE_CLIENT_DEFAULT_TASK_STACK_SIZE;
    UBaseType_t tx_prio = config->tx_task_priority
                              ? config->tx_task_priority
                              : AX25_AGWPE_CLIENT_DEFAULT_TASK_PRIORITY;
    uint32_t tx_depth = config->tx_queue_depth
                            ? config->tx_queue_depth
                            : AX25_AGWPE_CLIENT_DEFAULT_TX_QUEUE_DEPTH;

    ctx->send_mutex = xSemaphoreCreateMutex();
    if (ctx->send_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ctx->tx_queue = xQueueCreate(tx_depth, sizeof(ax25_agwpe_client_tx_item_t));
    if (ctx->tx_queue == NULL) {
        vSemaphoreDelete(ctx->send_mutex);
        ctx->send_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    agwpe_decoder_init(&ctx->decoder, ctx->on_rx_frame, ctx->user_data);

    ctx->running = true;

    if (xTaskCreate(tcp_tx_task, "agwpe_tx_task", tx_stk, ctx, tx_prio,
                    &ctx->tx_task_handle) != pdPASS) {
        ctx->running = false;
        vQueueDelete(ctx->tx_queue);
        ctx->tx_queue = NULL;
        vSemaphoreDelete(ctx->send_mutex);
        ctx->send_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(tcp_rx_task, "agwpe_rx_task", rx_stk, ctx, rx_prio,
                    &ctx->rx_task_handle) != pdPASS) {
        ctx->running = false;
        for (int i = 0; i < 20 && ctx->tx_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (ctx->tx_task_handle != NULL) {
            vTaskDelete(ctx->tx_task_handle);
            ctx->tx_task_handle = NULL;
        }
        vQueueDelete(ctx->tx_queue);
        ctx->tx_queue = NULL;
        vSemaphoreDelete(ctx->send_mutex);
        ctx->send_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void ax25_agwpe_client_deinit(ax25_agwpe_client_t *ctx)
{
    if (ctx == NULL || !ctx->running) {
        return;
    }

    ctx->running = false;

    xSemaphoreTake(ctx->send_mutex, portMAX_DELAY);
    int sock = ctx->sock;
    xSemaphoreGive(ctx->send_mutex);

    if (sock >= 0) {
        shutdown(sock, SHUT_RDWR);
    }

    for (int i = 0; i < 30 && ctx->rx_task_handle != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (ctx->rx_task_handle != NULL) {
        vTaskDelete(ctx->rx_task_handle);
        ctx->rx_task_handle = NULL;
    }

    for (int i = 0; i < 30 && ctx->tx_task_handle != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (ctx->tx_task_handle != NULL) {
        vTaskDelete(ctx->tx_task_handle);
        ctx->tx_task_handle = NULL;
    }

    if (ctx->tx_queue != NULL) {
        vQueueDelete(ctx->tx_queue);
        ctx->tx_queue = NULL;
    }

    if (ctx->send_mutex != NULL) {
        vSemaphoreDelete(ctx->send_mutex);
        ctx->send_mutex = NULL;
    }

    ctx->sock = -1;
}

esp_err_t ax25_agwpe_client_send_frame(const agwpe_frame_t *frame,
                                       ax25_agwpe_client_t *ctx)
{
    if (ctx == NULL || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ctx->send_mutex == NULL || ctx->tx_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t encoded[AGWPE_MAX_FRAME_SIZE];
    size_t encoded_len = agwpe_frame_encode(frame, encoded, sizeof(encoded));
    if (encoded_len == 0) {
        ESP_LOGE(TAG, "agwpe_frame_encode failed for kind '%c'",
                 frame->header.data_kind);
        return ESP_FAIL;
    }

    xSemaphoreTake(ctx->send_mutex, portMAX_DELAY);
    if (ctx->sock < 0) {
        xSemaphoreGive(ctx->send_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(ctx->send_mutex);

    ax25_agwpe_client_tx_item_t item = {
        .len = encoded_len,
    };
    memcpy(item.data, encoded, encoded_len);

    if (xQueueSend(ctx->tx_queue, &item, 0) != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t ax25_agwpe_client_request_version(ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    agwpe_build_version_req(&frame);
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_request_port_info(ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    agwpe_build_port_info_req(&frame);
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_request_port_caps(uint8_t port,
                                              ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    agwpe_build_port_cap_req(port, &frame);
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_register_callsign(uint8_t port,
                                              const char *callsign,
                                              ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_register_call(port, callsign, &frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_unregister_callsign(uint8_t port,
                                                const char *callsign,
                                                ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_unregister_call(port, callsign, &frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_enable_monitor(ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    agwpe_build_enable_monitor(&frame);
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_enable_raw(ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    agwpe_build_enable_raw(&frame);
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_send_raw(const ax25_frame_t *frame,
                                     uint8_t port,
                                     ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t agwpe_frame;
    esp_err_t err = ax25_to_agwpe_raw(frame, port, &agwpe_frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &agwpe_frame);
}

esp_err_t ax25_agwpe_client_send_unproto(const ax25_frame_t *frame,
                                         uint8_t port,
                                         ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t agwpe_frame;
    esp_err_t err = ax25_to_agwpe_unproto(frame, port, &agwpe_frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &agwpe_frame);
}

esp_err_t ax25_agwpe_client_connect(uint8_t port,
                                    const char *from_call,
                                    const char *to_call,
                                    ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_connect_req(port, from_call, to_call, &frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_connect_via(uint8_t port,
                                        const char *from_call,
                                        const char *to_call,
                                        const char *digipeaters[],
                                        uint8_t num_digis,
                                        ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_connect_via_req(port, from_call, to_call,
                                                digipeaters, num_digis, &frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_disconnect(uint8_t port,
                                       const char *from_call,
                                       const char *to_call,
                                       ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_disconnect_req(port, from_call, to_call, &frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_send_connected(uint8_t port,
                                           const char *from_call,
                                           const char *to_call,
                                           const uint8_t *data,
                                           size_t data_len,
                                           uint8_t pid,
                                           ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_send_data(port, from_call, to_call, data,
                                          data_len, pid, &frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_send_unproto_data(uint8_t port,
                                              const char *from_call,
                                              const char *to_call,
                                              const uint8_t *data,
                                              size_t data_len,
                                              uint8_t pid,
                                              ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_send_unproto(port, from_call, to_call, data,
                                             data_len, pid, &frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_send_unproto_via_data(uint8_t port,
                                                  const char *from_call,
                                                  const char *to_call,
                                                  const char *digipeaters[],
                                                  uint8_t num_digis,
                                                  const uint8_t *data,
                                                  size_t data_len,
                                                  uint8_t pid,
                                                  ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_send_unproto_via(port, from_call, to_call,
                                                 digipeaters, num_digis, data,
                                                 data_len, pid, &frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_query_outstanding(uint8_t port,
                                              const char *from_call,
                                              const char *to_call,
                                              ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    esp_err_t err = agwpe_build_outstanding_req(port, from_call, to_call, &frame);
    if (err != ESP_OK) {
        return err;
    }
    return send_built_frame(ctx, &frame);
}

esp_err_t ax25_agwpe_client_request_heard(uint8_t port,
                                          ax25_agwpe_client_t *ctx)
{
    agwpe_frame_t frame;
    agwpe_build_heard_req(port, &frame);
    return send_built_frame(ctx, &frame);
}