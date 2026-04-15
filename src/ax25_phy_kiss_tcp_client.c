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
 * @file ax25_phy_kiss_tcp_client.c
 * @brief TCP KISS client physical-layer driver
 */

#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "ax25_buffer.h"
#include "ax25_frame.h"
#include "ax25_print.h"
#include "ax25_phy_kiss_tcp_client.h"

static const char *TAG = "AX25_PHY_TCP";

#define AX25_PHY_TCP_LOG_FRAMES CONFIG_AX25_PHY_TCP_LOG_FRAMES
#define AX25_PHY_TCP_DEFAULT_CONNECT_TIMEOUT_MS 6000u
#define AX25_PHY_TCP_DEFAULT_RECONNECT_DELAY_MS 5000u
#define AX25_PHY_TCP_DEFAULT_TASK_STACK_SIZE 4096u
#define AX25_PHY_TCP_DEFAULT_TASK_PRIORITY 5u
#define AX25_PHY_TCP_DEFAULT_TX_QUEUE_DEPTH 8u

typedef struct {
    size_t len;
    uint8_t data[AX25_KISS_MAX_ENCODED_SIZE];
} ax25_phy_kiss_tcp_client_tx_item_t;

/* -------------------------------------------------------------------------
 * Private helpers
 * ---------------------------------------------------------------------- */

/**
 * Called by the KISS decoder for each complete KISS frame.
 * Parses the AX.25 content and delivers it to the application callback.
 */
static void kiss_frame_cb(uint8_t port_num, uint8_t command,
                          const uint8_t *data, size_t len, void *user_data)
{
    (void)port_num;
    if (command != 0) {
        return; /* ignore non-data commands */
    }

    ax25_phy_kiss_tcp_client_t *ctx = (ax25_phy_kiss_tcp_client_t *)user_data;

    ax25_frame_t frame;
    if (ax25_frame_parse(data, len, &frame) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to parse incoming frame (%zu bytes)", len);
        return;
    }
#if AX25_PHY_TCP_LOG_FRAMES
    ax25_print_frame("tcp_recv", &frame);
#endif
    ctx->on_rx_frame(&frame, ctx->user_data);
}

/**
 * @brief Non-blocking connect with a millisecond timeout.
 * @return 0 on success, -1 on failure (errno set).
 */
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
        .tv_sec  = timeout_ms / 1000,
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

static void tcp_tx_task(void *arg)
{
    ax25_phy_kiss_tcp_client_t *ctx = (ax25_phy_kiss_tcp_client_t *)arg;
    ax25_phy_kiss_tcp_client_tx_item_t item;

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

/** Receive task: manages the TCP connection and feeds received bytes into the
 *  KISS decoder.  Reconnects automatically on disconnection. */
static void tcp_rx_task(void *arg)
{
    ax25_phy_kiss_tcp_client_t *ctx = (ax25_phy_kiss_tcp_client_t *)arg;
    uint8_t rx_buf[256];

    while (ctx->running) {
        /* ---- DNS resolution ---- */
        struct addrinfo hints = {
            .ai_family   = AF_INET,
            .ai_socktype = SOCK_STREAM,
        };
        struct addrinfo *res = NULL;
        char port_str[8];
        snprintf(port_str, sizeof(port_str), "%u", (unsigned)ctx->port);

        if (getaddrinfo(ctx->host, port_str, &hints, &res) != 0 || res == NULL) {
            ESP_LOGW(TAG, "DNS lookup failed for %s", ctx->host);
            goto reconnect_delay;
        }

        /* ---- Socket creation ---- */
        int sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (sock < 0) {
            ESP_LOGW(TAG, "Socket create failed: %s", strerror(errno));
            freeaddrinfo(res);
            goto reconnect_delay;
        }

        /* 1-second receive timeout so the inner loop can check ctx->running. */
        struct timeval recv_tv = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &recv_tv, sizeof(recv_tv));

        struct timeval send_tv = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &send_tv, sizeof(send_tv));

        /* ---- Connect ---- */
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

        /* Publish the active socket so _send can use it. */
        xSemaphoreTake(ctx->send_mutex, portMAX_DELAY);
        ctx->sock = sock;
        xSemaphoreGive(ctx->send_mutex);

        ax25_kiss_decoder_reset(&ctx->kiss_decoder);

        /* ---- Receive loop ---- */
        while (ctx->running) {
            ssize_t len = recv(sock, rx_buf, sizeof(rx_buf), 0);
            if (len > 0) {
                ax25_kiss_decoder_process_bytes(&ctx->kiss_decoder, rx_buf, (size_t)len);
                continue;
            }
            if (len == 0) {
                ESP_LOGW(TAG, "Connection closed by %s:%u",
                         ctx->host, (unsigned)ctx->port);
                break;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue; /* recv timeout — check ctx->running */
            }
            if (ctx->running) {
                /* Suppress error log when deinit closed the socket. */
                ESP_LOGW(TAG, "recv error: %s", strerror(errno));
            }
            break;
        }

        /* Retract the socket before closing so _send stops using it. */
        xSemaphoreTake(ctx->send_mutex, portMAX_DELAY);
        ctx->sock = -1;
        xSemaphoreGive(ctx->send_mutex);

        close(sock);

        if (ctx->running) {
            ESP_LOGW(TAG, "Disconnected from %s:%u, reconnecting...",
                     ctx->host, (unsigned)ctx->port);
        }

reconnect_delay:
        /* Delay in short slices so deinit can interrupt quickly. */
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

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t ax25_phy_kiss_tcp_client_init(const ax25_phy_kiss_tcp_client_config_t *config,
                                        ax25_phy_kiss_tcp_client_t *ctx)
{
    if (!ctx || !config || !config->host || config->port == 0 || !config->on_rx_frame) {
        return ESP_ERR_INVALID_ARG;
    }

    ctx->host                = config->host;
    ctx->port                = config->port;
    ctx->on_rx_frame            = config->on_rx_frame;
    ctx->user_data           = config->user_data;
    ctx->connect_timeout_ms  = config->connect_timeout_ms ? config->connect_timeout_ms : AX25_PHY_TCP_DEFAULT_CONNECT_TIMEOUT_MS;
    ctx->reconnect_delay_ms  = config->reconnect_delay_ms ? config->reconnect_delay_ms : AX25_PHY_TCP_DEFAULT_RECONNECT_DELAY_MS;
    ctx->sock                = -1;
    ctx->running             = false;
    ctx->rx_task_handle      = NULL;
    ctx->tx_task_handle      = NULL;
    ctx->tx_queue            = NULL;

    uint32_t    rx_stk   = config->rx_task_stack_size ? config->rx_task_stack_size : AX25_PHY_TCP_DEFAULT_TASK_STACK_SIZE;
    UBaseType_t rx_prio  = config->rx_task_priority   ? config->rx_task_priority   : AX25_PHY_TCP_DEFAULT_TASK_PRIORITY;
    uint32_t    tx_stk   = config->tx_task_stack_size ? config->tx_task_stack_size : AX25_PHY_TCP_DEFAULT_TASK_STACK_SIZE;
    UBaseType_t tx_prio  = config->tx_task_priority   ? config->tx_task_priority   : AX25_PHY_TCP_DEFAULT_TASK_PRIORITY;
    uint32_t    tx_depth = config->tx_queue_depth     ? config->tx_queue_depth     : AX25_PHY_TCP_DEFAULT_TX_QUEUE_DEPTH;

    ctx->send_mutex = xSemaphoreCreateMutex();
    if (ctx->send_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ctx->tx_queue = xQueueCreate(tx_depth, sizeof(ax25_phy_kiss_tcp_client_tx_item_t));
    if (ctx->tx_queue == NULL) {
        vSemaphoreDelete(ctx->send_mutex);
        ctx->send_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    ax25_kiss_decoder_init(&ctx->kiss_decoder, kiss_frame_cb, ctx);

    ctx->running = true;

    if (xTaskCreate(tcp_tx_task, "tcp_tx_task", tx_stk, ctx, tx_prio,
                    &ctx->tx_task_handle) != pdPASS) {
        ctx->running = false;
        vQueueDelete(ctx->tx_queue);
        ctx->tx_queue = NULL;
        vSemaphoreDelete(ctx->send_mutex);
        ctx->send_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(tcp_rx_task, "tcp_rx_task", rx_stk, ctx, rx_prio,
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

void ax25_phy_kiss_tcp_client_deinit(ax25_phy_kiss_tcp_client_t *ctx)
{
    if (!ctx || !ctx->running) {
        return;
    }

    ctx->running = false;

    /* Shut down the active socket (without closing it) to unblock recv()
     * in the RX task.  The task is responsible for calling close(). */
    xSemaphoreTake(ctx->send_mutex, portMAX_DELAY);
    int sock = ctx->sock;
    xSemaphoreGive(ctx->send_mutex);

    if (sock >= 0) {
        shutdown(sock, SHUT_RDWR);
    }

    /* Wait for the RX/TX tasks to exit (each checks running at least every 1000 ms). */
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

    vSemaphoreDelete(ctx->send_mutex);
    ctx->send_mutex = NULL;
}

esp_err_t ax25_phy_kiss_tcp_client_send(const ax25_frame_t *frame,
                                        ax25_phy_kiss_tcp_client_t *ctx)
{
    if (!ctx || !frame) {
        return ESP_ERR_INVALID_ARG;
    }

#if AX25_PHY_TCP_LOG_FRAMES
    ax25_print_frame("tcp_send", frame);
#endif

    ax25_buffer_t raw = {0};
    if (ax25_frame_build(frame, &raw) != ESP_OK) {
        ESP_LOGE(TAG, "ax25_frame_build failed");
        return ESP_FAIL;
    }

    uint8_t encoded[AX25_KISS_MAX_ENCODED_SIZE];
    size_t encoded_len = ax25_kiss_encode(0, 0, raw.data, raw.len,
                                          encoded, sizeof(encoded));
    if (encoded_len == 0) {
        ESP_LOGE(TAG, "ax25_kiss_encode failed");
        return ESP_FAIL;
    }

    if (ctx->send_mutex == NULL || ctx->tx_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(ctx->send_mutex, portMAX_DELAY);

    if (ctx->sock < 0) {
        xSemaphoreGive(ctx->send_mutex);
        ESP_LOGW(TAG, "send: not connected");
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreGive(ctx->send_mutex);

    ax25_phy_kiss_tcp_client_tx_item_t item = {
        .len = encoded_len,
    };
    memcpy(item.data, encoded, encoded_len);

    if (xQueueSend(ctx->tx_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "send: tx queue full");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
