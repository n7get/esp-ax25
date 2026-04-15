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
 * @file ax25_kiss_tcp_server.c
 * @brief KISS-over-TCP server built on ax25_phy_tcp_server
 */

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "ax25_buffer.h"
#include "ax25_config.h"
#include "ax25_frame.h"
#include "ax25_kiss.h"
#include "ax25_kiss_tcp_server.h"
#include "ax25_print.h"

static const char *TAG = "AX25_KISS_TCP_SRV";

#define AX25_PHY_TCP_LOG_FRAMES CONFIG_AX25_PHY_TCP_LOG_FRAMES

static void *kiss_server_calloc_prefer_psram(size_t count, size_t elem_size)
{
#if CONFIG_SPIRAM
    void *ptr = heap_caps_calloc(count, elem_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr != NULL) {
        return ptr;
    }
#endif

    return calloc(count, elem_size);
}


static void sync_runtime_state(ax25_kiss_tcp_server_t *srv)
{
    srv->running = srv->tcp_server.running;
    srv->listen_sock = srv->tcp_server.listen_sock;
    srv->accept_task_handle = srv->tcp_server.accept_task_handle;
}

static ax25_kiss_tcp_server_conn_t *wrap_conn_from_tcp(
    ax25_kiss_tcp_server_t *srv,
    ax25_phy_tcp_server_conn_t *tcp_conn)
{
    ptrdiff_t idx = tcp_conn - srv->tcp_server.conns;
    if (idx < 0 || (size_t)idx >= srv->tcp_server.max_clients) {
        return NULL;
    }

    return &srv->conns[idx];
}

static void kiss_tcp_server_rx_frame_cb(uint8_t port_num, uint8_t command,
                                        const uint8_t *data, size_t len, void *user_data)
{
    (void)port_num;
    if (command != 0) {
        return;
    }

    ax25_kiss_tcp_server_conn_t *conn = (ax25_kiss_tcp_server_conn_t *)user_data;
    if (conn == NULL || conn->server == NULL) {
        return;
    }

    ax25_frame_t frame;
    if (ax25_frame_parse(data, len, &frame) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to parse incoming frame (%zu bytes)", len);
        return;
    }

#if AX25_PHY_TCP_LOG_FRAMES
    ax25_print_frame("tcp_srv_recv", &frame);
#endif

    if (conn->server->on_rx_frame) {
        conn->server->on_rx_frame(conn, &frame);
    }
}

static void on_tcp_connected(ax25_phy_tcp_server_conn_t *tcp_conn)
{
    ax25_kiss_tcp_server_t *srv =
        (ax25_kiss_tcp_server_t *)ax25_phy_tcp_server_get_user_data(tcp_conn);
    if (srv == NULL) {
        return;
    }

    ax25_kiss_tcp_server_conn_t *conn = wrap_conn_from_tcp(srv, tcp_conn);
    if (conn == NULL) {
        return;
    }

    conn->server = srv;
    conn->tcp_conn = tcp_conn;
    conn->in_use = true;
    conn->running = true;
    conn->user_data = NULL;
    ax25_kiss_decoder_reset(&conn->kiss_decoder);
    ax25_phy_tcp_server_conn_set_user_data(tcp_conn, conn);

    sync_runtime_state(srv);

    if (srv->on_connected) {
        srv->on_connected(conn);
    }
}

static void on_tcp_disconnected(ax25_phy_tcp_server_conn_t *tcp_conn)
{
    ax25_kiss_tcp_server_conn_t *conn =
        (ax25_kiss_tcp_server_conn_t *)ax25_phy_tcp_server_conn_get_user_data(tcp_conn);
    if (conn == NULL || conn->server == NULL) {
        return;
    }

    ax25_kiss_tcp_server_t *srv = conn->server;

    if (srv->on_disconnected) {
        srv->on_disconnected(conn);
    }

    conn->running = false;
    conn->in_use = false;
    conn->tcp_conn = NULL;
    conn->user_data = NULL;
    ax25_phy_tcp_server_conn_set_user_data(tcp_conn, NULL);

    sync_runtime_state(srv);
}

static void kiss_tcp_server_rx_bytes_cb(ax25_phy_tcp_server_conn_t *tcp_conn,
                                        const uint8_t *data,
                                        size_t len)
{
    ax25_kiss_tcp_server_conn_t *conn =
        (ax25_kiss_tcp_server_conn_t *)ax25_phy_tcp_server_conn_get_user_data(tcp_conn);
    if (conn == NULL) {
        return;
    }

    ax25_kiss_decoder_process_bytes(&conn->kiss_decoder, data, len);
}

esp_err_t ax25_kiss_tcp_server_init(ax25_kiss_tcp_server_on_connected_t on_connected,
                                    ax25_kiss_tcp_server_on_disconnected_t on_disconnected,
                                    ax25_kiss_tcp_server_on_rx_frame_t on_rx_frame,
                                    void *user_data,
                                    ax25_kiss_tcp_server_t *ctx)
{
    if (ctx == NULL || on_rx_frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(ctx, 0, sizeof(*ctx));

    /* Read configuration from ax25_config */
    uint16_t port = (uint16_t)ax25_cfg_get_int_global("net.kiss.port", 8000);
    if (port == 0) {
        ESP_LOGE(TAG, "KISS TCP server port must not be 0");
        return ESP_ERR_INVALID_ARG;
    }

    ctx->port = port;
    ctx->on_connected = on_connected;
    ctx->on_disconnected = on_disconnected;
    ctx->on_rx_frame = on_rx_frame;
    ctx->user_data = user_data;
    ctx->conn_task_stack_size = (uint32_t)ax25_cfg_get_int_global("net.kiss.conn_task_stack", 4096);
    if (ctx->conn_task_stack_size == 0) {
        ctx->conn_task_stack_size = 4096;
    }
    ctx->conn_task_priority = (UBaseType_t)ax25_cfg_get_int_global("net.kiss.conn_task_priority", 5);
    if (ctx->conn_task_priority == 0) {
        ctx->conn_task_priority = 5;
    }

    esp_err_t err = ax25_phy_tcp_server_init("net.kiss.port",
                                             on_tcp_connected,
                                             on_tcp_disconnected,
                                             kiss_tcp_server_rx_bytes_cb,
                                             ctx,
                                             &ctx->tcp_server);
    if (err != ESP_OK) {
        sync_runtime_state(ctx);
        return err;
    }

    ctx->conns = kiss_server_calloc_prefer_psram(ctx->tcp_server.max_clients,
                                                 sizeof(ax25_kiss_tcp_server_conn_t));
    if (ctx->conns == NULL) {
        ax25_phy_tcp_server_deinit(&ctx->tcp_server);
        sync_runtime_state(ctx);
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < ctx->tcp_server.max_clients; i++) {
        ctx->conns[i].server = ctx;
        ctx->conns[i].tcp_conn = NULL;
        ctx->conns[i].in_use = false;
        ctx->conns[i].running = false;
        ctx->conns[i].user_data = NULL;
        ax25_kiss_decoder_init(&ctx->conns[i].kiss_decoder,
                       kiss_tcp_server_rx_frame_cb,
                       &ctx->conns[i]);
    }

    sync_runtime_state(ctx);
    return ESP_OK;
}

void ax25_kiss_tcp_server_deinit(ax25_kiss_tcp_server_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ax25_phy_tcp_server_deinit(&ctx->tcp_server);
    sync_runtime_state(ctx);

    for (size_t i = 0; i < ctx->tcp_server.max_clients; i++) {
        ctx->conns[i].running = false;
        ctx->conns[i].tcp_conn = NULL;
        ctx->conns[i].in_use = false;
        ctx->conns[i].user_data = NULL;
        ax25_kiss_decoder_reset(&ctx->conns[i].kiss_decoder);
    }

    free(ctx->conns);
    ctx->conns = NULL;
}

esp_err_t ax25_kiss_tcp_server_conn_send(ax25_kiss_tcp_server_conn_t *conn,
                                         const ax25_frame_t *frame)
{
    if (conn == NULL || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (conn->tcp_conn == NULL || !conn->in_use || !conn->running) {
        return ESP_ERR_INVALID_STATE;
    }

#if AX25_PHY_TCP_LOG_FRAMES
    ax25_print_frame("tcp_srv_send", frame);
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

    return ax25_phy_tcp_server_conn_send(conn->tcp_conn, encoded, encoded_len);
}
