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

#include "ax25_log.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "ax25_router.h"
#include "ax25_print.h"
#include "ax25_frame.h"
#include "ax25_buffer.h"

static const char *TAG = "AX25_LOG";

/* Static router port storage — no heap allocation */
static ax25_router_port_t s_console_port_storage;
static ax25_router_port_t s_udp_port_storage;

typedef struct {
    int                udp_sock;
    struct sockaddr_in udp_addr;
    ax25_router_port_t *udp_port;
    ax25_router_port_t *console_port;
} ax25_log_state_t;

static ax25_log_state_t s_log = {
    .udp_sock = -1,
    .udp_port = NULL,
    .console_port = NULL,
};

static void log_console_cb(const ax25_frame_t *frame, void *user_data)
{
    if (frame == NULL) {
        return;
    }
    
    ax25_print_frame("router", frame);
}

static void log_udp_cb(const ax25_frame_t *frame, void *user_data)
{
    if (frame == NULL || s_log.udp_sock < 0) {
        return;
    }

    ax25_buffer_t raw = {0};
    if (ax25_frame_build(frame, &raw) != ESP_OK) {
        return;
    }

    int sent = sendto(s_log.udp_sock,
                      raw.data,
                      raw.len,
                      0,
                      (const struct sockaddr *)&s_log.udp_addr,
                      sizeof(s_log.udp_addr));
    if (sent < 0) {
        ESP_LOGW(TAG, "UDP send failed: errno=%d", errno);
    }
}

static esp_err_t resolve_udp_target(const char *host, uint16_t port, struct sockaddr_in *out)
{
    if (host == NULL || out == NULL || port == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    out->sin_family = AF_INET;
    out->sin_port = htons(port);

    if (inet_pton(AF_INET, host, &out->sin_addr) == 1) {
        return ESP_OK;
    }

    struct hostent *he = gethostbyname(host);
    if (he == NULL || he->h_addr_list == NULL || he->h_addr_list[0] == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(&out->sin_addr, he->h_addr_list[0], sizeof(out->sin_addr));
    return ESP_OK;
}

esp_err_t ax25_log_console(void)
{
    if (s_log.console_port) {
        return ESP_OK;
    }

    memset(&s_console_port_storage, 0, sizeof(s_console_port_storage));
    s_log.console_port = &s_console_port_storage;

    s_log.console_port->on_tx_frame = log_console_cb;
    s_log.console_port->mode = AX25_PORT_PROMISCUOUS;

    esp_err_t err = ax25_router_register_port(s_log.console_port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register console log port: %d", err);
        s_log.console_port = NULL;
        return err;
    }

    return ESP_OK;
}

esp_err_t ax25_log_udp(const char *host, uint16_t port)
{
    if (s_log.udp_port) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = resolve_udp_target(host, port, &s_log.udp_addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to resolve UDP target %s:%u", host ? host : "(null)", (unsigned)port);
        return err;
    }

    s_log.udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_log.udp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return ESP_FAIL;
    }

    memset(&s_udp_port_storage, 0, sizeof(s_udp_port_storage));
    s_log.udp_port = &s_udp_port_storage;

    s_log.udp_port->on_tx_frame = log_udp_cb;
    s_log.udp_port->mode = AX25_PORT_PROMISCUOUS;

    err = ax25_router_register_port(s_log.udp_port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register UDP log port: %d", err);
        s_log.udp_port = NULL;
        close(s_log.udp_sock);
        s_log.udp_sock = -1;
        return err;
    }

    return ESP_OK;
}

void ax25_log_deinit(void)
{
    if (s_log.console_port) {
        ax25_router_remove_port(s_log.console_port);
        /* Static storage — no free needed */
        s_log.console_port = NULL;
    }

    if (s_log.udp_port) {
        ax25_router_remove_port(s_log.udp_port);
        /* Static storage — no free needed */
        s_log.udp_port = NULL;
    }

    if (s_log.udp_sock >= 0) {
        close(s_log.udp_sock);
        s_log.udp_sock = -1;
    }
}
