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

#include "nvs_flash.h"

#include "ax25_config.h"

static void ensure_nvs_ready(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_erase());
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_init());
        return;
    }

    TEST_ASSERT_EQUAL(ESP_OK, err);
}

static void reset_config_state(void)
{
    ax25_cfg_deinit();
    ensure_nvs_ready();
}

TEST_CASE("Config: init merges local schema with library schema", "[ax25_config]")
{
    reset_config_state();

    static const ax25_cfg_param_t local_schema[] = {
        {
            .parameter = "example.mode",
            .nvs_key = "example_mode",
            .default_value = "term",
            .range = "kiss term",
            .type = AX25_CFG_TYPE_ENUM,
            .hide = false,
        },
        {
            .parameter = "example.secret",
            .nvs_key = "example_secret",
            .default_value = "",
            .range = "",
            .type = AX25_CFG_TYPE_STRING,
            .hide = true,
        },
    };

    TEST_ASSERT_EQUAL(ESP_OK,
                      ax25_cfg_init(local_schema,
                                    sizeof(local_schema) / sizeof(local_schema[0])));

    char value[32] = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_get("example.mode", value, sizeof(value)));
    TEST_ASSERT_EQUAL_STRING("term", value);

    memset(value, 0, sizeof(value));
    TEST_ASSERT_EQUAL(ESP_OK, ax25_cfg_get("net.kiss.port", value, sizeof(value)));
    TEST_ASSERT_EQUAL_STRING("8100", value);

    ax25_cfg_deinit();
}

TEST_CASE("Config: init rejects duplicate parameter names", "[ax25_config]")
{
    reset_config_state();

    static const ax25_cfg_param_t local_schema[] = {
        {
            .parameter = "net.kiss.port",
            .nvs_key = "example_dup_param",
            .default_value = "9999",
            .range = "1 65535",
            .type = AX25_CFG_TYPE_INT,
            .hide = false,
        },
    };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_cfg_init(local_schema,
                                    sizeof(local_schema) / sizeof(local_schema[0])));
}

TEST_CASE("Config: init rejects duplicate NVS keys", "[ax25_config]")
{
    reset_config_state();

    static const ax25_cfg_param_t local_schema[] = {
        {
            .parameter = "example.alt.port",
            .nvs_key = "net_kiss_port",
            .default_value = "9999",
            .range = "1 65535",
            .type = AX25_CFG_TYPE_INT,
            .hide = false,
        },
    };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      ax25_cfg_init(local_schema,
                                    sizeof(local_schema) / sizeof(local_schema[0])));
}