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

#include "ax25_config.h"
#include "ax25_config_defaults.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs.h"

/* -----------------------------------------------------------------------
 * Internal types — not exposed in the public header.
 * -------------------------------------------------------------------- */

typedef struct {
    uint16_t param_index;
    bool in_use;
    bool cleared;
    char value[AX25_CFG_VALUE_MAX_LEN];
} ax25_cfg_pending_t;

typedef struct ax25_cfg_ctx {
    const ax25_cfg_param_t *schema;
    size_t schema_count;
    ax25_cfg_param_t *schema_storage;

    char nvs_namespace[16];
    char nvs_backup_namespace[16];
    nvs_handle_t nvs_primary;
    nvs_handle_t nvs_backup;

    uint32_t commit_timeout_ms;
    TimerHandle_t commit_timer;
    bool commit_pending;

    ax25_cfg_pending_t pending[AX25_CFG_MAX_PENDING];

    ax25_cfg_output_fn_t out_fn;
    void *out_arg;
    char out_buf[512];
    size_t out_len;
} ax25_cfg_ctx_t;

static ax25_cfg_ctx_t s_ctx;

static const char *TAG = "ax25_cfg";
static const char NVS_PENDING_KEY[] = "cfg_pending";

static esp_err_t build_merged_schema(const ax25_cfg_param_t *local_schema,
                                     size_t local_schema_count,
                                     ax25_cfg_param_t **out_schema,
                                     size_t *out_schema_count);

static void cfg_output_flush(ax25_cfg_ctx_t *ctx)
{
    if (ctx == NULL || ctx->out_fn == NULL || ctx->out_len == 0) {
        return;
    }

    ctx->out_fn(ctx->out_buf, ctx->out_len, ctx->out_arg);
    ctx->out_len = 0;
}

static void cfg_output_append(ax25_cfg_ctx_t *ctx, const char *text, size_t len)
{
    if (ctx == NULL || ctx->out_fn == NULL || text == NULL || len == 0) {
        return;
    }

    size_t pos = 0;
    while (pos < len) {
        size_t avail = sizeof(ctx->out_buf) - ctx->out_len;
        if (avail == 0) {
            cfg_output_flush(ctx);
            avail = sizeof(ctx->out_buf);
        }

        size_t chunk = len - pos;
        if (chunk > avail) {
            chunk = avail;
        }

        memcpy(&ctx->out_buf[ctx->out_len], &text[pos], chunk);
        ctx->out_len += chunk;
        pos += chunk;
    }
}

static void cfg_output(ax25_cfg_ctx_t *ctx, const char *fmt, ...)
{
    if (ctx == NULL || ctx->out_fn == NULL || fmt == NULL) {
        return;
    }

    char line[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }

    size_t len = (size_t)n;
    if (len >= sizeof(line)) {
        len = sizeof(line) - 1;
    }

    cfg_output_append(ctx, line, len);
}

static const ax25_cfg_param_t *find_param(const ax25_cfg_ctx_t *ctx, const char *parameter, size_t *idx)
{
    if (ctx == NULL || parameter == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < ctx->schema_count; i++) {
        const ax25_cfg_param_t *p = &ctx->schema[i];
        if (p->parameter != NULL && strcmp(p->parameter, parameter) == 0) {
            if (idx != NULL) {
                *idx = i;
            }
            return p;
        }
    }
    return NULL;
}

static int find_pending_slot(ax25_cfg_ctx_t *ctx, uint16_t param_index)
{
    for (int i = 0; i < AX25_CFG_MAX_PENDING; i++) {
        if (ctx->pending[i].in_use && ctx->pending[i].param_index == param_index) {
            return i;
        }
    }
    return -1;
}

static int alloc_pending_slot(ax25_cfg_ctx_t *ctx)
{
    for (int i = 0; i < AX25_CFG_MAX_PENDING; i++) {
        if (!ctx->pending[i].in_use) {
            return i;
        }
    }
    return -1;
}

static const char *param_nvs_key(const ax25_cfg_param_t *p)
{
    return (p != NULL && p->nvs_key != NULL && p->nvs_key[0] != '\0') ? p->nvs_key : p->parameter;
}

static esp_err_t build_merged_schema(const ax25_cfg_param_t *local_schema,
                                     size_t local_schema_count,
                                     ax25_cfg_param_t **out_schema,
                                     size_t *out_schema_count)
{
    if (out_schema == NULL || out_schema_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_schema = NULL;
    *out_schema_count = 0;

    if (local_schema_count > 0 && local_schema == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total_schema_count = ax25_cfg_defaults_schema_count + local_schema_count;
    if (total_schema_count == 0 || total_schema_count > UINT16_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    ax25_cfg_param_t *merged_schema = calloc(total_schema_count, sizeof(*merged_schema));
    if (merged_schema == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memcpy(merged_schema,
           ax25_cfg_defaults_schema,
           ax25_cfg_defaults_schema_count * sizeof(*merged_schema));
    if (local_schema_count > 0) {
        memcpy(merged_schema + ax25_cfg_defaults_schema_count,
               local_schema,
               local_schema_count * sizeof(*merged_schema));
    }

    for (size_t i = 0; i < total_schema_count; i++) {
        const ax25_cfg_param_t *param = &merged_schema[i];
        const char *param_key = param_nvs_key(param);

        if (param->parameter == NULL || param->parameter[0] == '\0') {
            ESP_LOGE(TAG, "config schema entry %u has empty parameter name", (unsigned)i);
            free(merged_schema);
            return ESP_ERR_INVALID_ARG;
        }
        if (param_key == NULL || param_key[0] == '\0') {
            ESP_LOGE(TAG, "config schema entry '%s' has empty NVS key", param->parameter);
            free(merged_schema);
            return ESP_ERR_INVALID_ARG;
        }

        for (size_t j = i + 1; j < total_schema_count; j++) {
            const ax25_cfg_param_t *other = &merged_schema[j];
            const char *other_key = param_nvs_key(other);

            if (strcmp(param->parameter, other->parameter) == 0) {
                ESP_LOGE(TAG,
                         "duplicate config parameter '%s' in merged schema",
                         param->parameter);
                free(merged_schema);
                return ESP_ERR_INVALID_ARG;
            }
            if (strcmp(param_key, other_key) == 0) {
                ESP_LOGE(TAG,
                         "duplicate config NVS key '%s' in merged schema",
                         param_key);
                free(merged_schema);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    *out_schema = merged_schema;
    *out_schema_count = total_schema_count;
    return ESP_OK;
}

static bool parse_int_range(const char *range, long *out_min, long *out_max)
{
    if (range == NULL || out_min == NULL || out_max == NULL) {
        return false;
    }

    char tmp[64];
    strlcpy(tmp, range, sizeof(tmp));

    for (size_t i = 0; tmp[i] != '\0'; i++) {
        if (tmp[i] == ',') {
            tmp[i] = ' ';
        }
    }

    char *saveptr = NULL;
    char *a = strtok_r(tmp, " ", &saveptr);
    char *b = strtok_r(NULL, " ", &saveptr);
    if (a == NULL || b == NULL) {
        return false;
    }

    char *end = NULL;
    long minv = strtol(a, &end, 10);
    if (end == NULL || *end != '\0') {
        return false;
    }
    long maxv = strtol(b, &end, 10);
    if (end == NULL || *end != '\0') {
        return false;
    }

    *out_min = minv;
    *out_max = maxv;
    return true;
}

static bool token_equals_ci(const char *a, const char *b)
{
    if (a == NULL || b == NULL) {
        return false;
    }
    while (*a != '\0' && *b != '\0') {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return false;
        }
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static bool value_in_enum_range(const char *value, const char *range)
{
    if (value == NULL || range == NULL || range[0] == '\0') {
        return false;
    }

    char tmp[192];
    strlcpy(tmp, range, sizeof(tmp));

    for (size_t i = 0; tmp[i] != '\0'; i++) {
        if (tmp[i] == ',') {
            tmp[i] = ' ';
        }
    }

    char *saveptr = NULL;
    for (char *tok = strtok_r(tmp, " ", &saveptr); tok != NULL; tok = strtok_r(NULL, " ", &saveptr)) {
        if (tok[0] == '\0') {
            continue;
        }
        if (token_equals_ci(tok, value)) {
            return true;
        }
    }
    return false;
}

static esp_err_t normalize_value(const ax25_cfg_param_t *param,
                                 const char *value,
                                 char *normalized,
                                 size_t normalized_len,
                                 char *err,
                                 size_t err_len)
{
    if (param == NULL || value == NULL || normalized == NULL || normalized_len == 0) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "invalid arguments");
        }
        return ESP_ERR_INVALID_ARG;
    }

    switch (param->type) {
        case AX25_CFG_TYPE_INT: {
            char *end = NULL;
            long v = strtol(value, &end, 10);
            if (end == NULL || *end != '\0') {
                if (err != NULL && err_len > 0) {
                    snprintf(err, err_len, "'%s' is not an integer", value);
                }
                return ESP_ERR_INVALID_ARG;
            }
            if (param->range != NULL && param->range[0] != '\0') {
                long minv = 0;
                long maxv = 0;
                if (parse_int_range(param->range, &minv, &maxv) && (v < minv || v > maxv)) {
                    if (err != NULL && err_len > 0) {
                        snprintf(err, err_len, "'%s' out of range (%ld..%ld)", value, minv, maxv);
                    }
                    return ESP_ERR_INVALID_ARG;
                }
            }
            snprintf(normalized, normalized_len, "%ld", v);
            return ESP_OK;
        }
        case AX25_CFG_TYPE_BOOL: {
            if (token_equals_ci(value, "1") || token_equals_ci(value, "true") || token_equals_ci(value, "yes")) {
                strlcpy(normalized, "1", normalized_len);
                return ESP_OK;
            }
            if (token_equals_ci(value, "0") || token_equals_ci(value, "false") || token_equals_ci(value, "no")) {
                strlcpy(normalized, "0", normalized_len);
                return ESP_OK;
            }
            if (err != NULL && err_len > 0) {
                snprintf(err, err_len, "bool must be 0/1/true/false");
            }
            return ESP_ERR_INVALID_ARG;
        }
        case AX25_CFG_TYPE_ENUM: {
            if (param->range == NULL || param->range[0] == '\0') {
                if (err != NULL && err_len > 0) {
                    snprintf(err, err_len, "enum range missing");
                }
                return ESP_ERR_INVALID_ARG;
            }
            if (!value_in_enum_range(value, param->range)) {
                if (err != NULL && err_len > 0) {
                    snprintf(err, err_len, "'%s' not in enum {%s}", value, param->range);
                }
                return ESP_ERR_INVALID_ARG;
            }
            strlcpy(normalized, value, normalized_len);
            return ESP_OK;
        }
        case AX25_CFG_TYPE_STRING:
            strlcpy(normalized, value, normalized_len);
            return ESP_OK;
        default:
            if (err != NULL && err_len > 0) {
                snprintf(err, err_len, "unsupported type");
            }
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t get_stored_value(ax25_cfg_ctx_t *ctx, const ax25_cfg_param_t *param, char *out, size_t out_len)
{
    if (ctx == NULL || param == NULL || out == NULL || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t required = out_len;
    esp_err_t err = nvs_get_str(ctx->nvs_primary, param_nvs_key(param), out, &required);
    if (err == ESP_OK) {
        return ESP_OK;
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        strlcpy(out, param->default_value != NULL ? param->default_value : "", out_len);
        return ESP_OK;
    }
    return err;
}

static esp_err_t get_effective_value(ax25_cfg_ctx_t *ctx,
                                     const ax25_cfg_param_t *param,
                                     uint16_t param_index,
                                     char *out,
                                     size_t out_len)
{
    int slot = find_pending_slot(ctx, param_index);
    if (slot >= 0) {
        if (ctx->pending[slot].cleared) {
            strlcpy(out, param->default_value != NULL ? param->default_value : "", out_len);
        } else {
            strlcpy(out, ctx->pending[slot].value, out_len);
        }
        return ESP_OK;
    }

    return get_stored_value(ctx, param, out, out_len);
}

static esp_err_t mark_pending_set(ax25_cfg_ctx_t *ctx, uint16_t param_index, const char *value)
{
    int slot = find_pending_slot(ctx, param_index);
    if (slot < 0) {
        slot = alloc_pending_slot(ctx);
        if (slot < 0) {
            return ESP_ERR_NO_MEM;
        }
        memset(&ctx->pending[slot], 0, sizeof(ctx->pending[slot]));
        ctx->pending[slot].in_use = true;
        ctx->pending[slot].param_index = param_index;
    }

    ctx->pending[slot].cleared = false;
    strlcpy(ctx->pending[slot].value, value, sizeof(ctx->pending[slot].value));
    return ESP_OK;
}

static esp_err_t mark_pending_clear(ax25_cfg_ctx_t *ctx, uint16_t param_index)
{
    int slot = find_pending_slot(ctx, param_index);
    if (slot < 0) {
        slot = alloc_pending_slot(ctx);
        if (slot < 0) {
            return ESP_ERR_NO_MEM;
        }
        memset(&ctx->pending[slot], 0, sizeof(ctx->pending[slot]));
        ctx->pending[slot].in_use = true;
        ctx->pending[slot].param_index = param_index;
    }

    ctx->pending[slot].cleared = true;
    ctx->pending[slot].value[0] = '\0';
    return ESP_OK;
}

static void clear_pending(ax25_cfg_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    memset(ctx->pending, 0, sizeof(ctx->pending));
}

static esp_err_t copy_primary_to_backup(ax25_cfg_ctx_t *ctx)
{
    for (size_t i = 0; i < ctx->schema_count; i++) {
        const ax25_cfg_param_t *p = &ctx->schema[i];
        const char *key = param_nvs_key(p);
        char value[AX25_CFG_VALUE_MAX_LEN];
        size_t needed = sizeof(value);

        esp_err_t err = nvs_get_str(ctx->nvs_primary, key, value, &needed);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = nvs_erase_key(ctx->nvs_backup, key);
            if (err == ESP_ERR_NVS_NOT_FOUND) {
                err = ESP_OK;
            }
            if (err != ESP_OK) {
                return err;
            }
            continue;
        }
        if (err != ESP_OK) {
            return err;
        }

        err = nvs_set_str(ctx->nvs_backup, key, value);
        if (err != ESP_OK) {
            return err;
        }
    }

    return nvs_commit(ctx->nvs_backup);
}

static esp_err_t restore_backup_to_primary(ax25_cfg_ctx_t *ctx)
{
    for (size_t i = 0; i < ctx->schema_count; i++) {
        const ax25_cfg_param_t *p = &ctx->schema[i];
        const char *key = param_nvs_key(p);
        char value[AX25_CFG_VALUE_MAX_LEN];
        size_t needed = sizeof(value);

        esp_err_t err = nvs_get_str(ctx->nvs_backup, key, value, &needed);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = nvs_erase_key(ctx->nvs_primary, key);
            if (err == ESP_ERR_NVS_NOT_FOUND) {
                err = ESP_OK;
            }
            if (err != ESP_OK) {
                return err;
            }
            continue;
        }
        if (err != ESP_OK) {
            return err;
        }

        err = nvs_set_str(ctx->nvs_primary, key, value);
        if (err != ESP_OK) {
            return err;
        }
    }

    esp_err_t err = nvs_commit(ctx->nvs_primary);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(ctx->nvs_backup, NVS_PENDING_KEY);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }
    err = nvs_commit(ctx->nvs_backup);
    if (err != ESP_OK) {
        return err;
    }

    ctx->commit_pending = false;
    return ESP_OK;
}

static void commit_timeout_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    ax25_cfg_ctx_t *ctx = &s_ctx;

    esp_err_t err = restore_backup_to_primary(ctx);
    if (err != ESP_OK) {
        return;
    }

    esp_restart();
}

esp_err_t ax25_cfg_init(const ax25_cfg_param_t *local_schema,
                        size_t local_schema_count)
{
    if (s_ctx.schema != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *nvs_namespace = "ax25cfg";
    const char *nvs_backup_namespace = "ax25cfg_bak";
    uint32_t commit_timeout_ms = 300000U;
    ax25_cfg_param_t *schema_storage = NULL;
    size_t schema_count = 0;

    esp_err_t err = build_merged_schema(local_schema,
                                        local_schema_count,
                                        &schema_storage,
                                        &schema_count);
    if (err != ESP_OK) {
        return err;
    }

    ax25_cfg_ctx_t *ctx = &s_ctx;
    memset(ctx, 0, sizeof(*ctx));

    ctx->schema = schema_storage;
    ctx->schema_count = schema_count;
    ctx->schema_storage = schema_storage;
    ctx->commit_timeout_ms = (commit_timeout_ms == 0U) ? 300000U : commit_timeout_ms;

    strlcpy(ctx->nvs_namespace, nvs_namespace, sizeof(ctx->nvs_namespace));
    strlcpy(ctx->nvs_backup_namespace, nvs_backup_namespace, sizeof(ctx->nvs_backup_namespace));

    err = nvs_open(ctx->nvs_namespace, NVS_READWRITE, &ctx->nvs_primary);
    if (err != ESP_OK) {
        free(schema_storage);
        memset(ctx, 0, sizeof(*ctx));
        return err;
    }

    err = nvs_open(ctx->nvs_backup_namespace, NVS_READWRITE, &ctx->nvs_backup);
    if (err != ESP_OK) {
        nvs_close(ctx->nvs_primary);
        free(schema_storage);
        memset(ctx, 0, sizeof(*ctx));
        return err;
    }

    uint8_t pending = 0;
    err = nvs_get_u8(ctx->nvs_backup, NVS_PENDING_KEY, &pending);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ax25_cfg_deinit();
        return err;
    }

    if (pending == 1) {
        ctx->commit_pending = true;
        ctx->commit_timer = xTimerCreate("ax25_cfg_commit",
                                         pdMS_TO_TICKS(ctx->commit_timeout_ms),
                                         pdFALSE,
                                         ctx,
                                         commit_timeout_cb);
        if (ctx->commit_timer == NULL) {
            ax25_cfg_deinit();
            return ESP_ERR_NO_MEM;
        }
        if (xTimerStart(ctx->commit_timer, 0) != pdPASS) {
            ax25_cfg_deinit();
            return ESP_FAIL;
        }
        ESP_LOGW(TAG, "uncommitted config detected; rollback timer started");
    }

    return ESP_OK;
}

void ax25_cfg_deinit(void)
{
    ax25_cfg_ctx_t *ctx = &s_ctx;
    if (ctx->schema == NULL) {
        return;
    }

    if (ctx->commit_timer != NULL) {
        xTimerStop(ctx->commit_timer, 0);
        xTimerDelete(ctx->commit_timer, 0);
        ctx->commit_timer = NULL;
    }

    if (ctx->nvs_primary != 0) {
        nvs_close(ctx->nvs_primary);
    }
    if (ctx->nvs_backup != 0) {
        nvs_close(ctx->nvs_backup);
    }
    free(ctx->schema_storage);

    memset(ctx, 0, sizeof(*ctx));
}

void ax25_cfg_set_output(ax25_cfg_output_fn_t out_fn, void *out_arg)
{
    ax25_cfg_ctx_t *ctx = &s_ctx;
    if (ctx->schema == NULL) {
        return;
    }

    cfg_output_flush(ctx);
    ctx->out_fn = out_fn;
    ctx->out_arg = out_arg;
    ctx->out_len = 0;
}

esp_err_t ax25_cfg_get(const char *parameter, char *out, size_t out_len)
{
    ax25_cfg_ctx_t *ctx = &s_ctx;
    if (ctx->schema == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t idx = 0;
    const ax25_cfg_param_t *param = find_param(ctx, parameter, &idx);
    if (param == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    return get_effective_value(ctx, param, (uint16_t)idx, out, out_len);
}

void ax25_cfg_get_str(const char *parameter, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return;
    }

    if (ax25_cfg_get(parameter, out, out_len) != ESP_OK) {
        out[0] = '\0';
    }
}

int ax25_cfg_get_int(const char *parameter)
{
    char buf[32];

    if (ax25_cfg_get(parameter, buf, sizeof(buf)) != ESP_OK) {
        return 0;
    }

    char *end = NULL;
    long value = strtol(buf, &end, 10);
    if (end == NULL || end == buf || *end != '\0') {
        return 0;
    }

    return (int)value;
}

bool ax25_cfg_get_bool(const char *parameter)
{
    char buf[8];

    if (ax25_cfg_get(parameter, buf, sizeof(buf)) != ESP_OK) {
        return false;
    }

    if (token_equals_ci(buf, "1") || token_equals_ci(buf, "true") || token_equals_ci(buf, "yes")) {
        return true;
    }

    if (token_equals_ci(buf, "0") || token_equals_ci(buf, "false") || token_equals_ci(buf, "no")) {
        return false;
    }

    return false;
}

esp_err_t ax25_cfg_set(const char *parameter, const char *value, char *err, size_t err_len)
{
    ax25_cfg_ctx_t *ctx = &s_ctx;
    if (ctx == NULL || parameter == NULL || value == NULL) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "invalid arguments");
        }
        return ESP_ERR_INVALID_ARG;
    }

    size_t idx = 0;
    const ax25_cfg_param_t *param = find_param(ctx, parameter, &idx);
    if (param == NULL) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "unknown parameter '%s'", parameter);
        }
        return ESP_ERR_NOT_FOUND;
    }

    char normalized[AX25_CFG_VALUE_MAX_LEN];
    esp_err_t v_err = normalize_value(param, value, normalized, sizeof(normalized), err, err_len);
    if (v_err != ESP_OK) {
        return v_err;
    }

    esp_err_t set_err = mark_pending_set(ctx, (uint16_t)idx, normalized);
    if (set_err != ESP_OK && err != NULL && err_len > 0) {
        snprintf(err, err_len, "pending list full");
    }
    return set_err;
}

esp_err_t ax25_cfg_clear(const char *parameter, char *err, size_t err_len)
{
    ax25_cfg_ctx_t *ctx = &s_ctx;
    if (ctx == NULL || parameter == NULL) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "invalid arguments");
        }
        return ESP_ERR_INVALID_ARG;
    }

    size_t idx = 0;
    if (find_param(ctx, parameter, &idx) == NULL) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "unknown parameter '%s'", parameter);
        }
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t cl_err = mark_pending_clear(ctx, (uint16_t)idx);
    if (cl_err != ESP_OK && err != NULL && err_len > 0) {
        snprintf(err, err_len, "pending list full");
    }
    return cl_err;
}

esp_err_t ax25_cfg_save(char *err, size_t err_len)
{
    ax25_cfg_ctx_t *ctx = &s_ctx;
    if (ctx->schema == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t copy_err = copy_primary_to_backup(ctx);
    if (copy_err != ESP_OK) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "backup failed: %s", esp_err_to_name(copy_err));
        }
        return copy_err;
    }

    for (int i = 0; i < AX25_CFG_MAX_PENDING; i++) {
        if (!ctx->pending[i].in_use) {
            continue;
        }

        uint16_t idx = ctx->pending[i].param_index;
        if (idx >= ctx->schema_count) {
            continue;
        }

        const ax25_cfg_param_t *param = &ctx->schema[idx];
        const char *key = param_nvs_key(param);
        esp_err_t w_err;

        if (ctx->pending[i].cleared) {
            w_err = nvs_erase_key(ctx->nvs_primary, key);
            if (w_err == ESP_ERR_NVS_NOT_FOUND) {
                w_err = ESP_OK;
            }
        } else {
            w_err = nvs_set_str(ctx->nvs_primary, key, ctx->pending[i].value);
        }

        if (w_err != ESP_OK) {
            if (err != NULL && err_len > 0) {
                snprintf(err, err_len, "save failed for %s: %s", param->parameter, esp_err_to_name(w_err));
            }
            return w_err;
        }
    }

    esp_err_t c_err = nvs_commit(ctx->nvs_primary);
    if (c_err != ESP_OK) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "primary commit failed: %s", esp_err_to_name(c_err));
        }
        return c_err;
    }

    c_err = nvs_set_u8(ctx->nvs_backup, NVS_PENDING_KEY, 1);
    if (c_err != ESP_OK) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "pending flag failed: %s", esp_err_to_name(c_err));
        }
        return c_err;
    }

    c_err = nvs_commit(ctx->nvs_backup);
    if (c_err != ESP_OK) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "backup commit failed: %s", esp_err_to_name(c_err));
        }
        return c_err;
    }

    ctx->commit_pending = true;
    clear_pending(ctx);
    return ESP_OK;
}

esp_err_t ax25_cfg_commit(char *err, size_t err_len)
{
    ax25_cfg_ctx_t *ctx = &s_ctx;
    if (ctx->schema == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (ctx->commit_timer != NULL) {
        xTimerStop(ctx->commit_timer, 0);
        xTimerDelete(ctx->commit_timer, 0);
        ctx->commit_timer = NULL;
    }

    esp_err_t e_err = nvs_erase_key(ctx->nvs_backup, NVS_PENDING_KEY);
    if (e_err != ESP_OK && e_err != ESP_ERR_NVS_NOT_FOUND) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "erase pending flag failed: %s", esp_err_to_name(e_err));
        }
        return e_err;
    }

    esp_err_t c_err = nvs_commit(ctx->nvs_backup);
    if (c_err != ESP_OK) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "backup commit failed: %s", esp_err_to_name(c_err));
        }
        return c_err;
    }

    ctx->commit_pending = false;
    return ESP_OK;
}

esp_err_t ax25_cfg_revert(char *err, size_t err_len)
{
    ax25_cfg_ctx_t *ctx = &s_ctx;
    if (ctx->schema == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (ctx->commit_timer != NULL) {
        xTimerStop(ctx->commit_timer, 0);
        xTimerDelete(ctx->commit_timer, 0);
        ctx->commit_timer = NULL;
    }

    clear_pending(ctx);

    esp_err_t r_err = restore_backup_to_primary(ctx);
    if (r_err != ESP_OK) {
        if (err != NULL && err_len > 0) {
            snprintf(err, err_len, "revert failed: %s", esp_err_to_name(r_err));
        }
        return r_err;
    }

    return ESP_OK;
}

static void cmd_show(ax25_cfg_ctx_t *ctx, bool show_all, const char *prefix)
{
    for (size_t i = 0; i < ctx->schema_count; i++) {
        const ax25_cfg_param_t *p = &ctx->schema[i];

        if (prefix != NULL && prefix[0] != '\0') {
            size_t prefix_len = strlen(prefix);
            if (strncmp(p->parameter, prefix, prefix_len) != 0) {
                continue;
            }
        }

        char current[AX25_CFG_VALUE_MAX_LEN];
        char committed[AX25_CFG_VALUE_MAX_LEN];
        char line[AX25_CFG_VALUE_MAX_LEN * 3];
        get_effective_value(ctx, p, (uint16_t)i, current, sizeof(current));
        get_stored_value(ctx, p, committed, sizeof(committed));

        const char *defv  = p->default_value != NULL ? p->default_value : "";
        const char *curv  = p->hide ? "***" : current;
        const char *comv  = p->hide ? "***" : committed;
        const char *defvp = p->hide ? "***" : defv;

        bool is_prefix_show = (prefix != NULL && prefix[0] != '\0');
        if (!show_all && !is_prefix_show && strcmp(current, defv) == 0 && strcmp(committed, defv) == 0) {
            continue;
        }

        int written = snprintf(line, sizeof(line), "%s", p->parameter);
        if (written < 0 || (size_t)written >= sizeof(line)) {
            cfg_output(ctx, "%s\r\n", p->parameter);
            continue;
        }

        if (strcmp(current, committed) != 0) {
            written += snprintf(line + written, sizeof(line) - (size_t)written,
                                " cur=%s", curv);
        }
        if (strcmp(committed, defv) != 0) {
            written += snprintf(line + written, sizeof(line) - (size_t)written,
                                " com=%s", comv);
        }
        written += snprintf(line + written, sizeof(line) - (size_t)written,
                            " def=%s", defvp);

        cfg_output(ctx, "%s\r\n", line);
    }
}

static void cmd_dump_config(ax25_cfg_ctx_t *ctx)
{
    for (size_t i = 0; i < ctx->schema_count; i++) {
        const ax25_cfg_param_t *p = &ctx->schema[i];
        char current[AX25_CFG_VALUE_MAX_LEN];
        get_effective_value(ctx, p, (uint16_t)i, current, sizeof(current));

        const char *defv = p->default_value != NULL ? p->default_value : "";
        if (strcmp(current, defv) == 0) {
            continue;
        }

        cfg_output(ctx, "set %s=%s\r\n", p->parameter, p->hide ? "***" : current);
    }
}

static void cmd_help(ax25_cfg_ctx_t *ctx)
{
    cfg_output(ctx, "Config mode commands:\r\n");
    cfg_output(ctx, "  set <parameter>=<value>   Stage a value change (not yet persistent)\r\n");
    cfg_output(ctx, "  get <parameter>           Show effective value (includes staged change)\r\n");
    cfg_output(ctx, "  clear <parameter>         Stage reset to schema default\r\n");
    cfg_output(ctx, "  show                      Show changed values only\r\n");
    cfg_output(ctx, "  show all                  Show all parameters\r\n");
    cfg_output(ctx, "  show <prefix>             Show parameters by prefix (for example: wifi. or net.)\r\n");
    cfg_output(ctx, "  config                    Print set commands for all non-default values\r\n");
    cfg_output(ctx, "  save                      Write staged changes and arm rollback safety window\r\n");
    cfg_output(ctx, "  commit                    Confirm saved changes and cancel rollback\r\n");
    cfg_output(ctx, "  revert                    Restore last backup snapshot\r\n");
    cfg_output(ctx, "  reboot                    Restart now to apply saved/reverted values\r\n");
    cfg_output(ctx, "  exit                      Leave config mode and discard unsaved staged changes\r\n");
    cfg_output(ctx, "\r\n");
    cfg_output(ctx, "Notes:\r\n");
    cfg_output(ctx, "  - Changes from set/clear are staged in memory until save.\r\n");
    cfg_output(ctx, "  - save writes to NVS and starts a safety rollback timer.\r\n");
    cfg_output(ctx, "  - If commit is not done before timeout, previous config is restored on reboot.\r\n");
    cfg_output(ctx, "  - Hidden secrets are masked in show/config output.\r\n");
    cfg_output(ctx, "\r\n");
    cfg_output(ctx, "Value rules:\r\n");
    cfg_output(ctx, "  - bool: 0|1|true|false|yes|no\r\n");
    cfg_output(ctx, "  - int: must be in each parameter range\r\n");
    cfg_output(ctx, "  - enum: must match one of the listed tokens\r\n");
    cfg_output(ctx, "  - string: plain text\r\n");
    cfg_output(ctx, "\r\n");
    cfg_output(ctx, "Typical flow:\r\n");
    cfg_output(ctx, "  show wifi.\r\n");
    cfg_output(ctx, "  set wifi.sta.ssid=MySSID\r\n");
    cfg_output(ctx, "  set wifi.sta.password=MyPass\r\n");
    cfg_output(ctx, "  save\r\n");
    cfg_output(ctx, "  reboot\r\n");
    cfg_output(ctx, "  ...verify connectivity...\r\n");
    cfg_output(ctx, "  commit\r\n");
}

bool ax25_cfg_handle_command(const char *line)
{
    if (line == NULL) {
        return false;
    }
    if (s_ctx.schema == NULL) {
        return false;
    }
    ax25_cfg_ctx_t *ctx = &s_ctx;

#define CFG_RETURN(_v) do { cfg_output_flush(ctx); return (_v); } while (0)

    char cmd[16] = {0};
    char rest[192] = {0};
    (void)sscanf(line, "%15s %191[^\r]", cmd, rest);

    for (size_t i = 0; cmd[i] != '\0'; i++) {
        cmd[i] = (char)toupper((unsigned char)cmd[i]);
    }

    if (cmd[0] == '\0') {
        CFG_RETURN(false);
    }

    if (strcmp(cmd, "SET") == 0) {
        char *eq = strchr(rest, '=');
        if (eq == NULL) {
            cfg_output(ctx, "ERR usage: set <parameter>=<value>\r\n");
            CFG_RETURN(false);
        }

        *eq = '\0';
        char *parameter = rest;
        char *value = eq + 1;
        while (*parameter == ' ') parameter++;
        while (*value == ' ') value++;

        char err[128];
        esp_err_t set_err = ax25_cfg_set(parameter, value, err, sizeof(err));
        if (set_err != ESP_OK) {
            cfg_output(ctx, "ERR %s\r\n", err);
        } else {
            cfg_output(ctx, "OK\r\n");
        }
        CFG_RETURN(false);
    }

    if (strcmp(cmd, "GET") == 0) {
        char *parameter = rest;
        while (*parameter == ' ') parameter++;
        if (*parameter == '\0') {
            cfg_output(ctx, "ERR usage: get <parameter>\r\n");
            CFG_RETURN(false);
        }

        char value[AX25_CFG_VALUE_MAX_LEN];
        esp_err_t get_err = ax25_cfg_get(parameter, value, sizeof(value));
        if (get_err != ESP_OK) {
            cfg_output(ctx, "ERR unknown parameter '%s'\r\n", parameter);
        } else {
            cfg_output(ctx, "%s=%s\r\n", parameter, value);
        }
        CFG_RETURN(false);
    }

    if (strcmp(cmd, "CLEAR") == 0) {
        char *parameter = rest;
        while (*parameter == ' ') parameter++;
        if (*parameter == '\0') {
            cfg_output(ctx, "ERR usage: clear <parameter>\r\n");
            CFG_RETURN(false);
        }

        char err[128];
        esp_err_t clear_err = ax25_cfg_clear(parameter, err, sizeof(err));
        if (clear_err != ESP_OK) {
            cfg_output(ctx, "ERR %s\r\n", err);
        } else {
            cfg_output(ctx, "OK\r\n");
        }
        CFG_RETURN(false);
    }

    if (strcmp(cmd, "SHOW") == 0) {
        char *arg = rest;
        while (*arg == ' ') arg++;

        bool show_all = token_equals_ci(arg, "all");
        const char *prefix = NULL;
        if (!show_all && *arg != '\0') {
            prefix = arg;
        }

        cmd_show(ctx, show_all, prefix);
        CFG_RETURN(false);
    }

    if (strcmp(cmd, "SAVE") == 0) {
        char err[128];
        esp_err_t save_err = ax25_cfg_save(err, sizeof(err));
        if (save_err != ESP_OK) {
            cfg_output(ctx, "ERR %s\r\n", err);
        } else {
            cfg_output(ctx, "Configuration saved. Reboot to apply.\r\n");
        }
        CFG_RETURN(false);
    }

    if (strcmp(cmd, "CONFIG") == 0) {
        cmd_dump_config(ctx);
        CFG_RETURN(false);
    }

    if (strcmp(cmd, "COMMIT") == 0) {
        char err[128];
        esp_err_t commit_err = ax25_cfg_commit(err, sizeof(err));
        if (commit_err != ESP_OK) {
            cfg_output(ctx, "ERR %s\r\n", err);
        } else {
            cfg_output(ctx, "Configuration committed.\r\n");
        }
        CFG_RETURN(false);
    }

    if (strcmp(cmd, "REVERT") == 0) {
        char err[128];
        esp_err_t revert_err = ax25_cfg_revert(err, sizeof(err));
        if (revert_err != ESP_OK) {
            cfg_output(ctx, "ERR %s\r\n", err);
        } else {
            cfg_output(ctx, "Configuration reverted to backup. Reboot to apply.\r\n");
        }
        CFG_RETURN(false);
    }

    if (strcmp(cmd, "REBOOT") == 0) {
        cfg_output(ctx, "Rebooting...\r\n");
        cfg_output_flush(ctx);
        vTaskDelay(pdMS_TO_TICKS(250));
        esp_restart();
        CFG_RETURN(false);
    }

    if (strcmp(cmd, "EXIT") == 0) {
        clear_pending(ctx);
        CFG_RETURN(true);
    }

    if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "H") == 0 || strcmp(cmd, "?") == 0) {
        cmd_help(ctx);
        CFG_RETURN(false);
    }

    cmd_help(ctx);
    CFG_RETURN(false);

#undef CFG_RETURN
}

/* -------------------------------------------------------------------------
 * Module state query and library read API
 * ---------------------------------------------------------------------- */

bool ax25_cfg_is_initialized(void)
{
    return s_ctx.schema != NULL;
}

bool ax25_cfg_is_safety_timer_active(void)
{
    ax25_cfg_ctx_t *ctx = &s_ctx;

    if (ctx->schema == NULL || !ctx->commit_pending || ctx->commit_timer == NULL) {
        return false;
    }

    return xTimerIsTimerActive(ctx->commit_timer) == pdTRUE;
}

int ax25_cfg_get_int_global(const char *parameter, int default_val)
{
    if (s_ctx.schema == NULL || find_param(&s_ctx, parameter, NULL) == NULL) {
        return default_val;
    }
    return ax25_cfg_get_int(parameter);
}

bool ax25_cfg_get_bool_global(const char *parameter, bool default_val)
{
    if (s_ctx.schema == NULL || find_param(&s_ctx, parameter, NULL) == NULL) {
        return default_val;
    }
    return ax25_cfg_get_bool(parameter);
}

void ax25_cfg_get_str_global(const char *parameter,
                              char *out, size_t out_len,
                              const char *default_val)
{
    if (out == NULL || out_len == 0) {
        return;
    }
    if (s_ctx.schema == NULL || find_param(&s_ctx, parameter, NULL) == NULL) {
        if (default_val != NULL) {
            strlcpy(out, default_val, out_len);
        } else {
            out[0] = '\0';
        }
        return;
    }
    ax25_cfg_get_str(parameter, out, out_len);
}
