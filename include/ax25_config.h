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

#ifndef AX25_CONFIG_H
#define AX25_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AX25_CFG_MAX_PENDING 32
#define AX25_CFG_KEY_MAX_LEN 64
#define AX25_CFG_VALUE_MAX_LEN 160

typedef enum {
    AX25_CFG_TYPE_INT = 0,
    AX25_CFG_TYPE_BOOL,
    AX25_CFG_TYPE_ENUM,
    AX25_CFG_TYPE_STRING,
} ax25_cfg_type_t;

typedef struct {
    const char *parameter;
    const char *nvs_key;
    const char *default_value;
    const char *range;
    ax25_cfg_type_t type;
    bool hide;
} ax25_cfg_param_t;

typedef void (*ax25_cfg_output_fn_t)(const char *text, size_t len, void *arg);

/**
 * Returns true if ax25_cfg_init() has been called and the configuration
 * module is ready.  Use this instead of checking a context pointer.
 */
bool ax25_cfg_is_initialized(void);

/**
 * Returns true when the rollback safety timer is currently active.
 * This is armed after `save` + reboot when uncommitted config exists.
 */
bool ax25_cfg_is_safety_timer_active(void);

esp_err_t ax25_cfg_init(const ax25_cfg_param_t *local_schema,
                        size_t local_schema_count);

void ax25_cfg_deinit(void);

void ax25_cfg_set_output(ax25_cfg_output_fn_t out_fn, void *out_arg);

esp_err_t ax25_cfg_get(const char *parameter, char *out, size_t out_len);
void ax25_cfg_get_str(const char *parameter, char *out, size_t out_len);
int  ax25_cfg_get_int(const char *parameter);
bool ax25_cfg_get_bool(const char *parameter);
esp_err_t ax25_cfg_set(const char *parameter, const char *value, char *err, size_t err_len);
esp_err_t ax25_cfg_clear(const char *parameter, char *err, size_t err_len);

esp_err_t ax25_cfg_save(char *err, size_t err_len);
esp_err_t ax25_cfg_commit(char *err, size_t err_len);
esp_err_t ax25_cfg_revert(char *err, size_t err_len);

bool ax25_cfg_handle_command(const char *line);

/* -------------------------------------------------------------------------
 * Library read API
 *
 * Library components call these instead of compile-time Kconfig macros.
 * All functions return `default_val` when the module has not been
 * initialised or the requested parameter is not in the schema.
 * ---------------------------------------------------------------------- */

int   ax25_cfg_get_int_global (const char *parameter, int default_val);
bool  ax25_cfg_get_bool_global(const char *parameter, bool default_val);
void  ax25_cfg_get_str_global (const char *parameter,
                                char *out, size_t out_len,
                                const char *default_val);

#ifdef __cplusplus
}
#endif

#endif
