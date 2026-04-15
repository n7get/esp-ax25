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

#ifndef AX25_CONFIG_DEFAULTS_H
#define AX25_CONFIG_DEFAULTS_H

#include <stddef.h>

#include "ax25_config.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const ax25_cfg_param_t ax25_cfg_defaults_schema[];
extern const size_t ax25_cfg_defaults_schema_count;

#ifdef __cplusplus
}
#endif

#endif
