/* Copyright (c) 2017, Magic Leap, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#ifndef _MLGZ_BUFFER_H_
#define _MLGZ_BUFFER_H_
#include <linux/types.h>

#define MLGZ_CTRL_CMD_MAX_BUFFER_SIZE   (32)

uint8_t *mlgz_buffer_alloc(void);
void mlgz_ctrl_command_buffer_free(uint8_t *cmd);

#endif
