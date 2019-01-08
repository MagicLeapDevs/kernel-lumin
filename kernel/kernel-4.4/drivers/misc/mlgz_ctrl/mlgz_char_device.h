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

#ifndef __MLGZ_CHAR_DEVICE_H__
#define __MLGZ_CHAR_DEVICE_H__
#include "mlgz.h"

int mlgz_char_device_init(void);
void mlgz_char_device_exit(void);
int mlgz_char_device_add(struct mlgz_dev_data *dev_data);
void mlgz_char_device_remove(struct mlgz_dev_data *dev_data);
void mlgz_char_device_queue_received_data(const uint8_t *data,
				size_t len, struct mlgz_dev_data *dev_data);

#endif
