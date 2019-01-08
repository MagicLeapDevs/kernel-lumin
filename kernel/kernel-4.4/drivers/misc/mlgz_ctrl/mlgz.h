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

#ifndef __MLGZ_H__
#define __MLGZ_H__
#include <linux/ml-mux-client.h>
#include <linux/cdev.h>

struct mlgz_dev_data {
	bool open;
	struct ml_mux_client client;
	void *mlgz_cdev_data;
	struct mutex mtx;
};

ssize_t mlgz_mux_send_data(struct mlgz_dev_data *dev_data,
			const char *data, size_t len);

#endif
