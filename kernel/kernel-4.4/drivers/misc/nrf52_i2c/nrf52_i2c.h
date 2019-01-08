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

#ifndef __NRF52_I2C_H__
#define __NRF52_I2C_H__

#include <linux/ml-mux-ctrl.h>


#define NRF52_NAME	"nrf52_ml_mux"
#define to_nrf_data(p) container_of(p, struct nrf52_data, mlmux)

struct nrf52_data {
	struct device *dev;
	struct i2c_client *client;
	struct ml_mux_ctrl mlmux;
	struct dentry *debugfs;

	u8 *rx_buf;
	int i2c_irq_gpio;
	bool irq_enabled;
	const char *bl_fw_file;
	const char *app_fw_file;
	int reset_gpio;
	int nvidia_ready;
	int pending_fw_updates;
	struct mutex mutex_lock;
};

#endif
