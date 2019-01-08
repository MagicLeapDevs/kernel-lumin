/* Copyright (c) 2017-2018, Magic Leap, Inc. All rights reserved.
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

#ifndef __ML_6DOF_FPGA_IO__
#define __ML_6DOF_FPGA_IO__

#include <linux/types.h>
#include <linux/device.h>
#include "6dof-common.h"

#define FPGA_FREQ_ADJ_LEN      6

struct ml_fpga_io {
	const struct ml_fpga_ops *ops;
};

/**
 * struct ml_fpga_ops - ML BeltPack FPGA driver methods
 *
 * @revision: check revision of local and remote FPGA.
 * @set_power: enable/disable electromagnetic tracking.
 * @set_mode: switch between TDSP and ADC raw mode.
 * @get_freq_num: query a number of supported frequencies.
 * @freq: query frequency by index.
 * @get_freq: query receiver frequency index.
 * @ctrl_reg: program L2R control register.
 * @set_freq: set frequency of the receiver associated with the totem.
 * @rst_fine_freq: reset fine frequency adjust of the receiver
 *                 associated with the totem.
 * @adjust_freq: adjust frequency.
 * @read_adc: read 3 ADC values (LCH1, RCH1, LCH2).
 */
struct ml_fpga_ops {
	int (*revision)(struct ml_fpga_io *io);
	int (*set_power)(struct ml_fpga_io *io, bool);
	int (*set_mode)(struct ml_fpga_io *io, enum ml_6dof_mode);
	u8 (*get_freq_num)(void);
	struct ml_6dof_freq (*freq)(u8);
	int (*query_freq)(struct ml_fpga_io *io, u8 totem, u8 *);
	int (*ctrl_reg)(struct ml_fpga_io *io, bool, enum ml_6dof_mode, bool);
	int (*set_freq)(struct ml_fpga_io *io, u8, u8);
	int (*rst_fine_freq)(struct ml_fpga_io *io, u8);
	int (*adjust_freq)(struct ml_fpga_io *io, const u32 __user *);
	int (*read_adc)(struct ml_fpga_io *io, u32 adc[EMT_AXIS_NUM]);
};

int sixdof_fpga_io_register(struct ml_fpga_io *);
void sixdof_fpga_io_unregister(void);

#endif /* __ML_6DOF_FPGA_IO__ */
