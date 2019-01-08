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

#ifndef __ML_6DOF_ADC_IO__
#define __ML_6DOF_ADC_IO__

#include <linux/types.h>
#include <linux/device.h>
#include "6dof-common.h"

struct ml_adc_io {
	const struct ml_adc_ops  *ops;
};

/**
 * struct ml_adc_ops - ML ADC driver methods
 *
 * @power_on_gpio: assert/deassert 6DOF_RST_L gpio.
 * @set_power: enable/disable electromagnetic tracking.
 * @set_mode: switch between TDSP and ADC raw mode.
 */
struct ml_adc_ops {
	void (*power_on_gpio)(struct ml_adc_io *, bool);
	int (*set_power)(struct ml_adc_io *, u8);
	int (*set_mode)(struct ml_adc_io *, enum ml_6dof_mode);
};

int sixdof_adc_io_register(struct ml_adc_io *ctrl);
void sixdof_adc_io_unregister(void);

#endif /* __ML_6DOF_ADC_IO__ */
