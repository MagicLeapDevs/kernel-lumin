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

#ifndef __ML_6DOF_IO__
#define __ML_6DOF_IO__

#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/device.h>

int read_reg(struct device *dev, struct regmap *regmap, u32 reg, u32 *val);
int write_reg(struct device *dev, struct regmap *regmap, u32 reg, u32 val);
int set_bits(struct device *dev, struct regmap *regmap, u32 reg, u32 mask);
int clr_bits(struct device *dev, struct regmap *regmap, u32 reg, u32 mask);
int raw_read(struct device *dev, struct regmap *regmap, u32 reg,
	     void *val, size_t val_len);
int raw_write(struct device *dev, struct regmap *regmap, u32 reg,
	      const void *val, size_t val_len);

#endif /* __ML_6DOF_IO__ */
