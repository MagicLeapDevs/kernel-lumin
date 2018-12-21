/* Copyright (c) 2017-2018, Magic Leap, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/regmap.h>
#include "6dof-io.h"

int read_reg(struct device *dev, struct regmap *rmap, u32 reg, u32 *val)
{
	int ret;
	u32 reg_val = 0;

	ret = regmap_read(rmap, reg, &reg_val);
	if (!ret) {
		*val = reg_val & 0xFF;
		dev_dbg(dev, "%s: reg 0x%x, val = 0x%x\n", __func__, reg,
			*val);
	} else {
		dev_err(dev, "read reg 0x%x failed, %d\n", reg, ret);
	}

	return ret;
}

int write_reg(struct device *dev, struct regmap *rmap, u32 reg, u32 val)
{
	int ret;

	dev_dbg(dev, "%s: reg 0x%x, val = 0x%x\n", __func__, reg, val);
	ret = regmap_write(rmap, reg, val);
	if (ret)
		dev_err(dev, "write reg 0x%x failed, %d\n", reg, ret);
	return ret;
}

int set_bits(struct device *dev, struct regmap *rmap, u32 reg, u32 mask)
{
	int ret;
	u32 val;

	dev_dbg(dev, "%s: reg 0x%x, mask = 0x%x\n", __func__, reg, mask);
	ret = read_reg(dev, rmap, reg, &val);
	if (ret)
		return ret;

	/* Check if all bit are already set */
	if ((val & mask) == mask)
		return 0;

	return write_reg(dev, rmap, reg, (val | mask));
}

int clr_bits(struct device *dev, struct regmap *rmap, u32 reg, u32 mask)
{
	int ret;
	u32 val;

	dev_dbg(dev, "%s: reg 0x%x, mask = 0x%x\n", __func__, reg, mask);
	ret = read_reg(dev, rmap, reg, &val);
	if (ret)
		return ret;

	/* Check if all bit are already masked */
	if ((val & mask) == 0)
		return 0;

	return write_reg(dev, rmap, reg, val & ~mask);
}

int raw_read(struct device *dev, struct regmap *rmap, u32 reg, void *val,
	     size_t val_len)
{
	int ret;

	ret = regmap_raw_read(rmap, reg, val, val_len);
	if (ret)
		dev_err(dev, "read %zu bytes from reg 0x%x failed, %d\n",
			val_len, reg, ret);
	return ret;
}

int raw_write(struct device *dev, struct regmap *rmap, u32 reg,
	      const void *val, size_t val_len)
{
	int ret;

	ret = regmap_raw_write(rmap, reg, val, val_len);
	if (ret)
		dev_err(dev, "write %zu bytes to reg 0x%x failed, %d\n",
			val_len, reg, ret);
	return ret;

}
