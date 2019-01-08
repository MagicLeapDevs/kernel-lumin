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

/*
 * Driver for the Parade Technologies Inc. PS8713 USB 3.0
 * Repeater/Redriver.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>

#define PS8713_CHA_MODE_REG             0x10
#define PS8713_CHB_MODE_REG             0x20
#define PS8713_EQ1_REG_OFFSET           1
#define PS8713_EQ2_REG_OFFSET           2
#define PS8713_EQ3_REG_OFFSET           3
#define PS8713_EQ4_REG_OFFSET           4
#define PS8713_DRV1_REG_OFFSET          5
#define PS8713_DRV2_REG_OFFSET          6
#define PS8713_CFG1_REG_OFFSET          8

#define PS8713_CH_MAX			2
#define PS8713_REG_OFFSET_MAX		9

struct ps8713_data {
	u8 shadow_reg[PS8713_CH_MAX][PS8713_REG_OFFSET_MAX];
};

static int ps8713_write(struct i2c_client *client, u8 reg, u8 mask)
{
	s32 ret;
	u8 reg_val;
	u8 ch_num;
	u8 reg_off;
	struct ps8713_data *drv_data = i2c_get_clientdata(client);

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return ret;
	reg_val = ret;

	/* initial value for each setting will be 0 so use OR to set bits */
	reg_val |= mask;
	ret = i2c_smbus_write_byte_data(client, reg, reg_val);
	if (!ret) {
		ch_num  = (reg >> 4) - 1;
		reg_off =  reg & 0xF;
		drv_data->shadow_reg[ch_num][reg_off] = reg_val;
	}

	return ret;
}

/**
 * ps8713_init_reg - Setting initial config values in the registers based
 * on device tree. Initial values in registers are 0 so will only need to
 * set bits and not clear them.
 */
static int ps8713_init_reg(struct i2c_client *client)
{
	struct device_node *child;
	const char *chan_name;
	int ret;
	u8 reg_base;
	u8 mask;
	u32 index;
	u32 val;

	for_each_child_of_node(client->dev.of_node, child) {
		/* if node is present, it must have chan-name */
		ret = of_property_read_string(child, "chan-name", &chan_name);
		if (!ret) {
			if (strcmp(chan_name, "ch-a") == 0) {
				dev_info(&client->dev, "Chan A node found\n");
				reg_base = PS8713_CHA_MODE_REG;
			} else if (strcmp(chan_name, "ch-b") == 0) {
				dev_info(&client->dev, "Chan B node found\n");
				reg_base = PS8713_CHB_MODE_REG;
			} else {
				dev_err(&client->dev, "channel not correct\n");
				return -EINVAL;
			}
		} else {
			dev_err(&client->dev, "channel not correct/missing\n");
			return ret;
		}

		if (of_find_property(child, "ch-disable", NULL)) {
			ret = ps8713_write(client, reg_base, 1 << 7);
			if (ret)
				return ret;
		}

		if (of_find_property(child, "rx-det-disable", NULL)) {
			ret = ps8713_write(client, reg_base, 1 << 5);
			if (ret)
				return ret;
		}

		if (of_find_property(child, "auto-pwr-saving-disable", NULL)) {
			ret = ps8713_write(client, reg_base, 1 << 4);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "eq-level", &index)) {
			if (index == 0)
				mask = 0;
			else if (index >= 1 && index <= 3)
				mask = 1 << 2;
			else if (index >= 4 && index <= 6)
				mask = 3 << 2;
			else
				return -EINVAL;

			ret = ps8713_write(client, reg_base +
					PS8713_EQ3_REG_OFFSET, mask);
			if (ret)
				return ret;

			ret = ps8713_write(client, reg_base +
					PS8713_EQ1_REG_OFFSET,
					(1 << index) - 1);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "signal-det", &val)) {
			if (val > 7)
				return -EINVAL;
			ret = ps8713_write(client, reg_base +
					PS8713_EQ2_REG_OFFSET, val << 5);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "idle-exit", &val)) {
			if (val > 3)
				return -EINVAL;
			ret = ps8713_write(client, reg_base +
					PS8713_EQ2_REG_OFFSET, val << 2);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "idle-enter", &val)) {
			if (val > 3)
				return -EINVAL;
			ret = ps8713_write(client, reg_base +
					PS8713_EQ2_REG_OFFSET, val);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "input-term", &val)) {
			if (val != 0 && val != 7)
				return -EINVAL;
			ret = ps8713_write(client, reg_base +
					PS8713_EQ3_REG_OFFSET, val << 5);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "power-saving", &val)) {
			if (val > 7)
				return -EINVAL;
			ret = ps8713_write(client, reg_base +
					PS8713_EQ4_REG_OFFSET, val << 2);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "tune-lfps-swing", &val)) {
			if (val > 3)
				return -EINVAL;
			ret = ps8713_write(client, reg_base +
					PS8713_EQ4_REG_OFFSET, val);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "de-emphasis", &val)) {
			if (val > 3)
				return -EINVAL;
			ret = ps8713_write(client, reg_base +
					PS8713_DRV1_REG_OFFSET, val << 6);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "output-ampl", &val)) {
			if (val > 7)
				return -EINVAL;
			ret = ps8713_write(client, reg_base +
					PS8713_DRV2_REG_OFFSET, val << 5);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "output-term", &val)) {
			if (val != 0 && val != 7)
				return -EINVAL;
			ret = ps8713_write(client, reg_base +
					PS8713_DRV2_REG_OFFSET, val << 2);
			if (ret)
				return ret;
		}

		if (!of_property_read_u32(child, "idle-time-switch", &val)) {
			if (val > 3)
				return -EINVAL;
			ret = ps8713_write(client, reg_base +
					PS8713_CFG1_REG_OFFSET, val << 6);
			if (ret)
				return ret;
		}
	}

	dev_info(&client->dev, "i2c writes to configure registers passed\n");

	return 0;
}

static int ps8713_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct ps8713_data *drv_data;

	dev_info(&client->dev, "enter ps8713 probe\n");

	drv_data = devm_kzalloc(&client->dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	i2c_set_clientdata(client, drv_data);

	ret = ps8713_init_reg(client);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int ps8713_resume(struct device *dev)
{
	int ch, off;
	u8 reg;
	struct i2c_client *client	= to_i2c_client(dev);
	struct ps8713_data *drv_data	= i2c_get_clientdata(client);

	for (ch = 0; ch < PS8713_CH_MAX; ch++) {
		for (off = 0; off < PS8713_REG_OFFSET_MAX; off++) {
			reg = ((ch + 1) << 4) | off;
			ps8713_write(client, reg,
				     drv_data->shadow_reg[ch][off]);
		}
	}

	return 0;
}

static const struct dev_pm_ops ps8713_pm_ops = {
	.resume = ps8713_resume,
};
#define PS8713_PM_OPS_PTR	(&ps8713_pm_ops)
#else
#define PS8713_PM_OPS_PTR	NULL
#endif	/* CONFIG_PM_SLEEP */

static const struct i2c_device_id ps8713_id[] = {
	{ "ps8713", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ps8713_id);

#ifdef CONFIG_OF
static const struct of_device_id of_ps8713_match[] = {
	{ .compatible = "parade,ps8713", },
	{},
};

MODULE_DEVICE_TABLE(of, of_ps8713_match);
#endif

static struct i2c_driver ps8713_driver = {
	.driver = {
		.name = "ps8713",
		.of_match_table = of_match_ptr(of_ps8713_match),
		.pm = PS8713_PM_OPS_PTR,
	},
	.probe = ps8713_probe,
	.id_table = ps8713_id,
};

module_i2c_driver(ps8713_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("PS8713 I2C driver");
