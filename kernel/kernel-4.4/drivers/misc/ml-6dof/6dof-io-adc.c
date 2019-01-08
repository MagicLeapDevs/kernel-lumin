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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include "6dof-io.h"
#include "6dof-io-adc.h"

#define ADC_MASTER_POWER_REG       0x00
#define ADC_MASTER_POWER_UP        (1 << 0)

#define ADC_PLL_CTRL_REG           0x01
#define ADC_PLL_CTRL_MUTE          (1 << 6)
#define ADC_PLL_LRCLK_IN           (1 << 4)
#define ADC_PLL_MCS_128            0
#define ADC_PLL_MCS_256            1
#define ADC_PLL_MCS_384            2
#define ADC_PLL_MCS_512            3
#define ADC_PLL_MCS_768            4

#define ADC_SAI_CTRL0_REG          0x05
#define ADC_SAI_CTRL0_FS_8_12      0
#define ADC_SAI_CTRL0_FS_16_24     1
#define ADC_SAI_CTRL0_FS_32_48     2
#define ADC_SAI_CTRL0_FS_64_96     3
#define ADC_SAI_CTRL0_FS_128_192   4

#define ADC_DC_HPF_CAL_REG         0x1A
#define ADC_DC_HPF_C4              (1 << 3)
#define ADC_DC_HPF_C3              (1 << 2)
#define ADC_DC_HPF_C2              (1 << 1)
#define ADC_DC_HPF_C1              (1 << 0)

struct adc_data {
	struct device          *dev;
	struct i2c_client      *i2c;
	struct regmap          *adc_regmap;
	struct ml_adc_io       io;
	int                    gpio_reset;
};

#define io_2_pdata(x) container_of((x), struct adc_data, io)

static const struct regmap_config regmap_cfg = {
	.name       = "adc",
	.reg_stride = 1,
	.reg_bits   = 8,
	.val_bits   = 8,
};

static void adc_power_on_gpio(struct ml_adc_io *io, bool on)
{
	struct adc_data *data = io_2_pdata(io);

	dev_dbg(data->dev, "set gpio %d\n", on);
	gpio_set_value(data->gpio_reset, on);
}

static int adc_set_power(struct ml_adc_io *io, u8 on)
{
	struct adc_data *data = io_2_pdata(io);

	dev_dbg(data->dev, "set power %s\n", on ? "on" : "off");
	if (on)
		return set_bits(data->dev, data->adc_regmap,
				ADC_MASTER_POWER_REG,
				ADC_MASTER_POWER_UP);
	else
		return clr_bits(data->dev, data->adc_regmap,
				ADC_MASTER_POWER_REG,
				ADC_MASTER_POWER_UP);
}

static int adc_set_mode(struct ml_adc_io *io, enum ml_6dof_mode mode)
{
	struct adc_data *data = io_2_pdata(io);
	int ret;

	dev_dbg(data->dev, "set mode to %s\n", emt_mode_to_str(mode));
	switch (mode) {
	case EMT_MODE_TDSP:
		/* Auto-mute w/PLL unlock; 256xFs */
		ret = write_reg(data->dev, data->adc_regmap,
				ADC_PLL_CTRL_REG,
				(ADC_PLL_CTRL_MUTE | ADC_PLL_LRCLK_IN |
				 ADC_PLL_MCS_256));
		ret += write_reg(data->dev, data->adc_regmap,
				 ADC_SAI_CTRL0_REG,
				 ADC_SAI_CTRL0_FS_128_192);
		/* Enable high pass filter for all 4 channels */
		ret += write_reg(data->dev, data->adc_regmap,
				 ADC_DC_HPF_CAL_REG,
				 ADC_DC_HPF_C4 | ADC_DC_HPF_C3 |
				 ADC_DC_HPF_C2 | ADC_DC_HPF_C1);
		break;
	case EMT_MODE_ADC:
		/* Auto-mute w/PLL unlock; 128xFs */
		ret = write_reg(data->dev, data->adc_regmap,
				ADC_PLL_CTRL_REG,
				(ADC_PLL_CTRL_MUTE | ADC_PLL_LRCLK_IN |
				 ADC_PLL_MCS_128));
		ret += write_reg(data->dev, data->adc_regmap,
				 ADC_SAI_CTRL0_REG,
				 ADC_SAI_CTRL0_FS_64_96);
		/* Enable high pass filter for all 4 channels */
		ret += write_reg(data->dev, data->adc_regmap,
				 ADC_DC_HPF_CAL_REG,
				 ADC_DC_HPF_C4 | ADC_DC_HPF_C3 |
				 ADC_DC_HPF_C2 | ADC_DC_HPF_C1);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		return -EIO;

	return 0;
}

static const struct ml_adc_ops adc_ops = {
	.power_on_gpio   = adc_power_on_gpio,
	.set_power       = adc_set_power,
	.set_mode        = adc_set_mode,
};

static int adc_i2c_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *i2c_id)
{
	struct adc_data *cdata;
	int rst_gpio;
	int ret;

	dev_info(&i2c->dev, "%s:\n", __func__);
	cdata = devm_kzalloc(&i2c->dev, sizeof(struct adc_data),
			     GFP_KERNEL);
	if (cdata == NULL)
		return -ENOMEM;

	cdata->i2c = i2c;
	cdata->dev = &i2c->dev;
	cdata->io.ops = &adc_ops;

	rst_gpio = of_get_named_gpio(i2c->dev.of_node, "rst_gpio", 0);
	if (rst_gpio < 0) {
		dev_err(&i2c->dev, "failed to get reset gpio, %d\n", rst_gpio);
		return rst_gpio;
	} else if (!gpio_is_valid(rst_gpio)) {
		dev_err(&i2c->dev, "reset gpio %d is not valid/n", rst_gpio);
		return -EINVAL;
	}

	ret = devm_gpio_request(&i2c->dev, rst_gpio, "reset_gpio");
	if (ret) {
		dev_err(&i2c->dev, "failed to get gpio %d, %d\n", rst_gpio,
			ret);
		return ret;
	}

	/* Asserting 6DOF_RST_L to keep ADC in reset */
	ret = gpio_direction_output(rst_gpio, 0);
	if (ret) {
		dev_err(&i2c->dev, "failed to set gpio %d as an output, %d\n",
			rst_gpio, ret);
		return ret;
	}
	cdata->gpio_reset = rst_gpio;

	cdata->adc_regmap = devm_regmap_init_i2c(i2c, &regmap_cfg);
	if (IS_ERR(cdata->adc_regmap)) {
		ret = PTR_ERR(cdata->adc_regmap);
		dev_err(&i2c->dev, "failed to allocate regmap: %d\n", ret);
		return ret;
	}
	i2c_set_clientdata(i2c, cdata);

	ret = sixdof_adc_io_register(&cdata->io);
	if (ret)
		dev_err(&i2c->dev, "failed to register ADC: %d\n", ret);

	return ret;
}

static int adc_i2c_remove(struct i2c_client *i2c)
{
	dev_info(&i2c->dev, "%s\n", __func__);
	sixdof_adc_io_unregister();
	i2c_set_clientdata(i2c, NULL);
	return 0;
}

static const struct i2c_device_id adc_i2c_client_id[] = {
	{" wl_adc", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, adc_i2c_client_id);

static const struct of_device_id adc_i2c_of_match[] = {
	{ .compatible = "ml,adc-i2c", },
	{ }
};
MODULE_DEVICE_TABLE(of, adc_i2c_of_match);

static struct i2c_driver adc_i2c_driver = {
	.driver = {
		.name = "wl_adc",
		.of_match_table = of_match_ptr(adc_i2c_of_match),
	},
	.probe = adc_i2c_probe,
	.remove = adc_i2c_remove,
	.id_table = adc_i2c_client_id,
};

module_i2c_driver(adc_i2c_driver);

MODULE_DESCRIPTION("Magic Leap Wearable ADC I2C driver");
MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_LICENSE("GPL v2");

