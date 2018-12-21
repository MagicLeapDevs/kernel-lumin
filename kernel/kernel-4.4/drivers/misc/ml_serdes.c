/* Copyright (c) 2016-2017, Magic Leap, Inc. All rights reserved.
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
 * This driver exposes JTAG over GPIOS used to flash the SERDES FPGA chip
 *
 */
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include "../gpio/gpiolib.h"
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/tegra-aon-pm.h>

#define PULSE_HIGH 1
#define PULSE_LOW 0
#define JTAG_WRITE_LEN 12
#define JTAG_READ_LEN 4

#define ITERATIONS_PER_CLOCK 50
#define LSD_PROG_TDO_TDI_BIT 0x80

/* pinctrl bus macro */
#define SERDES_PINCTRL_FPGA_ACTIVE	0
#define SERDES_PINCTRL_FPGA_INACTIVE	1

struct serdes_platform_info {
	struct device *dev;
	struct mutex mutex_lock;
	struct regulator *regulator;
	int tck_gpio;
	int tdo_gpio;
	int tdi_gpio;
	int tms_gpio;
	int en_gpio;
	int nconfig_gpio;
	enum of_gpio_flags	dt_flags;
	struct pinctrl		*serdes_pinctrl;
	struct pinctrl_state	*serdes_jtag_active;
	struct pinctrl_state	*serdes_jtag_inactive;
	bool initialized;
	struct gpio_chip *tck_chip;
	int tck_offset;
	struct gpio_chip *tms_chip;
	int tms_offset;
	struct gpio_chip *tdi_chip;
	int tdi_offset;
	struct gpio_chip *tdo_chip;
	int tdo_offset;
};

static struct serdes_platform_info *init_jtag_info(struct device *dev)
{
	static struct serdes_platform_info *jtag_info;
	struct gpio_desc *tck_desc;
	struct gpio_desc *tms_desc;
	struct gpio_desc *tdi_desc;
	struct gpio_desc *tdo_desc;

	if (!dev && !jtag_info)
		dev_err(dev, "serdes_init_jtag_info; jtag_info: NULL\n");
	if (!dev && jtag_info)
		jtag_info = NULL;

	if (!jtag_info && dev) {
		jtag_info = dev_get_platdata(dev);
		if (jtag_info == NULL)
			dev_err(dev, "serdes_init_jtag_info; jtag_info: NULL\n");
		if (jtag_info != NULL) {
			tck_desc = gpio_to_desc(jtag_info->tck_gpio);
			if (tck_desc == NULL) {
				dev_err(dev, "serdes_init_jtag_info; tck_desc: NULL\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			jtag_info->tck_chip = tck_desc->chip;
			if (jtag_info->tck_chip == NULL) {
				dev_err(dev, "serdes_init_jtag_info; tck_chip: NULL\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			jtag_info->tck_offset = gpio_chip_hwgpio(tck_desc);
			if (jtag_info->tck_offset == 0) {
				dev_err(dev, "serdes_init_jtag_info; tck_offset: 0\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			tms_desc = gpio_to_desc(jtag_info->tms_gpio);
			if (tms_desc == NULL) {
				dev_err(dev, "serdes_init_jtag_info; tms_desc: NULL\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			jtag_info->tms_chip = tms_desc->chip;
			if (jtag_info->tms_chip == NULL) {
				dev_err(dev, "serdes_init_jtag_info; tms_chip: NULL\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			jtag_info->tms_offset = gpio_chip_hwgpio(tms_desc);
			if (jtag_info->tms_offset == 0) {
				dev_err(dev, "serdes_init_jtag_info; tms_offset: 0\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			tdi_desc = gpio_to_desc(jtag_info->tdi_gpio);
			if (tdi_desc == NULL) {
				dev_err(dev, "serdes_init_jtag_info; tdi_desc: NULL\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			jtag_info->tdi_chip = tdi_desc->chip;
			if (jtag_info->tdi_chip == NULL) {
				dev_err(dev, "serdes_init_jtag_info; tdi_chip: NULL\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			jtag_info->tdi_offset = gpio_chip_hwgpio(tdi_desc);
			if (jtag_info->tdi_offset == 0) {
				dev_err(dev, "serdes_init_jtag_info; tdi_offset: 0\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			tdo_desc = gpio_to_desc(jtag_info->tdo_gpio);
			if (tdo_desc == NULL) {
				dev_err(dev, "serdes_init_jtag_info; tdo_desc: NULL\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			jtag_info->tdo_chip = tdo_desc->chip;
			if (jtag_info->tdo_chip == NULL) {
				dev_err(dev, "serdes_init_jtag_info; tdo_chip: NULL\n");
				jtag_info = NULL;
			}
		}
		if (jtag_info != NULL) {
			jtag_info->tdo_offset = gpio_chip_hwgpio(tdo_desc);
			if (jtag_info->tdo_offset == 0) {
				dev_err(dev, "serdes_init_jtag_info; tdo_offset: 0\n");
				jtag_info = NULL;
			}
		}

		/*
		 * Initialize the direct addresses and set
		 * the TCK, TMS, and TDI pins as Output.
		*/

		if (jtag_info != NULL) {
			jtag_info->tck_chip->set(jtag_info->tck_chip,
					jtag_info->tck_offset, PULSE_LOW);
			jtag_info->tms_chip->set(jtag_info->tms_chip,
					jtag_info->tms_offset, PULSE_HIGH);
			jtag_info->tdi_chip->set(jtag_info->tdi_chip,
					jtag_info->tdi_offset, PULSE_LOW);
		}
	}

	return jtag_info;
}

static int ml_serdes_get_pinctrl(struct device *dev)
{
	struct serdes_platform_info *info = dev_get_platdata(dev);
	int ret = 0;

	info->serdes_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(info->serdes_pinctrl)) {
		ret = PTR_ERR(info->serdes_pinctrl);
		dev_err(dev, "devm_pinctrl_get error: %d\n", ret);
		return ret;
	}

	info->serdes_jtag_active = pinctrl_lookup_state(info->serdes_pinctrl,
						   "serdes_jtag_active");
	if (IS_ERR(info->serdes_jtag_active)) {
		ret = PTR_ERR(info->serdes_jtag_active);
		dev_err(dev, "serdes_jtag_active error: %d\n", ret);
		return ret;
	}

	info->serdes_jtag_inactive = pinctrl_lookup_state(info->serdes_pinctrl,
						   "serdes_jtag_inactive");
	if (IS_ERR(info->serdes_jtag_inactive)) {
		ret = PTR_ERR(info->serdes_jtag_inactive);
		dev_err(dev, "serdes_jtag_inactive error: %d\n", ret);
		return ret;
	}

	return ret;
}

static int ml_serdes_set_pinctrl(struct device *dev,
			      uint8_t state)
{
	struct serdes_platform_info *info = dev_get_platdata(dev);
	int ret = 0;

	if (!info->serdes_pinctrl) {
		dev_err(dev, "No serdes_pinctrl found!\n");
		return -EFAULT;
	}

	if (state == SERDES_PINCTRL_FPGA_ACTIVE &&
			info->serdes_jtag_active) {
		dev_dbg(dev, "%s pinctrl for FPGA JTAG active\n",
			__func__);
		ret = pinctrl_select_state(info->serdes_pinctrl,
					    info->serdes_jtag_active);

	} else if (state == SERDES_PINCTRL_FPGA_INACTIVE &&
			info->serdes_jtag_inactive){
		dev_dbg(dev, "%s pinctrl for FPGA JTAG inactive\n",
			__func__);
		ret = pinctrl_select_state(info->serdes_pinctrl,
					    info->serdes_jtag_inactive);
	} else
		dev_info(dev, "Invalid state requested\n");
	if (ret)
		dev_err(dev, "%s: pinctrl_select_state failed\n", __func__);
	return ret;
}

static ssize_t jtag_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct serdes_platform_info *info = dev_get_platdata(dev);
	int new_state = 0;
	int rc = 0;
	int on_val = 0;

	rc = kstrtoint(buf, 10, &new_state);
	if (rc != 0 || new_state < 0)
		return -EINVAL;
	new_state = !!new_state;

	mutex_lock(&info->mutex_lock);
	on_val = (info->dt_flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

	if (new_state && !info->initialized) {
		gpio_set_value(info->en_gpio, on_val);
		rc = ml_serdes_set_pinctrl(dev, SERDES_PINCTRL_FPGA_ACTIVE);
		if (rc)
			dev_err(dev, "pinctrl set fail: %d\n", rc);

		rc = devm_gpio_request_one(dev, info->tck_gpio,
			GPIOF_OUT_INIT_LOW, "tck");
		if (rc) {
			dev_err(dev, "tck request failed: %d\n",
				rc);
			goto fail;
		}

		rc = devm_gpio_request_one(dev, info->tdo_gpio,
			GPIOF_IN, "tdo");
		if (rc) {
			dev_err(dev, "tdo request failed: %d\n",
				rc);
			goto tdo_fail;
		}

		rc = devm_gpio_request_one(dev, info->tdi_gpio,
			GPIOF_OUT_INIT_LOW, "tdi");
		if (rc) {
			dev_err(dev, "tdi request failed: %d\n",
				rc);
			goto tdi_fail;
		}

		rc = devm_gpio_request_one(dev, info->tms_gpio,
			GPIOF_OUT_INIT_HIGH, "tms");
		if (rc) {
			dev_err(dev, "tms request failed: %d\n",
				rc);
			goto tms_fail;
		}

		rc = devm_gpio_request_one(dev, info->nconfig_gpio,
			GPIOF_OUT_INIT_HIGH, "nconfig");
		if (rc) {
			dev_err(dev, "nconfig request failed: %d\n",
				rc);
			goto nconfig_fail;
		}

		dev_warn(dev, "SERDES JTAG enabled\n");
		if (init_jtag_info(dev) == NULL)
			goto nconfig_fail;
		else
			info->initialized = true;

	} else if (!new_state && info->initialized) {
		gpio_set_value(info->en_gpio, !on_val);
		devm_gpio_free(dev, info->tck_gpio);
		devm_gpio_free(dev, info->tdi_gpio);
		devm_gpio_free(dev, info->tdo_gpio);
		devm_gpio_free(dev, info->tms_gpio);
		devm_gpio_free(dev, info->nconfig_gpio);
		ml_serdes_set_pinctrl(dev, SERDES_PINCTRL_FPGA_INACTIVE);
		dev_warn(dev, "SERDES JTAG disabled\n");
		info->initialized = false;
		init_jtag_info(NULL);
	} else
		dev_warn(dev, "%s: ignored write, no change\n",
		 __func__);

	mutex_unlock(&info->mutex_lock);
	return count;

nconfig_fail:
	devm_gpio_free(dev, info->tms_gpio);
tms_fail:
	devm_gpio_free(dev, info->tdi_gpio);
tdi_fail:
	devm_gpio_free(dev, info->tdo_gpio);
tdo_fail:
	devm_gpio_free(dev, info->tck_gpio);
fail:
	gpio_set_value(info->en_gpio, !on_val); /* Turn off FPGA JTAG */
	ml_serdes_set_pinctrl(dev, SERDES_PINCTRL_FPGA_INACTIVE);
	mutex_unlock(&info->mutex_lock);
	dev_err(dev, "%s: goto fail\n", __func__);
	return -EINVAL;
}
static DEVICE_ATTR_WO(jtag_en);

static ssize_t ml_serdes_gpio_helper(struct device *dev, const char *buf,
	size_t count, int gpio)
{
	struct serdes_platform_info *info = dev_get_platdata(dev);
	int new_state = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &new_state);
	if (rc != 0 || new_state < 0)
		goto fail;

	new_state = !!new_state;

	mutex_lock(&info->mutex_lock);
	if (!info->initialized) {
		dev_err(dev, "Enable serdes_en first\n");
		mutex_unlock(&info->mutex_lock);
		return -EINVAL;
	}
	gpio_set_value(gpio, new_state);
	mutex_unlock(&info->mutex_lock);
	return count;
fail:
	dev_err(dev, "%s: goto fail\n", __func__);
	return -EINVAL;
}

static ssize_t nconfig_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct serdes_platform_info *info = dev_get_platdata(dev);

	return ml_serdes_gpio_helper(dev, buf, count, info->nconfig_gpio);
}

static DEVICE_ATTR_WO(nconfig);

static uint32_t tdo_data;


static int lsd_jtag_bitbang_io(struct serdes_platform_info *jtag_info,
						int tms, int tdi, int read_tdo)
{
	int tdo = 0;
	static int first;
	static int prev_tms;
	static int prev_tdi;

	/* saturate tms/tdi/read_tdo  */
	if (tms > 0)
		tms = 1;

	if (tdi > 0)
		tdi = 1;

	if (read_tdo > 0)
		read_tdo = 1;

	if (!first) {
		prev_tms = !tms;
		prev_tdi = !tdi;
		first = 1;
		/* set clock low for first time only, */
		jtag_info->tck_chip->set_value(jtag_info->tck_chip,
				jtag_info->tck_offset, PULSE_LOW);
	}

	/* maintain state of tms and set only in case of toggle*/
	if (tms != prev_tms)
		jtag_info->tms_chip->set_value(jtag_info->tms_chip,
			jtag_info->tms_offset, (tms == 1) ?
			PULSE_HIGH : PULSE_LOW);
	/* maintain state of tdi and set only in case of toggle */
	if (tdi != prev_tdi)
		jtag_info->tdi_chip->set_value(jtag_info->tdi_chip,
			jtag_info->tdi_offset, (tdi == 1) ?
			PULSE_HIGH : PULSE_LOW);

	if (read_tdo)
		tdo = jtag_info->tdo_chip->get_value(jtag_info->tdo_chip,
			jtag_info->tdo_offset);

	jtag_info->tck_chip->set_value(jtag_info->tck_chip,
			jtag_info->tck_offset, PULSE_HIGH);

	jtag_info->tck_chip->set_value(jtag_info->tck_chip,
			jtag_info->tck_offset, PULSE_LOW);

	prev_tdi = tdi;
	prev_tms = tms;

	return tdo;
}

/* this new node will bit bang 32 bit of svf packet data in one go */
static ssize_t jtag_stream_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tms_data;
	uint32_t tms_sleep;
	static uint32_t tdi;
	uint32_t shift_bit;
	uint32_t shift_data;
	uint8_t tms_counts;
	uint8_t tms_pre;
	uint8_t tms_post;
	uint8_t data_count;
	uint8_t tms_pre_count;
	uint8_t tms_post_count;
	uint8_t jtag_io_read_bit = 1;
	struct serdes_platform_info *jtag_info = dev_get_platdata(dev);

	/*
	 * Previously svf_record would overlay the buf to allow access
	 * to the svf_record data elemenmts.  For performance reasons it
	 * is better to have the data elements in their own variables and
	 * access them directly.
	*/

	tms_counts	= (uint8_t)buf[0];
	tms_pre		= (uint8_t)buf[1];
	tms_post	= (uint8_t)buf[2];
	data_count	= (uint8_t)buf[3];
	tms_data	= (uint32_t)buf[4];
	tms_data	= (uint32_t)(tms_data << 8) | buf[5];
	tms_data	= (uint32_t)(tms_data << 8) | buf[6];
	tms_data	= (uint32_t)(tms_data << 8) | buf[7];
	tms_sleep	= (uint32_t)buf[8];
	tms_sleep	= (uint32_t)(tms_sleep << 8) | buf[9];
	tms_sleep	= (uint32_t)(tms_sleep << 8) | buf[10];
	tms_sleep	= (uint32_t)(tms_sleep << 8) | buf[11];

	jtag_io_read_bit = (data_count & LSD_PROG_TDO_TDI_BIT) >> 7;

	tms_pre_count = (tms_counts >> 4) & 0x0f;
	tms_post_count = tms_counts & 0x0f;
	tdo_data = 0;
	shift_data = tms_pre;

	/* Output the TMS bits that begin the transfer. */
	for (shift_bit = 0; shift_bit < tms_pre_count; shift_bit++) {
		lsd_jtag_bitbang_io(jtag_info, (shift_data & 1), 0, 0);
		shift_data = shift_data >> 1;
	}

	/* Output the data for the transfer. */
	if ((data_count & LSD_PROG_TDO_TDI_BIT) == 0)
		shift_data = tdi = tms_data;
	else
		shift_data = tdi;

	for (shift_bit = 0; (shift_bit + 1) <
			(data_count & ~LSD_PROG_TDO_TDI_BIT);
			shift_bit++) {
		tdo_data = tdo_data >> 1;
		tdo_data |= (lsd_jtag_bitbang_io(jtag_info, 0, (shift_data & 1),
					jtag_io_read_bit) << 31);
		shift_data = shift_data >> 1;
	}

	if (data_count == 0) {
		shift_bit = 0;
	} else {
		/* Special output to handle the first transfer
		 *	ending TMS bit.
		*/
		tdo_data = tdo_data >> 1;
		tdo_data |= (lsd_jtag_bitbang_io(jtag_info, (tms_post & 1),
			(shift_data & 1), jtag_io_read_bit) << 31);
		shift_data = tms_post;
		shift_data = shift_data >> 1;
		shift_bit = 1;
	}

	tdo_data = tdo_data >> (32 - (data_count & ~LSD_PROG_TDO_TDI_BIT));

	/*
	 * Output the TMS bits that end the transfer.
	 * Please note the shift_bit initialization depends
	 * on what came before; it is initialized there.
	*/
	for (; shift_bit < tms_post_count; shift_bit++) {
		lsd_jtag_bitbang_io(jtag_info, (shift_data & 1), 0, 0);
		shift_data = shift_data >> 1;
	}

	if (tms_sleep != 0)
		udelay(tms_sleep);

	return JTAG_WRITE_LEN;
}
static DEVICE_ATTR_WO(jtag_stream);

static ssize_t jtag_result_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	buf[3] = (tdo_data) & 0xFF;
	buf[2] = (tdo_data >> 8) & 0xFF;
	buf[1] = (tdo_data >> 16) & 0xFF;
	buf[0] = (tdo_data >> 24) & 0xFF;

	return JTAG_READ_LEN;
}
static DEVICE_ATTR_RO(jtag_result);

/* GPIO attribs */
static struct attribute *serdes_attributes[] = {
	&dev_attr_jtag_stream.attr,
	&dev_attr_jtag_result.attr,
	&dev_attr_jtag_en.attr,
	&dev_attr_nconfig.attr,
	NULL,
};

static const struct attribute_group serdes_attr_group = {
	.attrs = serdes_attributes,
};

static int ml_serdes_probe(struct platform_device *pdev)
{
	int rc;
	struct device_node *node;
	struct serdes_platform_info *info;
	unsigned long init_flags;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = &pdev->dev;
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "of_node not found!\n");
		return -ENOENT;
	}

	node = pdev->dev.of_node;
	if (!of_device_is_available(node))
		return -ENODEV;

	pdev->dev.platform_data = info;

	rc = ml_serdes_get_pinctrl(info->dev);
	if (rc < 0)
		dev_err(&pdev->dev, "unable to get pinctrl %d\n", rc);

	info->regulator = devm_regulator_get(&pdev->dev, "pwr");
	if (IS_ERR(info->regulator)) {
		dev_err(&pdev->dev, "Unable to read pwr regulator!\n");
		return PTR_ERR(info->regulator);
	}
	rc = regulator_enable(info->regulator);
	if (rc) {
		dev_err(&pdev->dev, "Failed to enable regulator: %d\n", rc);
		return rc;
	}

	/* Find all gpio nodes */
	info->tck_gpio = of_get_named_gpio(node, "tck_gpio", 0);
	if (!gpio_is_valid(info->tck_gpio)) {
		dev_err(&pdev->dev, "Invalid TCK GPIO pin!\n");
		return -EINVAL;
	}

	info->tdo_gpio = of_get_named_gpio(node, "tdo_gpio", 0);
	if (!gpio_is_valid(info->tdo_gpio)) {
		dev_err(&pdev->dev, "Invalid TDO GPIO pin!\n");
		return -EINVAL;
	}

	info->tdi_gpio = of_get_named_gpio(node, "tdi_gpio", 0);
	if (!gpio_is_valid(info->tdi_gpio)) {
		dev_err(&pdev->dev, "Invalid TDI GPIO pin!\n");
		return -EINVAL;
	}

	info->tms_gpio = of_get_named_gpio(node, "tms_gpio", 0);
	if (!gpio_is_valid(info->tms_gpio)) {
		dev_err(&pdev->dev, "Invalid TMS GPIO pin!\n");
		return -EINVAL;
	}

	info->en_gpio = of_get_named_gpio_flags(node, "enable_gpio",
						0, &info->dt_flags);
	if (!gpio_is_valid(info->en_gpio)) {
		dev_err(&pdev->dev, "Invalid ENABLE GPIO pin!\n");
		return -EINVAL;
	}

	info->nconfig_gpio = of_get_named_gpio(node, "nconfig_gpio", 0);
	if (!gpio_is_valid(info->nconfig_gpio)) {
		dev_err(&pdev->dev, "Invalid NCONFIG GPIO pin!\n");
		return -EINVAL;
	}

	/* disable FPGA lanes by default */
	init_flags = (info->dt_flags & OF_GPIO_ACTIVE_LOW) ?
				GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;
	rc = devm_gpio_request_one(&pdev->dev, info->en_gpio,
		init_flags, "en_gpio");
	if (rc)
		dev_err(&pdev->dev, "EN_GPIO request failed: %d\n", rc);

	mutex_init(&info->mutex_lock);

	rc = sysfs_create_group(&pdev->dev.kobj, &serdes_attr_group);
	if (rc < 0) {
		dev_err(&pdev->dev, "unable to create sysfs node %d\n", rc);
		goto err;
	}

	dev_info(&pdev->dev, "serdes jtag initialized\n");
	ml_serdes_set_pinctrl(&pdev->dev, SERDES_PINCTRL_FPGA_INACTIVE);

	return 0;

err:
	mutex_destroy(&info->mutex_lock);
	dev_err(&pdev->dev, "unable to initialize serdes jtag pins: %d\n", rc);
	return -EAGAIN;
}

static int ml_serdes_remove(struct platform_device *pdev)
{
	int err;
	struct serdes_platform_info *info;

	info = dev_get_platdata(&pdev->dev);
	err = regulator_disable(info->regulator);
	if (err)
		dev_err(&pdev->dev, "Failed to disable regulator: %d\n", err);
	sysfs_remove_group(&pdev->dev.kobj, &serdes_attr_group);
	mutex_destroy(&info->mutex_lock);

	return err;
}

static int ml_serdes_suspend(struct platform_device *pdev, pm_message_t state)
{
	int err;
	struct serdes_platform_info *info = dev_get_platdata(&pdev->dev);

	err = aon_pm_target_deep_idle();
	if (err) {
		dev_err(&pdev->dev, "aon-pm deepidle failed: %d\n", err);
		return err;
	}

	err = regulator_disable(info->regulator);
	if (err)
		dev_err(&pdev->dev, "Failed to disable regulator: %d\n", err);
	return err;
}

static int ml_serdes_resume(struct platform_device *pdev)
{
	int err;
	struct serdes_platform_info *info = dev_get_platdata(&pdev->dev);

	err = regulator_enable(info->regulator);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable regulator: %d\n", err);
		return err;
	}

	err = aon_pm_target_active();
	if (err)
		dev_err(&pdev->dev, "aon-pm active failed: %d\n", err);

	return err;
}

static const struct of_device_id ml_serdes_of_match[] = {
	{ .compatible = "ml,serdes", },
	{ },
};
MODULE_DEVICE_TABLE(of, ml_serdes_of_match);

static struct platform_driver ml_serdes_platform_driver = {
	.driver		= {
		.name	= "ml_serdes",
		.of_match_table = of_match_ptr(ml_serdes_of_match),
	},
	.probe		= ml_serdes_probe,
	.remove		= ml_serdes_remove,
	.suspend	= ml_serdes_suspend,
	.resume		= ml_serdes_resume
};

module_platform_driver(ml_serdes_platform_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("SERDES gpio exports");
