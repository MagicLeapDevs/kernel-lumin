
/* Copyright (c) 2018, Magic Leap, Inc. All rights reserved.
 * Copyright (C) 2010 Steven King <sfking@fdwdc.com>
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
 * Texas Instruments TMP108 SMBus temperature sensor driver
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/thermal.h>
#include <linux/of.h>

#define	DRIVER_NAME "tmp108"

#define	TMP108_TEMP_REG		0x00
#define	TMP108_CONF_REG		0x01
#define	TMP108_TLOW_REG		0x02
#define	TMP108_THIGH_REG	0x03

/* note: these bit definitions are byte swapped */
#define	TMP108_CONF_M0		0x0100
#define	TMP108_CONF_M1		0x0200
#define	TMP108_CONF_TM		0x0400
#define	TMP108_CONF_FL		0x0800
#define	TMP108_CONF_FH		0x1000
#define	TMP108_CONF_CR0		0x2000
#define	TMP108_CONF_CR1		0x4000
#define	TMP108_CONF_ID		0x8000
#define	TMP108_CONF_HYS0	0x0010
#define	TMP108_CONF_HYS1	0x0020
#define	TMP108_CONF_POL		0x0080

#define TMP108_RESOLUTION	(12)
#define TMP108_TREAD		(0)
#define TMP108_TLOW		(1)
#define TMP108_THIGH		(2)

/* straight from the datasheet */
#define TMP108_TEMP_MIN (-40000)
#define TMP108_TEMP_MAX (125000)

struct tmp108 {
	struct i2c_client *client;
	struct device *hwmon_dev;
	struct thermal_zone_device *tz;
	struct mutex lock;
	u16 config_orig;
	unsigned long last_update;
	int temp[3];  /* temperature, 0 = read, 1 = low, 2 = high */
	bool first_time;
	bool skip_programming;
	bool no_sleep;
};

/* convert left adjusted 12-bit TMP108 register value to milliCelsius */
static inline int tmp108_reg_to_mc(s16 temp)
{
	return ((temp >> (16 - TMP108_RESOLUTION)) * 1000) >>
			(TMP108_RESOLUTION - 8);
}

/* convert milliCelsius to left adjusted 12-bit TMP108 register value */
static inline u16 tmp108_mc_to_reg(int val)
{
	return DIV_ROUND_CLOSEST(val  << (TMP108_RESOLUTION - 8),
			1000) << (16 - TMP108_RESOLUTION);
}

static const u8 tmp108_reg[] = {
	TMP108_TEMP_REG,
	TMP108_TLOW_REG,
	TMP108_THIGH_REG,
};

static struct tmp108 *tmp108_update_device(struct device *dev)
{
	struct tmp108 *tmp108 = dev_get_drvdata(dev);
	struct i2c_client *client = tmp108->client;
	int status;
	int i;

	mutex_lock(&tmp108->lock);
	if (time_after(jiffies, tmp108->last_update + HZ)) {
		for (i = 0; i < ARRAY_SIZE(tmp108->temp); ++i) {
			status = i2c_smbus_read_word_swapped(client,
					tmp108_reg[i]);
			if (status >= 0)
				tmp108->temp[i] = tmp108_reg_to_mc(status);
		}
		tmp108->last_update = jiffies;
		tmp108->first_time = false;
	}
	mutex_unlock(&tmp108->lock);
	return tmp108;
}

static int tmp108_read_temp(void *dev, int *temp)
{
	struct tmp108 *tmp108 = tmp108_update_device(dev);

	/* Is it too early even to return a conversion? */
	if (tmp108->first_time) {
		dev_dbg(dev, "%s: Conversion not ready yet...\n", __func__);
		return -EAGAIN;
	}

	*temp = tmp108->temp[TMP108_TREAD];

	return 0;
}

static ssize_t tmp108_show_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute *sda = to_sensor_dev_attr(attr);
	struct tmp108 *tmp108 = tmp108_update_device(dev);

	/* Is it too early even to return a read? */
	if (tmp108->first_time)
		return -EAGAIN;

	return scnprintf(buf, PAGE_SIZE, "%d\n", tmp108->temp[sda->index]);
}

static ssize_t tmp108_set_temp(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sensor_device_attribute *sda = to_sensor_dev_attr(attr);
	struct tmp108 *tmp108 = dev_get_drvdata(dev);
	struct i2c_client *client = tmp108->client;
	long val;
	int status;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;
	val = clamp_val(val, TMP108_TEMP_MIN, TMP108_TEMP_MAX);

	mutex_lock(&tmp108->lock);
	tmp108->temp[sda->index] = val;
	status = i2c_smbus_write_word_swapped(client, tmp108_reg[sda->index],
			tmp108_mc_to_reg(val));
	mutex_unlock(&tmp108->lock);
	return status ? : count;
}

static SENSOR_DEVICE_ATTR(temp, S_IRUGO, tmp108_show_temp, NULL, 0);

static SENSOR_DEVICE_ATTR(temp_low, S_IWUSR | S_IRUGO, tmp108_show_temp,
			  tmp108_set_temp, 1);

static SENSOR_DEVICE_ATTR(temp_high, S_IWUSR | S_IRUGO, tmp108_show_temp,
			  tmp108_set_temp, 2);

static struct attribute *tmp108_attrs[] = {
	&sensor_dev_attr_temp.dev_attr.attr,
	&sensor_dev_attr_temp_low.dev_attr.attr,
	&sensor_dev_attr_temp_high.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(tmp108);

/* set TMP108_CONFIG correctly if skip-programming option is not used */
#define TMP108_CONFIG  (TMP108_CONF_M1 | TMP108_CONF_CR0 | TMP108_CONF_HYS0)
#define TMP108_CONFIG_IGNORE  (TMP108_CONF_FL | TMP108_CONF_FH)

static const struct thermal_zone_of_device_ops tmp108_of_thermal_ops = {
	.get_temp = tmp108_read_temp,
};

static int tmp108_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct device_node *node = client->dev.of_node;
	struct tmp108 *tmp108;
	const char *sensor_name;
	int status;
	u32 temp;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(dev, "SMBus word transactions not supported\n");
		return -ENODEV;
	}

	tmp108 = devm_kzalloc(dev, sizeof(*tmp108), GFP_KERNEL);
	if (!tmp108)
		return -ENOMEM;

	i2c_set_clientdata(client, tmp108);
	tmp108->client = client;

	sensor_name = of_get_property(node, "sensor-name", NULL);
	if (sensor_name == NULL) {
		dev_err(dev, "error: sensor name not found\n");
		return -ENOENT;
	}

	/* back up config reg even if skip programming; for resume */
	status = i2c_smbus_read_word_swapped(client, TMP108_CONF_REG);
	if (status < 0) {
		dev_err(dev, "error reading config register\n");
		return status;
	}
	tmp108->config_orig = status;

	/* skip-programming can be used for pre-programmed parts */
	tmp108->skip_programming = of_property_read_bool(node,
			"skip-programming");

	if (!tmp108->skip_programming) {
		status = i2c_smbus_write_word_swapped(client, TMP108_CONF_REG,
				TMP108_CONFIG);
		if (status < 0) {
			dev_err(dev, "error writing config register\n");
			goto fail_restore_config;
		}
		status = i2c_smbus_read_word_swapped(client, TMP108_CONF_REG);
		if (status < 0) {
			dev_err(dev, "error reading config register\n");
			goto fail_restore_config;
		}
		status &= ~TMP108_CONFIG_IGNORE;
		if (status != TMP108_CONFIG) {
			dev_err(dev, "error: config settings did not stick\n");
			status = -ENODEV;
			goto fail_restore_config;
		}
	}

	status = of_property_read_u32(node, "temp-high", &temp);
	if (status == 0) {
		/* Write the temperature to high temp register */
		tmp108->temp[TMP108_THIGH] = temp;
		status = i2c_smbus_write_word_swapped(client, TMP108_THIGH_REG,
				tmp108_mc_to_reg(temp));
		if (status < 0) {
			dev_err(dev, "error writing high temp register\n");
			goto fail_restore_config;
		}
	}

	tmp108->no_sleep = of_property_read_bool(node, "no-sleep");

	tmp108->last_update = jiffies;
	/* Mark that we are not ready with data until conversion is complete */
	tmp108->first_time = true;
	mutex_init(&tmp108->lock);

	hwmon_dev = hwmon_device_register_with_groups(dev, sensor_name,
			tmp108, tmp108_groups);
	if (IS_ERR(hwmon_dev)) {
		dev_err(dev, "unable to register tmp108 hwmon device\n");
		status = PTR_ERR(hwmon_dev);
		goto fail_mutex_destroy;
	}
	tmp108->hwmon_dev = hwmon_dev;
	tmp108->tz = thermal_zone_of_sensor_register(hwmon_dev, 0, hwmon_dev,
			&tmp108_of_thermal_ops);
	if (IS_ERR(tmp108->tz)) {
		dev_err(dev, "unable to register tmp108 thermal zone\n");
		tmp108->tz = NULL;
	}

	dev_info(dev, "initialized\n");

	return 0;

fail_mutex_destroy:
	mutex_destroy(&tmp108->lock);
fail_restore_config:
	if (!tmp108->skip_programming)
		i2c_smbus_write_word_swapped(client, TMP108_CONF_REG,
				tmp108->config_orig);
	return status;
}

static int tmp108_remove(struct i2c_client *client)
{
	struct tmp108 *tmp108 = i2c_get_clientdata(client);
	int status = 0;

	thermal_zone_of_sensor_unregister(tmp108->hwmon_dev, tmp108->tz);
	hwmon_device_unregister(tmp108->hwmon_dev);

	if (!tmp108->skip_programming)
		status = i2c_smbus_write_word_swapped(client, TMP108_CONF_REG,
				tmp108->config_orig);
	mutex_destroy(&tmp108->lock);
	return status;
}

#ifdef CONFIG_PM_SLEEP
static int tmp108_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp108 *tmp108 = i2c_get_clientdata(client);
	int config;

	if (tmp108->no_sleep)
		return 0;

	config = i2c_smbus_read_word_swapped(client, TMP108_CONF_REG);
	if (config < 0)
		return config;

	config &= ~(TMP108_CONF_M0 | TMP108_CONF_M1);
	return i2c_smbus_write_word_swapped(client, TMP108_CONF_REG, config);
}

static int tmp108_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp108 *tmp108 = i2c_get_clientdata(client);
	int config;

	if (tmp108->no_sleep)
		return 0;

	if (tmp108->skip_programming)
		config = tmp108->config_orig;
	else
		config = TMP108_CONFIG;

	return i2c_smbus_write_word_swapped(client, TMP108_CONF_REG, config);
}
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(tmp108_dev_pm_ops, tmp108_suspend, tmp108_resume);

static const struct i2c_device_id tmp108_id[] = {
	{ "tmp108", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmp108_id);

static struct i2c_driver tmp108_driver = {
	.driver.name	= DRIVER_NAME,
	.driver.pm	= &tmp108_dev_pm_ops,
	.probe		= tmp108_probe,
	.remove		= tmp108_remove,
	.id_table	= tmp108_id,
};

module_i2c_driver(tmp108_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("Texas Instruments TMP108 temperature sensor driver");
MODULE_LICENSE("GPL v2");
