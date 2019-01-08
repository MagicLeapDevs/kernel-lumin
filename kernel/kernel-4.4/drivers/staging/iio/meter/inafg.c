/*
 * Copyright (c) 2017, Magic Leap, Inc. All rights reserved.
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

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/sysfs.h>

/* NVIDIA ThermPro app requires this name */
#define VDD_IN_RAIL_NAME "VDD_IN"

static ssize_t rail_name_0_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	if (!buf)
		return -EFAULT;

	return sprintf(buf, "%s\n", VDD_IN_RAIL_NAME);
}
static DEVICE_ATTR_RO(rail_name_0);

static ssize_t in_power0_input_show(struct device *dev,
				    struct device_attribute *dev_attr,
				    char *buf)
{
	int *pwr = (int *)dev_get_drvdata(dev);

	if (!buf)
		return -EFAULT;

	return sprintf(buf, "%d\n", *pwr);
}

static ssize_t in_power0_input_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int ret;
	int *pwr = (int *)dev_get_drvdata(dev);

	if (!buf)
		return -EFAULT;

	ret = kstrtoint(buf, 10, pwr);
	if (ret < 0)
		return ret;

	return size;
}
static DEVICE_ATTR_RW(in_power0_input);

/* NVIDIA ThermPro app requires these attr names */
static struct attribute *inafg_attributes[] = {
	&dev_attr_rail_name_0.attr,
	&dev_attr_in_power0_input.attr,
	NULL,
};

static const struct attribute_group inafg_attr_group = {
	.attrs = inafg_attributes,
	.name = "iio_device",
};

static int inafg_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int ret;
	int *pwr;

	pwr = devm_kzalloc(&client->dev, sizeof(*pwr), GFP_KERNEL);
	if (!pwr)
		return -ENOMEM;

	dev_info(&client->dev, "inafg_probe\n");

	i2c_set_clientdata(client, pwr);

	ret = sysfs_create_group(&client->dev.kobj, &inafg_attr_group);
	if (ret) {
		dev_err(&client->dev, "Failed in sysfs group: %d\n", ret);
		return ret;
	}
	return 0;
}

static int inafg_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &inafg_attr_group);
	return 0;
}

static const struct i2c_device_id inafg_id[] = {
	{.name = "inafgx",},
	{},
};

static struct i2c_driver inafg_driver = {
	.driver = {
		.name	= "inafgx",
		.owner = THIS_MODULE,
	},
	.probe		= inafg_probe,
	.remove		= inafg_remove,
	.id_table	= inafg_id,
};

module_i2c_driver(inafg_driver);

MODULE_DESCRIPTION("magic leap inafg 1-Channel Power Monitor");
MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_LICENSE("GPL v2");
