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
 * Taken from -
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/suspend.h>
#include <linux/reboot.h>

struct gcm_data {
	struct device *dev;
	int power_gpio;
	int active_low;
	unsigned int irq;
	int wakeup;
	bool key_pressed;
	struct kref reboot_ref;
};

static void gpio_charger_mode_reboot(struct kref *kref)
{
	struct gcm_data *ddata = container_of(kref, struct gcm_data,
				reboot_ref);

	dev_info(ddata->dev, "requesting reboot\n");
	orderly_reboot();
}

static irqreturn_t gpio_charger_mode_irq_isr(int irq, void *dev_id)
{
	struct gcm_data *ddata = dev_id;

	dev_dbg(ddata->dev, "%s\n", __func__);
	if (!ddata->key_pressed) {
		if (ddata->wakeup)
			pm_wakeup_event(ddata->dev, 0);

		ddata->key_pressed = true;
		kref_put(&ddata->reboot_ref, gpio_charger_mode_reboot);
	}

	return IRQ_HANDLED;
}

static struct gcm_data *gpio_charger_mode_parse_dt(struct device *dev)
{
	struct device_node *node;
	struct gcm_data *ddata;
	enum of_gpio_flags flags = OF_GPIO_ACTIVE_LOW;

	node = dev->of_node;
	if (!node)
		return ERR_PTR(-ENODEV);

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return ERR_PTR(-ENOMEM);

	ddata->power_gpio = of_get_gpio_flags(node, 0, &flags);
	ddata->active_low = (flags & OF_GPIO_ACTIVE_LOW);

	ddata->irq = irq_of_parse_and_map(node, 0);

	if (!gpio_is_valid(ddata->power_gpio) && !ddata->irq) {
		dev_err(dev, "could not find gpio or irq\n");
		return ERR_PTR(-EINVAL);
	}

	ddata->wakeup = of_property_read_bool(node, "wakeup-source");

	return ddata;
}

static int gpio_charger_mode_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gcm_data *ddata;
	int ret;
	unsigned long irqflags;
	int irq;

	dev_info(dev, "probe\n");
	ddata = gpio_charger_mode_parse_dt(dev);
	if (IS_ERR(ddata))
		return PTR_ERR(ddata);

	ddata->dev = dev;
	platform_set_drvdata(pdev, ddata);

	ret = devm_gpio_request_one(dev, ddata->power_gpio, GPIOF_IN,
				    "power_irq");
	if (ret < 0) {
		dev_err(dev, "power gpio request failed: %d\n", ret);
		return ret;
	}

	if (!ddata->irq) {
		irq = gpio_to_irq(ddata->power_gpio);
		if (irq < 0) {
			ret = irq;
			dev_err(dev, "unable to get irq %d\n", irq);
			return ret;
		}
		ddata->irq = irq;
	}

	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	ret = devm_request_any_context_irq(dev, ddata->irq,
					gpio_charger_mode_irq_isr,
					irqflags, pdev->name, ddata);
	if (ret) {
		dev_err(dev, "failed to set up irq: %d\n", ret);
		return ret;
	}

	kref_init(&ddata->reboot_ref);
	device_init_wakeup(dev, ddata->wakeup);

	return ret;
}

static int gpio_charger_mode_remove(struct platform_device *pdev)
{
	device_init_wakeup(&pdev->dev, 0);

	return 0;
}

static int gpio_charger_mode_suspend(struct device *dev)
{
	struct gcm_data *ddata = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		enable_irq_wake(ddata->irq);

	return 0;
}

static int gpio_charger_mode_suspend_noirq(struct device *dev)
{
	struct gcm_data *ddata = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	kref_get(&ddata->reboot_ref);

	return 0;
}

static int gpio_charger_mode_resume(struct device *dev)
{
	struct gcm_data *ddata = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	if (device_may_wakeup(dev))
		disable_irq_wake(ddata->irq);

	return 0;
}

static void gpio_charger_mode_complete(struct device *dev)
{
	struct gcm_data *ddata = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	kref_put(&ddata->reboot_ref, gpio_charger_mode_reboot);
}

static const struct dev_pm_ops gpio_charger_mode_pm_ops = {
	.complete = gpio_charger_mode_complete,
	.suspend_noirq = gpio_charger_mode_suspend_noirq,
	.resume = gpio_charger_mode_resume,
	.suspend = gpio_charger_mode_suspend,
};

static const struct of_device_id gpio_charger_mode_of_match[] = {
	{ .compatible = "ml,gpio-charger-mode", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_charger_mode_of_match);

static struct platform_driver gpio_charger_mode_device_driver = {
	.probe		= gpio_charger_mode_probe,
	.remove		= gpio_charger_mode_remove,
	.driver		= {
		.name	= "gpio-charger-mode",
		.pm	= &gpio_charger_mode_pm_ops,
		.of_match_table = gpio_charger_mode_of_match,
	}
};

static int __init gpio_charger_mode_init(void)
{
	return platform_driver_register(&gpio_charger_mode_device_driver);
}

static void __exit gpio_charger_mode_exit(void)
{
	platform_driver_unregister(&gpio_charger_mode_device_driver);
}

postcore_initcall(gpio_charger_mode_init);
module_exit(gpio_charger_mode_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("GPIO handler for charger mode");
MODULE_ALIAS("platform:gpio-charger-mode");
