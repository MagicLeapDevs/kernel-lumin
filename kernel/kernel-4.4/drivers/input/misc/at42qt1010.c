/*
 * Atmel AT42QT1010 Single-key QTouch Touch Sensor Driver
 *
 *
 * Copyright (c) 2016-2017, Magic Leap, Inc. All rights reserved.
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

/* Number of ms to wait after turning on regulator */
#define REG_ENABLE_DELAY_MS	100

/* Heartbeat pulse width */
#define AT42QT1010_HEARTBEAT_PULSE_WIDTH_US 15

struct at42qt1010_data {
	struct device *dev;
	struct input_dev *input_dev;
	struct regulator *regulator;
	struct work_struct work;
	struct mutex mutex;
	int qtouch_irq;
	int mode_gpio;
	bool pwr_on;
	bool irq_enabled;
	u8 enable_on_suspend;
};

enum at42qt1010_mode {
	MODE_LOW_POWER,
	MODE_FAST,
	MODE_SYNC
};

static irqreturn_t at42qt1010_irq(int irq, void *data)
{
	struct at42qt1010_data *qtouch_data = (struct at42qt1010_data *) data;
	int state;

	pm_wakeup_event(qtouch_data->input_dev->dev.parent, 0);
	state = gpio_get_value(qtouch_data->qtouch_irq);
	if (unlikely(state < 0)) {
		dev_err(qtouch_data->dev,
			"Error getting gpio value for qtouch_irq: %d\n", state);
	} else {
		input_report_key(qtouch_data->input_dev, KEY_BATTERY, state);
		input_sync(qtouch_data->input_dev);
	}

	return IRQ_HANDLED;
}

static void at42qt1010_pwr(struct at42qt1010_data *data, bool enable)
{
	int err;

	mutex_lock(&data->mutex);
	if (enable != data->pwr_on) {
		if (enable) {
			err = regulator_enable(data->regulator);
			if (!err) {
				if (!data->irq_enabled) {
					msleep_interruptible(REG_ENABLE_DELAY_MS);
					enable_irq(gpio_to_irq(data->qtouch_irq));
					data->irq_enabled = true;
				}
				data->pwr_on = true;
			} else {
				dev_err(data->dev,
					"Failed to enable regulator: %d\n",
					err);
			}
		} else /* disable */ {
			if (data->irq_enabled) {
				disable_irq(gpio_to_irq(data->qtouch_irq));
				data->irq_enabled = false;
			}
			err = regulator_disable(data->regulator);
			if (err)
				dev_err(data->dev,
					"Failed to disable regulator: %d\n",
					err);
			else
				data->pwr_on = false;
		}
	}
	mutex_unlock(&data->mutex);
}

static void at42qt1010_work(struct work_struct *work)
{
	struct at42qt1010_data *qtouch_data = container_of(work,
		struct at42qt1010_data, work);

	at42qt1010_pwr(qtouch_data, true);
}

static int at42qt1010_probe(struct platform_device *pdev)
{
	int err;
	struct device_node *node;
	struct at42qt1010_data *qtouch_data;
	const char *mode;
	int value = MODE_LOW_POWER;

	qtouch_data = devm_kzalloc(&pdev->dev, sizeof(*qtouch_data),
				   GFP_KERNEL);
	if (!qtouch_data)
		return -ENOMEM;

	qtouch_data->regulator = devm_regulator_get(&pdev->dev, "pwr");
	if (IS_ERR(qtouch_data->regulator)) {
		dev_err(&pdev->dev, "Unable to read pwr regulator!\n");
		return PTR_ERR(qtouch_data->regulator);
	}

	qtouch_data->dev = &pdev->dev;
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "of_node not found!\n");
		return -ENOENT;
	}

	node = pdev->dev.of_node;
	qtouch_data->qtouch_irq = of_get_named_gpio(node, "gpio_irq", 0);
	if (!gpio_is_valid(qtouch_data->qtouch_irq)) {
		dev_err(&pdev->dev, "Invalid qtouch_irq GPIO pin!\n");
		return -EINVAL;
	}

	err = devm_gpio_request_one(&pdev->dev, qtouch_data->qtouch_irq,
		GPIOF_IN, "qtouch_irq");
	if (err) {
		dev_err(&pdev->dev, "qtouch_irq GPIO request failed: %d\n",
			err);
		return err;
	}

	gpio_set_debounce(qtouch_data->qtouch_irq,
				AT42QT1010_HEARTBEAT_PULSE_WIDTH_US * 2);

	qtouch_data->mode_gpio = of_get_named_gpio(node, "gpio_sync", 0);
	if (!gpio_is_valid(qtouch_data->mode_gpio)) {
		dev_err(&pdev->dev, "Invalid mode_gpio GPIO pin!\n");
		return -EINVAL;
	}

	err = devm_gpio_request(&pdev->dev, qtouch_data->mode_gpio,
		"qtouch_mode");
	if (err) {
		dev_err(&pdev->dev, "mode_gpio GPIO request failed: %d\n", err);
		return err;
	}

	err = of_property_read_string(node, "run_mode", &mode);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to read run_mode property: %d\n",
			err);
		return err;
	}

	/* Select run mode */
	if (strcmp(mode, "fast") == 0)
		value = MODE_FAST;
	else if (strcmp(mode, "low_power") == 0)
		value = MODE_LOW_POWER;
	else if (strcmp(mode, "sync") == 0) {
		dev_err(&pdev->dev, "Sync mode not supported\n");
			return -EPROTONOSUPPORT;
	}
	err = gpio_direction_output(qtouch_data->mode_gpio, value);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed gpio direction output: %d\n", err);
		return err;
	}

	err = of_property_read_u8(node, "enable_on_suspend",
		&qtouch_data->enable_on_suspend);
	if (err) {
		dev_err(&pdev->dev, "Failed to read enable_on_suspend property: %d\n",
			err);
		return err;
	}

	qtouch_data->input_dev = devm_input_allocate_device(&pdev->dev);
	if (!qtouch_data->input_dev)
		return -ENOMEM;

	qtouch_data->input_dev->name = "at42qt1010-qtouch";
	qtouch_data->input_dev->dev.parent = &pdev->dev;

	input_set_capability(qtouch_data->input_dev, EV_KEY, KEY_BATTERY);

	err = devm_request_threaded_irq(&pdev->dev,
		gpio_to_irq(qtouch_data->qtouch_irq), NULL, at42qt1010_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		pdev->name, qtouch_data);

	if (err) {
		dev_err(&pdev->dev, "Failed to request threaded irq: %d\n",
			err);
		return err;
	}
	disable_irq_nosync(gpio_to_irq(qtouch_data->qtouch_irq));

	err = input_register_device(qtouch_data->input_dev);
	platform_set_drvdata(pdev, qtouch_data);

	if (!err) {
		qtouch_data->pwr_on = false;
		qtouch_data->irq_enabled = false;
		mutex_init(&qtouch_data->mutex);
		INIT_WORK(&qtouch_data->work, at42qt1010_work);
		schedule_work(&qtouch_data->work);
	}

	return err;
}

static int at42qt1010_remove(struct platform_device *pdev)
{
	struct at42qt1010_data *qtouch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&qtouch_data->work);
	at42qt1010_pwr(qtouch_data, false);
	mutex_destroy(&qtouch_data->mutex);
	input_unregister_device(qtouch_data->input_dev);

	return 0;
}

static int at42qt1010_resume(struct platform_device *pdev)
{
	struct at42qt1010_data *qtouch_data = platform_get_drvdata(pdev);

	if (!qtouch_data->enable_on_suspend)
		schedule_work(&qtouch_data->work);

	return 0;
}

static int at42qt1010_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct at42qt1010_data *qtouch_data = platform_get_drvdata(pdev);

	if (!qtouch_data->enable_on_suspend) {
		cancel_work_sync(&qtouch_data->work);
		at42qt1010_pwr(qtouch_data, false);
	}

	return 0;
}

static const struct of_device_id at42qt1010_of_match[] = {
	{ .compatible = "atmel,at42qt1010", },
	{ }
};
MODULE_DEVICE_TABLE(of, at42qt1010_of_match);

static struct platform_driver at42qt1010_device_driver = {
	.probe		= at42qt1010_probe,
	.remove		= at42qt1010_remove,
	.suspend	= at42qt1010_suspend,
	.resume		= at42qt1010_resume,
	.driver		= {
		.name	= "atmel_at42qt1010_qtouch",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(at42qt1010_of_match),
	}
};
module_platform_driver(at42qt1010_device_driver);

MODULE_DESCRIPTION("Atmel AT42QT1010 QTouch Touch Sensor");
MODULE_AUTHOR("Magic Leap Inc.");
MODULE_LICENSE("GPL v2");

