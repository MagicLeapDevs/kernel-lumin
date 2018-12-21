/* Copyright (c) 2016, Magic Leap, Inc. All rights reserved.
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

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <sound/jack.h>
#include <linux/input.h>
#include <linux/switch.h>
#include <sound/soc.h>

#include "fsa8500.h"

static struct fsa8500_codec_data *gl_fsa8500_data;
static struct workqueue_struct *gl_jack_workq = NULL;

static const struct regmap_config fsa8500_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x17,
};

static int fsa8500_jack_det(struct device *dev, int ir1, int ir5)
{
	int inserted = FSA8500_JACK_NOCHANGE;
	int detected = 0;

	if (ir1 != 0) {
		dev_dbg(dev, "%s: int1_reg %#x\n", __func__, ir1);
		if ((ir1 & FSA8500_JACK_DISCONNECT) ==
			FSA8500_JACK_DISCONNECT) {
			dev_dbg(dev, "==> Jack Disconnected\n");
			inserted = FSA8500_JACK_REMOVED;
		} else {
			switch (ir1) {
			case FSA8500_JACK_3_POLE:
			case (FSA8500_JACK_3_POLE |
				FSA8500_JACK_LINT):
				dev_dbg(dev, "==> 3-pole.\n");
				detected = 1;
				break;
			case FSA8500_JACK_OMTP:
			case (FSA8500_JACK_OMTP | FSA8500_JACK_LINT):
				dev_dbg(dev, "==> HS-OMTP.\n");
				detected = 1;
				break;
			case FSA8500_JACK_CTIA:
			case (FSA8500_JACK_CTIA | FSA8500_JACK_LINT):
				dev_dbg(dev, "==> HS-CTIA.\n");
				detected = 1;
				break;
			case FSA8500_JACK_UART:
				dev_dbg(dev, "==> UART.\n");
				detected = 0;
				break;
			case FSA8500_JACK_LINT:
				dev_dbg(dev, "==> Lint.\n");
				detected = 0;
				break;
			case FSA8500_JACK_MOISTURE:
				dev_dbg(dev, "==> Moisture.\n");
				detected = 0;
				break;
			case FSA8500_JACK_UNKNOWN:
				dev_dbg(dev, "==> Unknown.\n");
				detected = 1;
				break;
			default:
				dev_dbg(dev, "==> TBD(%#3x).\n", ir1);
				detected = 1;
				break;
			}

			if (detected) {
				switch (ir5 & FSA8500_ACCESSORY_MASK) {
				case FSA8500_ACCESSORY1:
				case FSA8500_ACCESSORY2:
					inserted = FSA8500_JACK_INSERTED;
					break;
				case FSA8500_ACCESSORY0:
				case FSA8500_ACCESSORY3:
					/* HW can be wrong. Need to re-detect */
					inserted = FSA8500_JACK_REDETECT;
					break;
				}
			}
		}
	}

	dev_dbg(dev, "%s: inserted %#x\n", __func__, inserted);

	return inserted;
}

static int fsa8500_button_det(struct device *dev, int ir2)
{
	int report = SND_JACK_HEADPHONE;

	dev_dbg(dev, "%s: ir2 %#x\n", __func__, ir2);

	/* Note: for now, only short button press is supported. */
	switch (ir2) {
	case FSA8500_JACK_BUTTON_NONE:
		break;
	case FSA8500_JACK_BUTTON_1:
		report |= SND_JACK_BTN_3;
		break;
	case FSA8500_JACK_BUTTON_2:
		report |= SND_JACK_BTN_2;
		break;
	case FSA8500_JACK_BUTTON_7:
		report |= SND_JACK_BTN_1;
		break;
	default:
		dev_info(dev, "%s: unsupported button %#x\n",
			__func__, ir2);
	}

	return report;
}

static void fsa8500_reset_jack(struct regmap *regmap)
{
	/* Reset jack detecton. */
	regmap_write(regmap, FSA8500_RESET_REG, FSA8500_JACKDET_RESET);

	/* Setup button detection threshods */
	regmap_write(regmap, FSA8500_DET_TSHOD1_REG, FSA8500_DET_TSHOD1_VAL);
}

static void fsa8500_jack_detect_work(struct work_struct *work)
{
	unsigned int ir1, ir2, ir3, ir4, ir5;
	struct snd_soc_jack *jack = gl_fsa8500_data->hs_jack;
	struct fsa8500_codec_data *cdata = gl_fsa8500_data;
	int report = 0;

	ir1 = ir2 = ir3 = ir4 = ir5 = 0;
	regmap_read(cdata->regmap, FSA8500_INT1_STATUS_REG, &ir1);
	regmap_read(cdata->regmap, FSA8500_INT2_STATUS_REG, &ir2);
	regmap_read(cdata->regmap, FSA8500_INT3_STATUS_REG, &ir3);
	regmap_read(cdata->regmap, FSA8500_INT4_STATUS_REG, &ir4);
	regmap_read(cdata->regmap, FSA8500_INT5_STATUS_REG, &ir5);

	dev_dbg(&cdata->i2c->dev, "%s: INT ST: %#x %#x %#x %#x %#x. JKIN: %d\n",
		__func__, ir1, ir2, ir3, ir4, ir5, cdata->inserted);

	if (jack == NULL) {
		dev_err(&cdata->i2c->dev, "%s: no jack, bye.\n",
			__func__);
		return;
	}

	report = fsa8500_jack_det(&cdata->i2c->dev, ir1, ir5);
	if (report == FSA8500_JACK_REMOVED) {
		report = 0;
		cdata->inserted = 0;
		cdata->redetect = 0;
	} else if (report == FSA8500_JACK_INSERTED) {
		report = SND_JACK_HEADPHONE;
		cdata->inserted = 1;
		cdata->redetect = 0;
	} else if (report == FSA8500_JACK_REDETECT) {
		report = 0;
		cdata->redetect++;
		if (cdata->redetect <= FSA8500_JACK_DET_MAX)
			fsa8500_reset_jack(cdata->regmap);
		else
			cdata->redetect = 0;
	} else if (cdata->inserted) {
		report = fsa8500_button_det(&cdata->i2c->dev, ir2);
	}

	dev_dbg(&cdata->i2c->dev, "%s: report %#x, inserted %d, redetect %d\n",
		__func__, report, cdata->inserted, cdata->redetect);

	aic3206_hp_insertion(cdata->inserted);

	snd_soc_jack_report(jack, report,
				SND_JACK_HEADPHONE |
				SND_JACK_BTN_0 | SND_JACK_BTN_1 |
				SND_JACK_BTN_2 | SND_JACK_BTN_3);
}

static int fsa8500_init(struct device *dev, struct regmap *regmap,
			int *inserted)
{
	unsigned int val = 0;
	unsigned int ir1, ir2, ir3, ir4, ir5;

	fsa8500_reset_jack(regmap);
	regmap_read(regmap, FSA8500_DEV_ID_REG, &val);
	if (val != FSA8500_DEV_ID_VAL) {
		dev_err(dev, "Failed to verify fsa8500(%#x).\n", val);
		return -ENODEV;
	}

	ir1 = ir2 = ir3 = ir4 = ir5 = 0;
	regmap_read(regmap, FSA8500_INT1_STATUS_REG, &ir1);
	regmap_read(regmap, FSA8500_INT2_STATUS_REG, &ir2);
	regmap_read(regmap, FSA8500_INT3_STATUS_REG, &ir3);
	regmap_read(regmap, FSA8500_INT4_STATUS_REG, &ir4);
	regmap_read(regmap, FSA8500_INT5_STATUS_REG, &ir5);

	dev_dbg(dev, "%s: Power-On INT ST: %#x %#x %#x %#x %#x\n",
		__func__, ir1, ir2, ir3, ir4, ir5);

	*inserted = fsa8500_jack_det(dev, ir1, ir5);

	return 0;
}

static irqreturn_t fsa8500_isr(int irq, void *data)
{
	struct fsa8500_codec_data *cdata = data;

	queue_delayed_work(gl_jack_workq,
		&cdata->delayed_work,
		msecs_to_jiffies(FSA8500_JACK_DET_MS));

	return IRQ_HANDLED;
}

int fsa8500_set_hs_jack(struct snd_soc_jack *hs_jack)
{
	if (gl_fsa8500_data == NULL)
		return -ENODEV;

	gl_fsa8500_data->hs_jack = hs_jack;

	if (gl_fsa8500_data->inserted) {
		dev_dbg(&gl_fsa8500_data->i2c->dev, "%s: SND_JACK_HEADPHONE\n",
			__func__);

		aic3206_hp_insertion(gl_fsa8500_data->inserted);
		snd_soc_jack_report(hs_jack, SND_JACK_HEADPHONE,
				SND_JACK_HEADPHONE |
				SND_JACK_BTN_0 | SND_JACK_BTN_1 |
				SND_JACK_BTN_2 | SND_JACK_BTN_3);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(fsa8500_set_hs_jack);

static int fsa8500_i2c_probe(struct i2c_client *i2c,
				 const struct i2c_device_id *i2c_id)
{
	struct regmap *fsa8500_regmap;
	struct fsa8500_codec_data *cdata;
	int inserted, jack_gpio;
	int rc = 0;

	dev_info(&i2c->dev, "%s:\n", __func__);

	fsa8500_regmap = devm_regmap_init_i2c(i2c, &fsa8500_regmap_config);
	if (IS_ERR(fsa8500_regmap)) {
		rc = PTR_ERR(fsa8500_regmap);
		dev_err(&i2c->dev,
			"Failed to allocate register map: %d\n",
			rc);
		return rc;
	}

	rc = fsa8500_init(&i2c->dev, fsa8500_regmap, &inserted);
	if (rc != 0)
		return rc;

	cdata = devm_kzalloc(&i2c->dev,
				sizeof(struct fsa8500_codec_data),
				GFP_KERNEL);
	if (cdata == NULL)
		return -ENOMEM;

	cdata->regmap = fsa8500_regmap;
	cdata->i2c = i2c;
	cdata->inserted = (inserted == FSA8500_JACK_INSERTED);
	i2c_set_clientdata(i2c, cdata);
	gl_fsa8500_data = cdata;

	INIT_DELAYED_WORK(&cdata->delayed_work,
			fsa8500_jack_detect_work);

	gl_jack_workq = create_singlethread_workqueue("fsa8500_jack");
	if (gl_jack_workq == NULL)
		return -ENOMEM;

	jack_gpio = of_get_named_gpio(i2c->dev.of_node, "jack-gpios", 0);
	dev_dbg(&i2c->dev, "%s: jack-gpio %d\n", __func__, jack_gpio);
	if (gpio_is_valid(jack_gpio)) {
		i2c->irq = gpio_to_irq(jack_gpio);
		dev_dbg(&i2c->dev,
			"irq = %d for jack gpio %d\n",
			i2c->irq, jack_gpio);
	} else {
		dev_err(&i2c->dev,
			"%s: invalid gpio %d\n",
			__func__, jack_gpio);
		return -EINVAL;
	}

	if (i2c->irq > 0) {
		rc = request_threaded_irq(i2c->irq,
					NULL, fsa8500_isr,
					IRQF_TRIGGER_FALLING
					| IRQF_TRIGGER_RISING
					| IRQF_ONESHOT,
					"fsa8500", cdata);
		if (rc) {
			dev_err(&i2c->dev,
				"Failed to reguest IRQ: %d\n", rc);
			return -EINVAL;
		}
	} else {
		dev_err(&i2c->dev, "Invalid IRQ: %d\n", i2c->irq);
		return -EINVAL;
	}

	return 0;
}

static int __exit fsa8500_i2c_remove(struct i2c_client *i2c)
{
	dev_info(&i2c->dev, "%s: \n", __func__);

	snd_soc_unregister_codec(&i2c->dev);

	if (i2c->irq)
		free_irq(i2c->irq, gl_fsa8500_data);

	return 0;
}

static const struct i2c_device_id fsa8500_id[] = {
	{"fsa8500", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, fsa8500_id);

static const struct of_device_id fsa8500_of_match[] = {
	{ .compatible = "fc,fsa8500", },
	{ }
};
MODULE_DEVICE_TABLE(of, fsa8500_of_match);

static struct i2c_driver fsa8500_i2c_driver = {
	.driver = {
		.name = "fsa8500",
		.of_match_table = of_match_ptr(fsa8500_of_match),
	},
	.probe = fsa8500_i2c_probe,
	.remove = __exit_p(fsa8500_i2c_remove),
	.id_table = fsa8500_id,
};

module_i2c_driver(fsa8500_i2c_driver);

MODULE_DESCRIPTION("Magic Leap FSA8500 codec driver");
MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_LICENSE("GPL v2");
