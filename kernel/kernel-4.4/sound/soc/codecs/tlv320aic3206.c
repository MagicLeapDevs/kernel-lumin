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

#include "tlv320aic3206.h"

static struct aic3206_codec_data *gl_codec_data;

static const struct reg_default aic3206_reg[] = {
	{ 0x00, 0x0 },
	{ 0x00, 0x1 },
};

static const struct regmap_config aic3206_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x80,
};

static void aic3206_codec_init(struct device *dev,
				struct regmap *regmap);

static int aic3206_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	return 0;
}

void aic3206_hp_insertion(int hp_inserted)
{
	if (hp_inserted) {
		regmap_write(gl_codec_data->regmap,
				AIC3206_PAGE_REG, AIC3206_PAGE_1);
		regmap_write(gl_codec_data->regmap,
				AIC3206_OPWR_REG, AIC3206_OPWR_UP);
	} else {
		regmap_write(gl_codec_data->regmap,
				AIC3206_PAGE_REG, AIC3206_PAGE_1);
		regmap_write(gl_codec_data->regmap,
				AIC3206_OPWR_REG, AIC3206_OPWR_DOWN);
	}
}
EXPORT_SYMBOL(aic3206_hp_insertion);

static int aic3206_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	regmap_write(gl_codec_data->regmap,
			AIC3206_PAGE_REG, AIC3206_PAGE_0);
	regmap_write(gl_codec_data->regmap,
			AIC3206_DAC_SETUP2_REG, AIC3206_DAC_UNMUTE);

	return 0;
}

static void aic3206_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	regmap_write(gl_codec_data->regmap,
			AIC3206_PAGE_REG, AIC3206_PAGE_0);
	regmap_write(gl_codec_data->regmap,
			AIC3206_DAC_SETUP2_REG, AIC3206_DAC_MUTE);
	return;
}

static const struct snd_soc_dai_ops aic3206_dai_ops = {
	.prepare	= aic3206_pcm_prepare,
	.hw_params	= aic3206_hw_params,
	.shutdown	= aic3206_shutdown,
};

static struct snd_soc_dai_driver aic3206_dai = {
	.name = "tlv320aic3206-hifi",
	.playback = {
			.stream_name = "Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = AIC3206_RATES,
			.formats = AIC3206_FORMATS,
	},
	.capture = {
			.stream_name = "Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = AIC3206_RATES,
			.formats = AIC3206_FORMATS,
	},
	.ops = &aic3206_dai_ops,
};

static void aic3206_codec_init(struct device *dev,
				struct regmap *regmap)
{
	dev_dbg(dev, "%s:\n", __func__);

	/* Headphone playback */
	regmap_write(regmap, AIC3206_CLK_SET1_REG, AIC3206_PLL_CLKIN);
	regmap_write(regmap, AIC3206_CLK6_NDAC_REG, AIC3206_NDAC_2);
	regmap_write(regmap, AIC3206_CLK7_MDAC_REG, AIC3206_MDAC_7);
	regmap_write(regmap, AIC3206_OSR_SET1_REG, AIC3206_DOSR_MSB_0);
	regmap_write(regmap, AIC3206_OSR_SET2_REG, AIC3206_DOSR_LSB_128);
	regmap_write(regmap, AIC3206_CLK8_NDAC_REG, AIC3206_NDAC_7);
	regmap_write(regmap, AIC3206_CLK9_MDAC_REG, AIC3206_MDAC_2);

	regmap_write(regmap, AIC3206_AOSR_REG, AIC3206_AOSR_128);
	regmap_write(regmap, AIC3206_CLK3_J_REG, AIC3206_CLK3_J_7);
	regmap_write(regmap, AIC3206_CLK4_D_MSB_REG, AIC3206_CLK4_D_MSB_0);
	regmap_write(regmap, AIC3206_CLK5_D_LSB_REG, AIC3206_CLK5_D_LSB_0);
	regmap_write(regmap, AIC3206_CLK2_PLLPR_REG, AIC3206_CLK2_PLLPR_P1R4);
	regmap_write(regmap, AIC3206_DAC_PRB_REG, AIC3206_DAC_PRB_P4);

	regmap_write(regmap, AIC3206_DAC_LVOL_REG, AIC3206_DAC_VOL_N10DB);
	regmap_write(regmap, AIC3206_DAC_RVOL_REG, AIC3206_DAC_VOL_N10DB);

	regmap_write(regmap, AIC3206_PAGE_REG, AIC3206_PAGE_1);
	regmap_write(regmap, AIC3206_CPC_REG, AIC3206_CPC_CD6);
	regmap_write(regmap, AIC3206_PWRC1_REG, AIC3206_NOAVDD_OSC);
	regmap_write(regmap, AIC3206_PWRC2_REG, AIC3206_ANABLK_EN);
	regmap_write(regmap, AIC3206_REF_PWRUP_REG, AIC3206_REF_PWRUP_40MS);
	regmap_write(regmap, AIC3206_HPL_ROUTE_REG, AIC3206_HPL_LDACP);
	regmap_write(regmap, AIC3206_HPR_ROUTE_REG, AIC3206_HPR_RDACP);
	regmap_write(regmap, AIC3206_LOL_ROUTE_REG, AIC3206_LOL_LDACP);
	regmap_write(regmap, AIC3206_LOR_ROUTE_REG, AIC3206_LOR_RDACP);
	regmap_write(regmap, AIC3206_PAGE_REG, AIC3206_PAGE_0);
	regmap_write(regmap, AIC3206_DAC_SETUP1_REG, AIC3206_DAC_LL_RR_1S2C);
	regmap_write(regmap, AIC3206_PAGE_REG, AIC3206_PAGE_1);
	regmap_write(regmap, AIC3206_HP_DRIVER_REG, AIC3206_HP_GCM_DCOC);
	regmap_write(regmap, AIC3206_HPL_GAIN_REG, AIC3206_HP_GAIN_0DB);
	regmap_write(regmap, AIC3206_HPR_GAIN_REG, AIC3206_HP_GAIN_0DB);
	regmap_write(regmap, AIC3206_OPWR_REG, AIC3206_OPWR_UP);

	/* MIC rec IN3L */
	regmap_write(regmap, AIC3206_PAGE_REG, AIC3206_PAGE_1);
	regmap_write(regmap, AIC3206_MICBIAS_REG, AIC3206_MICBIAS_UP);
	regmap_write(regmap, AIC3206_LMICPGA_PT_REG, AIC3206_MICPGA_IN3L_10K);
	regmap_write(regmap, AIC3206_LMICPGA_NT_REG, AIC3206_MICPGA_IN3R_10K);
	regmap_write(regmap, AIC3206_FLOAT_IN_REG, AIC3206_FLOAT_IN3L_CM);
	regmap_write(regmap, AIC3206_LMICPGA_VOL_REG, AIC3206_LMICPGA_0DB);
	regmap_write(regmap, AIC3206_ADC_VOL_REG, AIC3206_ADC_LVOL_PG);
	regmap_write(regmap, AIC3206_PAGE_REG, AIC3206_PAGE_0);
	regmap_write(regmap, AIC3206_ADC_SETUP_REG, AIC3206_ADC_L_UP);
	regmap_write(regmap, AIC3206_ADC_FGAIN_REG, AIC3206_ADC_R_MUTED);
}

static int aic3206_suspend(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s: \n", __func__);
	return 0;
}

static int aic3206_resume(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s: \n", __func__);
	return 0;
}

static int aic3206_codec_probe(struct snd_soc_codec *codec)
{
	unsigned int val;

	dev_info(codec->dev, "%s:\n", __func__);

	gl_codec_data->codec = codec;

	if (gpio_is_valid(gl_codec_data->reset_gpio)) {
		udelay(1);
		gpio_set_value(gl_codec_data->reset_gpio, 1);
		/* Registers init will take no more than 1ms. */
		usleep_range(1000, 2000);
	} else {
		dev_err(&gl_codec_data->i2c->dev,
			"%s: invalid gpio %d\n", __func__, gl_codec_data->reset_gpio);
		return -EINVAL;
	}

	regmap_read(gl_codec_data->regmap, AIC3206_DEV_ID_REG, &val);
	if (val != AIC3206_DEV_ID_VAL) {
		dev_err(&gl_codec_data->i2c->dev,
			"Failed to verify aic3206(%#x).\n", val);
		return -ENODEV;
	}

	aic3206_codec_init(&gl_codec_data->i2c->dev,
				gl_codec_data->regmap);

	return 0;
}

static int aic3206_remove(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s: \n", __func__);
	return 0;
}

static const struct snd_kcontrol_new aic3206_snd_controls[] = {
};

static const struct snd_soc_dapm_widget aic3206_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_INPUT("IN"),
};

static const struct snd_soc_dapm_route aic3206_dapm_routes[] = {
	{ "OUT", NULL, "Playback" },
	{ "Capture", NULL, "IN" },
};

static struct snd_soc_codec_driver soc_codec_dev_aic3206 = {
	.probe = aic3206_codec_probe,
	.remove = aic3206_remove,
	.suspend = aic3206_suspend,
	.resume = aic3206_resume,
	.controls = aic3206_snd_controls,
	.num_controls = ARRAY_SIZE(aic3206_snd_controls),
	.dapm_widgets = aic3206_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aic3206_dapm_widgets),
	.dapm_routes = aic3206_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(aic3206_dapm_routes),
};

static int aic3206_i2c_probe(struct i2c_client *i2c,
				 const struct i2c_device_id *i2c_id)
{
	struct regmap *aic3206_regmap;
	struct aic3206_codec_data *cdata;
	int reset_gpio, rc;

	dev_info(&i2c->dev, "%s:\n", __func__);

	aic3206_regmap = devm_regmap_init_i2c(i2c, &aic3206_regmap_config);
	if (IS_ERR(aic3206_regmap)) {
		rc = PTR_ERR(aic3206_regmap);
		dev_err(&i2c->dev,
			"Failed to allocate register map: %d\n",
			rc);
		return rc;
	}

	cdata = devm_kzalloc(&i2c->dev,
				sizeof(struct aic3206_codec_data),
				GFP_KERNEL);
	if (cdata == NULL)
		return -ENOMEM;

	cdata->regmap = aic3206_regmap;
	cdata->i2c = i2c;

	i2c_set_clientdata(i2c, cdata);

	gl_codec_data = cdata;

	reset_gpio = of_get_named_gpio(i2c->dev.of_node, "reset-gpios", 0);
	dev_dbg(&i2c->dev, "%s: reset-gpio %d\n", __func__, reset_gpio);
	if (gpio_is_valid(reset_gpio)) {
		rc = devm_gpio_request_one(&i2c->dev, reset_gpio,
						GPIOF_OUT_INIT_LOW,
						"tlv320aic3206 reset");
		if (rc != 0) {
			dev_err(&i2c->dev,
				"Failed to request reset gpio %d\n", rc);
			return rc;
		}
	} else {
		dev_err(&i2c->dev,
			"%s: invalid gpio %d\n", __func__, reset_gpio);
		return -EINVAL;
	}
	gl_codec_data->reset_gpio = reset_gpio;

	dev_dbg(&i2c->dev, "%s: register aic3206 codec\n", __func__);
	rc = snd_soc_register_codec(&i2c->dev,
				&soc_codec_dev_aic3206,
				&aic3206_dai, 1);

	if (rc) {
		dev_err(&i2c->dev,
			"Failed to register aic3206 asoc codec %d\n", rc);
	}

	return rc;
}

static int __exit aic3206_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static const struct i2c_device_id tlc320aic3206_id[] = {
	{"tlv320aic3206", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tlc320aic3206_id);

static const struct of_device_id tlv320aic3206_of_match[] = {
	{ .compatible = "ti,tlv320aic3206", },
	{ }
};
MODULE_DEVICE_TABLE(of, tlv320aic3206_of_match);

static struct i2c_driver aic3206_i2c_driver = {
	.driver = {
		.name = "tlv320aic3206",
		.of_match_table = of_match_ptr(tlv320aic3206_of_match),
	},
	.probe = aic3206_i2c_probe,
	.remove = __exit_p(aic3206_i2c_remove),
	.id_table = tlc320aic3206_id,
};

module_i2c_driver(aic3206_i2c_driver);

MODULE_DESCRIPTION("Magic Leap TLV320AIC3206 codec driver");
MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_LICENSE("GPL v2");
