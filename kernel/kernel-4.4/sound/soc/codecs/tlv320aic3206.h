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

#ifndef __AIC3206_H__
#define __AIC3206_H__

struct aic3206_codec_data {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct snd_soc_codec *codec;
	int reset_gpio;
};

#define AIC3206_RATES	SNDRV_PCM_RATE_8000_96000
#define AIC3206_FORMATS	( SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_3LE | \
			SNDRV_PCM_FMTBIT_S32_LE)

/* AIC3206 register page-0. */
#define AIC3206_PAGE_REG	0x0
#define AIC3206_DEV_ID_REG	0x5
#define AIC3206_SWRST_REG	0x1
#define AIC3206_CLK_SET1_REG	0x4
#define AIC3206_CLK6_NDAC_REG	0xb
#define AIC3206_CLK7_MDAC_REG	0xc
#define AIC3206_OSR_SET1_REG	0xd
#define AIC3206_OSR_SET2_REG	0xe
#define AIC3206_CLK8_NDAC_REG	0x12
#define AIC3206_CLK9_MDAC_REG	0x13
#define AIC3206_AOSR_REG	0x14
#define AIC3206_CLK3_J_REG	0x6
#define AIC3206_CLK4_D_MSB_REG	0x7
#define AIC3206_CLK5_D_LSB_REG	0x8
#define AIC3206_CLK2_PLLPR_REG	0x5
#define AIC3206_AUDIF_REG	0x1b
#define AIC3206_DAC_PRB_REG	0x3c
#define AIC3206_DAC_SETUP1_REG	0x3f
#define AIC3206_DAC_SETUP2_REG	0x40
#define AIC3206_DAC_LVOL_REG	0x41
#define AIC3206_DAC_RVOL_REG	0x42

/* AIC3206 register page-1. */
#define AIC3206_OPWR_REG	0x9
#define AIC3206_CPC_REG		0x7c
#define AIC3206_PWRC1_REG	0x1
#define AIC3206_PWRC2_REG	0x2
#define AIC3206_REF_PWRUP_REG	0x7b
#define AIC3206_HPL_ROUTE_REG	0xc
#define AIC3206_HPR_ROUTE_REG	0xd
#define AIC3206_LOL_ROUTE_REG	0xe
#define AIC3206_LOR_ROUTE_REG	0xf
#define AIC3206_HP_DRIVER_REG	0x7d
#define AIC3206_HPL_GAIN_REG	0x10
#define AIC3206_HPR_GAIN_REG	0x11
#define AIC3206_MICBIAS_REG	0x33
#define AIC3206_LMICPGA_PT_REG	0x34
#define AIC3206_LMICPGA_NT_REG	0x36
#define AIC3206_FLOAT_IN_REG	0x3a
#define AIC3206_LMICPGA_VOL_REG 0x3b
#define AIC3206_ADC_VOL_REG	0x3e
#define AIC3206_ADC_SETUP_REG	0x51
#define AIC3206_ADC_FGAIN_REG	0x52

/* AIC3206 register values. */
#define AIC3206_PAGE_0		0x0
#define AIC3206_PAGE_1		0x1
#define AIC3206_DEV_ID_VAL	0x11
#define AIC3206_OPWR_DOWN	0x0
#define AIC3206_OPWR_UP		0x30
#define AIC3206_SW_RESET	0x1
#define AIC3206_PLL_CLKIN	0x7
#define AIC3206_NDAC_1		0x81
#define AIC3206_NDAC_2		0x82
#define AIC3206_NDAC_7		0x87
#define AIC3206_AOSR_128	0x80
#define AIC3206_CLK3_J_7	0x7
#define AIC3206_CLK4_D_MSB_0	0x0
#define AIC3206_CLK5_D_LSB_0	0x0
#define AIC3206_CLK2_PLLPR_P1R4	0x94
#define AIC3206_DOSR_MSB_0	0x0
#define AIC3206_DOSR_LSB_128	0x80
#define AIC3206_MDAC_2		0x82
#define AIC3206_MDAC_7		0x87
#define AIC3206_AUDIF_24BIT	0x20
#define AIC3206_DAC_PRB_P4	0x8
#define AIC3206_CPC_CD6		0x6
#define AIC3206_NOAVDD_OSC	0xa
#define AIC3206_ANABLK_EN	0x0
#define AIC3206_REF_PWRUP_40MS	0x5
#define AIC3206_HPL_LDACP	0x8
#define AIC3206_HPR_RDACP	0x8
#define AIC3206_LOL_LDACP	0x8
#define AIC3206_LOR_RDACP	0x8
#define AIC3206_DAC_LL_RR_1S2C	0xd6
#define AIC3206_HP_GCM_DCOC	0x12
#define AIC3206_HP_GAIN_0DB	0x0
#define AIC3206_DAC_UNMUTE	0x0
#define AIC3206_DAC_MUTE	0xC
#define AIC3206_MICBIAS_UP	0x40
#define AIC3206_MICPGA_IN3L_10K	0x4
#define AIC3206_MICPGA_IN3R_10K	0x4
#define AIC3206_FLOAT_IN3L_CM	0x8
#define AIC3206_LMICPGA_0DB	0x80
#define AIC3206_ADC_LVOL_PG	0x2
#define AIC3206_ADC_L_UP	0x80
#define AIC3206_ADC_R_MUTED	0x8
#define AIC3206_DAC_VOL_N10DB	0xec	/* -10dB */

#endif // __AIC3206_H__
