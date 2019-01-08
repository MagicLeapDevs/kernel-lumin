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

#ifndef __FSA8500_H__
#define __FSA8500_H__

extern void aic3206_hp_insertion(int hp_inserted);

/* FSA8500 headset codecs private data. */
struct fsa8500_codec_data {
	unsigned int inserted;
	unsigned int redetect;
	struct regmap *regmap;
	struct delayed_work delayed_work;
	struct i2c_client *i2c;
	struct snd_soc_jack *hs_jack;
};

/* FSA8500 Dev ID */
#define FSA8500_DEV_ID_REG	0x1
#define FSA8500_DEV_ID_VAL	0x10

/* FSA8500 interrupt status registers. */
#define FSA8500_INT1_STATUS_REG	0x2
#define FSA8500_INT2_STATUS_REG	0x3
#define FSA8500_INT3_STATUS_REG	0x4
#define FSA8500_INT4_STATUS_REG	0x5
#define FSA8500_INT5_STATUS_REG	0x6
#define FSA8500_DET_JACK_REG	0xC
#define FSA8500_DET_LINT_REG	0xF
#define FSA8500_DET_ESD_REG	0x10
#define FSA8500_DET_MOI_REG	0x12
#define FSA8500_DET_TSHOD1_REG	0x13
#define FSA8500_DET_TSHOD2_REG	0x14
#define FSA8500_RESET_REG	0x15

/* FSA8500 jack detections. */
#define FSA8500_JACK_3_POLE	0x01
#define FSA8500_JACK_OMTP	0x02
#define FSA8500_JACK_CTIA	0x04
#define FSA8500_JACK_UART	0x08
#define FSA8500_JACK_DISCONNECT	0x10
#define FSA8500_JACK_LINT	0x20
#define FSA8500_JACK_MOISTURE	0x40
#define FSA8500_JACK_UNKNOWN	0x80
#define FSA8500_ACCESSORY0	0x00
#define FSA8500_ACCESSORY1	0x01
#define FSA8500_ACCESSORY2	0x02
#define FSA8500_ACCESSORY3	0x04
#define FSA8500_ACCESSORY_MASK	0x07
#define FSA8500_DET_JACK_400MS	0x62
#define FSA8500_DET_JACK_500MS	0x82
#define FSA8500_DET_JACK_850MS	0xF2
#define FSA8500_DET_ESD_5MS	0xF8
#define FSA8500_DET_MOI_ON	0xA0
#define FSA8500_JACK_DET_MS	250
#define FSA8500_JACK_DET_MAX	5


/* FSA8500 buttons. */
#define FSA8500_JACK_BUTTON_NONE 0x00
#define FSA8500_JACK_BUTTON_1	0x01
#define FSA8500_JACK_BUTTON_2	0x02
#define FSA8500_JACK_BUTTON_3	0x04
#define FSA8500_JACK_BUTTON_4	0x08
#define FSA8500_JACK_BUTTON_5	0x10
#define FSA8500_JACK_BUTTON_6	0x20
#define FSA8500_JACK_BUTTON_7	0x40
#define FSA8500_DET_TSHOD1_VAL	0xac
#define FSA8500_GLB_RESET	0x01
#define FSA8500_JACKDET_RESET	0x02

#define FSA8500_JACK_NOCHANGE	0
#define FSA8500_JACK_INSERTED	1
#define FSA8500_JACK_REMOVED	2
#define FSA8500_JACK_REDETECT	3

int fsa8500_set_hs_jack(struct snd_soc_jack *hs_jack);

#endif // __FSA8500_H__
