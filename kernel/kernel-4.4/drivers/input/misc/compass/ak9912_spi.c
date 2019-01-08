/*
 * ak9912_spi.c
 *
 * Magic Leap driver for ak9912 (AKM Magnetometer)
 *
 * Copyright (c) 2017, Magic Leap, Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>

/* AK9912 register map */
#define AK9912_WHO_AM_I			0x00
#define AK9912_STAT_1			0x10
#define AK9912_STAT_1_DRDY		(1<<0)
#define AK9912_STAT_1_DOR		(1<<1)
#define AK9912_STAT_1_HSM		(1<<7)
#define AK9912_DATA_XL			0x11
#define AK9912_DATA_XH			0x12
#define AK9912_DATA_YL			0x13
#define AK9912_DATA_YH			0x14
#define AK9912_DATA_ZL			0x15
#define AK9912_DATA_ZH			0x16
#define AK9912_TMPS			0x17
#define AK9912_STAT_2			0x18
#define AK9912_STAT_2_HOFL		(1<<4)
#define AK9912_CTRL_1			0x30
#define AK9912_CTRL_1_NSF_DISABLE	(0<<5)
#define AK9912_CTRL_1_NSF_LOW		(1<<5)
#define AK9912_CTRL_1_NSF_MIDDLE	(2<<5)
#define AK9912_CTRL_1_NSF_HIGH		(3<<5)
#define AK9912_CTRL_1_NSF_MASK		(3<<5)
#define AK9912_CTRL_2			0x31
#define AK9912_CTRL_3			0x32
#define AK9912_CTRL_3_SRST		0x01
#define AK9912_TS_1			0x33
#define AK9912_TS_2			0x34
#define AK9912_TS_3			0x35
#define AK9912_I2CDIS			0x36
#define AK9912_I2CDIS_DISABLE_I2C	0x1b
#define AK9912_ASAX			0x60
#define AK9912_ASAY			0x61
#define AK9912_ASAZ			0x62

/* available modes */
enum ak9912_modes {
	POWER_DOWN_MODE		= 0x00,
	SINGLE_MEASURE_MODE	= 0x01,
	CONT_MEASURE_MODE_1	= 0x02,	/* 10Hz */
	CONT_MEASURE_MODE_2	= 0x04,	/* 20Hz */
	CONT_MEASURE_MODE_3	= 0x06,	/* 50Hz */
	CONT_MEASURE_MODE_4	= 0x08,	/* 100Hz */
	EXT_TRIG_MEASURE_MODE	= 0x0A,
	SELF_TEST_MODE		= 0x10,
	FUSE_ROM_ACCESS_MODE	= 0x1F,
};

/* AK9912 available ODR in Hz */
#define AK9912_MODE_1_HZ	10	/* 10Hz */
#define AK9912_MODE_2_HZ	20	/* 20Hz */
#define AK9912_MODE_3_HZ	50	/* 50Hz */
#define AK9912_MODE_4_HZ	100	/* 100Hz */

/* AK9912 r/w bits */
#define AK9912_RD_REG		(1<<7)
#define AK9912_WR_REG		~(1<<7)

/* mode change wait time */
#define AK9912_TWAIT_MAX	200
#define AK9912_TWAIT_MIN	150
#define AK9912_T_MEASURE_MAX	8000
#define AK9912_T_MEASURE_MIN	7500

#define AK9912_TEMP_BIT		(1<<7)
#define AK9912_HEX_TO_DEG_C(x)	(35 + (((120 - x) * 10) / 16))

/* AK9912 misc macros */
#define AK9912_MAX_BUF_SIZE	32
#define AK9912_MAX_RD_TRY	10
#define AK9912_WHO_AM_I_VAL	0x48

/* self test limits */
#define AK9912_ST_HX_HI		200
#define AK9912_ST_HX_LO		-200
#define AK9912_ST_HY_HI		200
#define AK9912_ST_HY_LO		-200
#define AK9912_ST_HZ_HI		-400
#define AK9912_ST_HZ_LO		-1600

struct ak9912_data {
	int16_t hx;
	int16_t hy;
	int16_t hz;
	int8_t  temp;
	int64_t ts;
};

struct ak9912_dev {
	uint8_t asa[3];
	enum ak9912_modes cur_mode;
	enum ak9912_modes prev_mode;
	struct spi_device *spi;
	struct input_dev *in_dev;
	struct mutex mode_lock;
	struct ak9912_data data;
};

static int ak9912_wr_reg(struct ak9912_dev *mag,
			 uint8_t reg,
			 uint8_t data)
{
	struct spi_device *spi = mag->spi;
	uint8_t tbuf[AK9912_MAX_BUF_SIZE];
	int ret = 0;
	struct spi_transfer t = {
		.bits_per_word  = 8,
		.tx_buf = tbuf,
		.rx_buf = NULL,
		.len = 2,
		.speed_hz = 3000000,	/* MAX speed for AK is 3.33 MHz */
		.delay_usecs = 1,
	};

	tbuf[0] = reg & AK9912_WR_REG;
	tbuf[1] = data;
	ret = spi_sync_transfer(spi, &t, 1);

	if (ret)
		dev_err(&spi->dev, "%s: spi sync status: %d\n", __func__, ret);

	return ret;
}

static int ak9912_rd_reg(struct ak9912_dev *mag,
			 uint8_t reg,
			 uint8_t *rbuf,
			 uint8_t len)
{
	struct spi_device *spi = mag->spi;
	uint8_t tbuf[AK9912_MAX_BUF_SIZE];
	int ret = 0;
	uint8_t *buf = NULL;
	struct spi_transfer t = {
		.bits_per_word  = 8,
		.tx_buf = tbuf,
		.rx_buf = buf,
		.len = len+1,
		.speed_hz = 3000000,	/* MAX speed for AK9912 is 3.33 MHz */
		.delay_usecs = 1,
	};

	if (rbuf == NULL)
		return -EFAULT;

	t.rx_buf = buf = kzalloc(t.len, GFP_KERNEL);
	if (!buf) {
		dev_err(&spi->dev, "%s: failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	memset(tbuf, 0, sizeof(uint8_t) * t.len);
	tbuf[0] = reg | AK9912_RD_REG;
	ret = spi_sync_transfer(spi, &t, 1);

	if (!ret)
		memcpy(rbuf, &buf[1], len);
	else
		dev_err(&spi->dev, "%s: spi sync failed: %d\n", __func__, ret);

	kfree(buf);
	return ret;
}

static enum ak9912_modes ak9912_rd_cur_mode(struct ak9912_dev *mag)
{
	enum ak9912_modes mode;
	struct spi_device *spi = mag->spi;

	mutex_lock(&mag->mode_lock);
	mode = mag->cur_mode;
	dev_dbg(&spi->dev, "%s: cur_mode : 0x%x!\n", __func__, mode);
	mutex_unlock(&mag->mode_lock);

	return mode;
}

static int ak9912_rd_data(struct ak9912_dev *mag)
{
	int ret = 0;
	uint8_t rd_try = 0;
	struct spi_device *spi = mag->spi;
	struct ak9912_data *data = &mag->data;
	struct timespec ts;
	uint8_t rbuf[AK9912_MAX_BUF_SIZE];
	enum ak9912_modes cur_mode;

	while (rd_try < AK9912_MAX_RD_TRY) {
		ret = ak9912_rd_reg(mag, AK9912_STAT_1, rbuf, 1);
		if (ret) {
			dev_err(&spi->dev, "%s: STAT1 rd failed !\n", __func__);
			return ret;
		}
		if (rbuf[0] & AK9912_STAT_1_DRDY)
			break;
		rd_try++;
		usleep_range(AK9912_TWAIT_MIN, AK9912_TWAIT_MAX);
	}
	if (rd_try >= AK9912_MAX_RD_TRY) {
		dev_err(&spi->dev, "%s: STAT1 max rd try failed !\n", __func__);
		return -EIO;
	}

	ret = ak9912_rd_reg(mag, AK9912_DATA_XL, rbuf, 8);
	if (ret) {
		dev_err(&spi->dev, "%s: DATA reg read failed !\n", __func__);
		return ret;
	}

	if (rbuf[7] & AK9912_STAT_2_HOFL) {
		dev_err(&spi->dev, "%s: mag sensor overflow !\n", __func__);
		return -ERANGE;
	}
	ktime_get_ts(&ts);
	data->ts = timespec_to_ns(&ts);

	data->hx = le16_to_cpup((__le16 *)&rbuf[0]);
	data->hy = le16_to_cpup((__le16 *)&rbuf[2]);
	data->hz = le16_to_cpup((__le16 *)&rbuf[4]);

	/* converting TMPS count to deg C */
	data->temp = AK9912_HEX_TO_DEG_C((__s16)rbuf[6]);

	cur_mode = ak9912_rd_cur_mode(mag);
	if (cur_mode >= CONT_MEASURE_MODE_1 &&
	    cur_mode <= CONT_MEASURE_MODE_4) {
		input_event(mag->in_dev, EV_REL, REL_X, data->hx);
		input_event(mag->in_dev, EV_REL, REL_Y, data->hy);
		input_event(mag->in_dev, EV_REL, REL_Z, data->hz);
		input_event(mag->in_dev, EV_REL, REL_MISC, data->temp);
		input_event(mag->in_dev, EV_MSC, MSC_TIMESTAMP, data->ts);
		input_sync(mag->in_dev);
	}

	return ret;
}

static enum ak9912_modes ak9912_rate_to_mode(struct ak9912_dev *mag,
					     uint8_t data_rate)
{
	enum ak9912_modes ret;

	switch (data_rate) {

	case AK9912_MODE_1_HZ:
		ret = CONT_MEASURE_MODE_1;
		break;

	case AK9912_MODE_2_HZ:
		ret = CONT_MEASURE_MODE_2;
		break;

	case AK9912_MODE_3_HZ:
		ret = CONT_MEASURE_MODE_3;
		break;

	case AK9912_MODE_4_HZ:
		ret = CONT_MEASURE_MODE_4;
		break;

	default:
		ret = 0;
		break;
	}

	return ret;
}

static int ak9912_mode_to_rate(struct ak9912_dev *mag,
			      enum ak9912_modes mode)
{
	int ret;

	switch (mode) {

	case CONT_MEASURE_MODE_1:
		ret = AK9912_MODE_1_HZ;
		break;

	case CONT_MEASURE_MODE_2:
		ret = AK9912_MODE_2_HZ;
		break;

	case CONT_MEASURE_MODE_3:
		ret = AK9912_MODE_3_HZ;
		break;

	case CONT_MEASURE_MODE_4:
		ret = AK9912_MODE_4_HZ;
		break;

	default:
		ret = 0;
		break;
	}

	return ret;
}

static int ak9912_set_mode(struct ak9912_dev *mag,
			   enum ak9912_modes mode)
{
	int ret = 0;
	struct spi_device *spi = mag->spi;
	uint8_t prev_mode;

	prev_mode = ak9912_rd_cur_mode(mag);

	if (prev_mode == mode)
		return 0;

	if (prev_mode != POWER_DOWN_MODE && mode != POWER_DOWN_MODE) {
		ret = ak9912_wr_reg(mag, AK9912_CTRL_2, POWER_DOWN_MODE);
		if (ret) {
			dev_err(&spi->dev, "%s: failed to set PD!\n", __func__);
			goto exit;
		}
		usleep_range(AK9912_TWAIT_MIN, AK9912_TWAIT_MAX);
	}
	mutex_lock(&mag->mode_lock);
	ret = ak9912_wr_reg(mag, AK9912_CTRL_2, mode);
	if (ret) {
		dev_err(&spi->dev, "%s: CTRL_2 wr failed !\n", __func__);
		goto exit;
	}
	mag->cur_mode = mode;
	mutex_unlock(&mag->mode_lock);
	usleep_range(AK9912_T_MEASURE_MIN, AK9912_T_MEASURE_MAX);

	dev_dbg(&spi->dev, "%s: prev_mode: 0x%02x new mode: 0x%02x\n", __func__,
		prev_mode, mode);

exit:

	return ret;
}

static int ak9912_rd_fuse_rom(struct ak9912_dev *mag)
{
	int ret = 0;
	struct spi_device *spi = mag->spi;

	ret |= ak9912_set_mode(mag, FUSE_ROM_ACCESS_MODE);
	ret |= ak9912_rd_reg(mag, AK9912_ASAX, &mag->asa[0], 3);
	ret |= ak9912_set_mode(mag, POWER_DOWN_MODE);
	if (ret)
		dev_err(&spi->dev, "%s: failed !\n", __func__);

	return ret;
}

static int ak9912_set_nsf(struct ak9912_dev *mag,
			  uint8_t level)
{
	int ret = 0;
	struct spi_device *spi = mag->spi;
	uint8_t ctrl_1;

	/* AK9912 has to be in PD to change NSF settings */
	ret = ak9912_set_mode(mag, POWER_DOWN_MODE);
	if (ret) {
		dev_err(&spi->dev, "%s: could not set PD mode\n", __func__);
		goto exit;
	}
	ret = ak9912_rd_reg(mag, AK9912_CTRL_1, &ctrl_1, 1);
	if (ret) {
		dev_err(&spi->dev, "%s: could not read CTRL1 reg\n", __func__);
		goto exit;
	}
	ctrl_1 &= ~AK9912_CTRL_1_NSF_MASK;
	ctrl_1 |= (level & AK9912_CTRL_1_NSF_MASK);
	ret = ak9912_wr_reg(mag, AK9912_CTRL_1, ctrl_1);

exit:
	return ret;
}

static int ak9912_en_temp_measure(struct ak9912_dev *mag, uint8_t en)
{
	int ret = 0;
	struct spi_device *spi = mag->spi;
	uint8_t ctrl_1;

	ret = ak9912_set_mode(mag, POWER_DOWN_MODE);
	if (ret) {
		dev_err(&spi->dev, "%s: could not set PD mode\n", __func__);
		goto exit;
	}
	ret = ak9912_rd_reg(mag, AK9912_CTRL_1, &ctrl_1, 1);
	if (ret) {
		dev_err(&spi->dev, "%s: could not read CTRL1 reg\n", __func__);
		goto exit;
	}
	ctrl_1 &= ~AK9912_TEMP_BIT;
	ctrl_1 |= (!!en ? AK9912_TEMP_BIT : 0);
	ret = ak9912_wr_reg(mag, AK9912_CTRL_1, ctrl_1);

exit:
	return ret;
}

static int ak9912_init(struct ak9912_dev *mag)
{
	int ret = 0;
	struct spi_device *spi = mag->spi;

	ret = ak9912_wr_reg(mag, AK9912_CTRL_3, AK9912_CTRL_3_SRST);
	if (ret) {
		dev_err(&spi->dev, "%s: failed to reset !\n", __func__);
		return -EIO;
	}
	ret = ak9912_wr_reg(mag, AK9912_I2CDIS, AK9912_I2CDIS_DISABLE_I2C);
	if (ret) {
		dev_err(&spi->dev, "%s: failed to disable i2c IF.\n", __func__);
		return -EIO;
	}
	ret = ak9912_rd_fuse_rom(mag);
	if (ret) {
		dev_err(&spi->dev, "%s: failed to read fuse rom !\n", __func__);
		return -EIO;
	}
	ret = ak9912_set_nsf(mag, AK9912_CTRL_1_NSF_MIDDLE);
	if (ret) {
		dev_err(&spi->dev, "%s: failed to set nsf !\n", __func__);
		return -EIO;
	}
	ret = ak9912_en_temp_measure(mag, 1);
	if (ret) {
		dev_err(&spi->dev, "%s: failed to en temp !\n", __func__);
		return -EIO;
	}
	mag->prev_mode = CONT_MEASURE_MODE_1;

	return ret;
}

static int ak9912_rd_one_sample(struct ak9912_dev *mag,
				uint8_t *rbuf)
{
	uint8_t ret = 0;
	uint8_t prev_mode = ak9912_rd_cur_mode(mag);

	ret |= ak9912_set_mode(mag, SINGLE_MEASURE_MODE);
	ret |= ak9912_rd_data(mag);
	ret |= ak9912_set_mode(mag, prev_mode);

	return ret;
}

static int ak9912_self_test(struct ak9912_dev *mag,
			    struct ak9912_data *st_data)
{
	int ret = 0;
	struct spi_device *spi = mag->spi;
	struct ak9912_data *data = &mag->data;
	uint8_t prev_mode = ak9912_rd_cur_mode(mag);

	if (prev_mode != POWER_DOWN_MODE)
		return -EACCES;

	ret = ak9912_set_mode(mag, SELF_TEST_MODE);
	if (ret) {
		dev_err(&spi->dev, "%s: failed to set mode !\n", __func__);
		return ret;
	}

	ret = ak9912_rd_data(mag);
	if (ret)
		goto exit;

	if (data->hx > AK9912_ST_HX_HI || data->hx < AK9912_ST_HX_LO) {
		dev_err(&spi->dev, "%s: hx: 0x%x out of range !\n",
			__func__, data->hx);
		return -ERANGE;
	}
	if (data->hy > AK9912_ST_HY_HI || data->hy < AK9912_ST_HY_LO) {
		dev_err(&spi->dev, "%s: hy: 0x%x out of range !\n",
			__func__, data->hy);
		return -ERANGE;
	}
	if (data->hz > AK9912_ST_HZ_HI || data->hz < AK9912_ST_HZ_LO) {
		dev_err(&spi->dev, "%s: hz: 0x%x out of range !\n",
			__func__, data->hz);
		return -ERANGE;
	}

	st_data->hx = data->hx;
	st_data->hy = data->hy;
	st_data->hz = data->hz;
exit:
	ret |= ak9912_set_mode(mag, prev_mode);
	if (ret)
		ret = -EINVAL;

	return ret;
}

static irqreturn_t ak9912_irq_threaded(int irq, void *data)
{
	struct ak9912_dev *mag = (struct ak9912_dev *)data;

	ak9912_rd_data(mag);

	return IRQ_HANDLED;
}

static int ak9912_in_dev_register(struct ak9912_dev *mag)
{
	struct spi_device *spi = mag->spi;
	struct device *dev = &spi->dev;
	int ret = 0;

	mag->in_dev = devm_input_allocate_device(dev);
	if (!mag->in_dev)
		return -ENOMEM;

	mag->in_dev->name = "ak9912_input_dev";
	mag->in_dev->id.bustype = BUS_SPI;
	mag->in_dev->dev.parent = dev;
	input_set_capability(mag->in_dev, EV_REL, REL_X);
	input_set_capability(mag->in_dev, EV_REL, REL_Y);
	input_set_capability(mag->in_dev, EV_REL, REL_Z);
	input_set_capability(mag->in_dev, EV_REL, REL_MISC);
	input_set_capability(mag->in_dev, EV_MSC, MSC_TIMESTAMP);

	ret = input_register_device(mag->in_dev);

	return ret;
}

static ssize_t enable_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf,
			    size_t count)
{
	struct ak9912_dev *mag = dev->platform_data;
	struct spi_device *spi = mag->spi;

	if (buf[0] == '0') {
		mag->prev_mode = ak9912_rd_cur_mode(mag);
		if (mag->prev_mode != POWER_DOWN_MODE) {
			ak9912_set_mode(mag, POWER_DOWN_MODE);
			disable_irq(spi->irq);
			dev_dbg(dev, "%s: disabling irq\n", __func__);
		}
	} else {
		if (ak9912_rd_cur_mode(mag) == POWER_DOWN_MODE) {
			enable_irq(spi->irq);
			ak9912_set_mode(mag, mag->prev_mode);
			dev_dbg(dev, "%s: enabling irq\n", __func__);
		}
	}

	return count;
}

static ssize_t enable_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct ak9912_dev *mag = dev->platform_data;
	enum ak9912_modes mode;

	mode = ak9912_rd_cur_mode(mag);

	return scnprintf(buf, PAGE_SIZE, "%02x\n", !!mode);
}

static DEVICE_ATTR_RW(enable);

static ssize_t whoami_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	uint8_t rbuf[AK9912_MAX_BUF_SIZE];
	struct ak9912_dev *mag = dev->platform_data;
	int ret;

	ret = ak9912_rd_reg(mag, AK9912_WHO_AM_I, rbuf, 2);
	if (ret) {
		dev_err(dev, "%s: can not read WHO_AM_I reg !", __func__);
		return ret;
	}
	return scnprintf(buf, PAGE_SIZE, "0x%02x%02x\n", rbuf[0], rbuf[1]);
}

static DEVICE_ATTR_RO(whoami);

static ssize_t self_test_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct ak9912_dev *mag = dev->platform_data;
	struct ak9912_data st_data;
	int ret;

	ret = ak9912_self_test(mag, &st_data);
	if (ret == -EACCES) {
		dev_err(dev, "%s: self test not available !\n", __func__);
		return ret;
	}
	if (ret) {
		dev_err(dev, "%s: self test failed !\n", __func__);
		return ret;
	}

	return scnprintf(buf, PAGE_SIZE, "hx:%d\nhy:%d\nhz:%d\nST passed !\n",
			st_data.hx, st_data.hy, st_data.hz);
}

static DEVICE_ATTR_RO(self_test);

static ssize_t data_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	uint8_t rbuf[AK9912_MAX_BUF_SIZE];
	struct ak9912_dev *mag = dev->platform_data;
	struct ak9912_data *data = &mag->data;
	int ret;

	if (ak9912_rd_cur_mode(mag) == POWER_DOWN_MODE) {
		ret = ak9912_rd_one_sample(mag, rbuf);
		if (ret) {
			dev_err(dev, "%s: reading data failed !\n", __func__);
			return ret;
		}
	}

	return scnprintf(buf, PAGE_SIZE, "hx:%d\nhy:%d\nhz:%d\ntemp:%d degC\n",
			data->hx, data->hy, data->hz, data->temp);
}

static DEVICE_ATTR_RO(data);

static ssize_t rate_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	uint8_t ret, data_rate;
	struct ak9912_dev *mag = dev->platform_data;
	enum ak9912_modes mode;

	ret = kstrtou8(buf, 10, &data_rate);
	if (ret) {
		dev_err(dev, "%s: failed to convert rate value", __func__);
		goto exit;
	}

	if (data_rate == ak9912_mode_to_rate(mag, ak9912_rd_cur_mode(mag)))
		goto exit;

	mode = ak9912_rate_to_mode(mag, data_rate);
	if (mode) {
		ret = ak9912_set_mode(mag, mode);
		if (ret) {
			dev_err(dev, "unable to set rate : %dHz\n", data_rate);
			goto exit;
		}
	} else {
			dev_err(dev, "Invalid rate, valid rates (10, 20, 50, 100 Hz)\n");
			goto exit;
	}

exit:
	return count;
}

static ssize_t rate_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct ak9912_dev *mag = dev->platform_data;

	return scnprintf(buf, PAGE_SIZE, "rate: %dHz\n",
			 ak9912_mode_to_rate(mag, ak9912_rd_cur_mode(mag)));
}

static DEVICE_ATTR_RW(rate);

static struct attribute *ak9912_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_self_test.attr,
	&dev_attr_whoami.attr,
	&dev_attr_data.attr,
	&dev_attr_rate.attr,
	NULL
};

static const struct attribute_group ak9912_attr_group = {
	.attrs = ak9912_attributes,
};

static int ak9912_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ak9912_dev *mag;
	int ret = 0;
	uint8_t whoami = 0;

	dev_info(dev, "%s: enter\n", __func__);

	if (spi->irq <= 0) {
		dev_err(dev, "%s: no irq defined\n", __func__);
		return -ENODEV;
	}

	mag = devm_kzalloc(dev, sizeof(struct ak9912_dev), GFP_KERNEL);
	if (!mag)
		return -ENOMEM;
	mag->spi = spi;
	dev->platform_data = mag;
	spi_set_drvdata(spi, mag);
	mutex_init(&mag->mode_lock);

	/* Its better to reset sensor and state of all pins */
	ret |= ak9912_init(mag);
	ret |= ak9912_rd_reg(mag, AK9912_WHO_AM_I, &whoami, 1);
	ret |= ak9912_set_mode(mag, POWER_DOWN_MODE);
	if (ret) {
		dev_err(dev, "%s: failed to communicate with chip !", __func__);
		goto free_mutex;
	}

	if (whoami != AK9912_WHO_AM_I_VAL) {
		dev_err(dev, "%s: unexpected WHO_AM_I : %d value !",
			__func__, whoami);
		ret = -EPROBE_DEFER;
		goto free_mutex;
	}

	ret = devm_request_threaded_irq(dev, spi->irq,
					NULL,
					ak9912_irq_threaded,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"ak9912_irq", mag);
	if (ret) {
		dev_err(dev, "%s: failed to request irq !\n", __func__);
		goto free_mutex;
	}
	disable_irq(spi->irq);

	ret = ak9912_in_dev_register(mag);
	if (ret) {
		dev_err(dev, "%s: failed to register input device\n", __func__);
		goto free_irq;
	}

	ret = sysfs_create_group(&spi->dev.kobj, &ak9912_attr_group);
	if (ret < 0) {
		dev_err(dev, "%s: failed to create sysfs node %d\n",
			__func__, ret);
		goto dereg_in_dev;
	}

	dev_info(dev, "%s: Success !\n", __func__);
	return ret;

dereg_in_dev:
	mag->in_dev = NULL;
free_irq:
	devm_free_irq(dev, spi->irq, mag);
free_mutex:
	mutex_destroy(&mag->mode_lock);
	spi_set_drvdata(spi, NULL);

	return ret;
}

static int ak9912_remove(struct spi_device *spi)
{
	int ret = 0;
	struct ak9912_dev *mag = spi_get_drvdata(spi);

	dev_info(&spi->dev, "%s: enter\n", __func__);
	sysfs_remove_group(&spi->dev.kobj, &ak9912_attr_group);
	mag->in_dev = NULL;
	devm_free_irq(&spi->dev, spi->irq, mag);
	mutex_destroy(&mag->mode_lock);
	spi_set_drvdata(spi, NULL);

	return ret;
}

static const struct of_device_id ak9912_match[] = {
	{ .compatible = "ml,ak9912" },
	{ },
};
MODULE_DEVICE_TABLE(of, ak9912_match);

static struct spi_driver ak9912_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ak9912",
		.of_match_table = ak9912_match,
	},
	.probe = ak9912_probe,
	.remove = ak9912_remove,
};
module_spi_driver(ak9912_spi_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("Magic Leap AK9912 Beltpack Driver");
MODULE_LICENSE("GPL v2");
