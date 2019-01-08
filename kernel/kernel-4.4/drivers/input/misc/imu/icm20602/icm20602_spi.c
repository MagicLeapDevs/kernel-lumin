/*
 * icm20602_spi.c
 *
 * Magic Leap driver for ICM20602 (Invensense's IMU in beltpack)
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
#include <linux/of.h>

/* ICM20602 register map */
#define ICM20602_SELF_TEST_X_ACCEL	0x0D
#define ICM20602_SELF_TEST_Y_ACCEL	0x0E
#define ICM20602_SELF_TEST_Z_ACCEL	0x0F

#define ICM20602_SMPLRT_DIV		0x19
#define ICM20602_CONFIG			0x1A
#define ICM20602_GYR_CONF		0x1B
#define ICM20602_ACC_CONF		0x1C
#define ICM20602_ACC_CONF_2		0x1D
#define ICM20602_GYR_LPM_CONF		0x1E
#define ICM20602_INT_PIN_CFG		0x37
#define ICM20602_INT_EN			0x38
#define ICM20602_INT_STAT		0x3A
#define ICM20602_INT_STAT_DATA_RDY_INT	(1<<0)
#define ICM20602_INT_STAT_GDRIVE_INT	(1<<2)
#define ICM20602_INT_STAT_FIFO_OF_INT	(1<<4)
#define ICM20602_INT_STAT_WOM_Z_INT	(1<<5)
#define ICM20602_INT_STAT_WOM_Y_INT	(1<<6)
#define ICM20602_INT_STAT_WOM_X_INT	(1<<7)
#define ICM20602_ACC_X_H		0x3B
#define ICM20602_ACC_X_L		0x3C
#define ICM20602_ACC_Y_H		0x3D
#define ICM20602_ACC_Y_L		0x3E
#define ICM20602_ACC_Z_H		0x3F
#define ICM20602_ACC_Z_L		0x40
#define ICM20602_TEMP_H			0x41
#define ICM20602_TEMP_L			0x42
#define ICM20602_GYR_X_H		0x43
#define ICM20602_GYR_X_L		0x44
#define ICM20602_GYR_Y_H		0x45
#define ICM20602_GYR_Y_L		0x46
#define ICM20602_GYR_Z_H		0x47
#define ICM20602_GYR_Z_L		0x48

#define ICM20602_SELF_TEST_X_GYRO	0x50
#define ICM20602_SELF_TEST_Y_GYRO	0x51
#define ICM20602_SELF_TEST_Z_GYRO	0x52

#define ICM20602_SIG_PATH_RST		0x68
#define ICM20602_SIG_PATH_RST_TEMP_RST	(1<<0)
#define ICM20602_SIG_PATH_RST_ACCEL_RST	(1<<1)
#define ICM20602_PWR_MGM_1		0x6B
#define ICM20602_PWR_MGM_1_DEVICE_RESET	(1<<7)
#define ICM20602_PWR_MGM_1_SLEEP	(1<<6)
#define ICM20602_PWR_MGM_1_CYCLE	(1<<5)
#define ICM20602_PWR_MGM_1_GYRO_STANDBY	(1<<4)
#define ICM20602_PWR_MGM_1_TEMP_DIS	(1<<3)
#define ICM20602_PWR_MGM_1_CLKSEL	(7<<0)
#define ICM20602_PWR_MGM_2		0x6C
#define ICM20602_PWR_MGM_2_GYR_STANDBY	(7<<0)
#define ICM20602_PWR_MGM_2_ACC_STANDBY	(7<<3)
#define ICM20602_I2C_IF			0x70
#define ICM20602_I2C_IF_DIS		(1<<6)
#define ICM20602_WHO_AM_I		0x75
#define ICM20602_WHO_AM_I_VAL		0x12

/* R/W bits */
#define ICM20602_RD_REG			(1<<7)
#define ICM20602_WR_REG			~(1<<7)

/* Misc */
#define ICM20602_MAX_BUF_SIZE		32
#define ICM20602_MAX_RD_TRY		10
#define ICM20602_IN_DEV_NAME_LEN	32

/* Default configs */
#define ICM20602_MAX_RATE		1000	/* Hz */
#define ICM20602_MIN_RATE		4	/* Hz */
#define ICM20602_DFLT_RATE		10	/* Hz */
#define ICM20602_DFLT_ACC_FSR		8	/* +-g */
#define ICM20602_DFLT_ACC_DLPF		3
#define ICM20602_DFLT_ACC_AVG		0	/* 4 samples */
#define ICM20602_DFLT_GYR_FSR		500	/* dps */
#define ICM20602_DFLT_GYR_DLPF		3
#define ICM20602_DFLT_GYR_AVG		0	/* 4 samples */

/* Temperature */
#define TEMP_SENS			3268
#define ROOM_TEMP_OFFSET		25

#define ICM20602_STARTUP_MIN		(2000)	/* 2ms */
#define ICM20602_STARTUP_MAX		(2500)	/* 2.5ms */

/* masks */
#define ICM20602_ACC_DLPF_MASK		(7<<0)
#define ICM20602_GYR_DLPF_MASK		(7<<0)
#define ICM20602_FS_SEL_MASK		(3<<3)
#define ICM20602_ACC_AVG_MASK		(3<<4)
#define ICM20602_GYR_AVG_MASK		(7<<4)

#define ICM20602_ACC_FSR_2G		2
#define ICM20602_ACC_FSR_4G		4
#define ICM20602_ACC_FSR_8G		8
#define ICM20602_ACC_FSR_16G		16

#define ICM20602_GYR_FSR_250_DPS	250
#define ICM20602_GYR_FSR_500_DPS	500
#define ICM20602_GYR_FSR_1000_DPS	1000
#define ICM20602_GYR_FSR_2000_DPS	2000

#define ICM20602_ACC_AVG_4_SAMPLES	4
#define ICM20602_ACC_AVG_8_SAMPLES	8
#define ICM20602_ACC_AVG_16_SAMPLES	16
#define ICM20602_ACC_AVG_32_SAMPLES	32

#define ICM20602_GYR_AVG_1_SAMPLES	1
#define ICM20602_GYR_AVG_2_SAMPLES	2
#define ICM20602_GYR_AVG_4_SAMPLES	4
#define ICM20602_GYR_AVG_8_SAMPLES	8
#define ICM20602_GYR_AVG_16_SAMPLES	16
#define ICM20602_GYR_AVG_32_SAMPLES	32
#define ICM20602_GYR_AVG_64_SAMPLES	64
#define ICM20602_GYR_AVG_128_SAMPLES	128

/* self test macros */
#define ICM20602_ACC_GYR_ST_MASK	0xE0

enum icm_state {
	DISABLED,
	ENABLED,
};

enum sensor_state {
	ICM20602_SENS_STATE_ACC_EN = 1,
	ICM20602_SENS_STATE_GYR_EN = 2,
};

/* Acc full scale select */
enum icm_acc_fs_sel {
	ICM20602_ACC_FS_SEL_2G,
	ICM20602_ACC_FS_SEL_4G,
	ICM20602_ACC_FS_SEL_8G,
	ICM20602_ACC_FS_SEL_16G,
};

/* Gyr full scale select */
enum icm_gyr_fs_sel {
	ICM20602_GYR_FS_SEL_250_DPS,
	ICM20602_GYR_FS_SEL_500_DPS,
	ICM20602_GYR_FS_SEL_1000_DPS,
	ICM20602_GYR_FS_SEL_2000_DPS,
};

/* Acc avg select */
enum icm_acc_avg_cfg {
	ICM20602_ACC_AVG_4_SAMPLES_CFG,
	ICM20602_ACC_AVG_8_SAMPLES_CFG,
	ICM20602_ACC_AVG_16_SAMPLES_CFG,
	ICM20602_ACC_AVG_32_SAMPLES_CFG,
};

/* Acc avg select */
enum icm_gyr_avg_cfg {
	ICM20602_GYR_AVG_1_SAMPLES_CFG,
	ICM20602_GYR_AVG_2_SAMPLES_CFG,
	ICM20602_GYR_AVG_4_SAMPLES_CFG,
	ICM20602_GYR_AVG_8_SAMPLES_CFG,
	ICM20602_GYR_AVG_16_SAMPLES_CFG,
	ICM20602_GYR_AVG_32_SAMPLES_CFG,
	ICM20602_GYR_AVG_64_SAMPLES_CFG,
	ICM20602_GYR_AVG_128_SAMPLES_CFG,
};

/* helper modes */
enum icm_modes {
	ICM20602_MODE_DISABLE_ACC,
	ICM20602_MODE_ENABLE_ACC,
	ICM20602_MODE_DISABLE_GYR,
	ICM20602_MODE_ENABLE_GYR,
};

struct icm20602_sample {
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t temp;
	uint64_t ts;
};

struct icm20602_config {
	/* control data reporting to input subsys */
	uint8_t en;
	/* full scale range */
	uint16_t fsr;
	/* low pass filter */
	uint8_t dlpf;
	/* averaging filter settings */
	uint8_t avg;
};

struct icm20602_drv_data {
	uint8_t st_en;
	uint16_t rate;
	struct icm20602_sample acc_data;
	struct icm20602_sample gyr_data;
	struct icm20602_config acc_conf;
	struct icm20602_config gyr_conf;
	struct spi_device *spi;
	struct device *dev;
	struct input_dev *in_dev_acc;
	struct input_dev *in_dev_gyr;
	struct mutex mode_lock;
};

static int icm20602_wr_reg(struct  icm20602_drv_data *icm,
			   uint8_t reg,
			   uint8_t data)
{
	struct spi_device *spi = icm->spi;
	struct device *dev = icm->dev;
	uint8_t tbuf[ICM20602_MAX_BUF_SIZE];
	int ret;
	struct spi_transfer t = {
		.bits_per_word  = 8,
		.tx_buf = tbuf,
		.rx_buf = NULL,
		.len = 2,
		.speed_hz = 5000000,	/* MAX speed for ICM is 10MHz */
		.delay_usecs = 100,
	};

	memset(tbuf, 0, t.len);
	tbuf[0] = reg & ICM20602_WR_REG;
	tbuf[1] = data;

	ret = spi_sync_transfer(spi, &t, 1);

	if (ret)
		dev_err(dev, "%s: spi sync failed: %d\n", __func__, ret);

	return ret;
}

static int icm20602_rd_reg(struct  icm20602_drv_data *icm,
			   uint8_t reg,
			   uint8_t *rbuf,
			   uint8_t len)
{
	struct spi_device *spi = icm->spi;
	struct device *dev = icm->dev;
	uint8_t tbuf[ICM20602_MAX_BUF_SIZE];
	int ret = 0;
	uint8_t *buf = NULL;
	struct spi_transfer t = {
		.bits_per_word  = 8,
		.tx_buf = tbuf,
		.rx_buf = buf,
		.len = len+1,
		.speed_hz = 5000000,	/* MAX speed for ICM is 10MHz */
		.delay_usecs = 100,
	};

	if (unlikely(len > ICM20602_MAX_BUF_SIZE))
		return -ENOMEM;

	t.rx_buf = buf = kzalloc(t.len, GFP_KERNEL);
	if (!buf) {
		dev_err(dev, "%s: failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	memset(tbuf, 0, t.len);
	tbuf[0] = reg | ICM20602_RD_REG;

	ret = spi_sync_transfer(spi, &t, 1);

	if (!ret)
		memcpy(rbuf, &buf[1], len);
	else
		dev_err(dev, "%s: spi sync failed: %d\n", __func__, ret);

	kfree(buf);
	return ret;
}

static int icm20602_set_mode(struct icm20602_drv_data *icm,
			     enum icm_modes mode)
{
	uint8_t ret;
	uint8_t pwr_mgm_1, pwr_mgm_2;
	uint8_t sensor_state = 0;
	struct device *dev = icm->dev;
	struct icm20602_config *acc_conf = &icm->acc_conf;
	struct icm20602_config *gyr_conf = &icm->gyr_conf;

	if ((acc_conf->en != 0 && mode == ICM20602_MODE_ENABLE_ACC) ||
	    (acc_conf->en == 0 && mode == ICM20602_MODE_DISABLE_ACC))
		return 0;

	if ((gyr_conf->en != 0 && mode == ICM20602_MODE_ENABLE_GYR) ||
	    (gyr_conf->en == 0 && mode == ICM20602_MODE_DISABLE_GYR))
		return 0;

	mutex_lock(&icm->mode_lock);
	ret = icm20602_rd_reg(icm, ICM20602_PWR_MGM_1, &pwr_mgm_1, 1);
	ret |= icm20602_rd_reg(icm, ICM20602_PWR_MGM_2, &pwr_mgm_2, 1);
	if (ret) {
		dev_err(dev, "%s: PWR_MGMT_1/2 read failed !", __func__);
		goto exit;
	}

	sensor_state = (gyr_conf->en << 1) | acc_conf->en;
	switch (mode) {

	case ICM20602_MODE_DISABLE_ACC:
		pwr_mgm_2 |= ICM20602_PWR_MGM_2_ACC_STANDBY;
		sensor_state &= ~ICM20602_SENS_STATE_ACC_EN;
		break;

	case ICM20602_MODE_ENABLE_ACC:
		pwr_mgm_2 &= ~ICM20602_PWR_MGM_2_ACC_STANDBY;
		sensor_state |= ICM20602_SENS_STATE_ACC_EN;
		break;

	case ICM20602_MODE_DISABLE_GYR:
		pwr_mgm_1 |= ICM20602_PWR_MGM_1_GYRO_STANDBY;
		sensor_state &= ~ICM20602_SENS_STATE_GYR_EN;
		break;

	case ICM20602_MODE_ENABLE_GYR:
		pwr_mgm_1 &= ~ICM20602_PWR_MGM_1_GYRO_STANDBY;
		sensor_state |= ICM20602_SENS_STATE_GYR_EN;
		break;

	default:
		dev_err(dev, "%s: wrong mode: %d", __func__, mode);
		ret = -ERANGE;
		goto exit;
	}

	if (!sensor_state) {
		pwr_mgm_1 |= ICM20602_PWR_MGM_1_SLEEP;
		dev_dbg(dev, "%s: going to sleep...\n", __func__);
	} else {
		pwr_mgm_1 &= ~ICM20602_PWR_MGM_1_SLEEP;
		dev_dbg(dev, "%s: not sleeping...\n", __func__);
	}

	ret = icm20602_wr_reg(icm, ICM20602_PWR_MGM_1, pwr_mgm_1);
	ret |= icm20602_wr_reg(icm, ICM20602_PWR_MGM_2, pwr_mgm_2);
	if (ret) {
		dev_err(dev, "%s: PWR_MGMT_1/2 wr failed !\n", __func__);
		goto exit;
	}

	gyr_conf->en = !!(sensor_state & ICM20602_SENS_STATE_GYR_EN);
	acc_conf->en = !!(sensor_state & ICM20602_SENS_STATE_ACC_EN);

exit:
	mutex_unlock(&icm->mode_lock);
	return ret;
}

static int icm20602_set_rate(struct icm20602_drv_data *icm,
			    uint16_t rate)
{
	int ret;
	uint8_t smplrt_div;
	struct device *dev = icm->dev;

	if ((rate > ICM20602_MAX_RATE) || (ICM20602_MAX_RATE % rate) ||
	    (rate < ICM20602_MIN_RATE))
		return -ERANGE;

	smplrt_div = (ICM20602_MAX_RATE / rate - 1);

	ret = icm20602_wr_reg(icm, ICM20602_SMPLRT_DIV, smplrt_div);
	if (ret) {
		dev_err(dev, "%s: failed to set rate !\n", __func__);
		return ret;
	}
	icm->rate = rate;

	dev_dbg(dev, "%s: rate set to '%dHz'  SAMPLRT_DIV: 0x%x\n",
		__func__, icm->rate, smplrt_div);

	return ret;
}

static int icm20602_set_acc_fsr(struct icm20602_drv_data *icm,
				uint8_t fsr)
{
	int ret;
	uint8_t acc_fs_sel, acc_config;
	struct device *dev = icm->dev;

	switch (fsr) {

	case ICM20602_ACC_FSR_2G:
		acc_fs_sel = ICM20602_ACC_FS_SEL_2G;
		break;

	case ICM20602_ACC_FSR_4G:
		acc_fs_sel = ICM20602_ACC_FS_SEL_4G;
		break;

	case ICM20602_ACC_FSR_8G:
		acc_fs_sel = ICM20602_ACC_FS_SEL_8G;
		break;

	case ICM20602_ACC_FSR_16G:
		acc_fs_sel = ICM20602_ACC_FS_SEL_16G;
		break;

	default:
		dev_err(dev, "%s: acc FSR value out of range !\n", __func__);
		return -ERANGE;
	}

	ret = icm20602_rd_reg(icm, ICM20602_ACC_CONF, &acc_config, 1);
	if (ret) {
		dev_err(dev, "%s: ICM20602_ACC_CONF read failed !", __func__);
		goto exit;
	}

	acc_config &= ~ICM20602_FS_SEL_MASK;
	acc_config |= acc_fs_sel << 3;

	ret = icm20602_wr_reg(icm, ICM20602_ACC_CONF, acc_config);
	if (ret) {
		dev_err(dev, "%s: failed to set fsr !\n", __func__);
		goto exit;
	}

	dev_dbg(dev, "%s:  Acc FSR set to '+-%dg'  ACC_CONF: 0x%x\n",
		__func__, fsr, acc_config);

exit:
	return ret;
}

static int icm20602_set_gyr_fsr(struct icm20602_drv_data *icm,
				uint16_t fsr)
{
	int ret;
	uint8_t gyr_fs_sel, gyr_config;
	struct device *dev = icm->dev;

	switch (fsr) {

	case ICM20602_GYR_FSR_250_DPS:
		gyr_fs_sel = ICM20602_GYR_FS_SEL_250_DPS;
		break;

	case ICM20602_GYR_FSR_500_DPS:
		gyr_fs_sel = ICM20602_GYR_FS_SEL_500_DPS;
		break;

	case ICM20602_GYR_FSR_1000_DPS:
		gyr_fs_sel = ICM20602_GYR_FS_SEL_1000_DPS;
		break;

	case ICM20602_GYR_FSR_2000_DPS:
		gyr_fs_sel = ICM20602_GYR_FS_SEL_2000_DPS;
		break;

	default:
		dev_err(dev, "%s: gyr FSR value out of range !\n", __func__);
		return -ERANGE;
	}

	ret = icm20602_rd_reg(icm, ICM20602_GYR_CONF, &gyr_config, 1);
	if (ret) {
		dev_err(dev, "%s: ICM20602_GYR_CONF read failed !", __func__);
		goto exit;
	}

	gyr_config &= ~ICM20602_FS_SEL_MASK;
	gyr_config |= gyr_fs_sel << 3;

	ret = icm20602_wr_reg(icm, ICM20602_GYR_CONF, gyr_config);
	if (ret) {
		dev_err(dev, "%s: failed to set fsr !\n", __func__);
		goto exit;
	}

	dev_dbg(dev, "%s:  Gyr FSR set to '%d dps'   GYR_CONF: 0x%x\n",
		__func__, fsr, gyr_config);

exit:
	return ret;
}

static int icm20602_set_acc_dlpf(struct icm20602_drv_data *icm,
				 uint8_t dlpf)
{
	uint8_t ret;
	uint8_t config;
	struct device *dev = icm->dev;

	if (dlpf > 7) {
		dev_err(dev, "%s: acc 'DLPF' value out of range !\n", __func__);
		return -ERANGE;
	}

	ret = icm20602_rd_reg(icm, ICM20602_ACC_CONF_2, &config, 1);
	if (ret) {
		dev_err(dev, "%s: failed to read config reg\n", __func__);
		return ret;
	}

	config &= ~(ICM20602_ACC_DLPF_MASK);
	config |= dlpf;

	ret = icm20602_wr_reg(icm, ICM20602_ACC_CONF_2, config);

	dev_dbg(dev, "%s: Acc DLPF set to '%d'   ACC_CONFIG_2 : 0x%x\n",
		__func__, dlpf, config);

	return ret;
}

static int icm20602_set_gyr_dlpf(struct icm20602_drv_data *icm, uint8_t dlpf)
{
	uint8_t ret;
	uint8_t config;
	struct device *dev = icm->dev;

	if (dlpf > 7) {
		dev_err(dev, "%s: gyr 'DLPF' value out of range !\n", __func__);
		return -ERANGE;
	}

	ret = icm20602_rd_reg(icm, ICM20602_CONFIG, &config, 1);
	if (ret) {
		dev_err(dev, "%s: failed to read config reg\n", __func__);
		return ret;
	}

	config &= ~(ICM20602_GYR_DLPF_MASK);
	config |= dlpf;

	ret = icm20602_wr_reg(icm, ICM20602_CONFIG, config);

	dev_dbg(dev, "%s: Gyr DLPF set to '%d'   CONFIG : 0x%x\n",
		__func__, dlpf, config);

	return ret;
}

static int icm20602_set_acc_avg(struct icm20602_drv_data *icm, uint8_t avg)
{
	uint8_t ret;
	uint8_t acc_config_2;
	uint8_t dec2_cfg;
	struct device *dev = icm->dev;

	switch (avg) {
	case ICM20602_ACC_AVG_4_SAMPLES:
		dec2_cfg = ICM20602_ACC_AVG_4_SAMPLES_CFG;
		break;
	case ICM20602_ACC_AVG_8_SAMPLES:
		dec2_cfg = ICM20602_ACC_AVG_8_SAMPLES_CFG;
		break;
	case ICM20602_ACC_AVG_16_SAMPLES:
		dec2_cfg = ICM20602_ACC_AVG_16_SAMPLES_CFG;
		break;
	case ICM20602_ACC_AVG_32_SAMPLES:
		dec2_cfg = ICM20602_ACC_AVG_32_SAMPLES_CFG;
		break;
	default:
		dev_err(dev, "%s: acc 'AVG' value out of range !\n", __func__);
		return -ERANGE;
	}

	ret = icm20602_rd_reg(icm, ICM20602_ACC_CONF_2, &acc_config_2, 1);
	if (ret) {
		dev_err(dev, "%s: failed to read ACC_CONF_2 reg\n", __func__);
		return ret;
	}

	acc_config_2 &= ~(ICM20602_ACC_AVG_MASK);
	acc_config_2 |= dec2_cfg << 4;

	ret = icm20602_wr_reg(icm, ICM20602_ACC_CONF_2, acc_config_2);

	dev_dbg(dev, "%s:  Acc AVG set to '%d'   ACC_CONFIG_2 : 0x%x\n",
		__func__, avg, acc_config_2);

	return ret;
}

static int icm20602_set_gyr_avg(struct icm20602_drv_data *icm, uint8_t avg)
{
	uint8_t ret;
	uint8_t lp_mode_config;
	uint8_t g_avgcfg;
	struct device *dev = icm->dev;

	switch (avg) {
	case ICM20602_GYR_AVG_1_SAMPLES:
		g_avgcfg = ICM20602_GYR_AVG_1_SAMPLES_CFG;
		break;
	case ICM20602_GYR_AVG_2_SAMPLES:
		g_avgcfg = ICM20602_GYR_AVG_2_SAMPLES_CFG;
		break;
	case ICM20602_GYR_AVG_4_SAMPLES:
		g_avgcfg = ICM20602_GYR_AVG_4_SAMPLES_CFG;
		break;
	case ICM20602_GYR_AVG_8_SAMPLES:
		g_avgcfg = ICM20602_GYR_AVG_8_SAMPLES_CFG;
		break;
	case ICM20602_GYR_AVG_16_SAMPLES:
		g_avgcfg = ICM20602_GYR_AVG_16_SAMPLES_CFG;
		break;
	case ICM20602_GYR_AVG_32_SAMPLES:
		g_avgcfg = ICM20602_GYR_AVG_32_SAMPLES_CFG;
		break;
	case ICM20602_GYR_AVG_64_SAMPLES:
		g_avgcfg = ICM20602_GYR_AVG_64_SAMPLES_CFG;
		break;
	case ICM20602_GYR_AVG_128_SAMPLES:
		g_avgcfg = ICM20602_GYR_AVG_128_SAMPLES_CFG;
		break;
	default:
		dev_err(dev, "%s: gyr 'AVG' value out of range !\n", __func__);
		return -ERANGE;
	}

	ret = icm20602_rd_reg(icm, ICM20602_GYR_LPM_CONF, &lp_mode_config, 1);
	if (ret) {
		dev_err(dev, "%s: failed to read lp_mode_conf reg\n", __func__);
		return ret;
	}

	lp_mode_config &= ~(ICM20602_GYR_AVG_MASK);
	lp_mode_config |= g_avgcfg << 4;

	ret = icm20602_wr_reg(icm, ICM20602_GYR_LPM_CONF, lp_mode_config);

	dev_dbg(dev, "%s:  Gyr AVG set to '%d'   GYR_LP_MODE_CONFIG : 0x%x\n",
		__func__, avg, lp_mode_config);

	return ret;
}

static int icm20602_acc_config(struct icm20602_drv_data *icm)
{
	uint8_t ret;
	struct icm20602_config *acc_conf = &icm->acc_conf;
	struct device *dev = icm->dev;

	ret = icm20602_set_acc_dlpf(icm, acc_conf->dlpf);
	if (ret) {
		dev_err(dev, "%s: failed to set acc dlpf value !\n", __func__);
		return ret;
	}

	ret = icm20602_set_acc_fsr(icm, acc_conf->fsr);
	if (ret) {
		dev_err(dev, "%s: failed to set acc fsr value !\n", __func__);
		return ret;
	}

	ret = icm20602_set_acc_avg(icm, acc_conf->avg);
	if (ret)
		dev_err(dev, "%s: failed to set acc avg value !\n", __func__);

	return ret;
}

static int icm20602_gyr_config(struct icm20602_drv_data *icm)
{
	uint8_t ret;
	struct icm20602_config *gyr_conf = &icm->gyr_conf;
	struct device *dev = icm->dev;

	ret = icm20602_set_gyr_dlpf(icm, gyr_conf->dlpf);
	if (ret) {
		dev_err(dev, "%s: failed to set gyr dlpf value !\n", __func__);
		return ret;
	}

	ret = icm20602_set_gyr_fsr(icm, gyr_conf->fsr);
	if (ret) {
		dev_err(dev, "%s: failed to set gyr fsr value !\n", __func__);
		return ret;
	}

	ret = icm20602_set_gyr_avg(icm, gyr_conf->avg);
	if (ret)
		dev_err(dev, "%s: failed to set gyr avg value !\n", __func__);

	return ret;
}

static int icm20602_init(struct icm20602_drv_data *icm)
{
	int ret = 0;
	struct device *dev = icm->dev;
	uint8_t whoami = 0;

	/* to ensure that interrupt line goes low after consecutive reboot*/
	ret = icm20602_wr_reg(icm, ICM20602_PWR_MGM_1,
			      ICM20602_PWR_MGM_1_DEVICE_RESET);
	if (ret) {
		dev_err(dev, "%s: failed to reset device !", __func__);
		return ret;
	}
	/* device will be in sleep mode after reset */
	usleep_range(ICM20602_STARTUP_MIN, ICM20602_STARTUP_MAX);

	ret = icm20602_wr_reg(icm, ICM20602_I2C_IF, ICM20602_I2C_IF_DIS);
	if (ret) {
		dev_err(dev, "%s: failed to disable I2C !", __func__);
		return ret;
	}

	/* without setting this DLPF_CFG, rate can not be changed */
	ret = icm20602_gyr_config(icm);
	if (ret) {
		dev_err(dev, "%s: failed to config gyro !\n", __func__);
		return ret;
	}

	ret = icm20602_acc_config(icm);
	if (ret) {
		dev_err(dev, "%s: failed to conf acc !\n", __func__);
		return ret;
	}

	ret = icm20602_wr_reg(icm, ICM20602_INT_EN,
			      ICM20602_INT_STAT_DATA_RDY_INT);
	if (ret) {
		dev_err(dev, "%s: INT_ENABLE wr failed !\n", __func__);
		return ret;
	}
	ret = icm20602_set_rate(icm, icm->rate);
	if (ret) {
		dev_err(dev, "%s: failed to set rate !\n", __func__);
		return ret;
	}

	ret = icm20602_rd_reg(icm, ICM20602_WHO_AM_I, &whoami, 1);
	if (ret) {
		dev_err(dev, "%s: failed to read WHO_AM_I reg!", __func__);
		return ret;
	}
	if (whoami != ICM20602_WHO_AM_I_VAL) {
		dev_err(dev, "%s: unexpected WHO_AM_I : 0x%x value !",
			__func__, whoami);
		return -EPROBE_DEFER;
	}

	return ret;
}

static void icm20602_report_input_data(struct input_dev *in_dev,
				       struct icm20602_sample *data)
{
	input_event(in_dev, EV_REL, REL_X, data->x);
	input_event(in_dev, EV_REL, REL_Y, data->y);
	input_event(in_dev, EV_REL, REL_Z, data->z);
	input_event(in_dev, EV_REL, REL_MISC, data->temp);
	input_event(in_dev, EV_MSC, MSC_TIMESTAMP, data->ts);
	input_sync(in_dev);
}

static int icm20602_rd_drdy_int(struct icm20602_drv_data *icm)
{
	uint8_t ret = 0;
	uint8_t rd_try = 0;
	struct device *dev = icm->dev;
	uint8_t int_status = 0;

	while (rd_try < ICM20602_MAX_RD_TRY) {
		ret = icm20602_rd_reg(icm, ICM20602_INT_STAT, &int_status, 1);
		if (ret) {
			dev_dbg(dev, "%s: failed to rd STAT_1 !\n", __func__);
			return ret;
		}
		if (int_status & ICM20602_INT_STAT_DATA_RDY_INT)
			break;
		rd_try++;
		udelay(10);
	}

	if (rd_try >= ICM20602_MAX_RD_TRY) {
		dev_warn(dev, "%s: max rd try reached !\n", __func__);
		return -EIO;
	}

	return ret;
}

static int icm20602_rd_acc_data(struct icm20602_drv_data *icm)
{
	int ret = 0;
	struct device *dev = icm->dev;
	struct icm20602_sample *acc_data = &icm->acc_data;
	uint8_t rbuf[ICM20602_MAX_BUF_SIZE];
	struct timespec ts;

	ret = icm20602_rd_reg(icm, ICM20602_ACC_X_H, rbuf, 6);
	if (ret) {
		dev_err(dev, "%s: failed to rd data reg !\n", __func__);
		return ret;
	}

	ktime_get_ts(&ts);
	acc_data->ts = timespec_to_ns(&ts);

	acc_data->x = be16_to_cpup((__be16 *)(&rbuf[0]));
	acc_data->y = be16_to_cpup((__be16 *)(&rbuf[2]));
	acc_data->z = be16_to_cpup((__be16 *)(&rbuf[4]));

	if (!icm->st_en)
		icm20602_report_input_data(icm->in_dev_acc, acc_data);

	return ret;
}

static int icm20602_rd_gyr_data(struct icm20602_drv_data *icm)
{
	int ret = 0;
	struct device *dev = icm->dev;
	struct icm20602_sample *gyr_data = &icm->gyr_data;
	uint8_t rbuf[ICM20602_MAX_BUF_SIZE];
	struct timespec ts;

	ret = icm20602_rd_reg(icm, ICM20602_GYR_X_H, rbuf, 6);
	if (ret) {
		dev_err(dev, "%s: failed to rd data reg !\n", __func__);
		return ret;
	}

	ktime_get_ts(&ts);
	gyr_data->ts = timespec_to_ns(&ts);

	gyr_data->x = be16_to_cpup((__be16 *)(&rbuf[0]));
	gyr_data->y = be16_to_cpup((__be16 *)(&rbuf[2]));
	gyr_data->z = be16_to_cpup((__be16 *)(&rbuf[4]));

	if (!icm->st_en)
		icm20602_report_input_data(icm->in_dev_gyr, gyr_data);

	return ret;
}

static int icm20602_rd_temp(struct icm20602_drv_data *icm)
{
	uint8_t ret;
	int16_t temp;
	uint8_t rbuf[ICM20602_MAX_BUF_SIZE];
	struct icm20602_sample *acc_data = &icm->acc_data;
	struct icm20602_sample *gyr_data = &icm->gyr_data;
	struct device *dev = icm->dev;

	ret = icm20602_rd_reg(icm, ICM20602_TEMP_H, rbuf, 2);
	if (ret) {
		dev_err(dev, "%s: failed to rd temperature reg !\n",
			__func__);
		return ret;
	}

	temp = be16_to_cpup((__be16 *)(&rbuf[0]));
	acc_data->temp = (((temp * 10) / TEMP_SENS) + ROOM_TEMP_OFFSET);
	gyr_data->temp = acc_data->temp;

	return ret;
}

static int icm20602_self_test(struct icm20602_drv_data *icm,
			     uint8_t start)
{
	int ret;
	uint8_t acc_config, gyr_config;
	struct device *dev = icm->dev;
	struct icm20602_config *acc_conf = &icm->acc_conf;
	struct icm20602_config *gyr_conf = &icm->gyr_conf;

	if (icm->st_en == !!start)
		return 0;

	if ((acc_conf->en || gyr_conf->en) == !!start)
		return -EACCES;

	ret  = icm20602_rd_reg(icm, ICM20602_ACC_CONF, &acc_config, 1);
	ret |= icm20602_rd_reg(icm, ICM20602_GYR_CONF, &gyr_config, 1);
	if (start) {
		acc_config |= ICM20602_ACC_GYR_ST_MASK;
		gyr_config |= ICM20602_ACC_GYR_ST_MASK;
	} else {
		acc_config &= ~ICM20602_ACC_GYR_ST_MASK;
		gyr_config &= ~ICM20602_ACC_GYR_ST_MASK;
	}

	ret |= icm20602_wr_reg(icm, ICM20602_ACC_CONF, acc_config);
	ret |= icm20602_wr_reg(icm, ICM20602_GYR_CONF, gyr_config);
	if (ret) {
		dev_err(dev, "%s: failed to start self test !", __func__);
		return ret;
	}

	if (start) {
		ret  = icm20602_set_mode(icm, ICM20602_MODE_ENABLE_GYR);
		ret |= icm20602_set_mode(icm, ICM20602_MODE_ENABLE_ACC);
		if (!ret)
			icm->st_en = ENABLED;
	} else {
		ret  = icm20602_set_mode(icm, ICM20602_MODE_DISABLE_GYR);
		ret |= icm20602_set_mode(icm, ICM20602_MODE_DISABLE_ACC);
		if (!ret)
			icm->st_en = DISABLED;
	}

	return ret;
}

static int icm20602_rd_st_code(struct icm20602_drv_data *icm,
			      uint8_t *acc_st_code,
			      uint8_t *gyr_st_code)
{
	int ret;
	struct device *dev = icm->dev;

	ret = icm20602_rd_reg(icm, ICM20602_SELF_TEST_X_ACCEL, acc_st_code, 3);
	if (ret) {
		dev_err(dev, "%s: failed to rd ACCEL_ST regs !\n", __func__);
		return ret;
	}

	ret = icm20602_rd_reg(icm, ICM20602_SELF_TEST_X_GYRO, gyr_st_code, 3);
	if (ret)
		dev_err(dev, "%s: failed to rd GYRO_ST regs !\n", __func__);

	return ret;
}

static irqreturn_t icm20602_irq_threaded(int irq, void *data)
{
	struct icm20602_drv_data *icm = (struct icm20602_drv_data *)data;
	struct icm20602_config *acc_conf = &icm->acc_conf;
	struct icm20602_config *gyr_conf = &icm->gyr_conf;

	if (acc_conf->en || gyr_conf->en) {
		if (icm20602_rd_drdy_int(icm))
			goto exit;

		if (icm20602_rd_temp(icm))
			goto exit;
	}

	if (acc_conf->en) {
		if (icm20602_rd_acc_data(icm))
			goto exit;
	}
	if (gyr_conf->en) {
		if (icm20602_rd_gyr_data(icm))
			goto exit;
	}

exit:
	return IRQ_HANDLED;
}

static int icm20602_in_dev_gyr_register(struct icm20602_drv_data *icm)
{
	int ret = 0;
	struct device *dev = icm->dev;

	icm->in_dev_gyr = devm_input_allocate_device(dev);
	if (!icm->in_dev_gyr)
		return -ENOMEM;

	icm->in_dev_gyr->name = "icm20602_gyr_input_dev";
	icm->in_dev_gyr->id.bustype = BUS_SPI;
	icm->in_dev_gyr->dev.parent = dev;
	input_set_capability(icm->in_dev_gyr, EV_REL, REL_X);
	input_set_capability(icm->in_dev_gyr, EV_REL, REL_Y);
	input_set_capability(icm->in_dev_gyr, EV_REL, REL_Z);
	input_set_capability(icm->in_dev_gyr, EV_REL, REL_MISC);
	input_set_capability(icm->in_dev_gyr, EV_MSC, MSC_TIMESTAMP);

	ret = input_register_device(icm->in_dev_gyr);

	return ret;
}

static int icm20602_in_dev_acc_register(struct icm20602_drv_data *icm)
{
	int ret = 0;
	struct device *dev = icm->dev;

	icm->in_dev_acc = devm_input_allocate_device(dev);
	if (!icm->in_dev_acc)
		return -ENOMEM;

	icm->in_dev_acc->name = "icm20602_acc_input_dev";
	icm->in_dev_acc->id.bustype = BUS_SPI;
	icm->in_dev_acc->dev.parent = dev;
	input_set_capability(icm->in_dev_acc, EV_REL, REL_X);
	input_set_capability(icm->in_dev_acc, EV_REL, REL_Y);
	input_set_capability(icm->in_dev_acc, EV_REL, REL_Z);
	input_set_capability(icm->in_dev_acc, EV_REL, REL_MISC);
	input_set_capability(icm->in_dev_acc, EV_MSC, MSC_TIMESTAMP);

	ret = input_register_device(icm->in_dev_acc);

	return ret;
}

static ssize_t acc_enable_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf,
			    size_t count)
{
	uint8_t ret = 0;
	struct icm20602_drv_data *icm = dev->platform_data;

	if (icm->st_en) {
		dev_err(dev, "%s: unavailable in ST mode !\n", __func__);
		goto exit;
	}

	ret = icm20602_set_mode(icm, (buf[0] == '0') ?
				ICM20602_MODE_DISABLE_ACC :
				ICM20602_MODE_ENABLE_ACC);
	if (ret)
		dev_err(dev, "%s: failed to switch mode for acc .\n", __func__);

exit:
	return count;
}

static ssize_t acc_enable_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct icm20602_drv_data *icm = dev->platform_data;
	struct icm20602_config *acc_conf = &icm->acc_conf;

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!acc_conf->en);
}
static DEVICE_ATTR_RW(acc_enable);

static ssize_t gyr_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	uint8_t ret = 0;
	struct icm20602_drv_data *icm = dev->platform_data;

	if (icm->st_en) {
		dev_err(dev, "%s: unavailable in ST mode !\n", __func__);
		goto exit;
	}

	ret = icm20602_set_mode(icm, (buf[0] == '0') ?
				ICM20602_MODE_DISABLE_GYR :
				ICM20602_MODE_ENABLE_GYR);
	if (ret)
		dev_err(dev, "%s: failed to switch mode for gyro.\n", __func__);

exit:
	return count;
}

static ssize_t gyr_enable_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct icm20602_drv_data *icm = dev->platform_data;
	struct icm20602_config *gyr_conf = &icm->gyr_conf;

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!gyr_conf->en);
}
static DEVICE_ATTR_RW(gyr_enable);

static ssize_t whoami_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	uint8_t rbuf[ICM20602_MAX_BUF_SIZE];
	struct icm20602_drv_data *icm = dev->platform_data;
	int ret;

	ret = icm20602_rd_reg(icm, ICM20602_WHO_AM_I, rbuf, 1);
	if (ret) {
		dev_err(dev, "%s: failed to read WHO_AM_I reg !", __func__);
		return ret;
	}
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", rbuf[0]);
}
static DEVICE_ATTR_RO(whoami);

static ssize_t acc_data_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct icm20602_drv_data *icm = dev->platform_data;
	struct icm20602_sample *acc_data = &icm->acc_data;
	struct icm20602_config *acc_conf = &icm->acc_conf;

	if (!acc_conf->en)
		return -EACCES;

	return scnprintf(buf, PAGE_SIZE, "%d, %d, %d, %d\n",
			acc_data->x, acc_data->y, acc_data->z, acc_data->temp);
}
static DEVICE_ATTR_RO(acc_data);

static ssize_t gyr_data_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct icm20602_drv_data *icm = dev->platform_data;
	struct icm20602_sample *gyr_data = &icm->gyr_data;
	struct icm20602_config *gyr_conf = &icm->gyr_conf;

	if (!gyr_conf->en)
		return -EACCES;

	return scnprintf(buf, PAGE_SIZE, "%d, %d, %d, %d\n",
			 gyr_data->x, gyr_data->y, gyr_data->z, gyr_data->temp);
}
static DEVICE_ATTR_RO(gyr_data);

static ssize_t rate_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	int ret;
	uint16_t rate;
	struct icm20602_drv_data *icm = dev->platform_data;

	ret = kstrtou16(buf, 10, &rate);
	if (ret) {
		dev_err(dev, "%s: failed to convert rate value", __func__);
		goto exit;
	}

	if (rate == icm->rate)
		goto exit;

	ret = icm20602_set_rate(icm, rate);
	if (ret)
		dev_err(dev, "Invalid rate value !\n");

exit:
	return count;
}

static ssize_t rate_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct icm20602_drv_data *icm = dev->platform_data;
	struct icm20602_config *acc_conf = &icm->acc_conf;
	struct icm20602_config *gyr_conf = &icm->gyr_conf;

	if (acc_conf->en || gyr_conf->en)
		return scnprintf(buf, PAGE_SIZE, "rate: %dHz\n", icm->rate);
	else
		return -EACCES;
}
static DEVICE_ATTR_RW(rate);

static ssize_t self_test_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	struct icm20602_drv_data *icm = dev->platform_data;
	int ret;

	ret = icm20602_self_test(icm, (buf[0] == '0') ? DISABLED : ENABLED);
	if (ret == -EACCES) {
		dev_err(dev, "%s: self test not available !\n", __func__);
		goto exit;
	}
	if (ret) {
		dev_err(dev, "%s: self test failed !\n", __func__);
		goto exit;
	}

	if (icm->st_en)
		dev_dbg(dev, "%s: ST started !\n", __func__);
	else
		dev_dbg(dev, "%s: ST stopped !\n", __func__);

exit:
	return count;
}

static DEVICE_ATTR_WO(self_test);

static ssize_t st_code_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int ret;
	struct icm20602_drv_data *icm = dev->platform_data;
	uint8_t acc_st_code[3], gyr_st_code[3];

	ret = icm20602_rd_st_code(icm, &acc_st_code[0], &gyr_st_code[0]);
	if (ret)
		return -EIO;

	return scnprintf(buf, PAGE_SIZE, "%d, %d, %d\n%d, %d, %d\n",
			 acc_st_code[0],  acc_st_code[1], acc_st_code[2],
			 gyr_st_code[0],  gyr_st_code[1], gyr_st_code[2]);
}
static DEVICE_ATTR_RO(st_code);

static struct attribute *icm20602_attributes[] = {
	&dev_attr_acc_enable.attr,
	&dev_attr_gyr_enable.attr,
	&dev_attr_whoami.attr,
	&dev_attr_acc_data.attr,
	&dev_attr_gyr_data.attr,
	&dev_attr_rate.attr,
	&dev_attr_self_test.attr,
	&dev_attr_st_code.attr,
	NULL
};

static const struct attribute_group icm20602_attr_group = {
	.attrs = icm20602_attributes,
};

static void icm20602_parse_dt(struct icm20602_drv_data *icm)
{
	unsigned int prop;
	struct spi_device *spi = icm->spi;
	struct device *dev = &spi->dev;
	struct device_node *nc = spi->dev.of_node;
	struct icm20602_config *acc_conf = &icm->acc_conf;
	struct icm20602_config *gyr_conf = &icm->gyr_conf;

	if (!of_property_read_u32(nc, "rate", &prop)) {
		icm->rate = prop;
	} else {
		dev_err(dev, "%s: prop 'rate' not found !\n", __func__);
		icm->rate = ICM20602_DFLT_RATE;
	}

	/* acc configs */
	if (!of_property_read_u32(nc, "acc_en", &prop)) {
		acc_conf->en = prop;
	} else {
		dev_err(dev, "%s: prop 'acc_en' not found !\n", __func__);
		acc_conf->en = 0;
	}

	if (!of_property_read_u32(nc, "acc_fsr", &prop)) {
		acc_conf->fsr = prop;
	} else {
		dev_err(dev, "%s: prop 'acc_fsr' not found !\n", __func__);
		acc_conf->fsr = ICM20602_DFLT_ACC_FSR;
	}

	if (!of_property_read_u32(nc, "acc_dlpf", &prop)) {
		acc_conf->dlpf = prop;
	} else {
		dev_err(dev, "%s: prop 'acc_dlpf' not found !\n", __func__);
		acc_conf->dlpf = ICM20602_DFLT_ACC_DLPF;
	}

	if (!of_property_read_u32(nc, "acc_avg", &prop)) {
		acc_conf->avg = prop;
	} else {
		dev_err(dev, "%s: prop 'acc_avg' not found !\n", __func__);
		acc_conf->avg = ICM20602_DFLT_ACC_AVG;
	}

	/* gyr configs */
	if (!of_property_read_u32(nc, "gyr_en", &prop)) {
		gyr_conf->en = prop;
	} else {
		dev_err(dev, "%s: prop 'gyr_en' not found !\n", __func__);
		gyr_conf->en = 0;
	}

	if (!of_property_read_u32(nc, "gyr_fsr", &prop)) {
		gyr_conf->fsr = prop;
	} else {
		dev_err(dev, "%s: prop 'gyr_fsr' not found !\n", __func__);
		gyr_conf->fsr = ICM20602_DFLT_GYR_FSR;
	}

	if (!of_property_read_u32(nc, "gyr_dlpf", &prop)) {
		gyr_conf->dlpf = prop;
	} else {
		dev_err(dev, "%s: prop 'gyr_dlpf' not found !\n", __func__);
		gyr_conf->dlpf = ICM20602_DFLT_GYR_DLPF;
	}

	if (!of_property_read_u32(nc, "gyr_avg", &prop)) {
		gyr_conf->avg = prop;
	} else {
		dev_err(dev, "%s: prop 'gyr_avg' not found !\n", __func__);
		gyr_conf->avg = ICM20602_DFLT_GYR_AVG;
	}
}

static int icm20602_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct icm20602_drv_data *icm = NULL;
	int ret = 0;

	dev_info(dev, "%s: enter\n", __func__);

	if (spi->irq <= 0) {
		dev_err(dev, "%s: no irq defined\n", __func__);
		return -ENODEV;
	}

	icm = devm_kzalloc(dev, sizeof(*icm), GFP_KERNEL);
	if (!icm)
		return -ENOMEM;

	icm->spi = spi;
	icm->dev = dev;
	dev->platform_data = icm;
	spi_set_drvdata(spi, icm);
	mutex_init(&icm->mode_lock);

	icm20602_parse_dt(icm);
	ret = icm20602_init(icm);
	if (ret) {
		dev_err(dev, "%s: init failed !", __func__);
		goto free_mutex;
	}

	ret = devm_request_threaded_irq(dev, spi->irq,
					NULL,
					icm20602_irq_threaded,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"icm20602_irq", icm);
	if (ret) {
		dev_err(dev, "%s: failed to request irq !\n", __func__);
		goto free_mutex;
	}

	ret = icm20602_in_dev_acc_register(icm);
	if (ret) {
		dev_err(dev, "%s: failed to regi acc input device\n", __func__);
		goto free_irq;
	}

	ret = icm20602_in_dev_gyr_register(icm);
	if (ret) {
		dev_err(dev, "%s: failed to regi gyr input device\n", __func__);
		goto dereg_in_dev_acc;
	}

	ret = sysfs_create_group(&spi->dev.kobj, &icm20602_attr_group);
	if (ret < 0) {
		dev_err(dev, "%s: failed to create sysfs :%d\n", __func__, ret);
		goto dereg_in_dev_gyr;
	}
	dev_info(dev, "%s: Success !\n", __func__);

	return ret;

dereg_in_dev_gyr:
	icm->in_dev_gyr = NULL;
dereg_in_dev_acc:
	icm->in_dev_acc = NULL;
free_irq:
	devm_free_irq(dev, spi->irq, icm);
free_mutex:
	mutex_destroy(&icm->mode_lock);
	spi_set_drvdata(spi, NULL);

	return ret;
}

static int icm20602_remove(struct spi_device *spi)
{
	int ret = 0;
	struct icm20602_drv_data *icm = spi_get_drvdata(spi);

	dev_info(&spi->dev, "%s: enter\n", __func__);
	sysfs_remove_group(&spi->dev.kobj, &icm20602_attr_group);
	icm->in_dev_gyr = NULL;
	icm->in_dev_acc = NULL;
	devm_free_irq(&spi->dev, spi->irq, icm);
	mutex_destroy(&icm->mode_lock);
	spi_set_drvdata(spi, NULL);
	return ret;
}

static const struct of_device_id icm20602_match[] = {
	{ .compatible = "ml,icm20602" },
	{ },
};
MODULE_DEVICE_TABLE(of, icm20602_match);

static struct spi_driver icm20602_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "icm20602",
		.of_match_table = icm20602_match,
	},
	.probe = icm20602_probe,
	.remove = icm20602_remove,
};
module_spi_driver(icm20602_spi_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("Magic Leap ICM20602 Beltpack Driver");
MODULE_LICENSE("GPL v2");
