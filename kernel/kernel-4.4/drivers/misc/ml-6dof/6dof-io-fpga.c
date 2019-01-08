/* Copyright (c) 2017-2018, Magic Leap, Inc. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <asm/unaligned.h>
#include "6dof-io.h"
#include "6dof-common.h"
#include "6dof-io-fpga.h"

/* Who am I register */
#define BP_FPGA_WHOAMI_REG             0x00
#define BP_FPGA_WHOAMI_VAL             0xbc

/* FPGA revision registers */
#define BP_FPGA_REV_MAJOR_REG          0x01
#define BP_FPGA_REV_MINOR_REG          0x0b
#define BP_FPGA_MIN_SUPPORTED_REV      0x0f02
#define WL_FPGA_MIN_SUPPORTED_REV      0x1001

/* Subsystem enable register */
#define BP_FPGA_SS_EN_REG              0x02
#define BP_FPGA_SS_EN_FREQ_CORRECTION  BIT(5)
#define BP_FPGA_SS_EN_PHASE_CORRECTION BIT(6)

/* System test register */
#define BP_FPGA_TEST_REG               0x03
#define BP_FPGA_TEST_ADC               BIT(7)

#define BP_FPGA_STATUS_REG             0x0d
#define BP_FPGA_STATUS_T1_PWR_LOCK     BIT(0)
#define BP_FPGA_STATUS_T2_PWR_LOCK     BIT(1)
#define BP_FPGA_STATUS_T1_W1_LOCK      BIT(2)
#define BP_FPGA_STATUS_T1_W2_LOCK      BIT(3)
#define BP_FPGA_STATUS_T1_W3_LOCK      BIT(4)
#define BP_FPGA_STATUS_T2_W1_LOCK      BIT(5)
#define BP_FPGA_STATUS_T2_W2_LOCK      BIT(6)
#define BP_FPGA_STATUS_T2_W3_LOCK      BIT(7)

/* s1new control register */
#define BP_FPGA_CTRL_REG               0x0c
#define BP_FPGA_CTRL_BYPASS_MODE       0x00
#define BP_FPGA_CTRL_TRANS_MODE        0x40
#define BP_FPGA_CTRL_NOTCH_MODE        0x80
#define BP_FPGA_CTRL_PLL_MODE          0xc0

/* L2R control register */
#define BP_FPGA_L2R_CTRL_REG           0x34
#define BP_FPGA_L2R_START_IO           BIT(0)

/* L2R status register */
#define BP_FPGA_L2R_STATUS_REG         0x35
#define BP_FPGA_L2R_STATUS_IN_PROG     BIT(2)
#define BP_FPGA_L2R_STATUS_OK          BIT(3)
#define BP_FPGA_L2R_STATUS_TOUT        BIT(4)

/* L2R TX memory register */
#define BP_FPGA_L2R_TX_REG             0x56

/* L2R RX memory register */
#define BP_FPGA_L2R_RX_REG             0x57

#define BP_FPGA_L2R_MAX_DATA_LEN       0xff

/* L2R command to program CORDIC memory */
#define L2R_CMD_CORDIC_MEM             0x00

/* L2R command to program EMT filter coefficient memory */
#define L2R_CMD_FILTER_COEFF_MEM       0x01

/* L2R command to program EMT CTRL register */
#define L2R_CMD_CORDIC_PLL_WRITE       0x00
#define L2R_CMD_FILTER_COEF_WRITE      0x01
#define L2R_CMD_EMT_CTRL_REG_WRITE     0x02
#define L2R_CMD_EMT_CTRL_REG_READ      0x03
#define L2R_CMD_ADC_READ               0x04

#define L2R_EMT_CTRL_REG_CAL           BIT(15)
#define L2R_EMT_CTRL_REG_EN            BIT(30)
#define L2R_EMT_CTRL_REG_MODE          BIT(31)
#define L2R_EMT_CTRL_REG_CAL_ENABLE    0x000009ff
#define L2R_EMT_CTRL_REG_CAL_DISABLE   0x00000a2a

struct l2r_ctrl_wd {
	u32 offset                     : 8;
	u32 reserved2                  : 8;
	u32 wd_count                   : 8;
	u32 cmd                        : 4;
	u32 reserved1                  : 4;
} __packed;

struct l2r_tx {
	u16                            address;
	struct l2r_ctrl_wd             ctrl_wd;
	u32                            buf[BP_FPGA_L2R_MAX_DATA_LEN];
} __packed;

struct l2r_status_wd {
	u32 reserved3                  : 6;
	u32 cmd_success                : 1;
	u32 reserved2                  : 7;
	u32 cmd_failure                : 1;
	u32 reserved1                  : 1;
	u32 rev_low                    : 8;
	u32 rev_high                   : 8;
} __packed;

struct l2r_rx {
	struct l2r_status_wd           status_wd;
	u32                            buf[BP_FPGA_L2R_MAX_DATA_LEN];
} __packed;

#define EMT_FREQ_NUM 21
#define EMT_DEF_FREQ_TOTEM_1   10
#define EMT_DEF_FREQ_TOTEM_2   11
static const u32 freq_list[EMT_FREQ_NUM][EMT_AXIS_NUM] = {
	{ 0x00200000, 0x00200000, 0x00200000 }, /**< x=93.75 y=93.75 z=93.75 */
	{ 0x23c28f5c, 0x24147ae1, 0x24666666 }, /**< x=26820 y=27060 z=27300 */
	{ 0x24e147ae, 0x25333333, 0x25851eb8 }, /**< x=27660 y=27900 z=28140 */
	{ 0x26000000, 0x2651eb85, 0x26a3d70a }, /**< x=28500 y=28740 z=28980 */
	{ 0x271eb851, 0x2770a3d7, 0x27c28f5c }, /**< x=29340 y=29580 z=29820 */
	{ 0x283d70a3, 0x288f5c28, 0x28e147ae }, /**< x=30180 y=30420 z=30660 */
	{ 0x295c28f5, 0x29ae147a, 0x2a000000 }, /**< x=31020 y=31260 z=31500 */
	{ 0x2a7ae147, 0x2acccccc, 0x2b1eb851 }, /**< x=31860 y=32100 z=32340 */
	{ 0x2b999999, 0x2beb851e, 0x2c3d70a3 }, /**< x=32700 y=32940 z=33180 */
	{ 0x2cb851eb, 0x2d0a3d70, 0x2d5c28f5 }, /**< x=33540 y=33780 z=34020 */
	{ 0x2dd70a3d, 0x2e28f5c2, 0x2e7ae147 }, /**< x=34380 y=34620 z=34860 */
	{ 0x2ef5c28f, 0x2f47ae14, 0x2f999999 }, /**< x=35220 y=35460 z=35700 */
	{ 0x30147ae1, 0x30666666, 0x30b851eb }, /**< x=36060 y=36300 z=36540 */
	{ 0x31333333, 0x31851eb8, 0x31d70a3d }, /**< x=36900 y=37140 z=37380 */
	{ 0x3251eb85, 0x32a3d70a, 0x32f5c28f }, /**< x=37740 y=37980 z=38220 */
	{ 0x3370a3d7, 0x33c28f5c, 0x34147ae1 }, /**< x=38580 y=38820 z=39060 */
	{ 0x348f5c28, 0x34e147ae, 0x35333333 }, /**< x=39420 y=39660 z=39900 */
	{ 0x35ae147a, 0x36000000, 0x3651eb85 }, /**< x=40260 y=40500 z=40740 */
	{ 0x36cccccc, 0x371eb851, 0x3770a3d7 }, /**< x=41100 y=41340 z=41580 */
	{ 0x37eb851e, 0x383d70a3, 0x388f5c28 }, /**< x=41940 y=42180 z=42420 */
	{ 0x33333333, 0x33333333, 0x33333333 }, /**< x=38400 y=38400 z=38400 */
};

enum em_coil_type {
	EM_COIL_M3427,             /**< (x, y, z) -> (z, -x, y) */
	EM_COIL_M3440REVB,         /**< (x, y, z) -> (z, -y, -x) */
	EM_COIL_M3427_NOT_UNICORN, /**< (x, y, z) -> (-x, -y, -z) */
	EM_COIL_NUM
};

struct em_coil_orientation_map {
	enum em_coil_type coil;
	u8                xyz;
};

static const struct em_coil_orientation_map s_coil_map[EM_COIL_NUM] = {
	{EM_COIL_M3427, 0x22},
	{EM_COIL_M3440REVB, 0x2b},
	{EM_COIL_M3427_NOT_UNICORN, 0x07},
};

struct fpga_data {
	struct device          *dev;
	struct i2c_client      *i2c;
	struct regmap          *rmap;
	struct ml_fpga_io      io;
	u8                     emcfg;
	u8                     freq_t1;
	u8                     freq_t2;
	struct l2r_tx          l2r_tx;
	struct l2r_rx          l2r_rx;
	u32                    freq_adjustment[FPGA_FREQ_ADJ_LEN];
	struct mutex           l2r_lock;
	struct workqueue_struct *l2r_work_queue;
	struct work_struct     l2r_work;
};

#define io_2_pdata(x) container_of((x), struct fpga_data, io)

static const struct regmap_config regmap_cfg = {
	.name         = "bp_fpga",
	.reg_stride   = 1,
	.reg_bits     = 8,
	.val_bits     = 8,
	.max_register = BP_FPGA_L2R_STATUS_REG,
};

static int fpga_set_power(struct ml_fpga_io *io, bool on)
{
	struct fpga_data *data = io_2_pdata(io);

	dev_dbg(data->dev, "set power %s\n", on ? "on" : "off");
	if (on) {
		return set_bits(data->dev, data->rmap,
				BP_FPGA_SS_EN_REG,
				BP_FPGA_SS_EN_FREQ_CORRECTION |
				BP_FPGA_SS_EN_PHASE_CORRECTION);
	} else {
		cancel_work_sync(&data->l2r_work);
		return clr_bits(data->dev, data->rmap,
				BP_FPGA_SS_EN_REG,
				BP_FPGA_SS_EN_FREQ_CORRECTION |
				BP_FPGA_SS_EN_PHASE_CORRECTION);
	}
}

static int fpga_set_mode(struct ml_fpga_io *io, enum ml_6dof_mode mode)
{
	struct fpga_data *data = io_2_pdata(io);
	u32 val;

	dev_dbg(data->dev, "set mode to %s\n", emt_mode_to_str(mode));
	switch (mode) {
	case EMT_MODE_TDSP:
		val = s_coil_map[data->emcfg].xyz | BP_FPGA_CTRL_NOTCH_MODE;
		return write_reg(data->dev, data->rmap, BP_FPGA_CTRL_REG, val);
	case EMT_MODE_ADC:
		return write_reg(data->dev, data->rmap, BP_FPGA_CTRL_REG, 0);
	default:
		return -EINVAL;
	}
}

static u8  fpga_get_freq_num(void)
{
	return EMT_FREQ_NUM;
}

static struct ml_6dof_freq fpga_freq(u8 idx)
{
	struct ml_6dof_freq freq;

	WARN_ON(idx >= EMT_FREQ_NUM);
	if (idx >= EMT_FREQ_NUM)
		idx = 0;

	freq.x = freq_list[idx][EMT_AXIS_X];
	freq.y = freq_list[idx][EMT_AXIS_Y];
	freq.z = freq_list[idx][EMT_AXIS_Z];

	return freq;
}

static int fpga_query_freq(struct ml_fpga_io *io, u8 totem, u8 *idx)
{
	struct fpga_data *data = io_2_pdata(io);

	switch (totem) {
	case ML_6DOF_TOTEM_1_ID:
		*idx = data->freq_t1;
		break;
	case ML_6DOF_TOTEM_2_ID:
		*idx = data->freq_t2;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int l2r_poll_status(struct fpga_data *data)
{
	u32 status;
	int rc, i = 0;

	do {
		rc = read_reg(data->dev, data->rmap, BP_FPGA_L2R_STATUS_REG,
			      &status);
		if (rc) {
			dev_err(data->dev,
				"failed to read l2r status reg, %d\n", rc);
			return rc;
		}

		if (status & BP_FPGA_L2R_STATUS_IN_PROG) {
			usleep_range(100, 150);
			continue;
		}

		if (status & BP_FPGA_L2R_STATUS_TOUT) {
			dev_err(data->dev, "l2r read timedout\n");
			return -ETIMEDOUT;
		}

		return 0;
	} while (++i < 3);

	return -ETIMEDOUT;
}

static int l2r_write_hdr(struct fpga_data *data, u8 cmd, u8 offset, u8 val_len)
{
	size_t sz;
	int rc;

	/* Populate control word */
	memset(&data->l2r_tx.ctrl_wd, 0, sizeof(struct l2r_ctrl_wd));
	data->l2r_tx.ctrl_wd.cmd = cmd;
	data->l2r_tx.ctrl_wd.offset = offset;
	data->l2r_tx.ctrl_wd.wd_count = val_len;
	cpu_to_be32s((u32 *)&data->l2r_tx.ctrl_wd);

	sz = sizeof(data->l2r_tx.address) + sizeof(data->l2r_tx.ctrl_wd);
	rc = raw_write(data->dev, data->rmap, BP_FPGA_L2R_TX_REG,
		       &data->l2r_tx, sz);
	if (rc)
		goto exit;

	rc = write_reg(data->dev, data->rmap, BP_FPGA_L2R_CTRL_REG,
		       BP_FPGA_L2R_START_IO);
exit:
	return rc;
}

static int l2r_write(struct fpga_data *data, u8 cmd, u8 offset, const u32 *val,
		     u8 val_len)
{
	size_t sz;
	u32 val32;
	int rc;

	if (val_len == 0)
		return -EINVAL;

	/* Populate control word */
	memset(&data->l2r_tx.ctrl_wd, 0, sizeof(struct l2r_ctrl_wd));
	data->l2r_tx.ctrl_wd.cmd = cmd;
	data->l2r_tx.ctrl_wd.offset = offset;
	data->l2r_tx.ctrl_wd.wd_count = val_len;
	cpu_to_be32s((u32 *)&data->l2r_tx.ctrl_wd);

	/* Populate data buffer */
	for (sz = 0; sz < val_len; sz++) {
		val32 = cpu_to_be32(val[sz]);
		put_unaligned(val32, &data->l2r_tx.buf[sz]);
	}

	sz = sizeof(data->l2r_tx.address) +
		sizeof(data->l2r_tx.ctrl_wd) +
		val_len * sizeof(u32);
	rc = raw_write(data->dev, data->rmap, BP_FPGA_L2R_TX_REG,
		       &data->l2r_tx, sz);
	if (rc)
		goto exit;

	rc = write_reg(data->dev, data->rmap, BP_FPGA_L2R_CTRL_REG,
		       BP_FPGA_L2R_START_IO);

exit:
	return rc;
}

static int l2r_read(struct fpga_data *data, u8 val_len)
{
	size_t sz;
	u16 val = 0;
	int rc;

	if (val_len == 0)
		return -EINVAL;

	rc = raw_write(data->dev, data->rmap, BP_FPGA_L2R_RX_REG, &val,
		       sizeof(u16));
	if (rc)
		goto exit;

	sz = sizeof(data->l2r_rx.status_wd) + val_len * sizeof(u32);
	rc = raw_read(data->dev, data->rmap, BP_FPGA_L2R_RX_REG, &data->l2r_rx,
		      sz);
	if (rc)
		goto exit;

	cpu_to_be32s((u32 *)&data->l2r_rx.status_wd);
	if (data->l2r_rx.status_wd.cmd_success) {
		be32_to_cpu_array(data->l2r_rx.buf, data->l2r_rx.buf, val_len);
	} else {
		dev_err(data->dev, "l2r read failure: status_wd = 0x%08x\n",
			*((const u32 *)&data->l2r_rx.status_wd));
		rc = -EIO;
	}

exit:
	return rc;
}

static int fpga_revision(struct ml_fpga_io *io)
{
	struct fpga_data *data = io_2_pdata(io);
	u32 val = 0;
	u16 rev;
	int rc;

	rc = read_reg(data->dev, data->rmap, BP_FPGA_REV_MAJOR_REG, &val);
	if (rc) {
		dev_err(data->dev, "BP failed to read FPGA rev, %d\n", rc);
		goto exit;
	}
	rev = ((val & 0x000000ff) << 8);

	rc = read_reg(data->dev, data->rmap, BP_FPGA_REV_MINOR_REG, &val);
	if (rc) {
		dev_err(data->dev, "failed to read BP FPGA rev, %d\n", rc);
		goto exit;
	}
	rev |= val;
	dev_info(data->dev, "BP FPGA rev 0x%04x\n", rev);

	if (rev < BP_FPGA_MIN_SUPPORTED_REV) {
		dev_err(data->dev,
			"BP: 0x%04x < min supported version 0x%04x\n",
			rev, BP_FPGA_MIN_SUPPORTED_REV);
		rc = -EFAULT;
		goto exit;
	}

	mutex_lock(&data->l2r_lock);
	rc = l2r_write_hdr(data, L2R_CMD_EMT_CTRL_REG_READ, 0, 1);
	if (rc)
		goto unlock_l2r;

	rc = l2r_poll_status(data);
	if (rc)
		goto unlock_l2r;

	rc = l2r_read(data, 1);
	if (rc)
		goto unlock_l2r;

	rev = (data->l2r_rx.status_wd.rev_high << 8);
	rev |= data->l2r_rx.status_wd.rev_low;

	dev_info(data->dev, "WL FPGA rev 0x%04x\n", rev);
	if (rev < WL_FPGA_MIN_SUPPORTED_REV) {
		dev_err(data->dev,
			"WL: 0x%04x < min supported version 0x%04x\n",
			rev, WL_FPGA_MIN_SUPPORTED_REV);
		rc = -EFAULT;
	}

unlock_l2r:
	mutex_unlock(&data->l2r_lock);

exit:
	return rc;
}

static int l2r_fpga_ctrl_reg(struct ml_fpga_io *io, bool on,
			     enum ml_6dof_mode mode, bool cal_en)
{
	struct fpga_data *data = io_2_pdata(io);
	u32 val = 0;
	int rc;

	dev_dbg(data->dev, "set power %s, mode = %s, cal = %d\n",
		emt_mode_to_str(mode), on ? "on" : "off", cal_en);

	mutex_lock(&data->l2r_lock);
	if (on)
		val |= L2R_EMT_CTRL_REG_EN;

	if (mode == EMT_MODE_ADC)
		val |= L2R_EMT_CTRL_REG_MODE;

	if (cal_en) {
		val |= L2R_EMT_CTRL_REG_CAL_ENABLE;
		val |= L2R_EMT_CTRL_REG_CAL;
	} else {
		val |= L2R_EMT_CTRL_REG_CAL_DISABLE;
	}

	rc = l2r_write(data, L2R_CMD_EMT_CTRL_REG_WRITE, 0, &val, 1);
	if (rc)
		dev_err(data->dev, "failed to program l2r ctrl reg, %d\n", rc);
	mutex_unlock(&data->l2r_lock);

	return rc;
}

static int l2r_fpga_set_freq(struct ml_fpga_io *io, u8 totem, u8 idx)
{
	struct fpga_data *data = io_2_pdata(io);
	u8 offset;
	int rc;

	if (idx >= EMT_FREQ_NUM || !ML_6DOF_IS_VALID_TOTEM_ID(totem))
		return -EINVAL;

	dev_dbg(data->dev, "totem_%d set freq idx %d\n", totem, idx);

	mutex_lock(&data->l2r_lock);
	offset = (totem == ML_6DOF_TOTEM_1_ID ? 0x00 : 0x03);
	rc = l2r_write(data, L2R_CMD_CORDIC_MEM, offset, &freq_list[idx][0],
		       EMT_AXIS_NUM);

	if (rc) {
		dev_err(data->dev,
			"failed to change receiver %d freq to %d, %d\n",
			totem, idx, rc);
		goto unlock;
	}

	if (totem == ML_6DOF_TOTEM_1_ID)
		data->freq_t1 = idx;
	else
		data->freq_t2 = idx;

unlock:
	mutex_unlock(&data->l2r_lock);
	return rc;
}

static int l2r_fpga_reset_fine_freq(struct ml_fpga_io *io, u8 totem)
{
	struct fpga_data *data = io_2_pdata(io);
	u32 val[3] = {0};
	u8 offset;
	int rc;

	dev_dbg(data->dev, "%s(%d)\n", __func__, totem);

	mutex_lock(&data->l2r_lock);
	offset = (totem == ML_6DOF_TOTEM_1_ID ? 0x06 : 0x09);
	rc = l2r_write(data, L2R_CMD_CORDIC_MEM, offset, val, 3);
	if (rc)
		dev_err(data->dev, "totem %d: failed to reset fine freq, %d\n",
			totem, rc);
	mutex_unlock(&data->l2r_lock);

	return rc;
}

static void l2r_fpga_adjust_freq_work(struct work_struct *work)
{
	struct fpga_data *data = container_of(work, struct fpga_data,
					      l2r_work);
	u8 offset;
	int rc;

	mutex_lock(&data->l2r_lock);
	offset = 0x06;
	rc = l2r_write(data, L2R_CMD_CORDIC_MEM, offset,
		       data->freq_adjustment, FPGA_FREQ_ADJ_LEN);
	if (rc)
		dev_err(data->dev, "failed to adjust freq, %d\n", rc);
	mutex_unlock(&data->l2r_lock);
}

static int l2r_fpga_adjust_freq(struct ml_fpga_io *io, const u32 *val)
{
	struct fpga_data *data = io_2_pdata(io);

	mutex_lock(&data->l2r_lock);
	memcpy(data->freq_adjustment, val, FPGA_FREQ_ADJ_LEN * sizeof(u32));
	mutex_unlock(&data->l2r_lock);
	/*
	 * Discard the previous adjustments if any are not applied yet.
	 * We are interested only in the most recent update.
	 */
	cancel_work_sync(&data->l2r_work);
	queue_work(data->l2r_work_queue, &data->l2r_work);
	return 0;
}

static int l2r_fpga_read_adc(struct ml_fpga_io *io, u32 adc[EMT_AXIS_NUM])
{
	struct fpga_data *data = io_2_pdata(io);
	int rc;

	mutex_lock(&data->l2r_lock);
	rc = l2r_write_hdr(data, L2R_CMD_ADC_READ, 0, EMT_AXIS_NUM);
	if (rc)
		goto unlock_l2r;

	rc = l2r_poll_status(data);
	if (rc)
		goto unlock_l2r;

	rc = l2r_read(data, EMT_AXIS_NUM + 1);
	if (rc)
		goto unlock_l2r;

	adc[EMT_AXIS_X] = data->l2r_rx.buf[EMT_AXIS_X];
	adc[EMT_AXIS_Y] = data->l2r_rx.buf[EMT_AXIS_Y];
	adc[EMT_AXIS_Z] = data->l2r_rx.buf[EMT_AXIS_Z];

unlock_l2r:
	mutex_unlock(&data->l2r_lock);
	return rc;
}

static const struct ml_fpga_ops fpga_ops = {
	.revision          = fpga_revision,
	.set_power         = fpga_set_power,
	.set_mode          = fpga_set_mode,
	.get_freq_num      = fpga_get_freq_num,
	.freq              = fpga_freq,
	.query_freq        = fpga_query_freq,
	.ctrl_reg          = l2r_fpga_ctrl_reg,
	.set_freq          = l2r_fpga_set_freq,
	.rst_fine_freq     = l2r_fpga_reset_fine_freq,
	.adjust_freq       = l2r_fpga_adjust_freq,
	.read_adc          = l2r_fpga_read_adc,
};

static int fpga_init(struct fpga_data *data)
{
	int ret;
	u32 val;
	const char *emcfg_str = NULL;
	struct device_node *np = of_find_node_by_path("/");

	data->emcfg = EM_COIL_M3440REVB;
	if (np) {
		if (of_property_read_string(np, "emcfg", &emcfg_str) ||
		    kstrtou8(emcfg_str, 0, &data->emcfg))
			dev_err(data->dev, "failed to read coil cfg\n");

		of_node_put(np);
	}

	dev_info(data->dev, "coil cfg = %d\n", data->emcfg);
	if (data->emcfg >= EM_COIL_NUM) {
		dev_err(data->dev, "unknown coil cfg = %d\n", data->emcfg);
		ret = -EFAULT;
		goto exit;
	}

	ret = read_reg(data->dev, data->rmap, BP_FPGA_WHOAMI_REG, &val);
	if (ret) {
		dev_err(data->dev, "failed to read HW ID, %d\n", ret);
		ret = -EPROBE_DEFER;
		goto exit;
	}

	if (val != BP_FPGA_WHOAMI_VAL) {
		dev_err(data->dev, "invalid HW id 0x%02x, expected 0x%02x\n",
			val, BP_FPGA_WHOAMI_VAL);
		ret = -ENODEV;
		goto exit;
	}

	ret = sixdof_fpga_io_register(&data->io);
	if (ret)
		dev_err(data->dev, "failed to register BP FPGA, %d\n", ret);

exit:
	return ret;
}

static int fpga_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *i2c_id)
{
	struct fpga_data *cdata;
	int ret;

	dev_info(&i2c->dev, "%s\n", __func__);
	cdata = devm_kzalloc(&i2c->dev, sizeof(struct fpga_data),
			     GFP_KERNEL);
	if (cdata == NULL)
		return -ENOMEM;

	cdata->i2c = i2c;
	cdata->dev = &i2c->dev;
	cdata->io.ops = &fpga_ops;
	cdata->rmap = devm_regmap_init_i2c(i2c, &regmap_cfg);
	if (IS_ERR(cdata->rmap)) {
		ret = PTR_ERR(cdata->rmap);
		dev_err(&i2c->dev, "failed to allocate 8bit regmap, %d\n",
			ret);
		return ret;
	}
	i2c_set_clientdata(i2c, cdata);

	mutex_init(&cdata->l2r_lock);
	cdata->l2r_work_queue = alloc_ordered_workqueue("l2r", WQ_HIGHPRI);
	if (!cdata->l2r_work_queue) {
		ret = -ENOMEM;
		goto destroy_mutex;
	}
	INIT_WORK(&cdata->l2r_work, l2r_fpga_adjust_freq_work);

	ret = fpga_init(cdata);
	if (ret) {
		dev_err(&i2c->dev, "failed to init FPGA, %d\n", ret);
		goto destroy_workqueue;
	}

	return 0;

destroy_workqueue:
	destroy_workqueue(cdata->l2r_work_queue);

destroy_mutex:
	mutex_destroy(&cdata->l2r_lock);

	return ret;
}

static int fpga_i2c_remove(struct i2c_client *i2c)
{
	struct fpga_data *data = i2c_get_clientdata(i2c);

	dev_info(&i2c->dev, "%s\n", __func__);
	sixdof_fpga_io_unregister();
	cancel_work_sync(&data->l2r_work);
	destroy_workqueue(data->l2r_work_queue);
	mutex_destroy(&data->l2r_lock);
	i2c_set_clientdata(i2c, NULL);

	return 0;
}

static const struct i2c_device_id fpga_i2c_client_id[] = {
	{ "fpga", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, fpga_i2c_client_id);

static const struct of_device_id fpga_i2c_of_match[] = {
	{ .compatible = "ml,bp-fpga-i2c", },
	{ }
};
MODULE_DEVICE_TABLE(of, fpga_i2c_of_match);

static struct i2c_driver fpga_i2c_driver = {
	.driver = {
		.name = "fpga",
		.of_match_table = of_match_ptr(fpga_i2c_of_match),
	},
	.probe = fpga_i2c_probe,
	.remove = fpga_i2c_remove,
	.id_table = fpga_i2c_client_id,
};

module_i2c_driver(fpga_i2c_driver);

MODULE_DESCRIPTION("Magic Leap Belt Pack FPGA I2C driver");
MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_LICENSE("GPL v2");

