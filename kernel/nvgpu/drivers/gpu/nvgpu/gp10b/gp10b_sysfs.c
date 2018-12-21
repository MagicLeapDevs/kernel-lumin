/*
 * GP10B specific sysfs files
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/platform_device.h>

#include "gk20a/gk20a.h"
#include "gk20a/pmu_gk20a.h"
#include "gp10b_sysfs.h"

#include <nvgpu/hw/gp10b/hw_gr_gp10b.h>

#define ROOTRW (S_IRWXU|S_IRGRP|S_IROTH)

static ssize_t ecc_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	u32 ecc_mask;
	u32 err = 0;

	err = sscanf(buf, "%d", &ecc_mask);
	if (err == 1) {
		err = g->ops.pmu.send_lrf_tex_ltc_dram_overide_en_dis_cmd
			(g, ecc_mask);
		if (err)
			dev_err(dev, "ECC override did not happen\n");
	} else
		return -EINVAL;
	return count;
}

static ssize_t ecc_enable_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return sprintf(buf, "ecc override =0x%x\n",
			g->ops.gr.get_lrf_tex_ltc_dram_override(g));
}

static DEVICE_ATTR(ecc_enable, ROOTRW, ecc_enable_read, ecc_enable_store);


static ssize_t czf_bypass_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val >= 4)
		return -EINVAL;

	g->gr.czf_bypass = val;

	return count;
}

static ssize_t czf_bypass_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return sprintf(buf, "%d\n", g->gr.czf_bypass);
}

static DEVICE_ATTR(czf_bypass, ROOTRW, czf_bypass_read, czf_bypass_store);

static ssize_t pd_max_batches_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val >= 64)
		return -EINVAL;

	g->gr.pd_max_batches = val;

	return count;
}

static ssize_t pd_max_batches_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return sprintf(buf, "%d\n", g->gr.pd_max_batches);
}

static DEVICE_ATTR(pd_max_batches, ROOTRW, pd_max_batches_read, pd_max_batches_store);

static int write_gfxp_wfi_timeout_count(struct gk20a *g, u32 val)
{
	gk20a_writel(g, gr_fe_gfxp_wfi_timeout_r(),
		gr_fe_gfxp_wfi_timeout_count_f(val));

	return 0;
}

static ssize_t gfxp_wfi_timeout_count_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	struct gr_gk20a *gr = &g->gr;
	unsigned long val = 0;
	int err = -1;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val >= 100*1000*1000) /* 100ms @ 1Ghz */
		return -EINVAL;

	gr->gfxp_wfi_timeout_count = val;

	if (!g->power_on)
		return count;

	err = gk20a_busy(g);
	if (err)
		return err;
	gr_gk20a_elpg_protected_call(g,
		write_gfxp_wfi_timeout_count(g, val));
	gk20a_idle(g);

	return count;
}

static ssize_t gfxp_wfi_timeout_count_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	struct gr_gk20a *gr = &g->gr;
	u32 val = gr->gfxp_wfi_timeout_count;

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static DEVICE_ATTR(gfxp_wfi_timeout_count, ROOTRW,
		gfxp_wfi_timeout_count_read, gfxp_wfi_timeout_count_store);

static ssize_t ldiv_slowdown_factor_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0) {
		dev_err(dev, "parse error for input SLOWDOWN factor\n");
		return -EINVAL;
	}

	if (val >= SLOWDOWN_FACTOR_FPDIV_BYMAX) {
		dev_err(dev, "Invalid SLOWDOWN factor\n");
		return -EINVAL;
	}

	if (val == platform->ldiv_slowdown_factor)
		return count;

	if (!g->power_on) {
		platform->ldiv_slowdown_factor = val;
	} else {
		err = gk20a_busy(g);
		if (err)
			return -EAGAIN;

		platform->ldiv_slowdown_factor = val;

		if (g->ops.pmu.pmu_pg_init_param)
			g->ops.pmu.pmu_pg_init_param(g,
						PMU_PG_ELPG_ENGINE_ID_GRAPHICS);

		gk20a_idle(g);
	}

	return count;
}

static ssize_t ldiv_slowdown_factor_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", platform->ldiv_slowdown_factor);
}

static DEVICE_ATTR(ldiv_slowdown_factor, ROOTRW,
			ldiv_slowdown_factor_read, ldiv_slowdown_factor_store);


void gp10b_create_sysfs(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int error = 0;

	g->gr.czf_bypass = gr_gpc0_prop_debug1_czf_bypass_init_v();

	error |= device_create_file(dev, &dev_attr_ecc_enable);
	error |= device_create_file(dev, &dev_attr_czf_bypass);
	error |= device_create_file(dev, &dev_attr_pd_max_batches);
	error |= device_create_file(dev, &dev_attr_gfxp_wfi_timeout_count);
	error |= device_create_file(dev, &dev_attr_ldiv_slowdown_factor);
	if (error)
		dev_err(dev, "Failed to create sysfs attributes!\n");
}

void gp10b_remove_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_ecc_enable);
	device_remove_file(dev, &dev_attr_czf_bypass);
	device_remove_file(dev, &dev_attr_pd_max_batches);
	device_remove_file(dev, &dev_attr_gfxp_wfi_timeout_count);
	device_remove_file(dev, &dev_attr_ldiv_slowdown_factor);
}
