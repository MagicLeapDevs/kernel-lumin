/*
 * Copyright (c) 2016 - 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/slab.h>

#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/cpu_emc.h>

#define CPU_EMC_TABLE_NUM		2

#define CPU_EMC_TABLE_SRC_DEFAULT	0
#define CPU_EMC_TABLE_SRC_DT		1

struct cpu_emc {
	struct tegra_bwmgr_client *bwmgr;
	unsigned long max_rate;
	u32 *cpu_emc_table[CPU_EMC_TABLE_NUM];
	int cpu_emc_table_size[CPU_EMC_TABLE_NUM];
	int cpu_emc_table_src;
};

static struct cpu_emc cpemc;
static struct kobject *cpu_emc_kobj;

static u32 default_emc_cpu_table[] = {
	/* cpu > 0MHz, emc 0MHz (Min) */
	0, 0,
	/* cpu > 275MHz, emc 50MHz */
	275000, 50000,
	/* cpu > 500MHz, emc 100MHz */
	500000, 100000,
	/* cpu > 725MHz, emc 200MHz */
	725000, 200000,
	/* cpu > 925MHz, emc 400MHz */
	975000, 400000,
	/* cpu > 1.3GHz, emc xGHz (Max) */
	1300000, INT_MAX,
};

void set_cpu_to_emc_freq(u32 cpu_freq)
{
	int src = cpemc.cpu_emc_table_src;
	int size = cpemc.cpu_emc_table_size[src];
	u32 *table = cpemc.cpu_emc_table[src];
	unsigned long emc_freq = 0;
	int i;

	for (i = 0; i < size; i += 2) {
		if (cpu_freq < table[i])
			break;
	}

	if (i)
		emc_freq = min(cpemc.max_rate, table[i - 1] * 1000UL);

	tegra_bwmgr_set_emc(cpemc.bwmgr, emc_freq,
		TEGRA_BWMGR_SET_EMC_FLOOR);

	pr_debug("cpu freq(kHz) %u emc_freq(KHz) %lu, using CPU/EMC table %d\n",
		 cpu_freq, emc_freq / 1000, src);
}

static int register_with_emc_bwmgr(void)
{
	struct tegra_bwmgr_client *bwmgr;
	int ret = 0;

	bwmgr = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_CPU_0);
	if (IS_ERR_OR_NULL(bwmgr)) {
		pr_err("emc bw manager registration failed\n");
		ret = -ENODEV;
		goto err_out;
	}
	cpemc.bwmgr = bwmgr;
err_out:
	return ret;
}

static u32 *cpufreq_emc_table_get(int *table_size)
{
	int freqs_num, ret = 0;
	u32 *freqs = NULL;
	struct device_node *np = NULL;
	const char *propname = "emc-cpu-limit-table";

	/* Find emc scaling node */
	np = of_get_scaling_node("emc-scaling-data");
	if (!np)
		return ERR_PTR(-ENODATA);

	/* Read frequency table */
	if (!of_find_property(np, propname, &freqs_num)) {
		pr_err("%s: %s is not found\n", __func__, propname);
		ret = -ENODATA;
		goto _out;
	}

	/* must have even entries */
	if (!freqs_num || (freqs_num % (sizeof(*freqs) * 2))) {
		pr_err("%s: invalid %s size %d\n", __func__, propname,
			freqs_num);
		ret = -ENODATA;
		goto _out;
	}

	freqs = kzalloc(freqs_num, GFP_KERNEL);
	if (!freqs) {
		ret = -ENOMEM;
		goto _out;
	}

	freqs_num /= sizeof(*freqs);
	if (of_property_read_u32_array(np, propname, freqs, freqs_num)) {
		pr_err("%s: failed to read %s\n", __func__, propname);
		ret = -ENODATA;
		goto _out;
	}

	of_node_put(np);
	*table_size = freqs_num;
	return freqs;

_out:
	kfree(freqs);
	of_node_put(np);
	return ERR_PTR(ret);
}

static int cpu_emc_tbl_from_dt(void)
{
	int src = CPU_EMC_TABLE_SRC_DT;

	cpemc.cpu_emc_table[src] =
		cpufreq_emc_table_get(&cpemc.cpu_emc_table_size[src]);
	if (IS_ERR(cpemc.cpu_emc_table[src])) {
		pr_warn("%s: No cpu emc table in DT, using default one\n",
			__func__);
		cpemc.cpu_emc_table_src = CPU_EMC_TABLE_SRC_DEFAULT;
	} else {
		cpemc.cpu_emc_table_src = CPU_EMC_TABLE_SRC_DT;
	}

	cpemc.max_rate = tegra_bwmgr_get_max_emc_rate();

	/* Initialize the default cpu emc table */
	src = CPU_EMC_TABLE_SRC_DEFAULT;
	cpemc.cpu_emc_table[src] = default_emc_cpu_table;
	cpemc.cpu_emc_table_size[src] = ARRAY_SIZE(default_emc_cpu_table);

	return 0;
}

static ssize_t cpu_emc_table_src_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cpemc.cpu_emc_table_src);
}

static ssize_t cpu_emc_table_src_store(struct kobject *kobj,
				       struct kobj_attribute *attr,
				       const char *buf, size_t n)
{
	u32 *table_dt = cpemc.cpu_emc_table[CPU_EMC_TABLE_SRC_DT];
	int src;

	if (kstrtouint(buf, 0, &src))
		return -EINVAL;

	if (src == CPU_EMC_TABLE_SRC_DT && !IS_ERR(table_dt))
		cpemc.cpu_emc_table_src = CPU_EMC_TABLE_SRC_DT;
	else
		cpemc.cpu_emc_table_src = CPU_EMC_TABLE_SRC_DEFAULT;

	pr_debug("cpu_emc_table_src was updated to %d\n",
		 cpemc.cpu_emc_table_src);

	return n;
}

static const struct kobj_attribute cpu_emc_attr =
	__ATTR(table_src, 0644, cpu_emc_table_src_show, cpu_emc_table_src_store);

int enable_cpu_emc_clk(void)
{
	int ret = 0;

	ret = register_with_emc_bwmgr();
	if (ret)
		goto err_out;

	ret = cpu_emc_tbl_from_dt();
	if (ret) {
		tegra_bwmgr_unregister(cpemc.bwmgr);
		goto err_out;
	}

	cpu_emc_kobj = kobject_create_and_add("tegra_cpu_emc", kernel_kobj);
	if (!cpu_emc_kobj) {
		pr_err("%s: failed to create tegra_cpu_emc kobj\n", __func__);
		ret = -EINVAL;
		goto err_out;
	}

	ret = sysfs_create_file(cpu_emc_kobj, &cpu_emc_attr.attr);
	if (ret) {
		pr_err("%s, failed to create sysfs attr for %s\n",
		       __func__, cpu_emc_attr.attr.name);
		goto err_out;
	}

err_out:
	return ret;
}

void disable_cpu_emc_clk(void)
{
	sysfs_remove_file(cpu_emc_kobj, &cpu_emc_attr.attr);
	kobject_put(cpu_emc_kobj);
	kfree(cpemc.cpu_emc_table);
	tegra_bwmgr_unregister(cpemc.bwmgr);
}
