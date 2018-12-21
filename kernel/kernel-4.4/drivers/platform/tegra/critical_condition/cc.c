/*
 * drivers/platform/tegra/cc.c
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#define pr_fmt(fmt) "cc: " fmt

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <soc/tegra/pmc.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <asm/cpu.h>
#include <asm/cputype.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/pm_qos.h>
#include <linux/string.h>

#include "cc.h"

/* Array of final cc_throt caps for each clk */
struct cc_throt_freqs *cc_freqs_table;
struct tegra_bwmgr_client *emc_throt_handle;
struct pm_qos_request gpu_cap;

struct crtlcond_platform_data cc_pdata;

#define CAP_TBL_CAP_FREQ(index)	(cc_freqs_table[index].cap_freq)
#define CAP_TBL_CAP_TYPE(index) (cc_freqs_table[index].type)

#define THROT_TBL_IDX(row, col)		(((row) * \
					cc_pdata.cc_throt->num_cap_clks) \
					+ (col))
#define THROT_VAL(tbl, row, col)	(tbl)[(THROT_TBL_IDX(row, col))]

static void gamedata_ram_read_write(void)
{
	char *buf;
	size_t len = cc_pdata.read_write_bytes;
	size_t retlen = 0;
	int ret = 0;

	retlen = gamedata_cvt_read(&len, &buf);
	if (!retlen) {
		pr_debug("crtcl_cond: Data not available in 'GameData RAM'\n");
		return;
	}
	pr_debug("crtcl_cond: writing Data into FRAM\n");
	ret = qspi_write(START_ADDRESS0, retlen, &len, buf);
	if (ret) {
		pr_info("FRAM driver is not up yet\n");
		return;
	}
}

static int crtcl_cond_reboot_cb(struct notifier_block *nb,
			      unsigned long event, void *unused)
{
	if (!(cc_pdata.flags & CC_PAGE_WRITE_STARTED)) {
		pr_debug("crtcl_cond: No DATA in GameData RAM, skip write\n");
		return NOTIFY_DONE;
	}
	gamedata_ram_read_write();

	return NOTIFY_DONE;
}

static struct notifier_block crtcl_cond_reboot_notifier = {
	.notifier_call = crtcl_cond_reboot_cb,
};

static int game_data_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int game_data_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t game_data_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	gamedata_cvt_write(buf, count);
	cc_pdata.flags |= CC_PAGE_WRITE_STARTED;

	if (!cc_pdata.cc_timer_started) {
		mod_timer(&cc_pdata.cc_timer,
				jiffies + cc_pdata.cc_timer_timeout_jiffies);
		cc_pdata.cc_timer_started = TRUE;
	}

	return count;
}

static const struct file_operations game_data_fops = {
	.owner		= THIS_MODULE,
	.write		= game_data_write,
	.open		= game_data_open,
	.release	= game_data_release,
};

static struct miscdevice gd_miscdev = {
	.name   = "gamedata",
	.fops   = &game_data_fops,
};

static DEFINE_PER_CPU(unsigned long, max_cpu_rate);
/* Apply thermal constraints on all policy updates */
static int tegra_throttle_policy_notifier(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned long cpu_max;

	if (event != CPUFREQ_ADJUST)
		return 0;

	cpu_max = per_cpu(max_cpu_rate, policy->cpu);

	if (policy->max > cpu_max)
		cpufreq_verify_within_limits(policy, 0, cpu_max);

	return 0;
}

static struct notifier_block tegra_throttle_cpufreq_nb = {
	.notifier_call = tegra_throttle_policy_notifier,
};

static void cc_throttle_update_cpu_cap(unsigned long rate)
{
	int cpu;

	/* Hz to Khz for cpufreq */
	rate = rate / 1000;

	for_each_present_cpu(cpu) {
		per_cpu(max_cpu_rate, cpu) = rate;
		pr_debug("cc_throttle: cpu=%d max_rate=%lu\n", cpu, rate);
		cpufreq_update_policy(cpu);
	}
}

static void cc_throttle_set_cap_clk(unsigned long cap_rate,
					int cap_clk_index)
{
	unsigned long max_rate = NO_CAP;
	unsigned int type = CAP_TBL_CAP_TYPE(cap_clk_index);

	/* Khz to Hz for clk_set_rate */
	if (cap_rate != NO_CAP)
		max_rate = cap_rate * 1000UL;

	if (CAP_TBL_CAP_FREQ(cap_clk_index) == max_rate)
		return;

	switch (type) {
	case THROT_MCPU:
	case THROT_BCPU:
		cc_throttle_update_cpu_cap(max_rate);
		CAP_TBL_CAP_FREQ(cap_clk_index) = max_rate;
		break;
	case THROT_EMC:
		if (!IS_ERR_OR_NULL(emc_throt_handle)) {
			tegra_bwmgr_set_emc(emc_throt_handle, max_rate,
						TEGRA_BWMGR_SET_EMC_CAP);
			CAP_TBL_CAP_FREQ(cap_clk_index) = max_rate;
		}
		break;
	case THROT_GPU:
		if (cap_rate == NO_CAP)
			cap_rate = PM_QOS_GPU_FREQ_MAX_DEFAULT_VALUE;
		pm_qos_update_request(&gpu_cap, cap_rate);
		CAP_TBL_CAP_FREQ(cap_clk_index) = max_rate;
		break;
	default:
		break;
	}
}

static int
cc_throttle_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long cur_state)
{
	int i, ret = 0;
	char event_string[20];
	char *envp[2] = { event_string, NULL };

	gamedata_ram_read_write();
	if (cc_pdata.cc_throt->cur_state == cur_state)
		goto out;

	cc_pdata.cc_throt->cur_state = cur_state;

	for (i = 0; i < cc_pdata.cc_throt->num_cap_clks; i++) {
		unsigned long cap_rate = NO_CAP;
		unsigned long cur_cap;

		cur_cap = THROT_VAL(cc_pdata.cc_throt->throt_table,
					cc_pdata.cc_throt->cur_state - 1, i);
		if (cur_cap < cap_rate)
			cap_rate = cur_cap;
		cc_throttle_set_cap_clk(cap_rate, i);
	}
	sprintf(event_string, "THERMAL_STATUS=%lu", cur_state);
	ret = kobject_uevent_env(&cdev->device.kobj, KOBJ_CHANGE, envp);
out:
	return ret;
}

static int
cc_throttle_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *max_state)
{
	*max_state = cc_pdata.cc_throt->throt_table_size /
					cc_pdata.cc_throt->num_cap_clks;
	return 0;
}

static int
cc_throttle_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *cur_state)
{
	*cur_state = cc_pdata.cc_throt->cur_state;
	return 0;
}

static struct thermal_cooling_device_ops cc_throttle_cooling_ops = {
	.get_max_state = cc_throttle_get_max_state,
	.get_cur_state = cc_throttle_get_cur_state,
	.set_cur_state = cc_throttle_set_cur_state,
};

void cc_periodic_save_cvt_data_work(struct work_struct *work)
{
	gamedata_ram_read_write();
}

static void cc_periodic_save_cvt_data_timer(unsigned long _data)
{
	if (!(cc_pdata.flags & CC_PAGE_WRITE_STARTED) &&
				(cc_pdata.last_reset_status != WDT_TIMEOUT)) {
		pr_debug("crtcl_cond: No DATA in GameData RAM, skip write\n");
		return;
	}

	schedule_work(&cc_pdata.cc_work);
	cc_pdata.last_reset_status = FALSE;
	mod_timer(&cc_pdata.cc_timer,
			jiffies + cc_pdata.cc_timer_timeout_jiffies);
}

static irqreturn_t gpio_cc_under_volt_irq(int irq, void *arg)
{
	if (!(cc_pdata.flags & CC_PAGE_WRITE_STARTED)) {
		pr_debug("crtcl_cond: No DATA in GameData RAM, skip write\n");
		return IRQ_HANDLED;
	}
	schedule_work(&cc_pdata.cc_work);

	return IRQ_HANDLED;
}

static int parse_throttle_table(struct device *dev, struct device_node *np)
{
	u32 len;
	int ret = 0;

	if (!of_get_property(np, "throttle_table", &len)) {
		dev_err(dev, "%s: No throttle_table?\n", np->full_name);
		ret = -EINVAL;
		goto err;
	}

	len = len / sizeof(u32);

	if ((len % cc_pdata.cc_throt->num_cap_clks) != 0) {
		dev_err(dev, "%s: Invalid throttle table length:%d clks:%d\n",
			np->full_name, len, cc_pdata.cc_throt->num_cap_clks);
		goto err;
	}

	cc_pdata.cc_throt->throt_table = devm_kzalloc(dev, sizeof(u32) * len,
					      GFP_KERNEL);
	if (IS_ERR_OR_NULL(cc_pdata.cc_throt->throt_table)) {
		ret = -ENOMEM;
		goto err;
	}

	ret = of_property_read_u32_array(np, "throttle_table",
					 cc_pdata.cc_throt->throt_table, len);
	if (ret) {
		dev_err(dev, "malformed table %s : entries:%d; ret: %d\n",
			np->full_name, len, ret);
		goto err;
	}
	cc_pdata.cc_throt->throt_table_size = len;
err:
	return ret;
}

static int parse_cc_throttle_dt_data(struct device *dev)
{
	size_t size;
	int i, ret = 0;
	const char *str;
	struct device_node *np = of_get_child_by_name(dev->of_node, "cc_cdev");

	if (IS_ERR_OR_NULL(np)) {
		dev_err(dev, "cc_cdev node not found\n");
		return -EINVAL;
	}

	cc_pdata.cc_throt->num_cap_clks = THROT_MAX_CAP_CLKS;
	size = sizeof(*cc_freqs_table) * cc_pdata.cc_throt->num_cap_clks;
	cc_freqs_table = devm_kzalloc(dev, size, GFP_KERNEL);
	if (IS_ERR_OR_NULL(cc_freqs_table)) {
		ret = -ENOMEM;
		goto err;
	}

	/* Populate cap clks used for cc_throttle cdevs */
	for (i = THROT_MCPU; i < cc_pdata.cc_throt->num_cap_clks; i++) {
		CAP_TBL_CAP_FREQ(i) = NO_CAP;
		CAP_TBL_CAP_TYPE(i) = i;
		if (i == THROT_GPU) {
			pm_qos_add_request(&gpu_cap, PM_QOS_GPU_FREQ_MAX,
					   PM_QOS_GPU_FREQ_MAX_DEFAULT_VALUE);
		}
		dev_dbg(dev, "type=%d\n", CAP_TBL_CAP_TYPE(i));
	}

	if (of_property_read_string(np, "cdev-type", &str) == 0) {
		cc_pdata.cc_throt->cdev_type = devm_kzalloc(dev,
					strlen(str) + 1, GFP_KERNEL);
		if (IS_ERR_OR_NULL(cc_pdata.cc_throt->cdev_type)) {
			ret = -ENOMEM;
			goto err;
		}
		cc_pdata.cc_throt->cdev_type = (char *)str;
	}

	ret = parse_throttle_table(dev, np);
	if  (ret)
		goto err;

err:
	return ret;
}

int cc_throttle_init(struct platform_device *pdev)
{
	int ret = 0;

	cc_pdata.cc_throt = devm_kzalloc(&pdev->dev, sizeof(struct cc_throttle),
								 GFP_KERNEL);
	if (IS_ERR_OR_NULL(cc_pdata.cc_throt)) {
		ret = -ENOMEM;
		goto err;
	}
	emc_throt_handle = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_THERMAL_CAP);
	if (IS_ERR_OR_NULL(emc_throt_handle))
		dev_err(&pdev->dev, "No valid bwmgr handle for emc\n");

	ret = parse_cc_throttle_dt_data(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Platform data parse failed.\n");
		return ret;
	}
err:
	return ret;
}

static int crtcl_cond_probe(struct platform_device *pdev)
{
	int ret, reset_reason;

	pdev->dev.platform_data = &cc_pdata;
	ret = cc_throttle_init(pdev);
	if (ret) {
		dev_err(&pdev->dev, "cc_throttling failed\n");
		return ret;
	}

	cpufreq_register_notifier(&tegra_throttle_cpufreq_nb,
					CPUFREQ_POLICY_NOTIFIER);

	ret = gamedata_cvt_probe(pdev);
	if (ret) {
		dev_err(&pdev->dev, "crtcl_cond: gamedata_cvt_probe failed\n");
		return ret;
	}

	cc_pdata.flags &= ~CC_PAGE_WRITE_STARTED;

	ret = register_reboot_notifier(&crtcl_cond_reboot_notifier);
	if (ret) {
		dev_err(&pdev->dev, "crtcl_cond: probe failed\n");
		return ret;
	}

	cc_pdata.cc_throt->cdev = thermal_cooling_device_register(
						cc_pdata.cc_throt->cdev_type,
						NULL,
						&cc_throttle_cooling_ops);
	if (IS_ERR_OR_NULL(cc_pdata.cc_throt->cdev)) {
		cc_pdata.cc_throt->cdev = NULL;
		dev_err(&pdev->dev, "coolding-device registration failed\n");
	}

	INIT_WORK(&cc_pdata.cc_work, cc_periodic_save_cvt_data_work);

	setup_timer(&cc_pdata.cc_timer, cc_periodic_save_cvt_data_timer, 0);
	cc_pdata.cc_timer_timeout_jiffies = msecs_to_jiffies(cc_pdata.
						cc_timer_timeout * 1000);
	ret = misc_register(&gd_miscdev);
	if (ret)
		dev_err(&pdev->dev, "cannot register miscdev (err=%d)\n", ret);

	if (gpio_is_valid(cc_pdata.under_volt_gpio)) {
		cc_pdata.under_volt_irq = gpio_to_irq(cc_pdata.under_volt_gpio);
		if (cc_pdata.under_volt_irq <= 0) {
			dev_err(&pdev->dev, "gpio_to_irq failed, err:%d\n",
				cc_pdata.under_volt_irq);
		} else {
			ret = devm_request_irq(&pdev->dev,
				cc_pdata.under_volt_irq, gpio_cc_under_volt_irq,
				IRQF_TRIGGER_RISING, "cc_under_volt_irq",
				NULL);
			if (ret < 0) {
				dev_err(&pdev->dev,
				"Unable to claim irq for under-volt\n");
			}
			dev_dbg(&pdev->dev,
			"registered critical-condition under-volt-irq: %d\n",
				cc_pdata.under_volt_irq);
		}
	}

	reset_reason = tegra_reset_reason_status();
	if (reset_reason == WDT_TIMEOUT) {
		pr_debug("%s: Reset Reason is Watchdog Timeout\n", __func__);
		cc_pdata.last_reset_status = reset_reason;
	}
	cc_periodic_save_cvt_data_timer((unsigned long int)&cc_pdata);

	return 0;
}

static int crtcl_cond_remove(struct platform_device *pdev)
{
	misc_deregister(&gd_miscdev);
	del_timer(&cc_pdata.cc_timer);
	cancel_work_sync(&cc_pdata.cc_work);
	unregister_reboot_notifier(&crtcl_cond_reboot_notifier);

	return 0;
}

static const struct of_device_id of_crtcl_cond_match[] = {
	{
		.compatible = "nvidia,tegra186-crtcl_cond",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, of_crtcl_cond_match);

static struct platform_driver crtcl_cond_driver = {
	.probe		= crtcl_cond_probe,
	.remove		= crtcl_cond_remove,
	.driver		= {
		.name	= "crtlcond",
		.owner	= THIS_MODULE,
		.of_match_table = of_crtcl_cond_match,
	},
};

static int __init crtcl_cond_driver_init(void)
{
	return platform_driver_register(&crtcl_cond_driver);
}
late_initcall(crtcl_cond_driver_init);

static int __init crtlcond_init(struct reserved_mem *rmem)
{
	cc_pdata.cvt_mem_address = rmem->base;
	cc_pdata.cvt_mem_size = rmem->size;

	return 0;
}
RESERVEDMEM_OF_DECLARE(tegra_crtlcond, "nvidia,gamedata", crtlcond_init);

