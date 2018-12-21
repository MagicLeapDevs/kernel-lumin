/* Copyright (c) 2016-2017, Magic Leap, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * This driver exposes basic capabilities to power up and down the wearable
 * from userspace, as well as a custom callback for the wearable IRQ.
 */
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>

#include <misc/ml_wearable.h>

/* Number of ms to wait between toggling regulator OFF/ON */
#define HARD_RESET_DELAY_MS			200

/* Number of ms to wait between toggling GPIO OFF/ON */
#define SOFT_RESET_DELAY_MS			200

/* Number of ms to wait between turning ON regulator and enabling GPIO */
#define REG_ENABLE_GPIO_ENABLE_DELAY_MS		200

/* Number of ms to wait between disabling GPIO and turning OFF regulator */
#define GPIO_DISABLE_REG_DISABLE_DELAY_MS	200

enum wearable_power_state {
	ML_WEARABLE_POWER_STATE_NOT_POWERED = 0,
	ML_WEARABLE_POWER_STATE_POWERED,
	ML_WEARABLE_POWER_STATE_MAX
};

enum wearable_reset_type {
	ML_WEARABLE_HARD_RESET = 0,
	ML_WEARABLE_SOFT_RESET,
	ML_WEARABLE_NO_RESET
};

/* interrupt to wearable */
enum ap_int_status {
	ML_WEARABLE_AP_INT_DISABLED = 0,
	ML_WEARABLE_AP_INT_ENABLED
};

enum audio_hsj_mux {
	ML_WEARABLE_HSJ_MUX_VSYNC = 0,
	ML_WEARABLE_HSJ_MUX_UART,
	ML_WEARABLE_HSJ_MUX_MAX
};

enum audio_hsj_rx_pin {
	ML_WEARABLE_HSJ_RX_UART = 0,
	ML_WEARABLE_HSJ_RX_EXTSYNC,
	ML_WEARABLE_HSJ_RX_MAX
};

struct ml_wearable_platform_info {
	struct device *dev;
	enum wearable_power_state power_state;
	enum ap_int_status int_status;
	enum wearable_reset_type reset;
	enum wearable_power_state new_power_state;
	enum audio_hsj_mux headset_signal;
	enum audio_hsj_rx_pin rx_pin_state;
	struct regulator *reg_pwr;
	bool reg_enable;
	int vsync_irq;   /* wearable to belt pack interrupt */
	int ap_int_gpio;
	int on_off_gpio;
	int vsync_mux_en_gpio;
	int invert_gpio_on_off;
	unsigned char vsync_interrupt_level;  /* for factory testing */
	int extsync_gpio;
	int uart_tx_gpio;
	int flt_irq_gpio;
	bool uart_tx_initialized;
	u32 max_curr;
	struct pinctrl *extsync_pinctrl;
	struct pinctrl_state *uart_rx_gpio_config;
	struct pinctrl_state *extsync_gpio_config;
	struct mutex mutex_lock;
	struct workqueue_struct *work_q;
	struct work_struct work_reset;
	struct work_struct work_power_state;
	ktime_t time_power_on;
};

/* Our queue of external GPIO events */
static struct extsync_event_log_struct extsync_event_log;

/*
 * This variable is the wearable IRQ handler which
 * will be called any time the wearable IRQ GPIO is changed.
 */
static void (*wearable_irq_custom_callback)(void);

/*
 * This function handles changing the wearable IRQ handler's
 * custom callback. The function is expected to handle its own
 * synchronization. The driver/caller should also ensure that
 * they unregister properly. Care should be taken that there is
 * only one driver using this. (This is a safe assumption as plan
 * of record only has this IRQ being used for time synchronization.)
 */
void ml_wearable_register_wearable_irq_custom_callback(void (*callback)(void))
{
	wearable_irq_custom_callback = callback;
}
EXPORT_SYMBOL(ml_wearable_register_wearable_irq_custom_callback);

/*
 * This function handles clearing the wearable IRQ handler's
 * custom callback.
 */
void ml_wearable_deregister_wearable_irq_custom_callback(void)
{
	wearable_irq_custom_callback = NULL;
}
EXPORT_SYMBOL(ml_wearable_deregister_wearable_irq_custom_callback);


/*
 * This variable is the extsync (audio jack) IRQ handler which
 * will be called any time the wearable IRQ GPIO is changed.
 */
static int (*extsync_irq_custom_callback)(struct extsync_event *);

void ml_wearable_register_extsync_irq_custom_callback(
	int (*callback)(struct extsync_event *))
{
	extsync_irq_custom_callback = callback;
	/* Ensure callback assignment (checkpatch won't allow volatile) */
	smp_wmb();
}
EXPORT_SYMBOL(ml_wearable_register_extsync_irq_custom_callback);

void ml_wearable_deregister_extsync_irq_custom_callback(void)
{
	extsync_irq_custom_callback = NULL;
	/* Ensure callback assignment (checkpatch won't allow volatile) */
	smp_wmb();
}
EXPORT_SYMBOL(ml_wearable_deregister_extsync_irq_custom_callback);


/*
 * This function handles changing the ON/OFF GPIO to wearable,
 * which can be from the attributes hard_reset, soft_reset, or
 * or directly changing attribute power_state.
 * The "reset" parameter determines whether the regulator is
 * toggled.
 * It will set the ap_interrupt gpio low before powering down
 * the wearable.
 * Function must be called with mutex_lock obtained.
 */
static int set_power_state_helper(struct device *dev,
		enum wearable_power_state new_state,
		enum wearable_reset_type reset)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	int rc = 0;
	bool state_changed = (new_state != info->power_state);

	switch (new_state) {
	case ML_WEARABLE_POWER_STATE_NOT_POWERED:
		if (state_changed)
			disable_irq(gpio_to_irq(info->vsync_irq));
		gpio_set_value(info->ap_int_gpio, 0);
		info->int_status = ML_WEARABLE_AP_INT_DISABLED;
		info->power_state = new_state;
		gpio_set_value(info->on_off_gpio, info->invert_gpio_on_off);
		dev_info(dev, "set gpio_on_off to %d",
				info->invert_gpio_on_off);
		if (reset == ML_WEARABLE_HARD_RESET && info->reg_enable) {
			msleep(GPIO_DISABLE_REG_DISABLE_DELAY_MS);
			rc = regulator_disable(info->reg_pwr);
			if (rc)
				break;
			info->reg_enable = false;
			dev_info(dev, "regulator disabled");
		}
		break;
	case ML_WEARABLE_POWER_STATE_POWERED:
		if (reset == ML_WEARABLE_HARD_RESET && !info->reg_enable) {
			rc = regulator_enable(info->reg_pwr);
			if (rc)
				break;
			info->reg_enable = true;
			dev_info(dev, "regulator enabled");
			msleep(REG_ENABLE_GPIO_ENABLE_DELAY_MS);
		}
		gpio_set_value(info->on_off_gpio,
				1 - info->invert_gpio_on_off);
		dev_info(dev, "set gpio_on_off to %d",
				1 - info->invert_gpio_on_off);
		if (state_changed)
			enable_irq(gpio_to_irq(info->vsync_irq));
		info->power_state = new_state;
		info->time_power_on = ktime_get_boottime();
		break;
	case ML_WEARABLE_POWER_STATE_MAX:
		break;
	}

	if (rc < 0)
		dev_err(dev, "unable to set power state to %d (rc %d)",
				new_state, rc);
	else
		dev_info(dev, "power state set to %d", new_state);

	return rc;
}

static int set_power_state(struct device *dev,
		enum wearable_power_state new_state,
		enum wearable_reset_type reset)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	int rc;

	mutex_lock(&info->mutex_lock);
	rc = set_power_state_helper(dev, new_state, reset);
	mutex_unlock(&info->mutex_lock);

	return rc;
}

static void power_state_work_fn(struct work_struct *work)
{
	struct ml_wearable_platform_info *info = container_of(work,
			struct ml_wearable_platform_info, work_power_state);

	set_power_state(info->dev, info->new_power_state, info->reset);
}

static void queue_work_power_state(struct device *dev,
		enum wearable_power_state new_state,
		enum wearable_reset_type reset)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);

	info->new_power_state = new_state;
	info->reset = reset;
	queue_work(info->work_q, &info->work_power_state);
}

static enum wearable_power_state get_power_state(struct device *dev)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	enum wearable_power_state state;

	mutex_lock(&info->mutex_lock);
	state = info->power_state;
	mutex_unlock(&info->mutex_lock);

	return state;
}

/*
 * A hard reset on the wearable unit is performed by cycling power.
 * A soft reset requests a power-down, which is much safer, as it allows
 * the wearable to issue the power-down sequence to the cameras, etc.
 * More effort will be needed to support the hard reset.
 */
static int wearable_reset(struct device *dev, enum wearable_reset_type reset)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	int rc;

	mutex_lock(&info->mutex_lock);
	dev_info(dev, "wearable reset");
	rc = set_power_state_helper(
			dev, ML_WEARABLE_POWER_STATE_NOT_POWERED, reset);
	if (!rc) {
		if (reset == ML_WEARABLE_HARD_RESET)
			msleep(HARD_RESET_DELAY_MS);
		else
			msleep(SOFT_RESET_DELAY_MS);
		rc = set_power_state_helper(
				dev, ML_WEARABLE_POWER_STATE_POWERED, reset);
	}
	mutex_unlock(&info->mutex_lock);

	return rc;
}

static void reset_work_fn(struct work_struct *work)
{
	struct ml_wearable_platform_info *info = container_of(work,
			struct ml_wearable_platform_info, work_reset);

	wearable_reset(info->dev, info->reset);
}

static void queue_work_reset(struct device *dev,
		enum wearable_reset_type reset)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);

	info->reset = reset;
	queue_work(info->work_q, &info->work_reset);
}

static ssize_t hard_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (count > 0)
		queue_work_reset(dev, ML_WEARABLE_HARD_RESET);

	return count;
}
static DEVICE_ATTR_WO(hard_reset);

static ssize_t soft_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (count > 0)
		queue_work_reset(dev, ML_WEARABLE_SOFT_RESET);

	return count;
}
static DEVICE_ATTR_WO(soft_reset);


static ssize_t power_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int new_state;
	int rc;

	rc = kstrtoint(buf, 10, &new_state);
	if (rc == 0 && new_state >= 0 && new_state <
			ML_WEARABLE_POWER_STATE_MAX) {
		queue_work_power_state(dev, new_state,
				ML_WEARABLE_HARD_RESET);
	}
	return count;
}

static ssize_t power_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", get_power_state(dev));
}
static DEVICE_ATTR_RW(power_state);

/*
 * Factory testing will loop various signals back to the interrupt pin.
 * factory_test attribute will indicate whether an interrupt has occurred.
 */
static ssize_t factory_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", info->vsync_interrupt_level);
}
static DEVICE_ATTR_RO(factory_test);

/*
 * power_on_time attribute will display the time when wearable was powered ON.
 */
static ssize_t power_on_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct timespec pwr_on_time;
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);

	pwr_on_time = ktime_to_timespec(info->time_power_on);

	return scnprintf(buf, PAGE_SIZE, "%lu.%09lu\n",
			pwr_on_time.tv_sec, pwr_on_time.tv_nsec);
}
static DEVICE_ATTR_RO(power_on_time);

static int set_ap_int_status(struct device *dev, enum ap_int_status new_status)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);

	mutex_lock(&info->mutex_lock);

	if (ML_WEARABLE_AP_INT_DISABLED == new_status)
		gpio_set_value(info->ap_int_gpio, 0);
	else
		gpio_set_value(info->ap_int_gpio, 1);

	info->int_status = new_status;
	dev_info(dev, "interrupt to wearable set to %d\n", new_status);

	mutex_unlock(&info->mutex_lock);

	return 0;
}

static enum ap_int_status get_ap_int_status(struct device *dev)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	enum ap_int_status status;

	mutex_lock(&info->mutex_lock);
	status = info->int_status;
	mutex_unlock(&info->mutex_lock);

	return status;
}

static ssize_t ap_interrupt_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int new_status;
	int rc;

	rc = kstrtoint(buf, 10, &new_status);

	if (rc == 0 && (new_status == ML_WEARABLE_AP_INT_DISABLED ||
			new_status == ML_WEARABLE_AP_INT_ENABLED))
		set_ap_int_status(dev, new_status);

	return count;
}

static ssize_t ap_interrupt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", get_ap_int_status(dev));
}
static DEVICE_ATTR_RW(ap_interrupt);

static int set_vsync_mux_en(struct device *dev, enum audio_hsj_mux routing)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);

	mutex_lock(&info->mutex_lock);

	gpio_set_value(info->vsync_mux_en_gpio,
		(routing == ML_WEARABLE_HSJ_MUX_VSYNC) ? 0 : 1);
	info->headset_signal = routing;
	dev_dbg(dev, "vsync enable line set to %d\n",
		(routing == ML_WEARABLE_HSJ_MUX_VSYNC) ? 0 : 1);

	mutex_unlock(&info->mutex_lock);

	return 0;
}

static int get_vsync_mux_en_state(struct device *dev)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	enum audio_hsj_mux routing;

	mutex_lock(&info->mutex_lock);
	routing = info->headset_signal;
	mutex_unlock(&info->mutex_lock);

	return routing;
}

static ssize_t vsync_mux_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int new_state;
	int rc;

	rc = kstrtoint(buf, 10, &new_state);
	if (rc == 0 && new_state >= 0 && new_state < ML_WEARABLE_HSJ_MUX_MAX)
		set_vsync_mux_en(dev, new_state);
	return count;
}

static ssize_t vsync_mux_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", get_vsync_mux_en_state(dev));
}
static DEVICE_ATTR_RW(vsync_mux_en);

static int set_extsync_en(struct device *dev, enum audio_hsj_rx_pin routing)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	int err;

	/* If this pin doesn't exist in this config, skip. */
	if (!gpio_is_valid(info->extsync_gpio))
		return 0;

	/* If we are already set to the request value, skip. */
	if (info->rx_pin_state == routing)
		return 0;

	mutex_lock(&info->mutex_lock);

	if (routing == ML_WEARABLE_HSJ_RX_UART) {
		disable_irq_nosync(gpio_to_irq(info->extsync_gpio));
		devm_gpio_free(dev, info->extsync_gpio);
		err = pinctrl_select_state(info->extsync_pinctrl,
			info->uart_rx_gpio_config);
		info->rx_pin_state = routing;

		kfree(extsync_event_log.event_list);
		extsync_event_log.event_list = NULL;
	} else {
		err = devm_gpio_request_one(dev, info->extsync_gpio,
			GPIOF_IN, "wearable_extsync_irq");
		if (err) {
			dev_err(dev, "wearable_extsync_irq GPIO request failed: %d\n",
				err);
			goto exit;
		}
		err = pinctrl_select_state(info->extsync_pinctrl,
			info->extsync_gpio_config);
		gpio_direction_input(info->extsync_gpio);
		enable_irq(gpio_to_irq(info->extsync_gpio));
		info->rx_pin_state = routing;
		if (!extsync_event_log.event_list && !err) {
			extsync_event_log.event_list =
				kmalloc_array(EXTSYNC_EVENT_QUEUE_SIZE,
					sizeof(struct extsync_event),
					GFP_KERNEL);
			if (!extsync_event_log.event_list)
				dev_err(dev, "extsync event log alloc failed.\n");
		}
	}
	if (err) {
		/*
		 * We leave rx_pin_state updated even if there was a late
		 * failure -- seems the lesser of two evils, so when
		 * called again later we have a better 'old' state.
		 */
		dev_err(dev, "pinctrl select failed for: %s\n",
			(routing == ML_WEARABLE_HSJ_RX_UART) ?
			"uart_rx" : "extsync");
		goto exit;
	}

	dev_dbg(dev, "uart_rx/extsync pin usage set to %d\n", routing);
exit:
	mutex_unlock(&info->mutex_lock);
	return err;
}

static int get_extsync_en_state(struct device *dev)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	enum audio_hsj_rx_pin routing;

	routing = info->rx_pin_state;

	return routing;
}

static ssize_t extsync_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int new_state;
	int rc;

	rc = kstrtoint(buf, 10, &new_state);
	if (rc == 0 && new_state >= 0 && new_state < ML_WEARABLE_HSJ_RX_MAX)
		set_extsync_en(dev, new_state);
	return count;
}

static ssize_t extsync_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", get_extsync_en_state(dev));
}
static DEVICE_ATTR_RW(extsync_en);

static ssize_t on_off_gpio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int new_state;
	int rc;
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);

	rc = kstrtoint(buf, 10, &new_state);
	if (rc != 0 || new_state < 0)
		goto fail;

	mutex_lock(&info->mutex_lock);
	gpio_set_value(info->on_off_gpio, !!new_state);
	mutex_unlock(&info->mutex_lock);

	if (!!new_state == !info->invert_gpio_on_off)
		info->time_power_on = ktime_get_boottime();

fail:
	return count;
}

static ssize_t on_off_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			gpio_get_value(info->on_off_gpio));
}
static DEVICE_ATTR_RW(on_off_gpio);

static ssize_t uart_tx_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	int new_state = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &new_state);
	if (rc != 0 || new_state < 0)
		goto fail;
	new_state = !!new_state;

	mutex_lock(&info->mutex_lock);
	if (new_state && !info->uart_tx_initialized) {
		rc = devm_gpio_request_one(dev, info->uart_tx_gpio,
			GPIOF_OUT_INIT_LOW, "uart_tx_gpio");
		if (rc) {
			dev_err(dev, "uart_tx GPIO request failed: %d\n",
				rc);
			mutex_unlock(&info->mutex_lock);
			goto fail;
		}
		gpio_direction_output(info->uart_tx_gpio, 0);
		dev_warn(dev, "UART_TX gpio enabled\n");
		info->uart_tx_initialized = true;
	} else if (!new_state && info->uart_tx_initialized) {
		dev_warn(dev, "UART_TX gpio disabled\n");
		devm_gpio_free(dev, info->uart_tx_gpio);
		info->uart_tx_initialized = false;
	} else
		dev_warn(dev, "%s: ignored write, no change\n",
		 __func__);

	mutex_unlock(&info->mutex_lock);
	return count;
fail:
	dev_err(dev, "%s: goto fail\n", __func__);
	return -EINVAL;
}
static DEVICE_ATTR_WO(uart_tx_en);

static ssize_t uart_tx_val_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	int new_state = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &new_state);
	if (rc != 0 || new_state < 0)
		goto fail;

	new_state = !!new_state;

	mutex_lock(&info->mutex_lock);
	if (!gpio_get_value(info->vsync_mux_en_gpio)) {
		dev_warn(dev, "%s: WARNING: vsync_mux_en off, enabling...\n",
		 __func__);
		gpio_set_value(info->vsync_mux_en_gpio, 1);
	}
	if (!info->uart_tx_initialized) {
		dev_err(dev, "Enable uart_tx_en first\n");
		mutex_unlock(&info->mutex_lock);
		return -EINVAL;
	}
	gpio_set_value(info->uart_tx_gpio, new_state);
	dev_info(dev, "UART_TX set %d\n", new_state);
	mutex_unlock(&info->mutex_lock);

	return count;
fail:
	dev_err(dev, "%s: goto fail\n", __func__);
	return -EINVAL;
}

static ssize_t uart_tx_val_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	int rc;

	mutex_lock(&info->mutex_lock);
	if (!info->uart_tx_initialized) {
		dev_err(dev, "Enable uart_tx_en first\n");
		mutex_unlock(&info->mutex_lock);
		return -EINVAL;
	}
	rc = snprintf(buf, PAGE_SIZE, "%d\n",
			gpio_get_value(info->uart_tx_gpio));
	mutex_unlock(&info->mutex_lock);
	return rc;
}
static DEVICE_ATTR_RW(uart_tx_val);

static ssize_t max_curr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ml_wearable_platform_info *info = dev_get_platdata(dev);
	int rc;

	if (info->max_curr == 0) {
		dev_err(dev, "No valid wearable max_curr value\n");
		return -EINVAL;
	}
	rc = snprintf(buf, PAGE_SIZE, "%d\n", info->max_curr);
	return rc;
}
static DEVICE_ATTR_RO(max_curr);

static struct attribute *wearable_attributes[] = {
	&dev_attr_power_state.attr,
	&dev_attr_hard_reset.attr,
	&dev_attr_soft_reset.attr,
	&dev_attr_factory_test.attr,
	&dev_attr_power_on_time.attr,
	&dev_attr_ap_interrupt.attr,
	&dev_attr_vsync_mux_en.attr,
	&dev_attr_extsync_en.attr,
	&dev_attr_on_off_gpio.attr,
	&dev_attr_uart_tx_en.attr,
	&dev_attr_uart_tx_val.attr,
	&dev_attr_max_curr.attr,
	NULL
};

static const struct attribute_group wearable_attr_group = {
	.attrs = wearable_attributes,
};

/* Bookkeeping for chardev that reports GPIO events */
static unsigned long int extsync_dev_is_open;
#define EXTSYNC_DEV_NAME "extsync_pin"

/* Extsync chardev helper fns */
#define EXTSYNC_DEV_OWNER (0)
static int extsync_dev_open(struct inode *inode, struct file *file)
{
	/* For extra safety, allow only one client to obtain chardev */
	if (test_and_set_bit_lock(EXTSYNC_DEV_OWNER, &extsync_dev_is_open))
		return -EBUSY;
	return 0;
}

static int extsync_dev_release(struct inode *inode, struct file *file)
{
	/* Release exclusive ownership of chardev */
	clear_bit_unlock(EXTSYNC_DEV_OWNER, &extsync_dev_is_open);
	return 0;
}

static ssize_t extsync_dev_read(struct file *filp,
				char *buffer,
				size_t buffsize,
				loff_t *offset)
{
	int output_size = 0;

	while (extsync_event_log.event_list
		&& (extsync_event_log.tail != extsync_event_log.head)) {
		char tempstr[128];
		int length;
		int tail;

		tail = extsync_event_log.tail;
		length = snprintf(tempstr, sizeof(tempstr),
			"Extsync (%6d): GPIO: %d mtime:%12llu ktime:%12llu\n",
			extsync_event_log.event_list[tail].event_num,
			extsync_event_log.event_list[tail].state,
			extsync_event_log.event_list[tail].mtime,
			extsync_event_log.event_list[tail].ktime);

		/*
		 * Check if we have room.
		 * If not, we exit without removing this entry from queue.
		 */
		if (buffsize < length)
			break;
		buffsize -= length;

		if (copy_to_user(buffer, tempstr, length))
			return -EFAULT;
		buffer += length;
		output_size += length;

		extsync_event_log.tail =
			(extsync_event_log.tail + 1) % EXTSYNC_EVENT_QUEUE_SIZE;
	}

	return output_size;
}

static ssize_t
extsync_dev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	return -EINVAL; /* Nope. */
}

static const struct file_operations extsync_dev_fops = {
	.owner = THIS_MODULE,
	.read = extsync_dev_read,
	.write = extsync_dev_write,
	.open = extsync_dev_open,
	.release = extsync_dev_release
};

static struct miscdevice extsync_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = EXTSYNC_DEV_NAME,
	.fops = &extsync_dev_fops,
};

/* Handler for interrupt from audio jack */
static irqreturn_t extsync_irq_handler(int irq, void *data)
{
	int (*callback)(struct extsync_event *);
	struct ml_wearable_platform_info *info = data;
	int new_head, cur_head;
	int status;

	callback = extsync_irq_custom_callback;

	if (info->rx_pin_state != ML_WEARABLE_HSJ_RX_EXTSYNC)
		return IRQ_HANDLED;

	if (!callback)
		return IRQ_HANDLED;

	if (!extsync_event_log.event_list)
		return IRQ_HANDLED;

	cur_head = extsync_event_log.head;
	new_head = (cur_head + 1) % EXTSYNC_EVENT_QUEUE_SIZE;

	extsync_event_log.event_ctr++;

	/* Only add to the queue if there's room. */
	if (new_head == extsync_event_log.tail)
		return IRQ_HANDLED;

	extsync_event_log.event_list[cur_head].state =
		gpio_get_value(info->extsync_gpio);

	/*
	 * If there are event_num discontinuities in the log,
	 * we've overflowed our event buffer.
	 */
	extsync_event_log.event_list[cur_head].event_num =
		extsync_event_log.event_ctr;

	/* Grab wearable timestamp */
	status = callback(&(extsync_event_log.event_list[cur_head]));

	/* Keep sample if callback succeeded */
	if (!status)
		extsync_event_log.head = new_head;

	return IRQ_HANDLED;
}

/* handler for interrupt from wearable */
static irqreturn_t wearable_irq_handler(int irq, void *data)
{
	void (*callback)(void);
	struct ml_wearable_platform_info *info;

	callback = wearable_irq_custom_callback;
	if (callback != NULL)
		callback();

	info = data;

	/* This is being used for factory testing. Each interrupt
	 * toggles the vsync_interrupt_level value, since it occurs on rising
	 * and falling edge. The factory_test attribute shows the level.
	 */
	info->vsync_interrupt_level = 1 - info->vsync_interrupt_level;

	return IRQ_HANDLED;
}

static int ml_wearable_control_probe(struct platform_device *pdev)
{
	int err;
	struct device_node *node;
	struct ml_wearable_platform_info *info;
	unsigned long gpio_flags = GPIOF_DIR_OUT;

	ml_wearable_deregister_wearable_irq_custom_callback();
	ml_wearable_deregister_extsync_irq_custom_callback();

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->reg_pwr = devm_regulator_get(&pdev->dev, "pwr");
	if (IS_ERR(info->reg_pwr)) {
		dev_err(&pdev->dev, "Unable to read pwr regulator!\n");
		return PTR_ERR(info->reg_pwr);
	}

	info->dev = &pdev->dev;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "of_node not found!\n");
		return -ENOENT;
	}

	node = pdev->dev.of_node;
	info->vsync_irq = of_get_named_gpio(node, "gpio_vsync_irq", 0);
	if (!gpio_is_valid(info->vsync_irq)) {
		dev_err(&pdev->dev, "Invalid vsync_irq GPIO pin!\n");
		return -EINVAL;
	}

	err = devm_gpio_request_one(&pdev->dev, info->vsync_irq,
		GPIOF_IN, "wearable_vsync_irq");
	if (err) {
		dev_err(&pdev->dev, "wearable_vsync_irq GPIO request failed: %d\n",
			err);
		return err;
	}

	info->vsync_interrupt_level = 0;

	node = pdev->dev.of_node;
	/*
	 * extsync_gpio doesn't exist in all configs -- so rather than exit
	 * if init fails, we proceed but skip some extsync related parts.
	 */
	info->extsync_gpio = of_get_named_gpio(node, "gpio_extsync_irq", 0);
	if (gpio_is_valid(info->extsync_gpio)) {
		err = misc_register(&extsync_misc);
		if (err) {
			dev_err(&pdev->dev, "%s misc_register failed.\n",
				EXTSYNC_DEV_NAME);
			return err;
		}
		dev_info(&pdev->dev, "%s registered\n", EXTSYNC_DEV_NAME);
	}

	info->extsync_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(info->extsync_pinctrl)) {
		err = PTR_ERR(info->extsync_pinctrl);
		dev_err(&pdev->dev, "devm_pinctrl_get error: %d\n", err);
		return err;
	}
	info->uart_rx_gpio_config =
		pinctrl_lookup_state(info->extsync_pinctrl, "uart_rx");
	if (IS_ERR(info->uart_rx_gpio_config)) {
		err = PTR_ERR(info->uart_rx_gpio_config);
		dev_err(&pdev->dev, "uart_rx_gpio_config error: %d\n", err);
		return err;
	}
	info->extsync_gpio_config =
		pinctrl_lookup_state(info->extsync_pinctrl, "ext_sync");
	if (IS_ERR(info->extsync_gpio_config)) {
		err = PTR_ERR(info->extsync_gpio_config);
		dev_err(&pdev->dev, "extsync_gpio_config error: %d\n", err);
		return err;
	}

	info->ap_int_gpio = of_get_named_gpio(node, "gpio_ap_int", 0);
	if (!gpio_is_valid(info->ap_int_gpio)) {
		dev_err(&pdev->dev, "Invalid ap_int_gpio GPIO pin!\n");
		return -EINVAL;
	}

	err = devm_gpio_request_one(&pdev->dev, info->ap_int_gpio,
			GPIOF_DIR_OUT | GPIOF_INIT_LOW, "ap_int");
	if (err) {
		dev_err(&pdev->dev, "ap_int_gpio GPIO request failed: %d\n",
				err);
		return err;
	}

	info->on_off_gpio = of_get_named_gpio(node, "gpio_on_off", 0);
	if (!gpio_is_valid(info->on_off_gpio)) {
		dev_err(&pdev->dev, "Invalid on_off_gpio GPIO pin!\n");
		return -EINVAL;
	}

	info->invert_gpio_on_off = of_property_read_bool(node,
			"gpio-on-off-active-low") ? 1 : 0;

	if (info->invert_gpio_on_off)
		gpio_flags |= GPIOF_INIT_LOW;
	else
		gpio_flags |= GPIOF_INIT_HIGH;

	err = devm_gpio_request_one(&pdev->dev, info->on_off_gpio,
			gpio_flags, "on_off");

	if (err) {
		dev_err(&pdev->dev, "on_off_gpio GPIO request failed: %d\n",
				err);
		return err;
	}

	info->vsync_mux_en_gpio =
		of_get_named_gpio(node, "gpio_vsync_mux_en", 0);
	if (!gpio_is_valid(info->vsync_mux_en_gpio)) {
		dev_err(&pdev->dev, "Invalid vsync_mux_en GPIO pin!\n");
		return -EINVAL;
	}

	err = devm_gpio_request_one(&pdev->dev, info->vsync_mux_en_gpio,
			GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "vsync_mux_en");
	if (err) {
		dev_err(&pdev->dev, "vsync_mux_en GPIO request failed: %d\n",
				err);
		return err;
	}

	info->flt_irq_gpio =
		of_get_named_gpio(node, "gpio_flt_irq", 0);
	if (gpio_is_valid(info->flt_irq_gpio)) {
		err = devm_gpio_request_one(&pdev->dev, info->flt_irq_gpio,
			GPIOF_DIR_IN | GPIOF_EXPORT_DIR_FIXED, "flt_irq");
		if (err) {
			dev_err(&pdev->dev,
				"gpio_flt_irq GPIO request failed: %d\n", err);
			return err;
		}

		err = gpio_export_link(&pdev->dev, "flt_irq",
					info->flt_irq_gpio);
		if (err) {
			dev_err(&pdev->dev,
				"flt_irq GPIO export_link failed: %d\n", err);
			return err;
		}
	}

	err = of_property_read_u32(node, "max_curr", &info->max_curr);
	if (err) {
		dev_err(&pdev->dev,
			"max_curr read failed %d\n", err);
		return err;
	}

	info->headset_signal = ML_WEARABLE_HSJ_MUX_UART;
	info->rx_pin_state = ML_WEARABLE_HSJ_RX_UART;

	mutex_init(&info->mutex_lock);


	info->uart_tx_gpio =
		of_get_named_gpio(node, "gpio_uart_tx", 0);
	if (!gpio_is_valid(info->uart_tx_gpio)) {
		dev_err(&pdev->dev, "Invalid uart_tx GPIO pin!\n");
		err = -EINVAL;
		goto err_1;
	}

	info->work_q = create_singlethread_workqueue("wearable_wq");
	if (!info->work_q) {
		dev_err(&pdev->dev, "unable to create wearable workqueue\n");
		goto err_1;
	}

	INIT_WORK(&info->work_reset, reset_work_fn);
	INIT_WORK(&info->work_power_state, power_state_work_fn);

	pdev->dev.platform_data = info;

	err = set_ap_int_status(&pdev->dev, ML_WEARABLE_AP_INT_DISABLED);
	if (err != 0)
		goto err_2;

	err = sysfs_create_group(&pdev->dev.kobj, &wearable_attr_group);
	if (err < 0) {
		dev_err(&pdev->dev, "unable to create sysfs node %d\n", err);
		goto err_2;
	}

	/* Set up handler for VSYNC signal */
	err = devm_request_irq(&pdev->dev,
		gpio_to_irq(info->vsync_irq), wearable_irq_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		pdev->name, info);
	if (err) {
		dev_err(&pdev->dev, "Failed to set up vsync irq handler: %d\n",
				err);
		goto err_2;
	}
	disable_irq_nosync(gpio_to_irq(info->vsync_irq));

	/* Set up handler for EXTSYNC signal */
	if (gpio_is_valid(info->extsync_gpio)) {
		err = devm_request_irq(&pdev->dev,
			gpio_to_irq(info->extsync_gpio), extsync_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			pdev->name, info);
		if (err) {
			dev_err(&pdev->dev, "Failed to set up extsync_irq handler: %d\n",
					err);
			goto err_2;
		}
		disable_irq_nosync(gpio_to_irq(info->extsync_gpio));
	}

	queue_work_power_state(&pdev->dev, ML_WEARABLE_POWER_STATE_POWERED,
			ML_WEARABLE_HARD_RESET);

	dev_info(&pdev->dev, "wearable control initialized\n");

	return 0;

err_2:
	destroy_workqueue(info->work_q);
err_1:
	mutex_destroy(&info->mutex_lock);
	if (gpio_is_valid(info->extsync_gpio))
		misc_deregister(&extsync_misc);
	kfree(extsync_event_log.event_list);
	sysfs_remove_link(&pdev->dev.kobj, "flt_irq");
	dev_err(&pdev->dev, "unable to initialize wearable: %d\n", err);
	return -EAGAIN;
}

static int ml_wearable_control_remove(struct platform_device *pdev)
{
	struct ml_wearable_platform_info *info;

	ml_wearable_deregister_wearable_irq_custom_callback();
	ml_wearable_deregister_extsync_irq_custom_callback();
	info = dev_get_platdata(&pdev->dev);
	if (gpio_is_valid(info->extsync_gpio))
		misc_deregister(&extsync_misc);

	sysfs_remove_link(&pdev->dev.kobj, "flt_irq");
	sysfs_remove_group(&pdev->dev.kobj, &wearable_attr_group);
	cancel_work_sync(&info->work_power_state);
	cancel_work_sync(&info->work_reset);
	flush_workqueue(info->work_q);
	destroy_workqueue(info->work_q);
	mutex_lock(&info->mutex_lock);
	set_power_state_helper(&pdev->dev, ML_WEARABLE_POWER_STATE_NOT_POWERED,
			       ML_WEARABLE_NO_RESET);
	kfree(extsync_event_log.event_list);
	mutex_unlock(&info->mutex_lock);
	mutex_destroy(&info->mutex_lock);

	return 0;
}

static const struct of_device_id ml_wearable_of_match[] = {
	{
		.compatible = "ml,wearable",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ml_wearable_of_match);

static struct platform_driver ml_wearable_platform_driver = {
	.driver		= {
		.name	= "ml_wearable",
		.of_match_table = of_match_ptr(ml_wearable_of_match),
	},
	.probe = ml_wearable_control_probe,
	.remove = ml_wearable_control_remove
};

module_platform_driver(ml_wearable_platform_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("Control interface to wearable");
