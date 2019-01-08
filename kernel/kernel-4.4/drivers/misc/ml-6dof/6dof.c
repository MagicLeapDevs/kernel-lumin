/* Copyright (c) 2017-2018, Magic Leap, Inc. All rights reserved.
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
 * Driver for Magic Leap Six Degrees Of Freedom
 */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mailbox_client.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/tegra-aon.h>
#include "ml-6dof-ioctl.h"
#include "aon-6dof-messages.h"
#include "6dof-io-fpga.h"
#include "6dof-io-adc.h"

#define SIXDOF_IVC_FRAME_SIZE   256
#define SIXDOF_MBOX_TOUT_MS     40
#define SIXDOF_REQ_TOUT_MS      200

struct sixdof_ctrl_msg {
	struct aon_6dof_request    req;
	struct aon_6dof_response   resp;
	struct completion          wait_on;
};

struct sixdof_data {
	struct device              *dev;
	struct miscdevice          mdev;
	atomic_t                   ref_count;
	struct mbox_client         cl;
	struct mbox_chan           *mbox;
	struct mutex               lock;
	bool                       wear_online;
	bool                       suspend;
	bool                       enabled;
	enum ml_6dof_mode          mode;
	struct sixdof_ctrl_msg     ctrl_msg[AON_6DOF_REQ_NUM];
};

static DEFINE_MUTEX(ctrl_mutex);

struct ml_6dof_io {
	struct ml_fpga_io        *fpga;
	struct ml_adc_io         *adc;
};

static struct ml_6dof_io ios;

const char *emt_mode_to_str(enum ml_6dof_mode mode)
{
	switch (mode) {
	case EMT_MODE_TDSP:
		return "tdsp";
	case EMT_MODE_ADC:
		return "raw";
	default:
		return "invalid";
	}
}

int sixdof_fpga_io_register(struct ml_fpga_io *io)
{
	int rc = 0;

	mutex_lock(&ctrl_mutex);
	if (ios.fpga  == NULL)
		ios.fpga = io;
	else
		rc = -EEXIST;
	mutex_unlock(&ctrl_mutex);

	return rc;
}
EXPORT_SYMBOL(sixdof_fpga_io_register);

void sixdof_fpga_io_unregister(void)
{
	mutex_lock(&ctrl_mutex);
	ios.fpga = NULL;
	mutex_unlock(&ctrl_mutex);
}
EXPORT_SYMBOL(sixdof_fpga_io_unregister);

int sixdof_adc_io_register(struct ml_adc_io *io)
{
	int rc = 0;

	mutex_lock(&ctrl_mutex);
	if (ios.adc == NULL)
		ios.adc = io;
	else
		rc = -EEXIST;
	mutex_unlock(&ctrl_mutex);

	return rc;
}
EXPORT_SYMBOL(sixdof_adc_io_register);

void sixdof_adc_io_unregister(void)
{
	mutex_lock(&ctrl_mutex);
	ios.adc = NULL;
	mutex_unlock(&ctrl_mutex);
}
EXPORT_SYMBOL(sixdof_adc_io_unregister);

static bool all_ios_registered(void)
{
	if (ios.fpga && ios.adc)
		return true;

	return false;
}

static bool is_sixdof_ready(const struct sixdof_data *sixdof)
{
	if (!all_ios_registered()) {
		dev_warn(sixdof->dev, "IO drv is not registered yet\n");
		return false;
	}

	if (!sixdof->wear_online)
		return false;

	if (sixdof->suspend) {
		dev_warn(sixdof->dev, "Suspended\n");
		return false;
	}

	return true;
}

static void sixdof_set_to_default(struct sixdof_data *sixdof)
{
	sixdof->enabled = false;
	sixdof->mode = EMT_MODE_TDSP;
}

static void sixdof_receive_ivc_msg(struct mbox_client *cl, void *rx_msg)
{
	struct sixdof_data              *sixdof;
	const struct tegra_aon_mbox_msg *aon_msg;
	const struct aon_6dof_response  *resp_in;
	struct sixdof_ctrl_msg          *msg;

	sixdof = container_of(cl, struct sixdof_data, cl);
	aon_msg = (struct tegra_aon_mbox_msg *)rx_msg;
	if (aon_msg->length > SIXDOF_IVC_FRAME_SIZE) {
		dev_err(sixdof->dev,
			"%s: invalid aon response length = %d bytes\n",
			__func__, aon_msg->length);
		return;
	}

	resp_in = (const struct aon_6dof_response *)aon_msg->data;
	if (!resp_in) {
		dev_err(sixdof->dev, "%s: empty response\n", __func__);
		return;
	}
	if (resp_in->resp_type >= AON_6DOF_REQ_NUM) {
		dev_err(sixdof->dev, "%s: invalid response id = %d\n",
			__func__, resp_in->resp_type);
		return;
	}

	print_hex_dump_debug("6dof: ", DUMP_PREFIX_NONE, 16, 1, resp_in,
			     sizeof(struct aon_6dof_response), 0);

	mutex_lock(&sixdof->lock);
	msg = &sixdof->ctrl_msg[resp_in->resp_type];
	memcpy(&msg->resp, resp_in, sizeof(struct aon_6dof_response));
	complete(&msg->wait_on);
	mutex_unlock(&sixdof->lock);
}

static int sixdof_tx_ivc_req(struct device *dev,
			     enum aon_6dof_req_type req_id,
			     const void *data)
{
	struct tegra_aon_mbox_msg msg;
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	struct aon_6dof_request *req;
	const struct aon_6dof_response *resp;
	int err;

	if (req_id >= AON_6DOF_REQ_NUM)
		return -EINVAL;

	req = &sixdof->ctrl_msg[req_id].req;
	resp = &sixdof->ctrl_msg[req_id].resp;

	req->req_type = req_id;
	switch (req_id) {
	case AON_6DOF_REQ_EM_DATA:
		/* Nothing to copy */
		break;
	case AON_6DOF_REQ_MODE:
		req->data.mode.mode = *(const u8 *)data;
		break;
	default:
		dev_err(dev, "invalid 6dof request id = %d\n", req_id);
		return -EINVAL;
	}

	msg.length = sizeof(struct aon_6dof_request);
	msg.data = (void *)req;
	mutex_lock(&sixdof->lock);
	reinit_completion(&sixdof->ctrl_msg[req_id].wait_on);
	mutex_unlock(&sixdof->lock);
	err = mbox_send_message(sixdof->mbox, (void *)&msg);
	if (err < 0) {
		dev_err(dev, "mbox_send_message failed, %d\n", err);
		return err;
	}

	err = wait_for_completion_timeout(&sixdof->ctrl_msg[req_id].wait_on,
					msecs_to_jiffies(SIXDOF_REQ_TOUT_MS));
	if (!err) {
		dev_err(sixdof->dev, "request [%d] timed out\n", req_id);
		return -ETIMEDOUT;
	}

	if (resp->status != AON_6DOF_STATUS_OK) {
		dev_err(dev, "request [%d] failed\n", req_id);
		return -EFAULT;
	}

	return 0;
}

static int ivc_mode_req(struct sixdof_data *sixdof, enum aon_6dof_mode mode)
{
	return sixdof_tx_ivc_req(sixdof->dev, AON_6DOF_REQ_MODE, &mode);
}

static void hw_power_off(struct sixdof_data *sixdof)
{
	dev_info(sixdof->dev, "%s\n", __func__);
	ios.fpga->ops->set_power(ios.fpga, false);
	ios.fpga->ops->ctrl_reg(ios.fpga, false, sixdof->mode, false);
	ios.adc->ops->set_power(ios.adc, false);
	ios.adc->ops->power_on_gpio(ios.adc, false);
	sixdof->enabled = false;
	dev_info(sixdof->dev, "6dof is off\n");
}

static int hw_power_on(struct sixdof_data *sixdof)
{
	int rc;

	dev_info(sixdof->dev, "%s\n", __func__);
	/* Program remote FPGA control register */
	rc = ios.fpga->ops->ctrl_reg(ios.fpga, true, sixdof->mode, false);

	/* Program ADC */
	ios.adc->ops->power_on_gpio(ios.adc, true);
	rc += ios.adc->ops->set_power(ios.adc, false);
	rc += ios.adc->ops->set_mode(ios.adc, sixdof->mode);
	rc += ios.adc->ops->set_power(ios.adc, true);

	/* Program local FPGA control register */
	rc += ios.fpga->ops->set_power(ios.fpga, true);
	rc += ios.fpga->ops->set_mode(ios.fpga, sixdof->mode);

	if (!rc) {
		sixdof->enabled = true;
		dev_info(sixdof->dev, "6dof is on\n");
	} else {
		dev_err(sixdof->dev, "failed to turn on 6dof\n");
		rc = -EIO;
	}
	return rc;
}

static ssize_t adc_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	u32 adc[EMT_AXIS_NUM];
	ssize_t rc = -EFAULT;

	mutex_lock(&ctrl_mutex);
	if (!is_sixdof_ready(sixdof) || ios.fpga->ops->read_adc(ios.fpga, adc))
		goto unlock;

	rc = scnprintf(buf, PAGE_SIZE, "0x%08x 0x%08x 0x%08x\n",
		       adc[EMT_AXIS_X], adc[EMT_AXIS_Y], adc[EMT_AXIS_Z]);
unlock:
	mutex_unlock(&ctrl_mutex);
	return rc;
}

static ssize_t enable_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d", sixdof->enabled);
}

static ssize_t coil4_tx_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t size)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	uint8_t on;
	int rc;

	rc = kstrtou8(buf, 0, &on);
	if (rc)
		return rc;

	mutex_lock(&ctrl_mutex);
	if (!is_sixdof_ready(sixdof)) {
		rc = -EFAULT;
		goto unlock;
	}

	rc = ios.fpga->ops->ctrl_reg(ios.fpga, true, sixdof->mode, on);

unlock:
	mutex_unlock(&ctrl_mutex);
	return (rc == 0 ? size : rc);
}

static ssize_t enable_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf,
			    size_t size)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	unsigned int pwr;
	int rc;

	rc = kstrtouint(buf, 0, &pwr);
	if (rc)
		return rc;

	mutex_lock(&ctrl_mutex);
	if (!is_sixdof_ready(sixdof)) {
		rc = -EFAULT;
		goto unlock;
	}

	if (!!pwr == sixdof->enabled) {
		dev_err(dev, "already in this power state\n");
		goto unlock;
	}

	if (pwr) {
		rc = hw_power_on(sixdof);
		rc += ivc_mode_req(sixdof, (sixdof->mode == EMT_MODE_TDSP ?
					AON_6DOF_MODE_TDSP :
					AON_6DOF_MODE_PASS_THRU));
	} else {
		hw_power_off(sixdof);
		ivc_mode_req(sixdof, AON_6DOF_MODE_DISABLED);
	}

unlock:
	mutex_unlock(&ctrl_mutex);
	return (rc == 0 ? size : rc);
}

static ssize_t mode_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	int rc = -EFAULT;

	mutex_lock(&ctrl_mutex);
	if (is_sixdof_ready(sixdof))
		rc = scnprintf(buf, PAGE_SIZE, "%d - %s", sixdof->mode,
			       (sixdof->mode == EMT_MODE_TDSP ? "tdsp\n" :
								"raw\n"));
	mutex_unlock(&ctrl_mutex);
	return rc;
}

static ssize_t mode_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t size)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	unsigned int mode;
	int rc;

	rc = kstrtouint(buf, 0, &mode);
	if (rc)
		return rc;

	if (mode != EMT_MODE_TDSP && mode != EMT_MODE_ADC)
		return -EINVAL;

	dev_info(dev, "%s(%s)\n", __func__, emt_mode_to_str(mode));
	mutex_lock(&ctrl_mutex);
	if (!is_sixdof_ready(sixdof)) {
		rc = -EFAULT;
		goto unlock;
	}

	if (!sixdof->enabled || sixdof->mode == mode) {
		dev_dbg(dev, "%s: mode has not changed\n", __func__);
		goto unlock;
	}

	sixdof->mode = mode;
	/* Program ADC */
	rc = ios.adc->ops->set_power(ios.adc, false);
	rc += ios.adc->ops->set_mode(ios.adc, mode);
	rc += ios.adc->ops->set_power(ios.adc, true);

	/* Program FPGA registers */
	rc += ios.fpga->ops->ctrl_reg(ios.fpga, true, mode, false);
	rc += ios.fpga->ops->set_mode(ios.fpga, mode);

	/* Send mode change request to SPE */
	rc += ivc_mode_req(sixdof, (sixdof->mode == EMT_MODE_TDSP ?
					AON_6DOF_MODE_TDSP :
					AON_6DOF_MODE_PASS_THRU));

unlock:
	mutex_unlock(&ctrl_mutex);
	return (rc == 0 ? size : -EIO);
}

static ssize_t supported_freq_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	struct ml_6dof_freq freq;
	u8 num, i;
	ssize_t rc = 0;

	mutex_lock(&ctrl_mutex);
	if (!is_sixdof_ready(sixdof)) {
		rc = -EFAULT;
		goto unlock;
	}

	num = ios.fpga->ops->get_freq_num();
	for (i = 0; i < num; i++) {
		freq = ios.fpga->ops->freq(i);
		rc += scnprintf(buf + rc, PAGE_SIZE - rc,
				"%d: x=0x%08x y=0x%08x z=0x%08x\n",
				i, freq.x, freq.y, freq.z);
	}

unlock:
	mutex_unlock(&ctrl_mutex);
	return rc;
}

static ssize_t sixdof_freq_query(struct device *dev, u8 totem, char *buf)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	u8 freq;
	ssize_t rc;

	mutex_lock(&ctrl_mutex);
	if (!is_sixdof_ready(sixdof)) {
		rc = -EFAULT;
		goto unlock;
	}

	rc = ios.fpga->ops->query_freq(ios.fpga, totem, &freq);
	if (!rc)
		rc = scnprintf(buf, PAGE_SIZE, "%d", freq);

unlock:
	mutex_unlock(&ctrl_mutex);
	return rc;
}

static int sixdof_freq_set(struct device *dev, u8 totem, const char *buf)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	u8 freq;
	ssize_t rc;

	rc = kstrtou8(buf, 0, &freq);
	if (rc)
		return rc;

	mutex_lock(&ctrl_mutex);
	dev_info(dev, "%s(totem = %d, freq = %d)\n", __func__, totem, freq);
	if (!is_sixdof_ready(sixdof)) {
		rc = -EFAULT;
		goto unlock;
	}

	rc = ios.fpga->ops->set_freq(ios.fpga, totem, freq);

unlock:
	mutex_unlock(&ctrl_mutex);
	return rc;
}

static ssize_t freq_controller_1_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	return sixdof_freq_query(dev, ML_6DOF_TOTEM_1_ID, buf);
}

static ssize_t freq_controller_2_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	return sixdof_freq_query(dev, ML_6DOF_TOTEM_2_ID, buf);
}

static ssize_t freq_controller_1_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t size)
{
	int rc = sixdof_freq_set(dev, ML_6DOF_TOTEM_1_ID, buf);

	return (rc != 0 ? rc : size);
}

static ssize_t freq_controller_2_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t size)
{
	int rc = sixdof_freq_set(dev, ML_6DOF_TOTEM_2_ID, buf);

	return (rc != 0 ? rc : size);
}

static ssize_t reset_fine_freq_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t size)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	uint8_t totem_id;
	int rc;

	rc = kstrtou8(buf, 0, &totem_id);
	if (rc)
		return rc;

	dev_dbg(dev, "%s(%d)\n", __func__, totem_id);
	if (!ML_6DOF_IS_VALID_TOTEM_ID(totem_id)) {
		dev_err(dev, "%s: invalid totem id %d\n", __func__, totem_id);
		return -EINVAL;
	}

	mutex_lock(&ctrl_mutex);
	if (!is_sixdof_ready(sixdof)) {
		rc = -EFAULT;
		goto unlock;
	}

	rc = ios.fpga->ops->rst_fine_freq(ios.fpga, totem_id);

unlock:
	mutex_unlock(&ctrl_mutex);
	return (rc != 0 ? rc : size);
}

static ssize_t wearable_state_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 sixdof->wear_online ? "online" : "offline");
}

static void sixdof_restore_state(struct sixdof_data *sixdof)
{
	uint8_t freq;

	/*
	 * If 6DOF was previously enabled (wearable crash?)
	 * then try to re-apply the previous configuration
	 */
	if (!sixdof->enabled)
		return;

	hw_power_on(sixdof);
	if (!ios.fpga->ops->query_freq(ios.fpga, ML_6DOF_TOTEM_1_ID, &freq)) {
		ios.fpga->ops->rst_fine_freq(ios.fpga, ML_6DOF_TOTEM_1_ID);
		ios.fpga->ops->set_freq(ios.fpga, ML_6DOF_TOTEM_1_ID, freq);
	}
	if (!ios.fpga->ops->query_freq(ios.fpga, ML_6DOF_TOTEM_2_ID, &freq)) {
		ios.fpga->ops->rst_fine_freq(ios.fpga, ML_6DOF_TOTEM_2_ID);
		ios.fpga->ops->set_freq(ios.fpga, ML_6DOF_TOTEM_2_ID, freq);
	}
}

#define SIXDOF_WEAR_STATE_STR_LEN 16
static ssize_t wearable_state_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct sixdof_data *sixdof = dev_get_platdata(dev);
	bool online;
	int rc = 0;

	if (!strncasecmp(buf, "online", SIXDOF_WEAR_STATE_STR_LEN))
		online = true;
	else if (!strncasecmp(buf, "offline", SIXDOF_WEAR_STATE_STR_LEN))
		online = false;
	else
		return -EINVAL;

	mutex_lock(&ctrl_mutex);
	if (online == sixdof->wear_online) {
		dev_dbg(dev, "%s: already %s\n", __func__,
			online ? "online" : "offline");
		goto unlock;
	}

	if (!all_ios_registered()) {
		rc = -ENODEV;
		goto unlock;
	}

	dev_info(dev, "%s: switching to %s\n", __func__,
		online ? "online" : "offline");
	if (online) {
		rc = ios.fpga->ops->revision(ios.fpga);
		if (rc)
			goto unlock;
		sixdof_restore_state(sixdof);
	}
	sixdof->wear_online = online;

unlock:
	mutex_unlock(&ctrl_mutex);
	return rc ? rc : size;
}

static DEVICE_ATTR_RO(adc);
static DEVICE_ATTR_WO(coil4_tx);
static DEVICE_ATTR_RW(enable);
static DEVICE_ATTR_RW(freq_controller_1);
static DEVICE_ATTR_RW(freq_controller_2);
static DEVICE_ATTR_RW(mode);
static DEVICE_ATTR_WO(reset_fine_freq);
static DEVICE_ATTR_RO(supported_freq);
static DEVICE_ATTR_RW(wearable_state);

static struct attribute *sixdof_attributes[] = {
	&dev_attr_adc.attr,
	&dev_attr_coil4_tx.attr,
	&dev_attr_enable.attr,
	&dev_attr_mode.attr,
	&dev_attr_supported_freq.attr,
	&dev_attr_freq_controller_1.attr,
	&dev_attr_freq_controller_2.attr,
	&dev_attr_reset_fine_freq.attr,
	&dev_attr_wearable_state.attr,
	NULL
};

static const struct attribute_group sixdof_attr_group = {
	.attrs = sixdof_attributes,
};

static int sixdof_cdev_open(struct inode *inode, struct file *filep)
{
	struct sixdof_data *sixdof = container_of(filep->private_data,
						  struct sixdof_data, mdev);

	if (!atomic_add_unless(&sixdof->ref_count, 1, 1)) {
		dev_err(sixdof->dev, "%s: sixdof char device is busy\n",
			__func__);
		return -EBUSY;
	}
	return 0;
}

static int sixdof_cdev_release(struct inode *inode, struct file *filep)
{
	struct sixdof_data *sixdof = container_of(filep->private_data,
						  struct sixdof_data, mdev);

	atomic_dec(&sixdof->ref_count);
	return 0;
}

static int sixdof_em_data_ioctl(struct sixdof_data *sixdof, unsigned long arg)
{
	const struct aon_6dof_resp_em_data *em;
	int rc;

	dev_dbg(sixdof->dev, "%s\n", __func__);
	if (!sixdof->enabled)
		return -EINVAL;

	rc = sixdof_tx_ivc_req(sixdof->dev, AON_6DOF_REQ_EM_DATA, NULL);
	if (rc)
		return rc;

	em = &sixdof->ctrl_msg[AON_6DOF_REQ_EM_DATA].resp.data.em;
	mutex_lock(&sixdof->lock);
	/* aon_6dof_resp_em_data & emt_data_ioctl_msg should match */
	rc = copy_to_user((void __user *)arg, em,
			  sizeof(struct emt_data_ioctl_msg));
	mutex_unlock(&sixdof->lock);
	return rc;
}

static long sixdof_cdev_ioctl(struct file *filep, unsigned int cmd,
			      unsigned long arg)
{
	int rc = -EINVAL;
	struct sixdof_data *sixdof = container_of(filep->private_data,
						  struct sixdof_data,
						  mdev);

	if (!is_sixdof_ready(sixdof))
		return -EFAULT;

	if (cmd == EMT_IOCTL_GET_EM_DATA)
		rc = sixdof_em_data_ioctl(sixdof, arg);

	return rc;
}

static long sixdof_cdev_write(struct file *filep, const char __user *user_buf,
			      size_t count, loff_t *pos)
{
	struct sixdof_data *sixdof = container_of(filep->private_data,
						  struct sixdof_data,
						  mdev);
	int rc;

	if (count != FPGA_FREQ_ADJ_LEN * sizeof(u32)) {
		dev_err(sixdof->dev, "%s: invalid data length %zu\n",
			__func__, count);
		return -EINVAL;
	}

	mutex_lock(&ctrl_mutex);
	if (is_sixdof_ready(sixdof))
		rc = ios.fpga->ops->adjust_freq(ios.fpga,
						(const u32 __user *) user_buf);
	else
		rc = -EFAULT;
	mutex_unlock(&ctrl_mutex);
	return rc ? rc : count;
}

static const struct file_operations sixdof_cdev_fops = {
	.owner          = THIS_MODULE,
	.open           = sixdof_cdev_open,
	.release        = sixdof_cdev_release,
	.unlocked_ioctl = sixdof_cdev_ioctl,
	.write          = sixdof_cdev_write,
	.llseek         = no_llseek,
};

#ifdef CONFIG_PM
static int sixdof_pm_suspend(struct device *dev)
{
	struct sixdof_data *sixdof = dev_get_drvdata(dev);

	sixdof->suspend = true;
	return 0;
}

static int sixdof_pm_resume(struct device *dev)
{
	struct sixdof_data *sixdof = dev_get_drvdata(dev);

	sixdof->suspend = false;
	return 0;
}

static const struct dev_pm_ops sixdof_pm_ops = {
	.suspend = sixdof_pm_suspend,
	.resume = sixdof_pm_resume,
};
#endif

static int sixdof_probe(struct platform_device *pdev)
{
	struct sixdof_data *sixdof = NULL;
	int i;
	int rc;
	bool ready;

	dev_dbg(&pdev->dev, "%s: enter\n", __func__);

	/* Make sure all transport drivers are registered */
	mutex_lock(&ctrl_mutex);
	ready = all_ios_registered();
	mutex_unlock(&ctrl_mutex);
	if (!ready) {
		dev_dbg(&pdev->dev, "IO drv is not registered yet\n");
		return -EPROBE_DEFER;
	}

	sixdof = devm_kzalloc(&pdev->dev, sizeof(struct sixdof_data),
			      GFP_KERNEL);
	if (!sixdof)
		return -ENOMEM;

	sixdof_set_to_default(sixdof);
	sixdof->dev = &pdev->dev;
	pdev->dev.platform_data = sixdof;
	platform_set_drvdata(pdev, sixdof);
	mutex_init(&sixdof->lock);

	for (i = 0; i < ARRAY_SIZE(sixdof->ctrl_msg); i++) {
		sixdof->ctrl_msg[i].req.req_type = i;
		sixdof->ctrl_msg[i].resp.resp_type = i;
		init_completion(&sixdof->ctrl_msg[i].wait_on);
	}

	/* misc device */
	sixdof->mdev.minor = MISC_DYNAMIC_MINOR;
	sixdof->mdev.fops = &sixdof_cdev_fops;
	sixdof->mdev.name = "ml-6dof";
	rc = misc_register(&sixdof->mdev);
	if (rc) {
		dev_err(&pdev->dev, "failed to register miscdev, %d\n", rc);
		goto failure;
	}

	sixdof->cl.dev = &pdev->dev;
	sixdof->cl.tx_block = true;
	sixdof->cl.tx_tout = SIXDOF_MBOX_TOUT_MS;
	sixdof->cl.knows_txdone = false;
	sixdof->cl.rx_callback = sixdof_receive_ivc_msg;
	sixdof->mbox = mbox_request_channel(&sixdof->cl, 0);
	if (IS_ERR(sixdof->mbox)) {
		dev_err(&pdev->dev, "can't get mailbox channel (%d)\n",
			(int)PTR_ERR(sixdof->mbox));
		rc = PTR_ERR(sixdof->mbox);
		goto unreg_mdev;
	}
	dev_info(&pdev->dev, "sixdof_data->mbox = %pK\n", sixdof->mbox);

	rc = sysfs_create_group(&pdev->dev.kobj, &sixdof_attr_group);
	if (rc) {
		dev_err(&pdev->dev, "unable to create sysfs node %d\n", rc);
		goto free_mbox;
	}
	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);
	return 0;

free_mbox:
	mbox_free_channel(sixdof->mbox);
	sixdof->mbox = NULL;

unreg_mdev:
	misc_deregister(&sixdof->mdev);

failure:
	mutex_destroy(&sixdof->lock);
	return rc;
}

static int sixdof_remove(struct platform_device *pdev)
{
	struct sixdof_data *sixdof = dev_get_platdata(&pdev->dev);

	dev_info(&pdev->dev, "%s\n", __func__);
	mbox_free_channel(sixdof->mbox);
	sysfs_remove_group(&pdev->dev.kobj, &sixdof_attr_group);
	misc_deregister(&sixdof->mdev);
	mutex_destroy(&sixdof->lock);
	return 0;
}

static const struct of_device_id sixdof_of_match[] = {
	{
		.compatible = "ml,aon-6dof",
	},
	{}
};
MODULE_DEVICE_TABLE(of, sixdof_of_match);

static struct platform_driver sixdof_driver = {
	.driver = {
		.name       = "6dof",
		.owner      = THIS_MODULE,
#if defined(CONFIG_PM)
		.pm         = &sixdof_pm_ops,
#endif
		.of_match_table = of_match_ptr(sixdof_of_match),
	},
	.remove =   sixdof_remove,
	.probe =    sixdof_probe,
};
module_platform_driver(sixdof_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("Magic Leap Six Degrees Of Freedom");
MODULE_LICENSE("GPL v2");

