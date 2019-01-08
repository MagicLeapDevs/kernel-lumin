/* Copyright (c) 2017, Magic Leap, Inc. All rights reserved.
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
 * This driver exposes basic capabilities to communicate with
 * EM Tracking task running on AON side from userspace.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/mailbox_client.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/tegra-aon.h>
#include "aon-em-raw-messages.h"
#include "em-raw-ioctl.h"

#define  DEVICE_NAME                "em_raw"
#define  CLASS_NAME                 "aon_em_raw"
#define  EMT_TX_BLOCK_PERIOD_MS     2000

static dev_t            s_devnum;
static struct class     *s_class;
static struct device    *s_device;
static atomic_t         s_ref_count;

struct aon_data_t {
	struct mbox_client                  cl;
	struct mbox_chan                    *mbox;
	struct completion                   wait_resp_received;
	struct emt_raw_aon_req              req;
	struct mutex                        lock;
};

struct emt_data_t {
	struct cdev                         cdev;
	struct emt_ioctl_msg_t              *msg;
	struct aon_data_t                   aon_data;
};

static int     emt_cdev_open(struct inode *, struct file *);
static int     emt_cdev_mmap(struct file *, struct vm_area_struct *);
static long    emt_cdev_ioctl(struct file *, unsigned int, unsigned long);
static int     emt_cdev_release(struct inode *, struct file *);

static void emt_aon_receive_ivc_msg(struct mbox_client *cl, void *rx_msg)
{
	struct aon_data_t               *aon_data;
	struct emt_data_t               *emt_data;
	const struct tegra_aon_mbox_msg *aon_msg;
	const struct emt_raw_aon_resp   *aon_resp;

	aon_data = container_of(cl, struct aon_data_t, cl);
	emt_data = container_of(aon_data, struct emt_data_t, aon_data);

	aon_msg = (struct tegra_aon_mbox_msg *)rx_msg;
	if (aon_msg->length > EMT_RAW_AON_MAX_MSG_SIZE) {
		pr_err("%s: invalid aon response length = %d bytes\n", __func__,
		       aon_msg->length);
		goto done;
	}

	mutex_lock(&emt_data->aon_data.lock);
	aon_resp = (void *)aon_msg->data;

	if (aon_resp) {
		if (aon_resp->length <= EMT_MAX_RESP_SZ &&
		    aon_resp->status == EMT_RAW_AON_STATUS_OK) {
			/* Copy response */
			emt_data->msg->resp_len = aon_resp->length;
			memcpy(emt_data->msg->resp_data, aon_resp->data,
			       aon_resp->length);
		} else {
			/* Error case */
			pr_err("%s: invalid response: status = %d, length = %d\n",
			       __func__, aon_resp->status, aon_resp->length);
			emt_data->msg->resp_len = 0;
		}
	}
	mutex_unlock(&emt_data->aon_data.lock);

done:
	complete(&aon_data->wait_resp_received);
}

static int emt_cdev_mmap_msg_ioctl(struct emt_data_t *emt_data,
				   unsigned long arg)
{
	struct tegra_aon_mbox_msg   tegra_aon_msg;
	struct emt_raw_aon_req      *aon_request = &emt_data->aon_data.req;
	int ret = 0;

	pr_debug("%s: req_len = %d\n", __func__, emt_data->msg->req_len);

	aon_request->length = emt_data->msg->req_len;
	memcpy(aon_request->data, emt_data->msg->req_data,
	       emt_data->msg->req_len);

	/* Populate tegra_aon_mbox_msg structure */
	tegra_aon_msg.length = sizeof(struct emt_raw_aon_req);
	tegra_aon_msg.data   = (void *)aon_request;

	ret = mbox_send_message(emt_data->aon_data.mbox,
				(void *)&tegra_aon_msg);
	if (ret < 0) {
		pr_err("%s: mbox_send_message failed\n", __func__);
		goto exit;
	} else {
		if (!wait_for_completion_timeout(
			&emt_data->aon_data.wait_resp_received,
			msecs_to_jiffies(EMT_TX_BLOCK_PERIOD_MS))) {
			pr_err("%s: timeout waiting for IPC response\n",
				__func__);
			ret = -ETIMEDOUT;
			goto exit;
		}

		mutex_lock(&emt_data->aon_data.lock);
		if (emt_data->msg->resp_len > 0)
			ret = 0;
		else
			ret = -EIO;
		mutex_unlock(&emt_data->aon_data.lock);
	}

exit:
	if (ret)
		pr_err("%s: failed, err = %d\n", __func__, ret);

	return ret;
}

static long emt_cdev_ioctl(struct file *filep, unsigned int cmd,
			   unsigned long arg)
{
	struct emt_data_t *emt_data = filep->private_data;
	int ret;

	pr_debug("%s\n", __func__);
	switch (cmd) {
	case EMT_IOCTL_MMAP_MSG:
		ret = emt_cdev_mmap_msg_ioctl(emt_data, arg);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static const struct file_operations cdev_fops = {
	.owner              = THIS_MODULE,
	.open               = emt_cdev_open,
	.mmap               = emt_cdev_mmap,
	.unlocked_ioctl     = emt_cdev_ioctl,
	.release            = emt_cdev_release,
};

static int emt_cdev_open(struct inode *inodep, struct file *filep)
{
	struct emt_data_t *emt_data = NULL;

	pr_info("%s\n", __func__);
	if (!atomic_add_unless(&s_ref_count, 1, 1)) {
		pr_err("%s: busy\n", __func__);
		return -EBUSY;
	}
	emt_data = container_of(inodep->i_cdev, struct emt_data_t, cdev);
	filep->private_data = emt_data;
	return 0;
}

static int emt_cdev_mmap(struct file *filep, struct vm_area_struct *vma)
{
	int ret;
	struct emt_data_t *emt_data = filep->private_data;
	long length = vma->vm_end - vma->vm_start;
	unsigned long start = vma->vm_start;
	char *vmalloc_area_ptr = (char *)emt_data->msg;
	unsigned long pfn;

	if (length > EMT_MSG_TOTAL_PAGES * PAGE_SIZE)
		return -EIO;

	/* Re-map each page individually */
	while (length > 0) {
		pfn = vmalloc_to_pfn(vmalloc_area_ptr);
		ret = remap_pfn_range(vma, start, pfn, PAGE_SIZE, PAGE_SHARED);
		if (ret < 0)
			return ret;

		start += PAGE_SIZE;
		vmalloc_area_ptr += PAGE_SIZE;
		length -= PAGE_SIZE;
	}
	return 0;
}

static int emt_cdev_release(struct inode *inodep, struct file *filep)
{
	pr_info("%s\n", __func__);
	atomic_dec(&s_ref_count);
	filep->private_data = NULL;
	return 0;
}

static int emt_register_cdev(struct platform_device *pdev)
{
	struct emt_data_t     *emt_data;
	int                     ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);
	emt_data = dev_get_drvdata(&pdev->dev);
	ret = alloc_chrdev_region(&s_devnum, 0, 1, DEVICE_NAME);
	if (ret) {
		dev_err(&pdev->dev, "%s: alloc_chrdev_region() failed\n",
			__func__);
		goto exit;
	}

	cdev_init(&emt_data->cdev, &cdev_fops);
	emt_data->cdev.owner = THIS_MODULE;
	emt_data->cdev.ops = &cdev_fops;
	ret = cdev_add(&emt_data->cdev, s_devnum, 1);
	if (ret) {
		dev_err(&pdev->dev, "%s: cdev_add() failed\n", __func__);
		goto cdev_reg;
	}

	s_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(s_class)) {
		dev_err(&pdev->dev, "Failed to register %s class\n",
			CLASS_NAME);
		ret = PTR_ERR(s_class);
		goto del_cdev;
	}

	s_device = device_create(s_class, NULL, s_devnum, NULL, DEVICE_NAME);
	if (IS_ERR(s_device)) {
		dev_err(&pdev->dev, "Failed to create %s cdev\n", DEVICE_NAME);
		ret = PTR_ERR(s_device);
		goto destroy_class;
	}
	return 0;

destroy_class:
	class_destroy(s_class);
	s_class = NULL;
del_cdev:
	cdev_del(&emt_data->cdev);
cdev_reg:
	unregister_chrdev_region(s_devnum, 1);
exit:
	return ret;
}

static void emt_unregister_cdev(struct platform_device *pdev)
{
	struct emt_data_t     *emt_data;

	dev_dbg(&pdev->dev, "%s\n", __func__);
	emt_data =  dev_get_drvdata(&pdev->dev);

	cdev_del(&emt_data->cdev);
	if (s_class) {
		device_destroy(s_class, s_devnum);
		class_destroy(s_class);
		s_class = NULL;
	}
	unregister_chrdev_region(s_devnum, 1);
}

static int emt_aon_probe(struct platform_device *pdev)
{
	struct emt_data_t *emt_data = NULL;
	struct device_node  *node = pdev->dev.of_node;
	int                 ret = 0;

	dev_info(&pdev->dev, "%s: enter\n", __func__);
	if (!node) {
		dev_err(&pdev->dev, "no platform data\n");
		return -ENOENT;
	}

	emt_data = devm_kzalloc(&pdev->dev, sizeof(struct emt_data_t),
				GFP_KERNEL);
	if (!emt_data)
		return -ENOMEM;

	emt_data->msg = vzalloc(EMT_MSG_TOTAL_PAGES * PAGE_SIZE);
	if (emt_data->msg == NULL)
		return -ENOMEM;

	mutex_init(&emt_data->aon_data.lock);

	platform_set_drvdata(pdev, emt_data);
	ret = emt_register_cdev(pdev);
	if (ret)
		goto exit_unreg_cdev;

	init_completion(&emt_data->aon_data.wait_resp_received);
	emt_data->aon_data.cl.dev = &pdev->dev;
	emt_data->aon_data.cl.tx_block = true;
	emt_data->aon_data.cl.tx_tout = EMT_TX_BLOCK_PERIOD_MS;
	emt_data->aon_data.cl.knows_txdone = false;
	emt_data->aon_data.cl.rx_callback = emt_aon_receive_ivc_msg;
	emt_data->aon_data.mbox = mbox_request_channel(&emt_data->aon_data.cl,
						       0);
	if (IS_ERR(emt_data->aon_data.mbox)) {
		dev_err(&pdev->dev, "can't get mailbox channel (%d)\n",
			(int)PTR_ERR(emt_data->aon_data.mbox));
		ret = PTR_ERR(emt_data->aon_data.mbox);
		goto exit_unreg_cdev;
	}
	dev_dbg(&pdev->dev, "emt_data->mbox = %p\n", emt_data->aon_data.mbox);
	return 0;

exit_unreg_cdev:
	emt_unregister_cdev(pdev);
	mutex_destroy(&emt_data->aon_data.lock);
	if (emt_data->msg)
		vfree(emt_data->msg);

	return ret;
}

static int emt_aon_remove(struct platform_device *pdev)
{
	struct emt_data_t     *emt_data;

	emt_data =  dev_get_drvdata(&pdev->dev);
	dev_info(&pdev->dev, "%s\n", __func__);
	mbox_free_channel(emt_data->aon_data.mbox);
	emt_unregister_cdev(pdev);
	mutex_destroy(&emt_data->aon_data.lock);
	if (emt_data->msg)
		vfree(emt_data->msg);

	return 0;
}

static const struct of_device_id emt_aon_of_match[] = {
	{
		.compatible = "ml,aon-em-raw",
	},
	{}
};
MODULE_DEVICE_TABLE(of, emt_aon_of_match);

static struct platform_driver emt_aon_driver = {
	.driver = {
		.name       = "em-raw",
		.owner      = THIS_MODULE,
		.of_match_table = of_match_ptr(emt_aon_of_match),
	},
	.remove =   emt_aon_remove,
	.probe =    emt_aon_probe,
};
module_platform_driver(emt_aon_driver);

MODULE_DESCRIPTION("Linux driver to communicate with EM tracking sensor");
MODULE_AUTHOR("Magic Leap LLC");
MODULE_LICENSE("GPL v2");

