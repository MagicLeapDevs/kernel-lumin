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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include "mlgz_buffer.h"
#include "mlgz_char_device.h"
#include "mlgz.h"

#define DEVICE_NAME     "mlgz_ctrl"
#define CLASS_NAME      "mlgz"
#define NUM_MINORS 1

enum mlgz_cdev_state {
	MLGZ_CDEV_READY = 0,
	MLGZ_CDEV_DESTROYED,
};

struct mlgz_cdev_data {
	struct mlgz_dev_data *dev_data;
	struct list_head queued;
	struct mutex list_mutex;
	wait_queue_head_t read_wq;
	struct cdev cdev;
	struct device *device;
	struct kref kref;
	int num;
	enum mlgz_cdev_state state;
};

static dev_t dev_number;
static struct class *mlgz_class;
static DEFINE_SPINLOCK(mlgz_lock);
static uint8_t cdev_count;

struct mlgz_ctrl_message {
	struct list_head list;
	uint8_t *data;
	size_t data_len;
};

static inline void mlgz_cdev_get(struct mlgz_cdev_data *cdev_data)
{
	unsigned long flags;

	pr_debug("mlgz_ctrl increment kref %d\n", cdev_data->num);
	spin_lock_irqsave(&mlgz_lock, flags);
	kref_get(&cdev_data->kref);
	spin_unlock_irqrestore(&mlgz_lock, flags);
}

static void mlgz_cdev_kref_release(struct kref *kref)
{
	struct mlgz_cdev_data *cdev_data;

	cdev_data = container_of(kref, struct mlgz_cdev_data, kref);
	pr_debug("mlgz_ctrl freeing cdev data %d\n", cdev_data->num);
	mutex_destroy(&cdev_data->list_mutex);
	kfree(cdev_data);
}

static inline void mlgz_cdev_put(struct mlgz_cdev_data *cdev_data)
{
	unsigned long flags;

	pr_debug("mlgz_ctrl decrement kref %d\n", cdev_data->num);
	spin_lock_irqsave(&mlgz_lock, flags);
	kref_put(&cdev_data->kref, mlgz_cdev_kref_release);
	spin_unlock_irqrestore(&mlgz_lock, flags);
}

static int mlgz_open(struct inode *inode, struct file *file)
{
	struct mlgz_cdev_data *cdev_data;

	cdev_data = container_of(inode->i_cdev,
				struct mlgz_cdev_data, cdev);
	pr_info("mlgz_ctrl opened %d\n", cdev_data->num);
	mlgz_cdev_get(cdev_data);
	file->private_data = cdev_data;

	return 0;
}

void mlgz_char_device_queue_received_data(const uint8_t *data,
			size_t len, struct mlgz_dev_data *dev_data)
{
	uint8_t *mlgz_buffer;
	struct mlgz_ctrl_message *msg;
	struct mlgz_cdev_data *cdev_data = dev_data->mlgz_cdev_data;

	/* For now we only have one queue, not sure if in the future
	 * if we ever have multiple char devices, this needs to be fixed
	 * by creating a queue per device
	 */
	if (len > MLGZ_CTRL_CMD_MAX_BUFFER_SIZE) {
		pr_err("Invalid length\n");
		return;
	}

	mlgz_buffer = mlgz_buffer_alloc();

	if (!mlgz_buffer) {
		pr_err("No memory mlgz_buffer\n");
		return;
	}

	msg = kmalloc(sizeof(struct mlgz_ctrl_message), GFP_KERNEL);
	if (!msg) {
		mlgz_ctrl_command_buffer_free(mlgz_buffer);
		return;
	}
	msg->data = mlgz_buffer;
	msg->data_len = len;    /*not used for now*/
	memcpy(mlgz_buffer, data, len);

	mutex_lock(&cdev_data->list_mutex);
	list_add_tail(&msg->list, &cdev_data->queued);
	mutex_unlock(&cdev_data->list_mutex);

	/* Notify functions waiting on the queue */
	wake_up_interruptible(&cdev_data->read_wq);
}

static ssize_t mlgz_write(struct file *file, const char *buf,
							size_t len, loff_t *pos)
{
	char *kbuf = NULL;
	ssize_t ret = 0;
	struct mlgz_cdev_data *cdev_data = file->private_data;
	struct mlgz_dev_data *dev_data = cdev_data->dev_data;

	/* Get data from user space */
	kbuf = kmalloc(len, GFP_KERNEL);

	if (kbuf == NULL)
		return -ENOMEM;

	ret = copy_from_user((void *)kbuf, (const void *)buf, len);

	if (ret != 0) {
		pr_err("mlgz_ctrl: Failed to read [%zu] bytes from user space.\n",
				ret);
		ret = -EFAULT;
	} else {
		ret = mlgz_mux_send_data(dev_data, kbuf, len);
	}

	kfree(kbuf);
	return ret;
}

static ssize_t mlgz_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
	struct mlgz_ctrl_message *msg;
	int ret = 0;
	struct mlgz_cdev_data *cdev_data = file->private_data;

	if (file->f_flags & O_NONBLOCK) {
		pr_debug("mlgz_ctrl: Nonblock read\n");
		if (list_empty(&cdev_data->queued))
			goto exit;
	}

	pr_debug("mlgz_ctrl: Blocking read until there is data\n");

	/* Wait here until something is in the queue */
	ret = wait_event_interruptible(cdev_data->read_wq,
		!list_empty(&cdev_data->queued) ||
		cdev_data->state == MLGZ_CDEV_DESTROYED);
	if (ret < 0)
		return ret;
	if (cdev_data->state == MLGZ_CDEV_DESTROYED)
		return -ENOTCONN;

	mutex_lock(&cdev_data->list_mutex);
	if (list_empty(&cdev_data->queued))
		goto exit;

	msg = list_entry(cdev_data->queued.next, struct mlgz_ctrl_message,
			 list);

	if (len >= msg->data_len)
		len = msg->data_len;

	ret = copy_to_user(buf, msg->data, len);
	if (ret) {
		pr_err("mlgz_ctrl: Failed to send %d chars to user\n", ret);
		goto exit;
	}

	pr_debug("mlgz_ctrl: Sent %zu characters to the user\n", len);
	list_del(&msg->list);
	mlgz_ctrl_command_buffer_free(msg->data);
	kfree(msg);

	*pos += len;

exit:
	mutex_unlock(&cdev_data->list_mutex);
	return ret ? -EFAULT : len;
}

static int mlgz_release(struct inode *inode, struct file *file)
{
	struct mlgz_cdev_data *cdev_data = file->private_data;

	pr_info("mlgz_ctrl release %d\n", cdev_data->num);
	mlgz_cdev_put(cdev_data);

	return 0;
}

static void mlgz_device_release(struct device *dev)
{
	struct mlgz_cdev_data *cdev_data = dev_get_drvdata(dev);

	if (!cdev_data) {
		pr_info("mlgz_device_release: nothing to release\n");
		return;
	}
	pr_info("mlgz_ctrl device release %d\n", cdev_data->num);
	mlgz_cdev_put(cdev_data);
	kfree(dev);
}


static const struct file_operations mlgz_fops = {
	.owner = THIS_MODULE,
	.open = mlgz_open,
	.write = mlgz_write,
	.read = mlgz_read,
	.release = mlgz_release
};

int mlgz_char_device_add(struct mlgz_dev_data *dev_data)
{
	struct mlgz_cdev_data *cdev_data;
	int ret = 0;

	cdev_data = kzalloc(sizeof(struct mlgz_cdev_data), GFP_KERNEL);
	if (!cdev_data)
		return -ENOMEM;

	kref_init(&cdev_data->kref);
	cdev_data->num = ++cdev_count;
	pr_info("mlgz_char_device_add %d\n", cdev_data->num);
	cdev_data->dev_data = dev_data;
	cdev_data->state = MLGZ_CDEV_READY;

	INIT_LIST_HEAD(&cdev_data->queued);
	mutex_init(&cdev_data->list_mutex);

	cdev_data->device = kzalloc(sizeof(struct device), GFP_KERNEL);
	if (!cdev_data->device) {
		ret = -ENOMEM;
		goto error_devalloc;
	}

	cdev_data->device->devt = dev_number;
	cdev_data->device->class = mlgz_class;
	cdev_data->device->parent = NULL;
	cdev_data->device->release = mlgz_device_release;
	dev_set_name(cdev_data->device, DEVICE_NAME);
	device_initialize(cdev_data->device);

	init_waitqueue_head(&cdev_data->read_wq);
	cdev_init(&cdev_data->cdev, &mlgz_fops);
	cdev_data->cdev.kobj.parent = &cdev_data->device->kobj;
	ret = cdev_add(&cdev_data->cdev, dev_number, 1);
	if (ret < 0) {
		pr_err("failed to add char device\n");
		goto error_cdev;
	}

	ret = device_add(cdev_data->device);
	if (ret) {
		pr_err("failed to add device\n");
		goto error_device;
	}
	dev_set_drvdata(cdev_data->device, cdev_data);

	dev_data->mlgz_cdev_data = (void *)cdev_data;
	pr_info("registered device major: %u, minor: %u\n",
			MAJOR(dev_number), MINOR(dev_number));

	return 0;

error_device:
	cdev_del(&cdev_data->cdev);
error_cdev:
	put_device(cdev_data->device);
	kfree(cdev_data->device);
error_devalloc:
	mutex_destroy(&cdev_data->list_mutex);
	kfree(cdev_data);

	return ret;
}

void mlgz_char_device_remove(struct mlgz_dev_data *dev_data)
{
	struct mlgz_cdev_data *cdev_data;
	struct mlgz_ctrl_message *ctrl_msg;
	struct mlgz_ctrl_message *temp;

	cdev_data = (struct mlgz_cdev_data *)dev_data->mlgz_cdev_data;
	if (!cdev_data)
		return;
	pr_info("mlgz_char_device_remove %d\n", cdev_data->num);

	cdev_data->state = MLGZ_CDEV_DESTROYED;

	wake_up_interruptible(&cdev_data->read_wq);

	cdev_del(&cdev_data->cdev);
	device_del(cdev_data->device);

	mutex_lock(&cdev_data->list_mutex);
	list_for_each_entry_safe(ctrl_msg, temp, &cdev_data->queued, list) {
		list_del(&ctrl_msg->list);
		kfree(ctrl_msg);
	}
	mutex_unlock(&cdev_data->list_mutex);
	put_device(cdev_data->device);
}

int mlgz_char_device_init(void)
{
	dev_t dev;
	int ret;

	pr_info("mlgz_char_device_init\n");
	mlgz_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(mlgz_class)) {
		ret = PTR_ERR(mlgz_class);
		pr_err("failed to create class\n");
		goto error_class;
	}

	ret = alloc_chrdev_region(&dev, 0, NUM_MINORS, DEVICE_NAME);
	if (ret < 0) {
		pr_err("cannot register device\n");
		goto error_chrdev;
	}

	dev_number = dev;

	return 0;

error_chrdev:
	class_destroy(mlgz_class);
error_class:
	return ret;
}

void mlgz_char_device_exit(void)
{
	pr_info("mlgz_char_device_exit\n");
	unregister_chrdev_region(dev_number, NUM_MINORS);
	class_destroy(mlgz_class);
}
