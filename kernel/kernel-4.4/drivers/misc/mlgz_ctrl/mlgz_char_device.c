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

static dev_t dev_number;
static struct class *mlgz_class;

static DECLARE_WAIT_QUEUE_HEAD(queue);
static LIST_HEAD(queued);
static DEFINE_MUTEX(list_mutex);

struct mlgz_ctrl_message {
	struct list_head list;
	uint8_t *data;
	size_t data_len;
};

static int mlgz_open(struct inode *inode, struct file *file)
{
	struct mlgz_dev_data *dev_data;

	pr_info("mlgz_ctrl opened\n");
	dev_data = container_of(inode->i_cdev,
				struct mlgz_dev_data, mlgz_c_dev);
	file->private_data = dev_data;
	return 0;
}

void mlgz_char_device_queue_received_data(const uint8_t *data,
			size_t len)
{
	uint8_t *mlgz_buffer;
	struct mlgz_ctrl_message *msg;

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

	mutex_lock(&list_mutex);
	list_add_tail(&msg->list, &queued);
	mutex_unlock(&list_mutex);

	/* Notify functions waiting on the queue */
	wake_up_interruptible(&queue);
}

static ssize_t mlgz_write(struct file *file, const char *buf,
							size_t len, loff_t *pos)
{
	char *kbuf = NULL;
	ssize_t ret = 0;

	struct mlgz_dev_data *dev_data = file->private_data;

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
	size_t ret = 0;

	if (file->f_flags & O_NONBLOCK) {
		pr_info("mlgz_ctrl: Nonblock read\n");
		if (list_empty(&queued))
			return 0;
	}

	pr_info("mlgz_ctrl: Blocking read until there is data\n");

	/* Wait here until something is in the queue */
	if (wait_event_interruptible(queue, !list_empty(&queued))
		== -ERESTARTSYS)
		return -ERESTARTSYS;

	mutex_lock(&list_mutex);

	if (list_empty(&queued)) {
		mutex_unlock(&list_mutex);
		return 0;
	}

	msg = list_entry(queued.next, struct mlgz_ctrl_message, list);

	if (len >= msg->data_len)
		len = msg->data_len;

	ret = copy_to_user(buf, msg->data, len);
	if (ret == 0) {
		pr_debug("mlgz_ctrl: Sent %zu characters to the user\n", len);

		list_del(&msg->list);
		mlgz_ctrl_command_buffer_free(msg->data);
		kfree(msg);

		*pos += len;
	} else
		pr_err("mlgz_ctrl: Failed to send %zu characters to the user\n",
				ret);

	mutex_unlock(&list_mutex);
	return ret ? -EFAULT : len;
}

static int mlgz_release(struct inode *inode, struct file *file)
{
	/*TODO:*/
	pr_info("mlgz_ctrl release\n");
	return 0;
}


static const struct file_operations mlgz_fops = {
	.owner = THIS_MODULE,
	.open = mlgz_open,
	.write = mlgz_write,
	.read = mlgz_read,
	.release = mlgz_release
};

int mlgz_char_device_init(struct mlgz_dev_data *dev_data)
{
	struct device *mlgz_device;

	pr_info("Initializing ML Gazell char device\n");
	if (alloc_chrdev_region(&dev_number, 0, 1, DEVICE_NAME) < 0) {
		pr_err("Can't register device\n");
		return -1;
	}

	cdev_init(&dev_data->mlgz_c_dev, &mlgz_fops);

	if (cdev_add(&dev_data->mlgz_c_dev, dev_number, 1) < 0) {
		pr_err("failed to add char device\n");
		goto error_cleanup_1;
	}

	mlgz_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(mlgz_class)) {
		pr_err("failed to create class\n");
		goto error_cleanup_2;
	}

	mlgz_device = device_create(mlgz_class, NULL, dev_number,
				NULL, DEVICE_NAME);

	if (IS_ERR(mlgz_device))
		goto error_cleanup_3;

	pr_info("registered device major: %u, minor: %u\n",
			MAJOR(dev_number), MINOR(dev_number));
	return 0;

error_cleanup_3:
	class_destroy(mlgz_class);
error_cleanup_2:
	cdev_del(&dev_data->mlgz_c_dev);
error_cleanup_1:
	unregister_chrdev_region(dev_number, 1);
	return -1;
}
void mlgz_char_device_exit(struct mlgz_dev_data *dev_data)
{
	pr_info("Unloading ML Gazell Module\n");

	/* Notify functions waiting on the queue */
	wake_up_interruptible(&queue);

	device_destroy(mlgz_class, dev_number);
	class_destroy(mlgz_class);
	unregister_chrdev_region(dev_number, 1);
	cdev_del(&dev_data->mlgz_c_dev);
}




