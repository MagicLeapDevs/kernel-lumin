/*
 * Copyright (c) 2017, Magic Leap, Inc. All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ml_sec/mlsec_debugfs.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

static struct dentry *g_usercopy_file;

static int usercopy_test_read(void *data, u64 *val)
{
	*val = 0;

	return 0;
}

static int usercopy_test_ops_open(struct inode *inode, struct file *file)
{
	return simple_attr_open(inode, file, usercopy_test_read, NULL, "%llu\n");
}

long (*usercopy_test_never_overriden_non_const_size)(long size);

static long non_const_size(long size)
{
	if (usercopy_test_never_overriden_non_const_size)
		return usercopy_test_never_overriden_non_const_size(size);

	return size;
}

static long usercopy_unlocked_ioctl(struct file *file, unsigned int cmd,
				    unsigned long ptr)
{
	long ret = -1;

	printk(KERN_DEBUG "usercopy test %d\n", cmd);

	switch (cmd) {
	case 0x1000: {
		size_t n = KMALLOC_MIN_SIZE + 1;
		void *obj = kmalloc(KMALLOC_MIN_SIZE, GFP_KERNEL);

		/* Should crash - copying more than the size of the object */
		ret = copy_to_user((void *)ptr, obj, non_const_size(n));
		kfree(obj);
		break;
	}
	case 0x1001: {
		long stack_var;

		/* Should crash - copying from user while trashing the stack  */
		ret = copy_from_user((void *)&stack_var, (void *)ptr,
				      non_const_size(THREAD_SIZE + 0x1000));
		break;
	}
	case 0x1002: {
		/* Should crash - copying directly from .text */
		ret = copy_to_user((void *)ptr, (void *)kmalloc,
				   non_const_size(0x20));
		break;
	}
	}

	return ret;
}

static const struct file_operations usercopy_test_fops = {
	.owner   = THIS_MODULE,
	.open    = usercopy_test_ops_open,
	.release = simple_attr_release,
	.unlocked_ioctl = usercopy_unlocked_ioctl,
	.read    = simple_attr_read,
	.write   = simple_attr_write,
	.llseek  = generic_file_llseek,
};

static void usercopy_test_exit(void)
{
	debugfs_remove(g_usercopy_file);
}

static int usercopy_test_init(void)
{
	g_usercopy_file = debugfs_create_file("usercopy_test",
					      0600,
					      g_mlsec_debugfs_dir,
					      NULL,
					      &usercopy_test_fops);
	if (g_usercopy_file == NULL)
		return -ENOMEM;

	return 0;
}

module_init(usercopy_test_init);
module_exit(usercopy_test_exit);
MODULE_LICENSE("GPL");
