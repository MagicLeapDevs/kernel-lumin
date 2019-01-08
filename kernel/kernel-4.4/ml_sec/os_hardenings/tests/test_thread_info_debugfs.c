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

#define OLD_THREAD_INFO_LOCATION(sp) ((unsigned long)(sp) & ~(THREAD_SIZE - 1))

static struct dentry *g_thread_info_file;

static int thread_info_test_read(void *data, u64 *val)
{
	struct thread_info *old_thread_info = (struct thread_info *)OLD_THREAD_INFO_LOCATION(&data);
	int old_value = old_thread_info->preempt_count;
	old_thread_info->preempt_count = -1;

	/*
	 * If we had overwritten thread_info with the assignment above,
	 * then preempt_count should be -1. Otherwise, it's not
	 * on the stack and its value should be positive
	 */
	*val = current_thread_info()->preempt_count != -1;
	old_thread_info->preempt_count = old_value;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(thread_info_test_fops, thread_info_test_read, NULL, "%llu\n");

static void thread_info_test_exit(void)
{
	debugfs_remove(g_thread_info_file);
}

static int thread_info_test_init(void)
{
	g_thread_info_file = debugfs_create_file("thread_info_test",
						0400,
						g_mlsec_debugfs_dir,
						NULL,
						&thread_info_test_fops);
	if (g_thread_info_file == NULL)
		return -ENOMEM;

	return 0;
}

module_init(thread_info_test_init);
module_exit(thread_info_test_exit);
MODULE_LICENSE("GPL");
