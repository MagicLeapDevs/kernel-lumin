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

static struct dentry *g_randstruct_file;

/*
 * Test offset of (supposedly) first element in task_struct.
 * This test relies on the fact that "state" is the first element defined in task_struct.
 * If "state" was moved and is not the first element - randomization is definitely on.
 * Note that there might be a false negative, if "state" element wasn't moved in randomization.
 */
static int randstruct_test_read(void *data, u64 *val)
{
	if (offsetof(struct task_struct, state) != 0)
		/* "state" isn't the first element, randomization is definitely on */
		*val = 1;
	else
		/* "state" is the first element, randomization is almost definitely off */
		*val = 0;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(randstruct_test_fops, randstruct_test_read, NULL, "%llu\n");

static void randstruct_test_exit(void)
{
	debugfs_remove(g_randstruct_file);
}

static int randstruct_test_init(void)
{
	g_randstruct_file = debugfs_create_file("randstruct_test",
						0400,
						g_mlsec_debugfs_dir,
						NULL,
						&randstruct_test_fops);
	if (g_randstruct_file == NULL)
		return -ENOMEM;

	return 0;
}

module_init(randstruct_test_init);
module_exit(randstruct_test_exit);
MODULE_LICENSE("GPL");