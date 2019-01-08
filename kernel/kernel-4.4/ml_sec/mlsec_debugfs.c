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

struct dentry *g_mlsec_debugfs_dir;
EXPORT_SYMBOL(g_mlsec_debugfs_dir);

static void mlsec_debugfs_exit(void)
{
	debugfs_remove(g_mlsec_debugfs_dir);
}

static int mlsec_debugfs_init(void)
{
	g_mlsec_debugfs_dir = debugfs_create_dir("ml_sec", NULL);
	if (g_mlsec_debugfs_dir == NULL)
		return -ENOMEM;

	return 0;
}

module_init(mlsec_debugfs_init);
module_exit(mlsec_debugfs_exit);
MODULE_LICENSE("GPL");
