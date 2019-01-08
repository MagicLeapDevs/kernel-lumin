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
#include "sandbox_dev.h"

static struct dentry *g_sandbox_file;
static struct dentry *g_jail_file;

static void sandbox_debugfs_exit(void)
{
	debugfs_remove(g_jail_file);
	debugfs_remove(g_sandbox_file);
}

static int sandbox_debugfs_init(void)
{
	g_sandbox_file = debugfs_create_bool("sandbox_permissive",
					     S_IRUSR | S_IWUSR,
					     g_mlsec_debugfs_dir,
					     &g_sandbox_dev_is_permissive);
	if (g_sandbox_file == NULL)
		return -ENOMEM;

	g_jail_file = debugfs_create_bool("jail_permissive",
					  S_IRUGO | S_IWUSR,
					  g_mlsec_debugfs_dir,
					  &g_sandbox_jail_is_permissive);
	if (g_jail_file == NULL)
		return -ENOMEM;

	return 0;
}

module_init(sandbox_debugfs_init);
module_exit(sandbox_debugfs_exit);
MODULE_LICENSE("GPL");
