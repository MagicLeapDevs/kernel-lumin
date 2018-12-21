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
#include "signing_dev.h"

static struct dentry *g_signing_file;

static void signing_debugfs_exit(void)
{
	debugfs_remove(g_signing_file);
}

static int signing_debugfs_init(void)
{
	g_signing_file = debugfs_create_bool("signing_permissive",
					     0600,
					     g_mlsec_debugfs_dir,
					     &g_signing_dev_is_permissive);
	if (g_signing_file == NULL)
		return -ENOMEM;

	return 0;
}

module_init(signing_debugfs_init);
module_exit(signing_debugfs_exit);
MODULE_LICENSE("GPL");
