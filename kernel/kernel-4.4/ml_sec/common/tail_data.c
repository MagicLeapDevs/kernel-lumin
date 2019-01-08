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

#include <linux/ml_sec/tail_data.h>
#include <linux/ml_sec/module_tools.h>

#ifdef CONFIG_MLSEC_ALLOW_PERMISSIVE_DEBUGGING
#include <linux/ml_sec/debugharden_debugfs.h>
#endif

bool is_debuggable(struct file *file)
{
	struct ml_tail_data taildata;

#ifdef CONFIG_MLSEC_ALLOW_PERMISSIVE_DEBUGGING
	/* If permissive debugging has been enabled - everything is debuggable */
	if (g_is_debug_permissive)
		return true;
#endif

	if (read_ml_tail_data(file, &taildata))
		return false;

	return taildata.is_debuggable;
}

int read_ml_tail_data(struct file *file, struct ml_tail_data *tail_data)
{
	int retval = -ENOKEY;
	loff_t pos;
	ssize_t len;
	struct ml_tail_data_ctrl tail_data_ctrl = {0};
	off_t taildata_ctrl_pos;
	off_t taildata_pos;

	/* Find start of signature data */
	pos = mod_seek_module_signature(file);
	if (pos < 0)
		/* No signature data - use EOF as starting position */
		pos = i_size_read(file_inode(file));

	/* Verify the tail data */
	if (pos < sizeof(struct ml_tail_data_ctrl))
		return retval;

	taildata_ctrl_pos = pos - sizeof(struct ml_tail_data_ctrl);

	len = kernel_read(file,
					taildata_ctrl_pos,
					(char *)&tail_data_ctrl,
					sizeof(struct ml_tail_data_ctrl));

	if (len != sizeof(struct ml_tail_data_ctrl)) {
		if (len < 0)
			retval = (int)len;
		return retval;
	}

	if (memcmp(&tail_data_ctrl.tail_data_magic,
			ML_TAIL_DATA_MAGIC_STRING,
			sizeof(tail_data_ctrl.tail_data_magic)) != 0)
		return retval;

	if (tail_data_ctrl.taildata_ver != ML_TAIL_DATA_LATEST_VERSION) {
		pr_err("ml_sec: file has an old taildata version (%s)\n",
			file->f_path.dentry->d_name.name);
		return retval;
	}

	/* Skip back to beginning of tail data */
	if (pos < sizeof(struct ml_tail_data))
		return retval;

	taildata_pos = pos - sizeof(struct ml_tail_data);

	len = kernel_read(file,
					taildata_pos,
					(char *)tail_data,
					sizeof(struct ml_tail_data));

	if (len != sizeof(struct ml_tail_data)) {
		if (len < 0)
			retval = (int)len;
		return retval;
	}

	return 0;
}
