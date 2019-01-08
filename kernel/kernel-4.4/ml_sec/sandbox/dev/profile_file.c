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

/* HOSTPROG is defined only when the file is compiled for a host program. */
#ifndef HOSTPROG
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/namei.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#endif

#ifdef CONFIG_MLSEC_EMULATOR
/*
 * this is to overcome the fact that vfs_path_lookup is not defined
 * in <linux/namei.h>@goldfish-4.4, while it does in kernel-4.4
 */
#include "../../fs/internal.h"
#endif

#include "profile_file.h"

/* Open "exec_path.ext" */
static struct file *open_file_ext(struct path *exec_path, const char *ext)
{
	int error;
	struct path path;
	struct file *file;

	const char *exec_filename = (const char *)exec_path->dentry->d_name.name;
	char filename[256];
	size_t len, ext_len;

	ext_len = ext ? strlen(ext) : 0;
	len = exec_filename ? strlen(exec_filename) : 0;
	if (len != 0)
		len++;

	if ((len + ext_len) >= sizeof(filename))
		return ERR_PTR(-EINVAL);

	if (len != 0) {
		memcpy(filename, exec_filename, len - 1);
		filename[len - 1] = '.';
	}
	if (ext_len != 0)
		memcpy(filename + len, ext, ext_len);
	filename[len + ext_len] = '\0';

	error = vfs_path_lookup(exec_path->dentry->d_parent, exec_path->mnt,
				filename, LOOKUP_FOLLOW, &path);
	if (error)
		return ERR_PTR(error);

	file = dentry_open(&path, O_RDONLY, current_cred());
	path_put(&path);
	return file;
}

/* Read file in "exec_path.ext" into text/len (includes NULL-termination) */
/* When done, call kfree(*text) */
static int read_text_file(struct path *exec_path, const char *ext, char **text, size_t *len)
{
	int error;
	struct file *file;
	loff_t filelen;
	char *buf;

	file = open_file_ext(exec_path, ext);
	if (IS_ERR(file))
		return PTR_ERR(file);

	filelen = vfs_llseek(file, 0, SEEK_END);
	if (filelen < 0) {
		error = (int)filelen;
		goto out;
	}

	/*
	 * Allocate the text buffer to store the file content in.
	 * Add 1 for the (string) terminating zero.
	 */
	buf = kmalloc((size_t)filelen + 1, GFP_KERNEL);
	if (!buf) {
		error = -ENOMEM;
		goto out;
	}

	error = kernel_read(file, 0, buf, filelen);
	if (error < 0) {
		kfree(buf);
		goto out;
	}

	buf[filelen] = 0; /* terminate the string */

	/*
	 * Set return values.
	 */
	*len = (size_t)filelen;
	*text = buf;

out:
	fput(file);
	return error;
}

/* Returns the next whitespace-delimited word in str */
static char *get_next_word(char *str, char **end)
{
	char *str_end;

	while (*str && isspace(*str))
		str++;

	str_end = str;
	while (*str_end && !isspace(*str_end))
		str_end++;

	*str_end = 0;
	*end = str_end;
	return str;
}

int sbox_parse_profile_file(struct path *exec_path, const char *ext,
			    bool (*parse_entry)(const char *str, void *user_data), void *user_data)
{
	int count;
	char *text = NULL;
	char *text_end, *ptr, *next;
	size_t len = 0;

	count = read_text_file(exec_path, ext, &text, &len);
	if (count < 0)
		return count;

	text_end = text + len;
	ptr = text;
	count = 0;
	while (ptr < text_end) {
		ptr = get_next_word(ptr, &next);
		if (*ptr) {
			if (parse_entry(ptr, user_data))
				count++;
		}
		ptr = next + 1;
	}

	kfree(text);
	return count;
}
