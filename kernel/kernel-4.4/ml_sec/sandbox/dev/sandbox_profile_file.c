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

#include "sandbox_profile_file.h"
#include "profile_file.h"

#define SANDBOX_PROFILE_EXT "sandbox.profile"

static bool parse_filter_entry(const char *name, void *filter)
{
#define _SANDBOX_FILTER(fname, str)	\
	if (strcmp(name, str) == 0) {				\
		*(int *)filter |= SANDBOX_FILTER_##fname;	\
		return true;					\
	}
#include "sandbox_filters.def"

	return false;
}

int sbox_create_sandbox_filter_from_file(struct path *exec_path)
{
	int filter = 0;
	int error = sbox_parse_profile_file(exec_path, SANDBOX_PROFILE_EXT, parse_filter_entry, &filter);
	if (error < 0)
		filter = error;
	return filter;
}

bool sbox_parse_sandbox_filter_to_string(int filter, char *buf, size_t size)
{
	bool retval = true;

	if (buf == NULL || size == 0)
		return false;

#define _SANDBOX_FILTER(name, str)	\
	if (filter & SANDBOX_FILTER_##name) {			\
		size_t str_len = sizeof(str) - 1;		\
		if ((str_len + 1) <= size) {			\
			memcpy(buf, str, str_len);		\
			buf += str_len;				\
			*buf++ = '|';				\
			size -= str_len + 1;			\
		}						\
		else						\
			retval = false;				\
	}
#include "sandbox_filters.def"

	if (filter)
		buf--;
	*buf = '\0';
	return retval;
}
