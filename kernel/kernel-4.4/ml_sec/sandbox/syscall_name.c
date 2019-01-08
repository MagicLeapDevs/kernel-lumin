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

#include <linux/string.h>
#include <asm/unistd_ext.h>
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)	(sizeof(arr) / sizeof((arr)[0]))
#endif

#define _INIT_STR_LEN_STRUCT(s)	{ .str = #s, .len = (sizeof(#s) - 1) }

static const struct {
	char const * const str;
	const size_t len;
} syscall_names[__X_NR_syscalls] = {
#undef  __SYSCALL
#define __SYSCALL(nr, sym) \
	[nr] = _INIT_STR_LEN_STRUCT(sym),

#include <asm/unistd_ext.h>
};

static const struct {
	char const * const str;
	const size_t len;
} name_prefixes[] = {
	/*
	 * IMPORTANT: The prefixes must have a different length
	 * and in ascending order.
	 */
	_INIT_STR_LEN_STRUCT(sys_),
	_INIT_STR_LEN_STRUCT(sys_new),
#ifdef __COMPAT_SYSCALL_NR
	_INIT_STR_LEN_STRUCT(compat_sys_),
	_INIT_STR_LEN_STRUCT(compat_sys_new),
#endif
};

const char *get_syscall_name(int x_nr)
{
	const char *str;
	size_t len;
	size_t i;

	if ((unsigned int)x_nr >= __X_NR_syscalls)
		return NULL;

	str = syscall_names[x_nr].str;
	len = syscall_names[x_nr].len;

	for (i = 0; i < ARRAY_SIZE(name_prefixes); ++i) {
		const char *pre_str = name_prefixes[i].str;
		size_t pre_len = name_prefixes[i].len;

		/*
		 * We do not need to continue comparison with other prefixes,
		 * as they are all longer than this.
		 */
		if (len <= pre_len)
			break;

		if (!memcmp(str, pre_str, pre_len))
			return (str + pre_len);
	}
	return str;
}

int lookup_syscall_nr(const char *name)
{
	size_t name_len;
	int x_nr, i;

	if (!name || !*name)
		return -1;

	name_len = strlen(name);
	for (x_nr = 0; x_nr < __X_NR_syscalls; ++x_nr) {
		const char *str = syscall_names[x_nr].str;
		size_t len = syscall_names[x_nr].len;
		if (!len)
			continue;

		for (i = 0; i < ARRAY_SIZE(name_prefixes); ++i) {
			const char *pre_str = name_prefixes[i].str;
			size_t pre_len = name_prefixes[i].len;

			if ((name_len + pre_len) != len)
				continue;

			if (!memcmp(name, str + pre_len, len - pre_len) &&
			    !memcmp(pre_str, str, pre_len))
				return x_nr;

			/* No need to look further, as all prefixes are of different sizes. */
			break;
		}
	}
	return -1;
}
