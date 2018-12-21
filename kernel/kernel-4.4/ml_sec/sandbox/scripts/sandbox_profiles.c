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

#include <linux/ml_sec/sandbox_profile.h>
#include <linux/kernel.h>
#include <linux/bug.h>

#include "syscall_filters.c"

#ifdef CONFIG_COMPAT
#include "syscall_filters_compat.c"
#endif

#ifdef CONFIG_MLSEC_SANDBOX_DEVELOP
#define CONST
#define USAGE_INIT(perm)	.usage = ATOMIC_INIT(1), \
				.permissive_syscalls = perm,
#else
#define CONST			const
#define USAGE_INIT(perm)	.permissive_syscalls = perm,
#endif

static CONST struct sandbox_profile sandbox_profiles[] = {
#include "sandbox_profiles.inc"
};

const struct sandbox_profile *sbox_lookup_profile(uint32_t tag)
{
	size_t i;
	for (i = 0; i < ARRAY_SIZE(sandbox_profiles); ++i) {
		if (sandbox_profiles[i].profile_tag == tag)
			return &sandbox_profiles[i];
	}
	return NULL;
}
