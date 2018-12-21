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

#ifndef __MLSEC_SBOX_SANDBOX_PROFILE_H
#define __MLSEC_SBOX_SANDBOX_PROFILE_H

#include <linux/types.h>
#ifdef CONFIG_MLSEC_SANDBOX_DEVELOP
#include <linux/atomic.h>
#endif

struct sandbox_profile {
	/* ProfileTag that identifies this sandbox profile */
	uint32_t profile_tag;

#ifdef CONFIG_MLSEC_SANDBOX_DEVELOP
	atomic_t usage;
#endif

	union {
		struct {
			/* Should NO_NEW_PRIVS be enabled */
			bool is_no_new_privs : 1;

			/* Enforce process' jail */
			bool enforce_jail : 1;

			/* Should the kernel enable mapping RWX sections */
			bool enable_jit : 1;

			/* Is syscall filtering permissive */
			bool permissive_syscalls : 1;
		};
		uint64_t flags;
	};

	/* Syscalls white-list filter to be applied */
	const uint8_t *syscall_filter;
#ifdef CONFIG_COMPAT
	const uint8_t *syscall_filter_compat;
#endif
} __attribute__((aligned(16)));

const struct sandbox_profile *sbox_lookup_profile(uint32_t tag);

#endif	/* __MLSEC_SBOX_SANDBOX_PROFILE_H */
