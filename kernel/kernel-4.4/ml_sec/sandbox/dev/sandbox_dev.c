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

#include <linux/slab.h>
#include <asm/unistd_ext.h>
#include <linux/ml_sec/sandbox.h>
#include "sandbox_dev.h"
#include "sandbox_profile_file.h"
#include "syscalls_profile_file.h"
#include "../sandbox_profile_tags.h"

bool g_sandbox_dev_is_permissive;
bool g_sandbox_jail_is_permissive;

bool sbox_dev_is_permissive(void)
{
	return g_sandbox_dev_is_permissive;
}

bool sbox_jail_is_permissive(void)
{
	return g_sandbox_jail_is_permissive;
}

void sbox_get_sandbox_profile(const struct sandbox_profile *profile)
{
	struct sandbox_profile *orig = (struct sandbox_profile *)profile;
	if (!orig)
		return;
	/* Reference count is bounded by the number of total processes. */
	atomic_inc(&orig->usage);
}

/* Increments the reference count of the sandbox_profile on @tsk */
void sbox_get_profile(struct task_struct *tsk)
{
	sbox_get_sandbox_profile(tsk->sandbox_profile);
}

void sbox_put_sandbox_profile(const struct sandbox_profile *profile)
{
	struct sandbox_profile *orig = (struct sandbox_profile *)profile;
	/* Clean up a single-reference. */
	if (orig && atomic_dec_and_test(&orig->usage))
		kfree(orig);
}

/* Decrements the ref count of tsk->sandbox_profile */
void sbox_put_profile(struct task_struct *tsk)
{
	sbox_put_sandbox_profile(tsk->sandbox_profile);
}

struct sandbox_profile *sbox_create_sandbox_profile_from_file(struct file *file)
{
	struct sandbox_profile *profile;
	int filter_flags;
	size_t size;
	bool syscalls;
#ifdef SBOX_VERBOSE
	char filter_names[64];
#endif

	/* Parse the sandbox runtime filter file */
	filter_flags = sbox_create_sandbox_filter_from_file(&file->f_path);
	if (filter_flags < 0)
		return NULL;

#ifdef SBOX_VERBOSE
	sbox_parse_sandbox_filter_to_string(filter_flags, filter_names, sizeof(filter_names));
	printk(KERN_INFO "sandbox: Read profile from file [%s]\n", filter_names);
#endif

	/*
	 * Allocate enough memory for the sandbox_profile struct and the syscall filters.
	 */
	size = sizeof(struct sandbox_profile);
	syscalls = (filter_flags & (SANDBOX_FILTER_SYSCALLS | SANDBOX_FILTER_SYSCALLS_PERMISSIVE)) != 0;
	if (syscalls) {
		size += SBOX_SYS_FILTER_SIZE(__X_NR_syscalls);
#ifdef CONFIG_COMPAT
		size += SBOX_SYS_FILTER_SIZE(__X_NR_compat_syscalls);
#endif
	}

	profile = kzalloc(size, GFP_KERNEL);
	if (!profile)
		return NULL;

	/* Apply runtime filter to profile */
	profile->profile_tag = 0;
	profile->is_no_new_privs = (filter_flags & SANDBOX_FILTER_NO_NEW_PRIVS) != 0;
	profile->enforce_jail = (filter_flags & SANDBOX_FILTER_JAIL) != 0;
	profile->enable_jit = (filter_flags & SANDBOX_FILTER_ENABLE_JIT) != 0;
	profile->permissive_syscalls = (filter_flags & SANDBOX_FILTER_SYSCALLS_PERMISSIVE) != 0;
	profile->allow_local_connections =
	(filter_flags & SANDBOX_FILTER_ALLOW_LOCAL_CONNECTIONS) != 0;

	/* Check for syscalls runtime filter file - and parse */
	if (syscalls) {
		int error;

		/*
		 * The syscall filters are allocated on the same buffer as the sandbox profile,
		 * and therefore will be freed together with the sandbox profile.
		 */
		profile->syscall_filter = (uint8_t *)profile + sizeof(struct sandbox_profile);
#ifdef CONFIG_COMPAT
		profile->syscall_filter_compat = profile->syscall_filter + SBOX_SYS_FILTER_SIZE(__X_NR_syscalls);
#endif

		error = sbox_create_syscall_filter_from_file(&file->f_path, profile);
		if (error) {
			const struct sandbox_profile *ml_profile;

			/* We don't consider ENOENT an error. */
			if (error != -ENOENT) {
				kfree(profile);
				return NULL;
			}

			/* Fall back to _ML_. */
			ml_profile = sbox_lookup_profile(SBOX_PROFILE_TAG__ML_);
			if (!ml_profile)
				return NULL;

			memcpy((uint8_t *)profile->syscall_filter,
			       ml_profile->syscall_filter,
			       SBOX_SYS_FILTER_SIZE(__X_NR_syscalls));
#ifdef CONFIG_COMPAT
			memcpy((uint8_t *)profile->syscall_filter_compat,
			       ml_profile->syscall_filter_compat,
			       SBOX_SYS_FILTER_SIZE(__X_NR_compat_syscalls));
#endif
		}
	}
	return profile;
}
