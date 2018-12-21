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

#ifndef __MLSEC_SBOX_DEV_SANDBOX_DEV_H
#define __MLSEC_SBOX_DEV_SANDBOX_DEV_H

#include <linux/fs.h>
#include <linux/ml_sec/sandbox_profile.h>

#ifdef CONFIG_MLSEC_SANDBOX_DEVELOP
struct sandbox_profile *sbox_create_sandbox_profile_from_file(struct file *file);
void sbox_get_sandbox_profile(const struct sandbox_profile *profile);
void sbox_put_sandbox_profile(const struct sandbox_profile *profile);
bool sbox_dev_is_permissive(void);
bool sbox_jail_is_permissive(void);
extern bool g_sandbox_dev_is_permissive;
extern bool g_sandbox_jail_is_permissive;
#else
static inline struct sandbox_profile *sbox_create_sandbox_profile_from_file(struct file *file) { return NULL; }
static inline void sbox_get_sandbox_profile(const struct sandbox_profile *profile) {}
static inline void sbox_put_sandbox_profile(const struct sandbox_profile *profile) {}
static inline bool sbox_dev_is_permissive(void) { return false; }
static inline bool sbox_jail_is_permissive(void) { return false; }
#endif

#endif	/* __MLSEC_SBOX_DEV_SANDBOX_DEV_H */
