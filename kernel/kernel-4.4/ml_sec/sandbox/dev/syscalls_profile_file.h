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

#ifndef __MLSEC_SBOX_DEV_SYSCALLS_PROFILE_FILE_H
#define __MLSEC_SBOX_DEV_SYSCALLS_PROFILE_FILE_H

#include <linux/path.h>
#include <linux/ml_sec/sandbox_profile.h>

int sbox_create_syscall_filter_from_file(struct path *exec_path, struct sandbox_profile *sandbox_profile);

#endif	/* __MLSEC_SBOX_DEV_SYSCALLS_PROFILE_FILE_H */
