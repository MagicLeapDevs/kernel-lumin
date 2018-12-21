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

/* Make the compatibility versions of syscalls to be used. */
#define __COMPAT_SYSCALL_NR

/*
 * Change the names of the functions in "syscall_name.c",
 * to create new "compat" versions.
 */
#define syscall_names		syscall_names_compat
#define name_prefixes		name_prefixes_compat
#define get_syscall_name	get_syscall_name_compat
#define lookup_syscall_nr	lookup_syscall_nr_compat

/*
 * Remove the "_wrapper" suffix of the syscall function names.
 */
#define compat_sys_sigreturn_wrapper		compat_sys_sigreturn
#define compat_sys_rt_sigreturn_wrapper		compat_sys_rt_sigreturn
#define compat_sys_pread64_wrapper		compat_sys_pread64
#define compat_sys_pwrite64_wrapper		compat_sys_pwrite64
#define compat_sys_mmap2_wrapper		compat_sys_mmap2
#define compat_sys_truncate64_wrapper		compat_sys_truncate64
#define compat_sys_ftruncate64_wrapper		compat_sys_ftruncate64
#define compat_sys_readahead_wrapper		compat_sys_readahead
#define compat_sys_statfs64_wrapper		compat_sys_statfs64
#define compat_sys_fstatfs64_wrapper		compat_sys_fstatfs64
#define compat_sys_fadvise64_64_wrapper		compat_sys_fadvise64_64
#define compat_sys_sync_file_range2_wrapper	compat_sys_sync_file_range2
#define compat_sys_fallocate_wrapper		compat_sys_fallocate


#include "syscall_name.c"
