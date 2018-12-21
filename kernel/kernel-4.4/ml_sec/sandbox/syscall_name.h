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

#ifndef __MLSEC_SBOX_SYSCALL_NAME_H
#define __MLSEC_SBOX_SYSCALL_NAME_H

#if defined(CONFIG_MLSEC_SANDBOX_AUDIT) || defined(CONFIG_MLSEC_SANDBOX_DEVELOP)

const char *get_syscall_name(int x_nr);
int lookup_syscall_nr(const char *name);

#ifdef CONFIG_COMPAT
const char *get_syscall_name_compat(int x_nr);
int lookup_syscall_nr_compat(const char *name);
#endif

#else /* !(CONFIG_MLSEC_SANDBOX_AUDIT || CONFIG_MLSEC_SANDBOX_DEVELOP) */

static inline const char *get_syscall_name(int x_nr) { return NULL; }
static inline int lookup_syscall_nr(const char *name) { return -1; }

#ifdef CONFIG_COMPAT
static inline const char *get_syscall_name_compat(int x_nr) { return NULL; }
static inline int lookup_syscall_nr_compat(const char *name) { return -1; }
#endif

#endif

#endif	/* __MLSEC_SBOX_SYSCALL_NAME_H */
