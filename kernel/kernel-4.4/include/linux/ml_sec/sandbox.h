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

#ifndef __MLSEC_SANDBOX_H
#define __MLSEC_SANDBOX_H

#include <linux/binfmts.h>

struct path;

#ifdef CONFIG_MLSEC_SANDBOX
#include <linux/ml_sec/sandbox_profile.h>

/* Deny the syscall without further inspection. */
#define SANDBOX_ACTION_DENY		0
/* Allow the syscall without further inspection. */
#define SANDBOX_ACTION_ALLOW		1
/* Inspect the syscall (usually its arguments) using a dedicated function. */
#define SANDBOX_ACTION_INSPECT		2

/*
 * The syscall filter is defined as an array of a 2-bits data type.
 * Therefore each byte currently contains 4 (= 8 / SANDBOX_ACTIONS_MASK_BITS),
 * elements.
 *
 * To access the n' element, we need to access the (n / 4) byte. But this byte contains 4 elements.
 * To access the exact element in this byte we need to access element number (n % 4).
 * For n % 4 =
 *   0: the mask for the element is 0x03 (= 3 << 0)
 *   1: the mask for the element is 0x0C (= 3 << 2)
 *   2: the mask for the element is 0x30 (= 3 << 4)
 *   3: the mask for the element is 0xC0 (= 3 << 6)
 */
#define SANDBOX_ACTIONS_MASK		3
#define SANDBOX_ACTIONS_MASK_BITS	2
#define SANDBOX_ACTIONS_PER_BYTE	(8 / SANDBOX_ACTIONS_MASK_BITS)

/* The size in bytes of a syscall filter of n elements. */
#define SBOX_SYS_FILTER_SIZE(n)		(((n) + (SANDBOX_ACTIONS_PER_BYTE - 1)) / SANDBOX_ACTIONS_PER_BYTE)
/* The index of the n element into a syscall filter. */
#define SBOX_SYS_FILTER_ELEM_INDEX(n)	((n) / SANDBOX_ACTIONS_PER_BYTE)
/* The offset of the n element into a syscall filter data byte. */
#define SBOX_SYS_FILTER_ELEM_OFFSET(n)	(((n) % SANDBOX_ACTIONS_PER_BYTE) * SANDBOX_ACTIONS_MASK_BITS)

int sbox_apply(struct linux_binprm *bprm);
int sbox_filter_syscall(int sys_nr, unsigned int action);
bool sbox_is_permissive(void);
#else
struct sandbox_profile;
static inline int sbox_apply(struct linux_binprm *bprm) { return 0; }
static inline int sbox_filter_syscall(int sys_nr, unsigned int action) { return 0; }
#endif

/*
 * In develop mode we use a ref-count for managing the sandbox profiles.
 * Specifically needed to free dynamically allocated profiles (from files).
 */
#ifdef CONFIG_MLSEC_SANDBOX_DEVELOP
void sbox_get_profile(struct task_struct *tsk);
void sbox_put_profile(struct task_struct *tsk);
#else
static inline void sbox_get_profile(struct task_struct *tsk) {}
static inline void sbox_put_profile(struct task_struct *tsk) {}
#endif

#ifdef CONFIG_MLSEC_SANDBOX_AUDIT
void sbox_audit_jail(const struct path *path, bool permissive);
void sbox_audit_jail_localhost(const char *op_name, unsigned short port);
void sbox_audit_syscall(int sys_nr, int error);
void sbox_free_audit_context(struct task_struct *tsk);
#else
static inline void sbox_audit_jail(const struct path *path, bool permissive) {}
static inline void sbox_audit_jail_localhost(const char *op_name, unsigned short port, bool permissive) {}
static inline void sbox_audit_syscall(int sys_nr, int error) {}
static inline void sbox_free_audit_context(struct task_struct *tsk) {}
#endif

#endif	/* __MLSEC_SANDBOX_H */
