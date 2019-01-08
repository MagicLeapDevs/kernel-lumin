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

/*
 * This file contains the implementations of the syscall inspection
 * functions, as well as their function table.
 */

#include <linux/ml_sec/sandbox.h>
#include <linux/personality.h>
#include <linux/mman.h>
#include <linux/prctl.h>
#include <asm/syscall.h>
#include <asm/unistd_ext.h>
#include "sandbox_profile_tags.h"

typedef int (*inspect_syscall_func)(unsigned long args[]);

static inline bool is_USER_profile(void)
{
	return current->sandbox_profile->profile_tag == SBOX_PROFILE_TAG_USER;
}

static int deny_syscall(unsigned long args[])
{
	return -EPERM;
}

/*
 * 1 fd version:
 * Only allow '*at' syscalls to act as their non '*at' versions.
 * Meaning that the supplied string is relative to the current directory.
 */
static int inspect_sys_at(unsigned long args[])
{
	int dfd = (int)args[0];

	return (dfd == AT_FDCWD) ? 0 : -EBADF;
}

/*
 * 2 fds version:
 * Only allow '*at' syscalls to act as their non '*at' versions.
 * Meaning that the supplied string is relative to the current directory.
 */
static int inspect_sys_at2(unsigned long args[])
{
	int olddfd = (int)args[0];
	int newdfd = (int)args[2];

	return (olddfd == AT_FDCWD && newdfd == AT_FDCWD) ? 0 : -EBADF;
}

/*
 * Only allow the current pid (as first argument).
 */
static int inspect_pid(unsigned long args[])
{
	pid_t pid = (pid_t)args[0];
	pid_t current_pid;
	struct task_struct *task;

	if (!pid)
		return 0;

	current_pid = task_tgid_vnr(current);
	if (pid == current_pid)
		return 0;

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	rcu_read_unlock();
	if (task && task_tgid_vnr(task) == current_pid)
		return 0;

	return -EPERM;
}

/*
 * Only allow the current process group ID (as first argument).
 */
static int inspect_pgid(unsigned long args[])
{
	pid_t pgid = (pid_t)args[0];
	struct pid *pgrp;

	if (!pgid)
		return 0;

	rcu_read_lock();
	pgrp = find_vpid(pgid);
	rcu_read_unlock();
	if (pgrp == task_pgrp(current))
		return 0;

	return -EPERM;
}

/*
 * Only allow the current uid (as first argument).
 */
static int inspect_uid(unsigned long args[])
{
	uid_t uid = (uid_t)args[0];
	const struct cred *cred;
	kuid_t kuid;

	if (!uid)
		return 0;

	cred = current_cred();
	kuid = make_kuid(cred->user_ns, uid);
	return uid_eq(kuid, cred->uid) ? 0 : -EPERM;
}

/*
 * Only allow to set the priority of the process group of the calling process.
 */
static int inspect_setpriority(unsigned long args[])
{
	int which = (int)args[0];
	int who = (int)args[1];

	if (!who)
		return 0;

	switch (which) {
	case PRIO_PROCESS:
		return inspect_pid(&args[1]);

	case PRIO_PGRP:
		return inspect_pgid(&args[1]);

	case PRIO_USER:
		return inspect_uid(&args[1]);

	default:
		return -EINVAL;
	}
}

/*
 * Only allow the current pid (as first argument) - tgkill specific.
 */
static int inspect_tgkill(unsigned long args[])
{
	pid_t tgid = (pid_t)args[0];

	/* If tgid is specified as -1, tgkill() is equivalent to tkill(). */
	if (tgid == -1)
		return 0;

	return inspect_pid(args);
}

/*
 * Only allow specific behaviors, commonly used, and are not problematic.
 */
static int inspect_madvise(unsigned long args[])
{
	int behavior = (int)args[2];

	switch (behavior) {
	case MADV_NORMAL:
	case MADV_RANDOM:
	case MADV_SEQUENTIAL:
	case MADV_WILLNEED:
	case MADV_DONTNEED:
	case MADV_REMOVE:
	case MADV_MERGEABLE:
	case MADV_UNMERGEABLE:
	case MADV_DONTFORK:
		return 0;
	}
	return -EPERM;
}

/*
 * Only allow specific commands, commonly used, and are not problematic.
 */
static int inspect_fcntl(unsigned long args[])
{
	unsigned int cmd = (unsigned int)args[1];

	switch (cmd) {
	case F_DUPFD:
	case F_SETSIG:
	case F_GETSIG:
		/* Do not allow the above commands to USER. */
		if (is_USER_profile())
			break;

	case F_GETFD:
	case F_SETFD:
	case F_GETFL:
	case F_SETFL:
	case F_GETLK:
	case F_SETLK:
	case F_SETLKW:
	case F_GETLK64:
	case F_SETLK64:
	case F_SETLKW64:
		return 0;
	}
	return -EPERM;
}

/*
 * Only allow specific options, commonly used, and are not problematic.
 */
static int inspect_prctl(unsigned long args[])
{
	int option = (int)args[0];
	switch (option) {
	case PR_SET_SECCOMP:
	case PR_GET_SECCOMP:
	case PR_SET_NO_NEW_PRIVS:
	case PR_GET_NO_NEW_PRIVS:
	case PR_SET_KEEPCAPS:
	case PR_GET_KEEPCAPS:
	case PR_GET_TIMING:
	case PR_SET_DUMPABLE:
	case PR_SET_FPEXC:
	case PR_GET_FPEXC:
	case PR_CAPBSET_READ:
	case PR_CAPBSET_DROP:
	case PR_GET_SECUREBITS:
		/* Do not allow the above options to USER. */
		if (is_USER_profile())
			break;

	case PR_GET_DUMPABLE:
	case PR_SET_PDEATHSIG:
	case PR_GET_PDEATHSIG:
	case PR_SET_NAME:
	case PR_GET_NAME:
	case PR_SET_VMA:
	case PR_SET_THP_DISABLE:
	case PR_GET_THP_DISABLE:
		return 0;
	}
	return -EPERM;
}

/*
 * Only allow to 'get' the personality,
 * or 'set' with just the address limit flags.
 */
static int inspect_personality(unsigned long args[])
{
	unsigned int personality = (unsigned int)args[0];

	if (personality == 0xffffffff)
		return 0;

	if ((personality & ~ADDR_LIMIT_32BIT) == PER_LINUX)
		return 0;

	if (is_compat_task() && (personality & ~ADDR_LIMIT_3GB) == PER_LINUX32)
		return 0;

	return -EPERM;
}

/*
 * Allow requesting a core dump for your own process
 */
static int inspect_setrlimit(unsigned long args[])
{
	int resource = (unsigned int)args[0];

	if (resource == RLIMIT_CORE)
		return 0;

	return -EPERM;
}

/*
 * Only allow the current pid (as first argument).
 */
static int inspect_rt_tgsigqueueinfo(unsigned long args[])
{
	pid_t tgid = (pid_t)args[0];

	return (tgid == -1 || tgid == task_tgid_vnr(current)) ? 0 : -EPERM;
}

/*
 * Deny creation of user namespaces, by not allowing the use of CLONE_NEWUSER.
 */
static int inspect_unshare(unsigned long args[])
{
	unsigned long unshare_flags = args[0];
	return ((unshare_flags & CLONE_NEWUSER) == 0) ? 0 : -EPERM;
}

/*
 * Only allow setpgid(0, 0).
 */
static int inspect_setpgid(unsigned long args[])
{
	pid_t pid = (pid_t)args[0];
	pid_t pgid = (pid_t)args[1];

	return (pid == 0 && pgid == 0) ? 0 : -EPERM;
}

static const inspect_syscall_func syscall_inspect_table[__X_NR_syscalls] = {
	[0 ... __X_NR_syscalls - 1] = deny_syscall,

	[__NR_personality]	 = inspect_personality,
	[__NR_tgkill]		 = inspect_tgkill,
	[__NR_madvise]		 = inspect_madvise,
	[__NR_prctl]		 = inspect_prctl,
	[__NR_fcntl]		 = inspect_fcntl,
#ifdef __NR_fcntl64
	[__NR_fcntl64]		 = inspect_fcntl,
#endif

	[__NR_setpriority]	 = inspect_setpriority,

	[__NR_sched_getaffinity] = inspect_pid,
	[__NR_sched_getattr]	 = inspect_pid,
	[__NR_sched_getparam]	 = inspect_pid,
	[__NR_sched_setaffinity] = inspect_pid,
	[__NR_sched_setattr]	 = inspect_pid,
	[__NR_sched_setparam]	 = inspect_pid,
	[__NR_sched_setscheduler] = inspect_pid,
	[__NR_sched_getscheduler] = inspect_pid,
	[__NR_sched_rr_get_interval] = inspect_pid,

	[__NR_execveat]		 = inspect_sys_at,
	[__NR_faccessat]	 = inspect_sys_at,
	[__NR_fchmodat]		 = inspect_sys_at,
	[__NR_fchownat]		 = inspect_sys_at,
	[__NR_mkdirat]		 = inspect_sys_at,
	[__NR_mknodat]		 = inspect_sys_at,
	[__NR_name_to_handle_at] = inspect_sys_at,
	[__NR_openat]		 = inspect_sys_at,
	[__NR_readlinkat]	 = inspect_sys_at,
	[__NR_symlinkat]	 = inspect_sys_at,
	[__NR_unlinkat]		 = inspect_sys_at,
	[__NR_utimensat]	 = inspect_sys_at,
	[__NR_linkat]		 = inspect_sys_at2,
	[__NR_renameat]		 = inspect_sys_at2,

	[__NR_rt_tgsigqueueinfo] = inspect_rt_tgsigqueueinfo,

	[__NR_setrlimit]	 = inspect_setrlimit,

#ifdef __NR_newfstatat
	[__NR_newfstatat]	 = inspect_sys_at,
#endif
#ifdef __NR_fstatat64
	[__NR_fstatat64]	 = inspect_sys_at,
#endif
#ifdef __NR_futimesat
	[__NR_futimesat]	 = inspect_sys_at,
#endif

	[__NR_clone]		 = inspect_unshare,
	[__NR_unshare]		 = inspect_unshare,

	[__NR_setpgid]		 = inspect_setpgid,
};

int sbox_inspect_syscall_compat(int sys_nr, unsigned long args[]);

#ifndef __COMPAT_SYSCALL_NR
int sbox_inspect_syscall(int sys_nr)
{
	int x_nr;
	struct task_struct *task = current;
	struct pt_regs *regs = task_pt_regs(task);
	unsigned long args[6];

	syscall_get_arguments(task, regs, 0, 6, args);

#ifdef CONFIG_COMPAT
	if (is_compat_task())
		return sbox_inspect_syscall_compat(sys_nr, args);
#endif
	x_nr = __X_SYSCALL_NR_TO_SERIAL(sys_nr);
	return syscall_inspect_table[x_nr](args);
}
#endif /* !__COMPAT_SYSCALL_NR */
