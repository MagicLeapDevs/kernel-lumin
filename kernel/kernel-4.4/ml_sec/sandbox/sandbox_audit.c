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

#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/ml_sec/sandbox.h>
#include <asm/unistd_ext.h>
#include <asm/syscall.h>
#include "syscall_name.h"

typedef unsigned long	bmask_t;
#define BMASK_SZ	(sizeof(bmask_t)*8)

#define FILTER_SIZE(n)	(((n) + (BMASK_SZ - 1)) / BMASK_SZ)

static void *get_sandbox_audit_context(struct task_struct *tsk)
{
	if (tsk->sandbox_audit_context)
		return tsk->sandbox_audit_context;

	task_lock(tsk);
	if (!tsk->sandbox_audit_context) {
		size_t size;
#ifdef CONFIG_COMPAT
		if (is_compat_task())
			size = FILTER_SIZE(__X_NR_compat_syscalls);
		else
#endif
		size = FILTER_SIZE(__X_NR_syscalls);

		tsk->sandbox_audit_context = kzalloc(size, GFP_KERNEL);
	}
	task_unlock(tsk);
	return tsk->sandbox_audit_context;
}

void sbox_free_audit_context(struct task_struct *tsk)
{
	if (tsk->sandbox_audit_context)
		kfree(tsk->sandbox_audit_context);
}

/*
 * Return 0 if recorded the first time.
 * Return 1 if recorded already before.
 * Return error on failure.
 */
static int record_audit_syscall(struct task_struct *tsk, int x_nr)
{
	bmask_t *ctx, *bmask;

	ctx = get_sandbox_audit_context(tsk);
	if (!ctx)
		return -ENOMEM;

	bmask = &ctx[x_nr / BMASK_SZ];
	return test_and_set_bit(x_nr % BMASK_SZ, bmask);
}

static const char *get_task_exe(char **buf, struct task_struct *tsk)
{
	const char *exe_name = NULL;
	*buf = NULL;
	if (tsk->mm) {
		struct file *exe_file = get_mm_exe_file(tsk->mm);
		if (exe_file) {
			/* We will allow 11 spaces for ' (deleted)' to be appended */
			char *path_buf = kmalloc(PATH_MAX+11, GFP_KERNEL);
			if (path_buf) {
				char *p = d_path(&exe_file->f_path, path_buf, PATH_MAX+11);
				if (IS_ERR(p))
					kfree(path_buf);
				else {
					exe_name = p;
					*buf = path_buf;
				}
			}
			fput(exe_file);
		}
	}
	return exe_name;
}

void sbox_audit_syscall(int sys_nr, int error)
{
	struct task_struct *tsk = current;
	int x_nr = __X_SYSCALL_NR_TO_SERIAL(sys_nr);
	char err_buf[20];
	char comm[sizeof(tsk->comm)];
	const char *sys_name;
	char *path_buf = NULL;
	const char *err_str = "denied";

	if (record_audit_syscall(tsk, x_nr) > 0)
		return;

#ifdef CONFIG_COMPAT
	if (is_compat_task())
		sys_name = get_syscall_name_compat(x_nr);
	else
#endif
	sys_name = get_syscall_name(x_nr);

	if (error) {
		snprintf(err_buf, sizeof(err_buf), "error=%d", -error);
		err_str = err_buf;
	}

	pr_warn("sandbox: %s syscall=%d {%s} pid=%d comm=%s exe=%s arch=%x compat=%d ip=0x%lx permissive=%d\n",
		err_str,
		sys_nr,
		sys_name ? sys_name : "",
		task_pid_nr(tsk),
		get_task_comm(comm, tsk),
		get_task_exe(&path_buf, tsk),
		syscall_get_arch(),
		is_compat_task(),
		KSTK_EIP(tsk),
		sbox_is_permissive());

	kfree(path_buf);
}

void sbox_audit_jail(const struct path *path, bool permissive)
{
	const char *pathname = (const char *)path->dentry->d_name.name;
	char *pathbuf = __getname();
	if (pathbuf) {
		const char *absname = d_absolute_path(path, pathbuf, PATH_MAX);
		if (!IS_ERR(absname))
			pathname = absname;
	}

	pr_warn("sandbox: failed jail exe=%s permissive=%d\n", pathname, permissive);

	if (pathbuf)
		__putname(pathbuf);
}

void sbox_audit_jail_localhost(const char *op_name, unsigned short port)
{
	struct task_struct *tsk = current;
	char comm[sizeof(tsk->comm)];
	char *path_buf = NULL;

	pr_warn("sandbox: denied by jail {%s(localhost:%u)} pid=%d comm=%s exe=%s permissive=%d\n",
		op_name,
		port,
		task_pid_nr(tsk),
		get_task_comm(comm, tsk),
		get_task_exe(&path_buf, tsk),
		sbox_is_permissive());

	kfree(path_buf);
}
