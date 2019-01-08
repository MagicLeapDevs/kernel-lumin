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

#include <asm/unistd.h>

/*
 * The ARM specific syscalls are not following, numerically, the other syscalls.
 * Although they are serial and only a few, their numbers are at a different offset.
 * To gather all the syscall numbers into one continuous series, we need to adjust
 * the offsets of these special syscall numbers to follow the others.
 *
 * This is especially needed for creating one continuous array of all the syscalls
 * including the generic as well as the ARM specific ones.
 */

#undef __X_NR_syscalls
#define __X_NR_syscalls	__NR_syscalls

#ifdef CONFIG_COMPAT
#define __X_ARM_SYSCALL_NR_TO_SERIAL(nr) ((nr) - __ARM_NR_COMPAT_BASE + __NR_compat_syscalls)
#define __X_SYSCALL_NR_TO_SERIAL(nr)	 (((unsigned)(nr) < __ARM_NR_COMPAT_BASE) ? (nr) : __X_ARM_SYSCALL_NR_TO_SERIAL(nr))

#define __X_NR_compat_syscalls		 (__X_ARM_SYSCALL_NR_TO_SERIAL(__ARM_NR_compat_set_tls) + 1)

#ifdef __COMPAT_SYSCALL_NR
#include <asm/unistd32.h>

#define __NR_cacheflush	__X_ARM_SYSCALL_NR_TO_SERIAL(__ARM_NR_compat_cacheflush)
__SYSCALL(__NR_cacheflush, compat_sys_cacheflush)
#define __NR_set_tls	__X_ARM_SYSCALL_NR_TO_SERIAL(__ARM_NR_compat_set_tls)
__SYSCALL(__NR_set_tls, compat_sys_set_tls)

#undef __X_NR_syscalls
#define __X_NR_syscalls	__X_NR_compat_syscalls

#endif /* __COMPAT_SYSCALL_NR */
#else
#define __X_SYSCALL_NR_TO_SERIAL(nr)	(nr)
#endif /* CONFIG_COMPAT */
