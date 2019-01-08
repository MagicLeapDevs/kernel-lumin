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

#define __X_ARM_SYSCALL_NR_TO_SERIAL(nr) ((nr) - __ARM_NR_BASE + __NR_syscalls)
#define __X_SYSCALL_NR_TO_SERIAL(nr)	 (((unsigned)(nr) < __ARM_NR_BASE) ? (nr) : __X_ARM_SYSCALL_NR_TO_SERIAL(nr))

#define __NR_breakpoint	__X_ARM_SYSCALL_NR_TO_SERIAL(__ARM_NR_breakpoint)
__SYSCALL(__NR_breakpoint, sys_breakpoint)
#define __NR_cacheflush	__X_ARM_SYSCALL_NR_TO_SERIAL(__ARM_NR_cacheflush)
__SYSCALL(__NR_cacheflush, sys_cacheflush)
#define __NR_usr26	__X_ARM_SYSCALL_NR_TO_SERIAL(__ARM_NR_usr26)
__SYSCALL(__NR_usr26, sys_usr26)
#define __NR_usr32	__X_ARM_SYSCALL_NR_TO_SERIAL(__ARM_NR_usr32)
__SYSCALL(__NR_usr32, sys_usr32)
#define __NR_set_tls	__X_ARM_SYSCALL_NR_TO_SERIAL(__ARM_NR_set_tls)
__SYSCALL(__NR_set_tls, sys_set_tls)

#define __X_NR_syscalls	(__NR_set_tls + 1)
