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
 * This file is used only for preprocessing the syscall numbers,
 * then processed by the "gen_syscall_filters.sh" script.
 */

#define __SYSCALL(nr, sym) _SYS_(nr, sym)

#include <asm/unistd_ext.h>

_NR_SYS_(__X_NR_syscalls)
_NR_COMPAT_SYS_(__X_NR_compat_syscalls)
