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

#ifndef __MLSEC_SIGNING_DEV_H
#define __MLSEC_SIGNING_DEV_H

#include <stdbool.h>

#ifdef CONFIG_MLSEC_ELF_SIGNING_DEVELOP
bool sig_dev_is_permissive(void);
extern bool g_signing_dev_is_permissive;
#else
static inline bool sig_dev_is_permissive(void) { return false; }
#endif

#endif	/* __MLSEC_SIGNING_DEV_H */
