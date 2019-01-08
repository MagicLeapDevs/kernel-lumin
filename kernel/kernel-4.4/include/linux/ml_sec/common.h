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

#ifndef __COMMON_H__
#define __COMMON_H__

/*
 *   Magic Leap Security Infrastructure
 *
 *   Common header for frequently used functions / utils etc.
 */

#include <linux/fs.h>

#ifdef CONFIG_MLSEC_COMMON

/*
 *  is_file_on_system_partition()
 *
 *  Returns true if the given file is on a verified partition.
 *  Currently the only verified partitions are: "system" and "vendor".
 *  Otherwise, returns false
*/
bool is_file_on_verified_partition(const struct file *file);

#else
static inline bool is_file_on_verified_partition(const struct file *file) { return true; };
#endif

/*
 *  Wrappers for printk, so we can control our printing code separately from
 *  the usual kernel print foundation
*/
#define ML_DBG_PRINT(fmt, ...)    printk(KERN_DEBUG "%s: " fmt, __func__, __VA_ARGS__)

#endif /* __COMMON_H__ */