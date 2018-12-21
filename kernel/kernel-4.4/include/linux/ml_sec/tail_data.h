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

#ifndef __MLSEC_TAIL_DATA_H
#define __MLSEC_TAIL_DATA_H

#include <linux/fs.h>
#include <uapi/linux/ml_sec/tail_data.h>

#ifdef __LITTLE_ENDIAN
#define TAG32(c4) \
	((((uint32_t)(c4) & 0xFF) << 24) | \
	 (((uint32_t)(c4) & 0xFF00) << 8) | \
	 (((uint32_t)(c4) & 0xFF0000) >> 8) | \
	 (((uint32_t)(c4) & 0xFF000000) >> 24))
#else
#define TAG32(c4) \
	((uint32_t)(c4))
#endif

#ifdef CONFIG_MLSEC_COMMON
/*
 *  is_debuggable()
 *
 *  Returns true if the file is marked as "debuggable" (in the tail data)
 */
bool is_debuggable(struct file *file);

/*
 *  read_ml_tail_data()
 *
 *  Reads the ML tail data for a specified file into tail_data
 *  Returns 0 on success
 */
int read_ml_tail_data(struct file *file, struct ml_tail_data *data);

#else
static inline bool is_debuggable(struct file *file) { return true; }
static inline int read_ml_tail_data(struct file *file, struct ml_tail_data *data) { return -1; }
#endif


#endif	/* __MLSEC_TAIL_DATA_H */
