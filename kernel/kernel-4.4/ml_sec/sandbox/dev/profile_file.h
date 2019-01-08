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

#ifndef __MLSEC_SBOX_DEV_PROFILE_FILE_H
#define __MLSEC_SBOX_DEV_PROFILE_FILE_H

#include <linux/fs.h>
#include <linux/path.h>

/*
 * Returns the number of entries successfully parsed, or error code.
 */
int sbox_parse_profile_file(struct path *exec_path, const char *ext,
			    bool (*parse_entry)(const char *str, void *user_data), void *user_data);

#endif	/* __MLSEC_SBOX_DEV_PROFILE_FILE_H */
