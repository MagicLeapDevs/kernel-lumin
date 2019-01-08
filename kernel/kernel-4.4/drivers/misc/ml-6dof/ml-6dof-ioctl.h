/* Copyright (c) 2017, Magic Leap, Inc. All rights reserved.
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
 * This file defines interface to communicate with EM
 * tracking kernel driver from user space components.
 */

#ifndef __ML_6DOF_IOCTL__
#define __ML_6DOF_IOCTL__

#include <linux/ioctl.h>
#include <linux/types.h>

#define EMT_IQ_SZ           9
#define EMT_MAX_TOTEMS      2

#define EMT_PLL_LOCK_FLAG(totem, channel) (1 << ((4 - (channel)) * (totem) - 1))

struct emt_data_ioctl_msg {
	uint32_t  i[EMT_MAX_TOTEMS][EMT_IQ_SZ];
	uint32_t  q[EMT_MAX_TOTEMS][EMT_IQ_SZ];
	uint64_t  timestamp;
	uint32_t  flags[EMT_MAX_TOTEMS];
} __attribute__((__packed__));

#define EMT_IOCTL_GET_EM_DATA  _IOR('E', 0, struct emt_data_ioctl_msg)

#endif /* __ML_6DOF_IOCTL__ */
