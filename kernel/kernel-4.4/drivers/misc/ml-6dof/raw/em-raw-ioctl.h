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

#ifndef __TEGRA_EMT__
#define __TEGRA_EMT__

#include <linux/ioctl.h>
#include <linux/types.h>

#define EMT_MAX_REQ_SZ      PAGE_SIZE
#define EMT_MAX_RESP_SZ     (PAGE_SIZE * 2)
#define EMT_MSG_TOTAL_PAGES 4

struct emt_ioctl_msg_t {
	uint16_t req_len;
	uint16_t resp_len;
	uint8_t  pad[PAGE_SIZE - (sizeof(uint16_t) * 2)];
	uint8_t  req_data[EMT_MAX_REQ_SZ];
	uint8_t  resp_data[EMT_MAX_RESP_SZ];
} __attribute__((__packed__));

#define EMT_IOCTL_MAGIC 'E'

#define EMT_IOCTL_MMAP_MSG  _IOW(EMT_IOCTL_MAGIC, 0, uint8_t)

#endif /* __TEGRA_EMT__ */
