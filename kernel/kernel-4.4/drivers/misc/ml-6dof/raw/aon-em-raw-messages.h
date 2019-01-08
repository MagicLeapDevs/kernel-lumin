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
 * This file defines interface to communicate with EM Tracking
 * IVC task running on AON side.
 */

#ifndef _EMT_RAW_AON_MESSAGES_H_
#define _EMT_RAW_AON_MESSAGES_H_

#include <linux/types.h>

#define TEGRA_IVC_ALIGN 64

/* All the enums and the fields inside the structs described in this header
 * file supports only uX type, where X can be 8,16,32.
 */

#define EMT_RAW_AON_MAX_DATA_SIZE 8192
#define EMT_RAW_AON_MAX_MSG_SIZE  (EMT_RAW_AON_MAX_DATA_SIZE + TEGRA_IVC_ALIGN)

/* This indicates the status of the IVC request sent from
 * CCPLEX was successful or not.
 */
enum emt_raw_aon_status {
	EMT_RAW_AON_STATUS_OK  = 0,
	EMT_RAW_AON_STATUS_ERR = 1,
};

/* This struct is used to send EMT_RAW data from CCPLEX to SPE.
 *
 * Fields:
 * length:  Current transfer length.
 * data:    Buffer that holds the data for the current transaction.
 */
struct emt_raw_aon_req {
	u32 length;
	u8 padding[TEGRA_IVC_ALIGN - sizeof(u32)];
	u8 data[EMT_RAW_AON_MAX_DATA_SIZE];
};

/* This struct is used to send EMT_RAW data from SPE to CCPLEX.
 *
 * Fields:
 * status:  Response in regard to the request (success/failure).
 * length:  Current transfer length.
 * data:    Buffer that holds the data for the current transaction.
 */
struct emt_raw_aon_resp {
	u32 status;
	u32 length;
	u8 padding[TEGRA_IVC_ALIGN - (sizeof(u32) * 2)];
	u8 data[EMT_RAW_AON_MAX_DATA_SIZE];
};

#endif /* _EMT_RAW_AON_MESSAGES_H_ */
