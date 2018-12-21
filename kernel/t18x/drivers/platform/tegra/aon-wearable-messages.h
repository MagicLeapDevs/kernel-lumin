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

#ifndef _AON_IVC_DATA_MESSAGES_H_
#define _AON_IVC_DATA_MESSAGES_H_

#include <linux/types.h>

#define TEGRA_IVC_DATA_ALIGN_SIZE 64
#define TEGRA_IVC_MAX_DATA_SIZE 1024
#define TEGRA_IVC_MAX_MSG_SIZE (TEGRA_IVC_MAX_DATA_SIZE + \
				TEGRA_IVC_DATA_ALIGN_SIZE)

/* This indicates the status of the IVC request sent from
 * CCPLEX was successful or not.
 */
enum aon_wearable_status {
	AON_WEARABLE_STATUS_OK = 0,
	AON_WEARABLE_STATUS_ERROR = 1,
};

/* This struct encapsulates the length and the respective data
 * associated with that request.
 * Fields:
 * length:	indicates the length of the data
 * data:	transferring payload.
 */
struct aon_wearable_request {
	u32 length;
	u8 padding[TEGRA_IVC_DATA_ALIGN_SIZE - sizeof(u32)];
	u8 data[TEGRA_IVC_MAX_DATA_SIZE];
};

/* This struct encapsulates the status of the response and the respective
 * data associated with that response.
 * Fields:
 * status:	response in regard to the request i.e success/failure.
 * resp_length:	indicates the length of the responsed.
 * data:	responsed data.
 */
struct aon_wearable_response {
	u32 status;
	u32 length;
	u8 padding[TEGRA_IVC_DATA_ALIGN_SIZE - sizeof(u32)*2];
	u8 data[TEGRA_IVC_MAX_DATA_SIZE];
};

#endif
