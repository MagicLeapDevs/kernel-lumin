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

#ifndef _AON_6DOF_AON_MESSAGES_H_
#define _AON_6DOF_AON_MESSAGES_H_

#include <linux/types.h>

#define TEGRA_IVC_ALIGN 64

/* All the enums and the fields inside the structs described in this header
 * file supports only uX type, where X can be 8,16,32.
 */
#define AON_6DOF_DATA_SZ    150

#define AON_6DOF_MAX_MSG_SIZE  (AON_6DOF_DATA_SZ + TEGRA_IVC_ALIGN)
#define AON_6DOF_MAX_NUM_TOTEMS 2
#define AON_6DOF_IQ_SZ 9

/* This enum represents the types of async requests assocaited
 * with AON ML 6DOF
 */
enum aon_6dof_req_type {
	AON_6DOF_REQ_EM_DATA = 0,
	AON_6DOF_REQ_MODE,
	AON_6DOF_REQ_NUM,      /**< should always be the last one */
};

/* This enum represents the mode of operation.
 */
enum aon_6dof_mode {
	AON_6DOF_MODE_DISABLED,
	AON_6DOF_MODE_TDSP,
	AON_6DOF_MODE_PASS_THRU,
};

/* This struct is used to setup mode of operation for
 * FPGA AON driver
 * Fields:
 * mode:  One of aon_6dof_mode
 */
struct aon_6dof_req_mode {
	u8 mode;
};

/* This structure defines request from CCPLEX to SPE for the AON
 * 6DOF sub-system.
 *
 * Fields:
 * req_type:	 Indicates the type of request. Supports init, setup and xfer.
 * data:	 Union of structs of all the request types.
 */
struct aon_6dof_request {
	u8 req_type;
	union {
		struct aon_6dof_req_mode mode;
	} data;
};

/* This struct represents 6 DOF EM data sent from SPE to CCPLEX.
 *
 * Fields:
 * i:            3x3 matrix of I data for every totem.
 * q:            3x3 matrix of Q data for every totem.
 * timestamp:    Timestamp of the FPGA.
 * status:       Flags associated with a sample of data for every totem.
 */
struct aon_6dof_resp_em_data {
	u32  i[AON_6DOF_MAX_NUM_TOTEMS][AON_6DOF_IQ_SZ];
	u32  q[AON_6DOF_MAX_NUM_TOTEMS][AON_6DOF_IQ_SZ];
	u64  timestamp;
	u32  flags[AON_6DOF_MAX_NUM_TOTEMS];
};

/* This indicates the status of the IVC request sent from
 * CCPLEX was successful or not.
 */
enum aon_6dof_status {
	AON_6DOF_STATUS_OK = 0,
	AON_6DOF_STATUS_ERROR = 1,
};

/* This struct is used to send 6 DOF messages from SPE to CCPLEX.
 *
 * Fields:
 * type:    One of aon_6dof_resp_type.
 * status:  Response in regard to the request i.e success/failure.
 * data:    Union of structs of response types.
 */
struct aon_6dof_response {
	u8 resp_type;
	u8 status;
	u8 padding[2];
	union {
		struct aon_6dof_resp_em_data em;
	} data;
};

#endif /* _AON_6DOF_AON_MESSAGES_H_ */
