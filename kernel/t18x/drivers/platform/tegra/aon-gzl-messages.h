/*
 * aon-gzl-messages.h
 *
 * AON Magic Leap Gazelle message definition
 *
 * Copyright (c) 2016, Magic Leap, Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _AON_GZL_MESSAGES_H_
#define _AON_GZL_MESSAGES_H_

#include <linux/types.h>

#define TEGRA_IVC_ALIGN 64

/* All the enums and the fields inside the structs described in this header
 * file supports only uX type, where X can be 8,16,32.
 */

//#define AON_GZL_MAX_DATA_SIZE 1024
#define AON_GZL_MAX_DATA_SIZE 64
#define AON_GZL_MAX_MSG_SIZE  (AON_GZL_MAX_DATA_SIZE + TEGRA_IVC_ALIGN)

/* This indicates the status of the IVC request sent from
 * CCPLEX was successful or not.
 */
enum aon_gzl_status {
    AON_GZL_STATUS_OK = 0,
    AON_GZL_STATUS_ERROR = 1,
};

/* This enum represents the types of ML MUX requests
 * assocaited with AON.
 */
enum aon_gzl_req_type {
    AON_GZL_MSG_WRITE_REQ     = 0,
    AON_GZL_MSG_NOTIFY_REQ    = 1,
};

/* This struct is used to send ML MUX data from CCPLEX to SPE.
 *
 * Fields:
 * length:  Current transfer length.
 * data:  Buffer that holds the data for the current transaction.
 */
struct aon_gzl_write_req {
    u32 length;
    u8 data[AON_GZL_MAX_DATA_SIZE];
};

/* This struct is used to enable/disable notifications
 * from SPE to CCPLEX.
 *
 * Fields:
 * enable_notification: Notification status (0 - disable / 1 - enable).
 */
struct aon_gzl_notify_req {
    u8 enable_notification;
};

/* This struct encapsulates the type of request and respective
 * data associated with that request.
 * Fields:
 * type:  Indicates the type of the request (write/notification).
 * data:  Union of structs of all the request types.
 */
struct aon_gzl_req {
    u32 type;
    union {
        struct aon_gzl_write_req        write_req;
        struct aon_gzl_notify_req       notify_req;
    } data;
};

/* This enum represents the types of ML MUX responses
 * assocaited with AON
 */
enum aon_gzl_resp_type {
    AON_GZL_MSG_WRITE_ACK     = 0,
    AON_GZL_MSG_NOTIFICATION  = 1,
};

/* This struct is used to send "write ack" on "write" request.
 *
 * Fields:
 * status:  Response in regard to the request (success/failure).
 */
struct aon_gzl_write_resp {
    u32 status;
};

/* This struct is used to send ML MUX data from SPE to CCPLEX.
 *
 * Fields:
 * length:  Current transfer length.
 * data:  Buffer that holds the data for the current transaction.
 */
struct aon_gzl_notification {
    u32 length;
    u8 data[AON_GZL_MAX_DATA_SIZE];
};

/* This struct encapsulates the type of response and respective
 * data associated with that response.
 * Fields:
 * type:  Indicates the type of the response (write ack or notification ack).
 * data:  Union of structs of all the response types.
 */
struct aon_gzl_resp {
    u32 type;
    union {
        struct aon_gzl_write_resp        write_resp;
        struct aon_gzl_notification      notification;
    } data;
};

#endif /* _AON_GZL_MESSAGES_H_ */
