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

#ifndef _ML_MUX_THERM_H
#define _ML_MUX_THERM_H

#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/thermal.h>
#include <linux/mutex.h>
#include <linux/ml-mux-client.h>

#define MLMUX_TZ_NAME_SIZE	(5)
#define MLMUX_MAX_NUM_TZ	(10)

struct mlmux_therm_zone {
	char name[MLMUX_TZ_NAME_SIZE];
	int16_t id;
	int16_t temp;
	uint32_t critical_temp;
	struct thermal_zone_device *tz;
};

struct mlmux_therm_dev_data {
	struct device *dev;
	struct ml_mux_client client;
	const char *chan_name;
	struct workqueue_struct *work_q;
	struct completion suspend;
	struct completion resume;
	struct mutex lock;
	struct mutex reg_lock;

	struct idr zones;
	bool chan_up;
};

struct mlmux_therm_work {
	struct work_struct work;
	struct mlmux_therm_dev_data *tdev;
	void *data;
};

/* mlmux messages */
enum mlmux_therm_rx_type {
	MLMUX_THERM_RX_REG,
	MLMUX_THERM_RX_UPDATE,
	MLMUX_THERM_RX_SUSPEND_ACK,
	MLMUX_THERM_RX_RESUME_ACK,
};

struct mlmux_therm_rx_reg {
	const char name[MLMUX_TZ_NAME_SIZE];
} __packed;

struct mlmux_therm_rx_update {
	int16_t id;
	int16_t temp;
} __packed;

struct mlmux_therm_rx_msg {
	uint8_t type;
	union {
		int data;
		struct mlmux_therm_rx_reg reg;
		struct mlmux_therm_rx_update update;
	} __packed u;
} __packed;
#define MLMUX_THERM_RX_SIZE(msg) \
	(sizeof(struct mlmux_therm_rx_msg) - \
	sizeof(((struct mlmux_therm_rx_msg *)0)->u) + \
	sizeof(((struct mlmux_therm_rx_msg *)0)->u.msg))

enum mlmux_therm_tx_type {
	MLMUX_THERM_TX_REG_ACK,
	MLMUX_THERM_TX_CHANGE_RATE,
	MLMUX_THERM_TX_UPDATE_NOW,
	MLMUX_THERM_TX_SUSPEND,
	MLMUX_THERM_TX_RESUME,
};

struct mlmux_therm_tx_ack {
	char name[MLMUX_TZ_NAME_SIZE];
	int16_t id;
} __packed;

struct mlmux_therm_tx_msg {
	uint8_t type;
	union {
		int32_t data;
		struct mlmux_therm_tx_ack ack;
	} __packed u;
} __packed;
#define MLMUX_THERM_TX_MSG_SIZE(msg) \
	(sizeof(struct mlmux_therm_tx_msg) - \
	sizeof(((struct mlmux_therm_tx_msg *)0)->u) + \
	sizeof(((struct mlmux_therm_tx_msg *)0)->u.msg))

#endif /* _ML_MUX_THERM_H */

