/* Copyright (c) 2016, Magic Leap, Inc. All rights reserved.
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

#ifndef _ML_MUX_CTRL_H
#define _ML_MUX_CTRL_H

#define ML_MUX_CTRL_NAME_LEN	(32)
#define ML_MAX_QUEUE_LEN	(30)
#define ML_MAX_WR_LEN		(PAGE_SIZE)
#define ML_MAX_RD_LEN		(PAGE_SIZE)

/**
 * struct ml_mux_ctrl: structure representing the mux controller
 * @dev: pointer to the device of the controller.
 * @name: name of the controller. For debugging and tracking.
 * @tx_data: this is a synchronous callback from inside internal queue.
 *		Upon completion of this command the frame slot will be
 *		made available for reuse.
 * @queue_len: length of the queue controller wants allocated for tx.
 * @max_tx_len: maximum length in bytes controller can send at once.
 * @max_rx_len: maximum length in bytes controller can receive at once.
 * @is_highpri: true indicates high(rt) priority of tx thread.
 */
struct ml_mux_ctrl {
	struct device *dev;
	char name[ML_MUX_CTRL_NAME_LEN];
	void (*tx_data)(struct ml_mux_ctrl *ctrl, uint32_t len, void *data);

	uint32_t queue_len;
	uint32_t max_tx_len;
	uint32_t max_rx_len;
	bool is_highpri;
};

void ml_mux_msg_receive(struct ml_mux_ctrl *ctrl, uint32_t len, void *msg);
int ml_mux_ctrl_register(struct ml_mux_ctrl *ctrl);
void ml_mux_ctrl_unregister(struct ml_mux_ctrl *ctrl);

#endif /* _ML_MUX_CTRL_H */

