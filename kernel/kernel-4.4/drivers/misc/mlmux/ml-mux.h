/* Copyright (c) 2016-2017, Magic Leap, Inc. All rights reserved.
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

#ifndef _ML_MUX_H
#define _ML_MUX_H

#define ML_MUX_CH_NAME_LEN	(32)
#define CTRL_CHAN		(0)
#define MAX_CHAN		(254)
#define INVALID_CHAN		(255)

struct ml_mux_frame {
	uint16_t len;
	uint8_t seq_num;
	uint8_t channel;
	uint8_t buf[0];
} __packed;

struct ml_mux_chan {
	char name[ML_MUX_CH_NAME_LEN];
	struct ml_mux *mux;
	struct ml_mux_client *cl;
	uint8_t local_num;
	uint8_t remote_num;
};

struct ml_mux {
	/* queue */
	struct ml_mux_ctrl *ctrl;
	struct task_struct *process_queue;
	struct ml_mux_queue *tx_queue;
	wait_queue_head_t q_wait;
	spinlock_t q_lock;

	/* channels */
	struct ml_mux_client client;
	struct idr chan_idr;
	struct mutex  chans_lock;

	/* tracking */
	uint8_t expect_seqn;
	bool remote_is_up;
	struct list_head list_entry;
};

struct ml_mux_chan *ml_mux_find_chan_byname(struct ml_mux *mux, char *name);
void ml_mux_reset_channels(struct ml_mux *mux);
void ml_mux_reset_mux(struct ml_mux *mux);

#endif /* _ML_MUX_H */

