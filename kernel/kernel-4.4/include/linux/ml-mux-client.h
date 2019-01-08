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

#ifndef _ML_MUX_CHAN_H
#define _ML_MUX_CHAN_H

#include <linux/device.h>

#define ML_MUX_CH_REQ_SUCCESS	(0)
#define ML_MUX_CH_REQ_OPENED	(1)
struct ml_mux_chan;

/**
 * struct ml_mux_client: structure representing the client of mux
 * @dev: pointer to the device of the client.
 * @ch: pointer to channel client is connected to.
 * @receive_cb: this is a synchronous callback from inside mux controller
 *		context. This receives a pointer to client, length of msg,
 *		and pointer to the msg. This function must be complete in
 *		minimum amount of time and must not sleep. The implementation,
 *		in general, should copy the data locally(possibly into queue)
 *		schedule a work and return.
 * @notify_open: this functions is called inside mux controller context.
 *		It receives a pointer to the client and a boolean signifying
 *		a change in associated channel status. If true the channel
 *		has been opened in the remote and data can now be sent over
 *		this channel. If false the channel is closing remotely or being
 *		freed by the controller. Upon closing of the channel any
 *		tx work by the client must be aborted and channel must no longer
 *		be used.
 *
 * Since the callbacks are all executed inside mux controller context they must
 * not make calls into the exposed API to avoid deadlocking.
 */
struct ml_mux_client {
	struct device *dev;
	struct ml_mux_chan *ch;

	void (*receive_cb)(struct ml_mux_client *cli, uint32_t len, void *msg);
	void (*notify_open)(struct ml_mux_client *cli, bool is_open);
};

int ml_mux_send_msg(struct ml_mux_chan *chan, uint32_t len, void *msg);
void ml_mux_release_channel(struct ml_mux_chan *chan);
int ml_mux_request_channel(struct ml_mux_client *client, char *name);

#endif /* _ML_MUX_CHAN_H */

