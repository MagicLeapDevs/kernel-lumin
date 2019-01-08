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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/ml-mux-ctrl.h>
#include <linux/ml-mux-client.h>
#include "ml-mux-ctrl-chan.h"
#include "ml-mux.h"

#define MUX_VERSION		(0xa01)
#define CTRL_CH_NAME		"control"

/* ml_mux controller internal protocol definition*/
enum ml_mux_ctrl_msg_type {
	ML_MUX_CTRL_MSG_HELLO,
	ML_MUX_CTRL_MSG_HELLO_ACK,
	ML_MUX_CTRL_MSG_OPEN,
	ML_MUX_CTRL_MSG_CLOSE,
	ML_MUX_CTRL_MSG_RESET,
	ML_MUX_CTRL_MSG_MAX
};

struct ml_mux_hello_msg {
	uint16_t version;
	uint16_t max_rx_len;
} __attribute__((__packed__));

struct ml_mux_open_close_msg {
	uint8_t chan;
	char name[ML_MUX_CH_NAME_LEN];
} __attribute__((__packed__));

struct ml_mux_reset_msg {
	uint8_t target_reset;
	uint32_t data;
} __attribute__((__packed__));

struct ml_mux_ctrl_msg {
	uint8_t id;
	union {
		struct ml_mux_hello_msg hello;
		struct ml_mux_open_close_msg open_close;
		struct ml_mux_reset_msg reset;
	} u;
} __attribute__((__packed__));

static int ml_mux_ctrl_send_hello(struct ml_mux *mux, bool is_ack)
{
	struct ml_mux_ctrl_msg ctrl_msg = {0};
	uint32_t len = 0;

	if (is_ack)
		ctrl_msg.id = ML_MUX_CTRL_MSG_HELLO_ACK;
	else
		ctrl_msg.id = ML_MUX_CTRL_MSG_HELLO;
	ctrl_msg.u.hello.version = MUX_VERSION;
	ctrl_msg.u.hello.max_rx_len = mux->ctrl->max_rx_len;
	len = sizeof(ctrl_msg.id) + sizeof(struct ml_mux_hello_msg);

	return ml_mux_send_msg(mux->client.ch, len, &ctrl_msg);
}

void ml_mux_ctrl_chan_open_remote(struct ml_mux *mux,
	struct ml_mux_chan *chan, bool open)
{
	struct ml_mux_ctrl_msg ctrl_msg = {0};
	uint32_t len = 0;

	if (open)
		ctrl_msg.id = ML_MUX_CTRL_MSG_OPEN;
	else
		ctrl_msg.id = ML_MUX_CTRL_MSG_CLOSE;
	ctrl_msg.u.open_close.chan = chan->local_num;
	strlcpy(ctrl_msg.u.open_close.name, chan->name, ML_MUX_CH_NAME_LEN);
	len = sizeof(ctrl_msg.id) + sizeof(struct ml_mux_open_close_msg);

	if (ml_mux_send_msg(mux->client.ch, len, &ctrl_msg))
		dev_dbg(mux->ctrl->dev, "%s: failed to send msg %d\n", __func__,
			ctrl_msg.id);
}

static void ml_mux_ctrl_send_open_chans(struct ml_mux *mux)
{
	struct ml_mux_chan *chan;
	int i;

	if (!idr_is_empty(&mux->chan_idr)) {
		idr_for_each_entry(&mux->chan_idr, chan, i) {
			if (i != CTRL_CHAN)
				ml_mux_ctrl_chan_open_remote(mux, chan, true);
		}
	}
}

static void ml_mux_handle_reset(struct ml_mux *mux)
{
	/* Reset channels */
	ml_mux_reset_channels(mux);

	/* Reset the ml_mux */
	ml_mux_reset_mux(mux);
}

static void ml_mux_process_hello(struct ml_mux *mux, uint32_t len,
				struct ml_mux_hello_msg *h_msg, bool is_ack)
{
	struct device *dev = mux->ctrl->dev;

	dev_dbg(dev, "%s: hello msg process\n", __func__);
	if (len != sizeof(*h_msg)) {
		dev_err(dev, "%s: hello msg is improper length\n", __func__);
		return;
	}

	if (mux->remote_is_up) {
		if (is_ack) {
			dev_warn(dev, "%s: repeated hello ack, ignoring\n",
				 __func__);
			return;
		}
		dev_warn(dev, "%s: repeated hello: assume reset\n", __func__);
		ml_mux_handle_reset(mux);
	}
	mux->remote_is_up = true;

	if (mux->ctrl->max_tx_len > h_msg->max_rx_len)
		mux->ctrl->max_tx_len = h_msg->max_rx_len;

	/* Reply to hello with hello ack */
	if (!is_ack) {
		if (ml_mux_ctrl_send_hello(mux, true)) {
			dev_err(dev, "%s: failed to send ack\n", __func__);
			return;
		}
	}
	ml_mux_ctrl_send_open_chans(mux);
}

static void ml_mux_process_open(struct ml_mux *mux, uint32_t len,
				struct ml_mux_open_close_msg *msg)
{
	struct device *dev = mux->ctrl->dev;
	struct ml_mux_chan *chan;
	int ret;

	dev_dbg(dev, "%s: open msg process\n", __func__);

	if (len != sizeof(*msg)) {
		dev_err(dev, "%s: open msg is improper length\n", __func__);
		return;
	}

	if (msg->chan > MAX_CHAN || msg->chan == INVALID_CHAN ||
			msg->chan == CTRL_CHAN) {
		dev_err(dev, "%s: invalid channel number %hhu\n", __func__,
			msg->chan);
		return;
	}

	chan = ml_mux_find_chan_byname(mux, msg->name);
	if (!chan) {
		chan = devm_kzalloc(dev, sizeof(*chan),
			GFP_KERNEL);
		if (!chan)
			return;

		ret = idr_alloc(&mux->chan_idr, chan, 1,
				MAX_CHAN + 1, GFP_KERNEL);
		if (ret < 0) {
			devm_kfree(dev, chan);
			dev_err(dev,
				"%s: ml_mux unable to create channel name=%s\n",
				__func__, msg->name);
			return;
		}

		chan->local_num = ret;
		chan->remote_num = msg->chan;
		chan->mux = mux;
		chan->cl = NULL;
		strlcpy(chan->name, msg->name, ML_MUX_CH_NAME_LEN);
	} else {
		if (chan->remote_num != INVALID_CHAN) {
			dev_warn(dev, "%s: remote chan already open", __func__);
			return;
		}
		chan->remote_num = msg->chan;
		chan->cl->notify_open(chan->cl, true);
	}
}

static void ml_mux_process_close(struct ml_mux *mux, uint32_t len,
				struct ml_mux_open_close_msg *msg)
{
	struct device *dev = mux->ctrl->dev;
	struct ml_mux_chan *chan;

	dev_dbg(dev, "%s: close msg process\n", __func__);

	if (len != sizeof(*msg)) {
		dev_err(dev, "%s: open msg is improper length\n", __func__);
		return;
	}

	if (msg->chan > MAX_CHAN || msg->chan == INVALID_CHAN ||
			msg->chan == CTRL_CHAN) {
		dev_err(dev, "%s: invalid channel number %hhu\n", __func__,
			msg->chan);
		return;
	}

	chan = ml_mux_find_chan_byname(mux, msg->name);
	if (!chan) {
		dev_warn(dev, "%s: channel not found!\n", __func__);
	} else {
		if (chan->cl) {
			chan->remote_num = INVALID_CHAN;
			chan->cl->notify_open(chan->cl, false);
		} else {
			idr_remove(&mux->chan_idr, chan->local_num);
			devm_kfree(dev, chan);
		}
	}
}

static void ml_mux_ctrl_msg_process(struct ml_mux_client *cl,
				uint32_t len, void *buf)
{
	struct ml_mux *mux = container_of(cl, struct ml_mux, client);
	struct ml_mux_ctrl_msg *msg = (struct ml_mux_ctrl_msg *)buf;
	int msg_len = len - sizeof(msg->id);

	switch (msg->id) {
	case ML_MUX_CTRL_MSG_HELLO:
		ml_mux_process_hello(mux, msg_len, &msg->u.hello, false);
		break;

	case ML_MUX_CTRL_MSG_HELLO_ACK:
		ml_mux_process_hello(mux, msg_len, &msg->u.hello, true);
		break;

	case ML_MUX_CTRL_MSG_OPEN:
		ml_mux_process_open(mux, msg_len, &msg->u.open_close);
		break;

	case ML_MUX_CTRL_MSG_CLOSE:
		ml_mux_process_close(mux, msg_len, &msg->u.open_close);
		break;

	case ML_MUX_CTRL_MSG_RESET:
		dev_err(mux->ctrl->dev, "%s: ignore reset msg\n", __func__);
		break;

	default:
		dev_err(mux->ctrl->dev, "%s: unknown msg\n", __func__);
		break;
	}
}

int ml_mux_init_ctrl_client(struct ml_mux *mux)
{
	int ret = 0;
	struct device *dev = mux->ctrl->dev;
	struct ml_mux_chan *ch;

	mux->client.dev = mux->ctrl->dev;
	mux->client.receive_cb = ml_mux_ctrl_msg_process;
	mux->client.notify_open = NULL;

	ch = devm_kzalloc(dev, sizeof(*ch), GFP_KERNEL);
	if (!ch)
		return -ENOMEM;

	mutex_lock(&mux->chans_lock);
	ret = idr_alloc(&mux->chan_idr, ch, CTRL_CHAN,
			CTRL_CHAN + 1, GFP_KERNEL);
	if (ret == CTRL_CHAN) {
		dev_dbg(dev, "ctrl channel number assigned\n");
		ch->local_num = CTRL_CHAN;
		ch->remote_num = CTRL_CHAN;
		ch->mux = mux;
		ch->cl = &mux->client;
		strlcpy(ch->name, CTRL_CH_NAME, ML_MUX_CH_NAME_LEN);
		mux->client.ch = ch;
	} else {
		mutex_unlock(&mux->chans_lock);
		dev_err(dev, "Failed to allocate ctrl channel (err %d)\n", ret);
		return -EINVAL;
	}

	ret = ml_mux_ctrl_send_hello(mux, false);
	if (ret)
		dev_warn(dev, "failed to send initial hello to remote mux\n");

	mutex_unlock(&mux->chans_lock);
	return 0;
}

