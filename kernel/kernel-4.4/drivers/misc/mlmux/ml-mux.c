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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/idr.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/ml-mux-ctrl.h>
#include <linux/ml-mux-client.h>
#include "ml-mux-ctrl-chan.h"
#include "ml-mux.h"

#define CREATE_TRACE_POINTS
#include "ml_mux_trace.h"

static LIST_HEAD(ctrl_list);
static DEFINE_MUTEX(ctrl_mutex);

struct ml_mux_queue {
	struct ml_mux_frame *frame;
	uint32_t num_frames;
	uint32_t frame_len;
	uint32_t slots_free;
	uint32_t idx_cur;
	uint32_t idx_free;
	uint8_t seq_num;
	bool q_stop;
};

#define LEN_SIZE (sizeof(uint16_t))
#define HEADER_SIZE (sizeof(struct ml_mux_frame))
#define FRAME_LEN_OVERHEAD (HEADER_SIZE - LEN_SIZE)

static struct ml_mux_queue *ml_mux_alloc_queue(struct device *dev,
					uint32_t num_frames,
					uint32_t buf_size)
{
	struct ml_mux_queue *queue;

	queue = devm_kzalloc(dev, sizeof(*queue), GFP_KERNEL);
	if (!queue)
		return ERR_PTR(-ENOMEM);

	queue->frame = devm_kzalloc(dev, num_frames * buf_size, GFP_KERNEL);
	if (!queue->frame)
		return ERR_PTR(-ENOMEM);

	queue->frame_len = buf_size;

	return queue;
}

static inline void ml_mux_init_queue(struct ml_mux_ctrl *ctrl,
				struct ml_mux_queue *queue)
{
	queue->slots_free = ctrl->queue_len;
	queue->num_frames = ctrl->queue_len;
}

static int ml_mux_ctrl_enqueue(struct ml_mux_chan *chan,
				uint16_t len, void *msg)
{
	unsigned long flags;
	struct ml_mux_frame *frame;
	struct ml_mux *mux = chan->mux;
	struct ml_mux_queue *queue = mux->tx_queue;

	spin_lock_irqsave(&mux->q_lock, flags);
	if (mux->tx_queue->q_stop) {
		spin_unlock_irqrestore(&mux->q_lock, flags);
		dev_warn(mux->ctrl->dev,
			"%s: q processing is stopped. msg cant be delivered.\n",
			__func__);
		return -ESRCH;
	}

	while (mux->tx_queue->slots_free == 0) {
		spin_unlock_irqrestore(&mux->q_lock, flags);
		wait_event_interruptible(mux->q_wait,
			(mux->tx_queue->slots_free > 0));
		spin_lock_irqsave(&mux->q_lock, flags);
		if (mux->tx_queue->q_stop) {
			spin_unlock_irqrestore(&mux->q_lock, flags);
			dev_warn(mux->ctrl->dev,
				 "%s: q processing stopped while waiting\n",
				 __func__);
			return -ESRCH;
		}
	}
	mux->tx_queue->slots_free--;

	frame = (struct ml_mux_frame *)((uint8_t *)queue->frame +
		queue->frame_len * queue->idx_free);
	frame->len = len + FRAME_LEN_OVERHEAD;
	frame->seq_num = queue->seq_num;
	frame->channel = chan->remote_num;
	memcpy(frame->buf, msg, (size_t)len);

	/* increase index of free frame and frame sequence number */
	queue->idx_free = queue->idx_free + 1 == queue->num_frames ?
			0 : queue->idx_free + 1;
	queue->seq_num++;
	spin_unlock_irqrestore(&mux->q_lock, flags);

	wake_up_process(mux->process_queue);

	return 0;
}

struct ml_mux_chan *ml_mux_find_chan_byname(struct ml_mux *mux,
						char *name)
{
	struct ml_mux_chan *ch_idx = NULL;
	struct ml_mux_chan *chan = NULL;
	int i;

	if (!idr_is_empty(&mux->chan_idr)) {
		idr_for_each_entry(&mux->chan_idr, ch_idx, i) {
			if (!strncmp(ch_idx->name, name, ML_MUX_CH_NAME_LEN)) {
				chan = ch_idx;
				break;
			}
		}
	}

	return chan;
}

static int ml_mux_msg_send_thread(void *argp)
{
	struct ml_mux *mux = (struct ml_mux *)argp;
	struct ml_mux_queue *queue = mux->tx_queue;
	struct ml_mux_frame *frame;
	unsigned long flags;
	int idx;

	while (1) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		spin_lock_irqsave(&mux->q_lock, flags);
		while (queue->slots_free == queue->num_frames) {
			spin_unlock_irqrestore(&mux->q_lock, flags);

			if (unlikely(kthread_should_stop()))
				goto exit;
			schedule();

			set_current_state(TASK_UNINTERRUPTIBLE);
			spin_lock_irqsave(&mux->q_lock, flags);
		}
		set_current_state(TASK_RUNNING);
		idx = queue->idx_cur;
		frame = (struct ml_mux_frame *)((uint8_t *)queue->frame +
			queue->frame_len * idx);
		spin_unlock_irqrestore(&mux->q_lock, flags);

		trace_ml_mux_msg_send(frame);

		mux->ctrl->tx_data(mux->ctrl, frame->len + LEN_SIZE, frame);

		spin_lock_irqsave(&mux->q_lock, flags);
		queue->idx_cur = queue->idx_cur + 1 == queue->num_frames ?
				0 : queue->idx_cur + 1;
		queue->slots_free++;
		spin_unlock_irqrestore(&mux->q_lock, flags);
		wake_up_interruptible(&mux->q_wait);
	}

exit:
	return 0;
}

/* Wait until the send thread has drained */
static void ml_mux_drain_queue(struct ml_mux *mux)
{
	struct ml_mux_queue *queue = mux->tx_queue;
	unsigned long flags;

	mux->tx_queue->q_stop = true;

	spin_lock_irqsave(&mux->q_lock, flags);

	/* If queue already drained, try to wake up the enqueue path so it
	 * can see the q_stop flag. Once it sees q_stop, it won't queue up
	 * anymore.
	 */
	if (queue->slots_free == queue->num_frames) {
		spin_unlock_irqrestore(&mux->q_lock, flags);
		wake_up_interruptible(&mux->q_wait);
		goto restart_q;
	}

	while (queue->slots_free != queue->num_frames) {
		spin_unlock_irqrestore(&mux->q_lock, flags);
		wait_event_interruptible(mux->q_wait,
				(queue->slots_free == queue->num_frames));
		spin_lock_irqsave(&mux->q_lock, flags);
	}
	spin_unlock_irqrestore(&mux->q_lock, flags);

restart_q:
	mux->tx_queue->q_stop = false;
}

static int ml_mux_init_thread(struct ml_mux *mux)
{
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	mux->process_queue = kthread_create(ml_mux_msg_send_thread,
					   mux, "%s",
					   mux->ctrl->name);
	if (IS_ERR(mux->process_queue)) {
		pr_err("failed to create process queue task\n");
		return PTR_ERR(mux->process_queue);
	}

	if (mux->ctrl->is_highpri) {
		dev_info(mux->ctrl->dev, "%s: q set to rt priority\n",
			__func__);
		sched_setscheduler(mux->process_queue, SCHED_FIFO, &param);
	}

	return 0;
}


/**
 * ml_mux_send_msg - function used to receive message, callbacks to client.
 * @chan:	Pointer to the receiving controller.
 * @len:	Length of the message in bytes.
 * @msg:	Pointer to the message
 *
 * This is used by the controller to deliver message to the end client.
 */
int ml_mux_send_msg(struct ml_mux_chan *chan, uint32_t len, void *msg)
{
	struct ml_mux *mux;
	struct device *dev;

	/* Sanity checks */
	if (!chan || !chan->cl) {
		pr_err("channel or client is null\n");
		return -EINVAL;
	}
	dev = chan->cl->dev;

	mux = chan->mux;
	if (!mux) {
		pr_err("mux is null\n");
		return -EINVAL;
	}

	if (!mux->remote_is_up && chan->remote_num != CTRL_CHAN) {
		dev_dbg(dev, "%s: mux remote is not up\n", __func__);
		return -ENOTCONN;
	}

	if (chan->remote_num == INVALID_CHAN) {
		dev_dbg(dev, "%s: remote channel is not open\n", __func__);
		return -ENOTCONN;
	}

	if ((len + HEADER_SIZE) > mux->ctrl->max_tx_len) {
		dev_dbg(dev, "%s: write length outside of ctrl limit range\n",
			__func__);
		return -EINVAL;
	}

	return ml_mux_ctrl_enqueue(chan, len, msg);
}
EXPORT_SYMBOL_GPL(ml_mux_send_msg);

/**
 * ml_mux_msg_receive - function used to receive message, callbacks to client.
 * @ctrl:	Pointer to the receiving controller.
 * @len:	Length of the message in bytes.
 * @buf:	Pointer to the message
 *
 * This is used by the controller to deliver message to the end client.
 */
void ml_mux_msg_receive(struct ml_mux_ctrl *ctrl, uint32_t len, void *msg)
{
	struct ml_mux_frame *frame = (struct ml_mux_frame *)msg;
	struct ml_mux_chan *chan;
	struct ml_mux *mux = NULL;
	struct ml_mux *mux_idx;
	uint32_t buf_len;
	uint8_t ch;

	list_for_each_entry(mux_idx, &ctrl_list, list_entry) {
		if (mux_idx->ctrl->dev == ctrl->dev) {
			mux = mux_idx;
			break;
		}
	}
	if (!mux) {
		dev_err(ctrl->dev, "%s: ml_mux is not found!\n", __func__);
		return;
	}

	/* Sanity check */
	if (len < HEADER_SIZE + 1) {
		dev_err(ctrl->dev, "%s: ml_mux msg is too small\n", __func__);
		return;
	}

	trace_ml_mux_msg_recv(frame);

	if (frame->seq_num != mux->expect_seqn) {
		dev_warn(ctrl->dev, "%s: Warning! Lost packets! [%hhu-%hhu]",
			__func__, mux->expect_seqn, frame->seq_num - 1);
	}
	mux->expect_seqn = frame->seq_num + 1;

	buf_len = frame->len - FRAME_LEN_OVERHEAD;
	ch = frame->channel;
	if (ch == INVALID_CHAN || ch >= MAX_CHAN) {
		dev_err(ctrl->dev, "%s: Invalid chan (%d)\n", __func__, ch);
		return;
	}

	mutex_lock(&mux->chans_lock);

	chan = idr_find(&mux->chan_idr, ch);
	if (!chan) {
		dev_err(ctrl->dev, "%s: channel %d not found\n", __func__, ch);
		goto unlock;
	}

	if (!chan->cl) {
		dev_err(ctrl->dev, "%s: local channel-%hhu has no client\n",
			__func__, chan->local_num);
		goto unlock;
	}
	chan->cl->receive_cb(chan->cl, buf_len, frame->buf);

unlock:
	mutex_unlock(&mux->chans_lock);
}
EXPORT_SYMBOL_GPL(ml_mux_msg_receive);

/**
 * ml_mux_request_channel - Client request of channel
 * @client:	Pointer to the client requesting the channel.
 * @name:	Pointer to the name of channel requested.
 *
 * This requests a channel from controller. Client is to call this
 * function from ->probe when it wants to create a channel of communication.
 * If channel already exists by indicated name it will try to become it's
 * client. If channel is already taken -EBUSY is returned. If mux controller
 * is not yet ready, -EPROBE_DEFER is returned. Successful request scenario
 * returns ML_MUX_CH_REQ_SUCCESS. Successful request and remote is already opened
 * scenario returns ML_MUX_CH_REQ_OPENED.
 */
int ml_mux_request_channel(struct ml_mux_client *client, char *name)
{
	struct ml_mux *mux_idx;
	struct ml_mux *mux = NULL;
	struct ml_mux_chan *chan;
	struct device_node *np;
	const __be32 *ph;
	int ret = ML_MUX_CH_REQ_SUCCESS;

	if (!client || !client->dev) {
		pr_err("%s: no client/dev given\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	if (!client->dev->of_node) {
		dev_err(client->dev, "%s: no of_node\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	if (!client->notify_open || !client->receive_cb) {
		dev_err(client->dev, "%s: no callback funcs\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	ph = of_get_property(client->dev->of_node, "mlmux", NULL);
	if (!ph) {
		dev_err(client->dev, "%s: mlmux prop not found\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	np = of_find_node_by_phandle(be32_to_cpup(ph));
	if (!np) {
		dev_err(client->dev, "%s: can't find mlmux node\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	mutex_lock(&ctrl_mutex);
	list_for_each_entry(mux_idx, &ctrl_list, list_entry) {
		if (mux_idx->ctrl->dev->of_node == np) {
			mux = mux_idx;
			break;
		}
	}
	mutex_unlock(&ctrl_mutex);
	if (!mux) {
		ret = -EPROBE_DEFER;
		goto exit;
	}

	mutex_lock(&mux->chans_lock);

	chan = ml_mux_find_chan_byname(mux, name);
	if (!chan) {
		chan = devm_kzalloc(mux->ctrl->dev, sizeof(*chan), GFP_KERNEL);
		if (!chan) {
			ret = -ENOMEM;
			goto exit_unlock;
		}
		if (!try_module_get(mux->ctrl->dev->driver->owner)) {
			ret = -ENODEV;
			goto exit_unlock;
		}
		ret = idr_alloc(&mux->chan_idr, chan, 0, MAX_CHAN, GFP_KERNEL);
		if (ret < 0)
			goto exit_put;

		chan->local_num = ret;
		chan->remote_num = INVALID_CHAN;
		chan->mux = mux;
		chan->cl = client;
		client->ch = chan;
		strlcpy(chan->name, name, ML_MUX_CH_NAME_LEN);
		ret = ML_MUX_CH_REQ_SUCCESS;
	} else if (!chan->cl) {
		if (!try_module_get(mux->ctrl->dev->driver->owner)) {
			ret = -ENODEV;
			goto exit_unlock;
		}
		chan->cl = client;
		client->ch = chan;
		ret = ML_MUX_CH_REQ_OPENED;
	} else {
		dev_err(client->dev, "%s: chan has a client\n", __func__);
		ret = -EBUSY;
		goto exit_unlock;
	}

	device_link_add(client->dev, mux->ctrl->dev, DL_FLAG_STATELESS);

	/* If remote is already up we should notify it about new channel */
	if (mux->remote_is_up)
		ml_mux_ctrl_chan_open_remote(mux, chan, true);

	mutex_unlock(&mux->chans_lock);
	return ret;

exit_put:
	module_put(mux->ctrl->dev->driver->owner);
exit_unlock:
	mutex_unlock(&mux->chans_lock);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(ml_mux_request_channel);

/**
 * ml_mux_release_channel - Client release of channel
 * @chan:	Pointer to the channel being released.
 *
 * This releases channel from client. Client is to call this
 * function only in its own context, and not from inside any of the
 * callbacks. This should be used on device or driver removal.
 */
void ml_mux_release_channel(struct ml_mux_chan *chan)
{
	struct ml_mux *mux;

	if (!chan)
		return;
	mux = chan->mux;
	mutex_lock(&mux->chans_lock);
	ml_mux_ctrl_chan_open_remote(mux, chan, false);
	device_link_remove(chan->cl->dev, mux->ctrl->dev);
	chan->cl = NULL;

	if (chan->remote_num == INVALID_CHAN) {
		idr_remove(&mux->chan_idr, chan->local_num);
		devm_kfree(mux->ctrl->dev, chan);
	}

	mutex_unlock(&mux->chans_lock);
	module_put(mux->ctrl->dev->driver->owner);
}
EXPORT_SYMBOL_GPL(ml_mux_release_channel);

static int ml_mux_ctrl_sanity_check(struct ml_mux_ctrl *ctrl)
{
	int ret = 0;

	if (!ctrl || !ctrl->dev) {
		pr_err("%s: Invalid ctrl/dev passed\n", __func__);
		ret = -EINVAL;
		goto exit;
	}
	if (!ctrl->name[0]) {
		dev_err(ctrl->dev, "reg ml_mux ctrl with no name\n");
		ret = -EINVAL;
		goto exit;
	}
	if (!ctrl->tx_data) {
		dev_err(ctrl->dev, "reg ml_mux ctrl '%s' with no defined tx_data!\n",
			ctrl->name);
		ret = -EINVAL;
		goto exit;
	}
	if (!ctrl->dev->of_node) {
		dev_err(ctrl->dev, "reg ml_mux ctrl '%s' with no dev of_node!\n",
			ctrl->name);
		ret = -EINVAL;
		goto exit;
	}
	if (ctrl->queue_len > ML_MAX_QUEUE_LEN) {
		dev_err(ctrl->dev, "queue length for '%s' out of range!\n",
			ctrl->name);
		ret = -EINVAL;
		goto exit;
	}
	if (ctrl->max_tx_len > ML_MAX_WR_LEN || ctrl->max_tx_len > 0xFFFF) {
		dev_err(ctrl->dev, "max_tx_len for '%s' out of range!\n",
			ctrl->name);
		ret = -EINVAL;
		goto exit;
	}
	if (ctrl->max_rx_len > ML_MAX_RD_LEN || ctrl->max_rx_len > 0xFFFF) {
		dev_err(ctrl->dev, "max_rx_len for '%s' out of range!\n",
			ctrl->name);
		ret = -EINVAL;
		goto exit;
	}

exit:
	return ret;
}

static int ml_mux_init(struct ml_mux *mux)
{
	int ret = 0;

	mux->tx_queue = ml_mux_alloc_queue(mux->ctrl->dev, mux->ctrl->queue_len,
		mux->ctrl->max_tx_len);
	if (IS_ERR(mux->tx_queue)) {
		dev_err(mux->ctrl->dev, "failed to alloc queue\n");
		ret = PTR_ERR(mux->tx_queue);
		goto exit;
	}
	ml_mux_init_queue(mux->ctrl, mux->tx_queue);

	ret = ml_mux_init_thread(mux);
	if (ret) {
		dev_err(mux->ctrl->dev, "failed to init tx thread\n");
		goto exit;
	}

	init_waitqueue_head(&mux->q_wait);
	spin_lock_init(&mux->q_lock);
	mutex_init(&mux->chans_lock);
	idr_init(&mux->chan_idr);
	mux->remote_is_up = false;

exit:
	return ret;
}

/**
 * ml_mux_ctrl_register - Register the controller
 * @ctrl:	Pointer to the controller.
 *
 * This registers the controller. This creates a corresponding mux
 * struct, default control channel, and keeps track of it in the list of
 * controller.
 */
int ml_mux_ctrl_register(struct ml_mux_ctrl *ctrl)
{
	int ret = 0;
	struct ml_mux *mux;

	ret = ml_mux_ctrl_sanity_check(ctrl);
	if (ret)
		goto exit;

	mux = devm_kzalloc(ctrl->dev, sizeof(*mux), GFP_KERNEL);
	if (!mux) {
		ret = -ENOMEM;
		goto exit;
	}
	mux->ctrl = ctrl;
	ret = ml_mux_init(mux);
	if (ret)
		goto exit;

	mutex_lock(&ctrl_mutex);
	list_add_tail(&mux->list_entry, &ctrl_list);
	mutex_unlock(&ctrl_mutex);

	ret = ml_mux_init_ctrl_client(mux);
	if (ret) {
		pr_err("failed to create control client\n");
		goto exit_list_del;
	}

	return ret;

exit_list_del:
	list_del(&mux->list_entry);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(ml_mux_ctrl_register);

static int ml_mux_free_channel(int id, void *ch, void *mb)
{
	struct ml_mux *mux = (struct ml_mux *)mb;
	struct ml_mux_chan *chan = (struct ml_mux_chan *)ch;

	if (!chan || id == CTRL_CHAN)
		return 0;

	if (chan->cl)
		chan->cl->notify_open(chan->cl, false);
	ml_mux_ctrl_chan_open_remote(mux, chan, false);
	module_put(chan->mux->ctrl->dev->driver->owner);

	return 0;
}

/* Notify that the channel is now closed, and reset the remote_num.
 * This must be called with chans_lock held.
 */
void ml_mux_reset_channels(struct ml_mux *mux)
{
	struct ml_mux_chan *chan;
	int i;

	if (!idr_is_empty(&mux->chan_idr)) {
		idr_for_each_entry(&mux->chan_idr, chan, i) {
			/* Don't reset control channel */
			if (chan->remote_num == CTRL_CHAN)
				continue;

			/* If no local clients, free the channel */
			if (!chan->cl) {
				idr_remove(&mux->chan_idr, chan->local_num);
				devm_kfree(mux->ctrl->dev, chan);
				continue;
			}

			chan->cl->notify_open(chan->cl, false);
			chan->remote_num = INVALID_CHAN;
		}
	}
}

/* Reset the mux state, drain outstanding buffers, and then re-init */
void ml_mux_reset_mux(struct ml_mux *mux)
{
	mutex_lock(&ctrl_mutex);
	mux->remote_is_up = false;
	ml_mux_drain_queue(mux);

	mux->tx_queue->seq_num = 0;
	mutex_unlock(&ctrl_mutex);
}

/**
 * ml_mux_ctrl_unregister - Unregister the controller
 * @ctrl:	Pointer to the controller.
 *
 * This unregisters the controller from list of mux controllers.
 * Cleans up the tx queue, closes all the channels notifying both the
 * clients and the remote before freeing the memory. Controller must
 * call unregister before releasing the device.
 */
void ml_mux_ctrl_unregister(struct ml_mux_ctrl *ctrl)
{
	struct ml_mux *mux = NULL;
	struct ml_mux *mux_idx;

	if (!ctrl)
		return;

	mutex_lock(&ctrl_mutex);
	list_for_each_entry(mux_idx, &ctrl_list, list_entry) {
		if (mux_idx->ctrl->dev == ctrl->dev) {
			mux = mux_idx;
			break;
		}
	}
	if (!mux) {
		mutex_unlock(&ctrl_mutex);
		dev_err(ctrl->dev, "ml_mux is not found!\n");
		return;
	}

	list_del(&mux->list_entry);

	/* stop and free all the channels */
	mutex_lock(&mux->chans_lock);
	idr_for_each(&mux->chan_idr, ml_mux_free_channel, mux);
	idr_destroy(&mux->chan_idr);
	mutex_unlock(&mux->chans_lock);

	mux->tx_queue->q_stop = true;
	kthread_stop(mux->process_queue);
	mutex_unlock(&ctrl_mutex);
}
EXPORT_SYMBOL_GPL(ml_mux_ctrl_unregister);

