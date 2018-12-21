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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mailbox_client.h>
#include <linux/tegra-aon.h>
#include <linux/jiffies.h>
#include <linux/completion.h>
#include <linux/ml-mux-ctrl.h>
#include <linux/ml-mux-client.h>

#include "aon-wearable-messages.h"

#define TX_BLOCK_PERIOD 60
#define WAIT_MS 60
#define MUX_AON_QUEUE_LEN 20

struct aon_wearable_data {
	struct device *dev;
	struct mbox_client cl;
	struct mbox_chan *mbox;
	struct ml_mux_ctrl mlmux;
	struct completion *data_completion;
};

static void mux_aon_send_msg(struct ml_mux_ctrl *mlmux, uint32_t length,
					const void *data)
{
	struct aon_wearable_data *mux_data = container_of(mlmux,
					struct aon_wearable_data, mlmux);
	struct aon_wearable_request req;
	struct tegra_aon_mbox_msg msg;

	int ret;

	if (length > TEGRA_IVC_MAX_DATA_SIZE) {
		dev_err(mux_data->dev, "message size exceeds the Tx Max length set in mlmux\n");
		return;
	}

	memcpy(req.data, data, length);
	req.length = length;

	msg.length = sizeof(struct aon_wearable_request);
	msg.data = (void *)&req;
	reinit_completion(mux_data->data_completion);

	ret = mbox_send_message(mux_data->mbox, (void *)&msg);

	if (ret < 0) {
		dev_err(mux_data->dev, "mbox_send_message failed\n");
		return;
	}

	ret = wait_for_completion_timeout(mux_data->data_completion,
				msecs_to_jiffies(WAIT_MS));

	if (!ret)
		dev_err(mux_data->dev, "mbox_send_message timeout\n");
}

static void aon_mux_recv_msg(struct mbox_client *cl,
					const void *rx_msg)
{
	struct aon_wearable_data *mux_data = container_of(cl,
					struct aon_wearable_data, cl);
	struct aon_wearable_response *resp;
	struct tegra_aon_mbox_msg *msg;

	msg = (struct tegra_aon_mbox_msg *)rx_msg;

	resp = (struct aon_wearable_response *)msg->data;

	if (resp->length == 0) {
		/* ACK sent from SPE, drop it */
		complete(mux_data->data_completion);
		if (resp->status == AON_WEARABLE_STATUS_ERROR)
			dev_err(mux_data->dev, "SPE Error: Fail to send msg to SPE");
	} else if (resp->status == AON_WEARABLE_STATUS_ERROR) {
		dev_err(mux_data->dev, "SPE Error: %s\n", (char *)resp->data);
	} else {
		ml_mux_msg_receive(&mux_data->mlmux, resp->length,
						(void *)&resp->data);
	}
}

static void aon_wearable_init_ml_mux_ctrl(struct device *dev,
		struct ml_mux_ctrl *mlmux)
{
	mlmux->dev = dev;
	strlcpy(mlmux->name, dev->of_node->name, sizeof(mlmux->name));
	mlmux->queue_len = MUX_AON_QUEUE_LEN;
	mlmux->max_tx_len = TEGRA_IVC_MAX_DATA_SIZE;
	mlmux->max_rx_len = TEGRA_IVC_MAX_DATA_SIZE;

	mlmux->tx_data = (void *)mux_aon_send_msg;
}

static int aon_wearable_probe(struct platform_device *pdev)
{
	struct aon_wearable_data *aondata;
	struct device *dev = &pdev->dev;

	int ret;

	aondata = devm_kzalloc(&pdev->dev, sizeof(*aondata), GFP_KERNEL);
	if (!aondata)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, aondata);
	aondata->dev = &pdev->dev;
	aondata->cl.dev = &pdev->dev;
	aondata->cl.tx_block = true;
	aondata->cl.tx_tout = TX_BLOCK_PERIOD;
	aondata->cl.knows_txdone = false;
	aondata->cl.rx_callback = (void *)aon_mux_recv_msg;

	aondata->data_completion = devm_kzalloc(&pdev->dev,
				sizeof(struct completion), GFP_KERNEL);

	if (!aondata->data_completion) {
		dev_err(dev, "aon_wearable out of memory.\n");
		return -ENOMEM;
	}

	init_completion(aondata->data_completion);

	aondata->mbox = mbox_request_channel(&aondata->cl, 0);

	if (IS_ERR(aondata->mbox)) {
		dev_warn(&pdev->dev,
			 "can't get mailbox channel (%d)\n",
			 (int)PTR_ERR(aondata->mbox));
		return PTR_ERR(aondata->mbox);
	}

	aon_wearable_init_ml_mux_ctrl(dev, &aondata->mlmux);

	ret = ml_mux_ctrl_register(&aondata->mlmux);
	if (ret)
		goto exit;

	dev_info(dev, "aon_wearable driver add in\n");
	return ret;

exit:
	mbox_free_channel(aondata->mbox);
	return ret;
}

static int aon_wearable_remove(struct platform_device *pdev)
{
	struct aon_wearable_data *aondata;

	aondata = dev_get_drvdata(&pdev->dev);
	mbox_free_channel(aondata->mbox);
	ml_mux_ctrl_unregister(&aondata->mlmux);

	return 0;
}

static const struct of_device_id aon_wearable_of_match[] = {
	{
		.compatible = "ml,aon-wearable",
	},
	{},
};
MODULE_DEVICE_TABLE(of, aon_wearable_of_match);

static struct platform_driver aon_wearable_driver = {
	.driver = {
		.name   = "aon-ml-wearable",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(aon_wearable_of_match),
	},
	.probe = aon_wearable_probe,
	.remove = aon_wearable_remove,
};

module_platform_driver(aon_wearable_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("AON Wearable DATA transfer driver");
MODULE_LICENSE("GPL v2");
