/*
 * aon-gzl.c
 *
 * AON Magic Leap gazelle pipe driver
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/mailbox_client.h>
#include <linux/tegra-aon.h>
#include <linux/input.h>
#include <linux/input/mltotem_ioctl.h>
#include "aon-gzl-messages.h"

#define TOTEM_INPUT_EVT_MAX_SZ      29
#define TOTEM_INPUT_EVT_MAX_PAYLOAD (TOTEM_INPUT_EVT_MAX_SZ + 1)
#define TOTEM_PKT_INPUT             0x00

struct mlt_pkt {
	u8          totem_id;      /**< one of TOTEM_ID_x */
	uint32_t    time_sync;     /**< not used */
	u8          payload_len;   /**< <= TOTEM_INPUT_EVT_MAX_SZ */
	u8          packet_type;   /**< should be TOTEM_PKT_INPUT */
	u8          payload[TOTEM_INPUT_EVT_MAX_SZ];
} __packed;

#define AON_GZL_TX_BLOCK_PERIOD 100
#define AON_GZL_PKT_SZ          sizeof(struct mlt_pkt)

struct aon_gzl_data {
    struct mbox_client                 cl;
    struct mbox_chan                   *mbox;
    struct completion                  wait_write_done;
    struct aon_gzl_resp                resp;
    struct input_dev                   *idev1;
    struct input_dev                   *idev2;
};

static void handle_notification(const struct aon_gzl_data *gzl,
                                const struct mlt_pkt *rx_pkt)
{
    struct input_dev     *idev;
    const u32            *p_u32;
    int                  i_evnt_val;
    int                  i = 0;

    /* Do sanity checking of received packet */
    if (rx_pkt->payload_len == 0 ||
        rx_pkt->payload_len > TOTEM_INPUT_EVT_MAX_PAYLOAD) {
        pr_debug("rx: payload_len = %d\n", rx_pkt->payload_len);
        return;
    }

    if (rx_pkt->packet_type == TOTEM_PKT_INPUT) {
        /* Input event data goes to input device */
        if (rx_pkt->totem_id == TOTEM_ID_1)
            idev = gzl->idev1;
        else if (rx_pkt->totem_id == TOTEM_ID_2)
            idev = gzl->idev2;
        else {
            pr_debug("rx: invalid totem_id = %d\n", rx_pkt->totem_id);
            return;
        }
    }

    while (i < rx_pkt->payload_len - sizeof(rx_pkt->packet_type)) {
        p_u32 = (const u32 *)((const u8 *)(rx_pkt->payload + i));
        i_evnt_val = le32_to_cpu(*p_u32);
        input_event(idev, EV_MSC, MSC_RAW, i_evnt_val);
        i += 4;
    }
    input_sync(idev);
}

static void aon_receive_ivc_gzl_msg(struct mbox_client *cl, void *rx_msg)
{
    struct tegra_aon_mbox_msg     *msg;
    struct aon_gzl_data        *gzl_data;
    const struct aon_gzl_resp  *resp;
    const struct mlt_pkt       *rx_pkt;

    gzl_data = container_of(cl, struct aon_gzl_data, cl);

    msg = (struct tegra_aon_mbox_msg *)rx_msg;
    resp = (void *)msg->data;

    if (resp) {
        pr_debug("%s: response type = %d\n", __func__, resp->type);
        switch (resp->type) {
            case AON_GZL_MSG_WRITE_ACK:
                gzl_data->resp.data.write_resp.status =
                    resp->data.write_resp.status;
                complete(&gzl_data->wait_write_done);
                break;

            case AON_GZL_MSG_NOTIFICATION:
                if (resp->data.notification.length == AON_GZL_PKT_SZ) {
                    rx_pkt = (const struct mlt_pkt *)resp->data.notification.data;
                    handle_notification(gzl_data, rx_pkt);
                }
                break;

            default:
                pr_debug("Invalid response type\n");
                return;
        }
    }
}

static int tegra_aon_gzl_input_dev_register(struct input_dev *idev,
                                            struct device *parent,
                                            const char *name)
{
    int ret;

    idev->name = name;
    idev->id.bustype = BUS_SPI;
    idev->dev.parent = parent;
    idev->evbit[0] = BIT(EV_MSC);
    __set_bit(MSC_RAW, idev->mscbit);
    ret = input_register_device(idev);
    if (ret)
        dev_err(parent, "failed to register input device %s, err %d\n",
                name, ret);
    return ret;
}

static void aon_gzl_set_notification(struct aon_gzl_data *aon_gzl, bool enable)
{
    struct aon_gzl_req req;
    struct tegra_aon_mbox_msg msg;
    int ret;

    req.type = AON_GZL_MSG_NOTIFY_REQ;
    req.data.notify_req.enable_notification = enable;

    msg.length = sizeof(struct aon_gzl_req);
    msg.data = (void *)&req;

    ret = mbox_send_message(aon_gzl->mbox, (void *)&msg);

    if (ret < 0)
        dev_err(aon_gzl->cl.dev, "mbox_send_message failed\n");
}

static int tegra_aon_gzl_probe(struct platform_device *pdev)
{
    struct aon_gzl_data *aon_gzl = NULL;
    int err;

    dev_err(&pdev->dev, "%s: enter\n", __func__);
    aon_gzl = devm_kzalloc(&pdev->dev, sizeof(struct aon_gzl_data), GFP_KERNEL);
    if (!aon_gzl) {
        dev_err(&pdev->dev, "Memory allocation failure\n");
        return -ENOMEM;
    }

    aon_gzl->idev1 = devm_input_allocate_device(&pdev->dev);
    if (!aon_gzl->idev1) {
        return -ENOMEM;
    }

    aon_gzl->idev2 = devm_input_allocate_device(&pdev->dev);
    if (!aon_gzl->idev2) {
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, aon_gzl);
    init_completion(&aon_gzl->wait_write_done);

    aon_gzl->cl.dev = &pdev->dev;
    aon_gzl->cl.tx_block = true;
    aon_gzl->cl.tx_tout = AON_GZL_TX_BLOCK_PERIOD;
    aon_gzl->cl.knows_txdone = false;
    aon_gzl->cl.rx_callback = aon_receive_ivc_gzl_msg;

    aon_gzl->mbox = mbox_request_channel(&aon_gzl->cl, 0);
    if (IS_ERR(aon_gzl->mbox)) {
        dev_err(&pdev->dev, "can't get mailbox channel (%d)\n",
                (int)PTR_ERR(aon_gzl->mbox));
        err = PTR_ERR(aon_gzl->mbox);
        return err;
    }
    dev_info(&pdev->dev, "aon_gzl->mbox = %p\n", aon_gzl->mbox);

    err = tegra_aon_gzl_input_dev_register(aon_gzl->idev1, &pdev->dev,
                                           "ml-totem1");
    if (err) {
        dev_err(&pdev->dev, "Failed to register ml-totem1, err: %d\n", err);
        goto err_free_mbox;
    }

    err = tegra_aon_gzl_input_dev_register(aon_gzl->idev2, &pdev->dev,
                                           "ml-totem2");
    if (err) {
        dev_err(&pdev->dev, "Failed to register ml-totem2, err: %d\n", err);
        goto err_unreg_idev1;
    }

    aon_gzl_set_notification(aon_gzl, true);

    return 0;

err_unreg_idev1:
    input_unregister_device(aon_gzl->idev1);

err_free_mbox:
    mbox_free_channel(aon_gzl->mbox);

    return err;
}

static int tegra_aon_gzl_remove(struct platform_device *pdev)
{
    struct aon_gzl_data *aon_gzl;
    dev_dbg(&pdev->dev, "%s\n", __func__);
    aon_gzl = dev_get_drvdata(&pdev->dev);
    input_unregister_device(aon_gzl->idev2);
    input_unregister_device(aon_gzl->idev1);
    mbox_free_channel(aon_gzl->mbox);

    return 0;
}


static const struct of_device_id tegra_aon_gzl_of_match[] = {
    {
        .compatible = "ml,aon-gzl",
    },
    {}
};
MODULE_DEVICE_TABLE(of, tegra_aon_gzl_of_match);

static struct platform_driver tegra_aon_gzl_driver = {
    .driver = {
        .name   = "aon-ml-gzl",
        .owner    = THIS_MODULE,
        .of_match_table = of_match_ptr(tegra_aon_gzl_of_match),
    },
    .remove = tegra_aon_gzl_remove,
    .probe =  tegra_aon_gzl_probe,
};
module_platform_driver(tegra_aon_gzl_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("Magic Leap AON GZL Driver");
MODULE_LICENSE("GPL v2");
