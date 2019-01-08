/*
 * Copyright (c) 2017, Magic Leap, Inc. All rights reserved.
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

#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/ml-mux-client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define MLMUX_I2C_MAX_DATA_LENGTH 36

#define MLMUX_I2C_8_BIT_READ   0
#define MLMUX_I2C_8_BIT_WRITE  1
#define MLMUX_I2C_16_BIT_READ  2
#define MLMUX_I2C_16_BIT_WRITE 3

struct mlmux_i2c_msg {
	uint8_t type;
	uint8_t seq;
	uint8_t addr;
	uint8_t pad;
	uint16_t reg;
	int16_t length;
	uint8_t data[MLMUX_I2C_MAX_DATA_LENGTH];
} __packed;

struct mlmux_i2c_dev {
	struct device *dev;
	struct i2c_adapter adap;
	bool adap_en;

	struct ml_mux_client client;
	const char *chan_name;

	struct mutex adap_lock;
	struct mutex lock;
	struct completion xfer;

	uint8_t seq;
	struct mlmux_i2c_msg txbuf;
	struct mlmux_i2c_msg rxbuf;

	struct workqueue_struct *notify_q;
};

struct mlmux_i2c_notify_work {
	struct work_struct notify_work;
	struct mlmux_i2c_dev *mlm;
	bool is_open;
};

static int mlmux_i2c_write(struct mlmux_i2c_dev *mlm, struct i2c_msg *msg)
{
	int ret;

	if (msg->len > MLMUX_I2C_MAX_DATA_LENGTH) {
		dev_err(mlm->dev, "length too large: %d\n", msg->len);
		return -EMSGSIZE;
	}

	/* We have no way to tell an 8-bit register address versus a 16-bit
	 * register address. Since it all just goes out the i2c bus
	 * regardless, we just send as an 8-bit write.
	 */
	mlm->txbuf.type = MLMUX_I2C_8_BIT_WRITE;
	mlm->txbuf.addr = msg->addr;
	mlm->txbuf.reg = msg->buf[0];
	mlm->txbuf.length = msg->len - 1;
	memcpy(mlm->txbuf.data, msg->buf + 1, mlm->txbuf.length);
	mutex_lock(&mlm->lock);
	mlm->txbuf.seq = mlm->seq++;
	reinit_completion(&mlm->xfer);
	mutex_unlock(&mlm->lock);

	ret = ml_mux_send_msg(mlm->client.ch, sizeof(mlm->txbuf), &mlm->txbuf);
	if (ret) {
		dev_err(mlm->dev, "Failed to mlmux send msg for write\n");
		return ret;
	}

	ret = wait_for_completion_timeout(&mlm->xfer, HZ);
	if (!ret)
		return -ETIMEDOUT;

	if (mlm->rxbuf.length < 0) {
		dev_err(mlm->dev, "Remote returned: %d\n", mlm->rxbuf.length);
		return -EINVAL;
	}

	return 0;
}

static int mlmux_i2c_rw(struct mlmux_i2c_dev *mlm, struct i2c_msg *msg_w,
			struct i2c_msg *msg_r)
{
	int ret;

	/* The pair of write/read must be for same slave address */
	if (msg_r->addr != msg_w->addr) {
		dev_err(mlm->dev, "read %d differs from write %d addresses\n",
			msg_r->addr, msg_w->addr);
		return -EINVAL;
	}

	/* The write length is assumed to be the register to read.
	 * Thus, it can only be of size 8- or 16-bits.
	 */
	if (msg_w->len > sizeof(mlm->txbuf.reg)) {
		dev_err(mlm->dev, "register length %d too large\n", msg_w->len);
		return -EINVAL;
	}

	/* Check the supported read size */
	if (msg_r->len > MLMUX_I2C_MAX_DATA_LENGTH) {
		dev_err(mlm->dev, "read length too long: %d\n", msg_r->len);
		return -EINVAL;
	}

	/* Use the write length to determine register addressing size */
	switch (msg_w->len) {
	case 1:
		mlm->txbuf.type = MLMUX_I2C_8_BIT_READ;
		mlm->txbuf.reg = msg_w->buf[0];
		break;
	case 2:
		mlm->txbuf.type = MLMUX_I2C_16_BIT_READ;
		mlm->txbuf.reg = msg_w->buf[0] << 8 | msg_w->buf[1];
		break;
	default:
		return -EMSGSIZE;
	}

	mlm->txbuf.addr = msg_r->addr;
	mlm->txbuf.length = msg_r->len;
	mutex_lock(&mlm->lock);
	mlm->txbuf.seq = mlm->seq++;
	reinit_completion(&mlm->xfer);
	mutex_unlock(&mlm->lock);

	ret = ml_mux_send_msg(mlm->client.ch, sizeof(mlm->txbuf), &mlm->txbuf);
	if (ret) {
		dev_err(mlm->dev, "Failed to mlmux send msg for read\n");
		return ret;
	}

	ret = wait_for_completion_timeout(&mlm->xfer, HZ);
	if (!ret)
		return -ETIMEDOUT;

	if (mlm->rxbuf.length < 0) {
		dev_err(mlm->dev, "Remote returned: %d\n", mlm->rxbuf.length);
		return -EINVAL;
	}

	if (mlm->rxbuf.length != mlm->txbuf.length) {
		dev_err(mlm->dev, "Unexpected bytes %d; expected: %d\n",
			mlm->rxbuf.length, mlm->txbuf.length);
		return -EINVAL;
	}

	memcpy(msg_r->buf, &mlm->rxbuf.data, msg_r->len);

	return 0;
}

/* Initial implementation is restricted to register based writes and require
 * bundling a typical i2c read into a single request over the channel. An i2c
 * read through the kernel is broken up into a write of the register address
 * followed by a read request. We'll assume streaming reading is not supported
 * and this will be used for register configuration purposes.
 */
static int
mlmux_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct mlmux_i2c_dev *mlm = i2c_get_adapdata(adap);
	int idx = 0;
	int ret;

	while (idx < num) {
		if (!msgs[idx].len) {
			ret = -EINVAL;
			goto out;
		}

		/* Reads must be accompanied by a write first for a register */
		if (msgs[idx].flags & I2C_M_RD) {
			dev_err(mlm->dev, "read requires a write packet\n");
			ret = -EINVAL;
			goto out;
		}

		/* A write followed by a read are combined into a single read */
		if (idx + 1 < num && (msgs[idx + 1].flags & I2C_M_RD)) {
			if (!msgs[idx + 1].len) {
				ret = -EINVAL;
				goto out;
			}
			ret = mlmux_i2c_rw(mlm, &msgs[idx], &msgs[idx + 1]);
			idx += 2;
		} else {
			ret = mlmux_i2c_write(mlm, &msgs[idx]);
			idx++;
		}

		if (ret)
			goto out;
	}

	ret = num;
out:
	return ret;
}

static u32 mlmux_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm mlmux_i2c_algo = {
	.master_xfer = mlmux_i2c_xfer,
	.functionality = mlmux_i2c_func,
};

static int mlmux_i2c_setup_adapter(struct mlmux_i2c_dev *mlm)
{
	int ret;

	mutex_lock(&mlm->adap_lock);
	if (mlm->adap_en) {
		dev_info(mlm->dev, "I2C Adapter getting re-registered\n");
		ret = 0;
		goto unlock;
	}

	i2c_set_adapdata(&mlm->adap, mlm);
	mlm->adap.algo = &mlmux_i2c_algo;
	mlm->adap.dev.parent = mlm->dev;
	mlm->adap.dev.of_node = mlm->dev->of_node;
	strlcpy(mlm->adap.name, "MLMUX I2C adapter", sizeof(mlm->adap.name));

	ret = i2c_add_adapter(&mlm->adap);
	if (ret)
		goto unlock;

	mlm->adap_en = true;

unlock:
	mutex_unlock(&mlm->adap_lock);
	return ret;
}

static void mlmux_i2c_destroy_adapter(struct mlmux_i2c_dev *mlm)
{
	mutex_lock(&mlm->adap_lock);
	if (!mlm->adap_en)
		goto unlock;

	i2c_del_adapter(&mlm->adap);
	mlm->adap_en = false;

unlock:
	mutex_unlock(&mlm->adap_lock);
}

static void mlmux_i2c_recv(struct ml_mux_client *cli, uint32_t len, void *msg)
{
	struct mlmux_i2c_dev *mlm;
	struct mlmux_i2c_msg *resp = (struct mlmux_i2c_msg *)msg;

	mlm = container_of(cli, struct mlmux_i2c_dev, client);

	if (len != sizeof(*resp)) {
		dev_err(mlm->dev, "Unexpected length %d versus %zu\n",
			len, sizeof(*resp));
		return;
	}

	if (resp->seq == mlm->rxbuf.seq)
		dev_warn(mlm->dev, "Duplicate sequence: %d\n", resp->seq);

	mutex_lock(&mlm->lock);
	if (resp->seq != mlm->txbuf.seq) {
		dev_err(mlm->dev, "Unexpected sequence %d; expected %d\n",
			resp->seq, mlm->txbuf.seq);
		goto unlock;
	}

	memcpy(&mlm->rxbuf, msg, len);

	complete(&mlm->xfer);

unlock:
	mutex_unlock(&mlm->lock);
}

static void mlmux_i2c_notify_work(struct work_struct *work)
{
	struct mlmux_i2c_notify_work *nw;
	struct mlmux_i2c_dev *mlm;
	int ret;

	nw = container_of(work, struct mlmux_i2c_notify_work, notify_work);
	mlm = nw->mlm;

	if (nw->is_open) {
		ret = mlmux_i2c_setup_adapter(mlm);
		if (ret)
			dev_err(mlm->dev, "Failed to add i2c adapter!\n");
		goto free_work;
	}

free_work:
	kfree(nw);
}

static void mlmux_queue_notify_work(struct mlmux_i2c_dev *mlm, bool open)
{
	struct mlmux_i2c_notify_work *nw;

	nw = kzalloc(sizeof(*nw), GFP_KERNEL);
	if (!nw)
		return;

	nw->mlm = mlm;
	nw->is_open = open;
	INIT_WORK(&nw->notify_work, mlmux_i2c_notify_work);

	queue_work(mlm->notify_q, &nw->notify_work);
}


static void mlmux_i2c_open(struct ml_mux_client *cli, bool is_open)
{
	struct mlmux_i2c_dev *mlm;

	mlm = container_of(cli, struct mlmux_i2c_dev, client);
	mlmux_queue_notify_work(mlm, is_open);
}

static int mlmux_i2c_parse_dt(struct mlmux_i2c_dev *mlm)
{
	struct device_node *np = mlm->dev->of_node;

	if (!np)
		return -ENODEV;

	if (of_property_read_string(np, "ml,chan-name", &mlm->chan_name)) {
		dev_err(mlm->dev, "ml,chan-name undefined!\n");
		return -EINVAL;
	}

	return 0;
}

static int mlmux_i2c_probe(struct platform_device *pdev)
{
	struct mlmux_i2c_dev *mlm;
	int ret;

	mlm = devm_kzalloc(&pdev->dev, sizeof(*mlm), GFP_KERNEL);
	if (!mlm)
		return -ENOMEM;

	/* Initialize sequence number to invalid */
	mlm->rxbuf.seq = 0xFF;

	mlm->dev = &pdev->dev;
	platform_set_drvdata(pdev, mlm);

	ret = mlmux_i2c_parse_dt(mlm);
	if (ret)
		return ret;

	init_completion(&mlm->xfer);
	mutex_init(&mlm->lock);
	mutex_init(&mlm->adap_lock);

	mlm->client.dev = mlm->dev;
	mlm->client.notify_open = mlmux_i2c_open;
	mlm->client.receive_cb = mlmux_i2c_recv;

	mlm->notify_q = alloc_workqueue(mlm->chan_name, WQ_UNBOUND, 1);
	if (!mlm->notify_q) {
		dev_info(mlm->dev, "Failed to allocate workqueue\n");
		return -ENOMEM;
	}

	ret = ml_mux_request_channel(&mlm->client, (char *)mlm->chan_name);
	if (ret < 0)
		goto mutex_destroy;

	/* Remote already setup the channel, can add our adapter */
	if (ret == ML_MUX_CH_REQ_OPENED)
		mlmux_queue_notify_work(mlm, true);
	else
		dev_info(mlm->dev, "adapter defer until channel open\n");

	return 0;

mutex_destroy:
	mutex_destroy(&mlm->adap_lock);
	mutex_destroy(&mlm->lock);

	return ret;
}

static int mlmux_i2c_remove(struct platform_device *pdev)
{
	struct mlmux_i2c_dev *mlm = platform_get_drvdata(pdev);

	mlmux_i2c_destroy_adapter(mlm);
	ml_mux_release_channel(mlm->client.ch);
	destroy_workqueue(mlm->notify_q);
	mutex_destroy(&mlm->adap_lock);
	mutex_destroy(&mlm->lock);

	return 0;
}

static const struct of_device_id mlmux_i2c_dt_match[] = {
	{ .compatible = "ml,i2c-mlmux" },
	{}
};
MODULE_DEVICE_TABLE(of, mlmux_i2c_dt_match);

static struct platform_driver mlmux_i2c_driver = {
	.driver = {
		.name = "i2c_mlmux",
		.of_match_table = mlmux_i2c_dt_match,
	},
	.probe  = mlmux_i2c_probe,
	.remove = mlmux_i2c_remove,
};
module_platform_driver(mlmux_i2c_driver);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c_mlmux");
MODULE_AUTHOR("Magic Leap, Inc.");
