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

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/firmware.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/ml-mux-ctrl.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include "nrf52_i2c.h"
#include "nrf52_fwupdate.h"

#define NRF_IRQ_GPIO_BL_TO_APP_SETTLE_TIME_MS 300

enum {
	NRF52_I2C_DBG_RST_REL = 0,
	NRF52_I2C_DBG_RST_HOLD = 1,
	NRF52_I2C_DBG_RST_TOGGLE = 2
};

static int charge_only;

static void nrf52_send_data(struct ml_mux_ctrl *ctrl, uint32_t len, void *data)
{
	struct nrf52_data *nrf52 = to_nrf_data(ctrl);
	struct i2c_msg msg;
	int ret = 0;

	msg.addr = nrf52->client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	ret = i2c_transfer(nrf52->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(nrf52->dev, "%s: i2c send fail (%d)\n", __func__, ret);
	}
}

static irqreturn_t nrf52_i2c_irq_handler(int irq, void *data)
{
	struct i2c_msg msg;
	struct nrf52_data *nrf52 = data;
	uint16_t pkt_size = 0;
	int ret = 0;

	msg.addr = nrf52->client->addr;
	msg.flags = I2C_M_RD;
	msg.len = 2;
	msg.buf = (u8 *)&pkt_size;
	ret = i2c_transfer(nrf52->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(nrf52->dev, "i2c recv fail (%d)\n", ret);
		goto exit;
	}
	if (pkt_size == 0 || pkt_size > nrf52->mlmux.max_rx_len) {
		dev_err(nrf52->dev, "i2c recv invalid size = %d\n", pkt_size);
		goto exit;
	}

	msg.addr = nrf52->client->addr;
	msg.flags = I2C_M_RD;
	msg.len = pkt_size + 2;
	msg.buf = nrf52->rx_buf;
	ret = i2c_transfer(nrf52->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(nrf52->dev, "%s: i2c recv fail (%d)\n", __func__, ret);
		goto exit;
	}

	ml_mux_msg_receive(&nrf52->mlmux, 2 + pkt_size, nrf52->rx_buf);

exit:
	return IRQ_HANDLED;
}

static int nrf52_init_irq(struct i2c_client *client, struct nrf52_data *nrf52)
{
	int ret = 0;

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				nrf52_i2c_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"nrf52_i2c_irq", nrf52);
		if (ret)
			dev_err(&client->dev, "Failed getting irq=%d err=%d\n",
				client->irq, ret);
	} else {
		dev_err(&client->dev, "device node \"interrupts\" required\n");
		ret = -ENODEV;
	}

	return ret;
}
static int nrf52_parse_dt(struct nrf52_data *nrf52)
{
	struct device *dev = nrf52->dev;
	struct device_node *np = dev->of_node;
	struct ml_mux_ctrl *mlmux = &nrf52->mlmux;
	uint32_t value;
	int ret = 0;

	if (!np) {
		dev_err(dev, "no device data\n");
		ret = -ENODEV;
		goto exit;
	}

	ret = of_property_read_u32(np, "queue-len", &value);
	if (ret) {
		dev_warn(dev, "Unable to read queue-len.\n");
		ret = -ENODEV;
		goto exit;
	}
	mlmux->queue_len = value;

	ret = of_property_read_u32(np, "max-write-len", &value);
	if (ret) {
		dev_warn(dev, "Unable to read max_tx_len.\n");
		ret = -ENODEV;
		goto exit;
	}
	mlmux->max_tx_len = value;

	ret = of_property_read_u32(np, "max-receive-len", &value);
	if (ret) {
		dev_warn(dev, "Unable to read max_rx_len.\n");
		ret = -ENODEV;
		goto exit;
	}
	mlmux->max_rx_len = value;

	mlmux->is_highpri = of_property_read_bool(np, "tx-highpri");
	if (mlmux->is_highpri)
		dev_info(dev, "the workqueue is set to high priority\n");

	ret = of_property_read_string(np, "bl_fw_file", &nrf52->bl_fw_file);
	if (ret)
		dev_warn(dev, "Unable to read bl_fw_file.\n");

	ret = of_property_read_string(np, "app_fw_file", &nrf52->app_fw_file);
	if (ret)
		dev_warn(dev, "Unable to read app_fw_file.\n");

	nrf52->reset_gpio = of_get_named_gpio(np, "nordic-reset", 0);
	if (!gpio_is_valid(nrf52->reset_gpio)) {
		dev_err(dev, "Invalid reset GPIO pin.\n");
		ret = -ENODEV;
		goto exit;
	}

	ret = devm_gpio_request(dev, nrf52->reset_gpio, "reset");
	if (ret) {
		dev_err(dev, "Reset GPIO request failed: %d\n", ret);
		ret = -EINVAL;
		goto exit;
	}

	nrf52->nvidia_ready = of_get_named_gpio(np, "nvidia-ready", 0);
	if (!gpio_is_valid(nrf52->nvidia_ready)) {
		dev_err(dev, "Invalid nvidia_ready GPIO pin.\n");
		ret = -ENODEV;
		goto exit;
	}

	ret = devm_gpio_request(dev, nrf52->nvidia_ready, "nvidia_ready");
	if (ret) {
		dev_err(dev, "Nvidia_ready GPIO request failed: %d\n", ret);
		ret = -EINVAL;
		goto exit;
	}
	strlcpy(mlmux->name, dev->of_node->name, sizeof(mlmux->name));

exit:
	return ret;
}

static int nrf52_init_ml_mux_ctrl(struct device *dev,
						struct ml_mux_ctrl *mlmux)
{
	mlmux->tx_data = nrf52_send_data;
	mlmux->dev = dev;

	return 0;
}

void nrf52_reset(struct nrf52_data *nrf52)
{
	gpio_set_value(nrf52->reset_gpio, 1);

	msleep(200);

	gpio_set_value(nrf52->reset_gpio, 0);
}

static void nrf52_pulse_gpio(int gpio, int milliseconds)
{
	int i;
	int gpio_value = gpio_get_value(gpio);
	int initial_gpio_val = gpio_value;

	for (i = 0; i < (milliseconds / 10); i++) {
		gpio_value = (gpio_value == 0 ? 1 : 0);
		gpio_set_value(gpio, gpio_value);
		msleep(10);
	}

	gpio_set_value(gpio, initial_gpio_val);
}

static void nrf52_bl_fw_callback(const struct firmware *fw, void *context)
{
	struct nrf52_data *nrf52 = (struct nrf52_data *) context;

	dev_info(nrf52->dev, "Loading Nrf Bootloader Firmware ...\n");
	mutex_lock(&nrf52->mutex_lock);

	/* Reset chip to bl mode */
	nrf52_reset(nrf52);
	nrf52_pulse_gpio(nrf52->nvidia_ready, 1000);

	/* Workaround for slow app startup on older version. */
	msleep(2000);

	if (nrf52_load_fw(fw, nrf52) != 0) {
		dev_info(nrf52->dev, "Unable to load BL Firmware\n");
		nrf52_reset(nrf52);
	}

	release_firmware(fw);
	nrf52->pending_fw_updates--;

	mutex_unlock(&nrf52->mutex_lock);
}

static void nrf52_app_fw_callback(const struct firmware *fw, void *context)
{
	int ret;
	struct nrf52_data *nrf52 = (struct nrf52_data *) context;

	/* Wait for BL updates to complete */
	while (nrf52->pending_fw_updates > 1)
		msleep(200);

	dev_info(nrf52->dev, "Loading Nrf Application Firmware ...\n");
	mutex_lock(&nrf52->mutex_lock);

	/* Reset chip to bl mode */
	nrf52_reset(nrf52);
	nrf52_pulse_gpio(nrf52->nvidia_ready, 1000);

	gpio_set_value(nrf52->nvidia_ready, 1);

	if (nrf52_load_fw(fw, nrf52) != 0) {
		dev_info(nrf52->dev, "Unable to load App Firmware\n");
		nrf52_reset(nrf52);
	}
	release_firmware(fw);
	nrf52->pending_fw_updates--;

	mutex_unlock(&nrf52->mutex_lock);

	/* If all fw updates are complete. Enable the IRQ.*/
	if (nrf52->pending_fw_updates == 0) {
		/* nrf bl pulls the irq gpio low when jumping to its app
		 * wait until nrf has booted into its app to avoid unwanted
		 * irq triggers
		 */
		msleep(NRF_IRQ_GPIO_BL_TO_APP_SETTLE_TIME_MS);
		ret = nrf52_init_irq(nrf52->client, nrf52);
		if (ret) {
			dev_err(nrf52->dev, "Failed nrf52_init_irq\n");
			ml_mux_ctrl_unregister(&nrf52->mlmux);
		}
	}
}

#ifdef CONFIG_DEBUG_FS
static int nrf52_dbg_reset_get(void *data, u64 *val)
{
	struct nrf52_data *nrf52 = (struct nrf52_data *)data;

	*val = gpio_get_value(nrf52->reset_gpio);

	return 0;
}

static int nrf52_dbg_reset_set(void *data, u64 val)
{
	struct nrf52_data *nrf52 = (struct nrf52_data *)data;
	struct device *dev;
	int gpio_high;

	dev = nrf52->dev;
	if (val < NRF52_I2C_DBG_RST_REL || val > NRF52_I2C_DBG_RST_TOGGLE) {
		dev_err(dev, "%s: invalid value '%llu'", __func__, val);
		return -EINVAL;
	}

	mutex_lock(&nrf52->mutex_lock);
	if (nrf52->pending_fw_updates == 0) {
		gpio_high = gpio_get_value(nrf52->reset_gpio);
		if (val == NRF52_I2C_DBG_RST_REL && gpio_high) {
			gpio_set_value(nrf52->reset_gpio, 0);
			msleep(NRF_IRQ_GPIO_BL_TO_APP_SETTLE_TIME_MS);
			enable_irq(nrf52->client->irq);
		} else if (val == NRF52_I2C_DBG_RST_HOLD && !gpio_high) {
			disable_irq(nrf52->client->irq);
			gpio_set_value(nrf52->reset_gpio, 1);
		} else if (val == NRF52_I2C_DBG_RST_TOGGLE) {
			if (!gpio_high)
				disable_irq(nrf52->client->irq);
			nrf52_reset(nrf52);
			msleep(NRF_IRQ_GPIO_BL_TO_APP_SETTLE_TIME_MS);
			enable_irq(nrf52->client->irq);
		}
	} else {
		dev_err(dev, "%s: nrf52 busy", __func__);
		mutex_unlock(&nrf52->mutex_lock);
		return -EBUSY;
	}
	mutex_unlock(&nrf52->mutex_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(nrf52_dbg_reset_fops, nrf52_dbg_reset_get,
			nrf52_dbg_reset_set, "%llu\n");

static struct dentry *nrf52_init_debugfs(struct nrf52_data *nrf52,
					       const char *name)
{
	struct dentry *debugfs;
	int ret;

	debugfs = debugfs_create_dir(name, NULL);
	if (IS_ERR_OR_NULL(debugfs)) {
		ret = PTR_ERR(debugfs);
		dev_warn(nrf52->dev, "Failed to create debugfs dir: %d\n", ret);
		return NULL;
	}
	debugfs_create_file("nrf52_reset", S_IRUGO | S_IWUSR, debugfs,
			    nrf52, &nrf52_dbg_reset_fops);

	return debugfs;
}
#else
static inline struct dentry *nrf52_init_debugfs(struct nrf52_data *nrf52,
					const char *name)
{
	return NULL;
}
#endif

/* Workaround to not allow nrf update if we're in charge only mode */
static int __init bootmode_setup(char *str)
{
	charge_only = !strncmp("charger", str, 7);
	return 1;
}
__setup("androidboot.mode=", bootmode_setup);

static void nrf52_deinit(struct i2c_client *client)
{
	struct nrf52_data *nrf52 = i2c_get_clientdata(client);

	if (nrf52->pending_fw_updates == 0)
		disable_irq(client->irq);
	ml_mux_ctrl_unregister(&nrf52->mlmux);
}

static int nrf52_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct nrf52_data *nrf52;
	struct device *dev = &client->dev;
	int ret = 0;

	dev_info(dev, "probing = %s\n", client->dev.of_node->name);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "FAILED to establish i2c functionality\n");
		return -ENXIO;
	}

	nrf52 = devm_kzalloc(dev, sizeof(*nrf52), GFP_KERNEL);
	if (!nrf52)
		return -ENOMEM;

	i2c_set_clientdata(client, nrf52);
	nrf52->client = client;
	nrf52->dev = dev;
	nrf52->pending_fw_updates = 0;
	nrf52->app_fw_file = NULL;
	nrf52->bl_fw_file = NULL;
	nrf52->reset_gpio = 0;
	mutex_init(&nrf52->mutex_lock);

	ret = nrf52_parse_dt(nrf52);
	if (ret)
		return ret;

	ret = nrf52_init_ml_mux_ctrl(dev, &nrf52->mlmux);
	if (ret) {
		dev_err(dev, "Unable to init ML_MUX controller\n");
		return ret;
	}

	nrf52->rx_buf = devm_kmalloc(dev, nrf52->mlmux.max_rx_len, GFP_KERNEL);
	if (!nrf52->rx_buf)
		return -ENOMEM;

	ret = ml_mux_ctrl_register(&nrf52->mlmux);
	if (ret)
		return ret;

	if (nrf52->bl_fw_file != NULL && !charge_only) {
		ret = request_firmware_nowait(THIS_MODULE, true,
			nrf52->bl_fw_file, dev, GFP_KERNEL,
			nrf52, nrf52_bl_fw_callback);
		if (ret)
			dev_err(dev, "failed to request bl firmware.\n");
		else
			nrf52->pending_fw_updates++;
	}

	if (nrf52->app_fw_file != NULL && !charge_only) {
		ret = request_firmware_nowait(THIS_MODULE, true,
			nrf52->app_fw_file, dev, GFP_KERNEL,
			nrf52, nrf52_app_fw_callback);
		if (ret)
			dev_err(dev, "failed to request app firmware.\n");
		else
			nrf52->pending_fw_updates++;
	}

	if (nrf52->pending_fw_updates == 0) {
		gpio_set_value(nrf52->nvidia_ready, 1);
		ret = nrf52_init_irq(client, nrf52);
		if (ret) {
			dev_err(dev, "Failed nrf52_init_irq.\n");
			goto err_1;
		}
	}

	nrf52->debugfs = nrf52_init_debugfs(nrf52, nrf52->client->name);

	return 0;

err_1:
	ml_mux_ctrl_unregister(&nrf52->mlmux);
	return ret;
}

static void nrf52_shutdown_i2c(struct i2c_client *client)
{
	struct nrf52_data *nrf52 = i2c_get_clientdata(client);

	gpio_set_value(nrf52->nvidia_ready, 0);
	nrf52_deinit(client);
}

static int nrf52_remove_i2c(struct i2c_client *client)
{
	struct nrf52_data *nrf52 = i2c_get_clientdata(client);

	debugfs_remove_recursive(nrf52->debugfs);
	nrf52_deinit(client);
	return 0;
}

static const struct i2c_device_id nrf52_id[] = {
	{NRF52_NAME, 0},
	{ }
};

static const struct of_device_id nrf52_match_table[] = {
	{ .compatible = "nordic,nrf52_ml_mux", },
};

static struct i2c_driver nrf52_driver = {
	.driver = {
		.name = NRF52_NAME,
		.of_match_table = nrf52_match_table,
	},
	.shutdown = nrf52_shutdown_i2c,
	.probe = nrf52_probe,
	.remove = nrf52_remove_i2c,
	.id_table = nrf52_id,
};

module_i2c_driver(nrf52_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("ML MUX controller driver over I2c");
MODULE_LICENSE("GPL v2");

