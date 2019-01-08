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

#include <linux/gpio.h>
#include <linux/ml-mux-client.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#define MLMUX_GPIO_NAME_LEN 25

enum mlmux_gpio_op_type {
	MLMUX_GPIO_READ = 0,
	MLMUX_GPIO_WRITE,
	MLMUX_GPIO_TOGGLE,
	MLMUX_GPIO_VALIDATE,
	MLMUX_GPIO_TAKE_OWNERSHIP,
	MLMUX_GPIO_FREE_OWNERSHIP,
	MLMUX_GPIO_CONFIG_DIRECTION,
};

enum mlmux_gpio_direction {
	MLMUX_GPIO_DIR_OUT = 0,
	MLMUX_GPIO_DIR_IN = 1,
};

struct mlmux_gpio_op {
	uint8_t seq;
	uint8_t op;
	char gpio_name[MLMUX_GPIO_NAME_LEN];
	uint8_t value;
	int status;
} __packed;

struct mlmux_gpio_def {
	const char *name;
	uint32_t gpio;
	uint8_t direction;
	uint8_t value;
	bool owned;
};

struct mlmux_gpio_info {
	struct device *dev;

	struct gpio_chip gc;
	bool ctrl_en;
	int ngpio;
	struct mlmux_gpio_def *gpios;

	struct mutex ctrl_lock;

	struct mutex xfer_lock;
	struct mutex seq_lock;
	struct completion xfer;

	uint8_t wait_seq;
	bool waiting;
	struct mlmux_gpio_op resp;

	struct ml_mux_client client;
	const char *chan_name;

	atomic_t seq;
};

/* Only allow a single GPIO Request at a time, caller must hold the
 * xfer_lock in order to use the single response buffer contents.
 */
static int
mlmux_gpio_send_request(struct mlmux_gpio_info *mlgi, struct mlmux_gpio_op *req)
{
	int ret;

	/* Ensure the seq and reinit are 'atomic' */
	mutex_lock(&mlgi->seq_lock);
	mlgi->wait_seq = req->seq;
	mlgi->waiting = true;
	reinit_completion(&mlgi->xfer);
	mutex_unlock(&mlgi->seq_lock);

	dev_dbg(mlgi->dev, "REQ: %s op: %d seq: %d level: %d\n",
		req->gpio_name, req->op, req->seq, req->value);

	ret = ml_mux_send_msg(mlgi->client.ch, sizeof(*req), req);
	if (ret) {
		dev_err(mlgi->dev, "%s failed send for op: %d\n",
			req->gpio_name, req->op);
		goto exit;
	}

	ret = wait_for_completion_timeout(&mlgi->xfer, HZ);
	if (!ret) {
		dev_err(mlgi->dev, "%s timeout waiting for op: %d\n",
			req->gpio_name, req->op);
		ret = -ETIMEDOUT;
		goto exit;
	}

	dev_dbg(mlgi->dev, "RESP: %s level: %d status: %d\n",
		req->gpio_name, mlgi->resp.value, mlgi->resp.status);

	if (mlgi->resp.status < 0) {
		dev_err(mlgi->dev, "%s op %d failed with: %d\n",
			req->gpio_name, req->op, mlgi->resp.status);
		ret = -EINVAL;
		goto exit;
	}

	ret = 0;

exit:
	mlgi->waiting = false;

	return ret;
}

/* Helper utility function to perform common request population such as filling
 * int the operation type, atomically getting the unique sequence number, and
 * fetching the actual 'name' of the GPIO the remote expects
 */
static inline void mlmux_gpio_init_req(struct mlmux_gpio_info *mlgi,
				       struct mlmux_gpio_op *req,
				       unsigned offset, uint8_t op)
{
	req->value = 0;
	req->op = op;
	req->seq = (uint8_t)atomic_inc_return(&mlgi->seq) - 1;
	strlcpy(req->gpio_name, mlgi->gpios[offset].name, MLMUX_GPIO_NAME_LEN);
}

static int mlmux_gpio_take_ownership(struct mlmux_gpio_info *mlgi, int offset)
{
	struct mlmux_gpio_op req;
	int ret;

	mlmux_gpio_init_req(mlgi, &req, offset, MLMUX_GPIO_TAKE_OWNERSHIP);

	mutex_lock(&mlgi->xfer_lock);
	ret = mlmux_gpio_send_request(mlgi, &req);
	if (!ret)
		mlgi->gpios[offset].owned = true;
	mutex_unlock(&mlgi->xfer_lock);

	return ret;
}

static void mlmux_gpio_free_ownership(struct mlmux_gpio_info *mlgi, int offset)
{
	struct mlmux_gpio_op req;

	mlmux_gpio_init_req(mlgi, &req, offset, MLMUX_GPIO_FREE_OWNERSHIP);

	mutex_lock(&mlgi->xfer_lock);
	(void)mlmux_gpio_send_request(mlgi, &req);
	mlgi->gpios[offset].owned = false;
	mutex_unlock(&mlgi->xfer_lock);
}

static int mlmux_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct mlmux_gpio_info *mlgi = gpiochip_get_data(chip);
	int ret;

	/* Need to take ownership */
	ret = mlmux_gpio_take_ownership(mlgi, offset);
	if (ret)
		return ret;

	ret = pinctrl_request_gpio(offset);
	if (ret)
		mlmux_gpio_free_ownership(mlgi, offset);

	return ret;
}

static void mlmux_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct mlmux_gpio_info *mlgi = gpiochip_get_data(chip);

	mlmux_gpio_free_ownership(mlgi, offset);
	pinctrl_free_gpio(offset);
}

static void mlmux_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mlmux_gpio_info *mlgi = gpiochip_get_data(chip);
	struct mlmux_gpio_op req;
	int ret;

	mlmux_gpio_init_req(mlgi, &req, offset, MLMUX_GPIO_WRITE);
	req.value = value;

	mutex_lock(&mlgi->xfer_lock);
	ret = mlmux_gpio_send_request(mlgi, &req);
	if (ret)
		goto exit;

	mlgi->gpios[offset].value = value;

exit:
	mutex_unlock(&mlgi->xfer_lock);
}

static int mlmux_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct mlmux_gpio_info *mlgi = gpiochip_get_data(chip);
	struct mlmux_gpio_op req;
	int ret;

	mlmux_gpio_init_req(mlgi, &req, offset, MLMUX_GPIO_READ);

	mutex_lock(&mlgi->xfer_lock);
	ret = mlmux_gpio_send_request(mlgi, &req);
	if (ret)
		goto exit;

	ret = mlgi->resp.value;
exit:
	mutex_unlock(&mlgi->xfer_lock);

	return ret;
}

static int mlmux_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct mlmux_gpio_info *mlgi = gpiochip_get_data(chip);
	struct mlmux_gpio_op req;
	int ret;

	mlmux_gpio_init_req(mlgi, &req, offset, MLMUX_GPIO_CONFIG_DIRECTION);
	req.value = MLMUX_GPIO_DIR_IN;

	mutex_lock(&mlgi->xfer_lock);
	ret = mlmux_gpio_send_request(mlgi, &req);
	if (ret)
		goto exit;

exit:
	mutex_unlock(&mlgi->xfer_lock);

	return ret;
}

static int mlmux_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	struct mlmux_gpio_info *mlgi = gpiochip_get_data(chip);
	struct mlmux_gpio_op req;
	int ret;

	mlmux_gpio_init_req(mlgi, &req, offset, MLMUX_GPIO_CONFIG_DIRECTION);
	req.value = MLMUX_GPIO_DIR_OUT;

	mutex_lock(&mlgi->xfer_lock);
	ret = mlmux_gpio_send_request(mlgi, &req);
	if (ret)
		goto exit;

	mlmux_gpio_init_req(mlgi, &req, offset, MLMUX_GPIO_WRITE);
	req.value = value;

	ret = mlmux_gpio_send_request(mlgi, &req);
	if (ret)
		goto exit;

	mlgi->gpios[offset].value = value;

exit:
	mutex_unlock(&mlgi->xfer_lock);

	return ret;
}

static int mlmux_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct mlmux_gpio_info *mlgi = gpiochip_get_data(chip);

	return mlgi->gpios[offset].direction;
}

static void mlmux_gpio_recv(struct ml_mux_client *cli, uint32_t len, void *msg)
{
	struct mlmux_gpio_info *mlgi;
	struct mlmux_gpio_op *resp = (struct mlmux_gpio_op *)msg;

	mlgi = container_of(cli, struct mlmux_gpio_info, client);

	if (len != sizeof(*resp)) {
		dev_err(mlgi->dev, "Unexpected length %d versus %zu\n",
			len, sizeof(*resp));
		return;
	}

	mutex_lock(&mlgi->seq_lock);
	if (!mlgi->waiting) {
		dev_err(mlgi->dev, "Unexpected sequence %d; not waiting\n",
			resp->seq);
		goto unlock;
	}

	if (resp->seq != mlgi->wait_seq) {
		dev_err(mlgi->dev, "Unexpected sequence %d; expected %d\n",
			resp->seq, mlgi->wait_seq);
		goto unlock;
	}

	mlgi->waiting = false;
	memcpy(&mlgi->resp, msg, len);

	complete(&mlgi->xfer);

unlock:
	mutex_unlock(&mlgi->seq_lock);
}

static int mlmux_gpio_find_def(struct mlmux_gpio_info *mlgi, uint32_t gpio)
{
	int i;

	for (i = 0; i < mlgi->ngpio; i++)
		if (mlgi->gpios[i].gpio == gpio)
			return i;

	return -ENODEV;
}

static int mlmux_gpio_xlate(struct gpio_chip *chip,
			    const struct of_phandle_args *spec,
			    u32 *flags)
{
	struct mlmux_gpio_info *mlgi = gpiochip_get_data(chip);
	int index;

	if (WARN_ON(chip->of_gpio_n_cells < 2)) {
		dev_err(mlgi->dev, "num cells less than 2!\n");
		return -EINVAL;
	}

	index = mlmux_gpio_find_def(mlgi, spec->args[0]);
	if (index < 0) {
		dev_err(mlgi->dev, "no such GPIO %d\n", spec->args[0]);
		return -ENODEV;
	}

	if (flags)
		*flags = spec->args[1];

	return index;
}

static int mlmux_gpio_setup_controller(struct mlmux_gpio_info *mlgi)
{
	int ret;
	int i;

	/* If no GPIOs are enabled in DT, don't add controller */
	if (mlgi->ngpio == 0)
		return 0;

	mutex_lock(&mlgi->ctrl_lock);
	if (mlgi->ctrl_en) {
		dev_info(mlgi->dev, "GPIO controller getting re-registered!\n");
		ret = 0;
		goto take_ownership;
	}

	mlgi->gc.label			= "mlmux-gpio";
	mlgi->gc.request		= mlmux_gpio_request;
	mlgi->gc.free			= mlmux_gpio_free;
	mlgi->gc.direction_input	= mlmux_gpio_direction_input;
	mlgi->gc.get			= mlmux_gpio_get;
	mlgi->gc.direction_output	= mlmux_gpio_direction_output;
	mlgi->gc.set			= mlmux_gpio_set;
	mlgi->gc.get_direction		= mlmux_gpio_get_direction;
	mlgi->gc.base			= -1;
	mlgi->gc.ngpio			= mlgi->ngpio;
	mlgi->gc.parent			= mlgi->dev;
	mlgi->gc.of_node		= mlgi->dev->of_node;
	mlgi->gc.of_xlate		= mlmux_gpio_xlate;
	mlgi->gc.of_gpio_n_cells	= 2;

	ret = gpiochip_add_data(&mlgi->gc, mlgi);
	if (ret < 0)
		goto unlock;

	mlgi->ctrl_en = true;

take_ownership:
	/* If the remote reset, we may need to re-take ownership for
	 * any clients who had previously gpio_request'd a pin.
	 */
	for (i = 0; i < mlgi->ngpio; i++) {
		if (!mlgi->gpios[i].owned)
			continue;

		/* Try to take ownership and log failures... */
		if (mlmux_gpio_take_ownership(mlgi, i))
			dev_warn(mlgi->dev,
				"Failed to re-take ownership of: %s\n",
				mlgi->gpios[i].name);
	}
unlock:
	mutex_unlock(&mlgi->ctrl_lock);

	return ret;
}

static void mlmux_gpio_destroy_controller(struct mlmux_gpio_info *mlgi)
{
	mutex_lock(&mlgi->ctrl_lock);
	if (!mlgi->ctrl_en)
		goto unlock;

	gpiochip_remove(&mlgi->gc);
	mlgi->ctrl_en = false;

unlock:
	mutex_unlock(&mlgi->ctrl_lock);
}

static void mlmux_gpio_open(struct ml_mux_client *cli, bool is_open)
{
	struct mlmux_gpio_info *mlgi;
	int ret;

	mlgi = container_of(cli, struct mlmux_gpio_info, client);
	if (is_open) {
		ret = mlmux_gpio_setup_controller(mlgi);
		if (ret)
			dev_err(mlgi->dev, "Failed to add gpio controller!\n");

		return;
	}
}

static int mlmux_gpio_parse_dt(struct mlmux_gpio_info *mlgi)
{
	struct device_node *np = mlgi->dev->of_node;
	struct device_node *child;
	int ret;
	int count = 0;

	if (!np)
		return -ENODEV;

	if (of_property_read_string(np, "ml,chan-name", &mlgi->chan_name)) {
		dev_err(mlgi->dev, "ml,chan-name undefined!\n");
		return -EINVAL;
	}

	/* First need to count the children GPIOs */
	for_each_available_child_of_node(np, child)
		count++;

	mlgi->ngpio = count;
	mlgi->gpios = devm_kzalloc(mlgi->dev,
				   count * sizeof(struct mlmux_gpio_def),
				   GFP_KERNEL);
	if (!mlgi->gpios)
		return -ENOMEM;

	count = 0;
	for_each_available_child_of_node(np, child) {
		struct mlmux_gpio_def *def = &mlgi->gpios[count];
		uint8_t direction;
		uint32_t gpio;

		ret = of_property_read_string(child, "ml,gpio-name",
					      &def->name);
		if (ret || strlen(def->name) > MLMUX_GPIO_NAME_LEN) {
			dev_err(mlgi->dev, "Bad gpio-name: %s\n", def->name);
			return -EINVAL;
		}

		ret = of_property_read_u32(child, "ml,gpio-num", &gpio);
		if (ret) {
			dev_err(mlgi->dev, "gpio-num not found\n");
			return -EINVAL;
		}
		def->gpio = gpio;

		ret = of_property_read_u8(child, "ml,dir", &direction);
		if (ret) {
			dev_err(mlgi->dev, "Direction not found\n");
			return -EINVAL;
		}

		def->direction = direction ? GPIOF_DIR_OUT : GPIOF_DIR_IN;

		count++;

		dev_dbg(mlgi->dev, "added: %s direction: %d\n",
			def->name, def->direction);
	}

	return 0;
}

static int mlmux_gpio_probe(struct platform_device *pdev)
{
	struct mlmux_gpio_info *mlgi;
	int ret;

	mlgi = devm_kzalloc(&pdev->dev, sizeof(*mlgi), GFP_KERNEL);
	if (!mlgi)
		return -ENODEV;

	mlgi->dev = &pdev->dev;
	mutex_init(&mlgi->ctrl_lock);
	mutex_init(&mlgi->xfer_lock);
	mutex_init(&mlgi->seq_lock);
	init_completion(&mlgi->xfer);
	platform_set_drvdata(pdev, mlgi);
	atomic_set(&mlgi->seq, 0);

	ret = mlmux_gpio_parse_dt(mlgi);
	if (ret)
		goto destroy_mutex;

	mlgi->client.dev = mlgi->dev;
	mlgi->client.notify_open = mlmux_gpio_open;
	mlgi->client.receive_cb = mlmux_gpio_recv;

	ret = ml_mux_request_channel(&mlgi->client, (char *)mlgi->chan_name);
	if (ret < 0)
		goto destroy_mutex;

	/* Remote already setup the channel, can add our adapter */
	if (ret == ML_MUX_CH_REQ_OPENED)
		mlmux_gpio_open(&mlgi->client, true);
	else
		dev_info(mlgi->dev, "defer until channel open\n");

	return 0;

destroy_mutex:
	mutex_destroy(&mlgi->seq_lock);
	mutex_destroy(&mlgi->xfer_lock);
	mutex_destroy(&mlgi->ctrl_lock);

	return ret;
}

static int mlmux_gpio_remove(struct platform_device *pdev)
{
	struct mlmux_gpio_info *mlgi = platform_get_drvdata(pdev);

	mlmux_gpio_destroy_controller(mlgi);
	ml_mux_release_channel(mlgi->client.ch);
	mutex_destroy(&mlgi->seq_lock);
	mutex_destroy(&mlgi->xfer_lock);
	mutex_destroy(&mlgi->ctrl_lock);

	return 0;
}

static const struct of_device_id mlmux_gpio_of_match[] = {
	{ .compatible = "ml,mlmux-gpio" },
	{ },
};

static struct platform_driver mlmux_gpio_driver = {
	.driver = {
		.name = "mlmux-gpio",
		.of_match_table = mlmux_gpio_of_match,
	},
	.probe = mlmux_gpio_probe,
	.remove = mlmux_gpio_remove,
};
module_platform_driver(mlmux_gpio_driver);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c_mlmux");
MODULE_AUTHOR("Magic Leap, Inc.");
