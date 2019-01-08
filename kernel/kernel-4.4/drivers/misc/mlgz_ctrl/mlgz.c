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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include "mlgz_char_device.h"
#include "mlgz.h"
#include "mlgz_opcode.h"

#define MLGZ_NAME "mlgz_control"

/*mux client callbacks*/
static void mlgz_mux_receive_callback(struct ml_mux_client *client,
			uint32_t len, void *msg)
{
	struct mlgz_dev_data *ml_dev =
		container_of(client, struct mlgz_dev_data, client);
	/*queue the received data, it will be read when*/
	/*an application reads from /dev/mlgz_ctrl*/
	mlgz_char_device_queue_received_data(msg, len, ml_dev);
}

static void mlgz_mux_notify_open(struct ml_mux_client *client,
			bool is_open)
{
	struct mlgz_dev_data *ml_dev =
		container_of(client, struct mlgz_dev_data, client);

	mutex_lock(&ml_dev->mtx);

	/* If the device is already open, or it is already closed, just exit */
	if (ml_dev->open == is_open)
		goto notify_open_cleanup;
	/* TODO: How do we handle errors here? */
	if (is_open) {
		mlgz_char_device_add(ml_dev);
		ml_dev->open = true;
	} else {
		mlgz_char_device_remove(ml_dev);
		ml_dev->open = false;
	}
notify_open_cleanup:
	mutex_unlock(&ml_dev->mtx);

}

ssize_t mlgz_mux_send_data(struct mlgz_dev_data *dev_data,
						const char *data, size_t len)
{
	if (!dev_data->open) {
		pr_err("mux not open\n");
		return -1;
	}
	ml_mux_send_msg(dev_data->client.ch, len, (void *)data);
	return len;
}

static int mlgz_probe(struct platform_device *pdev)
{
	int ret;
	struct mlgz_dev_data *ml_dev;

	dev_info(&pdev->dev, "mlgz_probe()\n");
	ml_dev = devm_kzalloc(&pdev->dev, sizeof(struct mlgz_dev_data),
							GFP_KERNEL);

	if (!ml_dev)
		return -ENOMEM;

	/* Set a pointer back to mlgz_dev_data struct */
	platform_set_drvdata(pdev, ml_dev);

	/* Initialize the mutex for this device */
	mutex_init(&ml_dev->mtx);

	/* At this point the only valid response from ml_mux_request_channel
	 * is ML_MUX_CH_REQ_OPENED, so we will initialize the device
	 */
	ret = mlgz_char_device_init();
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to initialize device.");
		goto err_char_dev;
	}

	/* Set call backs for the device */
	ml_dev->client.notify_open = mlgz_mux_notify_open;
	ml_dev->client.receive_cb = mlgz_mux_receive_callback;
	ml_dev->client.dev = &pdev->dev;

	ret = ml_mux_request_channel(&ml_dev->client, MLGZ_NAME);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to open channel.\n");
		goto err_mux_ch;
	}

	if (ret == ML_MUX_CH_REQ_SUCCESS) {
		/* Wait for notify_open */
		return 0;
	}

	return 0;

err_mux_ch:
	mlgz_char_device_exit();
err_char_dev:
	mutex_destroy(&ml_dev->mtx);

	return ret;

}

static int mlgz_remove(struct platform_device *pdev)
{
	struct mlgz_dev_data *ml_dev =
			platform_get_drvdata(pdev);

	mlgz_char_device_exit();
	mutex_destroy(&ml_dev->mtx);

	return 0;
}

static const struct of_device_id mlgz_match_table[] = {
	{ .compatible = "ml,mlgz_ctrl", },
	{ },
};

MODULE_DEVICE_TABLE(of, mlgz_match_table);

#ifdef CONFIG_PM_SLEEP
static int mlgz_suspend(struct device *dev)
{
	dev_dbg(dev, "suspend\n");

	return 0;
}

static int mlgz_resume(struct device *dev)
{
	dev_dbg(dev, "resume\n");

	return 0;
}

static const struct dev_pm_ops mlgz_pm_ops = {
	.suspend = mlgz_suspend,
	.resume = mlgz_resume
};
#define MLGZ_PM_OPS_PTR	(&mlgz_pm_ops)
#else
#define MLGZ_PM_OPS_PTR NULL
#endif

static struct platform_driver mlgz_driver = {
	.driver = {
		.name = MLGZ_NAME,
		.of_match_table = mlgz_match_table,
		.pm = MLGZ_PM_OPS_PTR,
	},
	.probe = mlgz_probe,
	.remove = mlgz_remove,
};

module_platform_driver(mlgz_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Magic Leap, LLC");
MODULE_DESCRIPTION("Custom Driver for ML Protocol over gazell");

