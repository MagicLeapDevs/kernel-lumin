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
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include "therm-mlmux.h"


#define MLMUX_THERM_SUSPEND_TO    (2000)
#define MLMUX_THERM_RESUME_TO     (2000)
#define MLMUX_THERM_CRITICAL_TEMP (120000)
#define MLMUX_THERM_POLLING_DELAY (10000)

static int mlmux_therm_get_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct mlmux_therm_zone *ml_tz = tzd->devdata;

	*temp = ml_tz->temp * 100;
	return 0;
}

static int mlmux_therm_get_trip_type(struct thermal_zone_device *tzd, int trip,
		enum thermal_trip_type *type)
{
	*type = THERMAL_TRIP_CRITICAL;
	return 0;
}

static int mlmux_therm_get_trip_temp(struct thermal_zone_device *tzd, int trip,
		int *temp)
{
	struct mlmux_therm_zone *ml_tz = tzd->devdata;

	*temp = ml_tz->critical_temp;
	return 0;
}

static int mlmux_therm_get_critical_temp(struct thermal_zone_device *tzd,
		int *temp)
{
	struct mlmux_therm_zone *ml_tz = tzd->devdata;

	*temp = ml_tz->critical_temp;
	return 0;
}

static struct thermal_zone_device_ops ml_tzd_ops = {
	.get_temp = mlmux_therm_get_temp,
	.get_trip_type = mlmux_therm_get_trip_type,
	.get_trip_temp = mlmux_therm_get_trip_temp,
	.get_crit_temp = mlmux_therm_get_critical_temp,
};

static void mlmux_therm_process_reg(struct work_struct *work)
{
	struct mlmux_therm_work *reg_w;
	struct mlmux_therm_dev_data *tdev;
	struct mlmux_therm_zone *ml_tz;
	struct mlmux_therm_tx_msg tx_msg;
	struct device_node *np;
	char prop_name[32];
	int ret;

	reg_w = container_of(work, struct mlmux_therm_work, work);
	tdev = reg_w->tdev;

	mutex_lock(&tdev->lock);
	ml_tz = kzalloc(sizeof(*ml_tz), GFP_KERNEL);
	if (!ml_tz)
		goto free;

	ml_tz->critical_temp = MLMUX_THERM_CRITICAL_TEMP;

	ml_tz->tz = thermal_zone_device_register((char *)reg_w->data,
			1, 0, ml_tz, &ml_tzd_ops, NULL, 0,
			MLMUX_THERM_POLLING_DELAY);
	if (IS_ERR(ml_tz->tz)) {
		dev_err(tdev->dev, "Reg therm_zone failed=%ld\n",
			PTR_ERR(ml_tz->tz));
		kfree(ml_tz);
		goto free;
	}
	ret = idr_alloc(&tdev->zones, ml_tz, 0, MLMUX_MAX_NUM_TZ, GFP_KERNEL);

	tx_msg.type = MLMUX_THERM_TX_REG_ACK;
	tx_msg.u.ack.id = ret;
	strncpy(tx_msg.u.ack.name, (char *)reg_w->data, MLMUX_TZ_NAME_SIZE);
	/* ack with success or failure */
	ml_mux_send_msg(tdev->client.ch, (uint32_t)MLMUX_THERM_TX_MSG_SIZE(ack),
					&tx_msg);
	if (ret < 0) {
		thermal_zone_device_unregister(ml_tz->tz);
		kfree(ml_tz);
		goto free;
	}
	strncpy(ml_tz->name, (char *)reg_w->data, MLMUX_TZ_NAME_SIZE);
	ml_tz->id = ret;

	np = tdev->dev->of_node;
	if (snprintf(prop_name, sizeof(prop_name), "trip-%s-critical-temp",
			ml_tz->name) > 0)
		of_property_read_u32(np, prop_name, &ml_tz->critical_temp);

free:
	mutex_unlock(&tdev->lock);
	devm_kfree(tdev->dev, reg_w->data);
	devm_kfree(tdev->dev, reg_w);
}

static inline void mlmux_therm_destroy(struct mlmux_therm_dev_data *tdev,
					  struct mlmux_therm_zone *ml_tz)
{
	thermal_zone_device_unregister(ml_tz->tz);
	idr_remove(&tdev->zones, ml_tz->id);
	kfree(ml_tz);
}

static void mlmux_therm_unregister_all(struct mlmux_therm_dev_data *tdev)
{
	struct mlmux_therm_zone *ml_tz;
	int i;

	mutex_lock(&tdev->reg_lock);
	if (!idr_is_empty(&tdev->zones))
		idr_for_each_entry(&tdev->zones, ml_tz, i)
			mlmux_therm_destroy(tdev, ml_tz);
	mutex_unlock(&tdev->reg_lock);
}

static int mlmux_queue_process_msg(struct mlmux_therm_dev_data *tdev,
				   const void *data,
				   uint16_t size,
				   void (*work_func)(struct work_struct *))
{
	int ret = 0;
	struct mlmux_therm_work *therm_work;

	therm_work = devm_kzalloc(tdev->dev, sizeof(*therm_work), GFP_KERNEL);
	if (!therm_work) {
		ret = -1;
		goto exit;
	}

	therm_work->data = devm_kzalloc(tdev->dev, size, GFP_KERNEL);
	if (!therm_work->data) {
		devm_kfree(tdev->dev, therm_work);
		ret = -1;
		goto exit;
	}
	memcpy(therm_work->data, data, size);
	therm_work->tdev = tdev;

	INIT_WORK(&therm_work->work, work_func);
	queue_work(tdev->work_q, &therm_work->work);

exit:
	return ret;
}

static inline int mlmux_size_check(struct mlmux_therm_dev_data *tdev,
					   uint32_t len, size_t exp)
{
	if (len != exp) {
		dev_err(tdev->dev, "Unexpected length %d vs %zu\n", len, exp);
		return -EMSGSIZE;
	}
	return 0;
}

static void mlmux_therm_recv(struct ml_mux_client *cli, uint32_t len, void *msg)
{
	struct mlmux_therm_dev_data *tdev;
	struct mlmux_therm_rx_msg *msg_rx = (struct mlmux_therm_rx_msg *)msg;
	struct mlmux_therm_zone *ml_tz;

	tdev = container_of(cli, struct mlmux_therm_dev_data, client);

	switch (msg_rx->type) {
	case MLMUX_THERM_RX_UPDATE:
		if (mlmux_size_check(tdev, len, MLMUX_THERM_RX_SIZE(update)))
			return;

		dev_dbg(tdev->dev, "id=%d, temp=%d\n", msg_rx->u.update.id,
			msg_rx->u.update.temp);

		mutex_lock(&tdev->lock);
		ml_tz = (struct mlmux_therm_zone *)idr_find(&tdev->zones,
							   msg_rx->u.update.id);
		if (!ml_tz) {
			dev_err(tdev->dev, "Can't find TZ %d\n",
				msg_rx->u.update.id);
			mutex_unlock(&tdev->lock);
			return;
		}
		ml_tz->temp = msg_rx->u.update.temp;
		mutex_unlock(&tdev->lock);

	break;

	case MLMUX_THERM_RX_REG:
		if (!mlmux_size_check(tdev, len, MLMUX_THERM_RX_SIZE(reg)))
			mlmux_queue_process_msg(tdev, &msg_rx->u.reg, len - 1,
						mlmux_therm_process_reg);
	break;

	case MLMUX_THERM_RX_SUSPEND_ACK:
		if (mlmux_size_check(tdev, len, MLMUX_THERM_RX_SIZE(data)))
			return;
		if (!msg_rx->u.data)
			complete(&tdev->suspend);
		else
			dev_err(tdev->dev, "Suspend failed ret = %d",
				msg_rx->u.data);
	break;

	case MLMUX_THERM_RX_RESUME_ACK:
		if (mlmux_size_check(tdev, len, MLMUX_THERM_RX_SIZE(data)))
			return;
		if (!msg_rx->u.data)
			complete(&tdev->resume);
		else
			dev_err(tdev->dev, "Resume failed ret = %d",
				msg_rx->u.data);
	break;

	default:
		dev_err(tdev->dev, "Unknown cmd type %d\n", msg_rx->type);
		return;
	}
}

static void mlmux_therm_process_open(struct work_struct *work)
{
	struct mlmux_therm_work *open_w;
	struct mlmux_therm_dev_data *tdev;

	open_w = container_of(work, struct mlmux_therm_work, work);
	tdev = open_w->tdev;

	mutex_lock(&tdev->lock);
	tdev->chan_up = *(bool *)open_w->data;
	if (!tdev->chan_up)
		mlmux_therm_unregister_all(tdev);
	mutex_unlock(&tdev->lock);

	devm_kfree(tdev->dev, open_w->data);
	devm_kfree(tdev->dev, open_w);
}

static void mlmux_therm_open(struct ml_mux_client *cli, bool is_open)
{
	struct mlmux_therm_dev_data *tdev;

	tdev = container_of(cli, struct mlmux_therm_dev_data, client);
	dev_info(tdev->dev, "mlmux channel is %s\n",
		 is_open ? "opened" : "closed");

	mlmux_queue_process_msg(tdev, &is_open, sizeof(is_open),
			       mlmux_therm_process_open);
}

static int mlmux_therm_parse_dt(struct mlmux_therm_dev_data *tdev)
{
	struct device_node *np = tdev->dev->of_node;

	if (!np) {
		dev_err(tdev->dev, "therm-mlmux node not found\n");
		return -ENODEV;
	}

	if (of_property_read_string(np, "ml,chan-name", &tdev->chan_name)) {
		dev_err(tdev->dev, "ml,chan-name undefined\n");
		return -EINVAL;
	}

	return 0;
}

static int mlmux_therm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mlmux_therm_dev_data *tdev = NULL;

	tdev = devm_kzalloc(&pdev->dev, sizeof(*tdev), GFP_KERNEL);
	if (!tdev) {
		ret = -ENOMEM;
		goto exit;
	}
	tdev->dev = &pdev->dev;
	platform_set_drvdata(pdev, tdev);
	ret = mlmux_therm_parse_dt(tdev);
	if (ret)
		goto exit;

	tdev->client.dev = tdev->dev;
	tdev->client.notify_open = mlmux_therm_open;
	tdev->client.receive_cb = mlmux_therm_recv;

	init_completion(&tdev->suspend);
	init_completion(&tdev->resume);
	mutex_init(&tdev->lock);
	mutex_init(&tdev->reg_lock);
	idr_init(&tdev->zones);

	tdev->work_q = alloc_workqueue(tdev->chan_name, WQ_UNBOUND, 1);
	if (!tdev->work_q) {
		dev_info(tdev->dev, "Failed to create workqueue\n");
		ret = -ENOMEM;
		goto exit_idr_destroy;
	}

	/* Revisit: Right now we do not care that channel is already open,
	*	    the next step in registration is controlled by msgs
	*/
	ret = ml_mux_request_channel(&tdev->client, (char *)tdev->chan_name);
	if (ret < 0)
		goto exit_wq_destroy;

	return 0;

exit_wq_destroy:
	destroy_workqueue(tdev->work_q);
exit_idr_destroy:
	idr_destroy(&tdev->zones);
	mutex_destroy(&tdev->lock);
	mutex_destroy(&tdev->reg_lock);
exit:
	return ret;
}

int mlmux_therm_remove(struct platform_device *pdev)
{
	struct mlmux_therm_dev_data *tdev = platform_get_drvdata(pdev);

	ml_mux_release_channel(tdev->client.ch);
	destroy_workqueue(tdev->work_q);
	mlmux_therm_unregister_all(tdev);
	idr_destroy(&tdev->zones);
	mutex_destroy(&tdev->lock);
	mutex_destroy(&tdev->reg_lock);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mlmux_therm_suspend(struct device *dev)
{
	int ret;
	unsigned long jiffies_left;
	struct mlmux_therm_dev_data *tdev = dev_get_drvdata(dev);
	struct mlmux_therm_tx_msg tx_msg;

	reinit_completion(&tdev->suspend);
	tx_msg.type = MLMUX_THERM_TX_SUSPEND;
	tx_msg.u.data = 1;
	ret = ml_mux_send_msg(tdev->client.ch,
			(uint32_t)MLMUX_THERM_TX_MSG_SIZE(data), &tx_msg);
	if (ret)
		return ret;

	jiffies_left = wait_for_completion_timeout(&tdev->suspend,
				msecs_to_jiffies(MLMUX_THERM_SUSPEND_TO));
	if (!jiffies_left)
		return -ETIMEDOUT;
	dev_dbg(dev, "suspend\n");

	return 0;
}

static int mlmux_therm_resume(struct device *dev)
{
	int ret;
	unsigned long jiffies_left;
	struct mlmux_therm_dev_data *tdev = dev_get_drvdata(dev);
	struct mlmux_therm_tx_msg tx_msg;

	reinit_completion(&tdev->resume);
	tx_msg.type = MLMUX_THERM_TX_RESUME;
	tx_msg.u.data = 1;

	ret = ml_mux_send_msg(tdev->client.ch,
			(uint32_t)MLMUX_THERM_TX_MSG_SIZE(data), &tx_msg);
	if (ret)
		return ret;

	jiffies_left = wait_for_completion_timeout(&tdev->resume,
				msecs_to_jiffies(MLMUX_THERM_RESUME_TO));
	if (!jiffies_left)
		return -ETIMEDOUT;
	dev_dbg(dev, "resume\n");

	return 0;
}

static const struct dev_pm_ops mlmux_therm_pm = {
	.suspend = mlmux_therm_suspend,
	.resume = mlmux_therm_resume,
};
#define MLMUX_THERM_PM_OPS	(&mlmux_therm_pm)
#else
#define MLMUX_THERM_PM_OPS	NULL
#endif


static const struct of_device_id ml_therm_match_table[] = {
	{ .compatible = "ml,therm_mlmux", },
	{ },
};
MODULE_DEVICE_TABLE(of, ml_therm_match_table);

static struct platform_driver ml_therm_driver = {
	.driver = {
		.name = "therm_mlmux",
		.of_match_table = ml_therm_match_table,
		.pm = MLMUX_THERM_PM_OPS,
	},
	.probe = mlmux_therm_probe,
	.remove = mlmux_therm_remove,
};

module_platform_driver(ml_therm_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("ML MUX client driver for thermal zones");
MODULE_LICENSE("GPL v2");
