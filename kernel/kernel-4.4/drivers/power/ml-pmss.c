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
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include "ml-pmss.h"
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/ktime.h>


#define MLMUX_PMSS_SUSPEND_TO    (2000)
#define MLMUX_PMSS_RESUME_TO     (2000)
#define MLMUX_PMSS_MIN_CURR_THRESH_MA (100)
#define ML_PMSS_VCOMP_ALERT_WINDOW_S (5)
#define ML_PMSS_PSY_CHECK_INIT_TIMER_DELAY (120*HZ)
#define ML_PMSS_PSY_CHECK_TIMER_DELAY (20*HZ)

#define ML_PMSS_NEQ(old, new, value) \
	(old->value != new->value)

#define ML_PMSS_THRESHOLD(old, new, value, thresh) \
	((new->value < thresh && old->value >= thresh) || \
	(new->value >= thresh && old->value < thresh))

#define ML_PMSS_STATE_CHANGE(member) \
	ML_PMSS_NEQ(state, new_state, member)

#define ML_PMSS_STATE_BATT_THRESH(member, thresh) \
	(ML_PMSS_THRESHOLD(state, new_state, member, thresh))

enum {
	PMSS_CHRG_SSTATE_UVLO,
	PMSS_CHRG_SSTATE_TRKL,
	PMSS_CHRG_SSTATE_FAST,
	PMSS_CHRG_SSTATE_FULL,
	PMSS_CHRG_SSTATE_NO_CHRG,
	PMSS_CHRG_SSTATE_NO_SUPP,
	PMSS_CHRG_SSTATE_FAULT,
} PMSS_CHRG_SSTATE;

static enum power_supply_property mlpmss_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_AUTHENTIC,
};

static enum power_supply_property mlpmss_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGER_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_USB_OTG,
	POWER_SUPPLY_PROP_USB_ORIENTATION,
	POWER_SUPPLY_PROP_CAP_MISMATCH,
	POWER_SUPPLY_PROP_CHARGER_INSUFFICIENT,
};

static const struct mlmux_pmss_usb_cd_t {
	int extcon;
	int psy_type;
} mlmux_pmss_usb_cd[] = {
	{EXTCON_CHG_USB_DCP, POWER_SUPPLY_TYPE_USB_DCP},
	{EXTCON_CHG_USB_CDP, POWER_SUPPLY_TYPE_USB_CDP},
	{EXTCON_NONE, POWER_SUPPLY_TYPE_UNKNOWN},
};

static inline bool mlmux_pmss_batt_cold(struct mlmux_pmss_batt *batt,
					   struct mlmux_pmss_state *state)
{
	return state->batt_temp < batt->cold_thresh;
}

static inline bool mlmux_pmss_batt_cool(struct mlmux_pmss_batt *batt,
					   struct mlmux_pmss_state *state)
{
	return state->batt_temp < batt->cool_thresh;
}

static inline bool mlmux_pmss_batt_warm(struct mlmux_pmss_batt *batt,
					   struct mlmux_pmss_state *state)
{
	return state->batt_temp >= batt->warm_thresh;
}

static inline bool mlmux_pmss_batt_hot(struct mlmux_pmss_batt *batt,
					   struct mlmux_pmss_state *state)
{
	return state->batt_temp >= batt->hot_thresh;
}

static inline bool mlmux_pmss_batt_curr_change(struct mlmux_pmss_state *state,
					   struct mlmux_pmss_state *new_state)
{
	bool dch_to_chg = (state->batt_curr >= 0 && new_state->batt_curr < 0);
	bool chg_to_dch = (state->batt_curr < 0 && new_state->batt_curr >= 0);

	return dch_to_chg || chg_to_dch;
}

static inline bool mlmux_pmss_batt_changed(struct mlmux_pmss_batt *batt,
					   struct mlmux_pmss_state *state,
					   struct mlmux_pmss_state *new_state)
{

	return ML_PMSS_STATE_CHANGE(soc) ||
		ML_PMSS_STATE_CHANGE(cable_conn) ||
		ML_PMSS_STATE_CHANGE(chrg_state) ||
		ML_PMSS_STATE_CHANGE(chrg_en) ||
		ML_PMSS_STATE_CHANGE(chrg_curr_max) ||
		ML_PMSS_STATE_CHANGE(req_shutdown) ||
		ML_PMSS_STATE_CHANGE(vbat_max) ||
		ML_PMSS_STATE_BATT_THRESH(batt_temp, batt->cool_thresh) ||
		ML_PMSS_STATE_BATT_THRESH(batt_temp, batt->warm_thresh) ||
		ML_PMSS_STATE_BATT_THRESH(batt_temp, batt->cold_thresh) ||
		ML_PMSS_STATE_BATT_THRESH(batt_temp, batt->hot_thresh);
}

static inline bool mlmux_pmss_chrg_changed(struct mlmux_pmss_state *state,
					   struct mlmux_pmss_state *new_state)
{
	return ML_PMSS_STATE_CHANGE(psy_type) ||
		ML_PMSS_STATE_CHANGE(cable_conn) ||
		ML_PMSS_STATE_CHANGE(is_dfp);
}

static int mlmux_pmss_proc_msg(struct mlmux_pmss_data *pmss,
				  const void *data, size_t size,
				  void (*work_func)(struct work_struct *))
{
	int ret = 0;
	struct mlmux_pmss_work *pmss_work;
	struct timer_list *timer = &pmss->psy_check_timer;

	pmss_work = devm_kzalloc(pmss->dev, sizeof(*pmss_work), GFP_KERNEL);
	if (!pmss_work) {
		ret = -ENOMEM;
		goto exit;
	}

	pmss_work->data = devm_kzalloc(pmss->dev, size, GFP_KERNEL);
	if (!pmss_work->data) {
		ret = -ENOMEM;
		devm_kfree(pmss->dev, pmss_work);
		goto exit;
	}
	memcpy(pmss_work->data, data, size);
	pmss_work->pmss = pmss;

	INIT_WORK(&pmss_work->work, work_func);
	queue_work(pmss->work_q, &pmss_work->work);
	mod_timer(timer, jiffies + ML_PMSS_PSY_CHECK_TIMER_DELAY);

exit:
	return ret;
}

static void mlmux_pmss_log_state(struct mlmux_pmss_data *pmss,
				struct mlmux_pmss_state *msg_recv)
{
	static int count;

	if (!count) {
		count = 10;
		dev_info(pmss->dev, "msg 0x%02x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
			*(uint8_t *)((uint32_t *)msg_recv + 5),
			*((uint32_t *)msg_recv + 4),
			*((uint32_t *)msg_recv + 3),
			*((uint32_t *)msg_recv + 2),
			*((uint32_t *)msg_recv + 1),
			*(uint32_t *)msg_recv);
	}
	count--;
}

static void mlmux_pmss_rx_state(struct work_struct *work)
{
	struct mlmux_pmss_work *pmss_work;
	struct mlmux_pmss_data *pmss;
	struct mlmux_pmss_state *new_state;
	struct mlmux_pmss_state *state;
	struct mlmux_pmss_batt *batt;
	bool usb_psy_update = false;
	bool batt_psy_update = false;
	bool attached = false;
	bool removed = false;
	bool host_mode = false;
	bool dr_swap = false;

	pmss_work = container_of(work, struct mlmux_pmss_work, work);
	pmss = pmss_work->pmss;
	state = &pmss->state;
	batt = &pmss->batt;
	new_state = (struct mlmux_pmss_state *)pmss_work->data;

	mutex_lock(&pmss->lock);
	if (!pmss->chan_up) {
		dev_info(pmss->dev, "rx_state: mlmux channel is not up\n");
		goto exit;
	}

	/*
	 * Always send uevent if the battery is hot or cold as an
	 * indication of extreme conditions.
	 */
	if (mlmux_pmss_batt_changed(batt, state, new_state) ||
	    mlmux_pmss_batt_cold(batt, new_state) ||
	    mlmux_pmss_batt_hot(batt, new_state) ||
	    mlmux_pmss_batt_curr_change(state, new_state))
		batt_psy_update = true;

	if (mlmux_pmss_chrg_changed(state, new_state)) {
		usb_psy_update = true;
		attached = new_state->cable_conn && !state->cable_conn;
		host_mode = !new_state->is_dfp && new_state->cable_conn;
		removed = !new_state->cable_conn && state->cable_conn;
		dr_swap = (new_state->is_dfp != state->is_dfp) &&
			  !attached && new_state->cable_conn;
	}

	mlmux_pmss_log_state(pmss, new_state);
	/* Temperature overridden through debugfs doesn't get updated */
	if (unlikely(pmss->manual_temp)) {
		new_state->batt_temp = state->batt_temp;
		new_state->chrg_temp = state->chrg_temp;
	}
	*state = *new_state;

	if (batt_psy_update)
		power_supply_changed(pmss->batt_psy);
	if (usb_psy_update) {
		if (host_mode) {
			dev_info(pmss->dev, "EXTCON USB: host mode\n");
			extcon_set_state(pmss->otg_edev, EXTCON_USB_HOST);
		} else if (attached) {
			dev_info(pmss->dev, "EXTCON USB: connected\n");
			extcon_set_state(pmss->otg_edev, EXTCON_USB);
		} else if (removed) {
			dev_info(pmss->dev, "EXTCON USB: disconnected\n");
			extcon_set_state(pmss->otg_edev, EXTCON_NONE);
		} else if (dr_swap) {
			dev_info(pmss->dev, "EXTCON USB: dr_swap\n");
			extcon_set_state(pmss->otg_edev,
					 new_state->is_dfp ?
					 EXTCON_USB : EXTCON_USB_HOST);
		}
		power_supply_changed(pmss->usb_psy);
	}
	if (pmss->batt_therm)
		thermal_zone_device_update(pmss->batt_therm);
	if (pmss->chrg_therm)
		thermal_zone_device_update(pmss->chrg_therm);

exit:
	mutex_unlock(&pmss->lock);
	devm_kfree(pmss->dev, new_state);
	devm_kfree(pmss->dev, pmss_work);
}

static void mlmux_pmss_rx_state_2(struct work_struct *work)
{
	struct mlmux_pmss_work *pmss_work;
	struct mlmux_pmss_data *pmss;
	struct mlmux_pmss_state_2 *new_state;
	struct mlmux_pmss_state_2 *state;

	pmss_work = container_of(work, struct mlmux_pmss_work, work);
	pmss = pmss_work->pmss;
	state = &pmss->state_2;
	new_state = (struct mlmux_pmss_state_2 *)pmss_work->data;

	mutex_lock(&pmss->lock);
	if (!pmss->chan_up) {
		dev_info(pmss->dev, "rx_state: mlmux channel is not up\n");
		goto exit;
	}

	*state = *new_state;

exit:
	mutex_unlock(&pmss->lock);
	devm_kfree(pmss->dev, new_state);
	devm_kfree(pmss->dev, pmss_work);
}

static void mlmux_pmss_rx_init(struct work_struct *work)
{
	struct mlmux_pmss_work *pmss_work;
	struct mlmux_pmss_data *pmss;
	struct mlmux_pmss_init *params;

	pmss_work = container_of(work, struct mlmux_pmss_work, work);
	pmss = pmss_work->pmss;
	params = (struct mlmux_pmss_init *)pmss_work->data;

	mutex_lock(&pmss->lock);
	if (pmss->chan_up)
		pmss->init_params = *params;
	else
		dev_info(pmss->dev, "rx_init: mlmux channel is not up\n");
	mutex_unlock(&pmss->lock);

	devm_kfree(pmss->dev, params);
	devm_kfree(pmss->dev, pmss_work);
}

static inline int mlmux_size_check(struct mlmux_pmss_data *pmss,
					   uint32_t len, size_t exp)
{
	if (len != exp) {
		dev_err(pmss->dev, "Unexpected length %d vs %zu\n", len, exp);
		return -EMSGSIZE;
	}
	return 0;
}

static void mlmux_pmss_recv(struct ml_mux_client *cli, uint32_t len, void *msg)
{
	struct mlmux_pmss_data *pmss;
	struct mlmux_pmss_rx_pkt *pkt = (struct mlmux_pmss_rx_pkt *)msg;

	pmss = container_of(cli, struct mlmux_pmss_data, client);

	switch (pkt->type) {
	case MLMUX_PMSS_RX_INIT:
		if (!mlmux_size_check(pmss, len, MLMUX_PMSS_RX_SIZE(init)))
			mlmux_pmss_proc_msg(pmss, &pkt->u.init,
				sizeof(pkt->u.init), mlmux_pmss_rx_init);
		break;
	case MLMUX_PMSS_RX_STATE:
		if (!mlmux_size_check(pmss, len, MLMUX_PMSS_RX_SIZE(state)))
			mlmux_pmss_proc_msg(pmss, &pkt->u.state,
				sizeof(pkt->u.state), mlmux_pmss_rx_state);
		break;
	case MLMUX_PMSS_RX_STATE_2:
		if (!mlmux_size_check(pmss, len, MLMUX_PMSS_RX_SIZE(state_2)))
			mlmux_pmss_proc_msg(pmss, &pkt->u.state_2,
				sizeof(pkt->u.state_2), mlmux_pmss_rx_state_2);
		break;
	case MLMUX_PMSS_RX_SUSPEND_ACK:
		if (mlmux_size_check(pmss, len, MLMUX_PMSS_RX_SIZE(data)))
			return;
		if (!pkt->u.data)
			complete(&pmss->suspend);
		else
			dev_err(pmss->dev, "Suspend fail ret=%d", pkt->u.data);
		break;
	case MLMUX_PMSS_RX_RESUME_ACK:
		if (mlmux_size_check(pmss, len, MLMUX_PMSS_RX_SIZE(data)))
			return;
		if (!pkt->u.data)
			complete(&pmss->resume);
		else
			dev_err(pmss->dev, "Resume fail ret=%d", pkt->u.data);
		break;
	default:
		dev_err(pmss->dev, "unknown pkt type: %d\n", pkt->type);
		break;
	}
}

static int mlmux_pmss_usb_prop_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static int mlmux_pmss_get_usb_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct mlmux_pmss_data *pmss = power_supply_get_drvdata(psy);
	struct mlmux_pmss_state *state = &pmss->state;
	struct mlmux_pmss_state_2 *state_2 = &pmss->state_2;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state->cable_conn;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = state->cable_conn;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = state_2->adpt_max_mA * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = state_2->adpt_mV * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = state->adpt_curr * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state->vbus * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = state->chrg_temp;
		break;
	case POWER_SUPPLY_PROP_CHARGER_TYPE:
		val->intval = state->psy_type;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (state->chrg_temp > MLMUX_PMSS_CHRG_TEMP_LIM)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_USB_OTG:
		val->intval = !state->is_dfp;
		break;
	case POWER_SUPPLY_PROP_USB_ORIENTATION:
		val->intval = state->cable_orient;
		break;
	case POWER_SUPPLY_PROP_CAP_MISMATCH:
		val->intval = state->chrg_cap_mismatch;
		break;
	case POWER_SUPPLY_PROP_CHARGER_INSUFFICIENT:
		val->intval = state->chrg_insufficient;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mlmux_pmss_set_usb_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct mlmux_pmss_data *pmss = power_supply_get_drvdata(psy);
	struct mlmux_pmss_tx_ctrl msg;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		msg.cmd = MLMUX_PMSS_TX_ADPT_CURR;
		msg.data = val->intval / 1000;
		break;
	default:
		return -EINVAL;
	}
	ret = ml_mux_send_msg(pmss->client.ch, sizeof(msg), &msg);
	if (ret)
		dev_err(pmss->dev, "mux send fail: %d. ret = %d\n",
			msg.cmd, ret);

	return ret;
}

static int mlmux_pmss_batt_prop_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static int64_t mlmux_pmss_get_batt_status(struct mlmux_pmss_state *const state)
{
	int64_t val;

	switch (state->chrg_state) {
	case PMSS_CHRG_SSTATE_FULL:
		val = POWER_SUPPLY_STATUS_FULL;
		break;
	case PMSS_CHRG_SSTATE_NO_CHRG:
		val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case PMSS_CHRG_SSTATE_FAST:
	case PMSS_CHRG_SSTATE_TRKL:
		val = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case PMSS_CHRG_SSTATE_NO_SUPP:
	case PMSS_CHRG_SSTATE_FAULT:
		val = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		val = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return val;
}

static int64_t mlmux_pmss_get_chrg_type(struct mlmux_pmss_state *const state)
{
	int64_t val;

	switch (state->chrg_state) {
	case PMSS_CHRG_SSTATE_FAST:
		val = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case PMSS_CHRG_SSTATE_TRKL:
		val = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case PMSS_CHRG_SSTATE_NO_CHRG:
	case PMSS_CHRG_SSTATE_FULL:
		val = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	default:
		val = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		break;
	}

	return val;
}

static int mlmux_pmss_get_batt_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct mlmux_pmss_data *pmss = power_supply_get_drvdata(psy);
	struct mlmux_pmss_state *state = &pmss->state;
	struct mlmux_pmss_state_2 *state_2 = &pmss->state_2;
	struct mlmux_pmss_init *init = &pmss->init_params;
	struct mlmux_pmss_batt *batt = &pmss->batt;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = mlmux_pmss_get_batt_status(state);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = state->soc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = state_2->full_charge_cap * 1000;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = mlmux_pmss_get_chrg_type(state);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = state->batt_present;
		break;
	case POWER_SUPPLY_PROP_AUTHENTIC:
		val->intval = state->batt_auth;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = state->vbat * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state->vbat_adc * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = state->vbat_max * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = state->batt_curr * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = init->batt_capacity * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = state->batt_temp;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = state->chrg_curr_max * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		val->intval = state->chrg_en;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (mlmux_pmss_batt_hot(batt, state))
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (mlmux_pmss_batt_cold(batt, state))
			val->intval = POWER_SUPPLY_HEALTH_COLD;
		else if (mlmux_pmss_batt_warm(batt, state))
			val->intval = POWER_SUPPLY_HEALTH_WARM;
		else if (mlmux_pmss_batt_cool(batt, state))
			val->intval = POWER_SUPPLY_HEALTH_COOL;
		else if (state->req_shutdown)
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = init->batt_id_res;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mlmux_pmss_set_batt_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct mlmux_pmss_data *pmss = power_supply_get_drvdata(psy);
	struct mlmux_pmss_tx_ctrl msg;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		msg.cmd = MLMUX_PMSS_TX_ALGO_EN;
		msg.data = !!val->intval;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		msg.cmd = MLMUX_PMSS_TX_CHRG_CURR;
		msg.data = val->intval / 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		msg.cmd = MLMUX_PMSS_TX_VSYS;
		msg.data = val->intval / 1000;
		break;
	default:
		return -EINVAL;
	}
	ret = ml_mux_send_msg(pmss->client.ch, sizeof(msg), &msg);
	if (ret)
		dev_err(pmss->dev, "mux send fail: %d. ret = %d\n",
			msg.cmd, ret);

	return ret;
}

static inline int mlmux_pmss_reg_batt_psy(struct mlmux_pmss_data *pmss,
				struct power_supply_config *batt_psy_cfg)
{
	int ret = 0;

	pmss->batt_desc.name = "battery";
	pmss->batt_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	pmss->batt_desc.no_thermal = pmss->en_therm_sens;
	pmss->batt_desc.properties = mlpmss_batt_props;
	pmss->batt_desc.num_properties = ARRAY_SIZE(mlpmss_batt_props);
	pmss->batt_desc.get_property = mlmux_pmss_get_batt_property;
	pmss->batt_desc.set_property = mlmux_pmss_set_batt_property;
	pmss->batt_desc.property_is_writeable =
		mlmux_pmss_batt_prop_is_writeable;

	pmss->batt_psy = power_supply_register(pmss->dev,
				&pmss->batt_desc, batt_psy_cfg);
	if (IS_ERR(pmss->batt_psy)) {
		ret = PTR_ERR(pmss->batt_psy);
		pmss->batt_psy = NULL;
		dev_err(pmss->dev, "batt_psy reg fail: ret=%d\n", ret);
		/* TODO: fail reg this should be high severety */
	}

	return ret;
}

static inline int mlmux_pmss_reg_usb_psy(struct mlmux_pmss_data *pmss,
				struct power_supply_config *usb_psy_cfg)
{
	int ret = 0;

	pmss->usb_desc.name = "usb_pd";
	pmss->usb_desc.type = POWER_SUPPLY_TYPE_USB_PD;
	pmss->usb_desc.no_thermal = pmss->en_therm_sens;
	pmss->usb_desc.properties = mlpmss_usb_props;
	pmss->usb_desc.num_properties = ARRAY_SIZE(mlpmss_usb_props);
	pmss->usb_desc.get_property = mlmux_pmss_get_usb_property;
	pmss->usb_desc.set_property = mlmux_pmss_set_usb_property;
	pmss->usb_desc.property_is_writeable = mlmux_pmss_usb_prop_is_writeable;

	pmss->usb_psy = power_supply_register(pmss->dev,
					&pmss->usb_desc, usb_psy_cfg);
	if (IS_ERR(pmss->usb_psy)) {
		ret = PTR_ERR(pmss->usb_psy);
		pmss->usb_psy = NULL;
		dev_err(pmss->dev, "usb_psy reg fail: ret=%d\n", ret);
		/* TODO: fail reg this should be high severety */
	}

	return ret;
}

static int mlmux_pmss_read_batt_temp(void *data, int *temp)
{
	struct mlmux_pmss_data *pmss = data;

	*temp = pmss->state.batt_temp * 100;

	return 0;
}

static int mlmux_pmss_read_chrg_temp(void *data, int *temp)
{
	struct mlmux_pmss_data *pmss = data;

	*temp = pmss->state.chrg_temp * 100;

	return 0;
}

static struct thermal_zone_of_device_ops tzd_batt_ops = {
	.get_temp = mlmux_pmss_read_batt_temp,
};

static struct thermal_zone_of_device_ops tzd_chrg_ops = {
	.get_temp = mlmux_pmss_read_chrg_temp,
};

static inline int mlmux_pmss_reg_batt_sensor(struct mlmux_pmss_data *pmss)
{
	int ret = 0;

	pmss->batt_therm = thermal_zone_of_sensor_register(pmss->dev,
			pmss->batt_id, pmss, &tzd_batt_ops);
	if (IS_ERR(pmss->batt_therm)) {
		ret = PTR_ERR(pmss->batt_therm);
		dev_err(pmss->dev, "Therm batt sensor reg fail: %d\n", ret);
		pmss->batt_therm = NULL;
	}

	return ret;
}

static inline int mlmux_pmss_reg_chrg_sensor(struct mlmux_pmss_data *pmss)
{
	int ret = 0;

	pmss->chrg_therm = thermal_zone_of_sensor_register(pmss->dev,
			pmss->chrg_id, pmss, &tzd_chrg_ops);
	if (IS_ERR(pmss->chrg_therm)) {
		ret = PTR_ERR(pmss->chrg_therm);
		dev_err(pmss->dev, "Therm chrg sensor reg fail: %d\n", ret);
		pmss->chrg_therm = NULL;
	}

	return ret;
}

static void mlmux_pmss_unregister_fw(struct mlmux_pmss_data *pmss)
{
	if (pmss->batt_psy) {
		power_supply_unregister(pmss->batt_psy);
		pmss->batt_psy = NULL;
	}
	if (pmss->batt_therm) {
		thermal_zone_of_sensor_unregister(pmss->dev,  pmss->batt_therm);
		pmss->batt_therm = NULL;
	}
	if (pmss->usb_psy) {
		power_supply_unregister(pmss->usb_psy);
		pmss->usb_psy = NULL;
	}
	if (pmss->chrg_therm) {
		thermal_zone_of_sensor_unregister(pmss->dev, pmss->chrg_therm);
		pmss->chrg_therm = NULL;
	}
}

static int mlmux_pmss_register_fw(struct mlmux_pmss_data *pmss)
{
	int ret = 0;
	struct power_supply_config batt_psy_cfg = {0};
	struct power_supply_config usb_psy_cfg = {0};

	batt_psy_cfg.drv_data = pmss;
	usb_psy_cfg.drv_data = pmss;
	/* TODO: fail reg this should be high severety */
	if (!pmss->batt_psy) {
		ret = mlmux_pmss_reg_batt_psy(pmss, &batt_psy_cfg);
		if (ret)
			goto exit;
	}
	if (!pmss->usb_psy) {
		ret = mlmux_pmss_reg_usb_psy(pmss, &usb_psy_cfg);
		if (ret)
			goto exit;
	}
	if (pmss->en_therm_sens) {
		if (!pmss->batt_therm)
			mlmux_pmss_reg_batt_sensor(pmss);
		if (!pmss->chrg_therm)
			mlmux_pmss_reg_chrg_sensor(pmss);
	}

exit:
	return ret;
}

void mlmux_pmss_usb_cd_work(struct work_struct *wrk)
{
	struct mlmux_pmss_data *pmss = container_of(wrk, struct mlmux_pmss_data,
						    usb_cd_work);
	struct extcon_dev *edev = pmss->usb_cd_edev;
	struct mlmux_pmss_tx_ctrl msg;
	const struct mlmux_pmss_usb_cd_t *cd = mlmux_pmss_usb_cd;
	const unsigned int *cbl = edev->supported_cable;
	int ret = 0;
	int i, j;

	/* find attached cable type */
	for (i = 0; cbl[i] != EXTCON_NONE; i++)
		if (extcon_get_cable_state_(pmss->usb_cd_edev, cbl[i]))
			break;

	/* find message to send */
	for (j = 0; cd[j].extcon != cbl[i] && cd[j].extcon != EXTCON_NONE; j++)
		continue;

	if (cd[j].extcon != EXTCON_NONE) {
		msg.cmd = MLMUX_PMSS_TX_BC12_NOTIFY;
		msg.data = cd[j].psy_type;
		ret = ml_mux_send_msg(pmss->client.ch, sizeof(msg), &msg);
		if (ret)
			dev_err(pmss->dev, "mux send fail: %d. ret = %d\n",
				msg.cmd, ret);
	} else {
		dev_warn(pmss->dev, "Unsupported cable detected, extcon=%d\n",
			 cbl[i]);
	}
}

int mlmux_pmss_usb_cd_notifier(struct notifier_block *nb,
			       unsigned long event, void *unused)
{
	struct mlmux_pmss_data *pmss;

	pmss = container_of(nb, struct mlmux_pmss_data, usb_cd_nb);
	schedule_work(&pmss->usb_cd_work);

	return NOTIFY_DONE;
}

static int mlmux_pmss_reg_usb_cd_extcon(struct mlmux_pmss_data *pmss)
{
	struct device_node *np;
	struct platform_device *bc12_pdev = NULL;
	int i = 0;
	int ret = 0;

	np = of_parse_phandle(pmss->dev->of_node, "bc12_cd", 0);
	if (np) {
		bc12_pdev = of_find_device_by_node(np);
		of_node_put(np);
		if (!bc12_pdev)
			return 0;
		pmss->usb_cd_edev = extcon_get_extcon_dev(bc12_pdev->name);
		/* We know device exists but might not have been registered with
		 * extcon yet. Let's wait.
		 */
		if (IS_ERR_OR_NULL(pmss->usb_cd_edev))
			return -EPROBE_DEFER;
	} else {
		/* BC1.2 Charger detector is optional */
		return 0;
	}
	INIT_WORK(&pmss->usb_cd_work, mlmux_pmss_usb_cd_work);
	pmss->usb_cd_nb.notifier_call = mlmux_pmss_usb_cd_notifier;

	for (i = 0; mlmux_pmss_usb_cd[i].extcon != EXTCON_NONE; i++) {
		ret = extcon_register_notifier(pmss->usb_cd_edev,
					       mlmux_pmss_usb_cd[i].extcon,
					       &pmss->usb_cd_nb);
		if (ret)
			dev_warn(pmss->dev, "unable to reg extcon=%d. err=%d\n",
				 mlmux_pmss_usb_cd[i].extcon, ret);
	}

	return 0;
}

static void mlmux_pmss_unreg_usb_cd_extcon(struct mlmux_pmss_data *pmss)
{
	const struct mlmux_pmss_usb_cd_t *cd = mlmux_pmss_usb_cd;
	int i;

	if (!pmss->usb_cd_edev)
		return;

	for (i = 0; cd[i].extcon != EXTCON_NONE; i++)
		extcon_unregister_notifier(pmss->usb_cd_edev, cd[i].extcon,
					   &pmss->usb_cd_nb);
}

static void mlmux_pmss_rx_open(struct work_struct *work)
{
	struct mlmux_pmss_work *pmss_work;
	struct mlmux_pmss_data *pmss;
	bool is_open;

	pmss_work = container_of(work, struct mlmux_pmss_work, work);
	pmss = pmss_work->pmss;
	is_open = *(bool *)pmss_work->data;

	mutex_lock(&pmss->lock);
	if (is_open && !pmss->chan_up) {
		if (mlmux_pmss_register_fw(pmss))
			goto exit;
	} else if (!is_open && pmss->chan_up) {
		mlmux_pmss_unregister_fw(pmss);
	} else {
		dev_info(pmss->dev, "channel is already %s\n",
			 is_open ? "opened" : "closed");
		goto exit;
	}

	pmss->chan_up = is_open;
	dev_info(pmss->dev, "mlmux channel is %s\n",
		 is_open ? "opened" : "closed");

exit:
	mutex_unlock(&pmss->lock);
}

static void mlmux_pmss_open(struct ml_mux_client *cli, bool is_open)
{
	struct mlmux_pmss_data *pmss;

	pmss = container_of(cli, struct mlmux_pmss_data, client);

	mlmux_pmss_proc_msg(pmss, &is_open, sizeof(is_open),
			    mlmux_pmss_rx_open);
}

static ssize_t mlmux_pmss_show_alert(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct mlmux_pmss_data *pmss = dev_get_drvdata(dev);
	int val = 0;

	dev_dbg(pmss->dev, "show alert %s\n", attr->attr.name);
	if (!strncmp(attr->attr.name, "vcomp_alert", strlen("vcomp_alert"))) {
		val = pmss->vcomp_alert;
	} else if (!strncmp(attr->attr.name, "psy_alert",
				strlen("psy_alert"))) {
		val = pmss->psy_unavailable;
	} else {
		dev_err(pmss->dev, "unknown attribute\n");
		return -EINVAL;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static DEVICE_ATTR(vcomp_alert, S_IRUGO, mlmux_pmss_show_alert, NULL);
static DEVICE_ATTR(psy_alert, S_IRUGO, mlmux_pmss_show_alert, NULL);

static struct attribute *mlmux_pmss_attributes[] = {
	&dev_attr_vcomp_alert.attr,
	&dev_attr_psy_alert.attr,
	NULL,
};

static const struct attribute_group mlmux_pmss_attr_group = {
	.attrs = mlmux_pmss_attributes,
};

static void mlmux_pmss_psy_alert_work(struct work_struct *work)
{
	struct mlmux_pmss_data *pmss;

	pmss = container_of(work, struct mlmux_pmss_data, psy_alert_work);
	dev_err(pmss->dev, "set psy alert\n");
	mutex_lock(&pmss->lock);
	pmss->psy_unavailable = true;
	mutex_unlock(&pmss->lock);
	sysfs_notify(&pmss->dev->kobj, NULL, "psy_alert");
}

static void mlmux_pmss_psy_check_timer_fn(unsigned long data)
{
	struct mlmux_pmss_data *pmss = (struct mlmux_pmss_data *)data;

	dev_dbg(pmss->dev, "psy check timer timed out\n");
	queue_work(pmss->work_q, &pmss->psy_alert_work);
}

static void mlmux_pmss_vcomp_alert_check(struct mlmux_pmss_data *pmss,
					 unsigned long time)
{
	unsigned long *p_irq_time = pmss->vcomp_irq_time;
	int move_size;
	int max_array_ind = ML_PMSS_MAX_ALLOWED_VCOMP_IRQS - 1;
	int i;

	if (pmss->vcomp_alert) {
		dev_info(pmss->dev, "alert already notified\n");
		return;
	}

	if ((time - *p_irq_time) < ML_PMSS_VCOMP_ALERT_WINDOW_S) {
		pmss->vcomp_alert = true;
		dev_err(pmss->dev, "set vcomp alert\n");
		sysfs_notify(&pmss->dev->kobj, NULL, "vcomp_alert");
		return;
	}

	/* Reset the array to start at the irq index
	 * that is within ML_PMSS_VCOMP_ALERT_WINDOW_S.
	 * We already know from the condition above that the 1st (index 0)
	 * irq is outside the window
	 */
	for (i = max_array_ind; i > 0; i--) {
		if (time - *(p_irq_time + i) > ML_PMSS_VCOMP_ALERT_WINDOW_S)
			break;
	}

	move_size = (max_array_ind - i) * sizeof(*p_irq_time);
	memmove(p_irq_time, (p_irq_time + i + 1), move_size);
	*(p_irq_time + max_array_ind - i) = time;
	pmss->vcomp_irq_count = (max_array_ind - i + 1);
}

static irqreturn_t mlmux_pmss_irq_handler(int irq, void *data)
{
	struct mlmux_pmss_data *pmss = (struct mlmux_pmss_data *)data;
	struct timespec irq_time = ktime_to_timespec(ktime_get_boottime());
	unsigned long irq_time_s = irq_time.tv_sec;

	mutex_lock(&pmss->lock);
	pmss->vcomp_irq_count++;
	dev_warn(pmss->dev, "vcomp_irq_count=%d time=%lu\n",
		 pmss->vcomp_irq_count, irq_time_s);
	if (pmss->vcomp_irq_count <= ML_PMSS_MAX_ALLOWED_VCOMP_IRQS)
		pmss->vcomp_irq_time[pmss->vcomp_irq_count - 1] = irq_time_s;
	else
		mlmux_pmss_vcomp_alert_check(pmss, irq_time_s);
	mutex_unlock(&pmss->lock);

	return IRQ_HANDLED;
}


#ifdef CONFIG_DEBUG_FS
static struct dentry *mlmux_pmss_init_debugfs(struct mlmux_pmss_data *pmss,
					       const char *name)
{
	struct dentry *debugfs;
	int ret;

	debugfs = debugfs_create_dir(name, NULL);
	if (IS_ERR(debugfs)) {
		ret = PTR_ERR(debugfs);
		dev_warn(pmss->dev, "Failed to create debugfs dir: %d\n", ret);
		return NULL;
	}
	debugfs_create_u8("manual", S_IRUGO | S_IWUSR, debugfs,
			  (u8 *)&pmss->manual_temp);
	debugfs_create_u16("chrg_temp", S_IRUGO | S_IWUSR, debugfs,
			   &pmss->state.chrg_temp);
	debugfs_create_u16("batt_temp", S_IRUGO | S_IWUSR, debugfs,
			   &pmss->state.batt_temp);

	return debugfs;
}
#else
static struct dentry *mlmux_pmss_init_debugfs(struct mlmux_pmss_data *pmss,
					       const char *name)
{
	return NULL;
}
#endif

static int mlmux_pmss_parse_dt(struct mlmux_pmss_data *pmss)
{
	struct device_node *np = pmss->dev->of_node;
	u32 val;
	s32 sval;
	bool enable;
	int ret;
	enum of_gpio_flags dt_flags = OF_GPIO_ACTIVE_LOW;

	if (!np)
		return -ENODEV;

	if (of_property_read_string(np, "ml,chan-name", &pmss->chan_name)) {
		dev_err(pmss->dev, "ml,chan-name undefined\n");
		return -EINVAL;
	}

	enable = of_property_read_bool(np, "#thermal-sensor-cells");
	if (enable) {
		if (!of_property_read_u32(np, "ml,batt_sens_id", &val))
			pmss->batt_id = val;
		else
			pmss->batt_id = MLMUX_PMSS_BATT_SENS_ID_DEF;

		if (!of_property_read_u32(np, "ml,chrg_sens_id", &val))
			pmss->chrg_id = val;
		else
			pmss->chrg_id = MLMUX_PMSS_CHRG_SENS_ID_DEF;

		if (pmss->batt_id == pmss->chrg_id) {
			dev_err(pmss->dev, "sens_id props are misconfigured\n");
			enable = false;
		}
	}
	pmss->en_therm_sens = enable;

	ret = of_property_read_s32(np, "ml,batt-cold-temp", &sval);
	if (ret) {
		dev_warn(pmss->dev, "Unable to read batt-cold-temp. Use default=%d\n",
				 MLMUX_PMSS_BATT_TEMP_LO_LIM);
		pmss->batt.cold_thresh = MLMUX_PMSS_BATT_TEMP_LO_LIM;
	} else {
		pmss->batt.cold_thresh = (int)sval;
	}

	ret = of_property_read_s32(np, "ml,batt-hot-temp", &sval);
	if (ret) {
		dev_warn(pmss->dev, "Unable to read batt-hot-temp. Use default=%d\n",
				 MLMUX_PMSS_BATT_TEMP_HI_LIM);
		pmss->batt.hot_thresh = MLMUX_PMSS_BATT_TEMP_HI_LIM;
	} else {
		pmss->batt.hot_thresh = (int)sval;
	}

	ret = of_property_read_s32(np, "ml,batt-cool-temp", &sval);
	if (ret)
		pmss->batt.cool_thresh = pmss->batt.cold_thresh;
	else
		pmss->batt.cool_thresh = (int)sval;

	ret = of_property_read_s32(np, "ml,batt-warm-temp", &sval);
	if (ret)
		pmss->batt.warm_thresh = pmss->batt.hot_thresh;
	else
		pmss->batt.warm_thresh = (int)sval;

	pmss->vcomp_alert_gpio = of_get_named_gpio_flags(np,
							"vcomp_alert_irq",
							0, &dt_flags);

	pmss->vcomp_trigger = (dt_flags & OF_GPIO_ACTIVE_LOW) ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

	return 0;
}

static int mlmux_pmss_init_irq(struct mlmux_pmss_data *pmss, const char *name)
{
	int ret = -ENOENT;

	ret = devm_gpio_request_one(pmss->dev, pmss->vcomp_alert_gpio,
				    GPIOF_IN,
				    "vcomp_alert_irq");
	if (ret) {
		dev_err(pmss->dev, "vcomp gpio request failed: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(pmss->dev,
					gpio_to_irq(pmss->vcomp_alert_gpio),
					NULL, mlmux_pmss_irq_handler,
					pmss->vcomp_trigger | IRQF_ONESHOT,
					name, pmss);
	if (ret)
		dev_err(pmss->dev, "failed to set up irq: %d\n", ret);
	else
		enable_irq_wake(gpio_to_irq(pmss->vcomp_alert_gpio));

	return ret;
}

static int mlmux_pmss_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mlmux_pmss_data *pmss = NULL;
	struct timer_list *timer;

	pmss = devm_kzalloc(&pdev->dev, sizeof(*pmss), GFP_KERNEL);
	if (!pmss) {
		ret = -ENOMEM;
		goto exit;
	}

	pmss->dev = &pdev->dev;
	platform_set_drvdata(pdev, pmss);

	ret = mlmux_pmss_parse_dt(pmss);
	if (ret)
		goto exit;

	if (gpio_is_valid(pmss->vcomp_alert_gpio)) {
		ret = mlmux_pmss_init_irq(pmss, pdev->name);
		if (ret)
			goto exit;
	} else {
		dev_warn(pmss->dev, "invalid vcomp_alert_irq GPIO pin!\n");
	}


	pmss->client.dev = pmss->dev;
	pmss->client.notify_open = mlmux_pmss_open;
	pmss->client.receive_cb = mlmux_pmss_recv;

	pmss->otg_edev = extcon_get_edev_by_phandle(pmss->dev, 0);
	if (IS_ERR(pmss->otg_edev)) {
		dev_info(&pdev->dev, "could not get extcon dev\n");
		ret = PTR_ERR(pmss->otg_edev);
		pmss->otg_edev = NULL;
		goto exit;
	}

	ret = mlmux_pmss_reg_usb_cd_extcon(pmss);
	if (ret)
		goto exit;

	init_completion(&pmss->suspend);
	init_completion(&pmss->resume);
	mutex_init(&pmss->lock);
	pmss->work_q = alloc_workqueue(pmss->chan_name, WQ_UNBOUND, 1);
	if (!pmss->work_q) {
		dev_info(pmss->dev, "Failed to create workqueue\n");
		ret = -ENOMEM;
		goto exit_mutex_destroy;
	}

	timer = &pmss->psy_check_timer;
	init_timer(timer);
	timer->function = mlmux_pmss_psy_check_timer_fn;
	timer->data = (unsigned long)pmss;
	INIT_WORK(&pmss->psy_alert_work, mlmux_pmss_psy_alert_work);
	mod_timer(timer, jiffies + ML_PMSS_PSY_CHECK_INIT_TIMER_DELAY);

	if (sysfs_create_group(&pmss->dev->kobj, &mlmux_pmss_attr_group)) {
		dev_err(pmss->dev, "Failed to create sysfs group\n");
		ret = -EINVAL;
		goto exit_wq_destroy;
	}

	ret = ml_mux_request_channel(&pmss->client, (char *)pmss->chan_name);
	if (ret == ML_MUX_CH_REQ_OPENED)
		mlmux_pmss_open(&pmss->client, true);
	else if (ret < 0)
		goto exit_rm_sysfs;

	pmss->debugfs = mlmux_pmss_init_debugfs(pmss, pdev->name);

	return 0;

exit_rm_sysfs:
	sysfs_remove_group(&pmss->dev->kobj, &mlmux_pmss_attr_group);
exit_wq_destroy:
	del_timer_sync(timer);
	destroy_workqueue(pmss->work_q);
exit_mutex_destroy:
	mutex_destroy(&pmss->lock);
	mlmux_pmss_unreg_usb_cd_extcon(pmss);
exit:
	return ret;
}

int mlmux_pmss_remove(struct platform_device *pdev)
{
	struct mlmux_pmss_data *pmss = platform_get_drvdata(pdev);

	dev_dbg(pmss->dev, "removing\n");
	del_timer_sync(&pmss->psy_check_timer);
	mlmux_pmss_unreg_usb_cd_extcon(pmss);
	ml_mux_release_channel(pmss->client.ch);
	sysfs_remove_group(&pmss->dev->kobj, &mlmux_pmss_attr_group);
	destroy_workqueue(pmss->work_q);
	debugfs_remove_recursive(pmss->debugfs);
	mlmux_pmss_unregister_fw(pmss);
	mutex_destroy(&pmss->lock);

	return 0;
}

static void mlmux_pmss_shutdown(struct platform_device *pdev)
{
	struct mlmux_pmss_data *pmss = platform_get_drvdata(pdev);

	dev_info(pmss->dev, "shutting down\n");
	del_timer_sync(&pmss->psy_check_timer);
	mlmux_pmss_unreg_usb_cd_extcon(pmss);
	ml_mux_release_channel(pmss->client.ch);
	destroy_workqueue(pmss->work_q);
	debugfs_remove_recursive(pmss->debugfs);
	mlmux_pmss_unregister_fw(pmss);
	mutex_destroy(&pmss->lock);
}

#ifdef CONFIG_PM_SLEEP
static int mlmux_pmss_suspend(struct device *dev)
{
	int ret;
	unsigned long jiffies_left;
	struct mlmux_pmss_data *pmss = dev_get_drvdata(dev);
	struct mlmux_pmss_tx_ctrl msg;

	reinit_completion(&pmss->suspend);
	msg.cmd = MLMUX_PMSS_TX_SUSPEND;
	msg.data = 1;
	ret = ml_mux_send_msg(pmss->client.ch, sizeof(msg), &msg);
	if (ret)
		return ret;

	jiffies_left = wait_for_completion_timeout(&pmss->suspend,
				       msecs_to_jiffies(MLMUX_PMSS_SUSPEND_TO));
	if (!jiffies_left)
		return -ETIMEDOUT;
	del_timer(&pmss->psy_check_timer);
	dev_dbg(pmss->dev, "suspend\n");

	return 0;
}

static int mlmux_pmss_resume(struct device *dev)
{
	int ret;
	unsigned long jiffies_left;
	struct mlmux_pmss_data *pmss = dev_get_drvdata(dev);
	struct mlmux_pmss_tx_ctrl msg;
	struct timer_list *timer = &pmss->psy_check_timer;

	reinit_completion(&pmss->resume);
	msg.cmd = MLMUX_PMSS_TX_RESUME;
	msg.data = 1;
	ret = ml_mux_send_msg(pmss->client.ch, sizeof(msg), &msg);
	if (ret)
		return ret;

	mod_timer(timer, jiffies + ML_PMSS_PSY_CHECK_TIMER_DELAY);
	jiffies_left = wait_for_completion_timeout(&pmss->resume,
					msecs_to_jiffies(MLMUX_PMSS_RESUME_TO));
	if (!jiffies_left)
		return -ETIMEDOUT;
	dev_dbg(pmss->dev, "resume\n");

	return 0;
}

static const struct dev_pm_ops mlmux_pmss_pm = {
	.suspend = mlmux_pmss_suspend,
	.resume = mlmux_pmss_resume,
};
#define MLMUX_PMSS_PM_OPS	(&mlmux_pmss_pm)
#else
#define MLMUX_PMSS_PM_OPS	NULL
#endif


static const struct of_device_id ml_pmss_match_table[] = {
	{ .compatible = "ml,pmss_ml_mux", },
	{ },
};
MODULE_DEVICE_TABLE(of, ml_pmss_match_table);

static struct platform_driver ml_pmss_driver = {
	.driver = {
		.name = "pmss_ml_mux",
		.of_match_table = ml_pmss_match_table,
		.pm = MLMUX_PMSS_PM_OPS,
	},
	.probe = mlmux_pmss_probe,
	.remove = mlmux_pmss_remove,
	.shutdown = mlmux_pmss_shutdown,
};

module_platform_driver(ml_pmss_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("ML MUX client driver for PMSS");
MODULE_LICENSE("GPL v2");

