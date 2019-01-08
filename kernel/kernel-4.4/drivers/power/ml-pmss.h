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

#ifndef _ML_MUX_PMSS_H
#define _ML_MUX_PMSS_H

#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/power_supply.h>
#include <linux/mutex.h>
#include <linux/thermal.h>
#include <linux/extcon.h>
#include <linux/types.h>
#include <linux/ml-mux-client.h>

#define MLMUX_PMSS_BATT_SENS_ID_DEF	(0)
#define MLMUX_PMSS_CHRG_SENS_ID_DEF	(1)
#define MLMUX_PMSS_BATT_TEMP_HI_LIM	(550)
#define MLMUX_PMSS_BATT_TEMP_LO_LIM	(50)
#define ML_PMSS_MAX_ALLOWED_VCOMP_IRQS (2)

struct mlmux_pmss_work {
	struct work_struct work;
	struct mlmux_pmss_data *pmss;
	void *data;
};

struct mlmux_pmss_init {
	uint32_t batt_id_res;
	uint16_t batt_capacity;
} __packed;

struct mlmux_pmss_state {
	uint8_t soc;
	uint8_t cable_conn : 1;
	uint8_t cable_orient : 1;
	uint8_t req_shutdown : 1;
	uint8_t psy_type : 4;
	uint8_t chrg_en : 1;
	uint8_t is_dfp : 1;
	uint8_t batt_present : 1;
	uint8_t chrg_state : 3;
	uint8_t chrg_cap_mismatch : 1;
	uint8_t chrg_insufficient : 1;
	uint8_t batt_auth : 1;
	int16_t vbat;
	int16_t vbat_adc;
	int16_t vbat_max;
	int16_t vbus;
	int16_t adpt_curr;
	int16_t batt_curr;
	int16_t chrg_curr_max;
	int16_t batt_temp;
	int16_t chrg_temp;
} __packed;

struct mlmux_pmss_state_2 {
	uint32_t full_charge_cap : 16;
	uint32_t adpt_max_mA : 16;
	uint32_t adpt_mV : 16;
	uint32_t chrgr_too_hot : 1;
	uint32_t rsvd5 : 15;
	uint32_t rsvd4;
	uint32_t rsvd3;
	uint32_t rsvd2;
	uint32_t rsvd1;
} __packed;

struct mlmux_pmss_rx_pkt {
	uint8_t type;
	union {
		int data;
		struct mlmux_pmss_state state;
		struct mlmux_pmss_state_2 state_2;
		struct mlmux_pmss_init init;
	} __packed u;
} __packed;
#define MLMUX_PMSS_RX_SIZE(msg) \
	(sizeof(struct mlmux_pmss_rx_pkt) - \
	sizeof(((struct mlmux_pmss_rx_pkt *)0)->u) + \
	sizeof(((struct mlmux_pmss_rx_pkt *)0)->u.msg))

enum mlmux_pmss_rx_type {
	MLMUX_PMSS_RX_INIT,
	MLMUX_PMSS_RX_STATE,
	MLMUX_PMSS_RX_SUSPEND_ACK,
	MLMUX_PMSS_RX_RESUME_ACK,
	MLMUX_PMSS_RX_STATE_2,
};

enum mlmux_pmss_tx_type {
	MLMUX_PMSS_TX_ALGO_EN,
	MLMUX_PMSS_TX_CHRG_CURR,
	MLMUX_PMSS_TX_VSYS,
	MLMUX_PMSS_TX_ADPT_CURR,
	MLMUX_PMSS_TX_UPDATE_NOW,
	MLMUX_PMSS_TX_UPDATE_RATE,
	MLMUX_PMSS_TX_SUSPEND,
	MLMUX_PMSS_TX_RESUME,
	MLMUX_PMSS_TX_ERASE_PD_CTRL,
	MLMUX_PMSS_TX_BC12_NOTIFY,
};

struct mlmux_pmss_tx_ctrl {
	uint8_t cmd;
	uint16_t data;
} __packed;

struct mlmux_pmss_batt {
	int cool_thresh;
	int cold_thresh;
	int warm_thresh;
	int hot_thresh;
};

struct mlmux_pmss_data {
	struct device *dev;
	struct ml_mux_client client;
	const char *chan_name;
	struct workqueue_struct *work_q;
	struct mutex lock;
	struct completion suspend;
	struct completion resume;
	bool chan_up;

	struct mlmux_pmss_init init_params;
	struct mlmux_pmss_state state;
	struct mlmux_pmss_state_2 state_2;

	struct power_supply *batt_psy;
	struct power_supply *usb_psy;
	struct power_supply_desc batt_desc;
	struct power_supply_desc usb_desc;
	struct extcon_dev *otg_edev;
	struct extcon_dev *usb_cd_edev;
	struct notifier_block usb_cd_nb;
	struct work_struct usb_cd_work;

	struct thermal_zone_device *batt_therm;
	struct thermal_zone_device *chrg_therm;
	bool en_therm_sens;
	int batt_id;
	int chrg_id;
	struct mlmux_pmss_batt batt;

	bool psy_unavailable;
	struct work_struct psy_alert_work;
	struct timer_list psy_check_timer;

	struct dentry *debugfs;
	bool manual_temp;

	bool vcomp_alert;
	int vcomp_alert_gpio; /* low voltage comp interrupt */
	int vcomp_trigger;
	int vcomp_irq_count; /* set in vcomp_alert irq */
	unsigned long vcomp_irq_time[ML_PMSS_MAX_ALLOWED_VCOMP_IRQS];
};

#endif /* _ML_MUX_PMSS_H */
