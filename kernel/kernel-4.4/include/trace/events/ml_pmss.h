/* Copyright (c) 2018, Magic Leap, Inc. All rights reserved.
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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM ml_pmss

#if !defined(_TRACE_ML_PMSS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_ML_PMSS_H

#include <linux/tracepoint.h>

#define psy_type_name(psy) { POWER_SUPPLY_TYPE_##psy, #psy }
#define show_psy_type(type)					\
	__print_symbolic(type,					\
			 psy_type_name(UNKNOWN),		\
			 psy_type_name(BATTERY),		\
			 psy_type_name(UPS),			\
			 psy_type_name(MAINS),			\
			 psy_type_name(USB),			\
			 psy_type_name(USB_DCP),		\
			 psy_type_name(USB_CDP),		\
			 psy_type_name(USB_ACA),		\
			 psy_type_name(USB_TYPE_C),		\
			 psy_type_name(USB_PD),			\
			 psy_type_name(USB_PD_DRP))

DECLARE_EVENT_CLASS(ml_pmss_state,

	TP_PROTO(struct mlmux_pmss_state *state),
	TP_ARGS(state),

	TP_STRUCT__entry(
		__field(uint8_t, soc)
		__field(uint8_t, cable_conn)
		__field(uint8_t, cable_orient)
		__field(uint8_t, req_shutdown)
		__field(uint8_t, psy_type)
		__field(uint8_t, chrg_en)
		__field(uint8_t, is_dfp)
		__field(uint8_t, batt_present)
		__field(uint8_t, chrg_state)
		__field(uint8_t, chrg_cap_mismatch)
		__field(uint8_t, chrg_insufficient)
		__field(uint8_t, batt_auth)
		__field(int16_t, vbat)
		__field(int16_t, vbat_adc)
		__field(int16_t, vbat_max)
		__field(int16_t, vbus)
		__field(int16_t, adpt_curr)
		__field(int16_t, batt_curr)
		__field(int16_t, chrg_curr_max)
		__field(int16_t, batt_temp)
		__field(int16_t, chrg_temp)
	),

	TP_fast_assign(
		__entry->cable_conn = state->cable_conn;
		__entry->cable_orient = state->cable_orient;
		__entry->req_shutdown = state->req_shutdown;
		__entry->psy_type = state->psy_type;
		__entry->chrg_en = state->chrg_en;
		__entry->is_dfp = state->is_dfp;
		__entry->batt_present = state->batt_present;
		__entry->chrg_state = state->chrg_state;
		__entry->chrg_cap_mismatch = state->chrg_cap_mismatch;
		__entry->chrg_insufficient = state->chrg_insufficient;
		__entry->batt_auth = state->batt_auth;
		__entry->vbat = state->vbat;
		__entry->vbat_adc = state->vbat_adc;
		__entry->vbat_max = state->vbat_max;
		__entry->vbus = state->vbus;
		__entry->adpt_curr = state->adpt_curr;
		__entry->batt_curr = state->batt_curr;
		__entry->chrg_curr_max = state->chrg_curr_max;
		__entry->batt_temp = state->batt_temp;
		__entry->chrg_temp = state->chrg_temp;
	),

	TP_printk("\ncable_conn=%hhd\n"
		  "cable_orient=%hhd\n"
		  "req_shutdown=%hhd\n"
		  "psy_type=%s\n"
		  "chrg_en=%hhd\n"
		  "is_partner_dfp=%hhd\n"
		  "batt_present=%hhd\n"
		  "chrg_state=%hhd\n"
		  "cap_mismatch=%hhd\n"
		  "chrg_insufficient=%hhd\n"
		  "batt_authentic=%hhd\n"
		  "vbat_fg=%hd\n"
		  "vbat_adc=%hd\n"
		  "vbat_max=%hd\n"
		  "vbus=%hd\n"
		  "adpt_curr=%hd\n"
		  "batt_curr=%hd\n"
		  "max_charge_curr=%hd\n"
		  "batt_temp=%hd\n"
		  "chrg_temp=%hd",
		  __entry->cable_conn,
		  __entry->cable_orient,
		  __entry->req_shutdown,
		  show_psy_type(__entry->psy_type),
		  __entry->chrg_en,
		  __entry->is_dfp,
		  __entry->batt_present,
		  __entry->chrg_state,
		  __entry->chrg_cap_mismatch,
		  __entry->chrg_insufficient,
		  __entry->batt_auth,
		  __entry->vbat,
		  __entry->vbat_adc,
		  __entry->vbat_max,
		  __entry->vbus,
		  __entry->adpt_curr,
		  __entry->batt_curr,
		  __entry->chrg_curr_max,
		  __entry->batt_temp,
		  __entry->chrg_temp)
);

/*
 * tracepoint name	ml_pmss: ml_pmss_state
 * description		tracepoint for state info recv
 */
DEFINE_EVENT(ml_pmss_state, ml_pmss_state,

	TP_PROTO(struct mlmux_pmss_state *state),
	TP_ARGS(state)
);

DECLARE_EVENT_CLASS(ml_pmss_state_2,

	TP_PROTO(struct mlmux_pmss_state_2 *state_2),
	TP_ARGS(state_2),

	TP_STRUCT__entry(
		__field(uint16_t, full_charge_cap)
		__field(uint16_t, adpt_max_mA)
		__field(uint16_t, adpt_mV)
	),

	TP_fast_assign(
		__entry->full_charge_cap = state_2->full_charge_cap;
		__entry->adpt_max_mA = state_2->adpt_max_mA;
		__entry->adpt_mV = state_2->adpt_mV;
	),

	TP_printk("\nfull_chrg_capacity=%hd\n"
		  "adpt_max_curr=%hd\n"
		  "adpt_max_volt=%hd",
		  __entry->full_charge_cap,
		  __entry->adpt_max_mA,
		  __entry->adpt_mV)
);

/*
 * tracepoint name	ml_pmss: ml_pmss_state_2
 * description		tracepoint for state_2 info recv
 */
DEFINE_EVENT(ml_pmss_state_2, ml_pmss_state_2,

	TP_PROTO(struct mlmux_pmss_state_2 *state_2),
	TP_ARGS(state_2)
);
#endif /* _TRACE_ML_PMSS_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
