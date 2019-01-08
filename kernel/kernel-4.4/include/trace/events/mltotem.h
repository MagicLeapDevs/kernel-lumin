/*
 * input_mltotem.h
 *
 * Magic Leap Totem Trace Events
 *
 * Copyright (c) 2018, Magic Leap, Inc.  All rights reserved.
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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mltotem

#if !defined(_TRACE_MLTOTEM_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MLTOTEM_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>

TRACE_EVENT(mlt_irq_hard,
	TP_PROTO(int irq),
	TP_ARGS(irq),
	TP_STRUCT__entry(
		__field(int, irq)
	),
	TP_fast_assign(
		__entry->irq = irq;
	),
	TP_printk("irq=%d", __entry->irq)
);

TRACE_EVENT(mlt_send_dof_evt,
	TP_PROTO(u8 caps, u32 timestamp),
	TP_ARGS(caps, timestamp),
	TP_STRUCT__entry(
		__field(u8, caps)
		__field(u32, timestamp)
	),
	TP_fast_assign(
		__entry->caps = caps;
		__entry->timestamp = timestamp;
	),
	TP_printk("caps=%x timestamp=%u",
		__entry->caps, __entry->timestamp)
);

DECLARE_EVENT_CLASS(mlt_input_sync,
	TP_PROTO(u8 caps, char *name),
	TP_ARGS(caps, name),
	TP_STRUCT__entry(
		__field(u8, caps)
		__array(char, name, 32)
	),
	TP_fast_assign(
		__entry->caps = caps;
		strncpy(__entry->name, name, 32);
	),
	TP_printk("caps=%x name=%s",
		__entry->caps, __entry->name)
);

DEFINE_EVENT(mlt_input_sync, mlt_input_sync_imu,
	TP_PROTO(u8 caps, char *name),
	TP_ARGS(caps, name)
);

DEFINE_EVENT(mlt_input_sync, mlt_input_sync_ctrl,
	TP_PROTO(u8 caps, char *name),
	TP_ARGS(caps, name)
);

#endif /*  _TRACE_MLTOTEM_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
