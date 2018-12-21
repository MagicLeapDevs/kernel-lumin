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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM ml_mux

#if !defined(_TRACE_ML_MUX_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_ML_MUX_H

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(ml_mux_frame,

	TP_PROTO(struct ml_mux_frame *frame),
	TP_ARGS(frame),

	TP_STRUCT__entry(
		__field(uint8_t, channel)
		__field(uint8_t, sequence)
		__field(uint16_t, length)
	),

	TP_fast_assign(
		__entry->channel = frame->channel;
		__entry->sequence = frame->seq_num;
		__entry->length = frame->len;
	),

	TP_printk("chan=%02x seq=%02X length=%X",
		__entry->channel, __entry->sequence, __entry->length)
);

/*
 * tracepoint name	ml_mux:ml_mux_msg_recv
 * description		tracepoint for incoming frames
 */
DEFINE_EVENT(ml_mux_frame, ml_mux_msg_recv,

	TP_PROTO(struct ml_mux_frame *frame),
	TP_ARGS(frame)
);

/*
 * tracepoint name	ml_mux:ml_mux_msg_send
 * description		tracepoint for outgoing frames
 */
DEFINE_EVENT(ml_mux_frame, ml_mux_msg_send,

	TP_PROTO(struct ml_mux_frame *frame),
	TP_ARGS(frame)
);
#endif /* _TRACE_ML_MUX_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE ml_mux_trace
#include <trace/define_trace.h>
