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

/*
 * This driver exposes basic capabilities to power up and down the wearable
 * from userspace, as well as a custom callback for the wearable IRQ.
 */
#ifndef __ML_WEARABLE_H
#define __ML_WEARABLE_H

#define EXTSYNC_EVENT_QUEUE_SIZE 100
struct extsync_event {
	uint64_t ktime;
	uint64_t mtime;
	int state;
	int event_num;
};

struct extsync_event_log_struct {
	int head;
	int tail;
	int event_ctr;
	struct extsync_event *event_list;
};


extern void ml_wearable_register_wearable_irq_custom_callback(
	void (*callback)(void));
extern void ml_wearable_deregister_wearable_irq_custom_callback(void);

extern void ml_wearable_register_extsync_irq_custom_callback(
	int (*callback)(struct extsync_event *));
extern void ml_wearable_deregister_extsync_irq_custom_callback(void);

#endif /* __ML_WEARABLE_H */
