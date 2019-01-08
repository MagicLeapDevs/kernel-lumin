/*
 * mltotem.h
 *
 * Magic Leap Totem Input Driver
 *
 * Copyright (c) 2017-2018, Magic Leap, Inc.  All rights reserved.
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

#ifndef __ML_TOTEM_H
#define __ML_TOTEM_H

#include <misc/ml_wearable.h>

void mltotem_register_irq_custom_callback(int (*callback)
					  (struct extsync_event *));
void mltotem_deregister_irq_custom_callback(void);

#endif  /* __ML_TOTEM_H */
