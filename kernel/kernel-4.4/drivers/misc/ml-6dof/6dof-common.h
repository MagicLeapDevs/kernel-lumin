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

#ifndef __ML_6DOF__
#define __ML_6DOF__

#define ML_6DOF_TOTEM_1_ID     1
#define ML_6DOF_TOTEM_2_ID     2
#define ML_6DOF_IS_VALID_TOTEM_ID(t)   ((t) == ML_6DOF_TOTEM_1_ID || \
					(t) == ML_6DOF_TOTEM_2_ID)

enum ml_6dof_axis {
	EMT_AXIS_X = 0,
	EMT_AXIS_Y,
	EMT_AXIS_Z,
	EMT_AXIS_NUM
};

struct ml_6dof_freq {
	u32 x;
	u32 y;
	u32 z;
};

enum ml_6dof_mode {
	EMT_MODE_TDSP,
	EMT_MODE_ADC,
};

const char *emt_mode_to_str(enum ml_6dof_mode mode);

#endif /* __ML_6DOF__ */
