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
#ifndef __MLGZ_OPCODE_H__
#define __MLGZ_OPCODE_H__

enum {
	HOST_RADIO,
} mlgz_dr_opcode;

enum {
	RADIO_ENABLE = 0,		/* Enable radio */
	RADIO_DISABLE = 1,		/* Disable radio */
} mlgz_radio_state;

struct mlgz_radio {
	uint8_t enable;
} __packed;

/* This is the struct passed to the host and retrieved from the host */
struct mlgz_ctrl_command {
	uint16_t opcode;
	uint8_t context;
	union {
		struct mlgz_radio radio;
	};
} __packed;

#endif /*__MLGZ_OPCODE_H__*/
