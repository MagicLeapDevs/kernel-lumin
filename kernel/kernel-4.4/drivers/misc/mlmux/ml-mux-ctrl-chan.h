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

#ifndef _ML_MUX_CTRL_CHAN_H
#define _ML_MUX_CTRL_CHAN_H

struct ml_mux;

int ml_mux_init_ctrl_client(struct ml_mux *mux);
void ml_mux_ctrl_chan_open_remote(struct ml_mux *mux,
	struct ml_mux_chan *chan, bool open);

#endif /* _ML_MUX_CTRL_CHAN_H */

