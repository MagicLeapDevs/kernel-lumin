/*
 * Tegra CSI4 device common APIs
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frankc@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CSI4_H__
#define __CSI4_H__
#define TEGRA_CSICIL_CLK_MHZ 204

#include "csi.h"

int csi4_start_streaming(struct tegra_csi_channel *chacsin,
		enum tegra_csi_port_num port_num);
void csi4_stop_streaming(struct tegra_csi_channel *chan,
		enum tegra_csi_port_num port_num);


extern struct tegra_csi_fops csi4_fops;

#endif
