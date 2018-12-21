/*
 * Copyright (c) 2017, Magic Leap, Inc. All rights reserved.
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

#ifndef __MLSEC_SBOX_SANDBOX_FILTERS_H
#define __MLSEC_SBOX_SANDBOX_FILTERS_H


enum {
#define _SANDBOX_FILTER(name, str)	_SANDBOX_SHIFT_FILTER_##name,
#include "sandbox_filters.def"
};

enum {
#define _SANDBOX_FILTER(name, str)	SANDBOX_FILTER_##name = (1 << _SANDBOX_SHIFT_FILTER_##name),
#include "sandbox_filters.def"
};

#endif	/* __MLSEC_SBOX_SANDBOX_FILTERS_H */
