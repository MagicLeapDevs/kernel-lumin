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

#include "mlgz_buffer.h"
#include <linux/slab.h>
#include <linux/mutex.h>

#define MAX_BUFFERS_ALLOCATED (64)
DEFINE_MUTEX(buffer_mutex);

struct mlgz_buffer {
	char used;
	uint8_t data[MLGZ_CTRL_CMD_MAX_BUFFER_SIZE];
};

static struct mlgz_buffer buffers[MAX_BUFFERS_ALLOCATED];

uint8_t *mlgz_buffer_alloc(void)
{
	int i;
	struct mlgz_buffer *buffer = NULL;

	mutex_lock(&buffer_mutex);
	for (i = 0; i < MAX_BUFFERS_ALLOCATED; i++) {
		if (!buffers[i].used) {
			buffers[i].used = 1;
			buffer = &buffers[i];
			break;
		}
	}

	mutex_unlock(&buffer_mutex);

	return buffer ? buffer->data : NULL;
}

void mlgz_ctrl_command_buffer_free(uint8_t *buffer)
{
	struct mlgz_buffer *buffer_container;

	/* Sanity check: make sure the buffer pointer is in our range
	* Our range is defined at the start by the 'buffers' array
	* And at the end by buffers + the number of structs we have
	* allocated
	*/
	if (((void *)buffer < (void *)buffers)
		|| ((void *)buffer >=
		(void *)(buffers + sizeof(struct mlgz_buffer)
				* MAX_BUFFERS_ALLOCATED)))
		return;

	mutex_lock(&buffer_mutex);
	buffer_container = (struct mlgz_buffer *)((uintptr_t)buffer
					- offsetof(struct mlgz_buffer, data));

	buffer_container->used = 0;
	mutex_unlock(&buffer_mutex);
}
