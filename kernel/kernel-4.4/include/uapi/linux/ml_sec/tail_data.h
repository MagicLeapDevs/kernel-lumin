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

#ifndef __MLSEC_TAIL_DATA_UAPI_H
#define __MLSEC_TAIL_DATA_UAPI_H

#define ML_TAIL_DATA_MAGIC_STRING "TAILDATA"
#define ML_TAIL_DATA_LATEST_VERSION (3)
#define ML_TAIL_DATA_SEINFO_MAXSIZE (64)
#define ML_TAIL_DATA_SERIAL_MINSIZE (12)
#define ML_TAIL_DATA_SERIAL_MAXSIZE (32)

#pragma pack(push, 1)
struct ml_tail_data_ctrl {
	/* Version of the ml_tail_data structure */
	uint8_t taildata_ver;

	/* Magic qword set to 'TAILDATA', to identify the ml_tail_data struct */
	uint64_t tail_data_magic;
};

struct ml_tail_data {
	/* Optional - length of attached developer certificate (cert behind this structure) */
	uint32_t devcert_len;

	/* ProfileTag that represents the sandbox profile to apply */
	uint32_t profile_tag;

	/* Flag that specifies if executable's process is debuggable  */
	uint8_t is_debuggable;

	/* Guardian's sentinel type */
	uint8_t sentinel_type;

	/* Guardian's sentinel version */
	uint16_t sentinel_version;

	/* Generic ID field, currently used for watermark */
	uint32_t id;

	union {
		/* In order to avoid a tail data version change for the device-specific sentinel feature, */
		/* we re-purposed the seinfo field as a union that may hold the serial instead. */
		/* That's possible since seinfo isn't applicable for sentinel images. */

		/* String that specifies the SELinux context to apply  */
		char seinfo[ML_TAIL_DATA_SEINFO_MAXSIZE];

		/* Powerpack's Serial. Used to generate device-specific sentinel images */
		char serial[ML_TAIL_DATA_SERIAL_MAXSIZE];
	};

	/* Tail data control fields that are used to verify the structure */
	struct ml_tail_data_ctrl ctrl;
};
#pragma pack(pop)
#endif
