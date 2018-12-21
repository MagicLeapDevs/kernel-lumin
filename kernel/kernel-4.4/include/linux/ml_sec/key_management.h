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

#ifndef _ML_SEC_SIGNING_KEY_MANAGEMENT_H_
#define _ML_SEC_SIGNING_KEY_MANAGEMENT_H_

/*
 *   Magic Leap key management security features
 */

#include <linux/key.h>

#ifdef CONFIG_MLSEC_ELF_SIGNING
/*
 * ml_sec_verify_key_legality - checks that the added key is not part
 * of the system_trusted_keyring.
 *
 * In practice: Checks that the added key is not a mock (or even
 * equivalent) of an already existing pre-installed key in the
 * system_trusted_keyring.
 *
 * @keyring_ref: A pointer to the destination keyring with possession flag.
 * @type: The type of key.
 * @description: The searchable description for the key.
 * @payload: The data to use to instantiate or update the key.
 * @plen: The length of @payload.
 *
 * Returns 0 on success or an error code
 */
int ml_sec_verify_key_legality(key_ref_t keyring_ref,
			       const char *type,
			       const char *description,
			       const void *payload,
			       size_t plen);
#else
static inline int ml_sec_verify_key_legality(key_ref_t keyring_ref,
			                     const char *type,
		                             const char *description,
		                             const void *payload,
		                             size_t plen) { return 0; }
#endif

#endif /* _ML_SEC_SIGNING_KEY_MANAGEMENT_H_ */
