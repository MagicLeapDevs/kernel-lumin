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

#ifndef _ML_SEC_SIGNING_SIGNATURE_VERIFICATION_H_
#define _ML_SEC_SIGNING_SIGNATURE_VERIFICATION_H_

/*
 *   Magic Leap signature verification
 */

#include <linux/fs.h>

/*
 *  The ELF verification API is split into should_verify and verify to allow the flexibility
 *  for the caller to both know that it should verify, and to decide when to do it.
 *
 *  Therefore, should_verify_signature does nothing but just checking criteria to see if
 *  verification needed (not the actual work).
 */

/*
 * sig_should_verify_signature()
 *
 * determines if one should verify a given file's signature or not.
 *
 * returns bool
 */
bool sig_should_verify_elf(struct file *file);


/*
 * sig_verify_elf_signature()
 *
 * performs the actual signature verification of a file.
 *
 * returns ESUCCESS (zero) on success, other code for an error
 */
int sig_verify_elf_signature(struct file *file);


/*
 * sig_check_if_ml_internal()
 *
 * Checks if a file is regarded as an ml "internal" file. Internal processes may receive special permissions.
 * A file is regarded as internal if it resides on a trusted partition (like "system" which is verified by dm-verity)
 * OR if it is signed by the "ml internal" certificate
 *
 * NOTE1: unlike should_verify_signature above, this function checks for a hardcoded certificate that the file uses,
 *        and make sure it is the certificate we use for our ML internal files.
 *
 * NOTE2: this function does not calculate and/or verify the certificate.
 *
 * Returns bool.
 */
#ifdef CONFIG_MLSEC_ELF_SIGNING
bool sig_check_if_ml_internal(struct file *file);
#else
static inline bool sig_check_if_ml_internal(struct file *file) { return true; }
#endif

/*
 * sig_verify_shared_object()
 *
 * Checks if the ELF file, is allowed to be loaded by the current task.
 *
 * Returns bool.
 */
#ifdef CONFIG_MLSEC_HARDEN_LOAD_SO
bool sig_is_load_elf_allowed(struct file *file);
#else
static inline bool sig_is_load_elf_allowed(struct file *file) { return true; }
#endif

#endif /* _ML_SEC_SIGNING_SIGNATURE_VERIFICATION_H_ */
