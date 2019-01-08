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

#ifndef __MODULE_TOOLS_H__
#define __MODULE_TOOLS_H__

#include <linux/fs.h>
#include <linux/module.h>

#ifdef CONFIG_MLSEC_COMMON
/*
 * mod_get_module_signature
 *
 * allocates a buffer and extracts the signature data and the signature struct.
 *
 * CALLER MUST FREE THE BUFFER!! using kfree().
 *
 * out_overall_len - the overall length of the returned buffer
 * out_sig_len - the length of the signature data at the beginning of the buffer,
 *               this also indicates the offset of the signature struct inside the buffer.
 *
 * returns the signature buffer pointer or NULL if failed.
 *
 */
u8 *mod_get_module_signature(struct file *file, size_t *out_overall_len, size_t *out_sig_len);

/*
 * mod_seek_module_signature
 *
 * this is a utility func.
 *
 * finds the file offset of the beginning of the signature data
 * this practically points to the signer's name.
 * rest of the data is described in the module_signature struct
 *
 * if signature wasn't found, returns -1
 *
 */

loff_t mod_seek_module_signature(struct file *file);

#else
static inline u8 *mod_get_module_signature(struct file *file, size_t *out_overall_len, size_t *out_sig_len) { return NULL; }
static inline loff_t mod_seek_module_signature(struct file *file) { return -1; }
#endif

#endif	/* __MODULE_TOOLS_H__ */
