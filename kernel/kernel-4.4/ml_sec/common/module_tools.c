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

#include <linux/ml_sec/module_tools.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <uapi/linux/ml_sec/module_sig_structure.h>
#include <crypto/public_key.h>
#include <linux/slab.h>

loff_t mod_seek_module_signature(struct file *file)
{
	const unsigned long markerlen = sizeof(MODULE_SIG_STRING) - 1;
	char buf[sizeof(struct module_signature) + markerlen];
	struct module_signature *ms;
	loff_t pos, filelen;
	ssize_t read_len;
	size_t sig_len;

	/* find the beginning of the module_signature struct */
	filelen = i_size_read(file_inode(file));
	if (filelen < (loff_t)sizeof(buf))
		return -1;

	pos = filelen - (loff_t)sizeof(buf);
	read_len = kernel_read(file, pos, buf, sizeof(buf));
	if ((read_len < 0) || (read_len != (ssize_t)sizeof(buf)))
		return -1;

	/* verify module signature magic */
	if (memcmp(buf + sizeof(buf) - markerlen, MODULE_SIG_STRING, markerlen) != 0)
		return -1;

	/* skip further back, behind the sigdata + signer name + key id */
	ms = (struct module_signature *)buf;
	sig_len = be32_to_cpu(ms->sig_len);
	if ((loff_t)sig_len >= pos)
		return -1;

	pos -= (loff_t)sig_len;
	if (((loff_t)ms->signer_len + ms->key_id_len) >= pos)
		return -1;

	pos -= (loff_t)ms->signer_len + ms->key_id_len;
	return pos;
}

u8 *mod_get_module_signature(struct file *file, size_t *out_overall_len, size_t *out_sig_len)
{
	const unsigned long markerlen = sizeof(MODULE_SIG_STRING) - 1;
	char buf[sizeof(struct module_signature) + markerlen];
	struct module_signature *sig_struct;
	loff_t pos = 0, filelen = 0;
	ssize_t read_len = 0;
	size_t sig_data_len = 0 ; /* sig_data_len is only the clean signature data above the struct */
	size_t overall_len = 0 ; /* sig_data_len + module_signature length */
	bool success = false;
	u8 *retval = NULL;

	/* seek to the beginning of the module_signature struct */
	filelen = i_size_read(file_inode(file));
	if (filelen < (loff_t)sizeof(buf))
		goto out;

	pos = filelen - (loff_t)sizeof(buf);
	read_len = kernel_read(file, pos, buf, sizeof(buf));
	if (read_len != (ssize_t)sizeof(buf))
		goto out;

	if (memcmp(buf + sizeof(buf) - markerlen, MODULE_SIG_STRING, markerlen) != 0)
		goto out;

	/* read and verify the module_signature struct */
	sig_struct = (struct module_signature *)buf;

	/*
	 * conforming to kernel 4.4 implementation, no field should be set except the id_type
	 * which must be set to PKEY_ID_PKCS7
	 */
	if ((sig_struct->key_id_len != 0) ||
	    (sig_struct->signer_len != 0) ||
	    (sig_struct->hash != 0)       ||
	    (sig_struct->algo != 0)       ||
	    (sig_struct->__pad[0] != 0)   ||
	    (sig_struct->__pad[1] != 0)   ||
	    (sig_struct->__pad[2] != 0)   ||
	    (sig_struct->id_type != PKEY_ID_PKCS7))
		goto out;

	sig_data_len = be32_to_cpu(sig_struct->sig_len);
	if ((loff_t)sig_data_len >= pos)
		goto out;

	pos -= (loff_t)sig_data_len;
	overall_len = sig_data_len + sizeof(struct module_signature);

	/*
	 * till here we have a valid signature at the end of the file, pointed by "pos" variable
	 * now we need to allocate a buffer and read the entire range
	 */
	retval = kmalloc(overall_len, GFP_KERNEL);
	if (retval == NULL)
		goto out;

	read_len = kernel_read(file, pos, retval, overall_len);
	if ((read_len < 0) || (read_len != (ssize_t)overall_len))
		goto out;

	if (out_overall_len != NULL)
		*out_overall_len = overall_len;

	if (out_sig_len != NULL)
		*out_sig_len = sig_data_len;

	success = true;
out:
	if ((!success) && (retval != NULL)) {
		kfree(retval);
		retval = NULL;
	}
	return retval;
}
