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

/*
 *
 *   Magic Leap Security Infrastructure
 *
 *   signature_verification.c - implements the elf signing verification mechanism
 *
 */

#include <linux/ml_sec/signature_verification.h>
#include <linux/ml_sec/common.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/file.h>
#include "../kernel/module-internal.h"
#include <linux/ml_sec/module_tools.h>
#include <crypto/pkcs7.h>
#include <keys/asymmetric-type.h>
#include <linux/slab.h>
#include "../crypto/asymmetric_keys/pkcs7_parser.h"
#include <keys/system_keyring.h>
#include <crypto/public_key.h>

#include "dev/signing_dev.h"

/* ID (serial + issuer) of the ML_INTERNAL key - for pinning */
const u8 ML_INTERNAL_ID[] = {
	0x00, 0x88, 0xf6, 0x82, 0x2a, 0xd9, 0x92, 0x87, 0x04, 0x31, 0x0b, 0x30,
	0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x55, 0x53, 0x31, 0x10,
	0x30, 0x0e, 0x06, 0x03, 0x55, 0x04, 0x08, 0x0c, 0x07, 0x46, 0x6c, 0x6f,
	0x72, 0x69, 0x64, 0x61, 0x31, 0x13, 0x30, 0x11, 0x06, 0x03, 0x55, 0x04,
	0x0a, 0x0c, 0x0a, 0x4d, 0x61, 0x67, 0x69, 0x63, 0x20, 0x4c, 0x65, 0x61,
	0x70, 0x31, 0x1c, 0x30, 0x1a, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0c, 0x13,
	0x4d, 0x61, 0x67, 0x69, 0x63, 0x20, 0x4c, 0x65, 0x61, 0x70, 0x20, 0x49,
	0x6e, 0x74, 0x65, 0x72, 0x6e, 0x61, 0x6c
};

/* TODO: Temporary ID used for Devrel, should be consolidated with original ID
 * when new cert is used */
const u8 ML_INTERNAL_ID_TMP[] = {
	0x00, 0xC6, 0xFE, 0x72, 0x8A, 0xDC, 0x0F, 0xC3, 0x2D, 0x31, 0x13, 0x30,
	0x11, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x0C, 0x0A, 0x4D, 0x61, 0x67, 0x69,
	0x63, 0x20, 0x4C, 0x65, 0x61, 0x70, 0x31, 0x13, 0x30, 0x11, 0x06, 0x03,
	0x55, 0x04, 0x03, 0x0C, 0x0A, 0x4D, 0x61, 0x67, 0x69, 0x63, 0x20, 0x4C,
	0x65, 0x61, 0x70
};

bool sig_should_verify_elf(struct file *file)
{
	if (is_file_on_verified_partition(file))
		/* No need to verify files on a verified partition, they are protected by dm-verity */
		return false;

	return true;
}

void pr_signing_issue(char *issue, struct file *file, int err)
{
	struct task_struct *tsk = current;
	char comm[sizeof(tsk->comm)];
	bool permissive = sig_dev_is_permissive();

	pr_warn("bin_signing: %s pid=%d comm=%s file=%s error=%d permissive=%d\n",
		issue,
		task_pid_nr(tsk),
		get_task_comm(comm, tsk),
		file->f_path.dentry->d_name.name,
		err,
		permissive);
}

int sig_verify_elf_signature(struct file *file)
{
	char *file_buf = NULL;
	off_t  file_len = 0;
	int    err      = -EBADF;
	const unsigned long marker_len = sizeof(MODULE_SIG_STRING) - 1;

	if (sig_dev_is_permissive()) {
		err = 0;
		goto out;
	}

	/* Read the entire file */
	file_len = i_size_read(file_inode(file));
	if (file_len <= marker_len) {
		err = -EBADF;
		pr_signing_issue("module too small", file, err);
		goto out;
	}

	file_buf = vmalloc(file_len); /* Using vmalloc as allocation size may be large */
	if (!file_buf) {
		err = -ENOMEM;
		pr_signing_issue("alloc failure", file, err);
		goto out;
	}

	if (file_len != kernel_read(file, 0, file_buf, file_len)) {
		err = -EIO;
		pr_signing_issue("kernel_read failure", file, err);
		goto out;
	}

	if (memcmp(file_buf + file_len - marker_len, MODULE_SIG_STRING, marker_len) != 0) {
		err = -EBADF;
		pr_signing_issue("module is unsigned", file, err);
		goto out;
	}

	/* Verify the signature */
	file_len -= marker_len; /* We truncate the module to discard the signature */
	err = mod_verify_sig(file_buf, &file_len);
	if (err) {
		pr_signing_issue("mod_verify_sig returned error", file, err);
	}

out:
	if (file_buf != NULL)
		vfree(file_buf);

	return err;
}

bool sig_check_if_ml_internal(struct file *file)
{
	struct pkcs7_message *pkcs7 = 0;
	struct asymmetric_key_id *key_id = NULL;
	u8 *signature_data = NULL;
	size_t overall_size = 0, sig_data_size = 0;
	bool retval = false;

	if (sig_dev_is_permissive())
		return true;

	/* everything on a verified partition is automatically treated as internal */
	if (is_file_on_verified_partition(file))
		return true;

	signature_data = mod_get_module_signature(file, &overall_size, &sig_data_size);
	if (signature_data == NULL) {
		ML_DBG_PRINT("module is unsigned (%s)\n", file->f_path.dentry->d_name.name);
		goto out;
	}

	pkcs7 = pkcs7_parse_message(signature_data, sig_data_size);
	if (IS_ERR(pkcs7))
		goto out;

	if (pkcs7->signed_infos == NULL)
		/* no signer serial number */
		goto out;

	if (pkcs7->signed_infos->next != NULL)
		/* more than one signer infos, definitely not ml_internal */
		goto out;

	key_id = pkcs7->signed_infos->signing_cert_id;
	if (key_id == NULL)
		goto out;

	if (key_id->len != sizeof(ML_INTERNAL_ID)) {
		/* TODO: Consolidate TMP ID with original */
		if (key_id->len != sizeof(ML_INTERNAL_ID_TMP))
			goto out;
	}

	retval = (memcmp(ML_INTERNAL_ID, key_id->data, key_id->len) == 0);
	/* TODO: Consolidate TMP ID with original */
	if (!retval)
		retval = (memcmp(ML_INTERNAL_ID_TMP, key_id->data, key_id->len) == 0);
out:
	if ((!IS_ERR(pkcs7)) && (pkcs7 != NULL))
		pkcs7_free_message(pkcs7);

	if (signature_data != NULL)
		kfree(signature_data);

	return retval;
}

/*
 * ml_sec_verify_key_legality - checks that the added key is not part
 * of the system_trusted_keyring .
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
			       size_t plen)
{
	struct key *system_keyring_equiv_key = NULL;
	struct x509_certificate *x509_cert = NULL;

	if ((payload == NULL) || (plen == 0))
		return 0;

	if ((key_ref_to_ptr(keyring_ref) != system_trusted_keyring) &&
	    (strcmp("asymmetric", type) == 0)) {
		x509_cert = x509_cert_parse(payload, plen);

		if (IS_ERR(x509_cert))
			return -EINVAL;

		system_keyring_equiv_key = x509_request_asymmetric_key(
						       system_trusted_keyring,
						       x509_cert->id, NULL,
						       true);
		x509_free_certificate(x509_cert);
		if (!IS_ERR(system_keyring_equiv_key)) {
			key_put(system_keyring_equiv_key);
			return -EINVAL;
		}
	}
	return 0;
}



#ifdef CONFIG_MLSEC_HARDEN_LOAD_SO

static inline bool parent_dir_equal(struct path *path1, struct path *path2)
{
	return path1->mnt == path2->mnt &&
	       path1->dentry->d_parent == path2->dentry->d_parent;
}

#define SIG_TYPE_INVALID		0
#define SIG_TYPE_ML_INTERNAL		1
#define SIG_TYPE_NON_ML_INTERNAL	2

bool sig_is_load_elf_allowed(struct file *file)
{
	struct mm_struct *mm = current->mm;
	u8 task_signature_type = mm->signature_type;
	struct file *exe_file = get_mm_exe_file(mm);
	bool retval = true;

	/* If this is the actual executable file, or it is ML internal,
	 * then everything is OK. */
	if (exe_file == file || sig_check_if_ml_internal(file))
		goto out;

	/* A lazy approach for setting the task signature type. */
	if (task_signature_type == SIG_TYPE_INVALID) {
		task_signature_type = SIG_TYPE_NON_ML_INTERNAL;
		if (exe_file) {
			if (sig_check_if_ml_internal(exe_file))
				task_signature_type = SIG_TYPE_ML_INTERNAL;
		}
		/* Set the task signature type. */
		mm->signature_type = task_signature_type;
	}

	/* An ml-internal executable cannot load
	 * a non-ml-internal SO! */
	if (task_signature_type == SIG_TYPE_ML_INTERNAL) {
		ML_DBG_PRINT("not allowed to load non-internal SO (%s)\n",
			     file->f_path.dentry->d_name.name);
		retval = false;
		goto out;
	}

	/* A non-ml-internal SO can only get loaded to
	 * an executable from same directory. */
	if (exe_file) {
		retval = parent_dir_equal(&exe_file->f_path, &file->f_path);
		if (!retval)
			ML_DBG_PRINT("SO (%s) must reside in same dir as loading exe (%s)\n",
				     exe_file->f_path.dentry->d_name.name,
				     file->f_path.dentry->d_name.name);
	}

out:
	if (exe_file)
		fput(exe_file);
	return retval;
}

#endif /* CONFIG_MLSEC_HARDEN_LOAD_SO */
