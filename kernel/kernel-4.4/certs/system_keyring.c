/* System trusted keyring for trusted public keys
 *
 * Copyright (C) 2012 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version
 * 2 of the Licence, or (at your option) any later version.
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cred.h>
#include <linux/err.h>
#include <keys/asymmetric-type.h>
#include <keys/system_keyring.h>
#include <crypto/pkcs7.h>
#include <linux/fs.h>
#include <linux/slab.h>

struct key *system_trusted_keyring;
EXPORT_SYMBOL_GPL(system_trusted_keyring);

struct key *devcerts_keyring; /* developer certificates keyring */

extern __initconst const u8 system_certificate_list[];
extern __initconst const unsigned long system_certificate_list_size;

/*
 * Load the compiled-in keys
 */
static __init int system_trusted_keyring_init(void)
{
	pr_notice("Initialise system trusted keyring\n");

	system_trusted_keyring =
		keyring_alloc(".system_keyring",
			      KUIDT_INIT(0), KGIDT_INIT(0), current_cred(),
			      ((KEY_POS_ALL & ~KEY_POS_SETATTR) |
			      KEY_USR_VIEW | KEY_USR_READ | KEY_USR_SEARCH),
			      KEY_ALLOC_NOT_IN_QUOTA, NULL);
	if (IS_ERR(system_trusted_keyring))
		panic("Can't allocate system trusted keyring\n");

	set_bit(KEY_FLAG_TRUSTED_ONLY, &system_trusted_keyring->flags);

	pr_notice("Initializing developer certificates keyring\n");

	devcerts_keyring =
		keyring_alloc(".devcerts_keyring",
			      KUIDT_INIT(0), KGIDT_INIT(0), current_cred(),
			      ((KEY_POS_ALL & ~KEY_POS_SETATTR) |
			      (KEY_USR_ALL & ~KEY_USR_SETATTR) |
			      (KEY_OTH_ALL & ~KEY_OTH_SETATTR)),
			      KEY_ALLOC_NOT_IN_QUOTA, NULL);
	if (IS_ERR(devcerts_keyring))
		panic("Can't allocate developers certificate keyring\n");

	set_bit(KEY_FLAG_TRUSTED_ONLY, &devcerts_keyring->flags);
	return 0;
}

/*
 * Must be initialised before we try and load the keys into the keyring.
 */
device_initcall(system_trusted_keyring_init);


/*
 * Read the built-in list of X.509 certificates.
 * CALLER MUST FREE *out_list_buf
 */
static int read_system_certificate_list(u8 **out_list_buf, loff_t *out_list_size)
{
	struct file *list_file;
	u8 *list_buf;
	loff_t list_size;
	int error;

	list_file = filp_open("/system_certificate_list", O_RDONLY, 0);
	if (IS_ERR(list_file))
		return PTR_ERR(list_file);

	list_size = i_size_read(file_inode(list_file));

	list_buf = kmalloc(list_size, GFP_KERNEL);
	if (list_buf == NULL) {
		filp_close(list_file, NULL);
		return -ENOMEM;
	}

	error = kernel_read(list_file, 0, (char *)list_buf, list_size);
	if (error < 0) {
		kfree(list_buf);
		filp_close(list_file, NULL);
		return error;
	}

	filp_close(list_file, NULL);
	*out_list_buf = list_buf;
	*out_list_size = list_size;
	return 0;
}

/*
 * Load the compiled-in list of X.509 certificates.
 */
static __init int load_system_certificate_list(void)
{
	key_ref_t key;
	const u8 *p, *end;
	size_t plen;
	int err;

	u8 *system_certificate_list;
	loff_t system_certificate_list_size;

	pr_notice("Loading compiled-in X.509 certificates\n");

	err = read_system_certificate_list(&system_certificate_list, &system_certificate_list_size);
	if (err < 0) {
		pr_err("Problem reading system certificate list file\n");
		return err;
	}

	p = system_certificate_list;
	end = p + system_certificate_list_size;
	while (p < end) {
		/* Each cert begins with an ASN.1 SEQUENCE tag and must be more
		 * than 256 bytes in size.
		 */
		if (end - p < 4)
			goto dodgy_cert;
		if (p[0] != 0x30 &&
		    p[1] != 0x82)
			goto dodgy_cert;
		plen = (p[2] << 8) | p[3];
		plen += 4;
		if (plen > end - p)
			goto dodgy_cert;

		key = key_create_or_update(make_key_ref(system_trusted_keyring, 1),
					   "asymmetric",
					   NULL,
					   p,
					   plen,
					   ((KEY_POS_ALL & ~KEY_POS_SETATTR) |
					   KEY_USR_VIEW | KEY_USR_READ),
					   KEY_ALLOC_NOT_IN_QUOTA |
					   KEY_ALLOC_TRUSTED);
		if (IS_ERR(key)) {
			pr_err("Problem loading in-kernel X.509 certificate (%ld)\n",
			       PTR_ERR(key));
		} else {
			set_bit(KEY_FLAG_BUILTIN, &key_ref_to_ptr(key)->flags);
			pr_notice("Loaded X.509 cert '%s'\n",
				  key_ref_to_ptr(key)->description);
			key_ref_put(key);
		}
		p += plen;
	}
	kfree(system_certificate_list);

	return 0;

dodgy_cert:
	kfree(system_certificate_list);
	pr_err("Problem parsing in-kernel X.509 certificate list\n");
	return 0;
}
late_initcall(load_system_certificate_list);

#ifdef CONFIG_SYSTEM_DATA_VERIFICATION

/**
 * Verify a PKCS#7-based signature on system data.
 * @data: The data to be verified.
 * @len: Size of @data.
 * @raw_pkcs7: The PKCS#7 message that is the signature.
 * @pkcs7_len: The size of @raw_pkcs7.
 * @usage: The use to which the key is being put.
 */
int system_verify_data(const void *data, unsigned long len,
		       const void *raw_pkcs7, size_t pkcs7_len,
		       enum key_being_used_for usage)
{
	struct pkcs7_message *pkcs7;
	bool trusted;
	int ret;

	pkcs7 = pkcs7_parse_message(raw_pkcs7, pkcs7_len);
	if (IS_ERR(pkcs7))
		return PTR_ERR(pkcs7);

	/* The data should be detached - so we need to supply it. */
	if (pkcs7_supply_detached_data(pkcs7, data, len) < 0) {
		pr_err("PKCS#7 signature with non-detached data\n");
		ret = -EBADMSG;
		goto error;
	}

	ret = pkcs7_verify(pkcs7, usage);
	if (ret < 0)
		goto error;

	ret = pkcs7_validate_trust(pkcs7, system_trusted_keyring, &trusted);
	if (ret < 0) {
		/*
		 * If verification through the system_trusted_keyring fails,
		 * we try to verify through the devcerts_keyring whose keys
		 * verify binaries' embedded signature signed locally
		 * by external developers at dev time.
		 */
		ret = pkcs7_validate_trust(pkcs7, devcerts_keyring, &trusted);
		if (ret < 0)
			goto error;
	}

	if (!trusted) {
		pr_err("PKCS#7 signature not signed with a trusted key\n");
		ret = -ENOKEY;
	}

error:
	pkcs7_free_message(pkcs7);
	pr_devel("<==%s() = %d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(system_verify_data);

#endif /* CONFIG_SYSTEM_DATA_VERIFICATION */
