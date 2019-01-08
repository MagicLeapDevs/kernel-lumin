/*
 * linux/fs/ext4/ext4_crypto.h
 *
 * Copyright (C) 2015, Google, Inc.
 * Copyright (C) 2018, Magic Leap, Inc. All rights reserved.
 *
 * This contains encryption header content for ext4
 *
 * Written by Michael Halcrow, 2015.
 */
#ifndef _EXT4_CRYPTO_PUBLIC_H
#define _EXT4_CRYPTO_PUBLIC_H

#define EXT4_KEY_DESCRIPTOR_SIZE 8

#define EXT4_ENCRYPTION_CONTEXT_FORMAT_V1 1
#define EXT4_KEY_DERIVATION_NONCE_SIZE 16

#define EXT4_POLICY_FLAGS_PAD_4		0x00
#define EXT4_POLICY_FLAGS_PAD_8		0x01
#define EXT4_POLICY_FLAGS_PAD_16	0x02
#define EXT4_POLICY_FLAGS_PAD_32	0x03
#define EXT4_POLICY_FLAGS_PAD_MASK	0x03
#define EXT4_POLICY_FLAGS_VALID		0x03

/* Encryption parameters */
#define EXT4_XTS_TWEAK_SIZE 16
#define EXT4_AES_128_ECB_KEY_SIZE 16
#define EXT4_AES_256_GCM_KEY_SIZE 32
#define EXT4_AES_256_CBC_KEY_SIZE 32
#define EXT4_AES_256_CTS_KEY_SIZE 32
#define EXT4_AES_256_XTS_KEY_SIZE 64
#define EXT4_MAX_KEY_SIZE 64

#define EXT4_KEY_DESC_PREFIX "ext4:"
#define EXT4_KEY_DESC_PREFIX_SIZE 5

extern char global_fbe_key[EXT4_MAX_KEY_SIZE];
extern char global_fbe_desc[EXT4_KEY_DESC_PREFIX_SIZE +
				 (EXT4_KEY_DESCRIPTOR_SIZE * 2) + 1];
extern bool global_fbe_added;

#endif	/* _EXT4_CRYPTO_PUBLIC_H */
