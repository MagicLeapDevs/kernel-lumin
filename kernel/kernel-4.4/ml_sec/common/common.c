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
 *   Magic Leap Security Infrastructure
 *
 *   common.c - implements the ml_sec common util functions.
 */

#include <linux/kernel.h>
#include <linux/mount.h>
#include <linux/ml_sec/common.h>
#include <linux/of.h>

static atomic64_t atomic_verified_sb[] = {
	ATOMIC64_INIT(0), /* /system */
	ATOMIC64_INIT(0)  /* /vendor */
};

#ifdef CONFIG_MLSEC_CHECK_VERITY
#include <linux/device-mapper.h>
#include "../../drivers/md/dm.h"

/*
 * Check if verity is marked as "unavailable" in the DTB
 * This is a hack needed in P0 where verity doesn't work
 */
static bool is_verity_unavailable_p0(void)
{
	struct device_node *chosen_node = of_find_node_by_path("/chosen");
	struct property *is_verity_unavailable = NULL;

	if (chosen_node) {
		is_verity_unavailable = of_find_property(chosen_node,
							"ml,disable-verity-check",
							NULL);
		of_node_put(chosen_node);
		if (is_verity_unavailable)
			return true;
	}

	return false;
};

/*
 * Check if the super_block has dm-verity enabled on it.
 */
static bool is_verified_partition_secure(const struct super_block *sb)
{
	bool retval = false;
	struct mapped_device *md = dm_get_md(sb->s_dev);

	if (md) {
		int srcu_idx;
		struct dm_table *table = dm_get_live_table(md, &srcu_idx);
		if (table) {
			unsigned int num = dm_table_get_num_targets(table);
			unsigned int i;
			for (i = 0; i < num; ++i) {
				struct dm_target *target = dm_table_get_target(table, i);
				if (target && !strcmp("verity", target->type->name)) {
					retval = true;
					break;
				}
			}

			dm_put_live_table(md, srcu_idx);
		}
		dm_put(md);
	}
	return retval;
}

#endif /* !CONFIG_MLSEC_CHECK_VERITY */
#include "../../fs/mount.h"

/*
 * This check is NOT secure, but is needed on debug builds since dm-verity
 * might be disabled. It would also be needed on targets that do not support
 * dm-verity (i.e., emmc based).
 */
static bool is_verified_partition_unsecure(const struct vfsmount *vfsmnt)
{
	const struct mount *mnt;
	const struct dentry *mountpoint;
	const char *mnt_name;

	if (!vfsmnt)
		return false;

	mnt = real_mount((struct vfsmount *)vfsmnt);
	mountpoint = mnt->mnt_mountpoint;
	if (!mountpoint)
		return false;

	mnt_name = mountpoint->d_name.name;
#define MOUNT_NAME_EQUALS(str) (!strncmp(mnt_name, str, strlen(str)))

	return MOUNT_NAME_EQUALS("system") ||
	       MOUNT_NAME_EQUALS("vendor");
}

bool is_file_on_verified_partition(const struct file *file)
{
	const struct vfsmount *mnt = file->f_path.mnt;
	const struct super_block *sb = mnt->mnt_sb;
	int i;

	for (i = 0; i < ARRAY_SIZE(atomic_verified_sb); ++i) {
		const struct super_block *verified_sb = (struct super_block *)atomic64_read(&atomic_verified_sb[i]);
		if (sb == verified_sb)
			return true;

		/*
		 * If we haven't found a matching verified super_block, while there is a free slot in the array,
		 * we try to verify the file's super_block directly, and update the array if we succeed.
		 */
		if (!verified_sb) {
#ifdef CONFIG_MLSEC_CHECK_VERITY
			bool emmc = is_verity_unavailable_p0();

			if ((emmc && is_verified_partition_unsecure(mnt)) ||
			    is_verified_partition_secure(sb)) {
#else
			if (is_verified_partition_unsecure(mnt)) {
#endif
				/*
				 * If the array has changed in the meanwhile, then do not update it.
				 */
				atomic64_cmpxchg(&atomic_verified_sb[i], 0, (long)sb);
				return true;
			}
		}
	}
	return false;
}
