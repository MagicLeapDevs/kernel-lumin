/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/export.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <video/tegra_dc_ext.h>
#include <trace/events/display.h>

#include "dc.h"
#include "dc_priv.h"
#include "dc_config.h"
#include "dev.h"
#include "sfd_priv.h"
#include "sfd_t186.h"

dev_t sfd_devno;
struct class *sfd_class;
static int head_count;
static atomic_t dc_open_count;

int tegra_dc_ext_get_num_outputs(void)
{
	/* TODO: decouple output count from head count */
	return head_count;
}

static int dev_cpy_from_usr(struct tegra_dc_ext_flip_windowattr_v2 *outptr,
				void *inptr, u32 usr_win_size, u32 win_num)
{
	int i = 0;
	u8 *srcptr;

	for (i = 0; i < win_num; i++) {
		srcptr  = (u8 *)inptr + (usr_win_size * i);

		if (copy_from_user(&outptr[i],
			(void __user *) (uintptr_t)srcptr, usr_win_size))
			return -EFAULT;
	}
	return 0;
}

static int dev_cpy_to_usr(void *outptr, u32 usr_win_size,
		struct tegra_dc_ext_flip_windowattr_v2 *inptr, u32 win_num)
{
	int i = 0;
	u8 *dstptr;

	for (i = 0; i < win_num; i++) {
		dstptr  = (u8 *)outptr + (usr_win_size * i);

		if (copy_to_user((void __user *) (uintptr_t)dstptr,
			&inptr[i], usr_win_size))
			return -EFAULT;
	}
	return 0;
}

static long sfd_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	void __user *user_arg = (void __user *)arg;
	struct tegra_dc_ext_user *user = filp->private_data;
	int ret;

	switch (cmd) {
	case TEGRA_DC_EXT_SET_NVMAP_FD:
		return 0;

	case TEGRA_DC_EXT_GET_WINDOW:
		return sfd_get_window(user, arg);

	case TEGRA_DC_EXT_FLIP4:
	{
		int ret;
		int win_num;
		int nr_user_data;
		struct tegra_dc_ext_flip_4 args;
		struct tegra_dc_ext_flip_windowattr_v2 *win;
		struct tegra_dc_ext_flip_user_data *flip_user_data;
		u32 *syncpt_id = NULL, *syncpt_val = NULL;
		int *syncpt_fd = NULL;
		int syncpt_idx = -1;

		u32 usr_win_size = sizeof(struct tegra_dc_ext_flip_windowattr);

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

                // No yuv bypass used.
		user->ext->dc->yuv_bypass_dirty = false;
                user->ext->dc->yuv_bypass = false;
		win_num = args.win_num;
		win = kzalloc(sizeof(*win) * win_num, GFP_KERNEL);

		if (args.flags &
				TEGRA_DC_EXT_FLIP_HEAD_FLAG_V2_ATTR)
			usr_win_size =
				sizeof(struct tegra_dc_ext_flip_windowattr_v2);

		if (dev_cpy_from_usr(win, (void *)args.win,
					usr_win_size, win_num)) {
			kfree(win);
			return -EFAULT;
		}

		nr_user_data = args.nr_elements;
		flip_user_data = kzalloc(sizeof(*flip_user_data)
					* nr_user_data, GFP_KERNEL);
		if (nr_user_data > 0) {
			if (copy_from_user(flip_user_data,
				(void __user *) (uintptr_t)args.data,
				sizeof(*flip_user_data) * nr_user_data)) {
				kfree(flip_user_data);
				kfree(win);
				return -EFAULT;
			}
		}

		if (syncpt_idx == -1)
			syncpt_fd = &args.post_syncpt_fd;

		ret = sfd_flip(user, win, win_num,
			syncpt_id, syncpt_val, syncpt_fd, args.dirty_rect,
			args.flags, flip_user_data, nr_user_data);

		if (nr_user_data > 0)
			kfree(flip_user_data);

		if (dev_cpy_to_usr((void *)args.win, usr_win_size,
					win, win_num) ||
			copy_to_user(user_arg, &args, sizeof(args))) {
			kfree(win);
			return -EFAULT;
		}

		kfree(win);
		return ret;
	}

	case TEGRA_DC_EXT_GET_IMP_USER_INFO:
	{
#ifdef CONFIG_TEGRA_NVDISPLAY
		struct tegra_dc_ext_imp_user_info *info;
		int ret = 0;

		info = kzalloc(sizeof(*info), GFP_KERNEL);
		if (!info)
			return -ENOMEM;

		if (copy_from_user(info, user_arg, sizeof(*info))) {
			kfree(info);
			return -EFAULT;
		}

		ret = tegra_nvdisp_get_imp_user_info(user->ext->dc, info);
		if (ret) {
			kfree(info);
			return ret;
		}

		if (copy_to_user(user_arg, info, sizeof(*info))) {
			kfree(info);
			return -EFAULT;
		}

		kfree(info);
		return ret;
#else
		return -EINVAL;
#endif
	}

#ifdef CONFIG_TEGRA_ISOMGR
	case TEGRA_DC_EXT_SET_PROPOSED_BW_3:
	{
		int ret = 0;
		int win_num;
		int nr_user_data;
		struct tegra_dc_ext_flip_4 args;
		struct tegra_dc_ext_flip_windowattr_v2 *win = NULL;
		struct tegra_dc_ext_flip_user_data *flip_user_data = NULL;
		bool imp_proposed = false;

		/* Keeping window attribute size as version1 for old
		 *  legacy applications
		 */
		u32 usr_win_size = sizeof(struct tegra_dc_ext_flip_windowattr);

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		win_num = args.win_num;
		win = kzalloc(sizeof(*win) * win_num, GFP_KERNEL);
		if (!win)
			return -ENOMEM;

		if (dev_cpy_from_usr(win, (void *)args.win,
					usr_win_size, win_num)) {
			ret = -EFAULT;
			goto free_and_ret;
		}

		nr_user_data = args.nr_elements;
		flip_user_data = kzalloc(sizeof(*flip_user_data)
					* nr_user_data, GFP_KERNEL);
		if (!flip_user_data) {
			ret = -ENOMEM;
			goto free_and_ret;
		}

		if (nr_user_data > 0) {
			if (copy_from_user(flip_user_data,
				(void __user *) (uintptr_t)args.data,
				sizeof(*flip_user_data) * nr_user_data)) {
				ret = -EFAULT;
				goto free_and_ret;
			}

			if (flip_user_data[0].data_type ==
				TEGRA_DC_EXT_FLIP_USER_DATA_IMP_DATA) {
				ret = tegra_dc_handle_imp_propose(
						user->ext->dc, flip_user_data);
				if (ret)
					goto free_and_ret;

				imp_proposed = true;
			}
		}

		if (!imp_proposed)
			ret = sfd_negotiate_bw(user, win, win_num);

free_and_ret:
		kfree(flip_user_data);
		kfree(win);
		return ret;
	}
#endif

	case TEGRA_DC_EXT_GET_VBLANK_SYNCPT:
	{
		u32 syncpt = sfd_get_vblank_syncpt(user);

		if (copy_to_user(user_arg, &syncpt, sizeof(syncpt)))
			return -EFAULT;

		return 0;
	}

	case TEGRA_DC_EXT_GET_STATUS:
	{
		struct tegra_dc_ext_status args;
		int ret;

		ret = sfd_get_status(user, &args);

		if (copy_to_user(user_arg, &args, sizeof(args)))
			return -EFAULT;

		return ret;
	}

	case TEGRA_DC_EXT_GET_FEATURES:
	{
		struct tegra_dc_ext_feature args;
		int ret;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		ret = sfd_get_feature(user, &args);

		if (copy_to_user(user_arg, &args, sizeof(args)))
			return -EFAULT;

		return ret;
	}

	case TEGRA_DC_EXT_SET_VBLANK:
	{
		struct tegra_dc_ext_set_vblank args;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		return sfd_set_vblank(user->ext, args.enable);
	}

	case TEGRA_DC_EXT_GET_SCANLINE:
	{
		u32 scanln;

		dev_dbg(&user->ext->dc->ndev->dev, "GET SCANLN IOCTL\n");

		/* If display has been disconnected return with error. */
		if (!user->ext->dc->connected)
			return -EACCES;

		scanln = tegra_dc_get_v_count(user->ext->dc);

		if (copy_to_user(user_arg, &scanln, sizeof(scanln)))
			return -EFAULT;

		return 0;
	}

	case TEGRA_DC_EXT_SET_SCANLINE:
	{
		struct tegra_dc_ext_scanline_info args;

		dev_dbg(&user->ext->dc->ndev->dev, "SET SCANLN IOCTL\n");

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		ret = sfd_vpulse3(user->ext, &args);

		if (copy_to_user(user_arg, &args, sizeof(args)))
			return -EFAULT;

		return ret;
	}

	default:
		return -EINVAL;
	}
}

static int sfd_open(struct inode *inode, struct file *filp)
{
	struct tegra_dc_ext_user *user;
	struct tegra_dc_ext *ext;
	int open_count;

	user = kzalloc(sizeof(*user), GFP_KERNEL);
	if (!user)
		return -ENOMEM;

	ext = container_of(inode->i_cdev, struct tegra_dc_ext, cdev);
	user->ext = ext;

	filp->private_data = user;

	open_count = atomic_inc_return(&dc_open_count);
	if (open_count < 1) {
		pr_err("%s: unbalanced dc open count=%d\n", __func__,
			open_count);
		return -EINVAL;
	}

	return 0;
}

static int sfd_release(struct inode *inode, struct file *filp)
{
	struct tegra_dc_ext_user *user = filp->private_data;
	struct tegra_dc_ext *ext = user->ext;
	unsigned int i;
	unsigned long int windows = 0;
	int open_count;

	for (i = 0; i < tegra_dc_get_numof_dispwindows(); i++) {
		if (ext->win[i].user == user) {
			sfd_put_window(user, i);
			windows |= BIT(i);
		}
	}

	if (ext->dc->enabled) {
		tegra_dc_blank_wins(ext->dc, windows);
		for_each_set_bit(i, &windows,
				tegra_dc_get_numof_dispwindows()) {
			sfd_unpin_window(&ext->win[i]);
			tegra_dc_disable_window(ext->dc, i);
		}
	}

	kfree(user);

	open_count = atomic_dec_return(&dc_open_count);
	if (open_count < 0) {
		pr_err("%s: unbalanced dc open count=%d\n", __func__,
			open_count);
		return -EINVAL;
	}

	if (open_count == 0)
		tegra_dc_reset_imp_state();

	return 0;
}

static const struct file_operations sfd_devops = {
	.owner =		THIS_MODULE,
	.open =			sfd_open,
	.release =		sfd_release,
	.unlocked_ioctl =	sfd_ioctl,
};

static int sfd_setup_windows(struct tegra_dc_ext *ext)
{
        int i, ret;

        for (i = 0; i < ext->dc->n_windows; i++) {
                struct tegra_dc_ext_win *win = &ext->win[i];
                char name[32];

                win->ext = ext;
                win->idx = i;

                snprintf(name, sizeof(name), "tegradc.%d/%c",
                         ext->dc->ndev->id, 'a' + i);
                win->flip_wq = create_singlethread_workqueue(name);
                if (!win->flip_wq) {
                        ret = -ENOMEM;
                        goto cleanup;
                }

                mutex_init(&win->lock);
                mutex_init(&win->queue_lock);
                INIT_LIST_HEAD(&win->timestamp_queue);
        }

        return 0;

cleanup:
        while (i--) {
                struct tegra_dc_ext_win *win = &ext->win[i];
                destroy_workqueue(win->flip_wq);
        }

        return ret;
}

struct tegra_dc_ext *sfd_register(struct platform_device *ndev,
					   struct tegra_dc *dc)
{
	int ret;
	struct tegra_dc_ext *ext;
	dev_t devno;
	char name[16];

	ext = kzalloc(sizeof(*ext), GFP_KERNEL);
	if (!ext)
		return ERR_PTR(-ENOMEM);

	BUG_ON(!sfd_devno);
	devno = sfd_devno + head_count + 1;

	cdev_init(&ext->cdev, &sfd_devops);
	ext->cdev.owner = THIS_MODULE;
	ret = cdev_add(&ext->cdev, devno, 1);
	if (ret) {
		dev_err(&ndev->dev, "Failed to create character device\n");
		goto cleanup_alloc;
	}

	ext->dev = device_create(sfd_class,
				 &ndev->dev,
				 devno,
				 NULL,
				 "ml_sfd_%d",
				 ndev->id);

	if (IS_ERR(ext->dev)) {
		ret = PTR_ERR(ext->dev);
		goto cleanup_cdev;
	}

	ext->dc = dc;

	ret = sfd_setup_windows(ext);
	if (ret)
		goto cleanup_device;

	mutex_init(&ext->cursor.lock);

	/* Setup scanline workqueues */
	ext->scanline_trigger = -1;
	snprintf(name, sizeof(name), "tegradc.%d/sl", dc->ndev->id);
	ext->scanline_wq = create_singlethread_workqueue(name);
	if (!ext->scanline_wq) {
		ret = -ENOMEM;
		goto cleanup_device;
	}
	/* Setup scanline mutex */
	mutex_init(&ext->scanline_lock);

	head_count++;

	return ext;

cleanup_device:
	device_del(ext->dev);

cleanup_cdev:
	cdev_del(&ext->cdev);

cleanup_alloc:
	kfree(ext);

	return ERR_PTR(ret);
}

void sfd_unregister(struct tegra_dc_ext *ext)
{
	int i;

	for (i = 0; i < ext->dc->n_windows; i++) {
		struct tegra_dc_ext_win *win = &ext->win[i];

		flush_workqueue(win->flip_wq);
		destroy_workqueue(win->flip_wq);
	}

	/* Remove scanline workqueues */
	flush_workqueue(ext->scanline_wq);
	destroy_workqueue(ext->scanline_wq);
	ext->scanline_wq = NULL;
	/* Clear trigger line value */
	ext->scanline_trigger = -1;
	/* Udate min val all the way to max. */
	nvhost_syncpt_set_min_eq_max_ext(ext->dc->ndev,
					ext->dc->vpulse3_syncpt);

	device_del(ext->dev);
	cdev_del(&ext->cdev);

	kfree(ext);

	head_count--;
}

int __init sfd_module_init(void)
{
	int ret, nheads = tegra_dc_get_numof_dispheads();

	if (nheads <= 0) {
		pr_err("%s: max heads:%d cannot be negative or zero\n",
			__func__, nheads);
		return -EINVAL;
	}

	sfd_class = class_create(THIS_MODULE, "sfd_class");
	if (!sfd_class) {
		printk(KERN_ERR "sfd: failed to create class\n");
		return -ENOMEM;
	}

	/* Reserve one character device per head, plus the control device */
	ret = alloc_chrdev_region(&sfd_devno,
				  0, nheads + 1,
				  "sfd_class");
	if (ret)
		goto cleanup_class;

	ret = tegra_dc_ext_control_init();
	if (ret)
		goto cleanup_region;

	return 0;

cleanup_region:
	unregister_chrdev_region(sfd_devno, nheads);

cleanup_class:
	class_destroy(sfd_class);

	return ret;
}

void __exit sfd_module_exit(void)
{
	unregister_chrdev_region(sfd_devno,
			tegra_dc_get_numof_dispheads());
	class_destroy(sfd_class);
}
