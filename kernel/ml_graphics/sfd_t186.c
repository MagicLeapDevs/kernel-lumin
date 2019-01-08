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
#include "sfd_t186.h"

static inline int test_bit_u32(int bitnum, const u32 *data, int entries)
{
        int i;

        for (i = 0; i < entries; i++) {
                if (bitnum < 32) {
                        if (1UL & (data[bitnum / 32] >> (bitnum & 31)))
                                return 1;
                } else {
                        bitnum -= 32;
                        data++;
                }
        }

        return 0;
}

static s64 sfd_timespec_to_ns(const struct tegra_timespec *ts)
{
        return ((s64) ts->tv_sec * NSEC_PER_SEC) + ts->tv_nsec;
}

int sfd_get_window(struct tegra_dc_ext_user *user,
                                   unsigned int n)
{
        struct tegra_dc_ext *ext = user->ext;
        struct tegra_dc_ext_win *win;
        int ret = 0;

        if ((n >= tegra_dc_get_numof_dispwindows()) ||
                !(ext->dc->valid_windows & BIT(n)))
                return -EINVAL;

        win = &ext->win[n];

        mutex_lock(&win->lock);

        if (!win->user) {
                win->user = user;
                win->enabled = false;
        } else {
                if (win->user != user)
                        ret = -EBUSY;
        }
        mutex_unlock(&win->lock);

        return ret;
}

int sfd_put_window(struct tegra_dc_ext_user *user,
                                   unsigned int n)
{
        struct tegra_dc_ext *ext = user->ext;
        struct tegra_dc_ext_win *win;
        int ret = 0;

        if ((n >= tegra_dc_get_numof_dispwindows()) ||
                !(ext->dc->valid_windows & BIT(n)))
                return -EINVAL;

        win = &ext->win[n];

        mutex_lock(&win->lock);

        if (win->user == user) {
                flush_workqueue(win->flip_wq);
                win->user = NULL;
                win->enabled = false;
        } else {
                ret = -EACCES;
        }

        mutex_unlock(&win->lock);

        return ret;
}

void sfd_set_windowattr_basic(struct tegra_dc_win *win,
                       const struct tegra_dc_ext_flip_windowattr_v2 *flip_win)
{
        win->flags = TEGRA_WIN_FLAG_ENABLED;
        if (flip_win->blend == TEGRA_DC_EXT_BLEND_PREMULT)
                win->flags |= TEGRA_WIN_FLAG_BLEND_PREMULT;
        else if (flip_win->blend == TEGRA_DC_EXT_BLEND_COVERAGE)
                win->flags |= TEGRA_WIN_FLAG_BLEND_COVERAGE;
        if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_TILED)
                win->flags |= TEGRA_WIN_FLAG_TILED;
        if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_INVERT_H)
                win->flags |= TEGRA_WIN_FLAG_INVERT_H;
        if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_INVERT_V)
                win->flags |= TEGRA_WIN_FLAG_INVERT_V;
        if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_GLOBAL_ALPHA)
                win->global_alpha = flip_win->global_alpha;
        else
                win->global_alpha = 255;
#if defined(CONFIG_TEGRA_DC_SCAN_COLUMN)
        if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_SCAN_COLUMN)
                win->flags |= TEGRA_WIN_FLAG_SCAN_COLUMN;
#endif
#if defined(CONFIG_TEGRA_DC_BLOCK_LINEAR)
        if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_BLOCKLINEAR) {
                win->flags |= TEGRA_WIN_FLAG_BLOCKLINEAR;
                win->block_height_log2 = flip_win->block_height_log2;
        }
#endif

        if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_LIMITED)
                win->flags |= TEGRA_WIN_FLAG_INPUT_RANGE_LIMITED;
        else if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_BYPASS)
                win->flags |= TEGRA_WIN_FLAG_INPUT_RANGE_BYPASS;
        else
                win->flags |= TEGRA_WIN_FLAG_INPUT_RANGE_FULL;

        if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_CS_REC601)
                win->flags |= TEGRA_WIN_FLAG_CS_REC601;
        else if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_CS_REC709)
                win->flags |= TEGRA_WIN_FLAG_CS_REC709;
        else if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_CS_REC2020)
                win->flags |= TEGRA_WIN_FLAG_CS_REC2020;
        else
                win->flags |= TEGRA_WIN_FLAG_CS_DEFAULT;

        win->fmt = flip_win->pixformat;
        win->x.full = flip_win->x;
        win->y.full = flip_win->y;
        win->w.full = flip_win->w;
        win->h.full = flip_win->h;
        /* XXX verify that this doesn't go outside display's active region */
        win->out_x = flip_win->out_x;
        win->out_y = flip_win->out_y;
        win->out_w = flip_win->out_w;
        win->out_h = flip_win->out_h;
        win->z = flip_win->z;

        win->stride = flip_win->stride;
        win->stride_uv = flip_win->stride_uv;
}

int sfd_set_windowattr(struct tegra_dc_ext *ext,
                               struct tegra_dc_win *win,
                               const struct sfd_flip_win *flip_win)
{
        int err = 0;
        struct tegra_dc_ext_win *ext_win = &ext->win[win->idx];
        s64 timestamp_ns;
        struct tegra_vrr *vrr = ext->dc->out->vrr;

        if (flip_win->handle[TEGRA_DC_Y] == NULL) {
                win->flags = 0;
                memset(ext_win->cur_handle, 0, sizeof(ext_win->cur_handle));
                return 0;
        }

        sfd_set_windowattr_basic(win, &flip_win->attr);

        memcpy(ext_win->cur_handle, flip_win->handle,
               sizeof(ext_win->cur_handle));

        /* XXX verify that this won't read outside of the surface */
        win->phys_addr = flip_win->phys_addr + flip_win->attr.offset;

        win->phys_addr_u = flip_win->handle[TEGRA_DC_U] ?
                flip_win->phys_addr_u : flip_win->phys_addr;
        win->phys_addr_u += flip_win->attr.offset_u;

        win->phys_addr_v = flip_win->handle[TEGRA_DC_V] ?
                flip_win->phys_addr_v : flip_win->phys_addr;
        win->phys_addr_v += flip_win->attr.offset_v;

        win->cde.cde_addr = 0;

        if ((s32)flip_win->attr.pre_syncpt_id >= 0) {
                nvhost_syncpt_wait_timeout_ext(ext->dc->ndev,
                                flip_win->attr.pre_syncpt_id,
                                flip_win->attr.pre_syncpt_val,
                                msecs_to_jiffies(5000), NULL, NULL);
        }

        if (err < 0)
                return err;

        if (tegra_platform_is_silicon()) {
                timestamp_ns = sfd_timespec_to_ns(&flip_win->attr.timestamp);

                if (timestamp_ns) {
                        /* XXX: Should timestamping be overridden by "no_vsync"
                         * flag */
                        if (vrr && vrr->enable) {
                                struct timespec tm;
                                s64 now_ns = 0;
                                s32 sleep_us = 0;
                                ktime_get_ts(&tm);
                                now_ns = timespec_to_ns(&tm);
                                sleep_us = (s32)div_s64(timestamp_ns -
                                        now_ns, 1000ll);

                                if (sleep_us > TEGRA_DC_TS_MAX_DELAY_US)
                                        sleep_us = TEGRA_DC_TS_MAX_DELAY_US;

                                if (sleep_us > 0)
                                        usleep_range(sleep_us, sleep_us +
                                                TEGRA_DC_TS_SLACK_US);
                        } else {
                                tegra_dc_config_frame_end_intr(win->dc, true);
                                err = wait_event_interruptible(
                                        win->dc->timestamp_wq,
                                        tegra_dc_is_within_n_vsync(win->dc,
                                                timestamp_ns));
                                tegra_dc_config_frame_end_intr(win->dc, false);
                        }
                }
        }

        return err;
}

#ifdef CONFIG_TEGRA_ISOMGR
int sfd_negotiate_bw(struct tegra_dc_ext_user *user,
                        struct tegra_dc_ext_flip_windowattr_v2 *wins,
                        int win_num)
{
        int i;
        int ret;
        struct tegra_dc_win *dc_wins[DC_N_WINDOWS];
        struct tegra_dc *dc = user->ext->dc;

        /* If display has been disconnected return with error. */
        if (!dc->connected)
                return -1;

        for (i = 0; i < win_num; i++) {
                int idx = wins[i].index;

                if (wins[i].buff_id > 0) {
                        sfd_set_windowattr_basic(&dc->tmp_wins[idx],
                                                          &wins[i]);
                } else {
                        dc->tmp_wins[idx].flags = 0;
                        dc->tmp_wins[idx].new_bandwidth = 0;
                }
                dc_wins[i] = &dc->tmp_wins[idx];
        }

        ret = tegra_dc_bandwidth_negotiate_bw(dc, dc_wins, win_num);

        return ret;
}
#endif

static int sfd_pin_window(struct tegra_dc_ext_user *user, u32 fd,
                            struct tegra_dc_dmabuf **dc_buf,
                            dma_addr_t *phys_addr)
{
        struct tegra_dc_ext *ext = user->ext;
        struct tegra_dc_dmabuf *dc_dmabuf;
        dma_addr_t dma_addr;

        *dc_buf = NULL;
        *phys_addr = -1;
        if (!fd)
                return 0;

        dc_dmabuf = kzalloc(sizeof(*dc_dmabuf), GFP_KERNEL);
        if (!dc_dmabuf)
                return -ENOMEM;

        dc_dmabuf->buf = dma_buf_get(fd);
        if (IS_ERR_OR_NULL(dc_dmabuf->buf))
                goto buf_fail;

        dc_dmabuf->attach = dma_buf_attach(dc_dmabuf->buf, ext->dev->parent);
        if (IS_ERR_OR_NULL(dc_dmabuf->attach))
                goto attach_fail;

        dc_dmabuf->sgt = dma_buf_map_attachment(dc_dmabuf->attach,
                                                DMA_TO_DEVICE);
        if (IS_ERR_OR_NULL(dc_dmabuf->sgt))
                goto sgt_fail;

        dma_addr = sg_dma_address(dc_dmabuf->sgt->sgl);
        if (dma_addr)
                *phys_addr = dma_addr;
        else
                *phys_addr = sg_phys(dc_dmabuf->sgt->sgl);

        *dc_buf = dc_dmabuf;

        return 0;
sgt_fail:
        dma_buf_detach(dc_dmabuf->buf, dc_dmabuf->attach);
attach_fail:
        dma_buf_put(dc_dmabuf->buf);
buf_fail:
        kfree(dc_dmabuf);
        return -ENOMEM;
}

int sfd_pin_windows(struct tegra_dc_ext_user *user,
                                struct tegra_dc_ext_flip_windowattr_v2 *wins,
                                int win_num,
                                struct sfd_flip_win *flip_wins,
                                bool *has_timestamp,
                                bool syncpt_fd)
{
        int i, ret;
        struct tegra_dc *dc = user->ext->dc;

        for (i = 0; i < win_num; i++) {
                struct sfd_flip_win *flip_win = &flip_wins[i];
                int index = wins[i].index;

                memcpy(&flip_win->attr, &wins[i], sizeof(flip_win->attr));

                if (has_timestamp && sfd_timespec_to_ns(
                        &flip_win->attr.timestamp)) {

                        /* Set first frame timestamp to 0 after device boot-up
                           to prevent wait on 1st flip request */
                        if (!dc->frame_end_timestamp)
                                memset(&flip_win->attr.timestamp, 0,
                                        sizeof(flip_win->attr.timestamp));
                        *has_timestamp = true;
                }

                if (index < 0 || !test_bit(index, &dc->valid_windows))
                        continue;

                ret = sfd_pin_window(user, flip_win->attr.buff_id,
                                              &flip_win->handle[TEGRA_DC_Y],
                                              &flip_win->phys_addr);
                if (ret)
                        return ret;

                flip_win->handle[TEGRA_DC_U] = NULL;
                flip_win->phys_addr_u = 0;

                flip_win->handle[TEGRA_DC_V] = NULL;
                flip_win->phys_addr_v = 0;

                flip_win->handle[TEGRA_DC_CDE] = NULL;
                flip_win->phys_addr_cde = 0;

                if (syncpt_fd) {
                        if (flip_win->attr.pre_syncpt_fd >= 0) {
                                BUG();
                        } else {
                                flip_win->attr.pre_syncpt_id = NVSYNCPT_INVALID;
                        }
                }
        }

        return 0;
}

void sfd_unpin_handles(struct tegra_dc_dmabuf *unpin_handles[],
                                       int nr_unpin)
{
        int i;

        for (i = 0; i < nr_unpin; i++) {
                dma_buf_unmap_attachment(unpin_handles[i]->attach,
                        unpin_handles[i]->sgt, DMA_TO_DEVICE);
                dma_buf_detach(unpin_handles[i]->buf,
                               unpin_handles[i]->attach);
                dma_buf_put(unpin_handles[i]->buf);
                kfree(unpin_handles[i]);
        }
}

void sfd_unpin_window(struct tegra_dc_ext_win *win)
{
        struct tegra_dc_dmabuf *unpin_handles[TEGRA_DC_NUM_PLANES];
        int nr_unpin = 0;

        if (win->cur_handle[TEGRA_DC_Y]) {
                int j;
                for (j = 0; j < TEGRA_DC_NUM_PLANES; j++) {
                        if (win->cur_handle[j])
                                unpin_handles[nr_unpin++] = win->cur_handle[j];
                }
                memset(win->cur_handle, 0, sizeof(win->cur_handle));
        }

        sfd_unpin_handles(unpin_handles, nr_unpin);
}

int sfd_get_status(struct tegra_dc_ext_user *user,
                                   struct tegra_dc_ext_status *status)
{
        struct tegra_dc *dc = user->ext->dc;

        memset(status, 0, sizeof(*status));

        if (dc->enabled)
                status->flags |= TEGRA_DC_EXT_FLAGS_ENABLED;

        return 0;
}

int sfd_get_feature(struct tegra_dc_ext_user *user,
                                   struct tegra_dc_ext_feature *feature)
{
        struct tegra_dc *dc = user->ext->dc;
        struct tegra_dc_feature *table = dc->feature;

        if (dc->enabled && feature->entries) {
                feature->length = table->num_entries;
                memcpy(feature->entries, table->entries, table->num_entries *
                                        sizeof(struct tegra_dc_feature_entry));
        }

        return 0;
}

u32 sfd_get_vblank_syncpt(struct tegra_dc_ext_user *user)
{
        struct tegra_dc *dc = user->ext->dc;

        return dc->vblank_syncpt;
}

int sfd_get_scanline(struct tegra_dc_ext *ext)
{
       return ext->scanline_trigger;
}

int sfd_set_vblank(struct tegra_dc_ext *ext, bool enable)
{
        struct tegra_dc *dc;
        int ret = 0;

        if (ext->vblank_enabled == enable)
                return 0;

        dc = ext->dc;

        if (enable)
                ret = tegra_dc_vsync_enable(dc);
        else if (ext->vblank_enabled)
                tegra_dc_vsync_disable(dc);

        if (!ret) {
                ext->vblank_enabled = enable;
                return 0;
        }
        return 1;
}

void sfd_setup_vpulse3(struct tegra_dc_ext *ext,
                                        unsigned int scanline_num)
{
        struct tegra_dc *dc = ext->dc;
        u32 old_state;

        tegra_dc_get(dc);
        mutex_lock(&dc->lock);
        old_state = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);

        /* write active version; this updates assembly as well */
        tegra_dc_writel(dc, WRITE_MUX_ACTIVE | READ_MUX_ACTIVE,
                        DC_CMD_STATE_ACCESS);
        tegra_dc_writel(dc, scanline_num, DC_DISP_V_PULSE3_POSITION_A);

        tegra_dc_writel(dc, old_state, DC_CMD_STATE_ACCESS);
        tegra_dc_readl(dc, DC_CMD_STATE_ACCESS); /* flush */
        mutex_unlock(&dc->lock);
        tegra_dc_put(dc);
}

void sfd_incr_vpulse3(struct tegra_dc_ext *ext)
{
        struct tegra_dc *dc = ext->dc;
        unsigned int vpulse3_sync_id = dc->vpulse3_syncpt;

        /* Request vpulse3 syncpt increment */
        tegra_dc_writel(dc, VPULSE3_COND | vpulse3_sync_id,
                                DC_CMD_GENERAL_INCR_SYNCPT);
}

int sfd_vpulse3(struct tegra_dc_ext *ext,
                                struct tegra_dc_ext_scanline_info *args)
{
        int ret = -EFAULT;
        struct tegra_dc *dc = ext->dc;
        struct sfd_scanline_data *data;
        unsigned int vpulse3_sync_id = dc->vpulse3_syncpt;
        unsigned int vpulse3_sync_max_val = 0;

        /* If display has been disconnected OR
         * Vpulse3 syncpt is invalid OR
         * scanline workqueue is not setup, return error
         */
        if (!dc->enabled || !dc->connected ||
                vpulse3_sync_id == NVSYNCPT_INVALID ||
                !ext->scanline_wq) {
                dev_err(&dc->ndev->dev, "DC not setup\n");
                return -EACCES;
        }

        /* TODO: Support id != 0 */
        if (args->id) {
                dev_err(&dc->ndev->dev, "Invalid scanline id\n");
                return -EINVAL;
        }

        /* TODO: Support frame != 0 and raw syncpt */
        if ((args->flags & TEGRA_DC_EXT_SCANLINE_FLAG_ENABLE) &&
                (args->frame ||
                args->flags & TEGRA_DC_EXT_SCANLINE_FLAG_RAW_SYNCPT)) {
                dev_err(&dc->ndev->dev, "Invalid args\n");
                return -EINVAL;
        }

        dev_dbg(&dc->ndev->dev, "vp3 id : %d\n", vpulse3_sync_id);

        /* Allocate arg for workqueue */
        data = kzalloc(sizeof(*data), GFP_KERNEL);
        if (!data)
                return -ENOMEM;

        if (args->flags & TEGRA_DC_EXT_SCANLINE_FLAG_ENABLE) {
                /* Increment max_val by 1 */
                vpulse3_sync_max_val =
                    nvhost_syncpt_incr_max_ext(dc->ndev, vpulse3_sync_id, 1);

                dev_dbg(&dc->ndev->dev, "vp3 max: %d\n", vpulse3_sync_max_val);

                /* Create a fencefd */
                ret = nvhost_syncpt_create_fence_single_ext(dc->ndev,
                                vpulse3_sync_id, vpulse3_sync_max_val,
                                "vpulse3-fence", &args->syncfd);

                if (ret) {
                        dev_err(&dc->ndev->dev,
                                "Failed creating vpulse3 fence err: %d\n", ret);

                        kfree(data);
                        return ret;
                }

                /* Pass trigger line value */
                data->triggered_line = args->triggered_line;
        } else {
                /* Clear trigger line value */
                data->triggered_line = -1;

                /* Pass current max_val */
                data->max_val = nvhost_syncpt_read_maxval(dc->ndev,
                                                        vpulse3_sync_id);
        }

        /* Queue work to setup/increment/remove scanline trigger */
        data->ext = ext;
        INIT_WORK(&data->work, sfd_scanline_worker);
        queue_work(ext->scanline_wq, &data->work);

        return 0;
}

void sfd_scanline_worker(struct work_struct *work)
{
        struct sfd_scanline_data *data =
                container_of(work, struct sfd_scanline_data, work);
        struct tegra_dc_ext *ext = data->ext;
        struct tegra_dc *dc = ext->dc;
        unsigned int vpulse3_sync_id = dc->vpulse3_syncpt;
        int min_val;

        /* Allow only one scanline operation at a time */
        mutex_lock(&ext->scanline_lock);

        /* Wait for frame end before programming new request */
        _tegra_dc_wait_for_frame_end(dc,
                div_s64(dc->frametime_ns, 1000000ll) * 2);

        if (data->triggered_line >= 0) {
                if (ext->scanline_trigger != data->triggered_line) {
                        dev_dbg(&dc->ndev->dev, "vp3 sl#: %d\n",
                                        data->triggered_line);

                        /* Setup vpulse3 trigger line */
                        sfd_setup_vpulse3(ext, data->triggered_line);

                        /* Save new scanline trigger state */
                        ext->scanline_trigger = data->triggered_line;
                }

                /* Request vpulse3 increment */
                sfd_incr_vpulse3(ext);
        } else {
                /* Clear scanline trigger state */
                ext->scanline_trigger = -1;

                /* Read current min_val */
                min_val = nvhost_syncpt_read_minval(dc->ndev, vpulse3_sync_id);

                /* Udate min_val to expected max only if they don't match */
                if (min_val != data->max_val) {
                        dev_dbg(&dc->ndev->dev, "vp3 mismatch; %d vs %d",
                                                min_val, data->max_val);

                        nvhost_syncpt_set_minval(dc->ndev, vpulse3_sync_id,
                                                data->max_val);
                }
        }

        /* Free arg allocated in the ioctl call */
        kfree(data);

        /* Release lock */
        mutex_unlock(&ext->scanline_lock);
}

static int lock_windows_for_flip(struct tegra_dc_ext_user *user,
                        struct tegra_dc_ext_flip_windowattr_v2 *win_attr,
                        int win_num)
{
        struct tegra_dc_ext *ext = user->ext;
        u8 idx_mask = 0;
        int i;

        BUG_ON(win_num > tegra_dc_get_numof_dispwindows());
        for (i = 0; i < win_num; i++) {
                int index = win_attr[i].index;

                if (index < 0 || !test_bit(index, &ext->dc->valid_windows))
                        continue;

                idx_mask |= BIT(index);
        }

        for (i = 0; i < win_num; i++) {
                struct tegra_dc_ext_win *win;

                if (!(idx_mask & BIT(i)))
                        continue;

                win = &ext->win[i];

                mutex_lock_nested(&win->lock, i);

                if (win->user != user)
                        goto fail_unlock;
        }

        return 0;

fail_unlock:
        do {
                if (!(idx_mask & BIT(i)))
                        continue;

                mutex_unlock(&ext->win[i].lock);
        } while (i--);

        return -EACCES;
}

static void unlock_windows_for_flip(struct tegra_dc_ext_user *user,
                                struct tegra_dc_ext_flip_windowattr_v2 *win,
                                int win_num)
{
        struct tegra_dc_ext *ext = user->ext;
        u8 idx_mask = 0;
        int i;

        BUG_ON(win_num > tegra_dc_get_numof_dispwindows());
        for (i = 0; i < win_num; i++) {
                int index = win[i].index;

                if (index < 0 || !test_bit(index, &ext->dc->valid_windows))
                        continue;

                idx_mask |= BIT(index);
        }
        for (i = win_num - 1; i >= 0; i--) {
                if (!(idx_mask & BIT(i)))
                        continue;

                mutex_unlock(&ext->win[i].lock);
        }
}

static int sanitize_flip_args(struct tegra_dc_ext_user *user,
                                struct tegra_dc_ext_flip_windowattr_v2 *win,
                                int win_num, __u16 **dirty_rect)
{
        int i, used_windows = 0;
        struct tegra_dc *dc = user->ext->dc;

        if (win_num > tegra_dc_get_numof_dispwindows())
                return -EINVAL;

        for (i = 0; i < win_num; i++) {
                int index = win[i].index;

                if (index < 0)
                        continue;

                if (index >= tegra_dc_get_numof_dispwindows() ||
                        !test_bit(index, &dc->valid_windows))
                        return -EINVAL;

                if (used_windows & BIT(index))
                        return -EINVAL;

                used_windows |= BIT(index);
        }

        if (!used_windows)
                return -EINVAL;

        if (*dirty_rect) {
                unsigned int xoff = (*dirty_rect)[0];
                unsigned int yoff = (*dirty_rect)[1];
                unsigned int width = (*dirty_rect)[2];
                unsigned int height = (*dirty_rect)[3];

                if ((!width && !height) ||
                        dc->mode.vmode == FB_VMODE_INTERLACED ||
                        !dc->out_ops ||
                        !dc->out_ops->partial_update ||
                        (!xoff && !yoff &&
                        (width == dc->mode.h_active) &&
                        (height == dc->mode.v_active))) {
                        /* Partial update undesired, unsupported,
                         * or dirty_rect covers entire frame. */
                        *dirty_rect = NULL;
                } else {
                        if (!width || !height ||
                                (xoff + width) > dc->mode.h_active ||
                                (yoff + height) > dc->mode.v_active)
                                return -EINVAL;

                        /* Constraint 7: H/V_DISP_ACTIVE >= 16.
                         * Make sure the minimal size of dirty region is 16*16.
                         * If not, extend the dirty region. */
                        if (width < 16) {
                                width = (*dirty_rect)[2] = 16;
                                if (xoff + width > dc->mode.h_active)
                                        (*dirty_rect)[0] = dc->mode.h_active -
                                                width;
                        }
                        if (height < 16) {
                                height = (*dirty_rect)[3] = 16;
                                if (yoff + height > dc->mode.v_active)
                                        (*dirty_rect)[1] = dc->mode.v_active -
                                                height;
                        }
                }
        }

        return 0;
}

void sfd_read_user_data(struct sfd_flip_data *data,
                        struct tegra_dc_ext_flip_user_data *flip_user_data,
                        int nr_user_data)
{
        int i = 0;

        for (i = 0; i < nr_user_data; i++) {
                struct tegra_dc_hdr *hdr;
                struct tegra_dc_ext_hdr *info;
                switch (flip_user_data[i].data_type) {
                case TEGRA_DC_EXT_FLIP_USER_DATA_HDR_DATA:
                        hdr = &data->hdr_data;
                        info = &flip_user_data[i].hdr_info;
                        if (flip_user_data[i].flags &
                                TEGRA_DC_EXT_FLIP_FLAG_HDR_ENABLE)
                                hdr->enabled = true;
                        if (flip_user_data[i].flags &
                                TEGRA_DC_EXT_FLIP_FLAG_HDR_DATA_UPDATED) {
                                data->hdr_cache_dirty = true;
                                hdr->eotf = info->eotf;
                                hdr->static_metadata_id =
                                                info->static_metadata_id;
                                memcpy(hdr->static_metadata,
                                        info->static_metadata,
                                        sizeof(hdr->static_metadata));
                        }
                        break;
#ifdef CONFIG_TEGRA_NVDISPLAY
                case TEGRA_DC_EXT_FLIP_USER_DATA_IMP_TAG:
                        data->imp_session_id =
                                        flip_user_data[i].imp_tag.session_id;
                        data->imp_dirty = true;
                        break;
#endif
                case TEGRA_DC_EXT_FLIP_USER_DATA_POST_SYNCPT:
                        /* Already handled in the core FLIP ioctl. */
                        break;
                default:
                        dev_err(&data->ext->dc->ndev->dev,
                                "Invalid FLIP_USER_DATA_TYPE\n");
                }
        }
        return;
}

void sfd_flip_worker(struct work_struct *work)
{
        struct sfd_flip_data *data =
                container_of(work, struct sfd_flip_data, work);
        int win_num = data->act_window_num;
        struct tegra_dc_ext *ext = data->ext;
        struct tegra_dc_win *wins[DC_N_WINDOWS];
        struct tegra_dc_dmabuf *unpin_handles[DC_N_WINDOWS *
                                               TEGRA_DC_NUM_PLANES];
        struct tegra_dc_dmabuf *old_handle;
        struct tegra_dc *dc = ext->dc;
        int i, nr_unpin = 0, nr_win = 0;
        bool skip_flip = false;
        bool wait_for_vblank = false;

        BUG_ON(win_num > tegra_dc_get_numof_dispwindows());
        for (i = 0; i < win_num; i++) {
                struct sfd_flip_win *flip_win = &data->win[i];
                int index = flip_win->attr.index;
                struct tegra_dc_win *win;
                struct tegra_dc_ext_win *ext_win;
                struct sfd_flip_data *temp = NULL;
                s64 head_timestamp = -1;
                int j = 0;
                u32 reg_val = 0;

                if (index < 0 || !test_bit(index, &dc->valid_windows))
                        continue;

                win = tegra_dc_get_window(dc, index);
                if (!win)
                        continue;
                ext_win = &ext->win[index];

                if (!(atomic_dec_and_test(&ext_win->nr_pending_flips)) &&
                        (flip_win->attr.flags & TEGRA_DC_EXT_FLIP_FLAG_CURSOR))
                        skip_flip = true;

                mutex_lock(&ext_win->queue_lock);
                list_for_each_entry(temp, &ext_win->timestamp_queue,
                                timestamp_node) {
                        if (!tegra_platform_is_silicon())
                                continue;
                        if (j == 0) {
                                if (unlikely(temp != data)) {
                                        /* Frame doesn't contain timestamp in list */
                                        break;
                                } else
                                        head_timestamp = sfd_timespec_to_ns(
                                                &flip_win->attr.timestamp);
                        } else {
                                s64 timestamp = sfd_timespec_to_ns(
                                        &temp->win[i].attr.timestamp);

                                skip_flip = !tegra_dc_does_vsync_separate(dc,
                                                timestamp, head_timestamp);
                                /* Look ahead only one flip */
                                break;
                        }
                        j++;
                }
                if (head_timestamp >= 0)
                        list_del(&data->timestamp_node);
                mutex_unlock(&ext_win->queue_lock);

                if (skip_flip)
                        old_handle = flip_win->handle[TEGRA_DC_Y];
                else
                        old_handle = ext_win->cur_handle[TEGRA_DC_Y];

                if (old_handle) {
                        int j;
                        for (j = 0; j < TEGRA_DC_NUM_PLANES; j++) {
                                if (skip_flip)
                                        old_handle = flip_win->handle[j];
                                else
                                        old_handle = ext_win->cur_handle[j];

                                if (!old_handle)
                                        continue;

                                unpin_handles[nr_unpin++] = old_handle;
                        }
                }

                if (!skip_flip)
                        sfd_set_windowattr(ext, win, &data->win[i]);

                if (flip_win->attr.swap_interval && !no_vsync)
                        wait_for_vblank = true;

                ext_win->enabled = !!(win->flags & TEGRA_WIN_FLAG_ENABLED);

                wins[nr_win++] = win;
        }

        //trace_sync_wt_ovr_syncpt_upd((data->win[win_num-1]).syncpt_max);

        if (dc->enabled && !skip_flip)
                tegra_dc_set_hdr(dc, &data->hdr_data, data->hdr_cache_dirty);

        if (dc->enabled && !skip_flip) {
                dc->blanked = false;
                if (dc->out_ops && dc->out_ops->vrr_enable)
                                dc->out_ops->vrr_enable(dc,
                                        data->flags &
                                        TEGRA_DC_EXT_FLIP_HEAD_FLAG_VRR_MODE);

                if (data->imp_dirty) {
                        dc->imp_dirty = true;
                        tegra_dc_adjust_imp(dc, true);
                }

                tegra_dc_update_windows(wins, nr_win,
                        data->dirty_rect_valid ? data->dirty_rect : NULL,
                        wait_for_vblank);
                /* TODO: implement swapinterval here */
                tegra_dc_sync_windows(wins, nr_win);
                //trace_scanout_syncpt_upd((data->win[win_num-1]).syncpt_max);
                //if (dc->out->vrr)
                        //trace_scanout_vrr_stats((data->win[win_num-1]).syncpt_max
                        //                                , dc->out->vrr->dcb);
                tegra_dc_program_bandwidth(dc, true);
                if (!tegra_dc_has_multiple_dc())
                        tegra_dc_call_flip_callback();

                if (data->imp_dirty)
                        tegra_dc_adjust_imp(dc, false);
        }

        if (data->imp_dirty)
                tegra_dc_release_common_channel(dc);

        if (!skip_flip) {
                for (i = 0; i < win_num; i++) {
                        struct sfd_flip_win *flip_win = &data->win[i];
                        int index = flip_win->attr.index;

                        if (index < 0 ||
                                !test_bit(index, &dc->valid_windows))
                                continue;

                        tegra_dc_incr_syncpt_min(dc, index,
                                        flip_win->syncpt_max);
                }
        }

        /* unpin and deref previous front buffers */
        sfd_unpin_handles(unpin_handles, nr_unpin);

        /* now DC has submitted buffer for display, try to release fbmem */
        tegra_fb_release_fbmem(ext->dc->fb);

        kfree(data);
}

int sfd_flip(struct tegra_dc_ext_user *user,
                             struct tegra_dc_ext_flip_windowattr_v2 *win,
                             int win_num,
                             __u32 *syncpt_id, __u32 *syncpt_val,
                             int *syncpt_fd, __u16 *dirty_rect, u8 flip_flags,
                             struct tegra_dc_ext_flip_user_data *flip_user_data,
                             int nr_user_data)
{
        struct tegra_dc_ext *ext = user->ext;
        struct sfd_flip_data *data;
        int work_index = -1;
        __u32 post_sync_val = 0, post_sync_id = NVSYNCPT_INVALID;
        int i, ret = 0;
        bool has_timestamp = false;

        /* If display has been disconnected return with error. */
        if (!ext->dc->connected)
                return -1;

        ret = sanitize_flip_args(user, win, win_num, &dirty_rect);
        if (ret)
                return ret;

        data = kzalloc(sizeof(*data), GFP_KERNEL);
        if (!data)
                return -ENOMEM;

        INIT_WORK(&data->work, sfd_flip_worker);
        data->ext = ext;
        data->act_window_num = win_num;
        if (dirty_rect) {
                memcpy(data->dirty_rect, dirty_rect, sizeof(data->dirty_rect));
                data->dirty_rect_valid = true;
        }

        BUG_ON(win_num > tegra_dc_get_numof_dispwindows());

        ret = sfd_pin_windows(user, win, win_num,
                                     data->win, &has_timestamp,
                                     syncpt_fd != NULL);
        if (ret)
                goto fail_pin;

        sfd_read_user_data(data, flip_user_data, nr_user_data);

        /*
         * If this flip needs to update the current IMP settings, reserve
         * exclusive access to the COMMON channel. This call can potentially
         * block.
         */
        if (data->imp_dirty) {
                ret = tegra_dc_reserve_common_channel(ext->dc);
                if (ret) {
                        dev_err(&ext->dc->ndev->dev,
                        "%s: DC %d flip failed to reserve the COMMON channel\n",
                                __func__, ext->dc->ctrl_num);
                        goto fail_pin;
                }

                ret = tegra_dc_validate_imp_queue(ext->dc,
                                                        data->imp_session_id);
                if (ret) {
                        dev_err(&ext->dc->ndev->dev,
                                "Couldn't find corresponding PROPOSE\n");
                        goto fail_pin;
                }
        }

        ret = lock_windows_for_flip(user, win, win_num);
        if (ret)
                goto fail_pin;

        if (!ext->enabled) {
                ret = -ENXIO;
                goto unlock;
        }

        BUG_ON(win_num > tegra_dc_get_numof_dispwindows());

        for (i = 0; i < win_num; i++) {
                u32 syncpt_max;
                int index = win[i].index;
                struct tegra_dc_ext_win *ext_win;

                if (index < 0 || !test_bit(index, &ext->dc->valid_windows))
                        continue;

                ext_win = &ext->win[index];

                syncpt_max = tegra_dc_incr_syncpt_max(ext->dc, index);

                data->win[i].syncpt_max = syncpt_max;

                /*
                 * Any of these windows' syncpoints should be equivalent for
                 * the client, so we just send back an arbitrary one of them
                 */
                post_sync_val = syncpt_max;
                post_sync_id = tegra_dc_get_syncpt_id(ext->dc, index);

                work_index = index;

                atomic_inc(&ext->win[work_index].nr_pending_flips);
        }
        if (work_index < 0) {
                ret = -EINVAL;
                goto unlock;
        }

        work_index = ffs(ext->dc->valid_windows);
        if (!work_index) {
                dev_err(&ext->dc->ndev->dev, "no valid window\n");
                ret = -EINVAL;
                goto unlock;
        }
        work_index -= 1; /* window index starts from 0 */

        if (syncpt_fd) {
                if (post_sync_id != NVSYNCPT_INVALID) {
                        ret = nvhost_syncpt_create_fence_single_ext(
                                        ext->dc->ndev, post_sync_id,
                                        post_sync_val + 1, "flip-fence",
                                        syncpt_fd);
                        if (ret) {
                                dev_err(&ext->dc->ndev->dev,
                                        "Failed creating fence err:%d\n", ret);
                                goto unlock;
                        }
                }
        } else {
                *syncpt_val = post_sync_val;
                *syncpt_id = post_sync_id;
        }

        //trace_flip_rcvd_syncpt_upd(post_sync_val);

        data->flags = flip_flags;

        queue_work(ext->win[work_index].flip_wq, &data->work);

        unlock_windows_for_flip(user, win, win_num);

        return 0;

unlock:
        unlock_windows_for_flip(user, win, win_num);

fail_pin:

        for (i = 0; i < win_num; i++) {
                int j;
                for (j = 0; j < TEGRA_DC_NUM_PLANES; j++) {
                        if (!data->win[i].handle[j])
                                continue;

                        dma_buf_unmap_attachment(data->win[i].handle[j]->attach,
                                data->win[i].handle[j]->sgt, DMA_TO_DEVICE);
                        dma_buf_detach(data->win[i].handle[j]->buf,
                                data->win[i].handle[j]->attach);
                        dma_buf_put(data->win[i].handle[j]->buf);
                        kfree(data->win[i].handle[j]);
                }
        }

        /* Release the COMMON channel in case of failure. */
        if (data->imp_dirty)
                tegra_dc_release_common_channel(ext->dc);

        kfree(data);

        return ret;
}

int sfd_restore(struct tegra_dc_ext *ext)
{
        int nwins = tegra_dc_get_numof_dispwindows();
        struct tegra_dc_win *wins[nwins];
        int i, nr_win = 0;

        for_each_set_bit(i, &ext->dc->valid_windows,
                        tegra_dc_get_numof_dispwindows())
                if (ext->win[i].enabled) {
                        wins[nr_win] = tegra_dc_get_window(ext->dc, i);
                        wins[nr_win++]->flags |= TEGRA_WIN_FLAG_ENABLED;
                }

        if (nr_win) {
                tegra_dc_update_windows(&wins[0], nr_win, NULL, true);
                tegra_dc_sync_windows(&wins[0], nr_win);
                tegra_dc_program_bandwidth(ext->dc, true);
        }

        return nr_win;
}

static void set_enable(struct tegra_dc_ext *ext, bool en)
{
        int i;

        /*
         * Take all locks to make sure any flip requests or cursor moves are
         * out of their critical sections
         */
        for (i = 0; i < ext->dc->n_windows; i++)
                mutex_lock_nested(&ext->win[i].lock, i);
        mutex_lock(&ext->cursor.lock);

        ext->enabled = en;

        mutex_unlock(&ext->cursor.lock);
        for (i = ext->dc->n_windows - 1; i >= 0 ; i--)
                mutex_unlock(&ext->win[i].lock);
}

void sfd_enable(struct tegra_dc_ext *ext)
{
        set_enable(ext, true);
}

int sfd_disable(struct tegra_dc_ext *ext)
{
        int i;
        unsigned long int windows = 0;

        set_enable(ext, false);

        /* Flush any scanline work */
        flush_workqueue(ext->scanline_wq);

        /*
         * Disable vblank requests
         */
        sfd_set_vblank(ext, false);

        /*
         * Flush the flip queue -- note that this must be called with dc->lock
         * unlocked or else it will hang.
         */
        for (i = 0; i < ext->dc->n_windows; i++) {
                struct tegra_dc_ext_win *win = &ext->win[i];

                flush_workqueue(win->flip_wq);
        }

        /*
         * Blank all windows owned by dcext driver, unpin buffers that were
         * removed from screen, and advance syncpt.
         */
        if (ext->dc->enabled) {
                for (i = 0; i < tegra_dc_get_numof_dispwindows(); i++) {
                        if (ext->win[i].user)
                                windows |= BIT(i);
                }

                tegra_dc_blank_wins(ext->dc, windows);
                if (!IS_ENABLED(CONFIG_FRAMEBUFFER_CONSOLE)) {
                        for_each_set_bit(i, &windows,
                                        tegra_dc_get_numof_dispwindows()) {
                                sfd_unpin_window(&ext->win[i]);
                        }
                }
        }

        return !!windows;
}

