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

#ifndef SFD_T186_H
#define SFD_T186_H

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

#define TEGRA_DC_TS_MAX_DELAY_US 1000000
#define TEGRA_DC_TS_SLACK_US 2000

struct sfd_flip_win {
        struct tegra_dc_ext_flip_windowattr_v2  attr;
        struct tegra_dc_dmabuf                  *handle[TEGRA_DC_NUM_PLANES];
        dma_addr_t                              phys_addr;
        dma_addr_t                              phys_addr_u;
        dma_addr_t                              phys_addr_v;
        dma_addr_t                              phys_addr_cde;
        /* field 2 */
        dma_addr_t                              phys_addr2;
        dma_addr_t                              phys_addr_u2;
        dma_addr_t                              phys_addr_v2;
        u32                                     syncpt_max;
};

struct sfd_scanline_data {
        struct tegra_dc_ext             *ext;
        struct work_struct              work;
        int                             triggered_line;
        int                             max_val;
};

struct sfd_flip_data {
        struct tegra_dc_ext             *ext;
        struct work_struct              work;
        struct sfd_flip_win             win[DC_N_WINDOWS];
        struct list_head                timestamp_node;
        int act_window_num;
        u16 dirty_rect[4];
        bool dirty_rect_valid;
        u8 flags;
        struct tegra_dc_hdr hdr_data;
        bool hdr_cache_dirty;
        bool imp_dirty;
        u64 imp_session_id;
};

int sfd_get_window(struct tegra_dc_ext_user *user, unsigned int n);
int sfd_put_window(struct tegra_dc_ext_user *user, unsigned int n);
int sfd_check_windowattr(struct tegra_dc_ext *ext, struct tegra_dc_win *win);
void sfd_set_windowattr_basic(struct tegra_dc_win *win,
                       const struct tegra_dc_ext_flip_windowattr_v2 *flip_win);
int sfd_set_windowattr(struct tegra_dc_ext *ext,
                               struct tegra_dc_win *win,
                               const struct sfd_flip_win *flip_win);
int sfd_negotiate_bw(struct tegra_dc_ext_user *user,
                        struct tegra_dc_ext_flip_windowattr_v2 *wins,
                        int win_num);
int sfd_pin_windows(struct tegra_dc_ext_user *user,
                                struct tegra_dc_ext_flip_windowattr_v2 *wins,
                                int win_num,
                                struct sfd_flip_win *flip_wins,
                                bool *has_timestamp,
                                bool syncpt_fd);
void sfd_unpin_handles(struct tegra_dc_dmabuf *unpin_handles[],
                                       int nr_unpin);
void sfd_unpin_window(struct tegra_dc_ext_win *win);
int sfd_get_status(struct tegra_dc_ext_user *user,
                                   struct tegra_dc_ext_status *status);
int sfd_get_feature(struct tegra_dc_ext_user *user,
                                   struct tegra_dc_ext_feature *feature);
u32 sfd_get_vblank_syncpt(struct tegra_dc_ext_user *user);
int sfd_get_scanline(struct tegra_dc_ext *ext);
int sfd_set_vblank(struct tegra_dc_ext *ext, bool enable);
void sfd_setup_vpulse3(struct tegra_dc_ext *ext,
                                        unsigned int scanline_num);
void sfd_incr_vpulse3(struct tegra_dc_ext *ext);
int sfd_vpulse3(struct tegra_dc_ext *ext,
                                struct tegra_dc_ext_scanline_info *args);
void sfd_scanline_worker(struct work_struct *work);
void sfd_read_user_data(struct sfd_flip_data *data,
                        struct tegra_dc_ext_flip_user_data *flip_user_data,
                        int nr_user_data);
void sfd_flip_worker(struct work_struct *work);
int sfd_flip(struct tegra_dc_ext_user *user,
                             struct tegra_dc_ext_flip_windowattr_v2 *win,
                             int win_num,
                             __u32 *syncpt_id, __u32 *syncpt_val,
                             int *syncpt_fd, __u16 *dirty_rect, u8 flip_flags,
                             struct tegra_dc_ext_flip_user_data *flip_user_data,
                             int nr_user_data);
#endif /* SFD_T186_H */
