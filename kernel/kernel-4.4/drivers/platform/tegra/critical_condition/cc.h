/*
 * Copyright (C) 2017 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>

/*
 * WDT_TIMEOUT = 2, is based on drivers/soc/tegra/pmc.c
 * "char *t186_pmc_rst_src"
 */
#define WDT_TIMEOUT				2
#define MAX_RECORD_CNT				3
#define START_ADDRESS0				0
#define CC_PAGE_WRITE_STARTED			1

#define FALSE					0
#define TRUE					1

/*
 * This enum is used to update buffer status while writing
 * data to GameData RAM cvt, currently using 2-buffer
 * @GD_BUFFER			To select buffer among 2-buffer to write/read
 *				to/from GameData RAM cvt by toggling this
 *				element.
 * @GD_BUFFER_CONTROL_REG	To select control-region where current state of
 *				'struct gamedata_cvt_context' save into
 *				3rd buffer(partition) of carveout, to use across
 *				system boot, so that if last reset was WDT, cc
 *				driver can select right buffer to get user-data.
 */
enum gd_buffer_number {
	GD_BUFFER = 1,
	GD_BUFFER_CONTROL_REG,
};

/*
 * structure to maintain gamedata cvt
 * @cxt_valid		This structure will be save into cvt and
 *			while reading back, this variable will be used to
 *			check validity of saved data.
 * @gd_ram_buffs	Contains array of buffers, currently 2 buffer will
 *			be used.
 * @phys_addr		Start of the physical cvt address
 * @size		Size of the pysical cvt address
 * @max_record_cnt	No. of buffer will be used.
 * @flags		flags to check buffer status, using
 *			member of 'enum gd_buffer_status'
 */
struct gamedata_cvt_context {
	unsigned int cxt_valid;
	struct gd_ram_buffer **gd_ram_buffs;
	phys_addr_t phys_addr;
	unsigned long size;
	unsigned int max_record_cnt;
	size_t record_size;
	unsigned int flags;
};

/*
 * structure to maintain per buffer, currently 2-buffer will be used
 * @paddr		physical address of one buffer.
 * @mem_size		buffer size for one buffer,
 * @vaddr		cpu-address mapped via ioremap for paddr
 * @game_data		data to be write/read to/from GameData RAM carveout
 * @data_size		data-size to be write/read to/from GameData RAM carveout
 * @system_alive	to check whether system fresh boot or already running
 *			system
 */
struct gd_ram_buffer {
	phys_addr_t paddr;
	size_t mem_size;
	void *vaddr;
	char *game_data;
	size_t data_size;
	bool system_alive;
};

#define NO_CAP		(ULONG_MAX) /* no cap */

enum throttle_type {
	THROT_MCPU,
	THROT_BCPU,
	THROT_EMC,
	THROT_GPU,
	THROT_MAX_CAP_CLKS,
};

/* Tracks the throttle freq. for the given clk */
struct cc_throt_freqs {
	unsigned long cap_freq;
	enum throttle_type type;
};

/*
 * Critical Condition throttle data
 * @cdev		To support thermal cooling device.
 * @cdev_type		Type of critical-condition cooling-device, will be
 *			filled by dtb.
 * @cur_state		current state of throttling table.
 * @throt_table_size	Size of the throttle table.
 * @num_cap_clks	No. of clks need to throttle, currently
 *			bcpu, mcpu, gpu and emc clocks
 * @throt_table		Throttle table as defined by *.dtb
 */
struct cc_throttle {
	struct thermal_cooling_device *cdev;
	char *cdev_type;
	unsigned long cur_state;
	int throt_table_size;
	int num_cap_clks;
	u32 *throt_table;
};
/*
 * Critical Condition Carveout platform data, this will be fill
 * from *.dtb value
 * @mem_size		Total memory size for gamedata_cvt, currently 1MiB
 * @mem_address		physical memory address to contain gamedata_cvt
 * @read_write_bytes	data-size to be write/read to/from FRAM
 * @flags		flags to check write has been done till now
 * @cc_timer		timer_list that will be used to trigger at certain time
 * @cc_timer_timeout	timer trigger time in sec. fill from dtb
 * @cc_timer_timeout_jiffies	timer value in jiffies
 * @cc_timer_started	if timer already started do not modify timer
 * @cc_work		work to be schedule when timer triggers
 * @last_reset_status	to save last reset status in case of wdt-timeout
 * @under_volt_irq	this variable used to contain irq no. for under-volt
 * @under_volt_gpio	this variable used to contain gpio no. will be used to
 *			convert gpio_to_irq
 * @cc_throt		Pointer to critical-condition throttle structure.
 */
struct crtlcond_platform_data {
	unsigned long			cvt_mem_size;
	unsigned long			cvt_mem_address;
	u32				read_write_bytes;
	int				flags;
	struct timer_list		cc_timer;
	int				cc_timer_timeout;
	unsigned long			cc_timer_timeout_jiffies;
	bool				cc_timer_started;
	struct work_struct		cc_work;
	unsigned int			last_reset_status;
	int				under_volt_irq;
	int				under_volt_gpio;
	struct cc_throttle		*cc_throt;
};

struct gd_ram_buffer *gd_ram_new(phys_addr_t start, size_t size);
void gd_ram_free(struct gd_ram_buffer *grb);

int gd_ram_write(struct gd_ram_buffer *grb, const void *s,
	unsigned int count);

int gd_read_from_cvt(struct gd_ram_buffer *grb);
void gd_ram_free_data(struct gd_ram_buffer *grb);

int gamedata_cvt_write(const char *buf, size_t size);
ssize_t gamedata_cvt_read(size_t *count, char **buf);
int gamedata_cvt_probe(struct platform_device *pdev);

int qspi_write(loff_t to, size_t len,
		size_t *retlen, const u_char *buf);
