/*
 * qspi-mtd.h: Mtd SPI interface for Nvidia Tegra210 QSPI controller.
 *
 * Author: Mike Lavender, mike@steroidmicros.com
 * Copyright (c) 2005, Intec Automation Inc.
 * Copyright (C) 2013-2017 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _LINUX_QSPI_MTD_H
#define _LINUX_QSPI_MTD_H
#include <linux/spi/qspi-tegra.h>

struct qspi_cmd {
	uint8_t op_code;
	uint8_t is_ddr;
	uint8_t bus_width;
	uint8_t post_txn;
};

struct qspi_addr {
	uint32_t address;
	uint8_t is_ddr;
	uint8_t len;
	uint8_t bus_width;
	uint8_t dummy_cycles;
};

struct qspi_data {
	uint8_t is_ddr;
	uint8_t bus_width;
};

struct qcmdset {
	struct qspi_cmd qcmd;
	struct qspi_addr qaddr;
	struct qspi_data qdata;
};

enum qspi_operation_mode {
	NORMAL_READ_4,
	NORMAL_READ_3,
	FAST_READ,
	DUAL_OUT_READ,
	QUAD_OUT_READ,
	DUAL_IO_READ,
	QUAD_IO_READ_4,
	QUAD_IO_READ_3,
	DDR_FAST_READ,
	DDR_DUAL_IO_READ,
	DDR_QUAD_IO_READ,
	PAGE_PROGRAM_4,
	PAGE_PROGRAM_3,
	QUAD_PAGE_PROGRAM,
	QPI_PAGE_PROGRAM,
	READ_ID,
	ERASE_SECT,
	ERASE_BULK,
	STATUS_READ,
	READ_ANY_REG,
	WRITE_ANY_REG,
	QPI_FRQAD,
	QPI_WQAD,
	CMD_EQPI,
	CMD_RDSR,
	OPERATION_MAX_LIMIT,
};

enum qspi_status {
	PASS,
	FAIL,
};

enum qspi_reg_read_code {
	RDSR1 = 0x05,
	RDSR2 = 0x07,
	RDCR = 0x35,
};

enum boot {
	FALSE,
	TRUE,
};

static struct qcmdset cmd_info_table[OPERATION_MAX_LIMIT] = {
	/*  NORMAL_READ_4 */
	{ {.op_code = 0x13, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X1, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/* NORMAL_READ_3 */
	{ {.op_code = 0x03, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 3,
			.bus_width = X1, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/*  FAST_READ */
	{ {.op_code = 0x0c, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X1, .dummy_cycles = 8},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/* DUAL_OUT_READ */
	{ {.op_code = 0x3c, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X1, .dummy_cycles = 8},
		{.is_ddr = FALSE, .bus_width = X2}
	},
	/* QUAD_OUT_READ */
	{ {.op_code = 0x6c, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X1, .dummy_cycles = 8},
		{.is_ddr = FALSE, .bus_width = X4}
	},
	/* DUAL_IO_READ */
	{ {.op_code = 0xBC, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X2, .dummy_cycles = 24},
		{.is_ddr = FALSE, .bus_width = X2}
	},
	/* QUAD_IO_READ_4 */
	{ {.op_code = 0xEC, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X4, .dummy_cycles = 40},
		{.is_ddr = FALSE, .bus_width = X4}
	},
	/* QUAD_IO_READ_3 */
	{ {.op_code = 0xEB, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 3,
			.bus_width = X4, .dummy_cycles = 40},
		{.is_ddr = FALSE, .bus_width = X4}
	},
	/* DDR_FAST_READ */
	{ {.op_code = 0x0E, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = TRUE, .len = 4,
			.bus_width = X1, .dummy_cycles = 6},
		{.is_ddr = TRUE, .bus_width = X1}
	},
	/* DDR_DUAL_IO_READ */
	{ {.op_code = 0xBE, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = TRUE, .len = 4,
			.bus_width = X2, .dummy_cycles = 24},
		{.is_ddr = TRUE, .bus_width = X2}
	},
	/* DDR_QUAD_IO_READ  Spansion - 56 Micron - 64 Dummy Cycles */
	{ {.op_code = 0xEE, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = TRUE, .len = 4,
			.bus_width = X4, .dummy_cycles = 72},
		{.is_ddr = TRUE, .bus_width = X4}
	},
	/* PAGE_PROGRAM_4 */
	{ {.op_code = 0x12, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X1, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/* PAGE_PROGRAM_3 */
	{ {.op_code = 0x02, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 3,
			.bus_width = X1, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/* QUAD_PAGE_PROGRAM */
	{ {.op_code = 0x34, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X1, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X4}
	},
	/* QPI_PAGE_PROGRAM */
	{ {.op_code = 0x12, .is_ddr = FALSE, .bus_width = X4, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X4, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X4}
	},
	/* READ ID*/
	{ {.op_code = 0x90, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 3,
			.bus_width = X1, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/* ERASE SECT */
	{ {.op_code = 0xdc, .is_ddr = FALSE, .bus_width = X1, .post_txn = 1},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X1, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/*  bulk erase*/
	{ {.op_code = 0x60, .is_ddr = FALSE, .bus_width = X1, .post_txn = 0},
		{.address = 0, .is_ddr = FALSE, .len = 0,
			.bus_width = X1, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/* STATUS READ */
	{ {.op_code = 0x01, .is_ddr = FALSE, .bus_width = X1, .post_txn = 1},
		{.address = 0, .is_ddr = FALSE, .len = 4,
			.bus_width = X1, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/* READ_ANY_REG */
	{ {.op_code = 0x65, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 3,
			.bus_width = X1, .dummy_cycles = 1},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/* WRITE_ANY_REG */
	{ {.op_code = 0x71, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 3,
			.bus_width = X1, .dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X1}
	},
	/*  QPI_FRQAD */
	{ {.op_code = 0xeb, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 4, .bus_width = X4,
			.dummy_cycles = 24},
		{.is_ddr = FALSE, .bus_width = X4}
	},
	/*  QPI_WQAD */
	{ {.op_code = 0x32, .is_ddr = FALSE, .bus_width = X1, .post_txn = 2},
		{.address = 0, .is_ddr = FALSE, .len = 3, .bus_width = X4,
			.dummy_cycles = 0},
		{.is_ddr = FALSE, .bus_width = X4}
	},
};

/* Flash opcodes. */
#define	OPCODE_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define	OPCODE_SE		0xdc	/* Sector erase (usually 256KiB) */
#define	OPCODE_SE_3		0xd8	/* Sector erase for 3-byte addr */
#define	OPCODE_RDID		0x9f	/* Read JEDEC ID */
#define QUAD_ENABLE		0x02    /* Enable Quad bit */
#define QUAD_DISABLE		0x00    /* Disable Quad bit */
#define OPCODE_WRR		0x01    /* Write Registers */
#define OPCODE_WRITE_ENABLE	0x06    /* Write Enable */
#define OPCODE_WRITE_DISABLE	0x04    /* Write Disable */
#define WEL_ENABLE		0x02    /* Enable WEL bit */
#define WEL_DISABLE		0x00    /* Disable WEL bit */
#define WIP_ENABLE		0x01    /* Enable WIP bit */

#define FRAM_CMD_RDID		0x9f
#define FRAM_CMD_RDSR		0x05
#define FRAM_CMD_EQPI		0x38
#define FRAM_CMD_DQPI		0xff
#define FRAM_CMD_WREN		0x06
#define FRAM_CMD_WQAD		0x12
#define FRAM_CMD_FRQAD		0xeb

/* JEDEC flash manufacturer id */
#define JEDEC_MFR(_jedec_id)	((_jedec_id) >> 16)
#define JEDEC_SPANSION		0x01
#define JEDEC_MICRON		0x20

#define INFO(_jedec_id, _ext_id, _sector_size,		\
		 _n_sectors, _subsector_size,		\
		_n_subsectors, _subsectors_soffset,	\
		_ss_erase_opcode, _page_size, _flags)	\
	((kernel_ulong_t)&(struct flash_info) {		\
	 .jedec_id = (_jedec_id),			\
	 .ext_id = (_ext_id),				\
	 .sector_size = (_sector_size),			\
	 .n_sectors = (_n_sectors),			\
	 .ss_size = (_subsector_size),			\
	 .n_subsectors = (_n_subsectors),		\
	 .ss_soffset = (_subsectors_soffset),		\
	 .ss_erase_opcode = (_ss_erase_opcode),		\
	 .page_size = (_page_size),			\
	 .flags = (_flags),				\
	 })

struct qspi {
	struct spi_device	*spi;
	struct mutex		lock;
	struct mtd_info		mtd;
	u16			page_size;
	u16			addr_width;
	u8			erase_opcode;
	struct qcmdset		cmd_table;
	u8			curr_cmd_mode;
	u8			is_quad_set;
	struct	flash_info	*flash_info;
};

/*
 * SPI device driver setup and teardown
 */

struct flash_info {
	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32		jedec_id;
	u16             ext_id;

	/* The size listed here is what works with OPCODE_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned	sector_size;
	u16		n_sectors;

	u16		n_subsectors;
	unsigned	ss_size;
	unsigned	ss_soffset;
	unsigned	ss_endoffset;
	unsigned	ss_erase_opcode;

	u16		page_size;
	u16		addr_width;

	u16		flags;
};

static const struct spi_device_id qspi_ids[] = {

	/* Spansion -- single (large) sector size only, at least
	 * for the chips listed here (without boot sectors).
	 */
	{	"s25fl128s",
		INFO(0x012018, 0, 64 * 1024, 256, 4 * 1024, 8, 0, 0x21, 256, 0)
	},
	{	"s25fs256s",
		INFO(0x010219, 0, 256 * 1024, 128, 4 * 1024, 8, 0, 0x21, 256, 0)
	},
	{	"s25fs512s",
		INFO(0x010220, 0, 256 * 1024, 256, 4 * 1024, 8, 0, 0x21, 512, 0)
	},
	{	"MT25QL512AB",
		INFO(0x20BA20, 0, 256 * 1024, 256, 0, 0, 0, 0, 256, 0)
	},
	{	"MT25QU128ABA",
		INFO(0x20BB18, 0x1044, 64 * 1024, 256, 0, 0, 0, 0, 256, 0)
	},
	{ },
};

static const struct spi_device_id qspi_ids_fram[] = {

	/* Fujitsu -- single (large) sector size only, at least
	 * for the chips listed here (without boot sectors).
	 */
	{	"MB85RQ4ML",
		INFO(0x047f29, 0, 512 * 1024, 8, 0, 0, 0, 0, 1024 * 20, 0)
	},
	{ },
};

MODULE_DEVICE_TABLE(spi, qspi_ids);

#endif /* _LINUX_QSPI_MTD_H */
