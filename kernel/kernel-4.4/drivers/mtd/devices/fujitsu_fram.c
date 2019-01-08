/*
 * MTD SPI driver for qspi FRAM flash chips
 *
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>

#include <linux/mtd/cfi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/of_platform.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/qspi_mtd.h>

struct qspi_fram {
	struct qspi	*fram_flash;
	bool		is_lc_mode_set;
} glbl_qspi_fram;

/*
 * NOTE: Below Macro is used to optimize the QPI/QUAD mode switch logic...
 * - QPI/QUAD mode is used for flash write. QUAD mode is used for flash read.
 * - When QPI is enabled, QUAD is don't care.
 * - If below macro is disabled...
 *  o QPI/QUAD mode is enabled/disabled at the start/end of each flash write
 *    function call.
 *  o QUAD mode is enabled/disabled at the start/end of each flash read
 *    function call.
 * - If below macro is enabled...
 *  o QPI/QUAD mode is enabled at the start of flash write. QPI/QUAD mode is
 *    disabled whenever erase is invoked. QPI mode is disabled on read.
 *  o QUAD mode is enabled at the start of flash read. QUAD mode is disabled
 *    whenever erase is invoked. QUAD is don't care in QPI mode.
 */

#define COMMAND_WIDTH				1
#define ADDRESS_WIDTH				4
#define WE_RETRY_COUNT				200
#define WIP_RETRY_COUNT				2000000
#define BITS8_PER_WORD				8
#define FRAM_CMD_WRSR				0x01
#define FRAM_LC_MODE_0				0x00

#define FILL_TX_BUF_LEN(tx, buf, nlen)			\
		tx.len = nlen,				\
		tx.tx_buf = buf

#define FILL_RX_BUF_LEN(rx, buf, nlen)			\
		rx.len = nlen,				\
		rx.rx_buf = buf

#define JEDEC_8(jedec) ((jedec) << 8)

static int qspi_write_en(struct qspi *flash,
		uint8_t is_enable, uint8_t is_sleep);
static int wait_till_ready(struct qspi *flash, uint8_t is_sleep);

static inline struct qspi *mtd_to_qspi(struct mtd_info *mtd)
{
	return container_of(mtd, struct qspi, mtd);
}

/*
 * Set Mode for transfer request
 * Function sets Bus width, DDR/SDR and opcode
 */

static void set_mode(struct spi_transfer *tfr, uint8_t is_ddr,
		uint8_t bus_width, uint8_t op_code)
{
	tfr->delay_usecs = set_op_mode(op_code) | set_bus_width(bus_width);
	if (is_ddr)
		tfr->delay_usecs |= set_sdr_ddr;
}

/*
 * Copy Parameters from default command table
 * Command table contains command, address and data
 * related information associated with opcode
 */

static void copy_cmd_default(struct qcmdset *qcmd, struct qcmdset *cmd_table)
{
	qcmd->qcmd.op_code = cmd_table->qcmd.op_code;
	qcmd->qcmd.is_ddr = cmd_table->qcmd.is_ddr;
	qcmd->qcmd.bus_width = cmd_table->qcmd.bus_width;
	qcmd->qaddr.address = cmd_table->qaddr.address;
	qcmd->qaddr.is_ddr = cmd_table->qaddr.is_ddr;
	qcmd->qaddr.len = cmd_table->qaddr.len;
	qcmd->qaddr.bus_width = cmd_table->qaddr.bus_width;
	qcmd->qaddr.dummy_cycles = cmd_table->qaddr.dummy_cycles;
	qcmd->qdata.is_ddr = cmd_table->qdata.is_ddr;
	qcmd->qdata.bus_width = cmd_table->qdata.bus_width;
}

/*
 * Copy Parameters from default command table
 * Command table contains command, address and data
 * related information associated with opcode
 */

static int read_sr1_reg(struct qspi *flash, uint8_t *regval)
{
	uint8_t tx_buf[1], rx_buf[1];
	int status = PASS, err;
	struct spi_transfer t[2];
	struct spi_message m;

	spi_message_init(&m);

	memset(t, 0, sizeof(t));
	tx_buf[0] = FRAM_CMD_RDSR;
	FILL_TX_BUF_LEN(t[0], tx_buf, COMMAND_WIDTH);
	t[0].bits_per_word = BITS8_PER_WORD;
	set_mode(&t[0], FALSE, flash->curr_cmd_mode, FRAM_CMD_RDSR);

	spi_message_add_tail(&t[0], &m);
	FILL_RX_BUF_LEN(t[1], rx_buf, COMMAND_WIDTH);
	t[1].bits_per_word = BITS8_PER_WORD;
	set_mode(&t[1], FALSE, flash->curr_cmd_mode, FRAM_CMD_RDSR);

	spi_message_add_tail(&t[1], &m);

	err = spi_sync(flash->spi, &m);
	if (err < 0) {
		pr_err("error: %s spi_sync call failed %d", __func__, status);
		status = FAIL;
	}

	*regval = rx_buf[0];
	return status;
}

/*
 * Enable/ Disable Write Enable Bit in Configuration Register
 * Set WEL bit to 1 before Erase and Write Operations
 */

static int qspi_write_en(struct qspi *flash,
		uint8_t is_enable, uint8_t is_sleep)
{
	struct spi_transfer t[1];
	uint8_t cmd_buf[5], regval;
	int status = 0, err, tried = 0, comp;
	struct spi_message m;

	do {
		if (tried++ == WE_RETRY_COUNT) {
			pr_err("tried max times not changing WE bit\n");
			return FAIL;
		}
		memset(t, 0, sizeof(t));
		spi_message_init(&m);

		if (is_enable) {
			cmd_buf[0] = FRAM_CMD_WREN;
			comp = WEL_ENABLE;
		} else {
			cmd_buf[0] = OPCODE_WRITE_DISABLE;
			comp = WEL_DISABLE;
		}

		FILL_TX_BUF_LEN(t[0], cmd_buf, COMMAND_WIDTH);
		t[0].bits_per_word = BITS8_PER_WORD;

		set_mode(&t[0], FALSE, flash->curr_cmd_mode,
				FRAM_CMD_RDSR);

		spi_message_add_tail(&t[0], &m);

		err = spi_sync(flash->spi, &m);
		if (err < 0) {
			pr_err("error: %s spi_sync call failed 0x%x",
				__func__, status);
			return err;
		}

		status = read_sr1_reg(flash, &regval);
		if (status) {
			pr_err("error: %s: RDSR failed: Status: 0x%x ",
				__func__, status);
			return status;
		}
	} while ((regval & WEL_ENABLE) != comp);

	return status;
}

/*
 * Wait till flash is ready for write operations.
 * Returns negative if error occurred.
 */

static int wait_till_ready(struct qspi *flash, uint8_t is_sleep)
{
	uint8_t regval, status = PASS;
	int tried = 0;

	do {
		if (tried++ == WIP_RETRY_COUNT) {
			pr_err("Tried max times not changing WIP bit\n");
			return FAIL;
		}

		status = read_sr1_reg(flash, &regval);
		if (status) {
			pr_err("error: %s: FRAM_CMD_RDSR failed: Status: 0x%x ",
				__func__, status);
			return status;
		}
	} while ((regval & WIP_ENABLE) == WIP_ENABLE);

	return status;
}

int _qspi_read(struct qspi *flash, loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
	struct spi_transfer t[3];
	struct spi_message m;
	uint8_t merge_cmd_addr = FALSE;
	uint8_t cmd_addr_buf[5];
	int addr_len, err;
	uint8_t status, regval;

	pr_debug("%s: %s from 0x%08x, len %zd\n",
		dev_name(&flash->spi->dev),
		__func__, (u32)from, len);

	spi_message_init(&m);

	if (!glbl_qspi_fram.is_lc_mode_set) {
		memset(t, 0, sizeof(t));
		err = qspi_write_en(flash, TRUE, FALSE);
		if (err) {
			pr_err("error: %s: WE failed: Status: x%x ",
				__func__, err);
		}

		cmd_addr_buf[0] = FRAM_CMD_WRSR;
		cmd_addr_buf[1] = FRAM_LC_MODE_0;
		FILL_TX_BUF_LEN(t[0], cmd_addr_buf, 2);
		spi_message_add_tail(&t[0], &m);
		t[0].cs_change = TRUE;
		spi_sync(flash->spi, &m);

		status = read_sr1_reg(flash, &regval);
		if (status) {
			pr_err("error: %s: RDSR failed: Status: 0x%x ",
				__func__, status);
		}
		glbl_qspi_fram.is_lc_mode_set = TRUE;
	}

	memset(t, 0, sizeof(t));
	/* take lock here to protect race condition
	 * in case of concurrent read and write with
	 * different cmd_mode selection.
	 */
	mutex_lock(&flash->lock);

	/* Set Controller data Parameters
	 * Set DDR/SDR, X1/X4 and Dummy Cycles from DT
	 */
	copy_cmd_default(&flash->cmd_table,
				&cmd_info_table[QPI_FRQAD]);
	/* check if possible to merge cmd and address */
	if ((flash->cmd_table.qcmd.is_ddr ==
		flash->cmd_table.qaddr.is_ddr) &&
		(flash->cmd_table.qcmd.bus_width ==
		flash->cmd_table.qaddr.bus_width) &&
		flash->cmd_table.qcmd.post_txn > 0) {

		merge_cmd_addr = TRUE;
		flash->cmd_table.qcmd.post_txn =
			flash->cmd_table.qcmd.post_txn - 1;
	}
	cmd_addr_buf[0] = flash->cmd_table.qcmd.op_code;
	cmd_addr_buf[1] = (from >> 16) & 0xFF;
	cmd_addr_buf[2] = (from >> 8) & 0xFF;
	cmd_addr_buf[3] = from & 0xFF;
	cmd_addr_buf[4] = 0xFF;

	if (merge_cmd_addr) {

		addr_len = (flash->cmd_table.qaddr.len + 1
			+ (flash->cmd_table.qaddr.dummy_cycles/8));
		FILL_TX_BUF_LEN(t[0], cmd_addr_buf, addr_len);

		set_mode(&t[0],
			flash->cmd_table.qcmd.is_ddr,
			flash->cmd_table.qcmd.bus_width,
			flash->cmd_table.qcmd.op_code);
		spi_message_add_tail(&t[0], &m);

		FILL_RX_BUF_LEN(t[1], buf, 20);
		set_mode(&t[1],
			flash->cmd_table.qdata.is_ddr,
			flash->cmd_table.qdata.bus_width,
			flash->cmd_table.qcmd.op_code);

		/* in-activate the cs at the end */
		t[1].cs_change = TRUE;
		spi_message_add_tail(&t[1], &m);

	} else {
		FILL_TX_BUF_LEN(t[0], &cmd_addr_buf[0], 1);

		set_mode(&t[0],
			flash->cmd_table.qcmd.is_ddr,
			flash->cmd_table.qcmd.bus_width,
			flash->cmd_table.qcmd.op_code);

		spi_message_add_tail(&t[0], &m);

		addr_len = (flash->cmd_table.qaddr.len + /* 1 +	*/
			(flash->cmd_table.qaddr.dummy_cycles/8));

		FILL_TX_BUF_LEN(t[1], &cmd_addr_buf[1], addr_len);
		set_mode(&t[1],
			flash->cmd_table.qaddr.is_ddr,
			flash->cmd_table.qaddr.bus_width,
			flash->cmd_table.qcmd.op_code);

		spi_message_add_tail(&t[1], &m);

		FILL_RX_BUF_LEN(t[2], buf, len);
		set_mode(&t[2],
			flash->cmd_table.qdata.is_ddr,
			flash->cmd_table.qdata.bus_width,
			flash->cmd_table.qcmd.op_code);

		/* in-activate the cs at the end */
		t[2].cs_change = TRUE;
		spi_message_add_tail(&t[2], &m);
	}

	spi_sync(flash->spi, &m);
	*retlen = m.actual_length -
		(flash->cmd_table.qaddr.len + 1 +
		(flash->cmd_table.qaddr.dummy_cycles/8));

	mutex_unlock(&flash->lock);
	return 0;
}

int _qspi_write(struct qspi *flash, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	struct spi_transfer t[2];
	struct spi_message m;
	uint8_t cmd_addr_buf[5];
	uint8_t opcode;
	int err = 0, addr_len;
	u32 offset = (unsigned long)to;

	pr_debug("%s: %s to 0x%08x, len %zd\n", dev_name(&flash->spi->dev),
			__func__, (u32)to, len);

	mutex_lock(&flash->lock);

	/* Set Controller data Parameters
	 * Set DDR/SDR, X1/X4 and Dummy Cycles from DT
	 */

	copy_cmd_default(&flash->cmd_table,
			&cmd_info_table[QPI_WQAD]);

	/* op-code */
	cmd_addr_buf[0] = opcode = flash->cmd_table.qcmd.op_code;
	/* address */
	cmd_addr_buf[1] = (offset >> 16) & 0xFF;
	cmd_addr_buf[2] = (offset >> 8) & 0xFF;
	cmd_addr_buf[3] = offset & 0xFF;

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	addr_len = flash->cmd_table.qaddr.len + 1;
	FILL_TX_BUF_LEN(t[0], cmd_addr_buf, addr_len);
	t[0].bits_per_word = BITS8_PER_WORD;
	set_mode(&t[0], flash->cmd_table.qcmd.is_ddr,
		flash->cmd_table.qcmd.bus_width, opcode);

	spi_message_add_tail(&t[0], &m);

	FILL_TX_BUF_LEN(t[1], buf, len);
	set_mode(&t[1], flash->cmd_table.qcmd.is_ddr,
		flash->cmd_table.qdata.bus_width, opcode);

	t[1].cs_change = TRUE;
	spi_message_add_tail(&t[1], &m);

	err = qspi_write_en(flash, TRUE, FALSE);
	if (err) {
		pr_err("error: %s: WE failed: Status: x%x ",
			__func__, err);
		goto clear_qmode;
	}

	spi_sync(flash->spi, &m);

	err = wait_till_ready(flash, FALSE);
	if (err) {
		pr_err("error: %s: WIP failed: Status: x%x ",
			__func__, err);
		goto clear_qmode;
	}

	*retlen = m.actual_length - (flash->cmd_table.qaddr.len + 1);
clear_qmode:

	mutex_unlock(&flash->lock);
	return err;
}

int qspi_read(loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
	if (!glbl_qspi_fram.fram_flash)
		return -EINVAL;

	return _qspi_read(glbl_qspi_fram.fram_flash, from, len, retlen, buf);
}
EXPORT_SYMBOL_GPL(qspi_read);

int qspi_write(loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	if (!glbl_qspi_fram.fram_flash)
		return -EINVAL;

	return _qspi_write(glbl_qspi_fram.fram_flash, to, len, retlen, buf);
}
EXPORT_SYMBOL_GPL(qspi_write);

/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int qspi_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
	struct qspi *flash = mtd_to_qspi(mtd);

	return _qspi_read(flash, from, len, retlen, buf);
}

static int qspi_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	struct qspi *flash = mtd_to_qspi(mtd);

	return _qspi_write(flash, to, len, retlen, buf);
}

static const struct spi_device_id *jedec_probe(struct spi_device *spi)
{
	int			tmp;
	u8			code = OPCODE_RDID;
	u8			id[5];
	u32			jedec;
	u16			ext_jedec;
	struct flash_info	*info;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 */
	tmp = spi_write_then_read(spi, &code, 1, id, 5);
	if (tmp < 0) {
		pr_debug("%s: error %d reading JEDEC ID\n",
				dev_name(&spi->dev), tmp);
		return ERR_PTR(tmp);
	}

	jedec = id[0];
	jedec = JEDEC_8(jedec);
	jedec |= id[1];
	jedec = JEDEC_8(jedec);
	jedec |= id[2];

	ext_jedec = id[3] << 8 | id[4];

	pr_info("%s: JEDEC id %x\n", dev_name(&spi->dev), jedec);
	pr_info("%s: JEDEC ext id %x\n", dev_name(&spi->dev), ext_jedec);
	for (tmp = 0; tmp < ARRAY_SIZE(qspi_ids_fram) - 1; tmp++) {
		info = (void *)qspi_ids_fram[tmp].driver_data;

		if (info->jedec_id == jedec) {
			if (info->ext_id != 0 && info->ext_id != ext_jedec)
				continue;
			return &qspi_ids_fram[tmp];
		}
	}
	pr_err("unrecognized JEDEC id %06x\n", jedec);
	return ERR_PTR(-ENODEV);
}

static int qspi_probe(struct spi_device *spi)
{
	const struct spi_device_id	*id;
	struct flash_platform_data	*data;
	struct qspi			*flash;
	struct flash_info		*info;
	unsigned			i;
	struct mtd_part_parser_data	ppdata;
	struct device_node __maybe_unused *np;
	struct tegra_qspi_device_controller_data *cdata = spi->controller_data;
	int ret;

	id = spi_get_device_id(spi);
	np = spi->dev.of_node;

#ifdef CONFIG_MTD_OF_PARTS
	if (!of_device_is_available(np))
		return -ENODEV;
#endif

	data = spi->dev.platform_data;
	if (data && data->type) {
		const struct spi_device_id *plat_id;

		for (i = 0; i < ARRAY_SIZE(qspi_ids) - 1; i++) {
			plat_id = &qspi_ids[i];
			if (strncmp(data->type, plat_id->name,
				sizeof(plat_id->name) - 1))
				continue;
			break;
		}

		if (i < ARRAY_SIZE(qspi_ids) - 1)
			id = plat_id;
		else
			dev_warn(&spi->dev, "unrecognized id %s\n", data->type);
	}

	info = (void *)id->driver_data;

	if (info->jedec_id) {
		const struct spi_device_id *jid;

		jid = jedec_probe(spi);
		if (IS_ERR(jid)) {
			return PTR_ERR(jid);
		} else if (jid != id) {
			dev_warn(&spi->dev, "found %s, expected %s\n",
					jid->name, id->name);
			id = jid;
			info = (void *)jid->driver_data;
		}
	}

	flash = devm_kzalloc(&spi->dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	flash->spi = spi;
	flash->flash_info = info;
	mutex_init(&flash->lock);

	dev_set_drvdata(&spi->dev, flash);

	if (data && data->name)
		flash->mtd.name = data->name;
	else
		flash->mtd.name = dev_name(&spi->dev);

	flash->mtd.type = MTD_FRAM;
	flash->mtd.writesize = 1;
	flash->mtd.flags = MTD_CAP_NVRAM;
	flash->mtd.size = info->sector_size * info->n_sectors;
	flash->mtd._read = qspi_mtd_read;
	flash->mtd._write = qspi_mtd_write;
	flash->mtd.erasesize = info->sector_size;

	ppdata.of_node = spi->dev.of_node;
	flash->mtd.dev.parent = &spi->dev;
	flash->page_size = info->page_size;
	flash->mtd.writebufsize = flash->page_size;

	flash->addr_width = ADDRESS_WIDTH;
	cdata = flash->spi->controller_data;

	if (info->n_subsectors) {
		info->ss_endoffset = info->ss_soffset +
					info->ss_size * info->n_subsectors;
		if (info->ss_endoffset > flash->mtd.size) {
			dev_err(&spi->dev, "%s SSErr %x %x %x %llx\n", id->name,
				info->n_subsectors, info->ss_soffset,
					info->ss_size, flash->mtd.size);
			return -EINVAL;
		}
		dev_info(&spi->dev, "%s SSG %x %x %x %llx\n", id->name,
				info->n_subsectors, info->ss_soffset,
					info->ss_size, flash->mtd.size);
	}

	dev_info(&spi->dev, "%s (%lld Kbytes)\n", id->name,
			(long long)flash->mtd.size >> 10);

	dev_info(&spi->dev,
	"mtd .name = %s, .size = 0x%llx (%lldMiB) .erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		flash->mtd.name,
		(long long)flash->mtd.size, (long long)(flash->mtd.size >> 20),
		flash->mtd.erasesize, flash->mtd.erasesize / 1024,
		flash->mtd.numeraseregions);

	if (flash->mtd.numeraseregions)
		for (i = 0; i < flash->mtd.numeraseregions; i++)
			dev_info(&spi->dev,
			"mtd.eraseregions[%d] ={.offset = 0x%llx,.erasesize = 0x%.8x (%uKiB),.numblocks = %d}\n",
			i, (long long)flash->mtd.eraseregions[i].offset,
			flash->mtd.eraseregions[i].erasesize,
			flash->mtd.eraseregions[i].erasesize / 1024,
			flash->mtd.eraseregions[i].numblocks);

	ret = mtd_device_parse_register(&flash->mtd, NULL, &ppdata,
			data ? data->parts : NULL,
			data ? data->nr_parts : 0);
	if (ret) {
		dev_err(&spi->dev, "%s: misc-device registration failed",
						id->name);
		return ret;
	}

	glbl_qspi_fram.fram_flash = flash;
	return ret;
}

static int qspi_remove(struct spi_device *spi)
{
	struct qspi	*flash = dev_get_drvdata(&spi->dev);
	int		status;

	/* Clean up MTD stuff. */
	status = mtd_device_unregister(&flash->mtd);

	return status;
}

static struct spi_driver qspi_mtd_driver = {
	.driver = {
		.name	= "qspi_mtd_fram",
		.owner	= THIS_MODULE,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table	= qspi_ids_fram,
	.probe	= qspi_probe,
	.remove	= qspi_remove,

};
module_spi_driver(qspi_mtd_driver);

MODULE_AUTHOR("Jeetesh Burman <jburman@nvidia.com>");
MODULE_DESCRIPTION("MTD SPI driver for Fujitsu's FRAM QSPI chip");
MODULE_LICENSE("GPL v2");
