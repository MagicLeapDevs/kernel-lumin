/* Copyright (c) 2016-2017, Magic Leap, Inc. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include "nrf52_i2c.h"
#include "nrf52_fwupdate.h"

static struct device *nrf_dev;
static struct i2c_client *client;

int nrf52_untar_fw(struct device *dev, const uint8_t *fw_tar, int size,
					fw_archive_t *fw_archive)
{
	int file_length = 0;
	tarfile_t *tfile;
	int i;

	nrf_dev = dev;
	tfile = (tarfile_t *) fw_tar;

	for (i = 0; i < 3; i++) {
		if (kstrtoint(tfile->header.size, 8, &file_length) != 0) {
			dev_err(nrf_dev, "Invalid NRF52 firmware file size\n");
			return -1;
		}

		if (strstr(tfile->header.filename, ".json") != NULL) {
			fw_archive->manifest.length = file_length;
			fw_archive->manifest.file_name = tfile->header.filename;
			fw_archive->manifest.file = tfile->file;
		} else if (strstr(tfile->header.filename, ".dat") != NULL) {
			fw_archive->init_file.length = file_length;
			fw_archive->init_file.file_name =
					tfile->header.filename;
			fw_archive->init_file.file = tfile->file;
		} else if (strstr(tfile->header.filename, ".bin") != NULL) {
			fw_archive->image_file.length = file_length;
			fw_archive->image_file.file_name =
					tfile->header.filename;
			fw_archive->image_file.file = tfile->file;
		} else {
			dev_err(nrf_dev,
					"Invalid NRF52 firmware file format\n");
			return -1;
		}
		tfile = (tarfile_t *) ((char *) tfile + sizeof(tar_t)
				+ roundup(file_length, 512));
	}

	return 0;
}
uint32_t crc32_compute(uint8_t const *p_data, uint32_t size,
				uint32_t const *p_crc)
{
	uint32_t crc;
	uint32_t i;
	uint32_t j;

	crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
	for (i = 0; i < size; i++) {
		crc = crc ^ p_data[i];
		for (j = 8; j > 0; j--)
			crc = (crc >> 1) ^
				(0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
	}
	return ~crc;
}

uint8_t uint32_encode(uint32_t value, uint8_t *p_encoded_data)
{
	p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
	p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
	p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
	p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
	return sizeof(uint32_t);
}

uint32_t uint32_decode(const uint8_t *p_encoded_data)
{
	return ((((uint32_t)((uint8_t *) p_encoded_data)[0]) << 0)
			| (((uint32_t)((uint8_t *) p_encoded_data)[1]) << 8)
			| (((uint32_t)((uint8_t *) p_encoded_data)[2]) << 16)
			| (((uint32_t)((uint8_t *) p_encoded_data)[3]) << 24));
}

int nrf52_read(uint8_t *data, int size)
{
	int ret = 0;
	int retry = 0;
	uint8_t read_size = 0;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = 1;
	msg.buf = &read_size;
	ret = i2c_transfer(client->adapter, &msg, 1);

	while ((read_size == 0) && (retry++ < I2C_READ_RETRIES)) {
		/*dev_err(nrf_dev, "read_size: %d\n", read_size);*/
		ret = i2c_transfer(client->adapter, &msg, 1);
	}

	if ((ret < 0) || read_size > size) {
		dev_err(nrf_dev, "\nFailed to read size from i2c device: %d\n",
				ret);
		return -EIO;
	}

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = read_size;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		dev_err(nrf_dev, "\nFailed to read data from i2c device: %d\n",
				ret);
		return -EIO;
	}

	/*dev_err(nrf_dev, "i2c_transport_read: Read %d bytes\n", read_size);*/
	/*DumpHex(data, read_size);*/

	return read_size;
}

int nrf52_write(uint8_t *data, int size)
{
	struct i2c_msg msg;
	int ret = 0;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = size;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(nrf_dev, "%s: i2c send fail (%d)\n", __func__, ret);
	}

	return ret;
}

int nrf52_send_request(nrf_dfu_req_t *req)
{
	uint8_t send_buf[MAX_REQUEST];

	int idx = 0;
	int ret = 0;

	switch (req->req_type) {
	case NRF_DFU_OBJECT_OP_CREATE:
		send_buf[idx++] = I2C_DFU_CTRL;
		send_buf[idx++] = I2C_DFU_OP_CODE_CREATE_OBJECT;
		send_buf[idx++] = req->obj_type;
		uint32_encode(req->object_size, send_buf + 3);
		idx += 4;
		break;

	case NRF_DFU_OBJECT_OP_CRC:
		send_buf[idx++] = I2C_DFU_CTRL;
		send_buf[idx++] = I2C_DFU_OP_CODE_CALCULATE_CRC;
		break;

	case NRF_DFU_OBJECT_OP_EXECUTE:
		send_buf[idx++] = I2C_DFU_CTRL;
		send_buf[idx++] = I2C_DFU_OP_CODE_EXECUTE_OBJECT;
		break;

	case NRF_DFU_OBJECT_OP_SELECT:
		send_buf[idx++] = I2C_DFU_CTRL;
		send_buf[idx++] = I2C_DFU_OP_CODE_SELECT_OBJECT;
		send_buf[idx++] = req->obj_type;
		break;

	case NRF_DFU_OBJECT_OP_WRITE:
		if((req->req_len + 1) > MAX_REQUEST) {
			dev_err(nrf_dev, "Invalid object request.\n");
			return -1;
		}
		send_buf[idx++] = I2C_DFU_PKT;
		memcpy(send_buf + idx, req->p_req, req->req_len);
		idx += req->req_len;
		break;

	default:
		break;
	}

	ret = nrf52_write(send_buf, idx);

	return ret;
}

int nrf52_read_response(nrf_dfu_res_t *res)
{
	uint8_t m_recv_buffer[MAX_RECEIVE];
	int op_code = 0;
	int res_code = 0;
	int ret = 0;

	ret = nrf52_read(m_recv_buffer, sizeof(m_recv_buffer));
	if (ret < 0) {
		dev_err(nrf_dev, "Read result response failed.\n");
		return -1;
	}

	op_code = m_recv_buffer[1];
	res_code = m_recv_buffer[2];

	switch (res_code) {
	case NRF_DFU_RES_CODE_INVALID:
		dev_err(nrf_dev, " DFU request failed: Invalid opcode.\n");
		return res_code;

	case NRF_DFU_RES_CODE_OP_CODE_NOT_SUPPORTED:
		dev_err(nrf_dev, " DFU request failed: Opcode not supported.\n");
		return res_code;

	case NRF_DFU_RES_CODE_INVALID_PARAMETER:
		dev_err(nrf_dev, " DFU request failed: Missing or invalid parameter value.\n");
		return res_code;

	case NRF_DFU_RES_CODE_INSUFFICIENT_RESOURCES:
		dev_err(nrf_dev, " DFU request failed: Not enough memory for the data object.\n");
		return res_code;

	case NRF_DFU_RES_CODE_INVALID_OBJECT:
		dev_err(nrf_dev, " DFU request failed: Data object does not match the firmware and hardware requirements, the signature is missing, or parsing the command failed.\n");
		return res_code;

	case NRF_DFU_RES_CODE_UNSUPPORTED_TYPE:
		dev_err(nrf_dev, " DFU request failed: Not a valid object type for a Create request.\n");
		return res_code;

	case NRF_DFU_RES_CODE_OPERATION_NOT_PERMITTED:
		dev_err(nrf_dev, " DFU request failed: The state of the DFU process does not allow this operation.\n");
		return res_code;

	case NRF_DFU_RES_CODE_OPERATION_FAILED:
		dev_err(nrf_dev, " DFU request failed: Operation failed.\n");
		return res_code;

	case NRF_DFU_RES_CODE_UPDATE_NOT_REQUIRED:
		dev_err(nrf_dev, " Software matches. No need to update.\n");
		return res_code;

	case NRF_DFU_RES_CODE_INVALID_VERSION:
		dev_err(nrf_dev, " DFU request failed: Invalid software version. Must be higher than current.\n");
		return res_code;

	case NRF_DFU_RES_CODE_DEBUG_NOT_PERMITTED:
		dev_err(nrf_dev, " DFU request failed: Debug package cannot be flashed by non-debug bootloader.\n");
		return res_code;

	case NRF_DFU_RES_CODE_UNSUPPORTED_HW:
		dev_err(nrf_dev, " DFU request failed: Unsupported hardware version.\n");
		return res_code;

	case NRF_DFU_RES_CODE_INCOMPATIBLE_SD:
		dev_err(nrf_dev, " DFU request failed: Incompatible softdevice.\n");
		return res_code;

	case NRF_DFU_RES_CODE_EXT_ERROR:
		dev_err(nrf_dev, " DFU request failed: Extended error.\n");
		return res_code;

	case NRF_DFU_RES_CODE_SUCCESS:
	default:
		break;

	}

	switch (op_code) {
	case I2C_DFU_OP_CODE_CREATE_OBJECT:
	case I2C_DFU_OP_CODE_EXECUTE_OBJECT:
		break;

	case I2C_DFU_OP_CODE_CALCULATE_CRC:
		res->offset =
			uint32_decode(m_recv_buffer + RES_CRC_OFFSET_OFFSET);
		res->crc =
			uint32_decode(m_recv_buffer + RES_CRC_CRC_OFFSET);
		break;

	case I2C_DFU_OP_CODE_SELECT_OBJECT:
		res->max_size =
			uint32_decode(m_recv_buffer + RES_SELECT_SIZE_OFFSET);
		res->offset =
			uint32_decode(m_recv_buffer + RES_SELECT_OFFSET_OFFSET);
		res->crc =
			uint32_decode(m_recv_buffer + RES_SELECT_CRC_OFFSET);

		break;

	default:
		break;

	}

	return res_code;
}

int nrf52_create_object(nrf_dfu_obj_type_t type, int size)
{
	nrf_dfu_req_t dfu_req;
	nrf_dfu_res_t dfu_res;

	memset(&dfu_req, 0, sizeof(nrf_dfu_req_t));
	memset(&dfu_res, 0, sizeof(nrf_dfu_res_t));

	dfu_req.req_type = NRF_DFU_OBJECT_OP_CREATE;
	dfu_req.obj_type = type;
	dfu_req.object_size = size;

	if (nrf52_send_request(&dfu_req) < 0) {
		dev_err(nrf_dev, "Failed to send create object request.\n");
		return -1;
	}

	return nrf52_read_response(&dfu_res);
}

int nrf52_create_command(int size)
{
	return nrf52_create_object(NRF_DFU_OBJ_TYPE_COMMAND, size);
}

int nrf52_create_data(int size)
{
	return nrf52_create_object(NRF_DFU_OBJ_TYPE_DATA, size);
}

int nrf52_request_CRC(uint32_t *crc)
{
	nrf_dfu_req_t dfu_req;
	nrf_dfu_res_t dfu_res;

	memset(&dfu_req, 0, sizeof(nrf_dfu_req_t));
	memset(&dfu_res, 0, sizeof(nrf_dfu_res_t));

	dfu_req.req_type = NRF_DFU_OBJECT_OP_CRC;

	if (nrf52_send_request(&dfu_req) < 0) {
		dev_err(nrf_dev, "Failed to send request CRC msg.\n");
		return -1;
	}

	if (nrf52_read_response(&dfu_res) != NRF_DFU_RES_CODE_SUCCESS) {
		dev_err(nrf_dev, "Request CRC failed.\n");
		return -1;
	}

	*crc = dfu_res.crc;

	return 0;
}

int nrf52_execute(void)
{
	nrf_dfu_req_t dfu_req;
	nrf_dfu_res_t dfu_res;

	memset(&dfu_req, 0, sizeof(nrf_dfu_req_t));
	memset(&dfu_res, 0, sizeof(nrf_dfu_res_t));

	dfu_req.req_type = NRF_DFU_OBJECT_OP_EXECUTE;

	if (nrf52_send_request(&dfu_req) < 0) {
		dev_err(nrf_dev, "Failed to send execute command.\n");
		return -1;
	}

	return nrf52_read_response(&dfu_res);
}

int nrf52_select_object(nrf_dfu_obj_type_t type)
{
	nrf_dfu_req_t dfu_req;
	nrf_dfu_res_t dfu_res;

	memset(&dfu_req, 0, sizeof(nrf_dfu_req_t));
	memset(&dfu_res, 0, sizeof(nrf_dfu_res_t));

	dfu_req.req_type = NRF_DFU_OBJECT_OP_SELECT;
	dfu_req.obj_type = type;

	if (nrf52_send_request(&dfu_req) < 0) {
		dev_err(nrf_dev, "Failed to send select object command.\n");
		return -1;
	}

	return nrf52_read_response(&dfu_res);
}

int nrf52_select_command(void)
{
	return nrf52_select_object(NRF_DFU_OBJ_TYPE_COMMAND);
}

int nrf52_select_data(void)
{
	return nrf52_select_object(NRF_DFU_OBJ_TYPE_DATA);
}

int nrf52_object_write(uint8_t *data, int size, uint32_t crc, uint32_t offset)
{
	uint32_t remote_crc;
	int ret = 0;
	nrf_dfu_req_t dfu_req;
	nrf_dfu_res_t dfu_res;


	memset(&dfu_req, 0, sizeof(nrf_dfu_req_t));
	memset(&dfu_res, 0, sizeof(nrf_dfu_res_t));

	dfu_req.req_type = NRF_DFU_OBJECT_OP_WRITE;
	dfu_req.p_req = data;
	dfu_req.req_len = size;
	ret = nrf52_send_request(&dfu_req);
	if (ret < 0)
		return ret;

	memset(&dfu_res, 0, sizeof(nrf_dfu_res_t));
	ret = nrf52_read_response(&dfu_res);
	if (ret < 0)
		return ret;

	remote_crc = dfu_res.crc;

	if (remote_crc != crc) {
		dev_err(nrf_dev,
				"Send data failed. Sent data CRC:0x%8X Response CRC CRC:0x%8X offset:%d\n",
				crc, dfu_res.crc, dfu_res.offset);
		ret = nrf52_request_CRC(&remote_crc);
		if (ret < 0) {
			dev_err(nrf_dev, "Failed CRC re-check\n");
			return -1;
		}
		dev_err(nrf_dev, "CRC re-check - expected:0x%8X got 0x%8X\n",
				crc, remote_crc);
		if (remote_crc != crc)
			return -1;
	}
	return 0;
}

int nrf52_send_firmware(struct device *dev, struct i2c_client *i2c_client_info,
				uint8_t *image_data, int image_data_length)
{
	int n;
	int pkt_size = 0;
	int obj_size = 0;
	int obj_bytes_sent = 0;
	int offset = 0;
	uint32_t crc = 0;

	client = i2c_client_info;
	nrf_dev = dev;


	while (offset < image_data_length) {

		if ((offset + DFU_OBJECT_SIZE) <= image_data_length)
			obj_size = DFU_OBJECT_SIZE;
		else
			obj_size = image_data_length - offset;

		if (nrf52_select_data() != NRF_DFU_RES_CODE_SUCCESS) {
			dev_err(nrf_dev, "Select data failed");
			return (-1);
		}

		if (nrf52_create_data(obj_size) != NRF_DFU_RES_CODE_SUCCESS) {
			dev_err(nrf_dev, "Create data failed");
			return (-1);
		}

		obj_bytes_sent = 0;
		for (n = 0; ((n <= (obj_size / DFU_PKT_SIZE))
				&& (obj_size - obj_bytes_sent)); n++) {

			if ((obj_bytes_sent + DFU_PKT_SIZE) < obj_size)
				pkt_size = DFU_PKT_SIZE;
			else
				pkt_size = obj_size - obj_bytes_sent;

			crc = crc32_compute((image_data + offset), pkt_size,
					&crc);
			if (nrf52_object_write((image_data + offset), pkt_size,
					crc, offset) < 0) {
				dev_err(nrf_dev, "Obj write command failed");
				return -1;
			}
			offset += pkt_size;
			obj_bytes_sent += pkt_size;
		}

		if (nrf52_execute() != NRF_DFU_RES_CODE_SUCCESS) {
			dev_err(nrf_dev, "Execute command failed");
			return (-1);
		}

		dev_info(nrf_dev, "Sent %d bytes out of %d\n", offset,
				image_data_length);

	}

	dev_info(nrf_dev, "Complete\n");
	return 0;
}

int nrf52_send_init(struct device *dev, struct i2c_client *i2c_client_info,
				uint8_t *init_data, int init_data_length)
{
	nrf_dfu_req_t dfu_req;
	nrf_dfu_res_t dfu_res;

	dfu_req.req_type = NRF_DFU_OBJECT_OP_WRITE;
	dfu_req.p_req = init_data;
	dfu_req.req_len = init_data_length;

	client = i2c_client_info;
	nrf_dev = dev;


	if(init_data_length > MAX_REQUEST) {
		dev_err(nrf_dev, "Init data packet too large.");
		return (-1);
	}

	if (nrf52_select_command() != NRF_DFU_RES_CODE_SUCCESS) {
		dev_err(nrf_dev, "Select command failed");
		return (-1);
	}

	if (nrf52_create_command(init_data_length)
				!= NRF_DFU_RES_CODE_SUCCESS) {
		dev_err(nrf_dev, "Create command failed");
		return (-1);
	}

	/* Send Init Data */
	if (nrf52_send_request(&dfu_req) < 0) {
		dev_err(nrf_dev, "Failed to send init request.\n");
		return -1;
	}

	if (nrf52_read_response(&dfu_res) != NRF_DFU_RES_CODE_SUCCESS) {
		dev_err(nrf_dev, "Init command failed");
		return (-1);
	}

	if (nrf52_execute() != NRF_DFU_RES_CODE_SUCCESS) {
		return (-1);
	}

	return 0;
}

int nrf52_reset_cmd(struct nrf52_data *nrf52)
{
	uint8_t reset_code[] = { 0x09, 0x00, 0x00, 0x00, 0x04, 0x01, 0x01, 0x01,
			0x01 };
	int ret;
	struct i2c_msg msg;

	/* Send Reset */
	msg.addr = nrf52->client->addr;
	msg.flags = 0;
	msg.len = 9;
	msg.buf = reset_code;
	ret = i2c_transfer(nrf52->client->adapter, &msg, 1);

	msleep(2000);

	return ret;
}

int nrf52_load_fw(const struct firmware *fw, struct nrf52_data *nrf52)
{
	fw_archive_t fw_archive;

	if (fw == NULL) {
		dev_err(nrf52->dev, "fw file not found\n");
		return -1;
	}

	dev_info(nrf52->dev, "nrf52_load_fw - size: %d\n",
			(int) fw->size);

	if (nrf52_untar_fw(nrf52->dev, fw->data, fw->size, &fw_archive)
			!= 0) {
		dev_err(nrf52->dev, "Failed fw untar.");
		return -1;
	}

	dev_info(nrf52->dev, "nrf52_load_fw - flash: %s\n",
			fw_archive.image_file.file_name);

	/* Deprecated reset cmd. For older bootloaders . */
	nrf52_reset_cmd(nrf52);

	if (nrf52_send_init(nrf52->dev, nrf52->client,
			fw_archive.init_file.file,
			fw_archive.init_file.length) < 0) {
		return -1;
	}

	if (nrf52_send_firmware(nrf52->dev, nrf52->client,
			fw_archive.image_file.file,
			fw_archive.image_file.length) < 0) {
		dev_err(nrf52->dev,
				"Failed to send firmware image data.");
		return -1;
	}

	return 0;
}
