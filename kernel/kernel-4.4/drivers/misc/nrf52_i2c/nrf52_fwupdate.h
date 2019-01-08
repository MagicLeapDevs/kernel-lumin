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

#ifndef NRFUPDATE_H_
#define NRFUPDATE_H_

#define DFU_OBJECT_SIZE							4096
#define MAX_RETRIES							2
#define NRF_RESET_GPIO							25
#define NRF_BL_MODE_GPIO						96
#define DFU_PKT_SIZE							20
#define MAX_RECEIVE							256
#define MAX_REQUEST							512
#define I2C_DFU_CTRL							0x01
#define I2C_DFU_PKT							0x02
#define I2C_READ_RETRIES						5
#define RES_CRC_OFFSET_OFFSET						3
#define RES_CRC_CRC_OFFSET						7
#define RES_SELECT_SIZE_OFFSET						3
#define RES_SELECT_OFFSET_OFFSET					7
#define RES_SELECT_CRC_OFFSET						11
#define TAR_FILENAME_SIZE						100
#define TAR_MODE_SIZE							8
#define TAR_OWNER_SIZE							8
#define TAR_GROUP_SIZE							8
#define TAR_FILE_SIZE							12
#define TAR_MTIME_SIZE							12
#define TAR_CHECKSUM_SIZE						8
#define TAR_LINKTO_SIZE							100
#define TAR_BRAND_SIZE							8
#define TAR_OWNERNAME_SIZE						32
#define TAR_GROUPNAME_SIZE						32
#define TAR_DEVMAJOR_SIZE						8
#define TAR_DEVMINOR_SIZE						8
#define TAR_PREFIX_SIZE							155
#define TAR_PAD_RESERVED_SIZE						12

/**@brief DFU request operation codes.
 *
 * @details The DFU transport layer creates request events of these types.
 * The implementation of @ref nrf_dfu_req_handler_on_req handles
 * requests of these types.
 */
typedef enum {

	NRF_DFU_OBJECT_OP_NONE						= 0,
	NRF_DFU_OBJECT_OP_CREATE					= 1,
	NRF_DFU_OBJECT_OP_WRITE						= 2,
	NRF_DFU_OBJECT_OP_EXECUTE					= 3,
	NRF_DFU_OBJECT_OP_CRC						= 4,
	NRF_DFU_OBJECT_OP_SELECT					= 6,
	NRF_DFU_OBJECT_OP_OTHER						= 7,
} nrf_dfu_req_op_t;

/**@brief DFU request result codes.
 *
 * @details The DFU transport layer creates request events of types
 * @ref nrf_dfu_req_op_t, which are handled by @ref
 * nrf_dfu_req_handler_on_req. That functions returns one of these
 * result codes.
 */
typedef enum {
	NRF_DFU_RES_CODE_INVALID					= 0x00,
	NRF_DFU_RES_CODE_SUCCESS					= 0x01,
	NRF_DFU_RES_CODE_OP_CODE_NOT_SUPPORTED				= 0x02,
	NRF_DFU_RES_CODE_INVALID_PARAMETER				= 0x03,
	NRF_DFU_RES_CODE_INSUFFICIENT_RESOURCES				= 0x04,
	NRF_DFU_RES_CODE_INVALID_OBJECT					= 0x05,
	NRF_DFU_RES_CODE_UNSUPPORTED_TYPE				= 0x07,
	NRF_DFU_RES_CODE_OPERATION_NOT_PERMITTED			= 0x08,
	NRF_DFU_RES_CODE_OPERATION_FAILED				= 0x0A,
	NRF_DFU_RES_CODE_EXT_ERROR					= 0x0B,
	NRF_DFU_RES_CODE_UPDATE_NOT_REQUIRED				= 0x0C,
	NRF_DFU_RES_CODE_INVALID_VERSION				= 0x0D,
	NRF_DFU_RES_CODE_DEBUG_NOT_PERMITTED				= 0x0E,
	NRF_DFU_RES_CODE_UNSUPPORTED_HW					= 0x0F,
	NRF_DFU_RES_CODE_INCOMPATIBLE_SD				= 0x10,
} nrf_dfu_res_code_t;

typedef enum {
	I2C_DFU_OP_CODE_CREATE_OBJECT					= 0x01,
	I2C_DFU_OP_CODE_SET_RECEIPT_NOTIF				= 0x02,
	I2C_DFU_OP_CODE_CALCULATE_CRC					= 0x03,
	I2C_DFU_OP_CODE_EXECUTE_OBJECT					= 0x04,
	I2C_DFU_OP_CODE_SELECT_OBJECT					= 0x06,
	I2C_DFU_OP_CODE_RESPONSE					= 0x60
} i2c_dfu_op_code_t;

/**@brief DFU object types.
 */
typedef enum {
	NRF_DFU_OBJ_TYPE_INVALID,
	NRF_DFU_OBJ_TYPE_COMMAND,
	NRF_DFU_OBJ_TYPE_DATA,
} nrf_dfu_obj_type_t;

/** @brief Definition of a DFU request sent from the transport layer.
 *
 * @details When the transport layer gets a DFU event, it calls the function
 * @ref nrf_dfu_req_handler_on_req to handle the DFU request.
 */
typedef struct {
	nrf_dfu_req_op_t req_type;

	union {
		struct {
			uint32_t obj_type;
			uint32_t object_size;
		};

		struct {
			uint8_t *p_req;
			uint32_t req_len;
		};
	};
} nrf_dfu_req_t;

/** @brief Response used during DFU operations.
 */
typedef struct {
	union {
		struct {
			uint8_t *p_res;
			uint32_t res_len;
		};

		struct {
			uint32_t max_size;
			uint32_t offset;
			uint32_t crc;
		};
	};
} nrf_dfu_res_t;


typedef struct {
	char filename[TAR_FILENAME_SIZE];
	char mode[TAR_MODE_SIZE];
	char owner[TAR_OWNER_SIZE];
	char group[TAR_GROUP_SIZE];
	char size[TAR_FILE_SIZE];
	char mtime[TAR_MTIME_SIZE];
	char checksum[TAR_CHECKSUM_SIZE];
	char type;
	char linkto[TAR_LINKTO_SIZE];
	char brand[TAR_BRAND_SIZE];
	char ownername[TAR_OWNERNAME_SIZE];
	char groupname[TAR_GROUPNAME_SIZE];
	char devmajor[TAR_DEVMAJOR_SIZE];
	char devminor[TAR_DEVMINOR_SIZE];
	char prefix[TAR_PREFIX_SIZE];
	char RESERVED[TAR_PAD_RESERVED_SIZE];
} tar_t;

typedef struct {
	tar_t header;
	uint8_t file[];
} tarfile_t;

typedef struct {
	int length;
	char *file_name;
	char *file;
} fw_file_t;
typedef struct {
	fw_file_t manifest;
	fw_file_t init_file;
	fw_file_t image_file;
} fw_archive_t;

int nrf52_load_fw(const struct firmware *fw, struct nrf52_data *nrf52);

#endif /* NRFUPDATE_H_ */
