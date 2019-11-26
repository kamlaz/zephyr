/*
 * Copyright (c) 2019, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __QSPI_NOR_H__
#define __QSPI_NOR_H__

#include <sys/util.h>

#define QSPI_NOR_MAX_ID_LEN     3

struct qspi_nor_config {
	/* JEDEC id from devicetree */
	u8_t id[QSPI_NOR_MAX_ID_LEN];
	/* Indicates support for BE32K */
	bool has_be32k;
	/* Size from devicetree, in bytes */
	u32_t size;
};

/* Status register bits */
#define QSPI_NOR_WIP_BIT         BIT(0)         /* Write in progress */
#define QSPI_NOR_WEL_BIT         BIT(1)         /* Write enable latch */
#define QSPI_NOR_QE_BIT          BIT(6)         /* Quad enable enable latch */

/* Flash opcodes */
#define QSPI_NOR_CMD_WRSR        0x01           /* Write status register */
#define QSPI_NOR_CMD_RDSR        0x05           /* Read status register */
#define QSPI_NOR_CMD_READ        0x03           /* Read data */
#define QSPI_NOR_CMD_WREN        0x06           /* Write enable */
#define QSPI_NOR_CMD_WRDI        0x04           /* Write disable */
#define QSPI_NOR_CMD_PP          0x02           /* Page program */
#define QSPI_NOR_CMD_SE          0x20           /* Sector erase */
#define QSPI_NOR_CMD_BE_32K      0x52           /* Block erase 32KB */
#define QSPI_NOR_CMD_BE          0xD8           /* Block erase */
#define QSPI_NOR_CMD_CE          0xC7           /* Chip erase */
#define QSPI_NOR_CMD_RDID        0x9F           /* Read JEDEC ID */
#define QSPI_NOR_CMD_MCHP_UNLOCK 0x98           /* Microchip: Global unblock */

/* Page, sector, and block size are standard, not configurable. */


/* Some devices support erase operations on 32 KiBy blocks.
 * Support is indicated by the has-be32k property.
 */
#define QSPI_NOR_BLOCK32_SIZE 0x8000

/**
 * @brief QSPI Address configuration
 * Length of the address field in bits.
 * Typical flash chips support 24bit address mode
 */
#define QSPI_ADDRESS_MODE_24BIT                 0x02
#define QSPI_ADDRESS_MODE_32BIT                 0x03

/**
 * @brief QSPI flash memory granularity defines
 */
#define QSPI_PAGE_SIZE                          0x0100U
#define QSPI_SECTOR_SIZE                        0x1000U
#define QSPI_BLOCK_SIZE                         0x10000U
#define QSPI_BLOCK32_SIZE                       0x8000

/**
 * @brief QSPI buffer structure
 * Structure used both for TX and RX purposes.
 *
 * @param buf is a valid pointer to a data buffer.
 * Can not be NULL.
 * @param len is the length of the data to be handled.
 * If no data to transmit/receive - pass 0.
 */
struct qspi_buf {
	u8_t *buf;
	size_t len;
};


/**
 * @brief QSPI command structure
 * Structure used for custom command usage.
 *
 * @param op_code is a command value (i.e 0x9F - get Jedec ID)
 * @param tx_buf structure used for TX purposes. Can be NULL if not used.
 * @param rx_buf structure used for RX purposes. Can be NULL if not used.
 */
struct qspi_cmd {
	u8_t op_code;
	const struct qspi_buf *tx_buf;
	const struct qspi_buf *rx_buf;
};

/**
 * @brief QSPI controller configuration structure
 */
struct qspi_config {
	/* Chip Select pin used to select memory */
	u16_t cs_pin;

	/* Frequency of the QSPI bus in [Hz]*/
	u32_t frequency;

	/* Polarity, phase, Mode
	 * 0x00: Mode 0: Data are captured on the clock rising edge and
	 *			data are sampled on a leading edge.
	 *			Base level of clock is 0.(CPOL=0, CPHA=0).
	 * 0x01: Mode 1: Data are captured on the clock rising edge and
	 *			data are sampled on a trailing edge.
	 *			Base level of clock is 0.(CPOL=0, CPHA=1).
	 * 0x02: Mode 2: Data are captured on the clock falling edge and
	 *			data are sampled on a leading edge.
	 *			Base level of clock is 1.(CPOL=1, CPHA=0).
	 * 0x03: Mode 3: Data are captured on the clock falling edge and
	 *			data are sampled on a trailing edge.
	 *			Base level of clock is 1.(CPOL=1, CPHA=1).
	 */
	u8_t mode : 2;

	/* Defines how many lines will be used for read/write operation
	 * 0x00: One data line
	 * 0x01: Two data lines
	 * 0x02: Four data lines
	 */
	u8_t data_lines : 2;

	/* Defines how many bits are used for address (8/16/24/32)
	 * 0x00: 8 bit address field
	 * 0x01: 16 bit address field
	 * 0x02: 24 bit address field
	 * 0x03: 32 bit address field
	 */
	u8_t address : 2;

	/* Specifies the Chip Select High Time.
	 * No of clock cycles when CS must remain high between commands.
	 * Note: Refer to the chip manufacturer to check what
	 * clock source is used to generate cs_high_time. In most cases
	 * it is system clock, but sometimes it is independent
	 * QSPI clock with prescaler.
	 *
	 *                            |<-CS_HIGH_TIME->|
	 *      .-.                   .----------------.
	 *  CS -' '-------------------'                '----------
	 *          .-. .-. .-. .-. .-.                  .-. .-. .-.
	 * SCK -----' '-' '-' '-' '-' '------------------' '-' '-' '
	 *        .---.---.---.---.---.                .---.---.---.
	 * DATA   |MSB|   |...|   |LSB|                |MSB|...|LSB|
	 *    ----'---'---'---'---'---'----------------'---'---'---'
	 */
	u8_t cs_high_time;
};


/**
 * @cond INTERNAL_HIDDEN
 *
 * These are for internal use only, so skip these in
 * public documentation.
 */
/**
 * @typedef qspi_api_configure
 * @brief Callback API for configuring QSPI driver
 * See qspi_configure() for argument descriptions
 */
typedef int (*qspi_api_configure)(struct device *dev,
				  const struct qspi_config *config);

/**
 * @typedef qspi_api_write
 * @brief Callback API for write operations
 * See qspi_write() for argument descriptions
 */
typedef int (*qspi_api_write)(struct device *dev,
			      const struct qspi_buf *tx_buf,
			      u32_t address);

/**
 * @typedef qspi_api_read
 * @brief Callback API for read operations
 * See qspi_read() for argument descriptions
 */
typedef int (*qspi_api_read)(struct device *dev,
			     const struct qspi_buf *rx_buf,
			     u32_t address);

/**
 * @typedef qspi_api_send_cmd
 * @brief Callback API for sending custom command
 * See qspi_send_cmd() for argument descriptions
 */
typedef int (*qspi_api_send_cmd)(struct device *dev,
				 const struct qspi_cmd *cmd);

/**
 * @typedef qspi_api_erase
 * @brief Callback API for erasing memory
 * See qspi_erase() for argument descriptions
 */
typedef int (*qspi_api_erase)(struct device *dev,
			      u32_t start_address,
			      u32_t length);

/**
 * @brief QSPI driver API
 * This is the mandatory API any QSPI driver needs to expose.
 */
struct qspi_driver_api {
	qspi_api_configure configure;
	qspi_api_write write;
	qspi_api_read read;
	qspi_api_send_cmd send_cmd;
	qspi_api_erase erase;
};

#endif /*__QSPI_NOR_H__*/
