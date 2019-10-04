/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __QSPI_NOR_H__
#define __QSPI_NOR_H__

#include <sys/util.h>

#define QSPI_NOR_MAX_ID_LEN	3

struct qspi_nor_config {
	/* JEDEC id from devicetree */
	u8_t id[QSPI_NOR_MAX_ID_LEN];

	/* Indicates support for BE32K */
	bool has_be32k;

	/* Size from devicetree, in bytes */
	u32_t size;
};

/* Status register bits */
#define QSPI_NOR_WIP_BIT         BIT(0)  /* Write in progress */
#define QSPI_NOR_WEL_BIT         BIT(1)  /* Write enable latch */

/* Flash opcodes */
#define QSPI_NOR_CMD_WRSR        0x01    /* Write status register */
#define QSPI_NOR_CMD_RDSR        0x05    /* Read status register */
#define QSPI_NOR_CMD_READ        0x03    /* Read data */
#define QSPI_NOR_CMD_WREN        0x06    /* Write enable */
#define QSPI_NOR_CMD_WRDI        0x04    /* Write disable */
#define QSPI_NOR_CMD_PP          0x02    /* Page program */
#define QSPI_NOR_CMD_SE          0x20    /* Sector erase */
#define QSPI_NOR_CMD_BE_32K      0x52    /* Block erase 32KB */
#define QSPI_NOR_CMD_BE          0xD8    /* Block erase */
#define QSPI_NOR_CMD_CE          0xC7    /* Chip erase */
#define QSPI_NOR_CMD_RDID        0x9F    /* Read JEDEC ID */
#define QSPI_NOR_CMD_MCHP_UNLOCK 0x98    /* Microchip: Global unblock */

/* Page, sector, and block size are standard, not configurable. */
#define QSPI_NOR_PAGE_SIZE    0x0100U
#define QSPI_NOR_SECTOR_SIZE  0x1000U
#define QSPI_NOR_BLOCK_SIZE   0x10000U

/* Some devices support erase operations on 32 KiBy blocks.
 * Support is indicated by the has-be32k property.
 */
#define QSPI_NOR_BLOCK32_SIZE 0x8000

/* Test whether offset is aligned. */
#define QSPI_NOR_IS_PAGE_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_PAGE_SIZE - 1U)) == 0)
#define QSPI_NOR_IS_SECTOR_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_SECTOR_SIZE - 1U)) == 0)
#define QSPI_NOR_IS_BLOCK_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_BLOCK_SIZE - 1U)) == 0)
#define QSPI_NOR_IS_BLOCK32_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_BLOCK32_SIZE - 1U)) == 0)

#endif /*__QSPI_NOR_H__*/
