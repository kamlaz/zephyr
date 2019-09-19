/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Includes ------------------------------------------------------------------*/
#include <zephyr.h>
//#include <drivers/flash.h>
#include <drivers/qspi.h>
#include <nrfx_qspi.h>
#include <device.h>
#include <stdio.h>

//#if (CONFIG_SPI_FLASH_W25QXXDV - 0)
/* NB: W25Q16DV is a JEDEC spi-nor device, but has a separate driver. */
/*
#define FLASH_DEVICE CONFIG_SPI_FLASH_W25QXXDV_DRV_NAME
#define FLASH_NAME "W25QXXDV"
#elif (CONFIG_SPI_NOR - 0) || defined(DT_INST_0_JEDEC_SPI_NOR_LABEL)
#define FLASH_DEVICE DT_INST_0_JEDEC_SPI_NOR_LABEL
#define FLASH_NAME "JEDEC SPI-NOR"
#else
#error Unsupported flash driver
#endif

#define FLASH_TEST_REGION_OFFSET 0xff000
#define FLASH_SECTOR_SIZE        4096
#define TEST_DATA_BYTE_0         0x55
#define TEST_DATA_BYTE_1         0xaa
#define TEST_DATA_LEN            2
*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

//#define FLASH_DEVICE CONFIG_QSPI
#define NO_ADDRESS			(-1)

#define QSPI_STD_CMD_WRSR	0x01
#define QSPI_STD_CMD_RSTEN  0x66
#define QSPI_STD_CMD_RST    0x99
#define QSPI_STD_CMD_QE		0x40

//------------
#define QSPI_STD_CMD_CE			0x60	//Chip erase
#define QSPI_STD_CMD_JEDEC_ID	0x9F	//Jedec ID
#define QSPI_STD_CMD_WREN		0x06	// Write enable
#define QSPI_STD_CMD_SE			0x20	// Write enable


#define JEDEC_ID_SIZE			3
#define CHIP_ERASE_SIZE			1
#define SECTOR_ERASE_SIZE		1	1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void qspi_configure(struct qspi_config * pCfg);
void print_buffer(struct qspi_buf * buffer, uint8_t * txbuff);

/* Private functions ---------------------------------------------------------*/
void main(void)
{
	printf("\nTest 1: Flash erase\n");
	uint8_t cmd[1] = {QSPI_STD_CMD_QE};
	uint8_t tx[16]={1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8};
	uint8_t rx[16]={0};
	uint32_t address = 0x0A;
	/* TX BUFFER */
	struct qspi_buf TX={
			.buf = &tx,
			.len = sizeof(tx)
	};
	struct qspi_buf_set TXbuffer={
			.buffers = &TX,
			.count = 1,
	};

	/* RX BUFFER */
	struct qspi_buf RX={
			.buf = &rx,
			.len = sizeof(tx)
	};
	struct qspi_buf_set RXbuffer={
			.buffers = &RX,
			.count = 1,
	};

	struct device *qspi;				// Defines pointer to qspi device
	struct qspi_config qspi_cfg;		// Config for qspi device


	printk("discochlosta!!\n");
	/* Get binding */
	qspi = device_get_binding(DT_NORDIC_NRF_QSPI_QSPI_0_LABEL);

	/* Check acquired pointer */
	if (!qspi) {
		printk("Could not find SPI driver\n");
		return;
	}

	/* Assign config */
	qspi_configure(&qspi_cfg);

	qspi_cmd_xfer(qspi, &qspi_cfg, cmd, 3, rx ,0 , QSPI_STD_CMD_JEDEC_ID, NO_ADDRESS);

	qspi_cmd_xfer(qspi, &qspi_cfg, cmd, 0, rx ,0 , QSPI_STD_CMD_SE, 0);

//	qspi_cmd_xfer(qspi, &qspi_cfg, NULL, 0, NULL ,0 , QSPI_STD_CMD_WREN, 0);

	/* Erases whole chip */
//	qspi_cmd_xfer(qspi, &qspi_cfg, NULL, 0, NULL ,0 , QSPI_STD_CMD_CE, 0);

//	qspi_read(qspi, &qspi_cfg, &rx,sizeof(tx), address);

	/* Write data to the external flash */
//	qspi_write(qspi, &qspi_cfg, &tx, sizeof(tx), address);

	/* wait for the next sample */
	qspi_read(qspi, &qspi_cfg, &rx,sizeof(tx), address);

	printk("\n I'm Alive!");
//	qspi_write(qspi, &qspi_cfg, &TXbuffer, address);

}


static void qspi_configure(struct qspi_config * pCfg){
	pCfg->prescaler = NRF_QSPI_FREQ_32MDIV16;
	pCfg->operation = (QSPI_CS_DELAY_SET(8) 							|
					   QSPI_DATA_LINES_SET(QSPI_DATA_LINES_SINGLE)		|
					   QSPI_ADDRESS_MODE_SET(QSPI_ADDRESS_MODE_24BIT));
}


