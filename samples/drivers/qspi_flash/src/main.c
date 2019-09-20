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
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

//#define FLASH_DEVICE CONFIG_QSPI
#define BUFF_SIZE			1024


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
enum{
	HAL_OK = 0x00,
	HAL_ERROR = 0x01
};
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void qspi_configure(struct qspi_config * pCfg);
void print_buffer(struct qspi_buf * buffer, uint8_t * txbuff);

/* TESTING FUNCTIONS */
int fill_buff(uint8_t *buff, uint32_t buff_size, uint32_t size_to_fill);
int test_loop(struct device *dev, const struct qspi_config *config,
				uint8_t * tx_buf, uint8_t * rx_buf, size_t size, uint32_t iterations);
void print_buff(uint8_t * pBuff, uint32_t size);
/* Private functions ---------------------------------------------------------*/
void main(void)
{
	printf("\nTest 1: Flash erase\n");
	static uint8_t tx[BUFF_SIZE]={0};
	static uint8_t rx[BUFF_SIZE]={0};

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

	if(test_loop(qspi,&qspi_cfg,tx,rx,BUFF_SIZE,BUFF_SIZE) != HAL_OK){
		printf("\nTEST: FAILED");
	}
	else{
		printf("\nTEST: PASSED");
	}
}


static void qspi_configure(struct qspi_config * pCfg){
	pCfg->frequency = 8000000;
	pCfg->operation = (	QSPI_CS_DELAY_SET(8) 							|
						QSPI_DATA_LINES_SET(QSPI_DATA_LINES_QUAD)		|
						QSPI_ADDRESS_MODE_SET(QSPI_ADDRESS_MODE_24BIT));
}

/* Loop for testing */
int test_loop(struct device *dev, const struct qspi_config *config,
				uint8_t * tx_buf, uint8_t * rx_buf, size_t size, uint32_t iterations){
	int retval = HAL_OK;
	uint32_t address = 0;
	for(uint32_t i=4; i<iterations; i+=4){
		printf("\n\n\n#------------------  TEST %d  ------------------#", i);

		memset(tx_buf,0,size);
		memset(rx_buf,0,size);
		/* fill tx buff with data */
		if(fill_buff(tx_buf,size,i) != HAL_OK){
			/* Filling Status: ERROR */
			break;
		}
		else{
			/* Filling Status: OK */
		}

		/* Erase desired section of the memory */
		printf("\n Erasing memory...");
		if(qspi_cmd_xfer(dev, config, NULL, 0, NULL ,0 , QSPI_STD_CMD_SE, 0) != HAL_OK){
			/* Erasing status: ERROR */
			printf("ERROR");
			retval = HAL_ERROR;
		}
		else{
			/* Erasing status: OK */
			printf("OK");
		}

		/* Write data to the flash */
		printf("\n Writting to memory...");
		if(qspi_write(dev, config, tx_buf, i, 0) != HAL_OK){
			/* Writting status: ERROR */
			printf("ERROR");
			retval = HAL_ERROR;
		}
		else{
			/* Writting status: OK */
			printf("OK");
		}

		/* Read data from flash */
		printf("\n Reading from memory...");
		if(qspi_read(dev, config, rx_buf, i, 0) != HAL_OK){
			/* Writting status: ERROR */
			printf("ERROR");
			retval = HAL_ERROR;
		}
		else{
			/* Writting status: OK */
			printf("OK");
		}

		/* Increase the address */
		address += i;
		/* Compare buffers */
		printf("\n Compare memory...");
		if(memcmp(tx_buf,rx_buf,i) != HAL_OK){
			printf("ERROR");
			retval = HAL_ERROR;
		}
		else{
			printf("OK");
		}
	}
	return retval;
}


int fill_buff(uint8_t *buff, uint32_t buff_size, uint32_t size_to_fill) {
	int retval = HAL_ERROR;
	printf("\n Filling buffer...");
	if (buff) {
		if (buff_size >= (size_to_fill)) {
			for (uint32_t i = 0; i < size_to_fill; i++) {
				*buff++ = (uint8_t)i;
			}
			printf("OK");
			retval = HAL_OK;
		} else {
			/* Size to fill is greater than buffer size. Exit with error */
			printf("ERROR");
		}
	} else {
		/* Pointer is NULL - exit with error  */
		printf("ERROR");
	}
	return retval;
}

void print_buff(uint8_t * pBuff, uint32_t size){
	printk("\nBuff: ");
	for(uint32_t i=0; i< size; i++){
		printk("0x%02X," ,*pBuff++);
	}
}
