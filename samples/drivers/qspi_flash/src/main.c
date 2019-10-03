/*
 * Copyright (c) 2019 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Includes ------------------------------------------------------------------*/
#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <drivers/qspi.h>
#include <logging/log.h>

#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(qspi_flash, LOG_LEVEL_DBG);

#if defined(TCA2_INTEGRATION)
#include "target_comm.h"
target_comm_t g_target_comm;
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define QSPI_STD_CMD_JEDEC_ID	0x9F	// CMD: Jedec ID
#define QSPI_STD_CMD_SE			0x20	// CMD: Sector erase

#define JEDEC_ID_SIZE			3		// Amount of bytes of JEDEC ID
//------------		BUFFERS
#define TEST_MAX_MEM			4096
#define FLASH_SECTOR_SIZE		4096

//------------
/* Private macro -------------------------------------------------------------*/
enum{
	HAL_OK = 0x00,
};
/* Private variables ---------------------------------------------------------*/
uint8_t chip_mem[TEST_MAX_MEM]={0};	// Holds data read from FLASH memory
uint8_t test[TEST_MAX_MEM]={0};		// Holds data that will be sent to FLASH memory

struct device *qspi;				// Defines pointer to qspi device
struct qspi_config qspi_cfg;		// Config for qspi device

/* Private function prototypes -----------------------------------------------*/
//static void qspi_configure(struct qspi_config * pCfg);
static void qspi_configure(void);
void qspi_init(uint32_t frequency, uint8_t addressMode, uint8_t dataLines);

/* TESTING FUNCTIONS */
uint32_t memory_test(uint32_t length);
uint32_t read_device_id(void);
uint32_t memory_complex_test(uint32_t length);

/* Private functions ---------------------------------------------------------*/
void main(void)
{
#if defined(TCA2_INTEGRATION)
	tca_init(&g_target_comm, &TARGET_COMM_DEFAULT_CONFIG);
#endif
	/* Get binding */
	qspi = device_get_binding(DT_NORDIC_NRF_QSPI_QSPI_0_LABEL);

	/* Check acquired pointer */
	if (!qspi) {
		LOG_INF("QSPI: Device driver not found.");
	}
	else{
		LOG_INF("QSPI: Device driver found.");
	}

	/* Assign config */
//	qspi_configure(&qspi_cfg);
//	qspi_configure();
	qspi_init(8000000,QSPI_ADDRESS_MODE_24BIT, QSPI_DATA_LINES_QUAD);

	/* Identify the flash */
	LOG_INF("Vendor ID: %x", read_device_id());

	/* Perform the test */
	memory_test(4);
	memory_complex_test(4);
#if defined(TCA2_INTEGRATION)
	while (1) {
		tca_send_and_receive(&g_target_comm);
	}
#endif

	LOG_INF("End.");
}



/**
 * @brief Configures QSPI peripherial
 *
 * @param pCfg	- pointer to the qspi structure
 * @retval None
 */
//static void qspi_configure(struct qspi_config * pCfg){
//	pCfg->frequency = 8000000;
//	pCfg->operation = (	QSPI_CS_DELAY_SET(8) 							|
//						QSPI_DATA_LINES_SET(QSPI_DATA_LINES_QUAD)		|
//						QSPI_ADDRESS_MODE_SET(QSPI_ADDRESS_MODE_24BIT));
//}
static void qspi_configure(void){
	qspi_cfg.frequency = 8000000;
	qspi_cfg.operation = (	QSPI_CS_DELAY_SET(8) 							|
						QSPI_DATA_LINES_SET(QSPI_DATA_LINES_QUAD)			|
						QSPI_ADDRESS_MODE_SET(QSPI_ADDRESS_MODE_24BIT));
}
//---------------------------------------------------------------------------		TEST FUNCTIONS

/**
 * @brief Initialises QSPI peripherial - API for testing purposes
 *
 * @param frequency		- Desired frequency of the QSPI bus.
 * @param addressMode	- Mode of addressing
 * @param addressMode	- Mode of used datalines (1,2 or 4)
 * @retval None
 */
void qspi_init(uint32_t frequency, uint8_t addressMode, uint8_t dataLines){
	qspi_cfg.frequency = frequency;
	qspi_cfg.operation = (	QSPI_CS_DELAY_SET(8) 				|
						QSPI_DATA_LINES_SET(dataLines)			|
						QSPI_ADDRESS_MODE_SET(addressMode));
}


/**
 * @brief Reads device ID (JEDEC ID)
 *
 * @param None
 * @retval 0 If failed, JEDEC ID if success.
 */
uint32_t read_device_id(void){
	uint32_t jedec_id = 0;
	/* Erase sector */
	if(qspi_cmd_xfer(qspi, &qspi_cfg, NULL, 0, chip_mem ,JEDEC_ID_SIZE , QSPI_STD_CMD_JEDEC_ID, 0) != HAL_OK){
		/* Erasing status: ERROR */
		LOG_WRN("Unable to sector erase");
		return 1;
	}

	memcpy(&jedec_id,chip_mem,JEDEC_ID_SIZE);

	return jedec_id;
}

/**
 * @brief Performs basic test of the QSPI FLASH memory
 *
 * @param length	length of the tests in bytes
 * @retval 0 If successful, errno code otherwise.
 */
uint32_t memory_test(uint32_t length){
	LOG_INF("memory_test(%d)", length);

	/* Check input parameter */
	if(length > FLASH_SECTOR_SIZE)
	{
		LOG_WRN("Simple memory test is limited to FLASH_SECTOR_SIZE=4096");
		return 1;
	}
	printf("\n Erasing memory...");
	/* Erase sector */
	if(qspi_cmd_xfer(qspi, &qspi_cfg, NULL, 0, NULL ,0 , QSPI_STD_CMD_SE, 0) != HAL_OK){
		/* Erasing status: ERROR */
		LOG_WRN("Unable to sector erase");
		return 2;
	}
	printf("OK");
	printf("\n Reading memory...");
	/* Chip read */
	if(qspi_read(qspi, &qspi_cfg, chip_mem, length, 0) != HAL_OK){
		/* Reading status: ERROR */
		LOG_WRN("Unable to read data");
		return 3;
	}
	printf("OK");
	printf("\n Checking memory...");

	/* Memory check -  expect all 1s */
	for(uint16_t i = 0; i < length; i++)
	{
		if(chip_mem[i] != 0xFF)
		{
			LOG_WRN("Memory read incorrect %d = %x", i, chip_mem[i]);
		return 4;
		}
	}

	printf("OK");
	printf("\n Writting to memory...");
	/* Fill Tx buff with data */
	for(uint32_t i = 0; i < length; i++)
	{
		test[i] = i % 0xFF;
	}

	/* Chip write */
	if(qspi_write(qspi, &qspi_cfg, test, length, 0) != HAL_OK){
		/* Writting status: ERROR */
		LOG_WRN("Unable to write data");
		return 5;
	}

	printf("OK");
	printf("\n Reading memory...");
	/* Chip read */
	if(qspi_read(qspi, &qspi_cfg, chip_mem, length, 0) != HAL_OK){
		/* Reading status: ERROR */
		LOG_WRN("Unable to read data");
		return 6;
	}

	printf("OK");
	printf("\n Comparing memory...");

	/* Compare write to read */
	for(uint32_t i = 0; i < length; i++)
	{
		if(test[i] != chip_mem[i])
		{
			LOG_WRN("Error: compare write to read mismatch %x vs %x", test[i], chip_mem[i]);
			return 7;
		}
	}


	printf("OK");

	LOG_INF("Memory test OK");

	return 0;
}

/**
 * @brief Performs comples test of the QSPI FLASH memory and QSPI bus
 * This tests changes QSPI configuration and check
 *
 * @param length	length of the tests in bytes
 * @retval 0 If successful, errno code otherwise.
 */
uint32_t memory_complex_test(uint32_t length){
	uint32_t result = 0;
	uint32_t success = 0;
	/* Testing frequencies */
	for (uint8_t i = 0; i<16; i++){
		/* Testing addresses */
		for(uint8_t j=0;j<4;j++){
			/* Testing data lines */
			for (uint8_t k = 0; k<4;k++){
				qspi_init( ((i+1)*1000000), j, k);
				if(memory_test(length) != 0){
					result++;
				}
				else{
					success++;
				}
			}
		}
	}
	printf("\nFailed: %d", result);
	printf("\nPassed: %d", success);
	return result;
}
