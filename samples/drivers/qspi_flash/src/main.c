/*
 * Copyright (c) 2019 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <drivers/qspi.h>
#include <logging/log.h>


/* Matches LFS_NAME_MAX */
#define MAX_PATH_LEN 255

LOG_MODULE_REGISTER(qspi_flash, LOG_LEVEL_DBG);

#if defined(TCA2_INTEGRATION)
#include "target_comm.h"
target_comm_t g_target_comm;
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define QSPI_STD_CMD_JEDEC_ID	0x9F	// CMD: Jedec ID
#define QSPI_STD_CMD_SE			0x20	// CMD: Sector erase
#define QSPI_STD_CMD_READ		0x03	// CMD: Read data
#define QSPI_STD_CMD_WRITE		0x02	// CMD: Read data

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

struct qspi_cmd cmdBuff;
struct qspi_buf txBuff;
struct device *qspi;				// Defines pointer to qspi device
struct qspi_config qspi_cfg;		// Config for qspi device
u8_t rxbuffer_1[1024]={0};
u8_t rxbuffer_2[512]={0};
u8_t rxbuffer_3[2048]={0};

/* scheduling priority used by each thread */
static struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);
static struct k_poll_event async_evt =
	K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
				 K_POLL_MODE_NOTIFY_ONLY,
				 &async_sig);
#define PRIORITY 7
/* Private function prototypes -----------------------------------------------*/
static void qspi_config(struct qspi_config * pCfg);
//void qspi_init(uint32_t frequency, uint8_t addressMode, uint8_t dataLines);

/* TESTING FUNCTIONS */
//uint32_t memory_test(uint32_t length);
//uint32_t read_device_id(void);
//uint32_t memory_complex_test(uint32_t length);

/* Private functions ---------------------------------------------------------*/
void main(void)
{
#if defined(TCA2_INTEGRATION)
	tca_init(&g_target_comm, &TARGET_COMM_DEFAULT_CONFIG);
#endif

	/* Get binding */
	qspi = device_get_binding(DT_NORDIC_NRF_QSPI_QSPI_0_LABEL);

//	/* Check acquired pointer */
	if (!qspi) {
		LOG_INF("QSPI: Device driver not found.");
	}
	else{
		LOG_INF("QSPI: Device driver found.");
	}

	/* Assign config */
	qspi_config(&qspi_cfg);//K_THREAD_DEFINE(blink3_id, STACKSIZE, blink3, NULL, NULL, NULL,
	//		PRIORITY, 0, 300);
	//
	//K_THREAD_DEFINE(blink4_id, STACKSIZE, blink4, NULL, NULL, NULL,
	//		PRIORITY, 1, 300);
	//
	//K_THREAD_DEFINE(blink5_id, STACKSIZE, blink5, NULL, NULL, NULL,
	//		PRIORITY, 0, 300);

	/* Set configuration of the driver */
	if( qspi_configure(qspi, &qspi_cfg) != 0){
		printk("\nInitialisation failed");
	}

	u8_t txbuffer[8]={0,0,0,4,5,6,7,8};
	u8_t rxbuffer[8]={0};
	u8_t jedecId = 0x9F;
	struct qspi_buf txBuff = {
			.buf = &txbuffer,
			.len = 4
	};

	struct qspi_buf rxBuff = {
			.buf = &rxbuffer,
			.len = 4
	};

	struct qspi_cmd cmdBuff = {
			.op_code = QSPI_STD_CMD_WRITE,
			.tx_buf = &txBuff,
			.rx_buf = NULL,
	};
//	if(qspi_send_cmd(qspi, &cmdBuff) != 0){
	if(qspi_send_cmd_async(qspi, &cmdBuff, &async_sig) != 0){
		printk("\nTRANSFER FAILED");
	}
	int result = 0;
	printk("\nErasing...");
	result = qspi_erase_async(qspi, 0, 0x1000U, &async_sig);
	if ( result != 0){
		printk("ERROR: %d", result);
	}
	else{
		printk("OK");
	}

	printk("\nWritting...");
	result = qspi_write_async(qspi, &txBuff, 0, &async_sig);
	if( result != 0){
		printk("ERROR: %d", result);
	}
	else{
		printk("OK");
	}

	printk("\nReading...");
	result = qspi_read_async(qspi, &rxBuff, 0, &async_sig);
	if( result != 0){
		printk("ERROR: %d", result);
	}
	else{
		printk("OK");
	}
	printk("\nCHlosta");
//	/* Identify the flash */
//	LOG_INF("Vendor ID: %x", read_device_id());
//
//	/* Perform the test */
//	memory_test(4);
//	memory_complex_test(4);
#if defined(TCA2_INTEGRATION)
	while (1) {
		tca_send_and_receive(&g_target_comm);
	}
#endif
//while(1){
//	k_sleep(500);
//	printk("\n MAIN TASK");
//}
	LOG_INF("End.");
}


//
///**
// * @brief Configures QSPI peripherial
// *
// * @param pCfg	- pointer to the qspi structure
// * @retval None
// */
static void qspi_config(struct qspi_config * pCfg){
	pCfg->cs_pin = DT_NORDIC_NRF_QSPI_40029000_JEDEC_QSPI_NOR_0_CS_PIN;
	pCfg->frequency = 1000000;
	pCfg->data_lines = QSPI_DATA_LINES_SINGLE;/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7
	pCfg->address = QSPI_ADDRESS_MODE_24BIT;
	pCfg->cs_high_time = 0;
	pCfg->mode = QSPI_MODE_0;
}

////---------------------------------------------------------------------------		TEST FUNCTIONS
//
///**
// * @brief Initialises QSPI peripherial - API for testing purposes
// *
// * @param frequency		- Desired frequency of the QSPI bus.
// * @param addressMode	- Mode of addressing
// * @param addressMode	- Mode of used datalines (1,2 or 4)
// * @retval None
// */
//void qspi_init(uint32_t frequency, uint8_t addressMode, uint8_t dataLines){
//	qspi_cfg.frequency = frequency;
//	qspi_cfg.operation = (	QSPI_CS_DELAY_SET(8) 				|
//						QSPI_DATA_LINES_SET(dataLines)			|
//						QSPI_ADDRESS_MODE_SET(addressMode));
//}
//
//
///**
// * @brief Reads device ID (JEDEC ID)
// *
// * @param None
// * @retval 0 If failed, JEDEC ID if success.
// */
//uint32_t read_device_id(void){
//	uint32_t jedec_id = 0;
//	/* Erase sector */
//	if(qspi_cmd_xfer(qspi, &qspi_cfg, NULL, 0, chip_mem ,JEDEC_ID_SIZE , QSPI_STD_CMD_JEDEC_ID, 0) != HAL_OK){
//		/* Erasing status: ERROR */
//		LOG_WRN("Unable to sector erase");
//		return 1;/* size of stack area used by each thread */

//	}
//
//	memcpy(&jedec_id,chip_mem,JEDEC_ID_SIZE);
//
//	return jedec_id;
//}
//
/**
 * @brief Performs basic test of the QSPI FLASH memory
 *
 * @param length	length of the tests in bytes
 * @retval 0 If successful, errno code otherwise.
 */
uint32_t memory_test(uint32_t length){
	LOG_INF("memory_test(%d)", length);

	struct qspi_buf rxBuff = {
			.buf = chip_mem,
			.len = length
	};

	struct qspi_buf txBuff = {
			.buf = test,
			.len = length
	};

	/* Check input parameter */
	if(length > FLASH_SECTOR_SIZE)
	{
		LOG_WRN("Simple memory test is limited to FLASH_SECTOR_SIZE=4096");
		return 1;
	}
	printf("\n Erasing memory...");
	/* Erase sector */

//	if(qspi_cmd_xfer(qspi, &qspi_cfg, NULL, 0, NULL ,0 , QSPI_STD_CMD_SE, 0) != HAL_OK){
	if(qspi_erase(qspi,0,0x1000U) != HAL_OK){
		/* Erasing status: ERROR */
		LOG_WRN("Unable to sector erase");
		return 2;
	}
	printf("OK");
	printf("\n Reading memory...");
	/* Chip read */
//	if(qspi_read(qspi, &qspi_cfg, chip_mem, length, 0) != HAL_OK){

	if(qspi_read(qspi, &rxBuff, 0) != HAL_OK){
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
//	if(qspi_write(qspi, &qspi_cfg, test, length, 0) != HAL_OK){
	if(qspi_write(qspi, &txBuff, 0) != HAL_OK){
		/* Writting status: ERROR */
		LOG_WRN("Unable to write data");
		return 5;
	}
	/* size of stack area used by each thread */
	#define STACKSIZE 1024

	/* scheduling priority used by each thread */
	#define PRIORITY 7
	printf("OK");
	printf("\n Reading memory...");
	/* Chip read */
//	if(qspi_read(qspi, &qspi_cfg, chip_mem, length, 0) != HAL_OK){
	if(qspi_read(qspi, &rxBuff, 0) != HAL_OK){
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
//
///**
// * @brief Performs comples test of the QSPI FLASH memory and QSPI bus
// * This tests changes QSPI configuration and check
// *
// * @param length	length of the tests in bytes
// * @retval 0 If successful, errno code otherwise.VSUP
// */
//uint32_t memory_complex_test(uint32_t length){
//	uint32_t result = 0;
//	uint32_t success = 0;
//	/* Testing frequencies */
//	for (uint8_t i = 0; i<16; i++){
//		/* Testing addresses */
//		for(uint8_t j=0;j<4;j++){
//			/* Testing data lines */
//			for (uint8_t k = 0; k<4;k++){
//				qspi_init( ((i+1)*1000000), j, k);
//				if(memory_test(length) != 0){
//					result++;
//				}
//				else{
//					success++;
//				}
//			}
//		}
//	}
//	printf("\nFailed: %d", result);
//	printf("\nPassed: %d", success);
//	return result;
//}
void blink1(void)
{
//	blink(PORT0, 100, LED0, 0);
	while(1){
		printk("\n ERASING");
		if(qspi_erase(qspi,0,0x1000U) != NRFX_SUCCESS){
			printk("\n ERASE FAILED");
		}
		else{
			printk("\n ERASE SUCCESS");
		}
//		printk("\n TASK1");
//		qspi_erase(qspi,0,0x1000U);
		k_sleep(1000);
	}
}

void blink2(void)
{
//	blink(PORT1, 1000, LED1, 1);
	while(1){
		printk("\n WRITTING");
		u8_t txbuffer[8]={1,2,3,4,1,2,3,4};
		struct qspi_buf txBuff = {
				.buf = &txbuffer,
				.len = 8
		};
		if(qspi_write(qspi, &txBuff, 0) != NRFX_SUCCESS){
			printk("\n WRITE FAILED");
		}
		else{
			printk("\n WRITE SUCCESS");
		}
		k_sleep(1000);
	}

}
void blink3(void)
{
	while(1){
		printk("\nReading_1...");
//		u8_t rxbuffer_1[256]={0};
		int result = 0;
		struct qspi_buf rxBuff = {
				.buf = &rxbuffer_1,
				.len = 1024
		};
		result = qspi_read_async(qspi, &rxBuff, 0, &async_sig);
		if( result != 0){
			printk("ERROR: %d", result);
		}
		else{
			printk("OK");
		}
	}
}

void blink4(void)
{
	while(1){
		printk("\nReading_2...");
//		u8_t rxbuffer_2[1024]={0};
		int result = 0;
		struct qspi_buf rxBuff = {
				.buf = &rxbuffer_2,
				.len = 512
		};
		result = qspi_read_async(qspi, &rxBuff, 0, &async_sig);
		if( result != 0){
			printk("ERROR: %d", result);
		}
		else{
			printk("OK");
		}
	}
}

void blink5(void)
{
	while(1){
		printk("\nReading_3...");

		int result = 0;
		struct qspi_buf rxBuff = {
				.buf = &rxbuffer_3,
				.len = 2048
		};
		result = qspi_read_async(qspi, &rxBuff, 0, &async_sig);
		if( result != 0){
			printk("ERROR: %d", result);
		}
		else{
			printk("OK");
		}
	}
}


//K_THREAD_DEFINE(blink1_id, STACKSIZE, blink1, NULL, NULL, NULL,
//		PRIORITY, 0, 200);
//K_THREAD_DEFINE(blink2_id, STACKSIZE, blink2, NULL, NULL, NULL,
//		PRIORITY, 0, 300);
K_THREAD_DEFINE(blink3_id, STACKSIZE, blink3, NULL, NULL, NULL,
		PRIORITY, 0, 300);

K_THREAD_DEFINE(blink4_id, STACKSIZE, blink4, NULL, NULL, NULL,
		PRIORITY, 1, 300);

K_THREAD_DEFINE(blink5_id, STACKSIZE, blink5, NULL, NULL, NULL,
		PRIORITY, 0, 300);
