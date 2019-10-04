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
//#include <drivers/qspi.h>
#include <logging/log.h>
#include <fs/fs.h>
#include <fs/littlefs.h>
#include <storage/flash_map.h>

/* Matches LFS_NAME_MAX */
#define MAX_PATH_LEN 255

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_storage_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &storage,
	.storage_dev = (void *)DT_FLASH_AREA_STORAGE_ID,
	.mnt_point = "/lfs",
};

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

//struct device *qspi;				// Defines pointer to qspi device
//struct qspi_config qspi_cfg;		// Config for qspi device

/* Private function prototypes -----------------------------------------------*/
//static void qspi_configure(struct qspi_config * pCfg);
//static void qspi_configure(void);
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
	struct fs_mount_t *mp = &lfs_storage_mnt;
	unsigned int id = (uintptr_t)mp->storage_dev;
	char fname[MAX_PATH_LEN];
	struct fs_statvfs sbuf;
	const struct flash_area *pfa;
	int rc;

	snprintf(fname, sizeof(fname), "%s/boot_count", mp->mnt_point);

	rc = flash_area_open(id, &pfa);
	if (rc < 0) {
		printk("FAIL: unable to find flash area %u: %d\n",
		       id, rc);
		return;
	}

	printk("Area %u at 0x%x on %s for %u bytes\n",
	       id, (unsigned int)pfa->fa_off, pfa->fa_dev_name,
	       (unsigned int)pfa->fa_size);

	/* Optional wipe flash contents */
	if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
		printk("Erasing flash area ... ");
		rc = flash_area_erase(pfa, 0, pfa->fa_size);
		printk("%d\n", rc);
		flash_area_close(pfa);
	}

	rc = fs_mount(mp);
	if (rc < 0) {
		printk("FAIL: mount id %u at %s: %d\n",
		       (unsigned int)mp->storage_dev, mp->mnt_point,
		       rc);
		return;
	}
	printk("%s mount: %d\n", mp->mnt_point, rc);

	rc = fs_statvfs(mp->mnt_point, &sbuf);
	if (rc < 0) {
		printk("FAIL: statvfs: %d\n", rc);
		goto out;
	}

	printk("%s: bsize = %lu ; frsize = %lu ;"
	       " blocks = %lu ; bfree = %lu\n",
	       mp->mnt_point,
	       sbuf.f_bsize, sbuf.f_frsize,
	       sbuf.f_blocks, sbuf.f_bfree);

	struct fs_dirent dirent;

	rc = fs_stat(fname, &dirent);
	printk("%s stat: %d\n", fname, rc);
	if (rc >= 0) {
		printk("\tfn '%s' siz %u\n", dirent.name, dirent.size);
	}

	struct fs_file_t file;

	rc = fs_open(&file, fname);
	if (rc < 0) {
		printk("FAIL: open %s: %d\n", fname, rc);
		goto out;
	}

	u32_t boot_count = 0;

	if (rc >= 0) {
		rc = fs_read(&file, &boot_count, sizeof(boot_count));
		printk("%s read count %u: %d\n", fname, boot_count, rc);
		rc = fs_seek(&file, 0, FS_SEEK_SET);
		printk("%s seek start: %d\n", fname, rc);

	}

	boot_count += 1;
	rc = fs_write(&file, &boot_count, sizeof(boot_count));
	printk("%s write new boot count %u: %d\n", fname,
	       boot_count, rc);

	rc = fs_close(&file);
	printk("%s close: %d\n", fname, rc);

	struct fs_dir_t dir = { 0 };

	rc = fs_opendir(&dir, mp->mnt_point);
	printk("%s opendir: %d\n", mp->mnt_point, rc);

	while (rc >= 0) {
		struct fs_dirent ent = { 0 };

		rc = fs_readdir(&dir, &ent);
		if (rc < 0) {
			break;
		}
		if (ent.name[0] == 0) {
			printk("End of files\n");
			break;
		}
		printk("  %c %u %s\n",
		       (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
		       ent.size,
		       ent.name);
	}

	(void)fs_closedir(&dir);

out:
	rc = fs_unmount(mp);
	printk("%s unmount: %d\n", mp->mnt_point, rc);









	/* Get binding */
//	qspi = device_get_binding(DT_NORDIC_NRF_QSPI_QSPI_0_LABEL);

//	/* Check acquired pointer */
//	if (!qspi) {
//		LOG_INF("QSPI: Device driver not found.");
//	}
//	else{
//		LOG_INF("QSPI: Device driver found.");
//	}





	/* Assign config */
//	qspi_configure(&qspi_cfg);
//	qspi_configure();
//	qspi_init(8000000,QSPI_ADDRESS_MODE_24BIT, QSPI_DATA_LINES_QUAD);
//
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

	LOG_INF("End.");
}


//
///**
// * @brief Configures QSPI peripherial
// *
// * @param pCfg	- pointer to the qspi structure
// * @retval None
// */
////static void qspi_configure(struct qspi_config * pCfg){
////	pCfg->frequency = 8000000;
////	pCfg->operation = (	QSPI_CS_DELAY_SET(8) 							|
////						QSPI_DATA_LINES_SET(QSPI_DATA_LINES_QUAD)		|
////						QSPI_ADDRESS_MODE_SET(QSPI_ADDRESS_MODE_24BIT));
////}
//static void qspi_configure(void){
//	qspi_cfg.frequency = 8000000;
//	qspi_cfg.operation = (	QSPI_CS_DELAY_SET(8) 							|
//						QSPI_DATA_LINES_SET(QSPI_DATA_LINES_QUAD)			|
//						QSPI_ADDRESS_MODE_SET(QSPI_ADDRESS_MODE_24BIT));
//}
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
//		return 1;
//	}
//
//	memcpy(&jedec_id,chip_mem,JEDEC_ID_SIZE);
//
//	return jedec_id;
//}
//
///**
// * @brief Performs basic test of the QSPI FLASH memory
// *
// * @param length	length of the tests in bytes
// * @retval 0 If successful, errno code otherwise.
// */
//uint32_t memory_test(uint32_t length){
//	LOG_INF("memory_test(%d)", length);
//
//	/* Check input parameter */
//	if(length > FLASH_SECTOR_SIZE)
//	{
//		LOG_WRN("Simple memory test is limited to FLASH_SECTOR_SIZE=4096");
//		return 1;
//	}
//	printf("\n Erasing memory...");
//	/* Erase sector */
//	if(qspi_cmd_xfer(qspi, &qspi_cfg, NULL, 0, NULL ,0 , QSPI_STD_CMD_SE, 0) != HAL_OK){
//		/* Erasing status: ERROR */
//		LOG_WRN("Unable to sector erase");
//		return 2;
//	}
//	printf("OK");
//	printf("\n Reading memory...");
//	/* Chip read */
//	if(qspi_read(qspi, &qspi_cfg, chip_mem, length, 0) != HAL_OK){
//		/* Reading status: ERROR */
//		LOG_WRN("Unable to read data");
//		return 3;
//	}
//	printf("OK");
//	printf("\n Checking memory...");
//
//	/* Memory check -  expect all 1s */
//	for(uint16_t i = 0; i < length; i++)
//	{
//		if(chip_mem[i] != 0xFF)
//		{
//			LOG_WRN("Memory read incorrect %d = %x", i, chip_mem[i]);
//		return 4;
//		}
//	}
//
//	printf("OK");
//	printf("\n Writting to memory...");
//	/* Fill Tx buff with data */
//	for(uint32_t i = 0; i < length; i++)
//	{
//		test[i] = i % 0xFF;
//	}
//
//	/* Chip write */
//	if(qspi_write(qspi, &qspi_cfg, test, length, 0) != HAL_OK){
//		/* Writting status: ERROR */
//		LOG_WRN("Unable to write data");
//		return 5;
//	}
//
//	printf("OK");
//	printf("\n Reading memory...");
//	/* Chip read */
//	if(qspi_read(qspi, &qspi_cfg, chip_mem, length, 0) != HAL_OK){
//		/* Reading status: ERROR */
//		LOG_WRN("Unable to read data");
//		return 6;
//	}
//
//	printf("OK");
//	printf("\n Comparing memory...");
//
//	/* Compare write to read */
//	for(uint32_t i = 0; i < length; i++)
//	{
//		if(test[i] != chip_mem[i])
//		{
//			LOG_WRN("Error: compare write to read mismatch %x vs %x", test[i], chip_mem[i]);
//			return 7;
//		}
//	}
//
//
//	printf("OK");
//
//	LOG_INF("Memory test OK");
//
//	return 0;
//}
//
///**
// * @brief Performs comples test of the QSPI FLASH memory and QSPI bus
// * This tests changes QSPI configuration and check
// *
// * @param length	length of the tests in bytes
// * @retval 0 If successful, errno code otherwise.
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
