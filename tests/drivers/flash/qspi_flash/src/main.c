/*
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <drivers/flash.h>
#include <device.h>

/* configuration derived from DT */
#define FLASH_SIMULATOR_BASE_OFFSET 0
#define FLASH_SIMULATOR_ERASE_UNIT 4096
#define FLASH_SIMULATOR_PROG_UNIT 4
#define FLASH_SIMULATOR_FLASH_SIZE DT_INST_0_JEDEC_QSPI_NOR_SIZE

/* Offset between pages */
#define TEST_SIM_FLASH_SIZE 4096

#define TEST_SIM_FLASH_END (TEST_SIM_FLASH_SIZE +\
			   FLASH_SIMULATOR_BASE_OFFSET)

static struct device *flash_dev;
static u8_t test_read_buf[TEST_SIM_FLASH_SIZE];

static u32_t p32_inc;

void pattern32_ini(u32_t val)
{
	p32_inc = val;
}

static u32_t pattern32_inc(void)
{
	return p32_inc++;
}

static u32_t pattern32_flat(void)
{
	return p32_inc;
}

static void test_check_pattern32(off_t start, u32_t (*pattern_gen)(void),
				 size_t size)
{
	off_t off;
	u32_t val32, r_val32;
	int rc;

	for (off = 0; off < size; off += 4) {
		val32 = pattern_gen();
//		printk("Val 0x%x off 0x%x\n", val32, off);
		rc = flash_write(flash_dev, start +
					    off,
				 &val32, sizeof(val32));
		zassert_equal(0, rc, "flash_write should succedd");
		rc = flash_read(flash_dev, start + off, &r_val32,
				sizeof(r_val32));
		zassert_equal(0, rc, "flash_write should succedd");
		zassert_equal(val32, r_val32,
			     "flash word at offset 0x%x has value 0x%x instead of 0x%x",
			     start + off, r_val32, val32);
	}
}

static void test_int(void)
{
	int rc;
	off_t i;

	flash_dev = device_get_binding(DT_INST_0_JEDEC_QSPI_NOR_LABEL);

	zassert_true(flash_dev != NULL,
		     "Simulated flash driver was not found!");

	rc = flash_read(flash_dev, FLASH_SIMULATOR_BASE_OFFSET,
			test_read_buf, sizeof(test_read_buf));
	zassert_equal(0, rc, "flash_read should succedd");

//	for (i = 0; i < sizeof(test_read_buf); i++) {
//		zassert_equal(0xff, test_read_buf[i],
//			     "sim flash byte at offset 0x%x has value 0x%0x02",
//			     i, test_read_buf[i]);
//	}
}

static void test_write_read(void)
{
	off_t off;
	u32_t val32 = 0, r_val32;
	int rc;

	for (off = 0; off < TEST_SIM_FLASH_SIZE; off += 4) {
		rc = flash_write_protection_set(flash_dev, false);
		zassert_equal(0, rc, NULL);
		rc = flash_write(flash_dev, FLASH_SIMULATOR_BASE_OFFSET +
					    off,
				 &val32, sizeof(val32));
		zassert_equal(0, rc,
			      "flash_write (%d) should succedd at off 0x%x", rc,
			       FLASH_SIMULATOR_BASE_OFFSET + off);
		val32++;
	}

	val32 = 0;

	for (off = 0; off < TEST_SIM_FLASH_SIZE; off += 4) {
		rc = flash_read(flash_dev, FLASH_SIMULATOR_BASE_OFFSET +
					   off,
				&r_val32, sizeof(r_val32));
		zassert_equal(0, rc, "flash_write should succedd");
		zassert_equal(val32, r_val32,
			     "flash byte at offset 0x%x has value 0x%0x02",
			     off, r_val32);
		val32++;
	}
}

static void test_erase(void)
{
	int rc;

	rc = flash_erase(flash_dev, FLASH_SIMULATOR_BASE_OFFSET +
			 FLASH_SIMULATOR_ERASE_UNIT,
			 FLASH_SIMULATOR_ERASE_UNIT);
	zassert_equal(0, rc, "flash_erase should succedd");

	TC_PRINT("Incremental pattern expected\n");
	pattern32_ini(0);
	test_check_pattern32(FLASH_SIMULATOR_BASE_OFFSET, pattern32_inc,
			     FLASH_SIMULATOR_ERASE_UNIT);

	TC_PRINT("Erased area expected\n");
	pattern32_ini(0xffffffff);
	test_check_pattern32(FLASH_SIMULATOR_BASE_OFFSET +
			     FLASH_SIMULATOR_ERASE_UNIT, pattern32_flat,
			     FLASH_SIMULATOR_ERASE_UNIT);

	TC_PRINT("Incremental pattern expected\n");
	pattern32_ini(FLASH_SIMULATOR_ERASE_UNIT*2 /
		      FLASH_SIMULATOR_PROG_UNIT);
	test_check_pattern32(FLASH_SIMULATOR_BASE_OFFSET +
			     FLASH_SIMULATOR_ERASE_UNIT*2, pattern32_inc,
			     FLASH_SIMULATOR_ERASE_UNIT*2);
}

static void test_access(void)
{
	u32_t data[2] = {0};
	int rc;

	rc = flash_write_protection_set(flash_dev, true);
	zassert_equal(0, rc, NULL);

	rc = flash_write(flash_dev, FLASH_SIMULATOR_BASE_OFFSET,
				 data, 4);
	zassert_equal(-EACCES, rc, "Unexpected error code (%d)", rc);

	rc = flash_erase(flash_dev, FLASH_SIMULATOR_BASE_OFFSET,
			 FLASH_SIMULATOR_ERASE_UNIT);
	zassert_equal(-EACCES, rc, "Unexpected error code (%d)", rc);
}

static void test_out_of_bounds(void)
{
	int rc;
	u8_t data[4] = {0};

	rc = flash_write_protection_set(flash_dev, false);

	rc = flash_write(flash_dev, FLASH_SIMULATOR_BASE_OFFSET - 4,
				 data, 4);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_write(flash_dev, FLASH_SIMULATOR_BASE_OFFSET - 4,
				 data, 8);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_write(flash_dev, TEST_SIM_FLASH_END,
				 data, 4);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_write(flash_dev, TEST_SIM_FLASH_END - 4,
				 data, 8);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_erase(flash_dev, FLASH_SIMULATOR_BASE_OFFSET -
			 FLASH_SIMULATOR_ERASE_UNIT,
			 FLASH_SIMULATOR_ERASE_UNIT);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_erase(flash_dev, TEST_SIM_FLASH_END,
			 FLASH_SIMULATOR_ERASE_UNIT);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_erase(flash_dev, FLASH_SIMULATOR_BASE_OFFSET -
			 FLASH_SIMULATOR_ERASE_UNIT*2,
			 FLASH_SIMULATOR_ERASE_UNIT*2);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_erase(flash_dev, TEST_SIM_FLASH_END -
			 FLASH_SIMULATOR_ERASE_UNIT,
			 FLASH_SIMULATOR_ERASE_UNIT*2);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_read(flash_dev, FLASH_SIMULATOR_BASE_OFFSET - 4,
				 data, 4);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_read(flash_dev, FLASH_SIMULATOR_BASE_OFFSET - 4,
				 data, 8);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_read(flash_dev, TEST_SIM_FLASH_END,
				 data, 4);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_read(flash_dev, TEST_SIM_FLASH_END - 4, data, 8);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);
}

static void test_align(void)
{
	int rc;
	u8_t data[4] = {0};

	rc = flash_read(flash_dev, FLASH_SIMULATOR_BASE_OFFSET + 1,
				 data, 4);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_write(flash_dev, FLASH_SIMULATOR_BASE_OFFSET + 1,
				 data, 4);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_write(flash_dev, FLASH_SIMULATOR_BASE_OFFSET,
				 data, 3);
	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_erase(flash_dev, FLASH_SIMULATOR_BASE_OFFSET + 1,
			 FLASH_SIMULATOR_ERASE_UNIT);

	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);

	rc = flash_erase(flash_dev, FLASH_SIMULATOR_BASE_OFFSET,
			 FLASH_SIMULATOR_ERASE_UNIT + 1);

	zassert_equal(-EINVAL, rc, "Unexpected error code (%d)", rc);
}

static void test_double_write(void)
{
	int rc;
	u8_t data[4] = {0};

	rc = flash_erase(flash_dev, FLASH_SIMULATOR_BASE_OFFSET,
			 FLASH_SIMULATOR_ERASE_UNIT);
	zassert_equal(0, rc, "flash_erase should succedd");

	rc = flash_write(flash_dev, FLASH_SIMULATOR_BASE_OFFSET,
				 data, 4);
	zassert_equal(0, rc, "flash_write should succedd");

	rc = flash_write(flash_dev, FLASH_SIMULATOR_BASE_OFFSET,
				 data, 4);
	zassert_equal(-EIO, rc, "Unexpected error code (%d)", rc);
}

#if 0
/**
 * @brief Performs basic test of the QSPI FLASH memory
 *
 * @param length	length of the tests in bytes
 * @retval 0 If successful, errno code otherwise.
 */
static void test_memory(void){
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
static void test_memory_complex(void){
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
#endif
void test_main(void)
{
	ztest_test_suite(flash_sim_api,
			 ztest_unit_test(test_int),
			 ztest_unit_test(test_write_read),
			 ztest_unit_test(test_erase),
			 ztest_unit_test(test_access),
			 ztest_unit_test(test_out_of_bounds),
			 ztest_unit_test(test_align),
			 ztest_unit_test(test_double_write));

	ztest_run_test_suite(flash_sim_api);
}
