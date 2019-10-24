/*
 * Copyright (c) 2019, Nordic Semiconductor ASA
 *
 * This driver is heavily inspired from the spi_flash_w25qxxdv.c SPI NOR driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/flash.h>
#include <drivers/qspi.h>
#include <init.h>
#include <string.h>
#include <logging/log.h>

#include "qspi_nor.h"
#include "flash_priv.h"

LOG_MODULE_REGISTER(qspi_nor, CONFIG_FLASH_LOG_LEVEL);

#define QSPI_NOR_MAX_ADDR_WIDTH 4

/**
 * struct qspi_nor_data - Structure for defining the QSPI NOR access
 * @qspi: The QSPI device
 * @qspi_cfg: The QSPI configuration
 * @sem: The semaphore to access to the flash
 */
struct qspi_nor_data {
	struct device *qspi;
	struct qspi_config qspi_cfg;
	struct k_sem sem;
};

#if defined(CONFIG_MULTITHREADING)
#define SYNC_INIT() k_sem_init(	\
		&((struct qspi_nor_data *)dev->driver_data)->sem, 1, UINT_MAX)
#define SYNC_LOCK() k_sem_take(&driver_data->sem, K_FOREVER)
#define SYNC_UNLOCK() k_sem_give(&driver_data->sem)
#else
#define SYNC_INIT()
#define SYNC_LOCK()
#define SYNC_UNLOCK()
#endif

static int qspi_set_active_mem(struct device *dev)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	static uint16_t current_mem = 0xFFFF;
	int ret = 0;
	/* We are using CS pin number to identify which memory is selected,
	 * as it is easy to compare and unique for every memory attached.
	 */
	if (current_mem != driver_data->qspi_cfg.cs_pin) {
		ret = qspi_configure(driver_data->qspi, &driver_data->qspi_cfg);
		current_mem = driver_data->qspi_cfg.cs_pin;
	}
	printk("%d", driver_data->qspi_cfg.cs_pin);
	return 0;
}

/**
 * @brief Retrieve the Flash JEDEC ID and compare it with the one expected
 *
 * @param dev The device structure
 * @param flash_id The flash info structure which contains the expected JEDEC ID
 * @return 0 on success, negative errno code otherwise
 */
static inline int qspi_nor_read_id(struct device *dev,
				  const struct qspi_nor_config *const flash_id)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;

	int ret = qspi_set_active_mem(dev);
	if (ret != 0) {
		return ret;
	}

	u8_t rx_b[QSPI_NOR_MAX_ID_LEN];
	struct qspi_buf q_rx_buf = {
		.buf = rx_b,
		.len = QSPI_NOR_MAX_ID_LEN
	};
	struct qspi_cmd cmd = {
		.op_code = QSPI_NOR_CMD_RDID,
		.rx_buf = &q_rx_buf,
		.tx_buf = NULL
	};

	if(qspi_send_cmd(driver_data->qspi, &cmd) != 0){
		return -EIO;
	}

	if (memcmp(flash_id->id, rx_b, QSPI_NOR_MAX_ID_LEN) != 0) {
		return -ENODEV;
	}

	return 0;
}

static int qspi_nor_read(struct device *dev, off_t addr, void *dest,
			size_t size)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	const struct qspi_nor_config *params = dev->config->config_info;

	int ret = qspi_set_active_mem(dev);
	if (ret != 0) {
		return ret;
	}

	/* should be between 0 and flash size */
	if ((addr < 0) || ((addr + size) > params->size)) {
		return -EINVAL;
	}
	struct qspi_buf rx_buf = {
		.buf = dest,
		.len = size
	};
	SYNC_LOCK();

	ret = qspi_read(driver_data->qspi, &rx_buf, addr);

	SYNC_UNLOCK();

	return ret;
}

static int qspi_nor_write(struct device *dev, off_t addr, const void *src,
			 size_t size)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	const struct qspi_nor_config *params = dev->config->config_info;

	int ret = qspi_set_active_mem(dev);
	if (ret != 0) {
		return ret;
	}
	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > params->size)) {
		return -EINVAL;
	}
	struct qspi_buf tx_buf = {
		.buf = src,
		.len = size
	};
	SYNC_LOCK();

	ret = qspi_write(driver_data->qspi, &tx_buf, addr);

	SYNC_UNLOCK();

	return 0;
}

static int qspi_nor_erase(struct device *dev, off_t addr, size_t size)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	const struct qspi_nor_config *params = dev->config->config_info;

	int ret = qspi_set_active_mem(dev);
	if (ret != 0) {
		return ret;
	}
	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > params->size)) {
		return -ENODEV;
	}

	SYNC_LOCK();

	ret = qspi_erase(driver_data->qspi, addr, size);

	SYNC_UNLOCK();

	return ret;
}

static int qspi_nor_write_protection_set(struct device *dev, bool write_protect)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	const struct qspi_nor_config *params = dev->config->config_info;

	int ret = qspi_set_active_mem(dev);
	if (ret != 0) {
		return ret;
	}
	struct qspi_cmd cmd = {
		.op_code = ((write_protect) ? QSPI_NOR_CMD_WRDI : QSPI_NOR_CMD_WREN),
		.tx_buf = NULL,
		.rx_buf = NULL
	};

	SYNC_LOCK();

	if(qspi_send_cmd(driver_data->qspi, &cmd) != 0){
		ret = -EIO;
	}

	if (params->id[0] == 0xbf && params->id[1] == 0x26 && ret == 0 && !write_protect) {
		cmd.op_code = QSPI_NOR_CMD_MCHP_UNLOCK;
		ret = qspi_send_cmd(driver_data->qspi, &cmd);
	}

	SYNC_UNLOCK();

	return ret;
}

/**
 * @brief Configure the flash
 *
 * @param dev The flash device structure
 * @param info The flash info structure
 * @return 0 on success, negative errno code otherwise
 */
static int qspi_nor_configure(struct device *dev)
{
	struct qspi_nor_data *data = dev->driver_data;
	const struct qspi_nor_config *params = dev->config->config_info;

	data->qspi = device_get_binding(DT_INST_0_JEDEC_QSPI_NOR_BUS_NAME);
	if (!data->qspi) {
		printk("errq");
		return -EINVAL;
	}

	int ret = qspi_set_active_mem(dev);
	if (ret != 0) {
		printk("errs");
		return ret;
	}

	/* now the spi bus is configured, we can verify the flash id */
	if (qspi_nor_read_id(dev, params) != 0) {
		printk("erri");
		return -ENODEV;
	}

	return 0;
}

/**
 * @brief Initialize and configure the flash
 *
 * @param name The flash name
 * @return 0 on success, negative errno code otherwise
 */
static int qspi_nor_init(struct device *dev)
{
	SYNC_INIT();
	printk("Init nor\n");
	return qspi_nor_configure(dev);
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

/* instance 0 size in bytes */
#define INST_0_BYTES (DT_INST_0_JEDEC_QSPI_NOR_SIZE / 8)

/* instance 0 page count */
#define LAYOUT_PAGES_COUNT (INST_0_BYTES / CONFIG_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE)

BUILD_ASSERT_MSG((CONFIG_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE * LAYOUT_PAGES_COUNT)
		 == INST_0_BYTES,
		 "QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE incompatible with flash size");

static const struct flash_pages_layout dev_layout = {
	.pages_count = LAYOUT_PAGES_COUNT,
	.pages_size = CONFIG_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE,
};
#undef LAYOUT_PAGES_COUNT

static void qspi_nor_pages_layout(struct device *dev,
				 const struct flash_pages_layout **layout,
				 size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api qspi_nor_api = {
	.read = qspi_nor_read,
	.write = qspi_nor_write,
	.erase = qspi_nor_erase,
	.write_protection = qspi_nor_write_protection_set,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = qspi_nor_pages_layout,
#endif
	.write_block_size = 1,
};


#define DATA_LINES(lines) \
	lines == 4 ? QSPI_DATA_LINES_QUAD :	\
	lines == 2 ? QSPI_DATA_LINES_DOUBLE :	\
	lines == 1 ? QSPI_DATA_LINES_SINGLE : -1

#define ADDRESS_SIZE(mode) \
	mode == 32 ? QSPI_ADDRESS_MODE_32BIT :	\
	mode == 24 ? QSPI_ADDRESS_MODE_24BIT :	\
	mode == 16 ? QSPI_ADDRESS_MODE_16BIT :	\
	mode == 8 ? QSPI_ADDRESS_MODE_8BIT : -1

static const struct qspi_nor_config flash_id = {
	.id = DT_INST_0_JEDEC_QSPI_NOR_JEDEC_ID,
	.has_be32k = DT_INST_0_JEDEC_QSPI_NOR_HAS_BE32K,
	.size = DT_INST_0_JEDEC_QSPI_NOR_SIZE / 8,
};

static struct qspi_nor_data qspi_nor_memory_data = {
	.qspi_cfg = {
		.cs_pin = DT_INST_0_JEDEC_QSPI_NOR_CS_PIN,
		.frequency = DT_INST_0_JEDEC_QSPI_NOR_FREQUENCY,
		.mode = DT_INST_0_JEDEC_QSPI_NOR_QSPI_MODE,
		.cs_high_time = DT_INST_0_JEDEC_QSPI_NOR_CS_HIGH_TIME,
		.data_lines = DATA_LINES(DT_INST_0_JEDEC_QSPI_NOR_DATA_LINES),
		.address = ADDRESS_SIZE(DT_INST_0_JEDEC_QSPI_NOR_ADDRESS_SIZE)
	}
};

DEVICE_AND_API_INIT(qspi_flash_memory, DT_INST_0_JEDEC_QSPI_NOR_LABEL,
		    &qspi_nor_init, &qspi_nor_memory_data,
		    &flash_id, POST_KERNEL, CONFIG_QSPI_NOR_INIT_PRIORITY,
		    &qspi_nor_api);
