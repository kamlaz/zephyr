/*
 * Copyright (c) 2018 Savoir-Faire Linux.
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
 * struct spi_nor_data - Structure for defining the SPI NOR access
 * @spi: The SPI device
 * @spi_cfg: The SPI configuration
 * @cs_ctrl: The GPIO pin used to emulate the SPI CS if required
 * @sem: The semaphore to access to the flash
 */
struct qspi_nor_data {
	struct device *qspi;
	struct qspi_config qspi_cfg;
#ifdef DT_INST_0_JEDEC_QSPI_NOR_CS_GPIOS_CONTROLLER
	struct qspi_cs_control cs_ctrl;
#endif /* DT_INST_0_JEDEC_SPI_NOR_CS_GPIOS_CONTROLLER */
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
	const struct qspi_nor_config *params = dev->config->config_info;


	u8_t buf[QSPI_NOR_MAX_ID_LEN];

	if(qspi_cmd_xfer(driver_data->qspi, &driver_data->qspi_cfg, NULL, 0, buf ,QSPI_NOR_MAX_ID_LEN , QSPI_NOR_CMD_RDID, 0) != 0){
		return -EIO;
	}

	if (memcmp(flash_id->id, buf, QSPI_NOR_MAX_ID_LEN) != 0) {
		return -ENODEV;
	}

	return 0;
}

static int qspi_nor_read(struct device *dev, off_t addr, void *dest,
			size_t size)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	const struct qspi_nor_config *params = dev->config->config_info;

	int ret;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((addr + size) > params->size)) {
		return -EINVAL;
	}

	SYNC_LOCK();

	ret = qspi_read(driver_data->qspi, &driver_data->qspi_cfg, dest, size, addr);

	SYNC_UNLOCK();

	return ret;
}

static int qspi_nor_write(struct device *dev, off_t addr, const void *src,
			 size_t size)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	const struct qspi_nor_config *params = dev->config->config_info;
	int ret;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > params->size)) {
		return -EINVAL;
	}

	SYNC_LOCK();

	ret = qspi_write(driver_data->qspi, &driver_data->qspi_cfg, src, size, addr);

	SYNC_UNLOCK();

	return 0;
}

static int qspi_nor_erase(struct device *dev, off_t addr, size_t size)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	const struct qspi_nor_config *params = dev->config->config_info;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > params->size)) {
		return -ENODEV;
	}

	SYNC_LOCK();

	while (size) {

		if (size == params->size) {
			/* chip erase */
			qspi_cmd_xfer(driver_data->qspi, &driver_data->qspi_cfg, NULL, 0, NULL ,0 , QSPI_NOR_CMD_CE, 0);
			size -= params->size;
		} else if ((size >= QSPI_NOR_BLOCK_SIZE)
			   && QSPI_NOR_IS_BLOCK_ALIGNED(addr)) {
			/* 64 KiB block erase */
			qspi_cmd_xfer(driver_data->qspi, &driver_data->qspi_cfg, NULL, 0, NULL ,0 , QSPI_NOR_CMD_BE, 0);
			addr += QSPI_NOR_BLOCK_SIZE;
			size -= QSPI_NOR_BLOCK_SIZE;
		} else if ((size >= QSPI_NOR_BLOCK32_SIZE)
			   && QSPI_NOR_IS_BLOCK32_ALIGNED(addr)) {
			/* 32 KiB block erase */
			qspi_cmd_xfer(driver_data->qspi, &driver_data->qspi_cfg, NULL, 0, NULL ,0 , QSPI_NOR_CMD_BE_32K, 0);
			addr += QSPI_NOR_BLOCK32_SIZE;
			size -= QSPI_NOR_BLOCK32_SIZE;
		} else if ((size >= QSPI_NOR_SECTOR_SIZE)
			   && QSPI_NOR_IS_SECTOR_ALIGNED(addr)) {
			/* sector erase */
			qspi_cmd_xfer(driver_data->qspi, &driver_data->qspi_cfg, NULL, 0, NULL ,0 , QSPI_NOR_CMD_SE, 0);
			addr += QSPI_NOR_SECTOR_SIZE;
			size -= QSPI_NOR_SECTOR_SIZE;
		} else {
			/* minimal erase size is at least a sector size */
			SYNC_UNLOCK();
			LOG_DBG("unsupported at 0x%lx size %zu", (long)addr,
				size);
			return -EINVAL;
		}
	}

	SYNC_UNLOCK();

	return 0;
}

static int qspi_nor_write_protection_set(struct device *dev, bool write_protect)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	int ret;

	SYNC_LOCK();

//	spi_nor_wait_until_ready(dev);
	ret = qspi_cmd_xfer(driver_data->qspi, &driver_data->qspi_cfg, NULL, 0, NULL ,0 , ((write_protect) ?
		      QSPI_NOR_CMD_WRDI : QSPI_NOR_CMD_WREN), 0);

#if DT_INST_0_JEDEC_QSPI_NOR_JEDEC_ID_0 == 0xbf && DT_INST_0_JEDEC_QSPI_NOR_JEDEC_ID_1 == 0x26
//	if (ret == 0 && !write_protect) {
//		ret = qspi_nor_cmd_write(dev, SPI_NOR_CMD_MCHP_UNLOCK);
//	}
#endif

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

	data->qspi = device_get_binding(DT_NORDIC_NRF_QSPI_QSPI_0_LABEL);
	if (!data->qspi) {
		return -EINVAL;
	}

	data->qspi_cfg.frequency = 8000000;
	data->qspi_cfg.operation = (	QSPI_CS_DELAY_SET(8) 								|
									QSPI_DATA_LINES_SET(QSPI_DATA_LINES_QUAD)			|
									QSPI_ADDRESS_MODE_SET(QSPI_ADDRESS_MODE_24BIT));

//#ifdef DT_INST_0_JEDEC_SPI_NOR_CS_GPIOS_CONTROLLER
//	data->cs_ctrl.gpio_dev =
//		device_get_binding(DT_INST_0_JEDEC_SPI_NOR_CS_GPIOS_CONTROLLER);
//	if (!data->cs_ctrl.gpio_dev) {
//		return -ENODEV;
//	}
//
//	data->cs_ctrl.gpio_pin = DT_INST_0_JEDEC_SPI_NOR_CS_GPIOS_PIN;
//	data->cs_ctrl.delay = CONFIG_SPI_NOR_CS_WAIT_DELAY;
//QSPI_STD_CMD_JEDEC_ID
//	data->spi_cfg.cs = &data->cs_ctrl;
//#endif /* DT_INST_0_JEDEC_SPI_NOR_CS_GPIOS_CONTROLLER */

	/* now the spi bus is configured, we can verify the flash id */
	if (qspi_nor_read_id(dev, params) != 0) {
//		return -ENODEV;
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

static const struct qspi_nor_config flash_id = {
	.id = DT_INST_0_JEDEC_QSPI_NOR_JEDEC_ID,
#ifdef DT_INST_0_JEDEC_QSPI_NOR_HAS_BE32K
	.has_be32k = true,
#endif /* DT_INST_0_JEDEC_QSPI_NOR_HAS_BE32K */
	.size = DT_INST_0_JEDEC_QSPI_NOR_SIZE / 8,
};

static struct qspi_nor_data qspi_nor_memory_data;

DEVICE_AND_API_INIT(qspi_flash_memory, DT_INST_0_JEDEC_QSPI_NOR_LABEL,
		    &qspi_nor_init, &qspi_nor_memory_data,
		    &flash_id, POST_KERNEL, CONFIG_QSPI_NOR_INIT_PRIORITY,
		    &qspi_nor_api);
