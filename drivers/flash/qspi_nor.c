/*
 * Copyright (c) 2019, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/flash.h>
#include <init.h>
#include <string.h>
#include <logging/log.h>

#include "qspi_nor.h"
#include "flash_priv.h"
#include <nrfx_qspi.h>

LOG_MODULE_REGISTER(qspi_nor, CONFIG_FLASH_LOG_LEVEL);

#define QSPI_MODE(cpol,cpha) \
	((cpol == 0)&&(cpha == 0)) == 1 ? 0 :	\
	((cpol == 1)&&(cpha == 1)) == 1 ? 1 : -1

#define IS_USED_WRITE_QUAD_MODE_CHOSEN(lines) \
	(nrf_qspi_writeoc_t)lines == NRF_QSPI_WRITEOC_PP4IO ? 1 :	\
	(nrf_qspi_writeoc_t)lines == NRF_QSPI_WRITEOC_PP4O ? 1 : -1

#define IS_USED_READ_QUAD_MODE_CHOSEN(lines) \
	(nrf_qspi_readoc_t)lines == NRF_QSPI_READOC_READ4IO ? 1 :	\
	(nrf_qspi_readoc_t)lines == NRF_QSPI_READOC_READ4O ? 1 : -1

#define DATA_LINES_WRITE(lines) \
	lines == 3 ? NRF_QSPI_WRITEOC_PP4IO :	\
	lines == 2 ? NRF_QSPI_WRITEOC_PP4O :	\
	lines == 1 ? NRF_QSPI_WRITEOC_PP2O :	\
	lines == 0 ? NRF_QSPI_WRITEOC_PP : -1

#define DATA_LINES_READ(lines) \
	lines == 4 ? NRF_QSPI_READOC_READ4IO :	\
	lines == 3 ? NRF_QSPI_READOC_READ4O :	\
	lines == 2 ? NRF_QSPI_READOC_READ2IO :	\
	lines == 1 ? NRF_QSPI_READOC_READ2O :	\
	lines == 0 ? NRF_QSPI_READOC_FASTREAD : -1

#define ADDRESS_SIZE(addr_size) \
	addr_size == 1 ? NRF_QSPI_ADDRMODE_32BIT :	\
	addr_size == 0 ? NRF_QSPI_ADDRMODE_24BIT : -1\

/**
 * @brief Gets size of the address field
 */
#define QSPI_ADDRESS_FIELD_SIZE 0x03
#define QSPI_ADDRESS_GET(_operation_) \
	(((_operation_) & QSPI_ADDRESS_FIELD_SIZE))

/**
 * @brief Test whether offset is aligned.
 */
#define QSPI_IS_PAGE_ALIGNED(_ofs) (((_ofs) & (QSPI_PAGE_SIZE - 1U)) == 0)
#define QSPI_IS_SECTOR_ALIGNED(_ofs) (((_ofs) & (QSPI_SECTOR_SIZE - 1U)) == 0)
#define QSPI_IS_BLOCK_ALIGNED(_ofs) (((_ofs) & (QSPI_BLOCK_SIZE - 1U)) == 0)
#define QSPI_IS_BLOCK32_ALIGNED(_ofs) (((_ofs) & (QSPI_BLOCK32_SIZE - 1U)) == 0)

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
 * struct qspi_nor_data - Structure for defining the QSPI NOR access
 * @qspi: The QSPI device
 * @qspi_cfg: The QSPI configuration
 * @sem: The semaphore to access to the flash
 */
struct qspi_nor_data {
	struct device *qspi;
	struct qspi_config qspi_cfg;
	struct k_sem sem;
	bool write_protection;
};

struct qspi_nrfx_data {
	struct k_sem lock;
	bool busy;
	int status;
	struct k_poll_signal internal_signal;
	struct k_poll_event internal_events;

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	u32_t pm_state;
#endif
};

/* Main config structure */
static nrfx_qspi_config_t QSPIconfig;

static inline struct qspi_nrfx_data *get_dev_data(struct device *dev)
{
	return dev->driver_data;
}

static inline void qspi_lock(struct device *dev,
			     struct k_poll_signal *signal)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);

	k_sem_take(&dev_data->lock, K_FOREVER);
}

static inline void qspi_unlock(struct device *dev)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);

	k_sem_give(&dev_data->lock);
}

static inline void qspi_wait_for_completion(struct device *dev, int status)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);

	k_poll(&dev_data->internal_events, 1, K_FOREVER);
	dev_data->internal_events.signal->signaled = 0;
	dev_data->internal_events.state = K_POLL_STATE_NOT_READY;
}

/**
 * @brief Get QSPI prescaler
 * Get supported frequency prescaler not exceeding the requested one.
 *
 * @param frequency - desired QSPI bus frequency
 * @retval NRF_SPI_PRESCALER in case of success or;
 *		   -EINVAL in case of failure
 */
static inline nrf_qspi_frequency_t get_nrf_qspi_prescaler(u32_t frequency)
{
	if (frequency < 2130000) {
		return NRF_QSPI_FREQ_32MDIV16;  /**< 2.00 MHz. */
	} else if (frequency < 2290000) {
		return NRF_QSPI_FREQ_32MDIV15;  /**< 2.13 MHz. */
	} else if (frequency < 2460000) {
		return NRF_QSPI_FREQ_32MDIV14;  /**< 2.29 MHz. */
	} else if (frequency < 2660000) {
		return NRF_QSPI_FREQ_32MDIV13;  /**< 2.46 MHz. */
	} else if (frequency < 2900000) {
		return NRF_QSPI_FREQ_32MDIV12;  /**< 2.66 MHz. */
	} else if (frequency < 3200000) {
		return NRF_QSPI_FREQ_32MDIV11;  /**< 2.90 MHz. */
	} else if (frequency < 3550000) {
		return NRF_QSPI_FREQ_32MDIV10;  /**< 3.20 MHz. */
	} else if (frequency < 4000000) {
		return NRF_QSPI_FREQ_32MDIV9;   /**< 3.55 MHz. */
	} else if (frequency < 4570000) {
		return NRF_QSPI_FREQ_32MDIV8;   /**< 4.00 MHz. */
	} else if (frequency < 5330000) {
		return NRF_QSPI_FREQ_32MDIV7;   /**< 4.57 MHz. */
	} else if (frequency < 6400000) {
		return NRF_QSPI_FREQ_32MDIV6;   /**< 5.33 MHz. */
	} else if (frequency < 8000000) {
		return NRF_QSPI_FREQ_32MDIV5;   /**< 6.40 MHz. */
	} else if (frequency < 10600000) {
		return NRF_QSPI_FREQ_32MDIV4;   /**< 8.00 MHz. */
	} else if (frequency < 16000000) {
		return NRF_QSPI_FREQ_32MDIV3;   /**< 10.6 MHz. */
	} else if (frequency < 32000000) {
		return NRF_QSPI_FREQ_32MDIV2;   /**< 16.0 MHz. */
	} else {
		return NRF_QSPI_FREQ_32MDIV1;   /**< 32.0 MHz. */
	}
}

/**
 * @brief QSPI handler
 *
 * @param event - driver event type
 * @param p_context - Pointer to context. Use in interrupt handler.
 * @retval None
 */
static void qspi_handler(nrfx_qspi_evt_t event, void *p_context)
{
	struct device *dev = p_context;
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);

	if (event == NRFX_QSPI_EVENT_DONE) {
		k_poll_signal_raise(&dev_data->internal_signal, 0x1);
	}
}

/* QSPI custom command - common function for async and pooling */
int qspi_send_cmd_internal(struct device *dev, const struct qspi_cmd *cmd)
{

	nrf_qspi_cinstr_conf_t cinstr_cfg = {
		.opcode = cmd->op_code,
		.io2_level = true,
		.io3_level = true,
		.wipwait = true,
		.wren = true
	};

	if (cmd->tx_buf && cmd->rx_buf) {
		cinstr_cfg.length = sizeof(cmd->op_code) + cmd->tx_buf->len
				    + cmd->rx_buf->len;
	} else if (cmd->tx_buf && !(cmd->rx_buf)) {
		cinstr_cfg.length = sizeof(cmd->op_code) + cmd->tx_buf->len;
	} else if (!(cmd->tx_buf) && (cmd->rx_buf)) {
		cinstr_cfg.length = sizeof(cmd->op_code) + cmd->rx_buf->len;
	} else {
		cinstr_cfg.length = sizeof(cmd->op_code);
	}

	int rescode = nrfx_qspi_cinstr_xfer(&cinstr_cfg, cmd->tx_buf->buf,
					    cmd->rx_buf->buf);

	qspi_unlock(dev);
	switch (rescode) {
	case NRFX_SUCCESS:
		return 0;
	case NRFX_ERROR_TIMEOUT:
		return -EBUSY;
	case NRFX_ERROR_BUSY:
		return -EBUSY;
	default:
		return -EBUSY;
	}
}

/* QSPI erase - common function for async and pooling */
int qspi_erase_internal(struct device *dev, u32_t addr, u32_t size)
{
	/* Check input parameters */
	if (!dev) {
		return -ENXIO;
	}
	if (!size) {
		return -EINVAL;
	}

	int rescode = 0;

	while (size) {
		if (size == 0xFFFFFFFF) {
			/* chip erase */
			rescode = nrfx_qspi_chip_erase();
			size -= 0xFFFFFFFF;
		} else if ((size >= QSPI_BLOCK_SIZE) &&
			   QSPI_IS_BLOCK_ALIGNED(addr)) {
			/* 64 kB block erase */
			rescode = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, addr);
			addr += QSPI_BLOCK_SIZE;
			size -= QSPI_BLOCK_SIZE;
		} else if ((size >= QSPI_SECTOR_SIZE) &&
			   QSPI_IS_SECTOR_ALIGNED(addr)) {
			/* 4kB sector erase */
			rescode = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, addr);
			addr += QSPI_SECTOR_SIZE;
			size -= QSPI_SECTOR_SIZE;
		} else {
			/* minimal erase size is at least a sector size */
			qspi_unlock(dev);
			LOG_DBG("unsupported at 0x%lx size %zu", (long)addr, size);
			return -EINVAL;
		}
		qspi_wait_for_completion(dev, rescode);
	}

	qspi_unlock(dev);
	switch (rescode) {
	case NRFX_SUCCESS:
		return 0;
	case NRFX_ERROR_BUSY:
		return -EBUSY;
	default:
		return -EBUSY;
	}
}

/* QSPI write in pooling mode */
int qspi_nrfx_write(struct device *dev, const struct qspi_buf *tx_buf,
		    u32_t address)
{
	/* Check input parameters */
	if (!dev) {
		return -ENXIO;
	}
	if (!tx_buf) {
		return -EINVAL;
	}
	/* write size must be multiple of 4 bytes */
	if (tx_buf->len % sizeof(uint32_t) ||
	    !(tx_buf->len > 0)) {
		return -EINVAL;
	}

	qspi_lock(dev, NULL);
	int rescode = nrfx_qspi_write(tx_buf->buf, tx_buf->len, address);

	qspi_wait_for_completion(dev, rescode);
	qspi_unlock(dev);
	switch (rescode) {
	case NRFX_SUCCESS:
		return 0;
	case NRFX_ERROR_INVALID_ADDR:
		return -EINVAL;
	case NRFX_ERROR_BUSY:
		return -EBUSY;
	default:
		return -EBUSY;
	}
}

/* QSPI read in pooling mode */
int qspi_nrfx_read(struct device *dev, const struct qspi_buf *rx_buf,
		   u32_t address)
{
	/* Check input parameters */
	if (!dev) {
		return -ENXIO;
	}
	if (!rx_buf) {
		return -EINVAL;
	}
	/* read size must be multiple of 4 bytes */
	if (rx_buf->len % sizeof(uint32_t) ||
	    !(rx_buf->len > 0)) {
		return -EINVAL;
	}

	qspi_lock(dev, NULL);
	int rescode = nrfx_qspi_read(rx_buf->buf, rx_buf->len, address);

	qspi_wait_for_completion(dev, rescode);
	qspi_unlock(dev);
	switch (rescode) {
	case NRFX_SUCCESS:
		return 0;
	case NRFX_ERROR_INVALID_ADDR:
		return -EINVAL;
	case NRFX_ERROR_BUSY:
		return -EBUSY;
	default:
		return -EBUSY;
	}
}

/* QSPI send custom command */
int qspi_nrfx_send_cmd(struct device *dev, const struct qspi_cmd *cmd)
{
	/* Check input parameters */
	if (!dev) {
		return -ENXIO;
	}
	if (!cmd) {
		return -EINVAL;
	}

	qspi_lock(dev, NULL);
	return qspi_send_cmd_internal(dev, cmd);
}

/* QSPI erase */
int qspi_nrfx_erase(struct device *dev, u32_t addr, u32_t size)
{
	/* Check input parameters */
	if (!dev) {
		return -ENXIO;
	}
	if (!size) {
		return -EINVAL;
	}

	qspi_lock(dev, NULL);
	return qspi_erase_internal(dev, addr, size);
}


/**
 * @brief Fills init struct
 *
 * @param config		- Pointer to the config struct provided by user
 * @param initStruct	- Pointer to the configuration struct
 * @retval None
 */
static inline void qspi_fill_init_struct(const struct qspi_config *config,
					 nrfx_qspi_config_t *initStruct)
{
	/* Configure XIP offset */
	initStruct->xip_offset = 0;

	/* Configure pins */
	initStruct->pins.sck_pin = DT_NORDIC_NRF_QSPI_QSPI_0_SCK_PIN;
	initStruct->pins.csn_pin = config->cs_pin;
	initStruct->pins.io0_pin = DT_NORDIC_NRF_QSPI_QSPI_0_IO_PINS_0;
	initStruct->pins.io1_pin = DT_NORDIC_NRF_QSPI_QSPI_0_IO_PINS_1;
	initStruct->pins.io2_pin = DT_NORDIC_NRF_QSPI_QSPI_0_IO_PINS_2;
	initStruct->pins.io3_pin = DT_NORDIC_NRF_QSPI_QSPI_0_IO_PINS_3;

	/* Configure IRQ priority */
	initStruct->irq_priority = (uint8_t)NRFX_QSPI_DEFAULT_CONFIG_IRQ_PRIORITY;

	/* Configure Protocol interface */
#ifdef DT_INST_0_NORDIC_QSPI_NOR_READOC_ENUM
	initStruct->prot_if.readoc = (nrf_qspi_writeoc_t) DATA_LINES_READ(DT_INST_0_NORDIC_QSPI_NOR_READOC_ENUM);
#else
	initStruct->prot_if.readoc = NRF_QSPI_READOC_FASTREAD;
#endif

#ifdef DT_INST_0_NORDIC_QSPI_NOR_WRITEOC_ENUM
	initStruct->prot_if.writeoc = (nrf_qspi_writeoc_t) DATA_LINES_WRITE(DT_INST_0_NORDIC_QSPI_NOR_WRITEOC_ENUM);
#else
	initStruct->prot_if.writeoc = NRF_QSPI_WRITEOC_PP;
#endif
	initStruct->prot_if.addrmode =
		(nrf_qspi_addrmode_t)config->address;
	initStruct->prot_if.dpmconfig = false;

	/* Configure physical interface */
	initStruct->phy_if.sck_freq = (nrf_qspi_frequency_t) get_nrf_qspi_prescaler(
		config->frequency);
	initStruct->phy_if.sck_delay = (uint8_t) config->cs_high_time;
	initStruct->phy_if.spi_mode = (nrf_qspi_spi_mode_t) config->mode;
	initStruct->phy_if.dpmen = false;
}

/* Configures QSPI memory for the transfer */
int qspi_nrfx_configure(struct device *dev, const struct qspi_config *config)
{
	if (!dev) {
		return -ENXIO;
	}
	if (!config) {
		return -EINVAL;
	}

	static u8_t isInitialised;

	qspi_fill_init_struct(config, &QSPIconfig);
	if (isInitialised != false) {
		nrfx_qspi_uninit();
	}

	int rescode = nrfx_qspi_init(&QSPIconfig, qspi_handler, dev);

	/* If quad transfer was choosen - enable it now */
	if(( IS_USED_WRITE_QUAD_MODE_CHOSEN(QSPIconfig.prot_if.writeoc) == 1) ||
		(IS_USED_READ_QUAD_MODE_CHOSEN(QSPIconfig.prot_if.readoc) == 1)){
		u8_t tx[1] = {0};
		tx[0] |= QSPI_NOR_QE_BIT;

		struct qspi_buf tx_buff = {
				.buf = tx,
				.len = sizeof(tx),
		};

		struct qspi_cmd cmd ={
			.op_code = QSPI_NOR_CMD_WRSR,
			.tx_buf = &tx_buff,
			.rx_buf = NULL,
		};
		qspi_nrfx_send_cmd(dev, &cmd);
	}

	isInitialised = true;
	switch (rescode) {
	case NRFX_SUCCESS:
		return 0;
	case NRFX_ERROR_TIMEOUT:
		return -ETIMEDOUT;
	case NRFX_ERROR_INVALID_STATE:
		return -ECANCELED;
	case NRFX_ERROR_INVALID_PARAM:
		return -EINVAL;
	default:
		return -EBUSY;
	}
}

static int qspi_set_active_mem(struct device *dev)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	static uint16_t current_mem = 0xFFFF;
	int ret = 0;
	/* We are using CS pin number to identify which memory is selected,
	 * as it is easy to compare and unique for every memory attached.
	 */
	if (current_mem != driver_data->qspi_cfg.cs_pin) {
		ret = qspi_nrfx_configure(driver_data->qspi, &driver_data->qspi_cfg);
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

	if(qspi_nrfx_send_cmd(driver_data->qspi, &cmd) != 0){
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

	ret = qspi_nrfx_read(driver_data->qspi, &rx_buf, addr);

	SYNC_UNLOCK();

	return ret;
}

static int qspi_nor_write(struct device *dev, off_t addr, const void *src,
			 size_t size)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	const struct qspi_nor_config *params = dev->config->config_info;

	if (driver_data->write_protection) {
		return -EACCES;
	}

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

	ret = qspi_nrfx_write(driver_data->qspi, &tx_buf, addr);

	SYNC_UNLOCK();

	return ret;
}

static int qspi_nor_erase(struct device *dev, off_t addr, size_t size)
{
	struct qspi_nor_data *const driver_data = dev->driver_data;
	const struct qspi_nor_config *params = dev->config->config_info;

	if (driver_data->write_protection) {
		return -EACCES;
	}

	int ret = qspi_set_active_mem(dev);
	if (ret != 0) {
		return ret;
	}
	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > params->size)) {
		return -ENODEV;
	}

	SYNC_LOCK();


	ret = qspi_nrfx_erase(driver_data->qspi, addr, size);

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

	driver_data->write_protection = write_protect;

	SYNC_LOCK();

	if(qspi_nrfx_send_cmd(driver_data->qspi, &cmd) != 0){
		ret = -EIO;
	}

	if (params->id[0] == 0xbf && params->id[1] == 0x26 && ret == 0 && !write_protect) {
		cmd.op_code = QSPI_NOR_CMD_MCHP_UNLOCK;
		ret = qspi_nrfx_send_cmd(driver_data->qspi, &cmd);
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

	data->qspi = device_get_binding(DT_INST_0_NORDIC_QSPI_NOR_BUS_NAME);
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
#define INST_0_BYTES (DT_INST_0_NORDIC_QSPI_NOR_SIZE / 8)

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
	.id = DT_INST_0_NORDIC_QSPI_NOR_JEDEC_ID,
	.has_be32k = DT_INST_0_NORDIC_QSPI_NOR_HAS_BE32K,
	.size = DT_INST_0_NORDIC_QSPI_NOR_SIZE / 8,
};

static struct qspi_nor_data qspi_nor_memory_data = {
	.qspi_cfg = {
		.cs_pin = DT_NORDIC_NRF_QSPI_QSPI_0_CSN_PINS_0,
		.frequency = DT_INST_0_NORDIC_QSPI_NOR_SCK_FREQUENCY,
		.mode = QSPI_MODE(DT_INST_0_NORDIC_QSPI_NOR_CPOL, DT_INST_0_NORDIC_QSPI_NOR_CPHA),
		.cs_high_time = DT_INST_0_NORDIC_QSPI_NOR_SCK_DELAY,
		.address = ADDRESS_SIZE(DT_INST_0_NORDIC_QSPI_NOR_ADDRESS_SIZE_32)
	}
};

DEVICE_AND_API_INIT(qspi_flash_memory, DT_INST_0_NORDIC_QSPI_NOR_LABEL,
		    &qspi_nor_init, &qspi_nor_memory_data,
		    &flash_id, POST_KERNEL, CONFIG_QSPI_NOR_INIT_PRIORITY,
		    &qspi_nor_api);

/* API definition */
static const struct qspi_driver_api qspi_nrfx_driver_api = {
	.configure = qspi_nrfx_configure,
	.write = qspi_nrfx_write,
	.read = qspi_nrfx_read,
	.send_cmd = qspi_nrfx_send_cmd,
	.erase = qspi_nrfx_erase,
};

#define QSPI_NRFX_QSPI_DEVICE(void)							 \
	static int qspi_init(struct device *dev)					 \
	{										 \
		IRQ_CONNECT(DT_NORDIC_NRF_QSPI_QSPI_0_IRQ_0,				 \
			    DT_NORDIC_NRF_QSPI_QSPI_0_IRQ_0_PRIORITY,			 \
			    nrfx_isr, nrfx_qspi_irq_handler, 0);			 \
		return 0;								 \
	}										 \
	static struct qspi_nrfx_data qspi_data = {					 \
		.lock = Z_SEM_INITIALIZER(qspi_data.lock, 1, UINT_MAX),			 \
		.internal_signal = K_POLL_SIGNAL_INITIALIZER(qspi_data.internal_signal), \
		.internal_events = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,		 \
							    K_POLL_MODE_NOTIFY_ONLY,	 \
							    &qspi_data.internal_signal), \
		.busy = false,								 \
	};										 \
	DEVICE_DEFINE(qspi, DT_NORDIC_NRF_QSPI_QSPI_0_LABEL,				 \
		      qspi_init,							 \
		      qspi_nrfx_pm_control,						 \
		      &qspi_data,							 \
		      NULL,								 \
		      POST_KERNEL, CONFIG_QSPI_INIT_PRIORITY,				 \
		      &qspi_nrfx_driver_api)

#ifdef CONFIG_QSPI_NRF_QSPI
QSPI_NRFX_QSPI_DEVICE();
#endif
