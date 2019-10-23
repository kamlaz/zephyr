/*
 * Copyright (c) 2019, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Includes ------------------------------------------------------------------*/
#include <sys/__assert.h>
#include <drivers/qspi.h>
#include <nrfx_qspi.h>
#include <stdio.h>

#define LOG_DOMAIN "qspi_nrfx_qspi"
#define LOG_LEVEL CONFIG_QSPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(qspi_nrfx_qspi);
/* Private define ------------------------------------------------------------*/
/* Page, sector, and block size are standard, not configurable. */

#define QSPI_DATA_LINES_FIELD_SIZE	0x03
#define QSPI_DATA_LINES_GET(_operation_)				\
	(((_operation_) & QSPI_DATA_LINES_FIELD_SIZE))

#define QSPI_ADDRESS_FIELD_SIZE	0x03
#define QSPI_ADDRESS_GET(_operation_)					\
	(((_operation_) & QSPI_ADDRESS_FIELD_SIZE))

/**
 * @brief QSPI Polarity & Phase Modes
 */

#define QSPI_NOR_PAGE_SIZE    			0x0100U
#define QSPI_NOR_SECTOR_SIZE  			0x1000U
#define QSPI_NOR_BLOCK_SIZE   			0x10000U
#define SPI_NOR_BLOCK32_SIZE			0x8000

/* Test whether offset is aligned. */
#define QSPI_NOR_IS_PAGE_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_PAGE_SIZE - 1U)) == 0)
#define QSPI_NOR_IS_SECTOR_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_SECTOR_SIZE - 1U)) == 0)
#define QSPI_NOR_IS_BLOCK_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_BLOCK_SIZE - 1U)) == 0)
#define QSPI_NOR_IS_BLOCK32_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_BLOCK32_SIZE - 1U)) == 0)
/* Imported variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Structure with basic structure */
struct qspi_nrfx_data {
	struct k_sem xfer;
	u8_t   busy;
	int status;
#ifdef CONFIG_QSPI_ASYNC
	struct k_poll_signal *signal;
	//bool asynchronous;
#endif /* CONFIG_QSPI_ASYNC */

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	u32_t pm_state;
#endif
};

/* Main config structure */
struct qspi_nrfx_config {
	nrfx_qspi_config_t config;
};
static const struct qspi_nrfx_config qspi_conf;
static nrfx_qspi_config_t QSPIconfig;

/* Exported types ------------------------------------------------------------*/
int qspi_nrfx_write(struct device *dev, const struct qspi_buf *tx_buf, u32_t address);
int qspi_nrfx_write_async(struct device *dev, const struct qspi_buf *tx_buf, u32_t address,struct k_poll_signal *async);

int qspi_nrfx_write_(struct device *dev, const struct qspi_buf *tx_buf, u32_t address);
int qspi_nrfx_read(struct device *dev, const struct qspi_buf *rx_buf, u32_t address);
int qspi_nrfx_send_cmd(struct device *dev, const struct qspi_cmd *cmd, const struct qspi_buf *rx_buf);
int qspi_nrfx_erase(struct device *dev, u32_t addr, u32_t size);
int qspi_nrfx_configure(struct device *dev, const struct qspi_config *config);

#ifdef CONFIG_QSPI_ASYNC
int qspi_nrfx_write_async(struct device *dev, const struct qspi_buf *tx_buf, u32_t address, struct k_poll_signal *async);
int qspi_nrfx_read_async(struct device *dev, const struct qspi_buf *rx_buf, u32_t address, struct k_poll_signal *async);
int qspi_nrfx_send_cmd_async(struct device *dev, const struct qspi_cmd *cmd, const struct qspi_buf *rx_buf, struct k_poll_signal *async);
int qspi_nrfx_erase_async(struct device *dev, u32_t addr, u32_t size, struct k_poll_signal *async);
#endif /* CONFIG_QSPI_ASYNC */

static inline void qspi_fill_init_struct(const struct qspi_config * config,nrfx_qspi_config_t * initStruct);
static inline int get_nrf_qspi_readoc(u8_t data_lines);
static inline int get_nrf_qspi_wrieoc(u8_t data_lines);
static inline int get_nrf_qspi_address_mode(u8_t address);
static inline nrf_qspi_frequency_t get_nrf_qspi_prescaler(u32_t frequency);

/* API definition */
static const struct qspi_driver_api qspi_nrfx_driver_api = {
	.write = qspi_nrfx_write,
	.read = qspi_nrfx_read,
	.send_cmd = qspi_nrfx_send_cmd,
	.erase = qspi_nrfx_erase,
	.configure = qspi_nrfx_configure,
#ifdef CONFIG_QSPI_ASYNC
	qspi_api_write_async 	qspi_nrfx_write_async;
	qspi_api_read_async 	qspi_nrfx_read_async;
	qspi_api_send_cmd_async	qspi_nrfx_send_cmd_async;
	qspi_api_erase_async 	qspi_nrfx_erase_async;
#endif /* CONFIG_QSPI_ASYNC */
};

static inline struct qspi_nrfx_data *get_dev_data(struct device *dev)
{
	return dev->driver_data;
}

static inline void qspi_lock(struct device *dev,
				    struct k_poll_signal *signal)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
	k_sem_take(&dev_data->xfer, K_FOREVER);

#ifdef CONFIG_QSPI_ASYNC
	dev_data->signal = signal;
#endif /* CONFIG_QSPI_ASYNC */
}

static inline void qspi_unlock(struct device *dev)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
#ifdef CONFIG_QSPI_ASYNC
	k_poll_signal_raise(dev_data->signal, dev_data->status);
#endif /* CONFIG_QSPI_ASYNC */
	k_sem_give(&dev_data->xfer);
}

/**
 * @brief Get QSPI read operation code
 * Amount of lines used for transfer
 *
 * @param operation - QSPI config field
 * @retval NRF_QSPI_READOC in case of success or
 * 		   -EINVAL in case of failure
 */
static inline int get_nrf_qspi_readoc(u8_t data_lines)
{
	if (QSPI_DATA_LINES_GET(data_lines) == QSPI_DATA_LINES_SINGLE) {			/**< SINGLE line operation. QSPI_DATA_LINES_SINGLE*/
		return NRF_QSPI_READOC_FASTREAD;
	}
	else if (QSPI_DATA_LINES_GET(data_lines) == QSPI_DATA_LINES_DOUBLE){		/**< DOUBLE line operation. QSPI_DATA_LINES_DOUBLE*/
		return NRF_QSPI_READOC_READ2IO;
	}
	else if (QSPI_DATA_LINES_GET(data_lines) == QSPI_DATA_LINES_QUAD){		/**< QUAD line operation. QSPI_DATA_LINES_QUAD*/
		return NRF_QSPI_READOC_READ4IO;
	}
	else{
		LOG_ERR("Not supported line type");
		return -EINVAL;
	}
}

/**
 * @brief Get QSPI read operation code
 * Amount of lines used for transfer
 *
 * @param operation - QSPI config field
 * @retval NRF_QSPI_READOC in case of success or
 * 		   -EINVAL in case of failure
 */
static inline int get_nrf_qspi_wrieoc(u8_t data_lines)
{
	if (QSPI_DATA_LINES_GET(data_lines) == QSPI_DATA_LINES_SINGLE) {			/**< SINGLE line operation. */
		return NRF_QSPI_WRITEOC_PP;
	}
	else if (QSPI_DATA_LINES_GET(data_lines) == QSPI_DATA_LINES_DOUBLE){		/**< DOUBLE line operation. */
		return NRF_QSPI_WRITEOC_PP2O;
	}
	else if (QSPI_DATA_LINES_GET(data_lines) == QSPI_DATA_LINES_QUAD){		/**< QUAD line operation. */
		return NRF_QSPI_WRITEOC_PP4IO;
	}
	else{
		LOG_ERR("Not supported line type");
		return -EINVAL;
	}
}

/**
 * @brief Get QSPI address mode
 * Two types of address are avaiable: 24bit or 32bit
 *
 * @param operation - QSPI config field
 * @retval NRF_SPI_ADDRMODE in case of success or
 * 		   -EINVAL in case of failure
 */
static inline int get_nrf_qspi_address_mode(u8_t address)
{
	if (QSPI_ADDRESS_GET(address) == QSPI_ADDRESS_MODE_24BIT) {		/**< 24 BIT ADDR. */
		return NRF_QSPI_ADDRMODE_24BIT;
	}
	else if (QSPI_ADDRESS_GET(address) == QSPI_ADDRESS_MODE_32BIT){	/**< 32 BIT ADDR. */
		return NRF_QSPI_ADDRMODE_32BIT;
	}
	else{
		LOG_ERR("Address modes other than 24 or 32 bits are not supported");
		return -EINVAL;
	}
}

/**
 * @brief Get QSPI prescaler
 * Function returns prescaler for required frequency
 *
 * @param frequency - QSPI config fieldess Memory address to
 * @retval NRF_SPI_PRESCALER in case of success or
 * 		   -EINVAL in case of failure
 */
static inline nrf_qspi_frequency_t get_nrf_qspi_prescaler(u32_t frequency)
{
	/* Get the highest supported frequency prescaler not exceeding the requested one. */

	if (frequency < 2130000) {
		return NRF_QSPI_FREQ_32MDIV16;	/**< 2.00 MHz. */
	} else if (frequency < 2290000) {
		return NRF_QSPI_FREQ_32MDIV15;	/**< 2.13 MHz. */
	} else if (frequency < 2460000) {
		return NRF_QSPI_FREQ_32MDIV14;	/**< 2.29 MHz. */
	} else if (frequency < 2660000) {
		return NRF_QSPI_FREQ_32MDIV13;	/**< 2.46 MHz. */
	} else if (frequency < 2900000) {
		return NRF_QSPI_FREQ_32MDIV12;	/**< 2.66 MHz. */
	} else if (frequency < 3200000) {
		return NRF_QSPI_FREQ_32MDIV11;	/**< 2.90 MHz. */
	} else if (frequency < 3550000) {
		return NRF_QSPI_FREQ_32MDIV10;	/**< 3.20 MHz. */
	} else if (frequency < 4000000) {
		return NRF_QSPI_FREQ_32MDIV9;	/**< 3.55 MHz. */
	} else if (frequency < 4570000) {
		return NRF_QSPI_FREQ_32MDIV8;	/**< 4.00 MHz. */
	} else if (frequency < 5330000) {
		return NRF_QSPI_FREQ_32MDIV7;	/**< 4.57 MHz. */
	} else if (frequency < 6400000) {
		return NRF_QSPI_FREQ_32MDIV6;	/**< 5.33 MHz. */
	} else if (frequency < 8000000) {
		return NRF_QSPI_FREQ_32MDIV5;	/**< 6.40 MHz. */
	} else if (frequency < 10600000) {
		return NRF_QSPI_FREQ_32MDIV4;	/**< 8.00 MHz. */
	} else if (frequency < 16000000) {
		return NRF_QSPI_FREQ_32MDIV3;	/**< 10.6 MHz. */
	} else if (frequency < 32000000) {
		return NRF_QSPI_FREQ_32MDIV2;	/**< 16.0 MHz. */
	} else {
		return NRF_QSPI_FREQ_32MDIV1;	/**<qspi_data 32.0 MHz. */
	}
}

/**
 * @brief QSPI handler
 * Amount of lines used for transfer
 *
 * @param event 	- TBD
 * @param p_context - TBD
 * @retval None
 */
static void qspi_handler(nrfx_qspi_evt_t event, void * p_context)
{
	struct device *dev = p_context;
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);

	if (event == NRFX_QSPI_EVENT_DONE) {
	/* Add functionality */
	qspi_unlock(dev);

	}
}

int qspi_nrfx_write(struct device *dev, const struct qspi_buf *tx_buf, u32_t address){
	/* Check input parameters */
	if (!dev) 							{ return -ENXIO;}
	if (!tx_buf)						{ return -EINVAL;}
	if (tx_buf->len % sizeof(uint32_t))	{ return -EINVAL;}	//write size must be multiple of 4 bytes

	qspi_lock(dev, NULL);
	return nrfx_qspi_write(tx_buf->buf, tx_buf->len, address);
}


int qspi_nrfx_read(struct device *dev, const struct qspi_buf *rx_buf, u32_t address){
	/* Check input parameters */
	if (!dev) 							{ return -ENXIO;}
	if (!rx_buf)						{ return -EINVAL;}
	if (rx_buf->len % sizeof(uint32_t))	{ return -EINVAL;}	//read size must be multiple of 4 bytes

	qspi_lock(dev, NULL);
	return nrfx_qspi_read(rx_buf->buf, rx_buf->len, address);
}

int qspi_nrfx_send_cmd(struct device *dev, const struct qspi_cmd *cmd, const struct qspi_buf *rx_buf){
	/* Check input parameters */
	if (!dev) 		{ return -ENXIO;}
	if (!cmd)		{ return -EINVAL;}
	qspi_lock(dev, NULL);

	uint8_t tmp_buff[9] = {0};int qspi_nrfx_write(struct device *dev, const struct qspi_buf *tx_buf, u32_t address){
		/* Check input parameters */
		if (!dev) 							{ return -ENXIO;}
		if (!tx_buf)						{ return -EINVAL;}
		if (tx_buf->len % sizeof(uint32_t))	{ return -EINVAL;}	//write size must be multiple of 4 bytes

		qspi_lock(dev, NULL);
		return nrfx_qspi_write(tx_buf->buf, tx_buf->len, address);
	}

	uint8_t cmd_size = 0;

	/* Fill address field */
	if(cmd->addr != 0xFFFFFFFF){
		/* Copy address field to the buffer */
		if(QSPIconfig.prot_if.addrmode == NRF_QSPI_ADDRMODE_24BIT){
			//Copy 24 bit address
			tmp_buff[0] = (uint8_t)(cmd->addr >> 16);
			tmp_buff[1] = (uint8_t)(cmd->addr >> 8);
			tmp_buff[2] = (uint8_t)(cmd->addr);
			cmd_size += 3;
		}
		else{
			//Copy 32 bit address
			tmp_buff[0] = (uint8_t)(cmd->addr >> 24);
			tmp_buff[1] = (uint8_t)(cmd->addr >> 16);
			tmp_buff[2] = (uint8_t)(cmd->addr >> 8);
			tmp_buff[3] = (uint8_t)(cmd->addr);
			cmd_size += sizeof(uint32_t);
		}
	}

	if(cmd->tx_buf != NULL && rx_buf != NULL){
		/* If there is a msg - copy it to the buffer */
		if(cmd->tx_buf->len > 0){
			memcpy(&tmp_buff[cmd_size],cmd->tx_buf->buf,cmd->tx_buf->len);
		}
		cmd_size +=(cmd->tx_buf->len + rx_buf->len);
	}
	else if(cmd->tx_buf != NULL && rx_buf == NULL){
		if(cmd->tx_buf->len > 0){
			memcpy(&tmp_buff[cmd_size],cmd->tx_buf->buf,cmd->tx_buf->len);
		}
		cmd_size +=(cmd->tx_buf->len);
	}
	else if(cmd->tx_buf == NULL && rx_buf != NULL){
		cmd_size +=(rx_buf->len);
	}


	nrf_qspi_cinstr_conf_t cinstr_cfg = {
		.opcode    = cmd->op_code,
		.length    = cmd_size +  sizeof(cmd->op_code),
		.io2_level = true,
		.io3_level = true,
		.wipwait   = true,
		.wren      = true
	};

	/* We assigned &tx_buf->buf[1], becouse second element in the array is first byte of our tx buffer */
	return nrfx_qspi_cinstr_xfer(&cinstr_cfg,tmp_buff, rx_buf->buf);
}

int qspi_nrfx_erase(struct device *dev, u32_t addr, u32_t size){
	/* Check input parameters */
	if (!dev) 		{ return -ENXIO;}

	k_sem_take(&(get_dev_data(dev)->xfer), K_FOREVER);
	int result = 0;

	while (size) {
		if (size == 0xFFFFFFFF) {		// TODO: investigate
//			/* chip erase */
			result = nrfx_qspi_chip_erase();
			size -= 0xFFFFFFFF;
		} else if ((size >= QSPI_NOR_BLOCK_SIZE)
				&& QSPI_NOR_IS_BLOCK_ALIGNED(addr)) {
			/* 64 KiB block erase */
			result = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,addr);

			addr += QSPI_NOR_BLOCK_SIZE;
			size -= QSPI_NOR_BLOCK_SIZE;
//		} else if ((size >= QSPI_NOR_BLOCK32_SIZE)
//				&& QSPI_NOR_IS_BLOCK32_ALIGNED(addr)) {
//			/* 32 KiB block erase */
//			// TODO: ADD FUCTION HERE
//			addr += QSPI_NOR_BLOCK32_SIZE;
//			size -= QSPI_NOR_BLOCK32_SIZE;
		} else if ((size >= QSPI_NOR_SECTOR_SIZE)
				&& QSPI_NOR_IS_SECTOR_ALIGNED(addr)) {
			/* sector erase */
			result = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB,addr);
			addr += QSPI_NOR_SECTOR_SIZE;
			size -= QSPI_NOR_SECTOR_SIZE;
		} else {
			/* minimal erase size is at least a sector size */
//			SYNC_UNLOCK();
			LOG_DBG("unsupported at 0x%lx size %zu", (long)addr,
					size);
			return -EINVAL;
		}
	}
	return result;
}


int qspi_nrfx_write_async(struct device *dev, const struct qspi_buf *tx_buf, u32_t address, struct k_poll_signal *async){
	/* Check input parameters */
	if (!dev) 							{ return -ENXIO;}
	if (!tx_buf)						{ return -EINVAL;}
	if (tx_buf->len % sizeof(uint32_t))	{ return -EINVAL;}	//write size must be multiple of 4 bytes

	qspi_lock(dev, async);
	return nrfx_qspi_write(tx_buf->buf, tx_buf->len, address);
}

int qspi_nrfx_read_async(struct device *dev, const struct qspi_buf *rx_buf, u32_t address, struct k_poll_signal *async){
	/* Check input parameters */
	if (!dev) 							{ return -ENXIO;}
	if (!rx_buf)						{ return -EINVAL;}
	if (rx_buf->len % sizeof(uint32_t))	{ return -EINVAL;}	//read size must be multiple of 4 bytes

	qspi_lock(dev, async);
	return nrfx_qspi_read(rx_buf->buf, rx_buf->len, address);
}

/* Configures QSPI memory for the transfer */
int qspi_nrfx_configure(struct device *dev, const struct qspi_config *config){
	if (!dev) 							{ return -ENXIO;}
	if (!config)						{ return -EINVAL;}

	int rescode = 0;
	qspi_fill_init_struct(config, &QSPIconfig);
	rescode = nrfx_qspi_init(&QSPIconfig, qspi_handler, dev);
	if (rescode != 0) {
		if (rescode == NRFX_ERROR_TIMEOUT) {
			return -ETIMEDOUT;
		} else if (rescode == NRFX_ERROR_INVALID_STATE) {
			return ECANCELED;
		} else {
			return -EINVAL;
		}
	}
	return 0;
}




/**
 * @brief Fills inits struct
 * Amount of lines used for transfer
 *
 * @param config 		- Config struct
 * @param configStruct	-
 * @retval None
 */
static inline void qspi_fill_init_struct(const struct qspi_config * config,nrfx_qspi_config_t * initStruct){
/* Configure XIP offset */
	initStruct->xip_offset  = NRFX_QSPI_CONFIG_XIP_OFFSET;

	/* Configure pins */
	initStruct->pins.sck_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_SCK_PIN;
	initStruct->pins.csn_pin  = config->cs_pin;
	initStruct->pins.io0_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO00_PIN;
	initStruct->pins.io1_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO01_PIN;
	initStruct->pins.io2_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO02_PIN;
	initStruct->pins.io3_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO03_PIN;

	/* Configure IRQ priority */
	initStruct->irq_priority   = (uint8_t)NRFX_QSPI_CONFIG_IRQ_PRIORITY;

	/* Configure Protocol interface */
	initStruct->prot_if.readoc     = (nrf_qspi_readoc_t)get_nrf_qspi_readoc(config->data_lines);
	initStruct->prot_if.writeoc    = (nrf_qspi_writeoc_t)get_nrf_qspi_wrieoc(config->data_lines);
	initStruct->prot_if.addrmode   = (nrf_qspi_addrmode_t)get_nrf_qspi_address_mode(config->address);
	initStruct->prot_if.dpmconfig  = false;

	/* COnfigure physical interface */
	initStruct->phy_if.sck_freq   = (nrf_qspi_frequency_t)get_nrf_qspi_prescaler(config->frequency);
	initStruct->phy_if.sck_delay  = (uint8_t)config->cs_high_time;
	initStruct->phy_if.spi_mode   = (nrf_qspi_spi_mode_t)config->mode;
	initStruct->phy_if.dpmen      = false;
}

/* Function for QSPI Power Management handling */
static int qspi_nrfx_pm_control(void)
{
	return 0;
}



#define QSPI_NRFX_QSPI_DEVICE(void)					       						\
	static int qspi_init(struct device *dev)			       					\
	{								       										\
		IRQ_CONNECT(DT_NORDIC_NRF_QSPI_QSPI_0_IRQ_0,		       				\
			    DT_NORDIC_NRF_QSPI_QSPI_0_IRQ_0_PRIORITY,      					\
			    nrfx_isr, nrfx_qspi_irq_handler, 0);	       					\
		return 0;																\
	}																			\
	static struct qspi_nrfx_data qspi_data = {									\
		.xfer = Z_SEM_INITIALIZER(qspi_data.xfer, 1, UINT_MAX),					\
		.busy = false,															\
	};																			\
	DEVICE_DEFINE(qspi, DT_NORDIC_NRF_QSPI_QSPI_0_LABEL,	       				\
		      qspi_init,					       								\
		      qspi_nrfx_pm_control,				       							\
		      &qspi_data,				       									\
		      NULL,				       											\
		      POST_KERNEL, CONFIG_QSPI_INIT_PRIORITY,		      	 			\
		      &qspi_nrfx_driver_api)

#ifdef CONFIG_QSPI_NRF_QSPI
QSPI_NRFX_QSPI_DEVICE();
#endif


