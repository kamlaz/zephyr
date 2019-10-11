

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
/**
 * @brief QSPI Polarity & Phase Modes
 */

/**
 * Clock Polarity: if set, clock idle state will be 1
 * and active state will be 0. If untouched, the inverse will be true
 * which is the default.
 */
#define QSPI_MODE_CPOL		BIT(0)

/**
 * Clock Phase: this dictates when is the data captured, and depends
 * clock's polarity. When QSPI_MODE_CPOL is set and this bit as well,
 * capture will occur on low to high transition and high to low if
 * this bit is not set (default). This is fully reversed if CPOL is
 * not set.
 */
#define QSPI_MODE_CPHA		BIT(1)


/**
 * No of data lines that are used for the transfer
 */
#define QSPI_DATA_LINES_SINGLE			0x00
#define QSPI_DATA_LINES_DOUBLE			0x01
#define QSPI_DATA_LINES_QUAD			0x02

/**
 * @brief QSPI Address configuration
 * Length of the address field. Typical flash chips support 24bit address mode
 * 0x00	-	8 bit address
 * 0x01	-	16 bit address
 * 0x02	-	24 bit address
 * 0x03	-	32 bit address
 */
#define QSPI_ADDRESS_MODE_8BIT			0x00
#define QSPI_ADDRESS_MODE_16BIT			0x01
#define QSPI_ADDRESS_MODE_24BIT			0x02
#define QSPI_ADDRESS_MODE_32BmeIT			0x03

#define QSPI_NOR_PAGE_SIZE    	0x0100U
#define QSPI_NOR_SECTOR_SIZE  	0x1000U
#define QSPI_NOR_BLOCK_SIZE   	0x10000U
#define SPI_NOR_BLOCK32_SIZE	0x8000

/* Test whether offset is aligned. */
#define QSPI_NOR_IS_PAGE_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_PAGE_SIZE - 1U)) == 0)
#define QSPI_NOR_IS_SECTOR_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_SECTOR_SIZE - 1U)) == 0)
#define QSPI_NOR_IS_BLOCK_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_BLOCK_SIZE - 1U)) == 0)
#define QSPI_NOR_IS_BLOCK32_ALIGNED(_ofs) (((_ofs) & (QSPI_NOR_BLOCK32_SIZE - 1U)) == 0)
/* Imported variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//struct k_sem qspi_sem;
struct qspi_buf {
	u8_t *buf;
	size_t len;
};

/* Exported types ------------------------------------------------------------*/
int qspi_nrfx_write(struct device *dev, const struct qspi_buf *tx_buf, u32_t address);
int qspi_nrfx_read(struct device *dev, const struct qspi_buf *rx_buf, u32_t address);
int qspi_nrfx_send_cmd(struct device *dev, const struct qspi_buf *tx_buf, const struct qspi_buf *rx_buf, u32_t address);
int qspi_nrfx_erase(struct device *dev, u32_t start_address, u32_t length);
int qspi_nrfx_set_act_mem(struct device *dev, const struct qspi_config *config);

/* API definition */
static const struct qspi_driver_api qspi_nrfx_driver_api = {
	.write = qspi_nrfx_write,
	.read = qspi_nrfx_read,
	.send_cmd = qspi_nrfx_send_cmd,
	.erase = qspi_nrfx_erase,
	.set_act_mem = qspi_nrfx_set_act_mem,
};


/*
 * qspi_api_write
 * 	ENXIO 		- No such device or address
 * 	EINVAL 		- invalid input parameter
 * 	EBUSY 		- device busy
 * 	ETIMEDOUT	- timeout
 */
int qspi_nrfx_write(struct device *dev, const struct qspi_buf *tx_buf, u32_t address){
	//__ASSERT(tx_buf, "Tx buffer cannot be null!");
	/* Check input parameters */
	if (!dev) 		{ return -ENXIO;}
	if (!tx_buf)	{ return -EINVAL;}

	int result = 0;
	return nrfx_qspi_write(tx_buf->buf, tx_buf->len, address);
}

int qspi_nrfx_read(struct device *dev, const struct qspi_buf *rx_buf, u32_t address){
	/* Check input parameters */
	if (!dev) 		{ return -ENXIO;}
	if (!rx_buf)	{ return -EINVAL;}

	int result = 0;
	return nrfx_qspi_read(rx_buf->buf, rx_buf->len, address);
}

int qspi_nrfx_erase(struct device *dev, u32_t start_address, u32_t length){
	/* Check input parameters */
	if (!dev) 		{ return -ENXIO;}

	int result = 0;

	while (length) {
		if (size == params->size) {		// TODO: investigate
//			/* chip erase */
			result = nrfx_qspi_chip_erase();
//			size -= params->size;
		} else if ((size >= QSPI_NOR_BLOCK_SIZE)
				&& QSPI_NOR_IS_BLOCK_ALIGNED(addr)) {
			/* 64 KiB block erase */
			nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,start_address)

			addr += QSPI_NOR_BLOCK_SIZE;
			size -= QSPI_NOR_BLOCK_SIZE;
		} else if ((size >= QSPI_NOR_BLOCK32_SIZE)
				&& QSPI_NOR_IS_BLOCK32_ALIGNED(addr)) {
			/* 32 KiB block erase */
			// TODO: ADD FUCTION HERE
			addr += QSPI_NOR_BLOCK32_SIZE;
			size -= QSPI_NOR_BLOCK32_SIZE;
		} else if ((size >= QSPI_NOR_SECTOR_SIZE)
				&& QSPI_NOR_IS_SECTOR_ALIGNED(addr)) {
			/* sector erase */
			snrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB,start_address)
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
	}
}




/*
 * qspi_api_config
 *	ENXIO No such device or address
 *	EBUSY device busy
 *	ECANCELED - operation cancelled (becoise peripherial has been initialised so far)
 *
 */

/*
 * qspi_api_write
 * 	ENXIO 		- No such device or address
 * 	EINVAL 		- invalid input parameter
 * 	EBUSY 		- device busy
 * 	ETIMEDOUT	- timeout
 */

/*
 * qspi_api_read
 * 	ENXIO 		- No such device or address
 * 	EINVAL 		- invalid input parameter
 * 	EBUSY 		- device busy
 * 	ETIMEDOUT	- timeout
 */

/*
 * qspi_api_erase
 * 	ENXIO 		- No such device
 * 	EINVAL 		- invalid input parameter
 * 	EBUSY 		- device busy
 * 	ETIMEDOUT	- timeout
 */

/*
 * qspi_api_cmd_xfer
 * 	ENXIO 		- No such device
 * 	EINVAL 		- invalid input parameter
 * 	EBUSY 		- device busy
 * 	ETIMEDOUT	- timeout
 */









///*
// * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
// *
// * SPDX-License-Identifier: Apache-2.0
// */
///* Includes ------------------------------------------------------------------*/
//#include <drivers/qspi.h>
//#include <nrfx_qspi.h>
//#include <stdio.h>
//
//#define LOG_DOMAIN "qspi_nrfx_qspi"
//#define LOG_LEVEL CONFIG_QSPI_LOG_LEVEL
//#include <logging/log.h>
//LOG_MODULE_REGISTER(qspi_nrfx_qspi);
//
//#include "qspi_context.h"
//
///* Private typedef -----------------------------------------------------------*/
///* Private define ------------------------------------------------------------*/
//#define NO_ADDRESS				(-1)				/**< No addres used. */
//#define CUSTOM_CMD_BUFF_SIZE	8					/**< Size of the buffer for custom commands. It is limited by . */
//
//#define QSPI_STD_CMD_WRSR		0x01				/**< Write Status Register command */
//#define QSPI_STD_CMD_QE			0x40				/**< Quad Enable command. */
//
///* Private macro -------------------------------------------------------------*/
///* Private variables ---------------------------------------------------------*/
//
///* Structure with basic structure */
//struct qspi_nrfx_data {
//	struct qspi_context ctx;
//	size_t chunk_len;
//	bool   busy;
//#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
//	u32_t pm_state;
//#endif
//};
//
///* Main config structure */
//struct qspi_nrfx_config {
//	nrfx_qspi_config_t config;
//};
//
///* Private function prototypes -----------------------------------------------*/
//static void transfer_next_chunk(struct device *dev);
//static int qspi_enable_quad_transfer(void);
//static int qspi_nrfx_cmd_xfer(struct device *dev,
//					const struct qspi_config *qspi_cfg,
//					const void *tx_buf,
//					size_t tx_len,
//					const void *rx_buf,
//					size_t rx_len,
//					u32_t op_code,
//					u32_t address);
//
//static int qspi_nrfx_write(struct device *dev,
//			       const struct qspi_config *qspi_cfg,
//			       const void  *tx_buf,
//				   size_t len,
//				   uint32_t address);
//
//static int qspi_nrfx_read(struct device *dev,
//					const struct qspi_config *qspi_cfg,
//					const void * rx_buf,
//					size_t len,
//					uint32_t address);
//
//static int qspi_nrfx_release(struct device *dev,
//			    const struct qspi_config *qspi_cfg);
//
///* API definition */
//static const struct qspi_driver_api qspi_nrfx_driver_api = {
//	.cmd_xfer = qspi_nrfx_cmd_xfer,
//	.write = qspi_nrfx_write,
//	.read = qspi_nrfx_read,
//#ifdef CONFIG_QSPI_ASYNC
//	.transceive_async = qspi_nrfx_transceive_async,
//#endif
//	.release = qspi_nrfx_release,
//};
//
//
///* Private functions ---------------------------------------------------------*/
//
///**
// * @brief Get device data
// * This function returns pointer to the device data
// *
// * @param dev - pointer to the device
// */
//static inline struct qspi_nrfx_data *get_dev_data(struct device *dev)
//{
//	return dev->driver_data;
//}
//
///**
// * @brief Get configuration of the device
// * This function returns pointer to the configuration of device
// *
// * @param dev - pointer to the device
// */
//static inline const struct qspi_nrfx_config *get_dev_config(struct device *dev)
//{
//	return dev->config->config_info;
//}
//
///**
// * @brief Get QSPI mode
// * This function returns actual QSPI mode
// *
// * @param operation - QSPI config field
// * @retval NRF_SPI_MODEin case of success or
// * 		   -EINVAL in case of failure
// */
//static inline int get_nrf_qspi_mode(u16_t operation)
//{
//	if (QSPI_MODE_GET(operation) & QSPI_MODE_CPOL) {
//		if (QSPI_MODE_GET(operation) & QSPI_MODE_CPHA) {
//			/* Mode 3 detected */
//			return NRF_QSPI_MODE_1;							/**< Mode 1 (CPOL=1, CPHA=1). */
//		}
//		else{
//			LOG_ERR("QSPI mode not supported");
//			return -EINVAL;
//		}
//	}
//	else{
//		if (QSPI_MODE_GET(operation) & QSPI_MODE_CPHA){
//			LOG_ERR("QSPI mode not supported");
//			return -EINVAL;
//		}
//		else{
//			/* Mode 0 detected */
//			return NRF_QSPI_MODE_0;							/**< Mode 0 (CPOL=0, CPHA=0). */
//		}
//	}
//}
//
///**
// * @brief Get QSPI prescaler
// * Function returns prescaler for required frequency
// *
// * @param frequency - QSPI config field
// * @retval NRF_SPI_PRESCALER in case of success or
// * 		   -EINVAL in case of failure
// */
//static inline nrf_qspi_frequency_t get_nrf_qspi_prescaler(u32_t frequency)
//{
//	/* Get the highest supported frequency prescaler not exceeding the requested one. */
//
//	if (frequency < 2130000) {
//		return NRF_QSPI_FREQ_32MDIV16;	/**< 2.00 MHz. */
//	} else if (frequency < 2290000) {
//		return NRF_QSPI_FREQ_32MDIV15;	/**< 2.13 MHz. */
//	} else if (frequency < 2460000) {
//		return NRF_QSPI_FREQ_32MDIV14;	/**< 2.29 MHz. */
//	} else if (frequency < 2660000) {
//		return NRF_QSPI_FREQ_32MDIV13;	/**< 2.46 MHz. */
//	} else if (frequency < 2900000) {
//		return NRF_QSPI_FREQ_32MDIV12;	/**< 2.66 MHz. */
//	} else if (frequency < 3200000) {
//		return NRF_QSPI_FREQ_32MDIV11;	/**< 2.90 MHz. */
//	} else if (frequency < 3550000) {
//		return NRF_QSPI_FREQ_32MDIV10;	/**< 3.20 MHz. */
//	} else if (frequency < 4000000) {
//		return NRF_QSPI_FREQ_32MDIV9;	/**< 3.55 MHz. */
//	} else if (frequency < 4570000) {
//		return NRF_QSPI_FREQ_32MDIV8;	/**< 4.00 MHz. */
//	} else if (frequency < 5330000) {
//		return NRF_QSPI_FREQ_32MDIV7;	/**< 4.57 MHz. */
//	} else if (frequency < 6400000) {
//		return NRF_QSPI_FREQ_32MDIV6;	/**< 5.33 MHz. */
//	} else if (frequency < 8000000) {
//		return NRF_QSPI_FREQ_32MDIV5;	/**< 6.40 MHz. */
//	} else if (frequency < 10600000) {
//		return NRF_QSPI_FREQ_32MDIV4;	/**< 8.00 MHz. */
//	} else if (frequency < 16000000) {
//		return NRF_QSPI_FREQ_32MDIV3;	/**< 10.6 MHz. */
//	} else if (frequency < 32000000) {
//		return NRF_QSPI_FREQ_32MDIV2;	/**< 16.0 MHz. */
//	} else {
//		return NRF_QSPI_FREQ_32MDIV1;	/**< 32.0 MHz. */
//	}
//}
//
//
///**
// * @brief Get QSPI address mode
// * Two types of address are avaiable: 24bit or 32bit
// *
// * @param operation - QSPI config field
// * @retval NRF_SPI_ADDRMODE in case of success or
// * 		   -EINVAL in case of failure
// */
//static inline int get_nrf_qspi_address_mode(u16_t operation)
//{
//	if (QSPI_ADDRESS_MODE_GET(operation) == QSPI_ADDRESS_MODE_24BIT) {		/**< 24 BIT ADDR. */
//		return NRF_QSPI_ADDRMODE_24BIT;
//	}
//	else if (QSPI_ADDRESS_MODE_GET(operation) == QSPI_ADDRESS_MODE_32BIT){	/**< 32 BIT ADDR. */
//		return NRF_QSPI_ADDRMODE_32BIT;
//	}
//	else{
//		LOG_ERR("Address modes other than 24 or 32 bits are not supported");
//		return -EINVAL;
//	}
//}
//
///**
// * @brief Get QSPI read operation code
// * Amount of lines used for transfer
// *
// * @param operation - QSPI config field
// * @retval NRF_QSPI_READOC in case of success or
// * 		   -EINVAL in case of failure
// */
//static inline int get_nrf_qspi_readoc(u16_t operation)
//{
//	if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_SINGLE) {			/**< SINGLE line operation. */
//		return NRF_QSPI_READOC_FASTREAD;
//	}
//	else if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_DOUBLE){		/**< DOUBLE line operation. */
//		return NRF_QSPI_READOC_READ2IO;
//	}
//	else if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_QUAD){		/**< QUAD line operation. */
//		return NRF_QSPI_READOC_READ4IO;
//	}
//	else{
//		LOG_ERR("Not supported line type");
//		return -EINVAL;
//	}
//}
//
///**
// * @brief Get QSPI read operation code
// * Amount of lines used for transfer
// *
// * @param operation - QSPI config field
// * @retval NRF_QSPI_READOC in case of success or
// * 		   -EINVAL in case of failure
// */
//static inline int get_nrf_qspi_wrieoc(u16_t operation)
//{
//	if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_SINGLE) {			/**< SINGLE line operation. */
//		return NRF_QSPI_WRITEOC_PP;
//	}
//	else if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_DOUBLE){		/**< DOUBLE line operation. */
//		return NRF_QSPI_WRITEOC_PP2O;
//	}
//	else if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_QUAD){		/**< QUAD line operation. */
//		return NRF_QSPI_WRITEOC_PP4IO;
//	}
//	else{
//		LOG_ERR("Not supported line type");
//		return -EINVAL;
//	}
//}
//
///**
// * @brief QSPI handler
// * Amount of lines used for transfer
// *
// * @param event 	- TBD
// * @param p_context - TBD
// * @retval None
// */
//static void qspi_handler(nrfx_qspi_evt_t event, void * p_context)
//{
//	struct device *dev = p_context;
//	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
//
//	if (event == NRFX_QSPI_EVENT_DONE) {
//		qspi_context_update_tx(&dev_data->ctx, 1, dev_data->chunk_len);
//		qspi_context_update_rx(&dev_data->ctx, 1, dev_data->chunk_len);
//
//		transfer_next_chunk(dev);
//	}
//
//}
//
///**
// * @brief Fill config struct
// * Internal function - fills init struct
// *
// * @param pConfig	- config structure to be filled
// * @param qspi_cfg	- config struct that is the source of data
// * @retval None
// */
//static inline void fill_config_struct(nrfx_qspi_config_t * pConfig, const struct qspi_config *qspi_cfg){
//	{
//		/* Configure XIP offset */
//		pConfig->xip_offset  = NRFX_QSPI_CONFIG_XIP_OFFSET;
//
//		/* Configure pins */
//		pConfig->pins.sck_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_SCK_PIN;
//		pConfig->pins.csn_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_CSN_PIN;
//		pConfig->pins.io0_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO00_PIN;
//		pConfig->pins.io1_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO01_PIN;
//		pConfig->pins.io2_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO02_PIN;
//		pConfig->pins.io3_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO03_PIN;
//
//		/* Configure IRQ priority */
//		pConfig->irq_priority   = (uint8_t)NRFX_QSPI_CONFIG_IRQ_PRIORITY;
//
//		/* Configure Protocol interface */
//		pConfig->prot_if.readoc     = (nrf_qspi_readoc_t)get_nrf_qspi_readoc(qspi_cfg->operation);
//		pConfig->prot_if.writeoc    = (nrf_qspi_writeoc_t)get_nrf_qspi_wrieoc(qspi_cfg->operation);
//		pConfig->prot_if.addrmode   = (nrf_qspi_addrmode_t)get_nrf_qspi_address_mode(qspi_cfg->operation);
//		pConfig->prot_if.dpmconfig  = false;
//
//		/* COnfigure physical interface */
//		pConfig->phy_if.sck_freq   = (nrf_qspi_frequency_t)get_nrf_qspi_prescaler(qspi_cfg->frequency);
//		pConfig->phy_if.sck_delay  = (uint8_t)QSPI_CS_DELAY_GET(qspi_cfg->operation);
//		pConfig->phy_if.spi_mode   = (nrf_qspi_spi_mode_t)get_nrf_qspi_mode(qspi_cfg->operation);
//		pConfig->phy_if.dpmen      = false;
//	}
//}
//
///**
// * @brief Configures QSPI
// * Uses config provided in qspi_cfg to fill QSPI configuration
// *
// * @param dev		- pointer to the device context
// * @param qspi_cfg	- config struct that is the source of the configuration data
// * @retval 0 in case of success
// * 			-EINVAL in case of failure
// */
//static int configure(struct device *dev, const struct qspi_config *qspi_cfg)
//{
//	struct qspi_context *ctx = &get_dev_data(dev)->ctx;
//
//	if (qspi_context_configured(ctx, qspi_cfg)) {
//		/* Already configured. No need to do it again. */
//		return 0;
//	}
//	/* Checking input parameters */
//	if (QSPI_CS_DELAY_GET(qspi_cfg->operation) > 0xFF) {
//		LOG_ERR("CS delay not supported");
//		return -EINVAL;
//	}
//
//	/* Check QSPI mode input parameter */
//	if(get_nrf_qspi_mode(qspi_cfg->operation) == (-EINVAL)){
//		return -EINVAL;
//	}
//
//	/* Check address mode input parameter */
//	if(get_nrf_qspi_address_mode(qspi_cfg->operation) == (-EINVAL)){
//		return -EINVAL;
//	}
//
//	/* Check data-line mode input parameter */
//	if(get_nrf_qspi_readoc(qspi_cfg->operation) == (-EINVAL)){
//		return -EINVAL;
//	}
//
//	/* Uninit QSPI - we cannot just rerun Init, since it will fail */
//	nrfx_qspi_uninit();
//
//	/* Fill config structure */
//	nrfx_qspi_config_t config;
//	fill_config_struct(&config, qspi_cfg);
//
//	/* Input parameters OK - initialise QSPI */
//	if(nrfx_qspi_init(&config, qspi_handler, dev ) != NRFX_SUCCESS){
//		return -EIO;
//	}
//
//	/* Config finished. Check if QUAD mode is chosen - if so, we have to enable QUAD transfer in flash device */
//	if(config.prot_if.readoc == NRF_QSPI_READOC_READ4IO){
//		 if (qspi_enable_quad_transfer() != NRFX_SUCCESS){
//			 return -EIO;
//		 }
//	}
//	return 0;
//}
//
///**
// * @brief Function transceives data over QSPI
// * Sends or receives next chunk of data
// *
// * @param dev		- pointer to the device context
// * @retval None
// */
//static void transceive_next_chunk(struct device *dev)
//{
//	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
//	struct qspi_context *ctx = &dev_data->ctx;
//
//	int error = 0;
//	nrfx_err_t result;
//
//	size_t chunk_len = qspi_context_longest_current_buf(ctx);
//	dev_data->chunk_len = chunk_len;
//
//	/* Create additional temporary buffer for tx purposes */
//	uint8_t tx_tmp_buf[8] = {0};
//	uint8_t tx_head_len = 0;
//
//	/* If there is an address -> add it to the buffer */
//	if(ctx->address != NO_ADDRESS){
//		tx_tmp_buf[0] = (uint8_t)(ctx->address >> 16);
//		tx_tmp_buf[1] = (uint8_t)(ctx->address >> 8);
//		tx_tmp_buf[2] = (uint8_t)(ctx->address);
//		tx_head_len = 3;
//	}
//
//	/* If there is a msg - copy it to the buffer */
//	if(ctx->tx_len > 0){
//		memcpy(&tx_tmp_buf[tx_head_len], ctx->tx_buf,ctx->tx_len);
//	}
//
//	/* Preapre config for transfer */
//	nrf_qspi_cinstr_conf_t cinstr_cfg = {
//		.opcode = ctx->op_code,
//		.length = NRF_QSPI_CINSTR_LEN_1B + (nrf_qspi_cinstr_len_t)chunk_len + (nrf_qspi_cinstr_len_t)(tx_head_len),
//		.io2_level = true,
//		.io3_level = true,
//		.wipwait = true,
//		.wren = true
//	};
//
//	/* Start transfer */
//	result = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_tmp_buf, ctx->rx_buf);
//
//	if (result != NRFX_SUCCESS) {
//		error = -EIO;
//		qspi_context_cs_control(ctx, false);
//
//		LOG_DBG("Transaction finished with status %d", error);
//
//		dev_data->busy = false;
//		return;
//	}
//	else{
//		return;
//	}
//}
//
///**
// * @brief Send data over QSPI
// * Sends data over QSPI
// *
// * @param	dev		- pointer to the device context
// * @retval	None
// */
//static void transfer_next_chunk(struct device *dev)
//{
//	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
//	struct qspi_context *ctx = &dev_data->ctx;
//	int error = 0;
//
//	size_t chunk_len = qspi_context_longest_current_buf(ctx);
//
//	if (chunk_len > 0) {
//		nrfx_err_t result;
//
//		dev_data->chunk_len = chunk_len;
//
//		result = nrfx_qspi_write(ctx->tx_buf, chunk_len, ctx->address);
//
//		if (result == NRFX_SUCCESS) {
//			return;
//		}
//
//		error = -EIO;
//	}
//
//	qspi_context_cs_control(ctx, false);
//
//	LOG_DBG("Transaction finished with status %d", error);
//
//	qspi_context_complete(ctx, error);
//	dev_data->busy = false;
//}
//
///**
// * @brief Receives data over QSPI
// * Receives data over QSPI
// *
// * @param	dev		- pointer to the device context
// * @retval	None
// */
//static void receive_next_chunk(struct device *dev)
//{
//	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
//	struct qspi_context *ctx = &dev_data->ctx;
//	int error = 0;
//
//	size_t chunk_len = qspi_context_longest_current_buf(ctx);
//
//	if (chunk_len > 0) {
//		nrfx_err_t result;
//
//		dev_data->chunk_len = chunk_len;
//		result = nrfx_qspi_read(ctx->rx_buf, chunk_len, ctx->address);
//
//		if (result == NRFX_SUCCESS) {
//			return;
//		}
//
//		error = -EIO;
//	}
//
//	qspi_context_cs_control(ctx, false);
//
//	LOG_DBG("Transaction finished with status %d", error);
//
//	qspi_context_complete(ctx, error);
//	dev_data->busy = false;
//}
//
///**
// * @brief Transceives data over QSPI
// * Function enables user to send custom commands to qspi flash device
// *
// * @param	dev			- pointer to the device context
// * @param	qspi_cfg	- pointer to the device configuration struct
// * @param	tx_buf		- pointer to the TX buffer	- if not used pass NULL
// * @param	tx_len		- amount of TX data 		- if not used pass 0. Maximum allowed length is 9 bytes (limitation of nrf_qspi_cinstr_conf_t)
// * @param	rx_buf		- pointer to the RX buffer	- if not used pass NULL
// * @param	rx_len		- amount of RX data 		- if not used pass 0
// * @param	op_code		- operation code (i.e "9F" - get JEDEC ID)
// * @param	address		- address - if not used, pass 0return
// * @retval	0 in case of success
// * 			errocode in case of failure
// */
//static int qspi_nrfx_cmd_xfer(struct device *dev,
//					const struct qspi_config *qspi_cfg,
//					const void *tx_buf,
//					size_t tx_len,
//					const void *rx_buf,
//					size_t rx_len,
//					u32_t op_code,
//					u32_t address)
//{
//	qspi_context_lock(&get_dev_data(dev)->ctx, false, NULL);
//	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
//	int error;
//
//	error = configure(dev, qspi_cfg);
//	if (error == 0) {
//		dev_data->busy = true;
//
//		qspi_context_buffers_setup(&dev_data->ctx, tx_buf, tx_len, rx_buf, rx_len, address, op_code, 1);
//		qspi_context_cs_control(&dev_data->ctx, true);
//		transceive_next_chunk(dev);
//
//	}
//	else{
//		qspi_context_release(&dev_data->ctx, error);
//		return error;
//	}
//	/* Wait for end of the transfer */
//	while(nrfx_qspi_mem_busy_check() != NRFX_SUCCESS){};
//	qspi_context_release(&dev_data->ctx, error);
//
//	return error;
//}
//
///**
// * @brief Writes data over QSPI
// * Function enables user to send data to qspi flash device
// *
// * @param	dev			- pointer to the device context
// * @param	qspi_cfg	- pointer to the device configuration struct
// * @param	tx_buf		- pointer to the TX buffer
// * @param	tx_len		- amount of TX data 		- MUST be dividable by 4
// * @param	address		- address, where the data will be saved
// * @retval	0 in case of success
// * 			errocode in case of failure
// */
//static int qspi_nrfx_write(struct device *dev,
//			       const struct qspi_config *qspi_cfg,
//			       const void  *tx_buf,
//				   size_t len,
//				   uint32_t address)
//{
//	qspi_context_lock(&get_dev_data(dev)->ctx, false, NULL);
//
//	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
//	int error;
//
//	error = configure(dev, qspi_cfg);
//	if (error == 0) {
//		dev_data->busy = true;
//
//		qspi_context_buffers_setup(&dev_data->ctx, tx_buf, len, NULL, 0, address, 0, 1);
//		qspi_context_cs_control(&dev_data->ctx, true);
//		transfer_next_chunk(dev);
//		error = qspi_context_wait_for_completion(&dev_data->ctx);
//	}
//
//	qspi_context_release(&dev_data->ctx, error);
//
//	return error;
//}
//
///**
// * @brief Reads data over QSPI
// * Function enables user to receive data from qspi flash device
// *
// * @param	dev			- pointer to the device context
// * @param	qspi_cfg	- pointer to the device configuration struct
// * @param	rx_buf		- pointer to the RX buffer
// * @param	rx_len		- amount of RX data
// * @param	address		- address, from data will be read
// * @retval	0 in case of success
// * 			errocode in case of failure
// */
//static int qspi_nrfx_read(struct device *dev,
//					const struct qspi_config *qspi_cfg,
//					const void * rx_buf,
//					size_t len,
//					uint32_t address)
//{
//	qspi_context_lock(&get_dev_data(dev)->ctx, false, NULL);net_buf
//
//	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
//	int error;
//
//	error = configure(dev, qspi_cfg);
//	if (error == 0) {
//		dev_data->busy = true;
//
//		qspi_context_buffers_setup(&dev_data->ctx, NULL, 0, rx_buf, len, address, 0, 1);
//		qspi_context_cs_control(&dev_data->ctx, true);
//		receive_next_chunk(dev);
//		error = qspi_context_wait_for_completion(&dev_data->ctx);
//	}
//
//	qspi_context_release(&dev_data->ctx, error);
//
//	return error;
//}
//
//#ifdef CONFIG_QSPI_ASYNC
//static int qspi_nrfx_transceive_async(struct device *dev,
//				     const struct qspi_config *qspi_cfg,
//				     const struct qspi_buf_set *tx_bufs,
//				     const struct qspi_buf_set *rx_bufs,
//				     struct k_poll_signal *async)return
//{
////	qspi_context_lock(&get_dev_data(dev)->ctx, true, async);
////	return transceive(dev, qspi_cfg, tx_bufs, rx_bufs);
//	return 0;
//}
//#endif /* CONFIG_QSPI_ASYNC */
//
///**
// * @brief Releases QSPI context
// *
// * @param	dev			- pointer to the device context
// * @param	qspi_cfg	- pointer to the device configuration struct
// * @retval	0 in case of success
// * 			errocode in case of failure
// */
//static int qspi_nrfx_release(struct device *dev,
//			    const struct qspi_config *qspi_cfg)
//{
//	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
//
//	if (!qspi_context_configured(&dev_data->ctx, qspi_cfg)) {
//		return -EINVAL;
//	}
//
//	if (dev_data->busy) {
//		return -EBUSY;
//	}
//
//	qspi_context_unlock_unconditionally(&dev_data->ctx);
//
//	return 0;
//}
//
//
//static int init_qspi(struct device *dev)
//{
//	/* This sets only default values of frequency, mode and bit order.
//	 * The proper ones are set in configure() when a transfer is started.
//	 */
//	nrfx_err_t result = nrfx_qspi_init(&get_dev_config(dev)->config, qspi_handler, dev);
//	if (result != NRFX_SUCCESS) {
//		LOG_ERR("Failed to initialize device: %s",
//			    dev->config->name);
//		return -EBUSY;
//	}
//
////#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
////	get_dev_data(dev)->pm_state = DEVICE_PM_ACTIVE_STATE;
////#endif
//	qspi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);
//
//	return 0;
//}
//
////----------------------------------------------------------------------------------------------------------------------------
///**
//  * @brief Updates StatusRegister of ext flash memory by sending "Quad Enable" (0x40).
//  *
//  * Use this command to enable QUAD transfer.
//  *
//  * @param None
//  * @retval An ErrorStatus enumeration value:
//  *          - NRFX_SUCCESS            If the operation was successful.
//  *          - NRFX_ERROR_TIMEOUT      If the external memory is busy or there are connection issues.
//  *          - NRFX_ERROR_BUSY         If the driver currently handles other operation.
//  */
//static int qspi_enable_quad_transfer(void){
//	uint32_t cmd = QSPI_STD_CMD_QE;
//	nrf_qspi_cinstr_conf_t cinstr_cfg = {
//		.opcode = QSPI_STD_CMD_WRSR,
//		.length = NRF_QSPIstatic const struct_CINSTR_LEN_2B,
//		.io2_level = true,
//		.io3_level = true,
//		.wipwait = true,
//		.wren = true
//	};
//
//	return nrfx_qspi_cinstr_xfer(&cinstr_cfg, &cmd, NULL);
//}
//
////----------------------------------------------------------------------------------------------------------------------------
//
//#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
//static int qspi_nrfx_pm_control(struct device *dev, u32_t ctrl_command,
//				void *context, device_pm_cb cb, void *arg)
//{
////	int ret = 0;
////return
////	if (ctrl_command == DEVICE_PM_SET_POWER_STATE) {
////		u32_t new_state = *((const u32_t *)context);
////
////		if (new_state != get_dev_data(dev)->pm_state) {
////			switch (new_state) {
////			case DEVICE_PM_ACTIVE_STATE:
////				init_qspi(dev);
////				/* Force reconfiguration before next transfer */
////				get_dev_data(dev)->ctx.config = NULL;
////				break;
////
////			case DEVICE_PM_LOW_POWER_STATE:
////			case DEVICE_PM_SUSPEND_STATE:
////			case DEVICE_PM_OFF_STATE:
////				nrfx_qspi_uninit(&get_dev_config(dev)->qspi);
////				break;
////
////			default:
////				ret = -ENOTSUP;
////			}
////			if (!ret) {
////				get_dev_data(dev)->pm_state = new_state;
////			}
////		}
////	} else {
////		assert(ctrl_command == DEVICE_PM_GET_POWER_STATE);
////		*((u32_t *)context) = get_dev_data(dev)->pm_state;
////	}
////
////	if (cb) {
////		cb(dev, ret, context, arg);
////	}
////
////	return ret;
//}
//#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */
//
//
//#define QSPI_NRFX_QSPI_DEVICE(void)					       						\
//	static int qspi_init(struct device *dev)			       					\
//	{								       										\
//		IRQ_CONNECT(DT_NORDIC_NRF_QSPI_QSPI_0_IRQ_0,		       				\
//			    DT_NORDIC_NRF_QSPI_QSPI_0_IRQ_0_PRIORITY,      					\
//			    nrfx_isr, nrfx_qspi_irq_handler, 0);	       					\
//	return init_qspi(dev);					       								\
//	}								       										\
//	static struct qspi_nrfx_data qspi_data = {		       						\
//		QSPI_CONTEXT_INIT_LOCK(qspi_data, ctx),		       						\
//		QSPI_CONTEXT_INIT_SYNC(qspi_data, ctx),		       						\
//		.busy = false,						       								\
//	};								       										\
//	static const struct qspi_nrfx_config qspiz_config = {	       				\
//		.config = {						       									\
//			.xip_offset  = NRFX_QSPI_CONFIG_XIP_OFFSET,                         \
//			.pins = {                                                           \
//			   .sck_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_SCK_PIN,                \
//			   .csn_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_CSN_PIN,                \
//			   .io0_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_IO00_PIN,               \
//			   .io1_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_IO01_PIN,               \
//			   .io2_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_IO02_PIN,               \
//			   .io3_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_IO03_PIN,               \
//			},                                                                  \
//			.irq_priority   = (uint8_t)NRFX_QSPI_CONFIG_IRQ_PRIORITY,           \
//			.prot_if = {                                                        \
//				.readoc     = (nrf_qspi_readoc_t)NRFX_QSPI_CONFIG_READOC,       \
//				.writeoc    = (nrf_qspi_writeoc_t)NRFX_QSPI_CONFIG_WRITEOC,     \
//				.addrmode   = (nrf_qspi_addrmode_t)NRF_QSPI_ADDRMODE_24BIT,   	\
//				.dpmconfig  = false,                                            \
//			},                                                                  \
//			.phy_if = {                                                         \
//				.sck_freq   = (nrf_qspi_frequency_t)NRFX_QSPI_CONFIG_FREQUENCY, \
//				.sck_delay  = (uint8_t)NRFX_QSPI_CONFIG_SCK_DELAY,              \
//				.spi_mode   = (nrf_qspi_spi_mode_t)NRFX_QSPI_CONFIG_MODE,       \
//				.dpmen      = false                                             \
//			},                                                                  \
//		}							       										\
//	};								       										\
//	DEVICE_DEFINE(qspi, DT_NORDIC_NRF_QSPI_QSPI_0_LABEL,	       				\
//		      qspi_init,					       								\
//		      qspi_nrfx_pm_control,				       							\
//		      &qspi_data,				       									\
//		      &qspiz_config,				       								\
//		      POST_KERNEL, CONFIG_QSPI_INIT_PRIORITY,		      	 			\
//		      &qspi_nrfx_driver_api)
//
//#ifdef CONFIG_QSPI_NRF_QSPI
//QSPI_NRFX_QSPI_DEVICE();
//#endif
