/*
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Includes ------------------------------------------------------------------*/
#include <drivers/qspi.h>
#include <nrfx_qspi.h>
#include <stdio.h>

#define LOG_DOMAIN "qspi_nrfx_qspi"
#define LOG_LEVEL CONFIG_QSPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(qspi_nrfx_qspi);

#include "qspi_context.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NO_ADDRESS			(-1)				/**< No addres used. */

#define QSPI_STD_CMD_WRSR	0x01				/**< Write Status Register command */
#define QSPI_STD_CMD_QE		0x40				/**< Quad Enable command. */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Structure with basic structure */
struct qspi_nrfx_data {
	struct qspi_context ctx;
	size_t chunk_len;
	bool   busy;
#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	u32_t pm_state;
#endif
};

/* Main config structure */
struct qspi_nrfx_config {
	nrfx_qspi_config_t config;
};
/* Private function prototypes -----------------------------------------------*/
static void transfer_next_chunk(struct device *dev);
static int qspi_enable_quad_transfer(void);
static int qspi_nrfx_cmd_xfer(struct device *dev,
					const struct qspi_config *qspi_cfg,
					const void *tx_buf,
					size_t tx_len,
					const void *rx_buf,
					size_t rx_len,
					u32_t op_code,
					u32_t address);

static int qspi_nrfx_write(struct device *dev,
			       const struct qspi_config *qspi_cfg,
			       const void  *tx_buf,
				   size_t len,
				   uint32_t address);

static int qspi_nrfx_read(struct device *dev,
					const struct qspi_config *qspi_cfg,
					const void * rx_buf,
					size_t len,
					uint32_t address);

static int qspi_nrfx_release(struct device *dev,
			    const struct qspi_config *qspi_cfg);

/* API definition */
static const struct qspi_driver_api qspi_nrfx_driver_api = {
	.cmd_xfer = qspi_nrfx_cmd_xfer,
	.write = qspi_nrfx_write,
	.read = qspi_nrfx_read,
#ifdef CONFIG_QSPI_ASYNC
	.transceive_async = qspi_nrfx_transceive_async,
#endif
	.release = qspi_nrfx_release,
};


/* Private functions ---------------------------------------------------------*/
static inline struct qspi_nrfx_data *get_dev_data(struct device *dev)
{
	return dev->driver_data;
}

static inline const struct qspi_nrfx_config *get_dev_config(struct device *dev)
{
	return dev->config->config_info;
}


static inline int get_nrf_qspi_mode(u16_t operation)
{
	if (QSPI_MODE_GET(operation) & QSPI_MODE_CPOL) {
		if (QSPI_MODE_GET(operation) & QSPI_MODE_CPHA) {
			/* Mode 3 detected */
			return NRF_QSPI_MODE_1;							/**< Mode 1 (CPOL=1, CPHA=1). */
		}
		else{
			LOG_ERR("QSPI mode not supported");
			return -EINVAL;
		}
	}
	else{
		if (QSPI_MODE_GET(operation) & QSPI_MODE_CPHA){
			LOG_ERR("QSPI mode not supported");
			return -EINVAL;
		}
		else{
			/* Mode 0 detected */
			return NRF_QSPI_MODE_0;							/**< Mode 0 (CPOL=0, CPHA=0). */
		}
	}
}

/* Function returns prescaler for required drequency */
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
		return NRF_QSPI_FREQ_32MDIV1;	/**< 32.0 MHz. */
	}
}

static inline int get_nrf_qspi_address_mode(u16_t operation)
{
	if (QSPI_ADDRESS_MODE_GET(operation) == QSPI_ADDRESS_MODE_24BIT) {		/**< 24 BIT ADDR. */
		return NRF_QSPI_ADDRMODE_24BIT;
	}
	else if (QSPI_ADDRESS_MODE_GET(operation) == QSPI_ADDRESS_MODE_32BIT){	/**< 32 BIT ADDR. */
		return NRF_QSPI_ADDRMODE_32BIT;
	}
	else{
		LOG_ERR("Address modes other than 24 or 32 bits are not supported");
		return -EINVAL;
	}
}

static inline int get_nrf_qspi_readoc(u16_t operation)
{
	if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_SINGLE) {			/**< SINGLE line operation. */
		return NRF_QSPI_READOC_FASTREAD;
	}
	else if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_DOUBLE){		/**< DOUBLE line operation. */
		return NRF_QSPI_READOC_READ2IO;
	}
	else if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_QUAD){		/**< QUAD line operation. */
		return NRF_QSPI_READOC_READ4IO;
	}
	else{
		LOG_ERR("Not supported line type");
		return -EINVAL;
	}
}

static inline int get_nrf_qspi_wrieoc(u16_t operation)
{
	if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_SINGLE) {			/**< SINGLE line operation. */
		return NRF_QSPI_WRITEOC_PP;
	}
	else if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_DOUBLE){		/**< DOUBLE line operation. */
		return NRF_QSPI_WRITEOC_PP2O;
	}
	else if (QSPI_DATA_LINES_GET(operation) == QSPI_DATA_LINES_QUAD){		/**< QUAD line operation. */
		return NRF_QSPI_WRITEOC_PP4IO;
	}
	else{
		LOG_ERR("Not supported line type");
		return -EINVAL;
	}
}


static void qspi_handler(nrfx_qspi_evt_t event, void * p_context)
{
	struct device *dev = p_context;
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);

	if (event == NRFX_QSPI_EVENT_DONE) {
		qspi_context_update_tx(&dev_data->ctx, 1, dev_data->chunk_len);
		qspi_context_update_rx(&dev_data->ctx, 1, dev_data->chunk_len);

		transfer_next_chunk(dev);
	}

}

static inline void fill_config_struct(nrfx_qspi_config_t * pConfig, const struct qspi_config *qspi_cfg){
	{
		/* Configure XIP offset */
		pConfig->xip_offset  = NRFX_QSPI_CONFIG_XIP_OFFSET;

		/* Configure pins */
		pConfig->pins.sck_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_SCK_PIN;
		pConfig->pins.csn_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_CSN_PIN;
		pConfig->pins.io0_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO00_PIN;
		pConfig->pins.io1_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO01_PIN;
		pConfig->pins.io2_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO02_PIN;
		pConfig->pins.io3_pin  = DT_NORDIC_NRF_QSPI_QSPI_0_IO03_PIN;

		/* Configure IRQ priority */
		pConfig->irq_priority   = (uint8_t)NRFX_QSPI_CONFIG_IRQ_PRIORITY;

		/* Configure Protocol interface */
		pConfig->prot_if.readoc     = (nrf_qspi_readoc_t)get_nrf_qspi_readoc(qspi_cfg->operation);
		pConfig->prot_if.writeoc    = (nrf_qspi_writeoc_t)get_nrf_qspi_wrieoc(qspi_cfg->operation);
		pConfig->prot_if.addrmode   = (nrf_qspi_addrmode_t)get_nrf_qspi_address_mode(qspi_cfg->operation);
		pConfig->prot_if.dpmconfig  = false;

		/* COnfigure physical interface */
		pConfig->phy_if.sck_freq   = (nrf_qspi_frequency_t)get_nrf_qspi_prescaler(qspi_cfg->frequency);
		pConfig->phy_if.sck_delay  = (uint8_t)QSPI_CS_DELAY_GET(qspi_cfg->operation);
		pConfig->phy_if.spi_mode   = (nrf_qspi_spi_mode_t)get_nrf_qspi_mode(qspi_cfg->operation);
		pConfig->phy_if.dpmen      = false;
	}
}

static int configure(struct device *dev, const struct qspi_config *qspi_cfg)
{
	struct qspi_context *ctx = &get_dev_data(dev)->ctx;
//	const nrfx_qspi_config_t * p_config = &get_dev_config(dev)->config;

	if (qspi_context_configured(ctx, qspi_cfg)) {
		/* Already configured. No need to do it again. */
		return 0;
	}
	/* Checking input parameters */
	if (QSPI_CS_DELAY_GET(qspi_cfg->operation) > 0xFF) {
		LOG_ERR("CS delay not supported");
		return -EINVAL;
	}

	/* Check QSPI mode input parameter */
	if(get_nrf_qspi_mode(qspi_cfg->operation) == (-EINVAL)){
		return -EINVAL;
	}

	/* Check address mode input parameter */
	if(get_nrf_qspi_address_mode(qspi_cfg->operation) == (-EINVAL)){
		return -EINVAL;
	}

	/* Check data-line mode input parameter */
	if(get_nrf_qspi_readoc(qspi_cfg->operation) == (-EINVAL)){
		return -EINVAL;
	}

	/* Uninit QSPI - we cannot just rerun Init, since it will fail */
	nrfx_qspi_uninit();

	/* Fill config structure */
	nrfx_qspi_config_t config;
	fill_config_struct(&config, qspi_cfg);

	/* Input parameters OK - initialise QSPI */
	if(nrfx_qspi_init(&config, qspi_handler, dev ) != NRFX_SUCCESS){
		return -EIO;
	}

	/* Config finished. Check if QUAD mode is chosen - if so, we have to enable QUAD transfer in flash device */
	if(config.prot_if.readoc == NRF_QSPI_READOC_READ4IO){
		 if (qspi_enable_quad_transfer() != NRFX_SUCCESS){
			 return -EIO;
		 }
	}
	return 0;
}

static void transceive_next_chunk(struct device *dev)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
	struct qspi_context *ctx = &dev_data->ctx;

	int error = 0;
	nrfx_err_t result;

	size_t chunk_len = qspi_context_longest_current_buf(ctx);
	dev_data->chunk_len = chunk_len;

	/* Create additional temporary buffer for tx purposes */
	uint8_t tx_tmp_buf[8] = {0};
	uint8_t tx_head_len = 0;

	/* If there is an address -> add it to the buffer */
	if(ctx->address != NO_ADDRESS){
		tx_tmp_buf[0] = (uint8_t)(ctx->address >> 16);
		tx_tmp_buf[1] = (uint8_t)(ctx->address >> 8);
		tx_tmp_buf[2] = (uint8_t)(ctx->address);
		tx_head_len = 3;
	}

	/* If there is a msg - copy it to the buffer */
	if(ctx->tx_len > 0){
		memcpy(&tx_tmp_buf[tx_head_len], ctx->tx_buf,ctx->tx_len);
	}

	/* Preapre config for transfer */
	nrf_qspi_cinstr_conf_t cinstr_cfg = {
		.opcode = ctx->op_code,
		.length = NRF_QSPI_CINSTR_LEN_1B + (nrf_qspi_cinstr_len_t)chunk_len + (nrf_qspi_cinstr_len_t)(tx_head_len),
		.io2_level = true,
		.io3_level = true,
		.wipwait = true,
		.wren = true
	};

	/* Start transfer */
	result = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_tmp_buf, ctx->rx_buf);

	if (result != NRFX_SUCCESS) {
		error = -EIO;
		qspi_context_cs_control(ctx, false);

		LOG_DBG("Transaction finished with status %d", error);

		dev_data->busy = false;
		return;
	}
	else{
		return;
	}
}

static void transfer_next_chunk(struct device *dev)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
	struct qspi_context *ctx = &dev_data->ctx;
	int error = 0;

	size_t chunk_len = qspi_context_longest_current_buf(ctx);

	if (chunk_len > 0) {
		nrfx_err_t result;

		dev_data->chunk_len = chunk_len;

		result = nrfx_qspi_write(ctx->tx_buf, chunk_len, ctx->address);

		if (result == NRFX_SUCCESS) {
			return;
		}

		error = -EIO;
	}

	qspi_context_cs_control(ctx, false);

	LOG_DBG("Transaction finished with status %d", error);

	qspi_context_complete(ctx, error);
	dev_data->busy = false;
}

static void receive_next_chunk(struct device *dev)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
	struct qspi_context *ctx = &dev_data->ctx;
	int error = 0;

	size_t chunk_len = qspi_context_longest_current_buf(ctx);

	if (chunk_len > 0) {
		nrfx_err_t result;

		dev_data->chunk_len = chunk_len;

		result = nrfx_qspi_read(ctx->rx_buf, chunk_len, ctx->address);

		if (result == NRFX_SUCCESS) {
			return;
		}

		error = -EIO;
	}

	qspi_context_cs_control(ctx, false);

	LOG_DBG("Transaction finished with status %d", error);

	qspi_context_complete(ctx, error);
	dev_data->busy = false;
}


static int qspi_nrfx_cmd_xfer(struct device *dev,
					const struct qspi_config *qspi_cfg,
					const void *tx_buf,
					size_t tx_len,
					const void *rx_buf,
					size_t rx_len,
					u32_t op_code,
					u32_t address)
{
	qspi_context_lock(&get_dev_data(dev)->ctx, false, NULL);
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
	int error;

	error = configure(dev, qspi_cfg);
	if (error == 0) {
		dev_data->busy = true;

		qspi_context_buffers_setup(&dev_data->ctx, tx_buf, tx_len, rx_buf, rx_len, address, op_code, 1);
		qspi_context_cs_control(&dev_data->ctx, true);
		transceive_next_chunk(dev);
//		error = qspi_context_wait_for_completion(&dev_data->ctx);

	}
	else{
		qspi_context_release(&dev_data->ctx, error);
		return error;
	}
	/* Wait for end of the transfer */
	while(nrfx_qspi_mem_busy_check() != NRFX_SUCCESS){};
	qspi_context_release(&dev_data->ctx, error);

	return error;
}

static int qspi_nrfx_write(struct device *dev,
			       const struct qspi_config *qspi_cfg,
			       const void  *tx_buf,
				   size_t len,
				   uint32_t address)
{
	qspi_context_lock(&get_dev_data(dev)->ctx, false, NULL);

	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
	int error;

	error = configure(dev, qspi_cfg);
	if (error == 0) {
		dev_data->busy = true;

		qspi_context_buffers_setup(&dev_data->ctx, tx_buf, len, NULL, 0, address, 0, 1);
		qspi_context_cs_control(&dev_data->ctx, true);
		transfer_next_chunk(dev);
		error = qspi_context_wait_for_completion(&dev_data->ctx);
	}

	qspi_context_release(&dev_data->ctx, error);

	return error;
}

static int qspi_nrfx_read(struct device *dev,
					const struct qspi_config *qspi_cfg,
					const void * rx_buf,
					size_t len,
					uint32_t address)
{
	qspi_context_lock(&get_dev_data(dev)->ctx, false, NULL);

	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
	int error;

	error = configure(dev, qspi_cfg);
	if (error == 0) {
		dev_data->busy = true;

		qspi_context_buffers_setup(&dev_data->ctx, NULL, 0, rx_buf, len, address, 0, 1);
		qspi_context_cs_control(&dev_data->ctx, true);
		receive_next_chunk(dev);
		error = qspi_context_wait_for_completion(&dev_data->ctx);
	}

	qspi_context_release(&dev_data->ctx, error);

	return error;
}

#ifdef CONFIG_QSPI_ASYNC
static int qspi_nrfx_transceive_async(struct device *dev,
				     const struct qspi_config *qspi_cfg,
				     const struct qspi_buf_set *tx_bufs,
				     const struct qspi_buf_set *rx_bufs,
				     struct k_poll_signal *async)
{
//	qspi_context_lock(&get_dev_data(dev)->ctx, true, async);
//	return transceive(dev, qspi_cfg, tx_bufs, rx_bufs);
	printf("Chlosta");
	return 0;
}
#endif /* CONFIG_QSPI_ASYNC */

static int qspi_nrfx_release(struct device *dev,
			    const struct qspi_config *qspi_cfg)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);

	if (!qspi_context_configured(&dev_data->ctx, qspi_cfg)) {
		return -EINVAL;
	}

	if (dev_data->busy) {
		return -EBUSY;
	}

	qspi_context_unlock_unconditionally(&dev_data->ctx);

	return 0;
}


static int init_qspi(struct device *dev)
{
	/* This sets only default values of frequency, mode and bit order.
	 * The proper ones are set in configure() when a transfer is started.
	 */
	nrfx_err_t result = nrfx_qspi_init(&get_dev_config(dev)->config, qspi_handler, dev);
	if (result != NRFX_SUCCESS) {
		LOG_ERR("Failed to initialize device: %s",
			    dev->config->name);
		return -EBUSY;
	}

//#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
//	get_dev_data(dev)->pm_state = DEVICE_PM_ACTIVE_STATE;
//#endif
	qspi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);

	return 0;
}

//----------------------------------------------------------------------------------------------------------------------------
/**
  * @brief Updates StatusRegister of ext flash memory by sending "Quad Enable" (0x40).
  *
  * Use this command to enable QUAD transfer.
  *
  * @param None
  * @retval An ErrorStatus enumeration value:
  *          - NRFX_SUCCESS            If the operation was successful.
  *          - NRFX_ERROR_TIMEOUT      If the external memory is busy or there are connection issues.
  *          - NRFX_ERROR_BUSY         If the driver currently handles other operation.
  */
static int qspi_enable_quad_transfer(void){
	uint32_t cmd = QSPI_STD_CMD_QE;
	nrf_qspi_cinstr_conf_t cinstr_cfg = {
		.opcode = QSPI_STD_CMD_WRSR,
		.length = NRF_QSPI_CINSTR_LEN_2B,
		.io2_level = true,
		.io3_level = true,
		.wipwait = true,
		.wren = true
	};

	return nrfx_qspi_cinstr_xfer(&cinstr_cfg, &cmd, NULL);
}

//----------------------------------------------------------------------------------------------------------------------------

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
static int qspi_nrfx_pm_control(struct device *dev, u32_t ctrl_command,
				void *context, device_pm_cb cb, void *arg)
{
//	int ret = 0;
//
//	if (ctrl_command == DEVICE_PM_SET_POWER_STATE) {
//		u32_t new_state = *((const u32_t *)context);
//
//		if (new_state != get_dev_data(dev)->pm_state) {
//			switch (new_state) {
//			case DEVICE_PM_ACTIVE_STATE:
//				init_qspi(dev);
//				/* Force reconfiguration before next transfer */
//				get_dev_data(dev)->ctx.config = NULL;
//				break;
//
//			case DEVICE_PM_LOW_POWER_STATE:
//			case DEVICE_PM_SUSPEND_STATE:
//			case DEVICE_PM_OFF_STATE:
//				nrfx_qspi_uninit(&get_dev_config(dev)->qspi);
//				break;
//
//			default:
//				ret = -ENOTSUP;
//			}
//			if (!ret) {
//				get_dev_data(dev)->pm_state = new_state;
//			}
//		}
//	} else {
//		assert(ctrl_command == DEVICE_PM_GET_POWER_STATE);
//		*((u32_t *)context) = get_dev_data(dev)->pm_state;
//	}
//
//	if (cb) {
//		cb(dev, ret, context, arg);
//	}
//
//	return ret;
}
#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */


#define QSPI_NRFX_QSPI_DEVICE(void)					       \
	static int qspi_init(struct device *dev)			       \
	{								       \
		IRQ_CONNECT(DT_NORDIC_NRF_QSPI_QSPI_0_IRQ_0,		       \
			    DT_NORDIC_NRF_QSPI_QSPI_0_IRQ_0_PRIORITY,      \
			    nrfx_isr, nrfx_qspi_irq_handler, 0);	       \
	return init_qspi(dev);					       \
	}								       \
	static struct qspi_nrfx_data qspi_data = {		       \
		QSPI_CONTEXT_INIT_LOCK(qspi_data, ctx),		       \
		QSPI_CONTEXT_INIT_SYNC(qspi_data, ctx),		       \
		.busy = false,						       \
	};								       \
	static const struct qspi_nrfx_config qspiz_config = {	       \
		.config = {						       \
			.xip_offset  = NRFX_QSPI_CONFIG_XIP_OFFSET,                         \
			.pins = {                                                           \
			   .sck_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_SCK_PIN,                                \
			   .csn_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_CSN_PIN,                                \
			   .io0_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_IO00_PIN,                                \
			   .io1_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_IO01_PIN,                                \
			   .io2_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_IO02_PIN,                                \
			   .io3_pin     = DT_NORDIC_NRF_QSPI_QSPI_0_IO03_PIN,                                \
			},                                                                  \
			.irq_priority   = (uint8_t)NRFX_QSPI_CONFIG_IRQ_PRIORITY,           \
			.prot_if = {                                                        \
				.readoc     = (nrf_qspi_readoc_t)NRFX_QSPI_CONFIG_READOC,       \
				.writeoc    = (nrf_qspi_writeoc_t)NRFX_QSPI_CONFIG_WRITEOC,     \
				.addrmode   = (nrf_qspi_addrmode_t)NRF_QSPI_ADDRMODE_24BIT,   \
				.dpmconfig  = false,                                            \
			},                                                                  \
			.phy_if = {                                                         \
				.sck_freq   = (nrf_qspi_frequency_t)NRFX_QSPI_CONFIG_FREQUENCY, \
				.sck_delay  = (uint8_t)NRFX_QSPI_CONFIG_SCK_DELAY,              \
				.spi_mode   = (nrf_qspi_spi_mode_t)NRFX_QSPI_CONFIG_MODE,       \
				.dpmen      = false                                             \
			},                                                                  \
		}							       \
	};								       \
	DEVICE_DEFINE(qspi, DT_NORDIC_NRF_QSPI_QSPI_0_LABEL,	       \
		      qspi_init,					       \
		      qspi_nrfx_pm_control,				       \
		      &qspi_data,				       \
		      &qspiz_config,				       \
		      POST_KERNEL, CONFIG_QSPI_INIT_PRIORITY,		       \
		      &qspi_nrfx_driver_api)

#ifdef CONFIG_QSPI_NRF_QSPI
QSPI_NRFX_QSPI_DEVICE();
#endif
