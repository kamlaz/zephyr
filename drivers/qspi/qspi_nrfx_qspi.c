/*
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/qspi.h>
#include <nrfx_qspi.h>
#include <stdio.h>

#define LOG_DOMAIN "qspi_nrfx_qspi"
#define LOG_LEVEL CONFIG_QSPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(qspi_nrfx_qspi);

#include "qspi_context.h"

#define WAIT_FOR_PERIPH() do { \
        while (!m_finished) {} \
        m_finished = false;    \
    } while (0)


struct qspi_nrfx_data {
	struct qspi_context ctx;
	size_t chunk_len;
	bool   busy;
#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	u32_t pm_state;
#endif
};

struct qspi_nrfx_config {
	nrfx_qspi_config_t config;
};

static volatile bool m_finished = false;

static inline struct qspi_nrfx_data *get_dev_data(struct device *dev)
{
	return dev->driver_data;
}

static inline const struct qspi_nrfx_config *get_dev_config(struct device *dev)
{
	return dev->config->config_info;
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



static inline nrf_qspi_addrmode_t get_nrf_qspi_address_mode(u16_t operation)
{
	if (QSPI_ADDRESS_MODE_GET(operation) & QSPI_ADDRESS_MODE_24BIT) {
		return NRF_QSPI_ADDRMODE_24BIT;
	}
	else if (QSPI_ADDRESS_MODE_GET(operation) & QSPI_ADDRESS_MODE_32BIT){
		return NRF_QSPI_ADDRMODE_32BIT;
	}
	else{
		LOG_ERR("Address modes other than 24 or 32 bits are not supported");
		return -EINVAL;
	}
}

//static inline nrf_qspi_bit_order_t get_nrf_qspi_bit_order(u16_t operation)
//{
////	if (operation & QSPI_TRANSFER_LSB) {
////		return NRF_QSPI_BIT_ORDER_LSB_FIRST;
////	} else {
////		return NRF_QSPI_BIT_ORDER_MSB_FIRST;
////	}
//}

static void qspi_handler(nrfx_qspi_evt_t event, void * p_context)
{
    UNUSED_PARAMETER(event);
    UNUSED_PARAMETER(p_context);
    m_finished = true;
}

static int configure(struct device *dev,
		     const struct qspi_config *qspi_cfg)
{
//	printf("\nConfiguration...");
//	struct qspi_context *ctx = &get_dev_data(dev)->ctx;
//	nrfx_qspi_config_t * p_config = &get_dev_config(dev)->config;
//
////	const nrfx_qspi_t *qspi = &get_dev_config(dev)->qspi;
////
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
//	if (QSPI_ADDRESS_MODE_GET(qspi_cfg->operation) != (QSPI_ADDRESS_MODE_24BIT )
//			|| QSPI_ADDRESS_MODE_GET(qspi_cfg->operation) != (QSPI_ADDRESS_MODE_32BIT)) {
//			LOG_ERR("Address mode not supported.");
//			return -EINVAL;
//	}
//
//	/* Input parameters OK - initialise QSPI */
//	ctx->config = qspi_cfg;
//	qspi_context_cs_configure(ctx);
//
//	nrfx_qspi_config_t config = NRFX_QSPI_DEFAULT_CONFIG;
//
//	if(nrfx_qspi_init(&config, qspi_handler, NULL ) != NRFX_SUCCESS){
//		return 1;
//	}
// err_code = nrf_drv_qspi_init(&config, qspi_handler, NULL);

	/* Sets address value */
//	nrf_qspi_addrconfig_set(qspi->p_reg,
//			get_nrf_qspi_address_mode(qspi_cfg->operation));
//
//	if (qspi_cfg->operation & QSPI_MODE_LOOP) {
//		LOG_ERR("Loopback mode is not supported");
//		return -EINVAL;
//	}
//
//	if ((qspi_cfg->operation & QSPI_LINES_MASK) != QSPI_LINES_SINGLE) {
//		LOG_ERR("Only single line mode is supported");
//		return -EINVAL;
//	}
//
//	if (QSPI_WORD_SIZE_GET(qspi_cfg->operation) != 8) {
//		LOG_ERR("Word sizes other than 8 bits"
//			    " are not supported");
//		return -EINVAL;
//	}
//
//	if (qspi_cfg->frequency < 125000) {
//		LOG_ERR("Frequencies lower than 125 kHz are not supported");
//		return -EINVAL;
//	}
//
//	ctx->config = qspi_cfg;
//	qspi_context_cs_configure(ctx);
//
//	nrf_qspi_configure(qspi->p_reg,
//			  get_nrf_qspi_mode(qspi_cfg->operation),
//			  get_nrf_qspi_bit_order(qspi_cfg->operation));
//	nrf_qspi_frequency_set(qspi->p_reg,
//			      get_nrf_qspi_frequency(qspi_cfg->frequency));
//
	return 0;
}

static void transfer_next_chunk(struct device *dev)
{
//	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
//	struct qspi_context *ctx = &dev_data->ctx;
//	int error = 0;
//
//	size_t chunk_len = qspi_context_longest_current_buf(ctx);
//
//	if (chunk_len > 0) {
//		nrfx_qspi_xfer_desc_t xfer;
//		nrfx_err_t result;
//
//		dev_data->chunk_len = chunk_len;
//
//		xfer.p_tx_buffer = ctx->tx_buf;
//		xfer.tx_length   = qspi_context_tx_buf_on(ctx) ? chunk_len : 0;
//		xfer.p_rx_buffer = ctx->rx_buf;
//		xfer.rx_length   = qspi_context_rx_buf_on(ctx) ? chunk_len : 0;
//		result = nrfx_qspi_xfer(&get_dev_config(dev)->qspi, &xfer, 0);
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
}

static int transceive(struct device *dev,
		      const struct qspi_config *qspi_cfg,
		      const struct qspi_buf_set *tx_bufs,
		      const struct qspi_buf_set *rx_bufs)
{
	printf("\nDUpa");
//	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
//	int error;
//
//	error = configure(dev, qspi_cfg);
//	if (error == 0) {
//		dev_data->busy = true;
//
//		qspi_context_buffers_setup(&dev_data->ctx, tx_bufs, rx_bufs, 1);
//		qspi_context_cs_control(&dev_data->ctx, true);
//
//		transfer_next_chunk(dev);
//
//		error = qspi_context_wait_for_completion(&dev_data->ctx);
//	}
//
//	qspi_context_release(&dev_data->ctx, error);
//
//	return error;
	return 0;
}

static int qspi_nrfx_transceive(struct device *dev,
			       const struct qspi_config *qspi_cfg,
			       const struct qspi_buf_set *tx_bufs,
			       const struct qspi_buf_set *rx_bufs)
{
	printf("CHlosta");
//	qspi_context_lock(&get_dev_data(dev)->ctx, false, NULL);
//	return transceive(dev, qspi_cfg, tx_bufs, rx_bufs);
	return 0;
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
	printf("Release");
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

	return 0;
}

static const struct qspi_driver_api qspi_nrfx_driver_api = {
	.transceive = qspi_nrfx_transceive,
#ifdef CONFIG_QSPI_ASYNC
	.transceive_async = qspi_nrfx_transceive_async,
#endif
	.release = qspi_nrfx_release,
};


static void event_handler(const nrfx_qspi_evt_t *p_event, void *p_context)
{
	struct device *dev = p_context;
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);

	if (*p_event == NRFX_QSPI_EVENT_DONE) {
		qspi_context_update_tx(&dev_data->ctx, 1, dev_data->chunk_len);
		qspi_context_update_rx(&dev_data->ctx, 1, dev_data->chunk_len);

		transfer_next_chunk(dev);
	}
}

static int init_qspi(struct device *dev)
{
	/* This sets only default values of frequency, mode and bit order.
	 * The proper ones are set in configure() when a transfer is started.
	 */
	nrfx_err_t result = nrfx_qspi_init(&get_dev_config(dev)->config, event_handler, dev);
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

//		.qspi = NRFX_QSPI_INSTANCE(0),
//nrfx_qspi_irq_handler
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
			   .sck_pin     = NRFX_QSPI_PIN_SCK,                                \
			   .csn_pin     = NRFX_QSPI_PIN_CSN,                                \
			   .io0_pin     = NRFX_QSPI_PIN_IO0,                                \
			   .io1_pin     = NRFX_QSPI_PIN_IO1,                                \
			   .io2_pin     = NRFX_QSPI_PIN_IO2,                                \
			   .io3_pin     = NRFX_QSPI_PIN_IO3,                                \
			},                                                                  \
			.irq_priority   = (uint8_t)NRFX_QSPI_CONFIG_IRQ_PRIORITY,           \
			.prot_if = {                                                        \
				.readoc     = (nrf_qspi_readoc_t)NRFX_QSPI_CONFIG_READOC,       \
				.writeoc    = (nrf_qspi_writeoc_t)NRFX_QSPI_CONFIG_WRITEOC,     \
				.addrmode   = (nrf_qspi_addrmode_t)NRFX_QSPI_CONFIG_ADDRMODE,   \
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
