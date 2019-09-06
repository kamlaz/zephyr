/*
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/qspi.h>
#include <nrfx_qspim.h>
#include <string.h>

#define LOG_DOMAIN "qspi_nrfx_qspim"
#define LOG_LEVEL CONFIG_QSPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(qspi_nrfx_qspim);

#include "qspi_context.h"

struct qspi_nrfx_data {
	struct qspi_context ctx;
	size_t chunk_len;
	bool   busy;
#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	u32_t pm_state;
#endif
#if (CONFIG_QSPI_NRFX_RAM_BUFFER_SIZE > 0)
	u8_t   buffer[CONFIG_QSPI_NRFX_RAM_BUFFER_SIZE];
#endif
};

struct qspi_nrfx_config {
	nrfx_qspim_t	   qspim;
	size_t		   max_chunk_len;
	nrfx_qspim_config_t config;
};

static inline struct qspi_nrfx_data *get_dev_data(struct device *dev)
{
	return dev->driver_data;
}

static inline const struct qspi_nrfx_config *get_dev_config(struct device *dev)
{
	return dev->config->config_info;
}

static inline nrf_qspim_frequency_t get_nrf_qspim_frequency(u32_t frequency)
{
	/* Get the highest supported frequency not exceeding the requested one.
	 */
	if (frequency < 250000) {
		return NRF_QSPIM_FREQ_125K;
	} else if (frequency < 500000) {
		return NRF_QSPIM_FREQ_250K;
	} else if (frequency < 1000000) {
		return NRF_QSPIM_FREQ_500K;
	} else if (frequency < 2000000) {
		return NRF_QSPIM_FREQ_1M;
	} else if (frequency < 4000000) {
		return NRF_QSPIM_FREQ_2M;
	} else if (frequency < 8000000) {
		return NRF_QSPIM_FREQ_4M;
#ifdef CONFIG_SOC_NRF52840
	} else if (frequency < 16000000) {
		return NRF_QSPIM_FREQ_8M;
	} else if (frequency < 32000000) {
		return NRF_QSPIM_FREQ_16M;
	} else {
		return NRF_QSPIM_FREQ_32M;
#else
	} else {
		return NRF_QSPIM_FREQ_8M;
#endif
	}
}

static inline nrf_qspim_mode_t get_nrf_qspim_mode(u16_t operation)
{
	if (QSPI_MODE_GET(operation) & QSPI_MODE_CPOL) {
		if (QSPI_MODE_GET(operation) & QSPI_MODE_CPHA) {
			return NRF_QSPIM_MODE_3;
		} else {
			return NRF_QSPIM_MODE_2;
		}
	} else {
		if (QSPI_MODE_GET(operation) & QSPI_MODE_CPHA) {
			return NRF_QSPIM_MODE_1;
		} else {
			return NRF_QSPIM_MODE_0;
		}
	}
}

static inline nrf_qspim_bit_order_t get_nrf_qspim_bit_order(u16_t operation)
{
	if (operation & QSPI_TRANSFER_LSB) {
		return NRF_QSPIM_BIT_ORDER_LSB_FIRST;
	} else {
		return NRF_QSPIM_BIT_ORDER_MSB_FIRST;
	}
}

static int configure(struct device *dev,
		     const struct qspi_config *qspi_cfg)
{
	struct qspi_context *ctx = &get_dev_data(dev)->ctx;
	const nrfx_qspim_t *qspim = &get_dev_config(dev)->qspim;

	if (qspi_context_configured(ctx, qspi_cfg)) {
		/* Already configured. No need to do it again. */
		return 0;
	}

	if (QSPI_OP_MODE_GET(qspi_cfg->operation) != QSPI_OP_MODE_MASTER) {
		LOG_ERR("Slave mode is not supported on %s",
			    dev->config->name);
		return -EINVAL;
	}

	if (qspi_cfg->operation & QSPI_MODE_LOOP) {
		LOG_ERR("Loopback mode is not supported");
		return -EINVAL;
	}

	if ((qspi_cfg->operation & QSPI_LINES_MASK) != QSPI_LINES_SINGLE) {
		LOG_ERR("Only single line mode is supported");
		return -EINVAL;
	}

	if (QSPI_WORD_SIZE_GET(qspi_cfg->operation) != 8) {
		LOG_ERR("Word sizes other than 8 bits"
			    " are not supported");
		return -EINVAL;
	}

	if (qspi_cfg->frequency < 125000) {
		LOG_ERR("Frequencies lower than 125 kHz are not supported");
		return -EINVAL;
	}

	ctx->config = qspi_cfg;
	qspi_context_cs_configure(ctx);

	nrf_qspim_configure(qspim->p_reg,
			   get_nrf_qspim_mode(qspi_cfg->operation),
			   get_nrf_qspim_bit_order(qspi_cfg->operation));
	nrf_qspim_frequency_set(qspim->p_reg,
			       get_nrf_qspim_frequency(qspi_cfg->frequency));

	return 0;
}

static void transfer_next_chunk(struct device *dev)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
	const struct qspi_nrfx_config *dev_config = get_dev_config(dev);
	struct qspi_context *ctx = &dev_data->ctx;
	int error = 0;

	size_t chunk_len = qspi_context_longest_current_buf(ctx);

	if (chunk_len > 0) {
		nrfx_qspim_xfer_desc_t xfer;
		nrfx_err_t result;
		const u8_t *tx_buf = ctx->tx_buf;
#if (CONFIG_QSPI_NRFX_RAM_BUFFER_SIZE > 0)
		if (qspi_context_tx_buf_on(ctx) && !nrfx_is_in_ram(tx_buf)) {
			if (chunk_len > sizeof(dev_data->buffer)) {
				chunk_len = sizeof(dev_data->buffer);
			}

			memcpy(dev_data->buffer, tx_buf, chunk_len);
			tx_buf = dev_data->buffer;
		}
#endif
		if (chunk_len > dev_config->max_chunk_len) {
			chunk_len = dev_config->max_chunk_len;
		}

		dev_data->chunk_len = chunk_len;

		xfer.p_tx_buffer = tx_buf;
		xfer.tx_length   = qspi_context_tx_buf_on(ctx) ? chunk_len : 0;
		xfer.p_rx_buffer = ctx->rx_buf;
		xfer.rx_length   = qspi_context_rx_buf_on(ctx) ? chunk_len : 0;

		/* This QSPIM driver is only used by the NRF52832 if
		   SOC_NRF52832_ALLOW_QSPIM_DEQSPITE_PAN_58 is enabled */
		if (IS_ENABLED(CONFIG_SOC_NRF52832) &&
		   (xfer.rx_length == 1 && xfer.tx_length <= 1)) {
			LOG_WRN("Transaction aborted since it would trigger nRF52832 PAN 58");
			error = -EIO;
		}

		if (!error) {
			result = nrfx_qspim_xfer(&dev_config->qspim, &xfer, 0);
			if (result == NRFX_SUCCESS) {
				return;
			}
			error = -EIO;
		}
	}

	qspi_context_cs_control(ctx, false);

	LOG_DBG("Transaction finished with status %d", error);

	qspi_context_complete(ctx, error);
	dev_data->busy = false;
}

static int transceive(struct device *dev,
		      const struct qspi_config *qspi_cfg,
		      const struct qspi_buf_set *tx_bufs,
		      const struct qspi_buf_set *rx_bufs)
{
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);
	int error;

	error = configure(dev, qspi_cfg);
	if (error == 0) {
		dev_data->busy = true;

		qspi_context_buffers_setup(&dev_data->ctx, tx_bufs, rx_bufs, 1);
		qspi_context_cs_control(&dev_data->ctx, true);

		transfer_next_chunk(dev);

		error = qspi_context_wait_for_completion(&dev_data->ctx);
	}

	qspi_context_release(&dev_data->ctx, error);

	return error;
}

static int qspi_nrfx_transceive(struct device *dev,
			       const struct qspi_config *qspi_cfg,
			       const struct qspi_buf_set *tx_bufs,
			       const struct qspi_buf_set *rx_bufs)
{
	qspi_context_lock(&get_dev_data(dev)->ctx, false, NULL);
	return transceive(dev, qspi_cfg, tx_bufs, rx_bufs);
}

#ifdef CONFIG_QSPI_ASYNC
static int qspi_nrfx_transceive_async(struct device *dev,
				     const struct qspi_config *qspi_cfg,
				     const struct qspi_buf_set *tx_bufs,
				     const struct qspi_buf_set *rx_bufs,
				     struct k_poll_signal *async)
{
	qspi_context_lock(&get_dev_data(dev)->ctx, true, async);
	return transceive(dev, qspi_cfg, tx_bufs, rx_bufs);
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

static const struct qspi_driver_api qspi_nrfx_driver_api = {
	.transceive = qspi_nrfx_transceive,
#ifdef CONFIG_QSPI_ASYNC
	.transceive_async = qspi_nrfx_transceive_async,
#endif
	.release = qspi_nrfx_release,
};


static void event_handler(const nrfx_qspim_evt_t *p_event, void *p_context)
{
	struct device *dev = p_context;
	struct qspi_nrfx_data *dev_data = get_dev_data(dev);

	if (p_event->type == NRFX_QSPIM_EVENT_DONE) {
		qspi_context_update_tx(&dev_data->ctx, 1, dev_data->chunk_len);
		qspi_context_update_rx(&dev_data->ctx, 1, dev_data->chunk_len);

		transfer_next_chunk(dev);
	}
}

static int init_qspim(struct device *dev)
{
	/* This sets only default values of frequency, mode and bit order.
	 * The proper ones are set in configure() when a transfer is started.
	 */
	nrfx_err_t result = nrfx_qspim_init(&get_dev_config(dev)->qspim,
					   &get_dev_config(dev)->config,
					   event_handler,
					   dev);
	if (result != NRFX_SUCCESS) {
		LOG_ERR("Failed to initialize device: %s",
			    dev->config->name);
		return -EBUSY;
	}

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	get_dev_data(dev)->pm_state = DEVICE_PM_ACTIVE_STATE;
#endif
	qspi_context_unlock_unconditionally(&get_dev_data(dev)->ctx);

	return 0;
}

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
static int qspim_nrfx_pm_control(struct device *dev, u32_t ctrl_command,
				void *context, device_pm_cb cb, void *arg)
{
	int ret = 0;

	if (ctrl_command == DEVICE_PM_SET_POWER_STATE) {
		u32_t new_state = *((const u32_t *)context);

		if (new_state != get_dev_data(dev)->pm_state) {
			switch (new_state) {
			case DEVICE_PM_ACTIVE_STATE:
				init_qspim(dev);
				/* Force reconfiguration before next transfer */
				get_dev_data(dev)->ctx.config = NULL;
				break;

			case DEVICE_PM_LOW_POWER_STATE:
			case DEVICE_PM_SUSPEND_STATE:
			case DEVICE_PM_OFF_STATE:
				nrfx_qspim_uninit(&get_dev_config(dev)->qspim);
				break;

			default:
				ret = -ENOTSUP;
			}
			if (!ret) {
				get_dev_data(dev)->pm_state = new_state;
			}
		}
	} else {
		assert(ctrl_command == DEVICE_PM_GET_POWER_STATE);
		*((u32_t *)context) = get_dev_data(dev)->pm_state;
	}

	if (cb) {
		cb(dev, ret, context, arg);
	}

	return ret;
}
#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */

#if NRFX_CHECK(NRFX_QSPIM_EXTENDED_ENABLED)
#define QSPI_NRFX_QSPIM_EXTENDED_CONFIG(idx) \
	.rx_delay = CONFIG_QSPI_##idx##_NRF_RX_DELAY,
#else
#define QSPI_NRFX_QSPIM_EXTENDED_CONFIG(idx)
#endif

#define QSPI_NRFX_QSPIM_DEVICE(idx)					       \
	static int qspi_##idx##_init(struct device *dev)			       \
	{								       \
		IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_QSPIM##idx),		       \
			    DT_NORDIC_NRF_QSPI_QSPI_##idx##_IRQ_0_PRIORITY,      \
			    nrfx_isr, nrfx_qspim_##idx##_irq_handler, 0);       \
		return init_qspim(dev);					       \
	}								       \
	static struct qspi_nrfx_data qspi_##idx##_data = {		       \
		QSPI_CONTEXT_INIT_LOCK(qspi_##idx##_data, ctx),		       \
		QSPI_CONTEXT_INIT_SYNC(qspi_##idx##_data, ctx),		       \
		.busy = false,						       \
	};								       \
	static const struct qspi_nrfx_config qspi_##idx##z_config = {	       \
		.qspim = NRFX_QSPIM_INSTANCE(idx),			       \
		.max_chunk_len = (1 << QSPIM##idx##_EASYDMA_MAXCNT_SIZE) - 1,   \
		.config = {						       \
			.sck_pin   = DT_NORDIC_NRF_QSPI_QSPI_##idx##_SCK_PIN,    \
			.mosi_pin  = DT_NORDIC_NRF_QSPI_QSPI_##idx##_MOSI_PIN,   \
			.miso_pin  = DT_NORDIC_NRF_QSPI_QSPI_##idx##_MISO_PIN,   \
			.ss_pin    = NRFX_QSPIM_PIN_NOT_USED,		       \
			.orc       = CONFIG_QSPI_##idx##_NRF_ORC,	       \
			.frequency = NRF_QSPIM_FREQ_4M,			       \
			.mode      = NRF_QSPIM_MODE_0,			       \
			.bit_order = NRF_QSPIM_BIT_ORDER_MSB_FIRST,	       \
			QSPI_NRFX_QSPIM_EXTENDED_CONFIG(idx)		       \
		}							       \
	};								       \
	DEVICE_DEFINE(qspi_##idx,					       \
		      DT_NORDIC_NRF_QSPI_QSPI_##idx##_LABEL,		       \
		      qspi_##idx##_init,					       \
		      qspim_nrfx_pm_control,				       \
		      &qspi_##idx##_data,				       \
		      &qspi_##idx##z_config,				       \
		      POST_KERNEL, CONFIG_QSPI_INIT_PRIORITY,		       \
		      &qspi_nrfx_driver_api)

#ifdef CONFIG_QSPI_0_NRF_QSPIM
QSPI_NRFX_QSPIM_DEVICE(0);
#endif

#ifdef CONFIG_QSPI_1_NRF_QSPIM
QSPI_NRFX_QSPIM_DEVICE(1);
#endif

#ifdef CONFIG_QSPI_2_NRF_QSPIM
QSPI_NRFX_QSPIM_DEVICE(2);
#endif

#ifdef CONFIG_QSPI_3_NRF_QSPIM
QSPI_NRFX_QSPIM_DEVICE(3);
#endif
