/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for QSPI drivers and applications
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_QSPI_H_
#define ZEPHYR_INCLUDE_DRIVERS_QSPI_H_

/**
 * @brief QSPI Interface
 * @defgroup qspi_interface QSPI Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief QSPI operational mode
 */
#define QSPI_OP_MODE_MASTER	0
#define QSPI_OP_MODE_SLAVE	BIT(0)
#define QSPI_OP_MODE_MASK	0x1
#define QSPI_OP_MODE_GET(_operation_) ((_operation_) & QSPI_OP_MODE_MASK)

/**
 * @brief QSPI Polarity & Phase Modes
 */

/**
 * Clock Polarity: if set, clock idle state will be 1
 * and active state will be 0. If untouched, the inverse will be true
 * which is the default.
 */
#define QSPI_MODE_CPOL		BIT(1)

/**
 * Clock Phase: this dictates when is the data captured, and depends
 * clock's polarity. When QSPI_MODE_CPOL is set and this bit as well,
 * capture will occur on low to high transition and high to low if
 * this bit is not set (default). This is fully reversed if CPOL is
 * not set.
 */
#define QSPI_MODE_CPHA		BIT(2)

/**
 * Whatever data is transmitted is looped-back to the receiving buffer of
 * the controller. This is fully controller dependent as some may not
 * support this, and can be used for testing purposes only.
 */
#define QSPI_MODE_LOOP		BIT(3)

#define QSPI_MODE_MASK		(0xE)
#define QSPI_MODE_GET(_mode_)			\
	((_mode_) & QSPI_MODE_MASK)

/**
 * @brief QSPI Transfer modes (host controller dependent)
 */
#define QSPI_TRANSFER_MSB	(0)
#define QSPI_TRANSFER_LSB	BIT(4)

/**
 * @brief QSPI word size
 */
#define QSPI_WORD_SIZE_SHIFT	(5)
#define QSPI_WORD_SIZE_MASK	(0x3F << QSPI_WORD_SIZE_SHIFT)
#define QSPI_WORD_SIZE_GET(_operation_)					\
	(((_operation_) & QSPI_WORD_SIZE_MASK) >> QSPI_WORD_SIZE_SHIFT)

#define QSPI_WORD_SET(_word_size_)		\
	((_word_size_) << QSPI_WORD_SIZE_SHIFT)

/**
 * @brief QSPI MISO lines
 *
 * Some controllers support dual, quad or octal MISO lines connected to slaves.
 * Default is single, which is the case most of the time.
 */
#define QSPI_LINES_SINGLE	(0 << 11)
#define QSPI_LINES_DUAL		(1 << 11)
#define QSPI_LINES_QUAD		(2 << 11)
#define QSPI_LINES_OCTAL		(3 << 11)

#define QSPI_LINES_MASK		(0x3 << 11)

/**
 * @brief Specific QSPI devices control bits
 */
/* Requests - if possible - to keep CS asserted after the transaction */
#define QSPI_HOLD_ON_CS		BIT(13)
/* Keep the device locked after the transaction for the current config.
 * Use this with extreme caution (see qspi_release() below) as it will
 * prevent other callers to access the QSPI device until qspi_release() is
 * properly called.
 */
#define QSPI_LOCK_ON		BIT(14)

/* Active high logic on CS - Usually, and by default, CS logic is active
 * low. However, some devices may require the reverse logic: active high.
 * This bit will request the controller to use that logic. Note that not
 * all controllers are able to handle that natively. In this case deferring
 * the CS control to a gpio line through struct qspi_cs_control would be
 * the solution.
 */
#define QSPI_CS_ACTIVE_HIGH	BIT(15)

/**
 * @brief QSPI Chip Select control structure
 *
 * This can be used to control a CS line via a GPIO line, instead of
 * using the controller inner CS logic.
 *
 * @param gpio_dev is a valid pointer to an actual GPIO device. A NULL pointer
 *        can be provided to full inhibit CS control if necessary.
 * @param gpio_pin is a number representing the gpio PIN that will be used
 *    to act as a CS line
 * @param delay is a delay in microseconds to wait before starting the
 *    transmission and before releasing the CS line
 */
struct qspi_cs_control {
	struct device	*gpio_dev;
	u32_t		gpio_pin;
	u32_t		delay;
};

/**
 * @brief QSPI controller configuration structure
 *
 * @param frequency is the bus frequency in Hertz
 * @param operation is a bit field with the following parts:
 *
 *     operational mode    [ 0 ]       - master or slave.
 *     mode                [ 1 : 3 ]   - Polarity, phase and loop mode.
 *     transfer            [ 4 ]       - LSB or MSB first.
 *     word_size           [ 5 : 10 ]  - Size of a data frame in bits.
 *     lines               [ 11 : 12 ] - MISO lines: Single/Dual/Quad/Octal.
 *     cs_hold             [ 13 ]      - Hold on the CS line if possible.
 *     lock_on             [ 14 ]      - Keep resource locked for the caller.
 *     cs_active_high      [ 15 ]      - Active high CS logic.
 * @param slave is the slave number from 0 to host controller slave limit.
 * @param cs is a valid pointer on a struct qspi_cs_control is CS line is
 *    emulated through a gpio line, or NULL otherwise.
 *
 * @note Only cs_hold and lock_on can be changed between consecutive
 * transceive call. Rest of the attributes are not meant to be tweaked.
 */
struct qspi_config {
	u32_t		frequency;
	u16_t		operation;
	u16_t		slave;

	const struct qspi_cs_control *cs;
};

/**
 * @brief QSPI buffer structure
 *
 * @param buf is a valid pointer on a data buffer, or NULL otherwise.
 * @param len is the length of the buffer or, if buf is NULL, will be the
 *    length which as to be sent as dummy bytes (as TX buffer) or
 *    the length of bytes that should be skipped (as RX buffer).
 */
struct qspi_buf {
	void *buf;
	size_t len;
};

/**
 * @brief QSPI buffer array structure
 *
 * @param buffers is a valid pointer on an array of qspi_buf, or NULL.
 * @param count is the length of the array pointed by buffers.
 */
struct qspi_buf_set {
	const struct qspi_buf *buffers;
	size_t count;
};

/**
 * @typedef qspi_api_io
 * @brief Callback API for I/O
 * See qspi_transceive() for argument descriptions
 */
typedef int (*qspi_api_io)(struct device *dev,
			  const struct qspi_config *config,
			  const struct qspi_buf_set *tx_bufs,
			  const struct qspi_buf_set *rx_bufs);

/**
 * @typedef qspi_api_io
 * @brief Callback API for asynchronous I/O
 * See qspi_transceive_async() for argument descriptions
 */
typedef int (*qspi_api_io_async)(struct device *dev,
				const struct qspi_config *config,
				const struct qspi_buf_set *tx_bufs,
				const struct qspi_buf_set *rx_bufs,
				struct k_poll_signal *async);

/**
 * @typedef qspi_api_release
 * @brief Callback API for unlocking QSPI device.
 * See qspi_release() for argument descriptions
 */
typedef int (*qspi_api_release)(struct device *dev,
			       const struct qspi_config *config);


/**
 * @brief QSPI driver API
 * This is the mandatory API any QSPI driver needs to expose.
 */
struct qspi_driver_api {
	qspi_api_io transceive;
#ifdef CONFIG_QSPI_ASYNC
	qspi_api_io_async transceive_async;
#endif /* CONFIG_QSPI_ASYNC */
	qspi_api_release release;
};

/**
 * @brief Read/write the specified amount of data from the QSPI driver.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid qspi_config structure instance.
 * @param tx_bufs Buffer array where data to be sent originates from,
 *        or NULL if none.
 * @param rx_bufs Buffer array where data to be read will be written to,
 *        or NULL if none.
 *
 * @retval 0 If successful, negative errno code otherwise. In case of slave
 *         transaction: if successful it will return the amount of frames
 *         received, negative errno code otherwise.
 */
__syscall int qspi_transceive(struct device *dev,
			     const struct qspi_config *config,
			     const struct qspi_buf_set *tx_bufs,
			     const struct qspi_buf_set *rx_bufs);

static inline int z_impl_qspi_transceive(struct device *dev,
				       const struct qspi_config *config,
				       const struct qspi_buf_set *tx_bufs,
				       const struct qspi_buf_set *rx_bufs)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->transceive(dev, config, tx_bufs, rx_bufs);
}

/**
 * @brief Read the specified amount of data from the QSPI driver.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid qspi_config structure instance.
 * @param rx_bufs Buffer array where data to be read will be written to.
 *
 * @retval 0 If successful, negative errno code otherwise.
 *
 * @note This function is an helper function calling qspi_transceive.
 */
static inline int qspi_read(struct device *dev,
			   const struct qspi_config *config,
			   const struct qspi_buf_set *rx_bufs)
{
	return qspi_transceive(dev, config, NULL, rx_bufs);
}

/**
 * @brief Write the specified amount of data from the QSPI driver.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid qspi_config structure instance.
 * @param tx_bufs Buffer array where data to be sent originates from.
 *
 * @retval 0 If successful, negative errno code otherwise.
 *
 * @note This function is an helper function calling qspi_transceive.
 */
static inline int qspi_write(struct device *dev,
			    const struct qspi_config *config,
			    const struct qspi_buf_set *tx_bufs)
{
	return qspi_transceive(dev, config, tx_bufs, NULL);
}

#ifdef CONFIG_QSPI_ASYNC
/**
 * @brief Read/write the specified amount of data from the QSPI driver.
 *
 * Note: This function is asynchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid qspi_config structure instance.
 * @param tx_bufs Buffer array where data to be sent originates from,
 *        or NULL if none.
 * @param rx_bufs Buffer array where data to be read will be written to,
 *        or NULL if none.
 * @param async A pointer to a valid and ready to be signaled
 *        struct k_poll_signal. (Note: if NULL this function will not
 *        notify the end of the transaction, and whether it went
 *        successfully or not).
 *
 * @retval 0 If successful, negative errno code otherwise. In case of slave
 *         transaction: if successful it will return the amount of frames
 *         received, negative errno code otherwise.
 */
static inline int qspi_transceive_async(struct device *dev,
				       const struct qspi_config *config,
				       const struct qspi_buf_set *tx_bufs,
				       const struct qspi_buf_set *rx_bufs,
				       struct k_poll_signal *async)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->transceive_async(dev, config, tx_bufs, rx_bufs, async);
}

/**
 * @brief Read the specified amount of data from the QSPI driver.
 *
 * Note: This function is asynchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid qspi_config structure instance.
 * @param rx_bufs Buffer array where data to be read will be written to.
 * @param async A pointer to a valid and ready to be signaled
 *        struct k_poll_signal. (Note: if NULL this function will not
 *        notify the end of the transaction, and whether it went
 *        successfully or not).
 *
 * @retval 0 If successful, negative errno code otherwise.
 *
 * @note This function is an helper function calling qspi_transceive_async.
 */
static inline int qspi_read_async(struct device *dev,
				 const struct qspi_config *config,
				 const struct qspi_buf_set *rx_bufs,
				 struct k_poll_signal *async)
{
	return qspi_transceive_async(dev, config, NULL, rx_bufs, async);
}

/**
 * @brief Write the specified amount of data from the QSPI driver.
 *
 * Note: This function is asynchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid qspi_config structure instance.
 * @param tx_bufs Buffer array where data to be sent originates from.
 * @param async A pointer to a valid and ready to be signaled
 *        struct k_poll_signal. (Note: if NULL this function will not
 *        notify the end of the transaction, and whether it went
 *        successfully or not).
 *
 * @retval 0 If successful, negative errno code otherwise.
 *
 * @note This function is an helper function calling qspi_transceive_async.
 */
static inline int qspi_write_async(struct device *dev,
				  const struct qspi_config *config,
				  const struct qspi_buf_set *tx_bufs,
				  struct k_poll_signal *async)
{
	return qspi_transceive_async(dev, config, tx_bufs, NULL, async);
}
#endif /* CONFIG_QSPI_ASYNC */

/**
 * @brief Release the QSPI device locked on by the current config
 *
 * Note: This synchronous function is used to release the lock on the QSPI
 *       device that was kept if, and if only, given config parameter was
 *       the last one to be used (in any of the above functions) and if
 *       it has the QSPI_LOCK_ON bit set into its operation bits field.
 *       This can be used if the caller needs to keep its hand on the QSPI
 *       device for consecutive transactions.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid qspi_config structure instance.
 */
__syscall int qspi_release(struct device *dev,
			  const struct qspi_config *config);

static inline int z_impl_qspi_release(struct device *dev,
				    const struct qspi_config *config)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->release(dev, config);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/qspi.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_QSPI_H_ */
