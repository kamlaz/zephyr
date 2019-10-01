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

#define QSPI_MODE_MASK		(0x3)
#define QSPI_MODE_GET(_mode_)			\
	((_mode_) & QSPI_MODE_MASK)
/**
 * @brief QSPI CS delay.
 */

/**
 * No of clock cycles when CS must remain high between two read/write
 * operations
 */
#define QSPI_CS_DELAY_FIELD_SIZE	0xFF
#define QSPI_CS_DELAY_SHIFT	(2)
#define QSPI_CS_DELAY_MASK	(QSPI_CS_DELAY_FIELD_SIZE << QSPI_CS_DELAY_SHIFT)
#define QSPI_CS_DELAY_GET(_operation_)					\
	(((_operation_) & QSPI_CS_DELAY_MASK) >> QSPI_CS_DELAY_SHIFT)

#define QSPI_CS_DELAY_SET(_cs_delay_)		\
	((_cs_delay_) << QSPI_CS_DELAY_SHIFT)

/**
 * @brief QSPI Read data lines
 */

/**
 * No of clock cycles when CS must remain high between two read/write
 * operations
 * Dual data line SPI. READ2O (opcode 0x3B).
 * Dual data line SPI. READ2IO (opcode 0xBB).
 * Quad data line SPI. READ4O (opcode 0x6B).
 * Quad data line SPI. READ4IO (opcode 0xEB).
 */
#define QSPI_DATA_LINES_SINGLE	0x00
#define QSPI_DATA_LINES_DOUBLE	0x01
#define QSPI_DATA_LINES_QUAD	0x02

#define QSPI_DATA_LINES_FIELD_SIZE	0x03
#define QSPI_DATA_LINES_SHIFT	(10)
#define QSPI_DATA_LINES_MASK	(QSPI_DATA_LINES_FIELD_SIZE << QSPI_DATA_LINES_SHIFT)
#define QSPI_DATA_LINES_GET(_operation_)					\
	(((_operation_) & QSPI_DATA_LINES_MASK) >> QSPI_DATA_LINES_SHIFT)

#define QSPI_DATA_LINES_SET(_read_data_lines_)		\
	((_read_data_lines_) << QSPI_DATA_LINES_SHIFT)


/**
 * @brief QSPI Address configuration
 */

/**
 * Length of the address field. Typical flash chips support 24bit address mode
 * 0x00	-	8 bit address
 * 0x01	-	16 bit address
 * 0x02	-	24 bit address
 * 0x03	-	32 bit address
 */
#define QSPI_ADDRESS_MODE_8BIT	0x00
#define QSPI_ADDRESS_MODE_16BIT	0x01
#define QSPI_ADDRESS_MODE_24BIT	0x02
#define QSPI_ADDRESS_MODE_32BIT	0x03

#define QSPI_ADDRESS_MODE_FIELD_SIZE	0x03
#define QSPI_ADDRESS_MODE_SHIFT	(12)
#define QSPI_ADDRESS_MODE_MASK	(QSPI_ADDRESS_MODE_FIELD_SIZE << QSPI_ADDRESS_MODE_SHIFT)
#define QSPI_ADDRESS_MODE_GET(_operation_)					\
	(((_operation_) & QSPI_ADDRESS_MODE_MASK) >> QSPI_ADDRESS_MODE_SHIFT)

#define QSPI_ADDRESS_MODE_SET(_address_)		\
	((_address_) << QSPI_ADDRESS_MODE_SHIFT)

/**
 * @brief QSPI RFU bits
 */

/**
 * Not used. Write/read does not impact functionality of the driver.
 */
#define QSPI_RFU_14		BIT(14)
#define QSPI_RFU_15		BIT(15)

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
 *     mode		   		   [ 0 : 1 ]   - Polarity, phase
 *     cs_delay            [ 2 : 9 ]   - CS delay. No of clock cycles when CS must remain high between two read/write operations
 *     data_lines 	       [ 10 : 11 ] - Defines how many lines will be used for read operation
 *     address             [ 12 : 13 ] - Defines how many bits are used for address
 *     RFU	               [ 14 : 15 ] - RFU
 * @param cs is a valid pointer on a struct qspi_cs_control is CS line is
 *    emulated through a gpio line, or NULL otherwise.
 *
 * @note Only cs_hold and lock_on can be changed between consecutive
 * transceive call. Rest of the attributes are not meant to be tweaked.
 */
struct qspi_config {
	u32_t		frequency;
	u16_t		operation;
	const struct qspi_cs_control *cs;
};

/**
 * @typedef qspi_api_io
 * @brief Callback API for I/O
 * See qspi_transceive() for argument descriptions
 */
typedef int (*qspi_api_io)(struct device *dev,
			  const struct qspi_config *config,
			  const void *tx_buf,
			  size_t tx_len,
			  const void *rx_buf,
			  size_t rx_len,
			  u32_t op_code,
			  u32_t address);

/**
 * @typedef qspi_api_write
 * @brief Callback API for I/O
 * See qspi_write() for argument descriptions
 */
typedef int (*qspi_api_write)(struct device *dev,
			  const struct qspi_config *config,
			  const void * tx_buf,
			  size_t len,
			  uint32_t address);


/**
 * @typedef qspi_api_read
 * @brief Callback API for I/O
 * See qspi_read() for argument descriptions
 */
typedef int (*qspi_api_read)(struct device *dev,
			  const struct qspi_config *config,
			  const void * rx_buf,
			  size_t len,
			  uint32_t address);


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
 *
 * @param transceive - us
 */
struct qspi_driver_api {
	qspi_api_io 	cmd_xfer;
	qspi_api_write 	write;
	qspi_api_read 	read;
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
 * @param tx_buf Buffer where data to be sent originates from,
 *        or NULL if none.
 * @param tx_len Amount of data to be transmitted from tx_buf. Value in bytes,
 *        or 0 if none.
 *
 * @param rx_bufs Buffer where data to be read will be written to,
 *        or NULL if none.
 * @param rx_len Amount of data to be received to rx_buf. Value in bytes,
 *        or 0 if none.
 *
 * @param op_code Operation code to be sent. It is defined in the flash manufacture's datasheet. One byte value,
 *        or 0 if none.
 *
 * @param address Addres where the operation will take place (i.e erase). Four byte value,
 *        or (-1) if none.
 *
 * @retval 0 If successful, negative errno code otherwise. In case of slave
 *         transaction: if successful it will return the amount of frames
 *         received, negative errno code otherwise.
 */
__syscall int qspi_cmd_xfer(struct device *dev,
					const struct qspi_config *config,
					const void *tx_buf,
					size_t tx_len,
					const void *rx_buf,
					size_t rx_len,
					u32_t op_code,
					u32_t address);

static inline int z_impl_qspi_cmd_xfer(struct device *dev,
					const struct qspi_config *config,
					const void *tx_buf,
					size_t tx_len,
					const void *rx_buf,
					size_t rx_len,
					u32_t op_code,
					u32_t address)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->cmd_xfer(dev, config, tx_buf, tx_len, rx_buf, rx_len, op_code, address);
}

/**
 * @brief Read the specified amount of data from the QSPI driver.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid qspi_config structure instance.
 * @param rx_bufs Amount of data to be received to rx_buf.
 * @param len Amount of data to be received to rx_buf.
 * @param address Addres where the operation will take place (i.e erase). Four byte value,
 *        or (-1) if none.
 *
 * @retval 0 If successful, negative errno code otherwise.
 *
 * @note This function is an helper function calling qspi_transceive.
 */
__syscall int qspi_read(struct device *dev,
				const struct qspi_config *config,
				const void * rx_buf,
				size_t len,
				uint32_t address);

static inline int z_impl_qspi_read(struct device *dev,
				const struct qspi_config *config,
				const void * rx_buf,
				size_t len,
				uint32_t address)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->read(dev, config, rx_buf, len, address);
}


/**
 * @brief Write the specified amount of data from the QSPI driver.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid qspi_config structure instance.
 * @param tx_buf Buffer where data to be sent originates from.
 * @param address Address where the data will be written. Four byte value,
 *        or (-1) if none.
 *
 * @retval 0 If successful, negative errno code otherwise.
 *
 * @note This function is an helper function calling qspi_transceive.
 */
__syscall int qspi_write(struct device *dev,
				const struct qspi_config *config,
				const void * tx_buf,
				size_t len,
				uint32_t address);

static inline int z_impl_qspi_write(struct device *dev,
				const struct qspi_config *config,
				const void * tx_buf,
				size_t len,
				uint32_t address)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->write(dev, config, tx_buf, len, address);
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
