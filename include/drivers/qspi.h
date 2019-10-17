/*
 * Copyright (c) 2019, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief Public APIs for QSPI drivers and applications
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_QSPI_H_
#define ZEPHYR_INCLUDE_DRIVERS_QSPI_H_

/**
 * @defgroup qspi_interface QSPI Interface
 * @ingroup io_interfaces
 * @brief QSPI Interface
 *
 * The QSPI API provides support for the standard flash
 * memory devices
 * @{
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The following #defines are used to configure the QSPI controller.
 */

/**
 * No of data lines that are used for the transfer. Some flash memories
 * supports one, two or four lines.
 */
#define QSPI_DATA_LINES_SINGLE			0x00
#define QSPI_DATA_LINES_DOUBLE			0x01
#define QSPI_DATA_LINES_QUAD			0x02

/**
 * SPI mode of the transmission
 * Mode 0: Data are captured on the clock rising edge and
 * 			data is output on a falling edge. Base level of clock is 0.
 * 			(CPOL=0, CPHA=0).
 * Mode 3: Data are captured on the clock falling edge and
 * 			data is output on a rising edge. Base level of clock is 1.
 * 			(CPOL=1, CPHA=1).
 */
#define QSPI_MODE_0						0x00
#define QSPI_MODE_3						0x03

/**
 * @brief QSPI Address configuration
 * Length of the address field in bits. Typical flash chips support 24bit address mode
 */
#define QSPI_ADDRESS_MODE_8BIT			0x00
#define QSPI_ADDRESS_MODE_16BIT			0x01
#define QSPI_ADDRESS_MODE_24BIT			0x02
#define QSPI_ADDRESS_MODE_32BIT			0x03

/**
 * @brief QSPI buffer structure
 * Structure used both for TX and RX purposes.
 *
 * @param buf is a valid pointer to a data buffer.
 * @param len is the length of the data to be handled.
 */
struct qspi_buf {
	u8_t *buf;
	size_t len;
};

/**
 * @brief QSPI controller configuration structure
 */
struct qspi_config {
	 /* Chip Select pin used to select memory */
	u16_t cs_pin;

	/* Frequency of the QSPI bus*/
	u32_t frequency;

	/* Polarity, phase, Mode
	 * 0x00: Mode 0: Data are captured on the clock rising edge and
	 * 			data is output on a falling edge. Base level of clock is 0.
	 * 			(CPOL=0, CPHA=0).
	 * 0x03: Mode 3: Data are captured on the clock falling edge and
	 * 			data is output on a rising edge. Base level of clock is 1.
	 * 			(CPOL=1, CPHA=1).
	 * */
	u8_t mode:2;

	/* Defines how many lines will be used for read/write operation
	 * 0x00: One data line
	 * 0x01: Two data lines
	 * 0x02: Four data lines
	 *  */
	u8_t data_lines:2;

	/* Defines how many bits are used for address (8/16/24/32)
	 * 0x00: 8 bit address field
	 * 0x01: 16 bit address field
	 * 0x02: 24 bit address field
	 * 0x03: 32 bit address field
	 * */
	u8_t address:2;

	/* Specifies the Chip Select High Time. No of clock cycles when CS must remain high between commands. */
	u8_t cs_high_time;
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * These are for internal use only, so skip these in
 * public documentation.
 */
/**
 * @typedef qspi_api_set_act_mem
 * @brief Callback API for I/O
 * See qspi_set_act_mem() for argument descriptions
 */
typedef int (*qspi_api_set_act_mem)(struct device *dev,
				const struct qspi_config *config);

/**
 * @typedef qspi_api_write
 * @brief Callback API for I/O
 * See qspi_write() for argument descriptions
 */
typedef int (*qspi_api_write)(struct device *dev,
				const struct qspi_buf *tx_buf,
				u32_t address);

/**
 * @typedef qspi_api_read
 * @brief Callback API for I/O
 * See qspi_read() for argument descriptions
 */
typedef int (*qspi_api_read)(struct device *dev,
				const struct qspi_buf *rx_buf,
				u32_t address);

/**
 * @typedef qspi_api_send_cmd)
 * @brief Callback API for I/O
 * See qspi_transceive() for argument descriptions
 */
typedef int (*qspi_api_send_cmd)(struct device *dev,
				const struct qspi_buf *tx_buf,
				const struct qspi_buf *rx_buf);

/**
 * @typedef qspi_api_erase
 * @brief Callback API for I/O
 * See qspi_erase() for argument descriptions
 */
typedef int (*qspi_api_erase)(struct device *dev,
				u32_t start_address,
				u32_t length);


/**
 * @brief QSPI driver API
 * This is the mandatory API any QSPI driver needs to expose.
 */
struct qspi_driver_api {
	qspi_api_write 			write;
	qspi_api_read 			read;
	qspi_api_send_cmd 		send_cmd;
	qspi_api_erase 			erase;
	qspi_api_set_act_mem 	set_act_mem;
};
/**
 * @endcond
 */


/**
 * @brief Writes desired amount of data to the external flash memory.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param tx_buf Buffer for transmit purpose. Consists of the fields:
 * 			*buf pointer to the buffer
 * 			len	 amount of data to be sent
 *
 * @param address Address where the data will be written.
 *
 * @retval  0 in case of success
 * @retval	-ENXIO No such device or address
 * @retval	-EINVAL invalid input parameter
 * @retval	-EBUSY device busy
 * @retval	-ETIMEDOUT timeout
 */
__syscall int qspi_write(struct device *dev, const struct qspi_buf *tx_buf, u32_t address);

static inline int z_impl_qspi_write(struct device *dev, const struct qspi_buf *tx_buf, u32_t address)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->write(dev, tx_buf, address);
}


/**
 * @brief Read desired amount of data from the external flash memory.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param rx_buf Buffer for receive purpose. Consists of the fields:
 * 			*buf pointer to the buffer
 * 			len	 amount of data to be received
 *
 * @param address Address from the data will be read.
 *
 * @retval  0 in case of success
 * @retval	-ENXIO No such device or address
 * @retval	-EINVAL invalid input parameter
 * @retval	-EBUSY device busy
 * @retval	-ETIMEDOUT timeout
 */
__syscall int qspi_read(struct device *dev, const struct qspi_buf *rx_buf, u32_t address);

static inline int z_impl_qspi_read(struct device *dev, const struct qspi_buf *rx_buf, u32_t address)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->read(dev, rx_buf, address);
}


/**
 * @brief Function for sending operation code, sending data, and receiving data from the memory device.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param tx_buf Buffer for transmit purpose. Consists of the fields:
 * 			*buf pointer to the buffer
 * 			len	 amount of data to be sent
 *
 * @param rx_buf Buffer for receive purpose. Consists of the fields:
 * 			*buf pointer to the buffer
 * 			len	 amount of data to be received

 * @retval  0 in case of success
 * @retval	-ENXIO No such device or address
 * @retval	-EINVAL invalid input parameter
 * @retval	-EBUSY device busy
 * @retval	-ETIMEDOUT timeout
 */
__syscall int qspi_send_cmd(struct device *dev, const struct qspi_buf *tx_buf, const struct qspi_buf *rx_buf);

static inline int z_impl_qspi_send_cmd(struct device *dev, const struct qspi_buf *tx_buf, const struct qspi_buf *rx_buf)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->send_cmd(dev, tx_buf, rx_buf);
}

/**
 * @brief Erases desired amount of flash memory - one memory block - 4KB, 64KB, or the whole chip.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param start_address Memory address to start erasing.
 * If chip erase is performed, address field is ommited.
 *
 * @param length Size of data to erase.

 * @retval  0 in case of success
 * @retval	-ENXIO No such device or address
 * @retval	-EINVAL invalid input parameter
 * @retval	-EBUSY device busy
 * @retval	-ETIMEDOUT timeout
 */
__syscall int qspi_erase(struct device *dev, u32_t start_address, u32_t length);

static inline int z_impl_qspi_erase(struct device *dev, u32_t start_address, u32_t length)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->erase(dev, start_address, length);
}


/**
 * @brief Configures qspi driver to desired memory.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config pointer to the configuration structure
 *
 * @retval  0 in case of success
 * @retval	-ENXIO No such device or address
 * @retval	-EINVAL invalid input parameter
 * @retval	-EBUSY device busy
 * @retval	-ETIMEDOUT timeout
 */
__syscall int qspi_set_act_mem(struct device *dev, const struct qspi_config *config);

static inline int z_impl_qspi_set_act_mem(struct device *dev, const struct qspi_config *config)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->set_act_mem(dev, config);
}


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/qspi.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_QSPI_H_ */
