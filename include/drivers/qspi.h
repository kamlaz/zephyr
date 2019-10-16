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
 * No of data lines that are used for the transfer
 */
#define QSPI_DATA_LINES_SINGLE			0x00
#define QSPI_DATA_LINES_DOUBLE			0x01
#define QSPI_DATA_LINES_QUAD			0x02

/**
 * No of data lines that are used for the transfer
 */
#define QSPI_MODE_0			0x00
#define QSPI_MODE_3			0x03


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
#define QSPI_ADDRESS_MODE_32BIT			0x03

/**
 * @brief QSPI buffer structure
 *
 * @param buf is a valid pointer on a data buffer.
 * @param len is the length of the buffer.
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

	/* Frequency of the QSPI */
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
 * @typedef qspi_api_io
 * @brief Callback API for I/O
 * See qspi_transceive() for argument descriptions
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
 * @typedef qspi_api_xfer
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
 *
 * @param transceive - us
 */
struct qspi_driver_api {
	qspi_api_write 			write;
	qspi_api_read 			read;
	qspi_api_send_cmd 		send_cmd;
	qspi_api_erase 			erase;
	qspi_api_set_act_mem 	set_act_mem;
};


/**
 * @brief Writes desired amount of data to he external flash memory.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param tx_buf Buffer for tx purpose. Consists of the fields:
 * 			*buf	- pointer to the buffer
 * 			len		- amount of data to be transfered
 *
 * @param address	-  Addres where the data will be written.
 *
 * @retval  0 in case of success
 * 			ENXIO 		- No such device or address
 * 			EINVAL 		- invalid input parameter
 * 			EBUSY 		- device busy
 * 			ETIMEDOUT	- timeout
 */
__syscall int qspi_write(struct device *dev, const struct qspi_buf *tx_buf, u32_t address);

static inline int z_impl_qspi_write(struct device *dev, const struct qspi_buf *tx_buf, u32_t address)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->write(dev, tx_buf, address);
}


/**
 * @brief Read desired amount of data to the external flash memory.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param rx_buf Buffer for rx purpose. Consists of the fields:
 * 			*buf	- pointer to the buffer
 * 			len		- amount of data to be transfered
 *
 * @param address	-  Addres from the data will be read.
 *
 * @retval  0 in case of success
 * 			ENXIO 		- No such device or address
 * 			EINVAL 		- invalid input parameter
 * 			EBUSY 		- device busy
 * 			ETIMEDOUT	- timeout
 */
__syscall int qspi_read(struct device *dev, const struct qspi_buf *rx_buf, u32_t address);

static inline int z_impl_qspi_read(struct device *dev, const struct qspi_buf *rx_buf, u32_t address)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->read(dev, rx_buf, address);
}


/**
 * @brief Send custom command to the external flash memory.
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param tx_buf Buffer for tx purpose. Consists of the fields:
 * 			*buf	- pointer to the buffer
 * 			len		- amount of data to be transfered
 *
 * @param rx_buf Buffer for rx purpose. Consists of the fields:
 * 			*buf	- pointer to the buffer
 * 			len		- amount of data to be transfered

 * @retval  0 in case of success
 * 			ENXIO 		- No such device or address
 * 			EINVAL 		- invalid input parameter
 * 			EBUSY 		- device busy
 * 			ETIMEDOUT	- timeout
 */
__syscall int qspi_send_cmd(struct device *dev, const struct qspi_buf *tx_buf, const struct qspi_buf *rx_buf);

static inline int z_impl_qspi_send_cmd(struct device *dev, const struct qspi_buf *tx_buf, const struct qspi_buf *rx_buf)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->send_cmd(dev, tx_buf, rx_buf);
}

/**
 * @brief Erases desired amount of flash memory
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param start_address Address, from the erasing will begin
 *
 * @param length number of bytes to erase

 * @retval  0 in case of success
 * 			ENXIO 		- No such device or address
 * 			EINVAL 		- invalid input parameter
 * 			EBUSY 		- device busy
 * 			ETIMEDOUT	- timeout
 */
__syscall int qspi_erase(struct device *dev, u32_t start_address, u32_t length);

static inline int z_impl_qspi_erase(struct device *dev, u32_t start_address, u32_t length)
{
	const struct qspi_driver_api *api =
		(const struct qspi_driver_api *)dev->driver_api;

	return api->erase(dev, start_address, length);
}


/**
 * @brief Configures qspi driver to desired memory
 *
 * Note: This function is synchronous.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config pointer to the configuration structure
 *
 * @retval  0 in case of success
 * 			ENXIO 		- No such device or address
 * 			EINVAL 		- invalid input parameter
 * 			EBUSY 		- device busy
 * 			ETIMEDOUT	- timeout
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
