/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/qspi.h>
#include <syscall_handler.h>
#include <string.h>

/* This assumes that bufs and buf_copy are copies from the values passed
 * as syscall arguments.
 */
static void copy_and_check(struct qspi_buf_set *bufs,
			   struct qspi_buf *buf_copy,
			   int writable, void *ssf)
{
	size_t i;

	if (bufs->count == 0) {
		bufs->buffers = NULL;
		return;
	}

	/* Validate the array of struct qspi_buf instances */
	Z_OOPS(Z_SYSCALL_MEMORY_ARRAY_READ(bufs->buffers,
					   bufs->count,
					   sizeof(struct qspi_buf)));

	/* Not worried abuot overflow here: _SYSCALL_MEMORY_ARRAY_READ()
	 * takes care of it.
	 */
	bufs->buffers = memcpy(buf_copy,
			       bufs->buffers,
			       bufs->count * sizeof(struct qspi_buf));

	for (i = 0; i < bufs->count; i++) {
		/* Now for each array element, validate the memory buffers
		 * that they point to
		 */
		const struct qspi_buf *buf = &bufs->buffers[i];

		Z_OOPS(Z_SYSCALL_MEMORY(buf->buf, buf->len, writable));
	}
}

/* This function is only here so tx_buf_copy and rx_buf_copy can be allocated
 * using VLA.  It assumes that both tx_bufs and rx_bufs will receive a copy of
 * the values passed to the syscall as arguments.  It also assumes that the
 * count member has been verified and is a value that won't lead to stack
 * overflow.
 */
static u32_t copy_bufs_and_transceive(struct device *dev,
				      const struct qspi_config *config,
				      struct qspi_buf_set *tx_bufs,
				      struct qspi_buf_set *rx_bufs,
				      void *ssf)
{
	struct qspi_buf tx_buf_copy[tx_bufs->count ? tx_bufs->count : 1];
	struct qspi_buf rx_buf_copy[rx_bufs->count ? rx_bufs->count : 1];

	copy_and_check(tx_bufs, tx_buf_copy, 0, ssf);
	copy_and_check(rx_bufs, rx_buf_copy, 1, ssf);

	return z_impl_qspi_transceive((struct device *)dev, config,
				    tx_bufs, rx_bufs);
}

Z_SYSCALL_HANDLER(qspi_transceive, dev, config_p, tx_bufs, rx_bufs)
{
	const struct qspi_config *config = (const struct qspi_config *)config_p;
	struct qspi_buf_set tx_bufs_copy;
	struct qspi_buf_set rx_bufs_copy;
	struct qspi_config config_copy;

	Z_OOPS(Z_SYSCALL_MEMORY_READ(config, sizeof(*config)));
	Z_OOPS(Z_SYSCALL_DRIVER_QSPI(dev, transceive));

	if (tx_bufs) {
		const struct qspi_buf_set *tx =
			(const struct qspi_buf_set *)tx_bufs;

		Z_OOPS(Z_SYSCALL_MEMORY_READ(tx_bufs,
					     sizeof(struct qspi_buf_set)));
		memcpy(&tx_bufs_copy, tx, sizeof(tx_bufs_copy));
		Z_OOPS(Z_SYSCALL_VERIFY(tx_bufs_copy.count < 32));
	} else {
		memset(&tx_bufs_copy, 0, sizeof(tx_bufs_copy));
	}

	if (rx_bufs) {
		const struct qspi_buf_set *rx =
			(const struct qspi_buf_set *)rx_bufs;

		Z_OOPS(Z_SYSCALL_MEMORY_READ(rx_bufs,
					     sizeof(struct qspi_buf_set)));
		memcpy(&rx_bufs_copy, rx, sizeof(rx_bufs_copy));
		Z_OOPS(Z_SYSCALL_VERIFY(rx_bufs_copy.count < 32));
	} else {
		memset(&rx_bufs_copy, 0, sizeof(rx_bufs_copy));
	}

	memcpy(&config_copy, config, sizeof(*config));
	if (config_copy.cs) {
		const struct qspi_cs_control *cs = config_copy.cs;

		Z_OOPS(Z_SYSCALL_MEMORY_READ(cs, sizeof(*cs)));
		if (cs->gpio_dev) {
			Z_OOPS(Z_SYSCALL_OBJ(cs->gpio_dev, K_OBJ_DRIVER_GPIO));
		}
	}

	/* ssf is implicit system call stack frame parameter, used by
	 * _SYSCALL_* APIs when something goes wrong.
	 */
	return copy_bufs_and_transceive((struct device *)dev,
					&config_copy,
					&tx_bufs_copy,
					&rx_bufs_copy,
					ssf);
}

Z_SYSCALL_HANDLER(qspi_release, dev, config_p)
{
	const struct qspi_config *config = (const struct qspi_config *)config_p;

	Z_OOPS(Z_SYSCALL_MEMORY_READ(config, sizeof(*config)));
	Z_OOPS(Z_SYSCALL_DRIVER_QSPI(dev, release));
	return z_impl_qspi_release((struct device *)dev, config);
}
