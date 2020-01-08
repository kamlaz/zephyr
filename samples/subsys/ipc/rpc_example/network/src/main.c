/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <errno.h>
#include <drivers/ipm.h>
#include <sys/printk.h>
#include <device.h>
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include <init.h>

#include <ipm.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/device.h>
#include <metal/alloc.h>

/* Configuration defines */

#define SHM_START_ADDR      (DT_IPC_SHM_BASE_ADDRESS + 0x400)
#define SHM_SIZE            0x7c00
#define SHM_DEVICE_NAME     "sram0.shm"

#define VRING_COUNT         2
#define VRING_TX_ADDRESS    (SHM_START_ADDR + SHM_SIZE - 0x400)
#define VRING_RX_ADDRESS    (VRING_TX_ADDRESS - 0x400)
#define VRING_ALIGNMENT     4
#define VRING_SIZE          16

#define VDEV_STATUS_ADDR    DT_IPC_SHM_BASE_ADDRESS


struct rpmsg_device *rdev;

static K_SEM_DEFINE(ready_sem, 0, 1);
static K_SEM_DEFINE(rx_sem, 0, 1);

static struct device *ipm_tx_handle;
static struct device *ipm_rx_handle;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_rx_sem, 0, 1);
static K_SEM_DEFINE(ept_sem, 0, 1);

static volatile unsigned int received_data;

/* End of configuration defines */

static metal_phys_addr_t shm_physmap[] = { SHM_START_ADDR };
static struct metal_device shm_device = {
		.name = SHM_DEVICE_NAME,
		.bus = NULL,
		.num_regions = 1,
		.regions = {
			{
				.virt = (void*) SHM_START_ADDR,
				.physmap = shm_physmap,
				.size = SHM_SIZE,
				.page_shift = 0xffffffff,
				.page_mask = 0xffffffff,
				.mem_flags = 0,
				.ops = { NULL },
			},
		},
		.node = { NULL },
		.irq_num = 0,
		.irq_info = NULL };

static struct virtqueue *vq[2];
static struct rpmsg_endpoint ep;

static unsigned char virtio_get_status(struct virtio_device *vdev) {
	return VIRTIO_CONFIG_STATUS_DRIVER_OK;
}

static void virtio_set_status(struct virtio_device *vdev, unsigned char status) {
	sys_write8(status, VDEV_STATUS_ADDR);
}

static u32_t virtio_get_features(struct virtio_device *vdev) {
	return BIT(VIRTIO_RPMSG_F_NS);
}

static void virtio_set_features(struct virtio_device *vdev, u32_t features) {
	/* No need for implementation */
}

static void virtio_notify(struct virtqueue *vq) {
	int status;

	status = ipm_send(ipm_tx_handle, 0, 0, NULL, 0);
	if (status != 0) {
		printk("ipm_send failed to notify: %d", status);
	}
}

const struct virtio_dispatch dispatch = {
		.get_status = virtio_get_status,
		.set_status = virtio_set_status,
		.get_features = virtio_get_features,
		.set_features = virtio_set_features,
		.notify = virtio_notify,
};

static void ipm_callback(void *context, u32_t id, volatile void *data) {
	k_sem_give(&data_sem);
}

static int endpoint_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
		u32_t src, void *priv) {
	received_data = *((unsigned int*) data);

	k_sem_give(&data_rx_sem);

	return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ep) {
	rpmsg_destroy_ept(ep);
}

static unsigned int receive_message(void) {
	while (k_sem_take(&data_rx_sem, K_NO_WAIT) != 0) {
		int status = k_sem_take(&data_sem, K_FOREVER);

		if (status == 0) {
			virtqueue_notification(vq[1]);
		}
	}
	return received_data;
}

static int send_message(unsigned int message) {
	return rpmsg_send(&ep, &message, sizeof(message));
}

int rpmsg_platform_init(void) {
	int err;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;

	unsigned int message = 0U;

	static struct virtio_vring_info rvrings[2];
	static struct virtio_device vdev;
	static struct rpmsg_virtio_device rvdev;
	static struct metal_io_region *io;
	static struct metal_device *device;

	/* Libmetal setup */
	err = metal_init(&metal_params);
	if (err) {
		printk("metal_init: failed - error code %d", err);
		return err;
	}

	err = metal_register_generic_device(&shm_device);
	if (err) {
		printk("Couldn't register shared memory device: %d", err);
		return err;
	}

	err = metal_device_open("generic", SHM_DEVICE_NAME, &device);
	if (err) {
		printk("metal_device_open failed: %d", err);
		return err;
	}

	io = metal_device_io_region(device, 0);
	if (!io) {
		printk("metal_device_io_region failed to get region");
		return -ENODEV;
	}

	/* IPM setup */
	ipm_rx_handle = device_get_binding("IPM_1");
	if (!ipm_rx_handle) {
		printk("Could not get RX IPM device handle");
		return -ENODEV;
	}

	ipm_tx_handle = device_get_binding("IPM_0");
	if (!ipm_tx_handle) {
		printk("Could not get TX IPM device handle");
		return -ENODEV;
	}

	ipm_register_callback(ipm_rx_handle, ipm_callback, NULL);

	/* Virtqueue setup */
	vq[0] = virtqueue_allocate(VRING_SIZE);
	if (!vq[0]) {
		printk("virtqueue_allocate failed to alloc vq[0]");
		return -ENOMEM;
	}

	vq[1] = virtqueue_allocate(VRING_SIZE);
	if (!vq[1]) {
		printk("virtqueue_allocate failed to alloc vq[1]");
		return -ENOMEM;
	}

	rvrings[0].io = io;
	rvrings[0].info.vaddr = (void*) VRING_TX_ADDRESS;
	rvrings[0].info.num_descs = VRING_SIZE;
	rvrings[0].info.align = VRING_ALIGNMENT;
	rvrings[0].vq = vq[0];

	rvrings[1].io = io;
	rvrings[1].info.vaddr = (void*) VRING_RX_ADDRESS;
	rvrings[1].info.num_descs = VRING_SIZE;
	rvrings[1].info.align = VRING_ALIGNMENT;
	rvrings[1].vq = vq[1];

	vdev.role = RPMSG_REMOTE;
	vdev.vrings_num = VRING_COUNT;
	vdev.func = &dispatch;
	vdev.vrings_info = &rvrings[0];

	/* setup rvdev */
	err = rpmsg_init_vdev(&rvdev, &vdev, NULL, io, NULL);
	if (err != 0) {
		printk("rpmsg_init_vdev failed %d\n", err);
	}

	/* Get RPMsg device from RPMsg VirtIO device */
	rdev = rpmsg_virtio_get_rpmsg_device(&rvdev);

	err = rpmsg_create_ept(&ep, rdev, "k", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			endpoint_cb, rpmsg_service_unbind);
	if (err != 0) {
		printk("rpmsg_create_ept failed %d\n", err);
	}

	while (message < 9) {
		message = receive_message();
		printk("Remote core received a message: %d\n", message);

		message++;
		err = send_message(message);
		if (err < 0) {
			printk("send_message(%d) failed with status %d\n", message, err);
		}
	}

	/* Clean-up */
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
	printk("OpenAMP demo ended.\n");

	return 0;
}

void main(void) {
	printk("OpenAMP[network] Start Init\n");
	if (rpmsg_platform_init() != 0) {
		printk("Failed to initialise!\n");
	}
}
