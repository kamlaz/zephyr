/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Includes ------------------------------------------------------------------*/
#include <zephyr.h>
#include <errno.h>
#include <drivers/ipm.h>
#include <sys/printk.h>

/* Private includes ----------------------------------------------------------*/
#include <ipm.h>
#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/device.h>
#include <metal/alloc.h>
#include "rpmsg.h"

/* Private typedef -----------------------------------------------------------*/
enum {
	TX_ONGOING, RX_READY,
};

typedef struct {
	void (*rx_callback)(u8_t *buffer, size_t len); /* Rx callback */
	struct k_sem data_rx_sem;						/* RX data semaphore */
	struct k_sem data_tx_sem;						/* TX data semaphore */
	struct k_sem ept_sem;							/* Endpoint semaphore */
	u8_t status; 									/* Current status */
	u8_t response; 									/* Holds response from TX */
} rpc_ctx;

/* Private define ------------------------------------------------------------*/
#define MASTER		0
#define REMOTE		1
#define RMPSG_PROPERTY 	REMOTE					/* Configuration of the library */

/* Configuration of RX task --------------------------------------------------*/
#define STACKSIZE 1024			/* size of stack area used by each thread */
#define PRIORITY 7				/* scheduling priority used by each thread */

/* Private macro -------------------------------------------------------------*/
/* Shared memory configuration */
#define SHM_START_ADDR      (DT_IPC_SHM_BASE_ADDRESS + 0x400)
#define SHM_SIZE            0x7c00
#define SHM_DEVICE_NAME     "sram0.shm"

#define VRING_COUNT         2
#define VRING_TX_ADDRESS    (SHM_START_ADDR + SHM_SIZE - 0x400)
#define VRING_RX_ADDRESS    (VRING_TX_ADDRESS - 0x400)
#define VRING_ALIGNMENT     4
#define VRING_SIZE          16

#define VDEV_STATUS_ADDR    DT_IPC_SHM_BASE_ADDRESS

/* Private variables ---------------------------------------------------------*/
/* Handlers for TX and RX channels */
static struct device *ipm_tx_handle;
static struct device *ipm_rx_handle;

static struct rpmsg_virtio_device rvdev;
#if (RMPSG_PROPERTY == MASTER)
#else
struct rpmsg_device *rdev;
#endif

static metal_phys_addr_t shm_physmap[] = { SHM_START_ADDR };
static struct metal_device shm_device = {
		.name = SHM_DEVICE_NAME,
		.bus = NULL,
		.num_regions = 1,
		.regions =
			{
				{ .virt = (void*) SHM_START_ADDR,
				.physmap = shm_physmap,
				.size = SHM_SIZE,
				.page_shift =0xffffffff,
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

static K_THREAD_STACK_DEFINE(rx_thread_stack,STACKSIZE);
static struct k_thread rx_thread_data;

static rpc_ctx rpc; 						/* Main context structure */

/* Private function prototypes -----------------------------------------------*/
static unsigned char virtio_get_status(struct virtio_device *vdev) {
	return VIRTIO_CONFIG_STATUS_DRIVER_OK;
}

void virtio_set_status(struct virtio_device *vdev, unsigned char status) {
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

const struct virtio_dispatch dispatch = { .get_status = virtio_get_status,
		.set_status = virtio_set_status, .get_features = virtio_get_features,
		.set_features = virtio_set_features, .notify = virtio_notify, };

/* Callback launch right after some data arrived. */
static void ipm_callback(void *context, u32_t id, volatile void *data) {
	k_sem_give(&rpc.data_rx_sem);
}

static int endpoint_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
		u32_t src, void *priv) {
	if (rpc.status != TX_ONGOING) {
		/* First of all - send ACK */
		u8_t tmp = 0;
		rpmsg_send(&ep, &tmp, 1);

		/* We have just received data package */
		if (rpc.rx_callback) {
			rpc.rx_callback(data, len);
		}
	} else {
		/* We were sending data - we received the reponse */
		rpc.status = RX_READY;
		u8_t *rec_data = data;
		rpc.response = *rec_data;
		k_sem_give(&rpc.data_tx_sem);
	}

	return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ep) {
	rpmsg_destroy_ept(ep);
}

#if (RMPSG_PROPERTY == MASTER)
static void ns_bind_cb(struct rpmsg_device *rdev, const char *name, u32_t dest) {
	(void) rpmsg_create_ept(&ep, rdev, name, RPMSG_ADDR_ANY, dest, endpoint_cb,
			rpmsg_service_unbind);
	k_sem_give(&rpc.ept_sem);
}
#endif

static void rx_thread(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		int status = k_sem_take(&rpc.data_rx_sem, K_FOREVER);
		if (status == 0) {
#if (RMPSG_PROPERTY == MASTER)
			virtqueue_notification(vq[0]);
#else
			virtqueue_notification(vq[1]);
#endif
		}
	}
}

int ipc_init(void) {
	int err;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;

	static struct virtio_vring_info rvrings[2];
	static struct virtio_device vdev;
	static struct metal_io_region *shm_io;
	static struct metal_device *device;
#if (RMPSG_PROPERTY == MASTER)
	static struct rpmsg_virtio_shm_pool shpool;
#endif

	/* Init semaphores */
	k_sem_init(&rpc.data_rx_sem, 0, 1);
	k_sem_init(&rpc.data_tx_sem, 0, 1);
	k_sem_init(&rpc.ept_sem, 0, 1);
	rpc.status = RX_READY;

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

	shm_io = metal_device_io_region(device, 0);
	if (!shm_io) {
		printk("metal_device_io_region failed to get region");
		return -ENODEV;
	}

#if(RMPSG_PROPERTY == MASTER)
	/* IPM setup */
	ipm_tx_handle = device_get_binding("IPM_1");
	if (!ipm_tx_handle) {
		printk("Could not get TX IPM device handle");
		return -ENODEV;
	}

	ipm_rx_handle = device_get_binding("IPM_0");
	if (!ipm_rx_handle) {
		printk("Could not get RX IPM device handle");
		return -ENODEV;
	}
#else
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
#endif

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

	rvrings[0].io = shm_io;
	rvrings[0].info.vaddr = (void*) VRING_TX_ADDRESS;
	rvrings[0].info.num_descs = VRING_SIZE;
	rvrings[0].info.align = VRING_ALIGNMENT;
	rvrings[0].vq = vq[0];

	rvrings[1].io = shm_io;
	rvrings[1].info.vaddr = (void*) VRING_RX_ADDRESS;
	rvrings[1].info.num_descs = VRING_SIZE;
	rvrings[1].info.align = VRING_ALIGNMENT;
	rvrings[1].vq = vq[1];

#if(RMPSG_PROPERTY == MASTER)
	vdev.role = RPMSG_MASTER;
#else
	vdev.role = RPMSG_REMOTE;
#endif
	vdev.vrings_num = VRING_COUNT;
	vdev.func = &dispatch;
	vdev.vrings_info = &rvrings[0];

#if(RMPSG_PROPERTY == MASTER)
	/* This step is only required if you are VirtIO device master.
	 * Initialize the shared buffers pool.
	 */
	rpmsg_virtio_init_shm_pool(&shpool, (void*) SHM_START_ADDR, SHM_SIZE);
	err = rpmsg_init_vdev(&rvdev, &vdev, ns_bind_cb, shm_io, &shpool);
	if (err) {
		printk("rpmsg_init_vdev failed %d\n", err);
		return err;
	}

	/* Since we are using name service, we need to wait for a response
	 * from NS setup and than we nek_sem_take(&data_sem, K_FOREVER);ed to process it
	 */
	k_sem_take(&rpc.data_rx_sem, K_FOREVER);
	virtqueue_notification(vq[0]);

	/* Wait til nameservice ep is setup */
	k_sem_take(&rpc.ept_sem, K_FOREVER);

	printk("Endpoint set successfully.\n");

#else
	/* setup rvdev */
	err = rpmsg_init_vdev(&rvdev, &vdev, NULL, shm_io, NULL);
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
#endif

	/* Setup thread for RX data processing. */
	k_thread_create(&rx_thread_data,
					rx_thread_stack,
					K_THREAD_STACK_SIZEOF(rx_thread_stack),
					rx_thread,
					NULL,
					NULL,
					NULL,
					K_PRIO_COOP(PRIORITY),
					0,
					K_NO_WAIT);

#if(RMPSG_PROPERTY == MASTER)
	/* Master has to send first msg */
	err = rpmsg_send(&ep, "hello!", sizeof("hello!"));
	if (err < 0) {
		printk("send_message failed with status %d\n", err);
	}
#else
	/* Remote has to receive first msg */
#endif

	return 0;
}

void ipc_deinit(void) {
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
	printk("OpenAMP deinitialised.\n");
}

int ipc_transmit(u8_t *buff, size_t buff_len, uint32_t timeout) {
	int ret = 0;
	rpc.status = TX_ONGOING;
	ret = rpmsg_send(&ep, buff, buff_len);
	if (ret > 0) {
		/* Wait here until the response */
		if (k_sem_take(&rpc.data_tx_sem, K_MSEC(timeout)) != 0) {
			printk("Failed to take TX sem!");
			ret = -1;
		} else {
			ret = rpc.response;
		}
	}

	return ret;
}

void ipc_register_rx_callback(void (*rx_callback)(u8_t *buffer, size_t len)) {
	rpc.rx_callback = rx_callback;
}
