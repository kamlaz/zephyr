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
#include <drivers/gpio.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include <init.h>

/* Private defines */
#include "rpmsg.h"

/* LED defines */
#define LED_PORT	DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED		DT_ALIAS_LED0_GPIOS_PIN

/* 1000 msec = 1 sec */
#define SLEEP_TIME	1000


void main(void) {
	u32_t cnt = 0;
	struct device *dev;

	dev = device_get_binding(LED_PORT);
	/* Set LED pin as output */
	gpio_pin_configure(dev, LED, GPIO_DIR_OUT);


	printf("OpenAMP[application] Start Init\n");
	if (rpmsg_platform_init() != 0) {
		printf("Failed to initialise!\n");
	}

	unsigned int message = 0U;
	while (1) {
		message = receive_message();
		switch(message){
		case 69:
			gpio_pin_write(dev, LED, 0);
			break;

		case 55:
			gpio_pin_write(dev, LED, 1);
			break;
		default:
			break;
		}
	}
}

/* Make sure we clear out the status flag very early (before we bringup the
 * secondary core) so the secondary core see's the proper status
 */
int init_status_flag(struct device *arg) {
	virtio_set_status(NULL, 0);

	return 0;
}

SYS_INIT(init_status_flag, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
