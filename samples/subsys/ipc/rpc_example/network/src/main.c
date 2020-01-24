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
#include <console/console.h>

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

/* Private defines */
#include "rpmsg.h"

void printf_buff(u8_t * buff, u8_t size){
	printf("\nBuff:");
	for(u8_t i=0; i< size; i++){
		printf("%c", *buff);
		buff++;
	}
}

void rx_callback(u8_t *buffer, size_t len) {
	printf_buff(buffer, len);
}

void main(void) {
	printk("OpenAMP[network] Start Init\n");
	ipc_register_rx_callback(rx_callback);
	if (ipc_init() != 0) {
		printk("Failed to initialise!\n");
	}

	while (1) {
		if (ipc_transmit("dup", sizeof("dup"), 50) != 0) {
			printf("\nTX: failed");
		} else {
			printf("\nTX: success");
		}
		k_sleep(500);
	}
}
