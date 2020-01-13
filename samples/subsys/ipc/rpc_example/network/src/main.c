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

static const char prompt[] = "Start typing characters to see them echoed back\r\n";


typedef struct{
	u8_t * name;
	u8_t id;
	void(*func)(void);
}cmd;

const cmd led_on = {
	.name = "LED_ON",
	.id = 69,
};

const cmd led_off = {
	.name = "LED_OFF",
	.id = 55,
};

const cmd command_tab[] = {led_on, led_off};

typedef struct{
	u8_t buff[64];
	u8_t idx;
} buff;

buff console_buffer = {
		.buff = {0},
		.idx = 0,
};

void add_to_buff(buff * buffer, u8_t value){
	buffer->buff[buffer->idx] = value;
	buffer->idx++;
}

u8_t get_buff_len(buff * buffer){
	return buffer->idx;
}

void reset_buff(buff * buffer){
	memset(buffer->buff,0,sizeof(buffer->buff));
	buffer->idx = 0;
}

void main(void) {
	printk("OpenAMP[network] Start Init\n");
	if (rpmsg_platform_init() != 0) {
		printk("Failed to initialise!\n");
	}

//	console_init();
//	printk("You should see another line with instructions below. If not,\n");
//	printk("the (interrupt-driven) console device doesn't work as expected:\n");
//	console_write(NULL, prompt, sizeof(prompt) - 1);
//	receive_message();
	int ret = send_message("dupa", sizeof("dupa"));
	if (ret < 0) {
		printk("Failed. ERR: %d", ret);
	} else {
		printk("OK");
	}
	/*
	 *  save_to_flash
	 *  read_from_flash
	 *  led_set(u8_t led_no)
	 *  led_reset(u8_t led_no)
	 */

//	while (1) {
//		u8_t c = console_getchar();
//		add_to_buff(&console_buffer, c);
//		if (c != '\r') {
//			console_putchar(c);
//		} else {
//			u8_t cmd_found = 0;
//			/* Parsing command */
//			for (u8_t i = 0; i < 2; i++) {
//				if (memcmp(console_buffer.buff, command_tab[i].name,
//						get_buff_len(&console_buffer) - 1) == 0) {
//
//					cmd_found = 1;
//					send_message(command_tab[i].id);
//					break;
//				} else {
//				}
//			}
//			/* ANALYSE RESULT */
//			if (cmd_found) {
//				printk("\n    CMD FOUND\n");
//
//			} else {
//				printk("\n    CMD MISMATCH\n");
//			}
//			reset_buff(&console_buffer);
//		}
//	}
}




