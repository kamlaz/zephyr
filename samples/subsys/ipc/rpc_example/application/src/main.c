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

void printf_buff(u8_t * buff, u8_t size){
	printf("\nBuff:");
	for(u8_t i=0; i< size; i++){
		printf("%c", *buff);
		buff++;
	}
}

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

void rx_callback(u8_t *buffer, size_t len) {
	printf_buff(buffer, len);
}


void main(void) {
	u32_t cnt = 0;
	struct device *dev;

	dev = device_get_binding(LED_PORT);
	/* Set LED pin as output */
	gpio_pin_configure(dev, LED, GPIO_DIR_OUT);

	ipc_register_rx_callback(rx_callback);
	printf("OpenAMP[application] Start Init\n");
	if (ipc_init() != 0) {
		printf("Failed to initialise!\n");
	}

	unsigned int message = 0U;
	static unsigned int buff[16]={0};

	console_init();
	printk("You should see another line with instructions below. If not,\n");
	printk("the (interrupt-driven) console device doesn't work as expected:\n");
	console_write(NULL, prompt, sizeof(prompt) - 1);



	while (1) {
		u8_t c = console_getchar();
		add_to_buff(&console_buffer, c);
		if (c != '\r') {
			console_putchar(c);
			add_to_buff(buff,c);
		} else {
			ipc_transmit(buff, sizeof(buff), 50);
			reset_buff(buff);
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
