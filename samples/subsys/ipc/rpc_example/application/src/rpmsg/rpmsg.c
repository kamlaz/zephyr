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

