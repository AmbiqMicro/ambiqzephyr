/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_COMPAT_H_
#define ZEPHYR_COMPAT_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define am_util_stdio_printf(...) printk(__VA_ARGS__)

#endif /* ZEPHYR_COMPAT_H_ */
