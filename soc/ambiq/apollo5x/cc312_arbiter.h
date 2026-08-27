/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_AMBIQ_APOLLO5X_CC312_ARBITER_H_
#define ZEPHYR_SOC_AMBIQ_APOLLO5X_CC312_ARBITER_H_

#include <zephyr/kernel.h>

struct k_mutex *ambiq_cc312_arbiter_lock(void);
void ambiq_cc312_arbiter_connect(void);
void ambiq_cc312_arbiter_prepare(uint32_t wait_mask);
int ambiq_cc312_arbiter_wait(k_timeout_t timeout);
uint32_t ambiq_cc312_arbiter_seen(void);
void ambiq_cc312_arbiter_finish(void);

#endif /* ZEPHYR_SOC_AMBIQ_APOLLO5X_CC312_ARBITER_H_ */
