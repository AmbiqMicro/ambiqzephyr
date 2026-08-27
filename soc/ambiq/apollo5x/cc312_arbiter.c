/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include "cc312_arbiter.h"

#if DT_HAS_COMPAT_STATUS_OKAY(ambiq_crypto_aes)
#define CC312_IRQN     DT_IRQN(DT_INST(0, ambiq_crypto_aes))
#define CC312_IRQ_PRIO DT_IRQ(DT_INST(0, ambiq_crypto_aes), priority)
#elif DT_HAS_COMPAT_STATUS_OKAY(ambiq_crypto_sha)
#define CC312_IRQN     DT_IRQN(DT_INST(0, ambiq_crypto_sha))
#define CC312_IRQ_PRIO DT_IRQ(DT_INST(0, ambiq_crypto_sha), priority)
#elif DT_HAS_COMPAT_STATUS_OKAY(ambiq_crypto_chacha)
#define CC312_IRQN     DT_IRQN(DT_INST(0, ambiq_crypto_chacha))
#define CC312_IRQ_PRIO DT_IRQ(DT_INST(0, ambiq_crypto_chacha), priority)
#endif

static K_MUTEX_DEFINE(cc312_lock);

struct k_mutex *ambiq_cc312_arbiter_lock(void)
{
	return &cc312_lock;
}

#ifdef CC312_IRQN

static K_SEM_DEFINE(cc312_sem, 0, 1);
static atomic_t cc312_seen;
static atomic_t cc312_wait_mask;

static void ambiq_cc312_arbiter_isr(const void *arg)
{
	uint32_t irr_val;
	uint32_t clear_mask;
	uint32_t irq_seen;
	uint32_t irq_wait_mask;

	ARG_UNUSED(arg);

	irr_val = CRYPTO->HOSTRGFIRR;
	if (irr_val == 0U) {
		return;
	}

	clear_mask = irr_val;
	if ((irr_val & CRYPTO_HOSTRGFIRR_AHBERRINT_Msk) != 0U) {
		clear_mask |= CRYPTO_HOSTRGFICR_AXIERRCLEAR_Msk;
	}
	CRYPTO->HOSTRGFICR = clear_mask;

	irq_seen = (uint32_t)atomic_or(&cc312_seen, irr_val) | irr_val;
	irq_wait_mask = (uint32_t)atomic_get(&cc312_wait_mask);

	if (((irq_seen & irq_wait_mask) != 0U) ||
	    ((irr_val & CRYPTO_HOSTRGFIRR_AHBERRINT_Msk) != 0U)) {
		k_sem_give(&cc312_sem);
	}
}

void ambiq_cc312_arbiter_connect(void)
{
	static bool connected;

	if (connected) {
		return;
	}
	connected = true;

	IRQ_CONNECT(CC312_IRQN, CC312_IRQ_PRIO, ambiq_cc312_arbiter_isr, NULL, 0);
	irq_disable(CC312_IRQN);
}

void ambiq_cc312_arbiter_prepare(uint32_t wait_mask)
{
	irq_disable(CC312_IRQN);
	k_sem_reset(&cc312_sem);
	(void)atomic_set(&cc312_seen, 0U);
	(void)atomic_set(&cc312_wait_mask, wait_mask);
}

int ambiq_cc312_arbiter_wait(k_timeout_t timeout)
{
	int ret;

	irq_enable(CC312_IRQN);
	ret = k_sem_take(&cc312_sem, timeout);
	irq_disable(CC312_IRQN);
	return ret;
}

void ambiq_cc312_arbiter_finish(void)
{
	irq_disable(CC312_IRQN);
	(void)atomic_set(&cc312_wait_mask, 0U);
}

uint32_t ambiq_cc312_arbiter_seen(void)
{
	return (uint32_t)atomic_get(&cc312_seen);
}

#else /* !CC312_IRQN */

void ambiq_cc312_arbiter_connect(void)
{
}

void ambiq_cc312_arbiter_prepare(uint32_t wait_mask)
{
	ARG_UNUSED(wait_mask);
}

int ambiq_cc312_arbiter_wait(k_timeout_t timeout)
{
	ARG_UNUSED(timeout);
	return -ENOSYS;
}

void ambiq_cc312_arbiter_finish(void)
{
}

uint32_t ambiq_cc312_arbiter_seen(void)
{
	return 0U;
}

#endif /* CC312_IRQN */
