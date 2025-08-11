/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_ctimer

/**
 * @file ambiq_ctimer.c
 * @brief Ambiq Apollo CTIMER-based sys_clock driver
 *
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>

/* ambiq-sdk includes */
#include <soc.h>

#define COUNTER_MAX UINT32_MAX

#define CYC_PER_TICK (sys_clock_hw_cycles_per_sec() / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS    ((k_ticks_t)(COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES   (MAX_TICKS * CYC_PER_TICK)

#define MIN_DELAY 1

#define TIMER_CLKSRC (DT_INST_PROP(0, clk_source))

#if defined(CONFIG_TEST)
const int32_t z_sys_timer_irq_for_test = CTIMER_IRQn;
#endif

/* Timer configuration */
am_hal_ctimer_config_t s_continuous_timer = {
	/* Create 32-bit timer by linking timers A and B */
	.ui32Link = 1,
	/* Set up TimerA. */
	.ui32TimerAConfig = (AM_HAL_CTIMER_FN_CONTINUOUS | AM_HAL_CTIMER_INT_ENABLE |
			     AM_HAL_CTIMER_XT_32_768KHZ),
	/* Set up TimerB. */
	/* TimerB should be 0 when running in 32-bit 'linked' mode */
	.ui32TimerBConfig = 0,
};

/* Elapsed ticks since the previous kernel tick was announced, It will get accumulated every time
 * ctimer_isr is triggered, or sys_clock_set_timeout/sys_clock_elapsed API is called.
 * It will be cleared after sys_clock_announce is called,.
 */
static uint32_t g_tick_elapsed;

/* Value of CTIMER counter when the previous timer API is called, this value is
 * aligned to tick boundary. It is updated along with the g_tick_elapsed value.
 */
static uint32_t g_last_time_stamp;

/* Spinlock to sync between Compare ISR and update of Compare register */
static struct k_spinlock g_lock;

static ALWAYS_INLINE void update_tick_counter(void)
{

	/* Get elapsed cycles */
	uint32_t elapsed_cycle = am_hal_ctimer_read_both();

	/* Get elapsed ticks. */
	uint32_t dticks = elapsed_cycle / CYC_PER_TICK;

	g_last_time_stamp += elapsed_cycle;
	g_tick_elapsed += dticks;
}

static void ambiq_ctimer_delta_set(uint32_t ui32Delta)
{
	am_hal_ctimer_compare_set(0, AM_HAL_CTIMER_BOTH, 0, ui32Delta);
}

static void ctimer_isr(const void *arg)
{
	ARG_UNUSED(arg);

	uint32_t irq_status = am_hal_ctimer_int_status_get(false);

	if (irq_status & AM_HAL_CTIMER_INT_TIMERA0) {
		am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

		k_spinlock_key_t key = k_spin_lock(&g_lock);

		/*Calculate the elapsed ticks based on the current cycle count*/
		am_hal_ctimer_stop(0, AM_HAL_CTIMER_BOTH);

		update_tick_counter();

		if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {

			/* Get the counter value to trigger the next tick interrupt. */
			uint64_t next = (uint64_t)g_last_time_stamp + CYC_PER_TICK;

			/* Read current cycle count. */
			uint32_t now = am_hal_ctimer_read_both();

			/* If current cycle count is smaller than the last time stamp, a counter
			 * overflow happened. We need to extend the current counter value to 64 bits
			 * and add 0xFFFFFFFF to get the correct elapsed cycles.
			 */
			uint64_t now_64 = (g_last_time_stamp <= now) ? (uint64_t)now
								     : (uint64_t)now + COUNTER_MAX;

			uint32_t delta = (now_64 + MIN_DELAY < next) ? (next - now_64) : MIN_DELAY;

			/* Set delta. */
			ambiq_ctimer_delta_set(delta);
		}

		am_hal_ctimer_clear(0, AM_HAL_CTIMER_BOTH);
		am_hal_ctimer_config(0, &s_continuous_timer);
		am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

		k_spin_unlock(&g_lock, key);

		sys_clock_announce(g_tick_elapsed);
		g_tick_elapsed = 0;
	}
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	/* Adjust the ticks to the range of [1, MAX_TICKS]. */
	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks, 1, (int32_t)MAX_TICKS);

	k_spinlock_key_t key = k_spin_lock(&g_lock);

	/* Update the internal tick counter*/
	update_tick_counter();

	k_spin_unlock(&g_lock, key);

	uint64_t next = ticks * CYC_PER_TICK;

	uint32_t delta = (next / CYC_PER_TICK) * CYC_PER_TICK;

	am_hal_ctimer_stop(0, AM_HAL_CTIMER_BOTH);
	am_hal_ctimer_clear(0, AM_HAL_CTIMER_BOTH);
	am_hal_ctimer_config(0, &s_continuous_timer);

	if (delta <= MIN_DELAY) {
		/*If the delta value is smaller than or equal to MIN_DELAY, trigger a interrupt
		 * immediately*/
		am_hal_ctimer_int_set(AM_HAL_CTIMER_INT_TIMERA0);
	} else {
		ambiq_ctimer_delta_set(delta);
	}

	am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&g_lock);
	update_tick_counter();
	k_spin_unlock(&g_lock, key);

	return g_tick_elapsed;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return am_hal_ctimer_read_both();
}

int ctimer_init(void)
{
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);

	am_hal_ctimer_stop(0, AM_HAL_CTIMER_BOTH);
	am_hal_ctimer_clear(0, AM_HAL_CTIMER_BOTH);
	am_hal_ctimer_config(0, &s_continuous_timer);
	am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

	g_last_time_stamp = am_hal_ctimer_read_both();

	NVIC_ClearPendingIRQ(CTIMER_IRQn);
	IRQ_CONNECT(CTIMER_IRQn, 0, ctimer_isr, 0, 0);
	irq_enable(CTIMER_IRQn);

	am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

	/* Start timer with period CYC_PER_TICK if tickless is not enabled */
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		ambiq_ctimer_delta_set(CYC_PER_TICK);
	}

	return 0;
}

SYS_INIT(ctimer_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
