/*
 * Copyright (c) 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/arch.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <zephyr/timing/timing.h>
#include <soc.h>

#ifndef CONFIG_CORTEX_M_SYSTICK
#define CLOCK_HW_CYCLES_PER_SEC	AM_HAL_CLKGEN_FREQ_MAX_HZ
#else
#define CLOCK_HW_CYCLES_PER_SEC CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
#endif

void soc_timing_init(void)
{
#ifndef CONFIG_CORTEX_M_SYSTICK
	am_hal_systick_init(AM_HAL_SYSTICK_CLKSRC_INT);
#endif
}

void soc_timing_start(void)
{
	am_hal_systick_stop();
	am_hal_systick_load(0x00FFFFFF);
	am_hal_systick_start();
}

void soc_timing_stop(void)
{
	am_hal_systick_stop();
}

timing_t soc_timing_counter_get(void)
{
	return (0x00FFFFFF - am_hal_systick_count());
}

uint64_t soc_timing_cycles_get(volatile timing_t *const start,
			       volatile timing_t *const end)
{
	return *end - *start;
}

uint64_t soc_timing_freq_get(void)
{
	return CLOCK_HW_CYCLES_PER_SEC;
}

uint64_t soc_timing_cycles_to_ns(uint64_t cycles)
{
	return cycles * NSEC_PER_SEC / CLOCK_HW_CYCLES_PER_SEC;
}

uint64_t soc_timing_cycles_to_ns_avg(uint64_t cycles, uint32_t count)
{
	return (uint32_t)soc_timing_cycles_to_ns(cycles) / count;
}

uint32_t soc_timing_freq_get_mhz(void)
{
	return (uint32_t)(soc_timing_freq_get() / 1000000);
}
