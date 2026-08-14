/*
 * Copyright (c) 2025 Ambiq LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>

#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

#ifdef CONFIG_SOC_AMBIQ_DEEPER_SLEEP_RECOVERY

#if DT_NODE_EXISTS(DT_ALIAS(pm_recovery))
#define AMBIQ_RECOVERY_NODE DT_ALIAS(pm_recovery)
#elif DT_NODE_EXISTS(DT_ALIAS(sw0))
#define AMBIQ_RECOVERY_NODE DT_ALIAS(sw0)
#endif

#ifdef AMBIQ_RECOVERY_NODE

static void ambiq_deeper_sleep_allow(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_DISK, PM_ALL_SUBSTATES);
}

static K_TIMER_DEFINE(ambiq_deeper_sleep_timer, ambiq_deeper_sleep_allow, NULL);

static int ambiq_deeper_sleep_boot_window(void)
{
	static const struct gpio_dt_spec pin =
		GPIO_DT_SPEC_GET(AMBIQ_RECOVERY_NODE, gpios);

	if (!gpio_is_ready_dt(&pin) || gpio_pin_configure_dt(&pin, GPIO_INPUT) < 0) {
		return 0;
	}

	if (gpio_pin_get_dt(&pin) <= 0) {
		return 0;
	}

	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_DISK, PM_ALL_SUBSTATES);
	k_timer_start(&ambiq_deeper_sleep_timer,
		      K_MSEC(CONFIG_SOC_AMBIQ_DEEPER_SLEEP_BOOT_WINDOW_MS), K_NO_WAIT);

	LOG_WRN("recovery pin asserted at boot: suspend-to-disk blocked for %d ms",
		CONFIG_SOC_AMBIQ_DEEPER_SLEEP_BOOT_WINDOW_MS);

	return 0;
}

SYS_INIT(ambiq_deeper_sleep_boot_window, APPLICATION, 0);

#endif /* AMBIQ_RECOVERY_NODE */
#endif /* CONFIG_SOC_AMBIQ_DEEPER_SLEEP_RECOVERY */

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	__disable_irq();
	__set_BASEPRI(0);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE: {
		/* Put ARM core to normal sleep. */
		sys_trace_idle();
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
		sys_trace_idle_exit();
		break;
	}
	case PM_STATE_SUSPEND_TO_RAM: {
		/* Put ARM core to deep sleep. */
		/* Cotex-m: power down, register value preserve.*/
		/* Cache: power down*/
		/* MRAM: power down*/
		/* ITCM + DTCM: retention, active on request*/
		/* Sram: retention, active on request*/
		sys_trace_idle();
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
		sys_trace_idle_exit();
		break;
	}
#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
	case PM_STATE_SUSPEND_TO_DISK: {
		/* Ambiq deeper sleep. */
		/* Only NVM memory is retained */
		sys_trace_idle();
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEPER);
		sys_trace_idle_exit();
		break;
	}
#endif
	default: {
		LOG_DBG("Unsupported power state %u", state);
		break;
	}
	}
}

/**
 * @brief PM State Exit Post Operations
 *
 * For PM_STATE_SUSPEND_TO_IDLE:
 *   Nothing is needed after soc woken up.
 *
 * For PM_STATE_SUSPEND_TO_RAM:
 *   Flash, cache, sram automatically switch
 *   to active state on wake up
 *
 * @param state PM State
 * @param substate_id Unused
 *
 */
void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	__enable_irq();
	irq_unlock(0);
}
