/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_counter

#include <zephyr/drivers/counter.h>
#include <zephyr/spinlock.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/atomic.h>
/* ambiq-sdk includes */
#include <soc.h>

LOG_MODULE_REGISTER(ambiq_counter, CONFIG_COUNTER_LOG_LEVEL);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
#define SOC_TIMER_BASE CTIMER_BASE
#elif defined(CONFIG_SOC_SERIES_APOLLO4X) || defined(CONFIG_SOC_SERIES_APOLLO5X)
#define SOC_TIMER_BASE TIMER_BASE
#endif
static void counter_ambiq_isr(void *arg);

struct counter_ambiq_config {
	struct counter_config_info counter_info;
	uint32_t instance;
	uint32_t clk_src;
	void (*irq_config_func)(void);
};

struct counter_ambiq_data {
	counter_alarm_callback_t callback;
	uint32_t freq;
	void *user_data;
#if defined(CONFIG_PM_DEVICE)
	bool pm_lock_held;
#endif
};

static struct k_spinlock lock;

#if defined(CONFIG_SOC_SERIES_APOLLO3X) || defined(CONFIG_SOC_SERIES_APOLLO4X) ||                  \
	defined(CONFIG_SOC_SERIES_APOLLO5X)
/*
 * Check if the timer clock source is a high-frequency clock that stops during sleep.
 * High-frequency clocks (HFRC, HFRC2, XTAL_HS, HCLK) stop in sleep mode.
 * Low-frequency clocks (LFRC, XT, RTC) continue running in sleep mode.
 */
static inline bool is_high_freq_clock(uint32_t clk_src)
{
	switch (clk_src) {
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	/* HFRC clock variants - stop during sleep */
	case AM_HAL_CTIMER_HFRC_12MHZ:
	case AM_HAL_CTIMER_HFRC_3MHZ:
	case AM_HAL_CTIMER_HFRC_187_5KHZ:
	case AM_HAL_CTIMER_HFRC_47KHZ:
	case AM_HAL_CTIMER_HFRC_12KHZ:
	/* HCLK (system clock) - stops during sleep */
	case AM_HAL_CTIMER_HCLK_DIV4:
		return true;
#elif defined(CONFIG_SOC_APOLLO510) || defined(CONFIG_SOC_APOLLO510B)
	/* HFRC clock variants - stop during sleep */
	case AM_HAL_TIMER_CLOCK_HFRC_DIV4:
	case AM_HAL_TIMER_CLOCK_HFRC_DIV16:
	case AM_HAL_TIMER_CLOCK_HFRC_DIV64:
	case AM_HAL_TIMER_CLOCK_HFRC_DIV256:
	case AM_HAL_TIMER_CLOCK_HFRC_DIV1024:
	case AM_HAL_TIMER_CLOCK_HFRC_DIV4K:
	/* HFRC2 125MHz clock variants - stop during sleep */
	case AM_HAL_TIMER_CLOCK_HFRC2_125MHz_DIV8:
	case AM_HAL_TIMER_CLOCK_HFRC2_125MHz_DIV16:
	case AM_HAL_TIMER_CLOCK_HFRC2_125MHz_DIV32:
	case AM_HAL_TIMER_CLOCK_HFRC2_125MHz_DIV64:
	case AM_HAL_TIMER_CLOCK_HFRC2_125MHz_DIV128:
	case AM_HAL_TIMER_CLOCK_HFRC2_125MHz_DIV256:
	/* XTAL high-speed variants - stop during sleep */
	case AM_HAL_TIMER_CLOCK_XTAL_HS:
	case AM_HAL_TIMER_CLOCK_XTAL_HS_DIV2:
	case AM_HAL_TIMER_CLOCK_XTAL_HS_DIV4:
		return true;
#elif defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
	/* PLL post-divider - stops during sleep */
	case AM_HAL_TIMER_CLOCK_PLL_POSTDIV:
	/* RF XTAL variants - stop during sleep */
	case AM_HAL_TIMER_CLOCK_RF_XTAL:
	case AM_HAL_TIMER_CLOCK_RF_XTAL_DIV2:
	case AM_HAL_TIMER_CLOCK_RF_XTAL_DIV4:
	/* External reference clock variants - stop during sleep */
	case AM_HAL_TIMER_CLOCK_XTHS_EXTREF_CLK:
	case AM_HAL_TIMER_CLOCK_XTHS_EXTREF_CLK_DIV2:
	case AM_HAL_TIMER_CLOCK_XTHS_EXTREF_CLK_DIV4:
	case AM_HAL_TIMER_CLOCK_XTHS_EXTREF_CLK_DIV8:
		return true;
#elif defined(CONFIG_SOC_SERIES_APOLLO4X)
	/* HFRC clock variants - stop during sleep */
	case AM_HAL_TIMER_CLOCK_HFRC_DIV4:
	case AM_HAL_TIMER_CLOCK_HFRC_DIV16:
	case AM_HAL_TIMER_CLOCK_HFRC_DIV64:
	case AM_HAL_TIMER_CLOCK_HFRC_DIV256:
	case AM_HAL_TIMER_CLOCK_HFRC_DIV1024:
	case AM_HAL_TIMER_CLOCK_HFRC_DIV4K:
		return true;
#endif
	default:
		/* All other clocks (LFRC, XT, RTC, etc.) are low-frequency and continue in sleep */
		return false;
	}
}
#endif
/* CONFIG_SOC_SERIES_APOLLO3X || CONFIG_SOC_SERIES_APOLLO4X || CONFIG_SOC_SERIES_APOLLO5X */

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
static void counter_irq_config_func(void)
{
	/* Apollo3 counters share the same irq number,
	 * connect to counter0 once when init and handle
	 * different banks in counter_ambiq_isr
	 */
	static atomic_t global_irq_init = ATOMIC_INIT(1);

	if (!atomic_cas(&global_irq_init, 1, 0)) {
		return;
	}

	/* Shared irq config default to ctimer0. */
	NVIC_ClearPendingIRQ(CTIMER_IRQn);
	IRQ_CONNECT(CTIMER_IRQn, DT_IRQ(DT_INST_PARENT(0), priority), counter_ambiq_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(CTIMER_IRQn);
};
#endif

static uint32_t get_clock_cycles(uint32_t clock_sel)
{
	uint32_t ret = 0;

	switch (clock_sel) {
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	case 1:
		ret = 12000000;
		break;
	case 2:
		ret = 3000000;
		break;
	case 3:
		ret = 187500;
		break;
	case 4:
		ret = 47000;
		break;
	case 5:
		ret = 12000;
		break;
	case 6:
		ret = 32768;
		break;
	case 7:
		ret = 16384;
		break;
	case 8:
		ret = 2048;
		break;
	case 9:
		ret = 256;
		break;
	case 10:
		ret = 512;
		break;
	case 11:
		ret = 32;
		break;
	case 12:
	case 13:
		ret = 1;
		break;
	case 14:
		ret = 100;
		break;
	case 15: /* AM_HAL_CTIMER_HCLK_DIV4: 48MHz / 4 = 12MHz */
		ret = 12000000;
		break;
	case 16:
		ret = 8192;
		break;
	case 17:
		ret = 4096;
		break;
	case 18:
		ret = 1024;
		break;
#endif /* CONFIG_SOC_SERIES_APOLLO3X */

#if defined(CONFIG_SOC_SERIES_APOLLO4X) || defined(CONFIG_SOC_SERIES_APOLLO5X)
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		ret = 96000000 / (4 << (clock_sel * 2));
		break;
	case 6:
		ret = 1000;
		break;
#endif /* CONFIG_SOC_SERIES_APOLLO4X || CONFIG_SOC_SERIES_APOLLO5X */

#if defined(CONFIG_SOC_SERIES_APOLLO4X) || defined(CONFIG_SOC_APOLLO510) ||                        \
	defined(CONFIG_SOC_APOLLO510B)
	case 7:
		ret = 500;
		break;
	case 8:
		ret = 31;
		break;
	case 9:
		ret = 1;
		break;
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		ret = 32768 / (1 << (clock_sel - 10));
		break;
	case 16:
		ret = 256;
		break;
	case 17:
		ret = 100;
		break;
#endif /* CONFIG_SOC_SERIES_APOLLO4X || CONFIG_SOC_APOLLO510/B */

#if defined(CONFIG_SOC_APOLLO510) || defined(CONFIG_SOC_APOLLO510B)
	case 18:
		ret = 512;
		break;
	case 19:
	case 20:
	case 21:
	case 22:
	case 23:
	case 24:
		ret = 125000000 / (8 << (clock_sel - 19));
		break;
	case 25:
	case 26:
	case 27:
		ret = 48000000 / (1 << (clock_sel - 25));
		break;
#endif /* CONFIG_SOC_APOLLO510/B */

#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
	case 7:
		ret = 31;
		break;
	case 8:
		ret = 1;
		break;
	case 9:
		ret = 32768;
		break;
	case 10:
		ret = 8192;
		break;
	case 11:
		ret = 2048;
		break;
	case 12:
		ret = 1024;
		break;
	case 13:
		ret = 32;
		break;
	case 14:
		ret = 100;
		break;
	case 15:
		ret = 512;
		break;
	case 16:
		ret = 256;
		break;
	case 17: {
		uint32_t status;

		status =
			am_hal_clkmgr_clock_config_get(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, &ret, NULL);
		if (status != AM_HAL_STATUS_SUCCESS) {
			ret = 0; /* Fallback to default if query fails */
		}
		break;
	}
	case 18:
	case 19:
	case 20:
		ret = 48000000 / (1 << (clock_sel - 18));
		break;
	case 21:
	case 22:
	case 23:
	case 24: {
		am_hal_clkmgr_board_info_t board;
		am_hal_clkmgr_board_info_get(&board);
		ret = board.ui32ExtRefClkFreq / (1 << (clock_sel - 21));
	} break;
#endif /* CONFIG_SOC_APOLLO510L || CONFIG_SOC_APOLLO330P */

	default:
		ret = 32768;
		break;
	}

	return ret;
}

static int counter_ambiq_init(const struct device *dev)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	const struct counter_ambiq_config *cfg = dev->config;
	struct counter_ambiq_data *data = dev->data;

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	/* Timer configuration */
	am_hal_ctimer_config_t sContTimer;
	/* Create 32-bit timer */
	sContTimer.ui32Link = 1;
	/* Set up TimerA. */
	sContTimer.ui32TimerAConfig = (AM_HAL_CTIMER_FN_REPEAT | AM_HAL_CTIMER_INT_ENABLE |
				       (cfg->clk_src << CTIMER_CTRL0_TMRA0CLK_Pos));
	/* Set up TimerB. */
	sContTimer.ui32TimerBConfig = 0;

	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

	data->freq = get_clock_cycles(cfg->clk_src);

	am_hal_ctimer_clear(cfg->instance, AM_HAL_CTIMER_BOTH);
	am_hal_ctimer_config(cfg->instance, &sContTimer);
	counter_irq_config_func();
#else
	am_hal_timer_config_t tc;

	am_hal_timer_default_config_set(&tc);
	tc.eInputClock = cfg->clk_src;
	tc.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
	tc.ui32PatternLimit = 0;

	data->freq = get_clock_cycles(cfg->clk_src);

	if (am_hal_timer_config(cfg->instance, &tc) != AM_HAL_STATUS_SUCCESS) {
		k_spin_unlock(&lock, key);
		return -EIO;
	}
	cfg->irq_config_func();
#endif

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_start(const struct device *dev)
{
	const struct counter_ambiq_config *cfg = dev->config;
	k_spinlock_key_t key = k_spin_lock(&lock);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_ctimer_start(cfg->instance, AM_HAL_CTIMER_TIMERA);
#else
	am_hal_timer_start(cfg->instance);
#endif

	k_spin_unlock(&lock, key);

#if defined(CONFIG_PM_DEVICE)
	/*
	 * If using a high-frequency clock, prevent sleep while timer is running
	 * because HF clocks stop during sleep. LF clocks can continue in sleep.
	 */
	if (is_high_freq_clock(cfg->clk_src)) {
		struct counter_ambiq_data *data = dev->data;
		int ret;

		if (!data->pm_lock_held) {
			/* Block all sleep states that would stop high-frequency clocks */
			pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
			pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
			pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_DISK, PM_ALL_SUBSTATES);
#endif
			ret = pm_device_runtime_get(dev);
			if (ret < 0) {
				/* Unwind the policy locks on error */
				pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE,
							 PM_ALL_SUBSTATES);
				pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
				pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_DISK,
							 PM_ALL_SUBSTATES);
#endif
				/* Stop the timer since we can't prevent sleep */
				key = k_spin_lock(&lock);
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
				am_hal_ctimer_stop(cfg->instance, AM_HAL_CTIMER_BOTH);
#else
				am_hal_timer_stop(cfg->instance);
#endif
				k_spin_unlock(&lock, key);
				return ret;
			}
			data->pm_lock_held = true;
		}
	}
#endif

	return 0;
}

static int counter_ambiq_stop(const struct device *dev)
{
	const struct counter_ambiq_config *cfg = dev->config;

	k_spinlock_key_t key = k_spin_lock(&lock);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_ctimer_stop(cfg->instance, AM_HAL_CTIMER_BOTH);
#else
	am_hal_timer_stop(cfg->instance);
#endif

	k_spin_unlock(&lock, key);

#if defined(CONFIG_PM_DEVICE)
	/*
	 * If using a high-frequency clock, allow sleep now that timer is stopped.
	 */
	if (is_high_freq_clock(cfg->clk_src)) {
		struct counter_ambiq_data *data = dev->data;
		int ret;

		if (data->pm_lock_held) {
			ret = pm_device_runtime_put(dev);
			if (ret < 0) {
				LOG_WRN("Failed to release PM runtime reference: %d", ret);
				/* Continue with lock release to avoid PM usage count leak */
			}
			/* Release all sleep state locks */
			pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
			pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
			pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_DISK, PM_ALL_SUBSTATES);
#endif
			data->pm_lock_held = false;
		}
	}
#endif

	return 0;
}

static int counter_ambiq_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_ambiq_config *cfg = dev->config;

	k_spinlock_key_t key = k_spin_lock(&lock);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	*ticks = (am_hal_ctimer_read(cfg->instance, AM_HAL_CTIMER_TIMERA)) |
		 (am_hal_ctimer_read(cfg->instance, AM_HAL_CTIMER_TIMERB) << 16);
#else
	*ticks = am_hal_timer_read(cfg->instance);
#endif

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_set_alarm(const struct device *dev, uint8_t chan_id,
				   const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(chan_id);
	struct counter_ambiq_data *data = dev->data;
	const struct counter_ambiq_config *cfg = dev->config;
	uint32_t now;

	counter_ambiq_get_value(dev, &now);

	k_spinlock_key_t key = k_spin_lock(&lock);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0C0);
	am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0C0);

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0) {
		am_hal_ctimer_compare_set(cfg->instance, AM_HAL_CTIMER_BOTH, 0,
					  now + alarm_cfg->ticks);
	} else {
		am_hal_ctimer_compare_set(cfg->instance, AM_HAL_CTIMER_BOTH, 0, alarm_cfg->ticks);
	}
#else
	/* Enable interrupt, due to counter_ambiq_cancel_alarm() disables it*/
	am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(cfg->instance, AM_HAL_TIMER_COMPARE1));
	am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(cfg->instance, AM_HAL_TIMER_COMPARE1));

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0) {
		am_hal_timer_compare1_set(cfg->instance, now + alarm_cfg->ticks);
	} else {
		am_hal_timer_compare1_set(cfg->instance, alarm_cfg->ticks);
	}
#endif

	data->user_data = alarm_cfg->user_data;
	data->callback = alarm_cfg->callback;

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	ARG_UNUSED(chan_id);
	const struct counter_ambiq_config *cfg = dev->config;
	k_spinlock_key_t key = k_spin_lock(&lock);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_ctimer_int_disable(AM_HAL_CTIMER_INT_TIMERA0C0);
	/* Reset the compare register */
	am_hal_ctimer_compare_set(cfg->instance, AM_HAL_CTIMER_BOTH, 0, 0);
#else
	am_hal_timer_interrupt_disable(AM_HAL_TIMER_MASK(cfg->instance, AM_HAL_TIMER_COMPARE1));
	/* Reset the compare register */
	am_hal_timer_compare1_set(cfg->instance, 0);
#endif

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_set_top_value(const struct device *dev, const struct counter_top_cfg *cfg)
{
	const struct counter_ambiq_config *config = dev->config;

	if (cfg->ticks != config->counter_info.max_top_value) {
		return -ENOTSUP;
	} else {
		return 0;
	}
}

static uint32_t counter_ambiq_get_pending_int(const struct device *dev)
{
	return 0;
}

static uint32_t counter_ambiq_get_top_value(const struct device *dev)
{
	const struct counter_ambiq_config *config = dev->config;

	return config->counter_info.max_top_value;
}

static uint32_t counter_ambiq_get_freq(const struct device *dev)
{
	const struct counter_ambiq_data *data = dev->data;

	return data->freq;
}

#if defined(CONFIG_PM_DEVICE)
static int counter_ambiq_pm_action(const struct device *dev, enum pm_device_action action)
{
	ARG_UNUSED(dev);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/*
		 * For high-frequency clocks, PM policy locks prevent this from being called
		 * while timer is running. For low-frequency clocks, timer continues in sleep
		 * and retains configuration, so no action needed.
		 */
		break;
	case PM_DEVICE_ACTION_RESUME:
		/*
		 * Timer configuration is retained during sleep. If timer was using a
		 * low-frequency clock and running, it continued during sleep.
		 * No restoration needed.
		 */
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static DEVICE_API(counter, counter_api) = {
	.start = counter_ambiq_start,
	.stop = counter_ambiq_stop,
	.get_value = counter_ambiq_get_value,
	.set_alarm = counter_ambiq_set_alarm,
	.cancel_alarm = counter_ambiq_cancel_alarm,
	.set_top_value = counter_ambiq_set_top_value,
	.get_pending_int = counter_ambiq_get_pending_int,
	.get_top_value = counter_ambiq_get_top_value,
	.get_freq = counter_ambiq_get_freq,
};

#define APOLLO3_HANDLE_SHARED_TIMER_IRQ(n)                                                         \
	static const struct device *const dev_##n = DEVICE_DT_INST_GET(n);                         \
	struct counter_ambiq_data *const data_##n = dev_##n->data;                                 \
	uint32_t status_##n = CTIMERn(n)->INTSTAT;                                                 \
	status_##n &= CTIMERn(n)->INTEN;                                                           \
	if (status_##n) {                                                                          \
		CTIMERn(n)->INTCLR = AM_HAL_CTIMER_INT_TIMERA0C0;                                  \
		counter_ambiq_get_value(dev_##n, &now);                                            \
		if (data_##n->callback) {                                                          \
			data_##n->callback(dev_##n, 0, now, data_##n->user_data);                  \
		}                                                                                  \
	}

static void counter_ambiq_isr(void *arg)
{
	uint32_t now = 0;

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	ARG_UNUSED(arg);

	DT_INST_FOREACH_STATUS_OKAY(APOLLO3_HANDLE_SHARED_TIMER_IRQ)
#else
	const struct device *dev = (const struct device *)arg;
	struct counter_ambiq_data *data = dev->data;
	const struct counter_ambiq_config *cfg = dev->config;

	am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(cfg->instance, AM_HAL_TIMER_COMPARE1));
	counter_ambiq_get_value(dev, &now);

	if (data->callback) {
		data->callback(dev, 0, now, data->user_data);
	}
#endif
}

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
/* Apollo3 counters share the same irq number, connect irq here will cause build error, so we
 * leave this function blank here and do it in counter_irq_config_func
 */
#define AMBIQ_COUNTER_CONFIG_FUNC(idx)                                                             \
	static void counter_irq_config_func_##idx(void)                                            \
	{                                                                                          \
	}
#else
#define AMBIQ_COUNTER_CONFIG_FUNC(idx)                                                             \
	static void counter_irq_config_func_##idx(void)                                            \
	{                                                                                          \
		NVIC_ClearPendingIRQ(DT_IRQN(DT_INST_PARENT(idx)));                                \
		IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(idx)), DT_IRQ(DT_INST_PARENT(idx), priority),   \
			    counter_ambiq_isr, DEVICE_DT_INST_GET(idx), 0);                        \
		irq_enable(DT_IRQN(DT_INST_PARENT(idx)));                                          \
	}
#endif

#define AMBIQ_COUNTER_INIT(idx)                                                                    \
	BUILD_ASSERT(DT_CHILD_NUM_STATUS_OKAY(DT_INST_PARENT(idx)) == 1,                           \
		     "Too many children for Timer!");                                              \
	static void counter_irq_config_func_##idx(void);                                           \
	static struct counter_ambiq_data counter_data_##idx;                                       \
	static const struct counter_ambiq_config counter_config_##idx = {                          \
		.instance = (DT_REG_ADDR(DT_INST_PARENT(idx)) - SOC_TIMER_BASE) /                  \
			    DT_REG_SIZE(DT_INST_PARENT(idx)),                                      \
		.clk_src = DT_ENUM_IDX(DT_INST_PARENT(idx), clk_source),                           \
		.counter_info = {.max_top_value = UINT32_MAX,                                      \
				 .flags = COUNTER_CONFIG_INFO_COUNT_UP,                            \
				 .channels = 1},                                                   \
		.irq_config_func = counter_irq_config_func_##idx,                                  \
	};                                                                                         \
	AMBIQ_COUNTER_CONFIG_FUNC(idx)                                                             \
	IF_ENABLED(CONFIG_PM_DEVICE,                                                            \
		   (PM_DEVICE_DT_INST_DEFINE(idx, counter_ambiq_pm_action);)) \
	DEVICE_DT_INST_DEFINE(idx, counter_ambiq_init, PM_DEVICE_DT_INST_GET(idx),         \
			      &counter_data_##idx, &counter_config_##idx, PRE_KERNEL_1,            \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_api);
DT_INST_FOREACH_STATUS_OKAY(AMBIQ_COUNTER_INIT);
