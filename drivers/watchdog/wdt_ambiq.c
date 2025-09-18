/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_watchdog

#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#ifdef CONFIG_TASK_WDT
#include <zephyr/task_wdt/task_wdt.h>
#endif

#include <errno.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_ambiq, CONFIG_WDT_LOG_LEVEL);

typedef void (*ambiq_wdt_cfg_func_t)(void);

struct wdt_ambiq_config {
	uint32_t base;
	uint32_t irq_num;
	uint32_t clk_freq;
	ambiq_wdt_cfg_func_t cfg_func;
};

struct wdt_ambiq_data {
	wdt_callback_t callback;
	uint32_t timeout;
	bool reset;
#ifdef CONFIG_TASK_WDT
	bool task_wdt_initialized;
	int task_wdt_channel_id;
	bool channel_creation_in_progress;  /* Prevent re-entrant channel creation */
#endif
};

uint32_t wdt_ambiq_clk_select[] =
#if defined(CONFIG_SOC_SERIES_APOLLO3X) || defined(CONFIG_SOC_SERIES_APOLLO4X)
	{128, 16, 1};
#else
	{128, 16, 1, 32768, 16384};
#endif

static void wdt_ambiq_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct wdt_ambiq_data *data = dev->data;

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_wdt_int_clear();
#else
	uint32_t status;
	am_hal_wdt_interrupt_status_get(AM_HAL_WDT_MCU, &status, true);
	am_hal_wdt_interrupt_clear(AM_HAL_WDT_MCU, status);
#endif

	if (data->callback) {
		data->callback(dev, 0);
	}
}

#ifdef CONFIG_TASK_WDT
/* Task watchdog callback wrapper */
static void wdt_ambiq_task_callback(int channel_id, void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct wdt_ambiq_data *data = dev->data;

	if (data->callback) {
		data->callback(dev, channel_id);
	}
}
#endif

static int wdt_ambiq_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_ambiq_config *dev_cfg = dev->config;
	struct wdt_ambiq_data *data = dev->data;
	am_hal_wdt_config_t cfg;
	
#ifdef CONFIG_TASK_WDT
	LOG_INF("Setup called: dev=%p, initialized=%d, channel_id=%d, in_progress=%d, timeout=%d", 
	        dev, data->task_wdt_initialized, data->task_wdt_channel_id, 
	        data->channel_creation_in_progress, data->timeout);
	
	/* Only create task watchdog channel once (re-entry protection) */
	if (data->task_wdt_initialized && data->task_wdt_channel_id == -1 && 
	    !data->channel_creation_in_progress) {
		
		data->channel_creation_in_progress = true;  /* Set flag to prevent re-entry */
		LOG_INF("Creating task watchdog channel during setup");
		uint32_t timeout_ms = data->timeout * 1000 / dev_cfg->clk_freq;  /* Convert back to ms */
		
		LOG_INF("About to call task_wdt_add with %u ms", timeout_ms);
		int channel_id = task_wdt_add(timeout_ms, wdt_ambiq_task_callback, (void *)dev);
		
		LOG_INF("task_wdt_add returned: %d", channel_id);
		if (channel_id >= 0) {
			data->task_wdt_channel_id = channel_id;
			LOG_INF("Set data->task_wdt_channel_id = %d", data->task_wdt_channel_id);
			LOG_INF("Created task watchdog channel %d (%u ms)", channel_id, timeout_ms);
		} else {
			LOG_WRN("Failed to create task watchdog channel (%d), using hardware only", channel_id);
		}
		data->channel_creation_in_progress = false;  /* Clear flag when done */
		
	} else {
		LOG_INF("Skipping task watchdog channel creation: initialized=%d, channel_id=%d, in_progress=%d", 
		        data->task_wdt_initialized, data->task_wdt_channel_id, data->channel_creation_in_progress);
	}
#endif

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	uint32_t ui32ClockSource = AM_HAL_WDT_LFRC_CLK_DEFAULT;

	if (dev_cfg->clk_freq == 128) {
		ui32ClockSource = AM_HAL_WDT_LFRC_CLK_128HZ;
	} else if (dev_cfg->clk_freq == 16) {
		ui32ClockSource = AM_HAL_WDT_LFRC_CLK_16HZ;
	} else if (dev_cfg->clk_freq == 1) {
		ui32ClockSource = AM_HAL_WDT_LFRC_CLK_1HZ;
	}
	cfg.ui32Config = ui32ClockSource | _VAL2FLD(WDT_CFG_RESEN, data->reset) |
			 AM_HAL_WDT_ENABLE_INTERRUPT;
	cfg.ui16InterruptCount = data->timeout;
	cfg.ui16ResetCount = data->timeout;
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);
	am_hal_wdt_init(&cfg);
	am_hal_wdt_int_enable();
	am_hal_wdt_start();
#else
	if (dev_cfg->clk_freq == 128) {
		cfg.eClockSource = AM_HAL_WDT_128HZ;
	} else if (dev_cfg->clk_freq == 16) {
		cfg.eClockSource = AM_HAL_WDT_16HZ;
	} else if (dev_cfg->clk_freq == 1) {
		cfg.eClockSource = AM_HAL_WDT_1HZ;
#if defined(CONFIG_SOC_SERIES_APOLLO5X)
	} else if (dev_cfg->clk_freq == 32768) {
		cfg.eClockSource = AM_HAL_WDT_XTAL_HS;
	} else if (dev_cfg->clk_freq == 16384) {
		cfg.eClockSource = AM_HAL_WDT_XTAL_HS_DIV2;
	}
#else
	}
#endif

	cfg.bInterruptEnable = true;
	cfg.ui32InterruptValue = data->timeout;
	cfg.bResetEnable = data->reset;
	cfg.ui32ResetValue = data->timeout;
#if defined(CONFIG_SOC_SERIES_APOLLO4X)
	cfg.bAlertOnDSPReset = false;
#endif
	am_hal_wdt_config(AM_HAL_WDT_MCU, &cfg);
	am_hal_wdt_interrupt_enable(AM_HAL_WDT_MCU, AM_HAL_WDT_INTERRUPT_MCU);
	am_hal_wdt_start(AM_HAL_WDT_MCU, false);
#endif
	return 0;
}

static int wdt_ambiq_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_wdt_halt();
#else
	am_hal_wdt_stop(AM_HAL_WDT_MCU);
#endif
	return 0;
}

static int wdt_ambiq_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	const struct wdt_ambiq_config *dev_cfg = dev->config;
	struct wdt_ambiq_data *data = dev->data;
	uint32_t timeout = cfg->window.max * dev_cfg->clk_freq / 1000;

	if (cfg->window.min != 0U || cfg->window.max == 0U) {
		return -EINVAL;
	}

	if (timeout == 0 || timeout > 256) {
		return -EINVAL;
	}

	data->timeout = timeout;
	data->callback = cfg->callback;

	switch (cfg->flags) {
	case WDT_FLAG_RESET_CPU_CORE:
	case WDT_FLAG_RESET_SOC:
		data->reset = true;
		break;
	case WDT_FLAG_RESET_NONE:
		data->reset = false;
		break;
	default:
		LOG_ERR("Unsupported watchdog config flag");
		return -EINVAL;
	}

#ifdef CONFIG_TASK_WDT
	if (data->task_wdt_initialized) {
		/* Multi-stage mode - store config for later setup in wdt_setup() */
		LOG_INF("Task watchdog config stored (%u ms)", cfg->window.max);
		return 0;  /* Return success, actual channel will be created in setup */
	}
#endif

	/* Single-stage mode - original behavior */
	LOG_INF("Using single-stage mode (%u ms)", cfg->window.max);
	return 0;
}

static int wdt_ambiq_feed(const struct device *dev, int channel_id)
{
#ifdef CONFIG_TASK_WDT
	struct wdt_ambiq_data *data = dev->data;

	if (data->task_wdt_initialized && data->task_wdt_channel_id >= 0) {
		/* Multi-stage mode - feed the specific task watchdog channel */
		LOG_DBG("Feeding task watchdog channel %d", channel_id);
		return task_wdt_feed(data->task_wdt_channel_id);
	}
#endif

	/* Single-stage mode - original behavior */
	LOG_DBG("Feeding hardware watchdog directly");
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	am_hal_wdt_restart();
#else
	am_hal_wdt_restart(AM_HAL_WDT_MCU);
#endif

	return 0;
}

/*
 * Multi-stage watchdog support:
 * 
 * When CONFIG_TASK_WDT is enabled, this driver provides multi-stage watchdog
 * functionality using the Zephyr task watchdog subsystem. The hardware watchdog
 * acts as a fallback safety mechanism.
 * 
 * When CONFIG_TASK_WDT is disabled, the driver operates in single-stage mode
 * using only the hardware watchdog.
 * 
 * Multi-stage mode features:
 * - Multiple independent watchdog channels
 * - Software timer management with hardware fallback
 * - Suspend/resume: call task_wdt_suspend() and task_wdt_resume()
 * - Channel deletion: call task_wdt_delete(channel_id)
 */


static int wdt_ambiq_init(const struct device *dev)
{
	const struct wdt_ambiq_config *dev_cfg = dev->config;
	uint8_t clk_index = sizeof(wdt_ambiq_clk_select) / sizeof(uint32_t);

	LOG_INF("Starting watchdog init");

	for (uint8_t i = 0; i < clk_index; i++) {
		if (dev_cfg->clk_freq == wdt_ambiq_clk_select[i]) {
			break;
		}
		if (i == clk_index - 1) {
			return -ENOTSUP;
		}
	}

	LOG_INF("Clock validation passed");
	NVIC_ClearPendingIRQ(dev_cfg->irq_num);
	dev_cfg->cfg_func();
	irq_enable(dev_cfg->irq_num);
	LOG_INF("Hardware setup complete");

#ifdef CONFIG_TASK_WDT
	struct wdt_ambiq_data *data = dev->data;
	
	LOG_INF("About to initialize task watchdog");
	/* Initialize task watchdog - pass hardware watchdog only if fallback enabled */
#ifdef CONFIG_TASK_WDT_HW_FALLBACK
	int ret = task_wdt_init(dev);  /* Use this device as hardware fallback */
#else
	int ret = task_wdt_init(NULL); /* No hardware fallback */
#endif
	LOG_INF("Task watchdog init returned: %d", ret);
	if (ret == 0) {
		data->task_wdt_initialized = true;
		data->task_wdt_channel_id = -1;
		data->channel_creation_in_progress = false;
		LOG_INF("Multi-stage watchdog initialized");
	} else {
		data->task_wdt_initialized = false;
		data->task_wdt_channel_id = -1;
		data->channel_creation_in_progress = false;
		LOG_WRN("Task watchdog init failed (%d), using single-stage mode", ret);
	}
#endif

	return 0;
}

static DEVICE_API(wdt, wdt_ambiq_driver_api) = {
	.setup = wdt_ambiq_setup,
	.disable = wdt_ambiq_disable,
	.install_timeout = wdt_ambiq_install_timeout,
	.feed = wdt_ambiq_feed,
};

#define AMBIQ_WDT_INIT(n)                                                                          \
	static struct wdt_ambiq_data wdt_ambiq_data##n;                                            \
	static void ambiq_wdt_cfg_func_##n(void)                                                   \
	{                                                                                          \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), wdt_ambiq_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
	};                                                                                         \
	static const struct wdt_ambiq_config wdt_ambiq_config##n = {                               \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clk_freq = DT_INST_PROP(n, clock_frequency),                                      \
		.irq_num = DT_INST_IRQN(n),                                                        \
		.cfg_func = ambiq_wdt_cfg_func_##n};                                               \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, wdt_ambiq_init, NULL, &wdt_ambiq_data##n, &wdt_ambiq_config##n,   \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                    \
			      &wdt_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_WDT_INIT)
