/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Ambiq I2S power-measurement sample
 *
 * Zephyr counterpart to AmbiqSuite
 * boards/common5/examples/audio/i2s_power.
 *
 * Boot with UART console, print status, then suspend the console and apply
 * the AmbiqSuite minimum memory profile so VDD_MCU reflects I2S TX running
 * power. Unused peripherals are left disabled via the board overlay.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/printk.h>

#include <am_mcu_apollo.h>

LOG_MODULE_REGISTER(ambiq_i2s_power, LOG_LEVEL_INF);

#define I2S_TX DT_ALIAS(i2s_tx)

/* Match AmbiqSuite i2s_power defaults (PROFILE_I2S_RUN). */
#define SAMPLE_BIT_WIDTH   32U
#define NUMBER_OF_CHANNELS 2U
#define SAMPLE_FREQ_HZ     16000U
#define SAMPLE_BLOCKS_MS   10U
#define TIMEOUT_MS         1000U
#define SINE_AMP           0x400000
#define CONSOLE_DRAIN_MS   50

#define BYTES_PER_SAMPLE sizeof(int32_t)
#define SAMPLES_PER_BLOCK                                                                          \
	((SAMPLE_FREQ_HZ * SAMPLE_BLOCKS_MS / 1000U) * NUMBER_OF_CHANNELS)
#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT 4U
#define SINE_PERIOD_SAMPLES 8U

K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

static const int32_t sine_period[SINE_PERIOD_SAMPLES] = {
	0,
	(int32_t)(SINE_AMP * 0.70710678f),
	SINE_AMP,
	(int32_t)(SINE_AMP * 0.70710678f),
	0,
	-(int32_t)(SINE_AMP * 0.70710678f),
	-SINE_AMP,
	-(int32_t)(SINE_AMP * 0.70710678f),
};

static uint32_t sine_phase;

/**
 * Apply AmbiqSuite i2s_power minimum memory / APBDMA profile.
 *
 * Note: AmbiqSuite also calls AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL. Under
 * Zephyr that is replaced by the board overlay (unused nodes disabled) plus
 * runtime console suspend after the banner.
 */
static void ambiq_i2s_power_memory_profile(void)
{
	am_hal_pwrctrl_sram_memcfg_t sram_cfg = {
#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
		.eSRAMCfg = AM_HAL_PWRCTRL_SRAM_0P75M,
		.eSRAMRetain = AM_HAL_PWRCTRL_SRAM_0P75M,
#else
		.eSRAMCfg = AM_HAL_PWRCTRL_SRAM_1M,
		.eSRAMRetain = AM_HAL_PWRCTRL_SRAM_1M,
#endif
		.eActiveWithMCU = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithGFX = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithDISP = AM_HAL_PWRCTRL_SRAM_NONE,
	};

	am_hal_pwrctrl_mcu_memory_config_t mcu_cfg = {
		.eROMMode = AM_HAL_PWRCTRL_ROM_AUTO,
#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
		.eDTCMCfg = AM_HAL_PWRCTRL_DTCM128K,
		.eNVMCfg = AM_HAL_PWRCTRL_NVM,
#else
		.eDTCMCfg = AM_HAL_PWRCTRL_ITCM128K_DTCM256K,
		.eNVMCfg = AM_HAL_PWRCTRL_NVM0_ONLY,
#endif
		.eRetainDTCM = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
		.bKeepNVMOnInDeepSleep = false,
	};

	/* Save ~35 uW (AmbiqSuite i2s_power comment). */
	MCUCTRL->APBDMACTRL_b.HYSTERESIS = 0x0;

	am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP, 0);
	MCUCTRL->XTALCTRL = 0;
	am_hal_rtc_osc_disable();

	VCOMP->PWDKEY = VCOMP_PWDKEY_PWDKEY_Key;

	am_hal_pwrctrl_mcu_memory_config(&mcu_cfg);
	am_hal_pwrctrl_sram_config(&sram_cfg);

	k_msleep(10);
}

static void fill_sine_block(void *mem_block)
{
	int32_t *buf = mem_block;

	for (uint32_t i = 0; i < SAMPLES_PER_BLOCK; i += NUMBER_OF_CHANNELS) {
		int32_t sample = sine_period[sine_phase % SINE_PERIOD_SAMPLES];

		buf[i] = sample;
		buf[i + 1] = sample;
		sine_phase++;
	}
}

static void ambiq_i2s_power_shutdown_console(void)
{
	const struct device *cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	struct log_backend const *backend;
	int ret;

	printk("Shutting down console for power measurement\n");

	/* Drain UART TX / log processing before suspending the device. */
	(void)log_process();
	k_msleep(CONSOLE_DRAIN_MS);

	backend = log_backend_get_by_name("uart");
	if (backend != NULL) {
		log_backend_disable(backend);
	}

	if (!device_is_ready(cons)) {
		return;
	}

	ret = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
	if (ret < 0) {
		/* Avoid further console use; measurement path continues. */
		ARG_UNUSED(ret);
	}
}

static int ambiq_i2s_power_start_tx(const struct device *i2s_dev)
{
	struct i2s_config cfg;
	void *tx_block;
	int ret;
	int primed;

	cfg.word_size = SAMPLE_BIT_WIDTH;
	cfg.channels = NUMBER_OF_CHANNELS;
	cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	cfg.frame_clk_freq = SAMPLE_FREQ_HZ;
	cfg.mem_slab = &mem_slab;
	cfg.block_size = BLOCK_SIZE;
	cfg.timeout = TIMEOUT_MS;

	ret = i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		return ret;
	}

	for (primed = 0; primed < 2; primed++) {
		ret = k_mem_slab_alloc(&mem_slab, &tx_block, K_MSEC(TIMEOUT_MS));
		if (ret < 0) {
			return ret;
		}
		fill_sine_block(tx_block);
		ret = i2s_write(i2s_dev, tx_block, BLOCK_SIZE);
		if (ret < 0) {
			k_mem_slab_free(&mem_slab, tx_block);
			return ret;
		}
	}

	return i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
}

int main(void)
{
	const struct device *const i2s_dev = DEVICE_DT_GET(I2S_TX);
	void *tx_block;
	int ret;

	if (!device_is_ready(i2s_dev)) {
		LOG_ERR("I2S device not ready");
		return 0;
	}

	LOG_INF("Ambiq I2S power example");
	LOG_INF("I2S0 TX %u Hz / %u ch / %u-bit (AmbiqSuite i2s_power profile)", SAMPLE_FREQ_HZ,
		NUMBER_OF_CHANNELS, SAMPLE_BIT_WIDTH);
	LOG_INF("Unused peripherals disabled in overlay; console stays up for banner only");

	ambiq_i2s_power_shutdown_console();

	ambiq_i2s_power_memory_profile();

	ret = ambiq_i2s_power_start_tx(i2s_dev);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = k_mem_slab_alloc(&mem_slab, &tx_block, K_MSEC(TIMEOUT_MS));
		if (ret < 0) {
			break;
		}

		fill_sine_block(tx_block);
		ret = i2s_write(i2s_dev, tx_block, BLOCK_SIZE);
		if (ret < 0) {
			k_mem_slab_free(&mem_slab, tx_block);
			break;
		}
	}

	(void)i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_STOP);
	return 0;
}
