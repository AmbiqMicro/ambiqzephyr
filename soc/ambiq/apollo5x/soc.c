/*
 * Copyright (c) 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>
#include <zephyr/mem_mgmt/mem_attr.h>
#ifdef CONFIG_CACHE_MANAGEMENT
#include <zephyr/dt-bindings/memory-attr/memory-attr-arm.h>
#endif /* CONFIG_CACHE_MANAGEMENT */

#ifdef CONFIG_NOCACHE_MEMORY
#include <zephyr/linker/linker-defs.h>
#endif /* CONFIG_NOCACHE_MEMORY */

#if (CONFIG_COREMARK == 1)
#include "icache_prefill.h"
#endif

#include <zephyr/dt-bindings/power/ambiq_power.h>

#include <soc.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

#if (defined(CONFIG_BOARD_APOLLO510DL_EVB) || defined(CONFIG_BOARD_APOLLO330MP_EVB)) && \
	defined(CONFIG_LOG_BACKEND_SWO)
#error "LOG_BACKEND_SWO is not supported on this board configuration"
#endif

static am_hal_pwrctrl_mcu_mode_e ambiq_perf_mode_to_hal(uint32_t mode)
{
#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
	switch (mode) {
	case AMBIQ_POWER_MODE_LOW_POWER:
		return AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER;
	case AMBIQ_POWER_MODE_HIGH_PERFORMANCE:
		return AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1;
	case AMBIQ_POWER_MODE_HIGH_PERFORMANCE_2:
		return AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2;
	default:
		return AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER;
	}
#else
	switch (mode) {
	case AMBIQ_POWER_MODE_LOW_POWER:
		return AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER;
	case AMBIQ_POWER_MODE_HIGH_PERFORMANCE_2:
		/* fall through */
	case AMBIQ_POWER_MODE_HIGH_PERFORMANCE:
		return AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE;
	default:
		return AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER;
	}
#endif
}

int apollo5x_set_performance_mode(uint32_t mode)
{
	uint32_t status;
	am_hal_pwrctrl_mcu_mode_e current_mode;
	am_hal_pwrctrl_mcu_mode_e target_mode = ambiq_perf_mode_to_hal(mode);

	status = am_hal_pwrctrl_mcu_mode_status(&current_mode);
	if (status != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("Failed to read MCU mode: 0x%x", status);
		return -EIO;
	}

	if (current_mode == target_mode) {
		return 0;
	}

	status = am_hal_pwrctrl_mcu_mode_select(target_mode);
	if (status != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("Failed to switch MCU mode (%d): 0x%x", mode, status);
		return -EIO;
	}

	return 0;
}

#define SCRATCH0_OEM_RCV_RETRY_MAGIC 0xA86

#if IS_ENABLED(CONFIG_SOC_AMBIQ_APOLLO5X_BLE_LP) &&                                                \
	(defined(CONFIG_SOC_APOLLO330P) || defined(CONFIG_SOC_APOLLO510L))
/*
 * Mirrors AmbiqSuite boards/apollo330mP_evb/examples/power/ble_freertos_fit_lp
 * after am_hal_pwrctrl_low_power_init(): MCU sleeps deeply between BLE events while
 * RSS (NETAOL) and retained SRAM stay configured for IPC/stack.
 */
static void ambiq_apollo5x_ble_lp_fit_early_init(void)
{
	am_hal_pwrctrl_periph_e periph;

	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_RTC_SEL_LFRC, NULL);
	am_hal_rtc_osc_select(AM_HAL_RTC_OSC_LFRC);
	am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP, NULL);
	MCUCTRL->XTALCTRL = 0;
	am_hal_rtc_osc_disable();

	for (periph = AM_HAL_PWRCTRL_PERIPH_IOSFD0; periph < AM_HAL_PWRCTRL_PERIPH_MAX; periph++) {
		if (periph == AM_HAL_PWRCTRL_PERIPH_NETAOL) {
			break;
		}
		am_hal_pwrctrl_periph_disable(periph);
	}

	MCUCTRL->DBGCTRL = 0;

	am_hal_pwrctrl_mcu_memory_config_t mcu_mem = {
		.eROMMode = AM_HAL_PWRCTRL_ROM_AUTO,
		.eDTCMCfg = AM_HAL_PWRCTRL_DTCM128K,
		.eRetainDTCM = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
		.eNVMCfg = AM_HAL_PWRCTRL_NVM,
		.bKeepNVMOnInDeepSleep = false,
	};

	am_hal_pwrctrl_mcu_memory_config(&mcu_mem);

	MCUCTRL->MRAMCRYPTOPWRCTRL_b.MRAM0LPREN = 1;
	MCUCTRL->MRAMCRYPTOPWRCTRL_b.MRAM0SLPEN = 0;
	MCUCTRL->MRAMCRYPTOPWRCTRL_b.MRAM0PWRCTRL = 0;

	am_hal_cachectrl_icache_disable();
	am_hal_cachectrl_dcache_disable();

	am_hal_pwrctrl_pwrmodctl_cpdlp_t cpdlp = {
		.eRlpConfig = AM_HAL_PWRCTRL_RLP_OFF,
		.eElpConfig = AM_HAL_PWRCTRL_ELP_OFF,
		.eClpConfig = AM_HAL_PWRCTRL_CLP_OFF,
	};

	am_hal_pwrctrl_pwrmodctl_cpdlp_config(cpdlp);
	am_hal_cachectrl_caches_power_control(false);

	CLKGEN->OCTRL_b.SECURERTCOSEL = 1;
	CLKGEN->OCTRL_b.RTCOSEL = 1;
	CLKGEN->MISC_b.CM4DAXICLKGATEEN = 1;

	am_hal_pwrctrl_sram_memcfg_t sram_mem = {
		.eSRAMCfg = AM_HAL_PWRCTRL_SRAM_0P75M,
		.eActiveWithMCU = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithGFX = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithDISP = AM_HAL_PWRCTRL_SRAM_NONE,
		.eSRAMRetain = AM_HAL_PWRCTRL_SRAM_0P75M,
	};

	am_hal_pwrctrl_sram_config(&sram_mem);
}
#endif

void soc_early_init_hook(void)
{
	/* Enable Loop and branch info cache */
	SCB->CCR |= SCB_CCR_LOB_Msk;
	__DSB();
	__ISB();

	if ((MCUCTRL->SCRATCH0 >> 20) == SCRATCH0_OEM_RCV_RETRY_MAGIC) {
		/*
		 * Clear the scratch register
		 */
		MCUCTRL->SCRATCH0 = 0x00;
	}

	/* Internal timer15 for SPOT manager */
	IRQ_CONNECT(TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A, 0, hal_internal_timer_isr, 0, 0);

	/* Initialize for low power in the power control block */
	am_hal_pwrctrl_low_power_init();

	/* Enable SIMOBUCK for the apollo5 Family */
	am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT, NULL);

	/*
	 * Set default temperature for spotmgr to room temperature
	 */
	am_hal_pwrctrl_temp_thresh_t dummy;

	am_hal_pwrctrl_temp_update(25.0f, &dummy);

#if IS_ENABLED(CONFIG_SOC_AMBIQ_APOLLO5X_BLE_LP) &&                                                \
	(defined(CONFIG_SOC_APOLLO330P) || defined(CONFIG_SOC_APOLLO510L))
	ambiq_apollo5x_ble_lp_fit_early_init();
#elif (CONFIG_COREMARK == 1)
	/* Enable Icache*/
	sys_cache_instr_enable();

	/* Enable Dcache */
	sys_cache_data_enable();

	am_hal_pwrctrl_pwrmodctl_cpdlp_t sDefaultCpdlpConfig = {
		.eRlpConfig = AM_HAL_PWRCTRL_RLP_ON,
		.eElpConfig = AM_HAL_PWRCTRL_ELP_RET,
		.eClpConfig = AM_HAL_PWRCTRL_CLP_ON};

	am_hal_pwrctrl_pwrmodctl_cpdlp_config(sDefaultCpdlpConfig);

	/* Clear 64KB of iCache before starting Coremark */
	icache_prefill();

	/* Use LFRC instead of XT */
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_RTC_SEL_LFRC, 0);
	am_hal_rtc_osc_select(AM_HAL_RTC_OSC_LFRC);

	/* Configure XTAL for deepsleep */
	am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP, 0);
	MCUCTRL->XTALCTRL = 0;
	am_hal_rtc_osc_disable();

	VCOMP->PWDKEY = VCOMP_PWDKEY_PWDKEY_Key;
	/* Temporary fix to set DBGCTRL Register to 0 */
	MCUCTRL->DBGCTRL = 0;

	/* Powering down various peripheral power domains */
	am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_DEBUG);
	am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
	am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);

#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
	/* Using LFRC instead of XT for RTC Clock Source */
	CLKGEN->OCTRL_b.SECURERTCOSEL = 1;
	CLKGEN->OCTRL_b.RTCOSEL = 1;

	/* Clk Gating for CM4 Processor */
	CLKGEN->MISC_b.CM4DAXICLKGATEEN = 1;
#endif

	am_hal_pwrctrl_sram_memcfg_t SRAMMemCfg = {.eSRAMCfg = AM_HAL_PWRCTRL_SRAM_NONE,
						   .eActiveWithMCU = AM_HAL_PWRCTRL_SRAM_NONE,
						   .eActiveWithGFX = AM_HAL_PWRCTRL_SRAM_NONE,
						   .eActiveWithDISP = AM_HAL_PWRCTRL_SRAM_NONE,
						   .eSRAMRetain = AM_HAL_PWRCTRL_SRAM_NONE};

	am_hal_pwrctrl_mcu_memory_config_t McuMemCfg = {
		.eROMMode = AM_HAL_PWRCTRL_ROM_AUTO,
#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
		.eDTCMCfg = AM_HAL_PWRCTRL_DTCM128K,
#else
		.eDTCMCfg = AM_HAL_PWRCTRL_ITCM32K_DTCM128K,
#endif
		.eRetainDTCM = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
		.eNVMCfg = AM_HAL_PWRCTRL_NVM,
#else
		.eNVMCfg = AM_HAL_PWRCTRL_NVM0_ONLY,
#endif
		.bKeepNVMOnInDeepSleep = false};

	am_hal_pwrctrl_mcu_memory_config(&McuMemCfg);

	MCUCTRL->MRAMCRYPTOPWRCTRL_b.MRAM0LPREN = 1;
	MCUCTRL->MRAMCRYPTOPWRCTRL_b.MRAM0SLPEN = 0;
	MCUCTRL->MRAMCRYPTOPWRCTRL_b.MRAM0PWRCTRL = 1;

	/* Disable SRAM */
	am_hal_pwrctrl_sram_config(&SRAMMemCfg);
#else
	/* Default: caches on (see CONFIG_SOC_AMBIQ_APOLLO5X_BLE_LP for FIT-style BLE low power). */
	sys_cache_instr_enable();
	sys_cache_data_enable();
#ifdef CONFIG_CORTEX_M_DWT
	am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DEBUG);
#endif
#endif
}

#if CONFIG_CACHE_MANAGEMENT
bool buf_in_nocache(uintptr_t buf, size_t len_bytes)
{
	bool buf_within_nocache = false;

	if (buf == 0 || len_bytes == 0) {
		return buf_within_nocache;
	}

#if CONFIG_NOCACHE_MEMORY
	/* Check if buffer is in nocache region defined by the linker */
	buf_within_nocache = (buf >= ((uintptr_t)_nocache_ram_start)) &&
			     ((buf + len_bytes - 1) <= ((uintptr_t)_nocache_ram_end));
	if (buf_within_nocache) {
		return true;
	}
#endif /* CONFIG_NOCACHE_MEMORY */

	/* Check if buffer is in nocache memory region defined in DT */
	buf_within_nocache =
		mem_attr_check_buf((void *)buf, len_bytes, DT_MEM_ARM(ATTR_MPU_RAM_NOCACHE)) == 0;

	return buf_within_nocache;
}
#endif
