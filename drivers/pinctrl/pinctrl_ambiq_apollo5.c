/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>

/* ambiq-sdk includes */
#include <am_mcu_apollo.h>

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	am_hal_gpio_pincfg_t pincfg = {0};

	pincfg.GP.cfg_b.uFuncSel		= pin->alt_func;
	pincfg.GP.cfg_b.eGPInput		= pin->input_enable ? AM_HAL_GPIO_PIN_INPUT_ENABLE
														: AM_HAL_GPIO_PIN_INPUT_NONE;
	pincfg.GP.cfg_b.eGPOutCfg		= pin->push_pull    ? AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL
				    									: pin->open_drain
														? AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN
				    									: pin->tristate
														? AM_HAL_GPIO_PIN_OUTCFG_TRISTATE
														: AM_HAL_GPIO_PIN_OUTCFG_DISABLE;
	pincfg.GP.cfg_b.eDriveStrength	= pin->drive_strength;
#if !defined(CONFIG_SOC_SERIES_APOLLO5X)
	pincfg.GP.cfg_b.uSlewRate		= pin->slew_rate;
#endif
	pincfg.GP.cfg_b.uNCE			= pin->nce;
	pincfg.GP.cfg_b.eCEpol			= pin->nce_pol;

	if (pin->bias_pull_up) {
		pincfg.GP.cfg_b.ePullup		= pin->ambiq_pull_up_ohms + AM_HAL_GPIO_PIN_PULLUP_1_5K;
	} else if (pin->bias_pull_down) {
		pincfg.GP.cfg_b.ePullup		= AM_HAL_GPIO_PIN_PULLDOWN_50K;
	}

	am_hal_gpio_pinconfig(pin->pin_num, pincfg);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(pins++);
	}

	return 0;
}
