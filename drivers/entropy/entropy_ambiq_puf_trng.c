/*
 * Copyright (c) 2025 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_puf_trng

#include <string.h>
#include "soc.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/entropy.h>

LOG_MODULE_REGISTER(ambiq_puf_trng_entropy, CONFIG_ENTROPY_LOG_LEVEL);

/*
 * Select the correct TRNG memory base address.
 *
 * The TRNG is implemented in the OTP memory
 * and the TRNG address is memory mapped to the OTP.
 *
 */
#define TRNG_BASE DT_INST_REG_ADDR(0)

static inline uint32_t entropy_ambiq_get_trng_u32(void)
{
	return AM_REGVAL(TRNG_BASE);
}

static int entropy_ambiq_get_trng(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	ARG_UNUSED(dev);

	/* Validate input parameters */
	if (length == 0 || buffer == NULL) {
		return -EINVAL;
	}

	uint8_t *byte_buffer = buffer;

	while (length > 0) {
		uint32_t word = entropy_ambiq_get_trng_u32();
		size_t copy_length = MIN(sizeof(uint32_t), length);

		memcpy(byte_buffer, &word, copy_length);
		byte_buffer += copy_length;
		length -= copy_length;
	}

	return 0;
}

static int entropy_ambiq_trng_init(const struct device *dev)
{
	uint32_t ui32Status;
	bool bPeripheralEnabled = false;

	/* Check and Power on OTP if it is not already on. */
	ui32Status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bPeripheralEnabled);
	if (AM_HAL_STATUS_SUCCESS != ui32Status) {
		return -EBUSY;
	}

	if (!bPeripheralEnabled) {
		ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
		if (AM_HAL_STATUS_SUCCESS != ui32Status) {
			return EBUSY;
		}
	}

	return 0;
}

static DEVICE_API(entropy, entropy_ambiq_api_funcs) = {
	.get_entropy = entropy_ambiq_get_trng,
};

DEVICE_DT_INST_DEFINE(0, entropy_ambiq_trng_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_ENTROPY_INIT_PRIORITY, &entropy_ambiq_api_funcs);
