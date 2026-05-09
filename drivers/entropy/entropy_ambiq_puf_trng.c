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
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

LOG_MODULE_REGISTER(ambiq_puf_trng_entropy, CONFIG_ENTROPY_LOG_LEVEL);

/*
 * Select the correct TRNG memory base address.
 *
 * The TRNG is implemented in the OTP memory
 * and the TRNG address is memory mapped to the OTP.
 *
 */
#define TRNG_BASE DT_INST_REG_ADDR(0)

/* Max Fail Count  for errors */
#define MAX_FAIL_COUNT 5

static inline uint32_t get_trng_u32(void)
{
	return AM_REGVAL(TRNG_BASE);
}

static int trng_pm_action(const struct device *dev, enum pm_device_action action)
{
	ARG_UNUSED(dev);

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
	case PM_DEVICE_ACTION_SUSPEND:
	case PM_DEVICE_ACTION_TURN_ON:
	case PM_DEVICE_ACTION_TURN_OFF:
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int entropy_ambiq_get_trng(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	int ret;
	uint8_t *byte_buffer;
	uint8_t fail_cnt = 0;

	/* Validate input parameters */
	if (length == 0 || buffer == NULL) {
		return -EINVAL;
	}

	/* Acquire power domain via runtime PM */
	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	byte_buffer = buffer;

	/*
	 * While the passed in length is greater than zero
	 * grab data from RNG and save to output.
	 */
	while ((length > 0) && (fail_cnt < MAX_FAIL_COUNT)) {
		uint32_t word = get_trng_u32();

		/* This is a failure mode where the RNG doesn't have enough randomness */
		if (word == 0xdeaddead) {
			fail_cnt++;
			continue;
		}

		size_t copy_length = MIN(sizeof(uint32_t), length);
		/*
		 * If the length is less than 4 bytes, we only copy the
		 * requested number of bytes.
		 */
		memcpy(byte_buffer, &word, copy_length);
		byte_buffer += copy_length;
		length -= copy_length;
	}

	/* Release power domain */
	(void)pm_device_runtime_put(dev);

	if (fail_cnt >= MAX_FAIL_COUNT) {
		return -EIO;
	}

	return 0;
}

static DEVICE_API(entropy, entropy_ambiq_api_funcs) = {
	.get_entropy = entropy_ambiq_get_trng,
};

static int entropy_ambiq_init(const struct device *dev)
{
	return pm_device_runtime_enable(dev);
}

PM_DEVICE_DT_INST_DEFINE(0, trng_pm_action);

DEVICE_DT_INST_DEFINE(0, entropy_ambiq_init, PM_DEVICE_DT_INST_GET(0), NULL, NULL, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &entropy_ambiq_api_funcs);
