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
#include <zephyr/drivers/misc/ambiq_pwrctrl/ambiq_pwrctrl.h>

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

/* Acquire OTP and CRYPTO power rails for TRNG access.
 * The TRNG is memory-mapped into OTP, and CRYPTO must also be powered on
 * for proper operation. Both rails are managed through ambiq_pwrctrl to
 * allow sharing with other drivers (e.g., AES crypto).
 */
static int entropy_ambiq_periph_acquire(void)
{
	int ret;

	ret = ambiq_pwrctrl_acquire(AMBIQ_PWRCTRL_OTP);
	if (ret) {
		LOG_ERR("Failed to acquire OTP power: %d", ret);
		return ret;
	}

	ret = ambiq_pwrctrl_acquire(AMBIQ_PWRCTRL_CRYPTO);
	if (ret) {
		LOG_ERR("Failed to acquire CRYPTO power: %d", ret);
		(void)ambiq_pwrctrl_release(AMBIQ_PWRCTRL_OTP);
		return ret;
	}

	return 0;
}

static void entropy_ambiq_periph_release(void)
{
	(void)ambiq_pwrctrl_release(AMBIQ_PWRCTRL_CRYPTO);
	(void)ambiq_pwrctrl_release(AMBIQ_PWRCTRL_OTP);
}

static int entropy_ambiq_get_trng(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	int ret;
	uint8_t *byte_buffer;
	uint8_t fail_cnt = 0;

	ARG_UNUSED(dev);

	/* Validate input parameters */
	if (length == 0 || buffer == NULL) {
		return -EINVAL;
	}

	/* Power on OTP and CRYPTO peripherals */
	ret = entropy_ambiq_periph_acquire();
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

	/* Power down OTP and CRYPTO before returning */
	entropy_ambiq_periph_release();

	if (fail_cnt >= MAX_FAIL_COUNT) {
		return -EIO;
	}

	return 0;
}

static DEVICE_API(entropy, entropy_ambiq_api_funcs) = {
	.get_entropy = entropy_ambiq_get_trng,
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &entropy_ambiq_api_funcs);
