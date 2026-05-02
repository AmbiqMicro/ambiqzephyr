/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/misc/ambiq_pwrctrl/ambiq_pwrctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <soc.h>

LOG_MODULE_REGISTER(ambiq_pwrctrl, CONFIG_AMBIQ_PWRCTRL_LOG_LEVEL);

/* Translate ambiq_pwrctrl_periph -> HAL identifier.
 *
 * The OTP rail only exists on Apollo5X (incl. Apollo330P); on Apollo4X the
 * AM_HAL_PWRCTRL_PERIPH_OTP enumerator is not defined, so we leave the slot
 * unset and reject runtime requests for it below. The puf_trng entropy
 * driver — the only OTP consumer — is gated by devicetree and is not
 * instantiated on Apollo4X boards, so this is a build-time guard only.
 */
static const am_hal_pwrctrl_periph_e periph_to_hal[AMBIQ_PWRCTRL_NUM] = {
#ifdef CONFIG_AMBIQ_PWRCTRL_HAS_OTP
	[AMBIQ_PWRCTRL_OTP] = AM_HAL_PWRCTRL_PERIPH_OTP,
#endif
	[AMBIQ_PWRCTRL_CRYPTO] = AM_HAL_PWRCTRL_PERIPH_CRYPTO,
	[AMBIQ_PWRCTRL_DISP] = AM_HAL_PWRCTRL_PERIPH_DISP,
};

static K_MUTEX_DEFINE(refcount_lock);
static unsigned int refcount[AMBIQ_PWRCTRL_NUM];

int ambiq_pwrctrl_acquire(enum ambiq_pwrctrl_periph periph)
{
	uint32_t status;
	int ret = 0;

	if ((unsigned int)periph >= AMBIQ_PWRCTRL_NUM) {
		return -EINVAL;
	}
#ifndef CONFIG_AMBIQ_PWRCTRL_HAS_OTP
	if (periph == AMBIQ_PWRCTRL_OTP) {
		return -ENOTSUP;
	}
#endif

	k_mutex_lock(&refcount_lock, K_FOREVER);

	if (refcount[periph] == 0U) {
		status = am_hal_pwrctrl_periph_enable(periph_to_hal[periph]);
		if (status != AM_HAL_STATUS_SUCCESS) {
			LOG_ERR("enable periph %d failed: 0x%x", periph, status);
			ret = -EIO;
			goto out;
		}
	}
	refcount[periph]++;

out:
	k_mutex_unlock(&refcount_lock);
	return ret;
}

int ambiq_pwrctrl_release(enum ambiq_pwrctrl_periph periph)
{
	uint32_t status;
	int ret = 0;

	if ((unsigned int)periph >= AMBIQ_PWRCTRL_NUM) {
		return -EINVAL;
	}
#ifndef CONFIG_AMBIQ_PWRCTRL_HAS_OTP
	if (periph == AMBIQ_PWRCTRL_OTP) {
		return -ENOTSUP;
	}
#endif

	k_mutex_lock(&refcount_lock, K_FOREVER);

	if (refcount[periph] == 0U) {
		LOG_WRN("release of periph %d with refcount already 0", periph);
		ret = -EINVAL;
		goto out;
	}

	refcount[periph]--;
	if (refcount[periph] == 0U) {
		if (periph == AMBIQ_PWRCTRL_CRYPTO) {
			/* CRYPTO uses the dedicated power-down control rather
			 * than the generic periph_disable, to follow the AmbiqSuite
			 * recommended sequence for the CC312 engine.
			 */
			status = am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN,
							NULL);
		} else {
			status = am_hal_pwrctrl_periph_disable(periph_to_hal[periph]);
		}
		if (status != AM_HAL_STATUS_SUCCESS) {
			/* Restore the count: the rail is still notionally up. */
			refcount[periph]++;
			LOG_ERR("disable periph %d failed: 0x%x", periph, status);
			ret = -EIO;
			goto out;
		}
	}

out:
	k_mutex_unlock(&refcount_lock);
	return ret;
}
