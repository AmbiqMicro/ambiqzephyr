/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Shared, refcounted access to Ambiq peripheral power rails.
 *
 * Several Ambiq peripherals are powered through a shared rail that is enabled
 * by am_hal_pwrctrl_periph_enable() and disabled by am_hal_pwrctrl_periph_disable().
 * The HAL enable call is idempotent but the disable call is not — it powers
 * the rail off regardless of how many drivers are still using it. That breaks
 * scenarios such as:
 *
 *   - The OTP rail backs both the TRNG (entropy driver) and the CC312 AES
 *     engine (crypto driver). If the crypto driver disables OTP after a
 *     transform, the entropy driver loses its memory-mapped TRNG.
 *
 *   - The DISP rail backs MIPI-DSI, MIPI-DBI, JDI, and the SPI display
 *     controller. Any one of them suspending tears down the rail for the
 *     others.
 *
 * This API maintains a per-peripheral reference count and only calls into the
 * HAL on 0->1 (enable) and 1->0 (disable) transitions, so multiple drivers can
 * share a rail safely. It is internally serialized with a mutex; callers may
 * invoke it from any context that may sleep (init, PM hooks, work handlers).
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_AMBIQ_PWRCTRL_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_AMBIQ_PWRCTRL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Peripheral identifiers managed by this module. */
enum ambiq_pwrctrl_periph {
	/** OTP rail — backs TRNG (entropy) and CC312 AES (crypto). */
	AMBIQ_PWRCTRL_OTP = 0,
	/** CC312 AES engine power. */
	AMBIQ_PWRCTRL_CRYPTO,
	/** Display rail — backs MIPI-DSI, MIPI-DBI, JDI, display SPI. */
	AMBIQ_PWRCTRL_DISP,
	AMBIQ_PWRCTRL_NUM,
};

/**
 * @brief Acquire a reference on a peripheral, powering it up on the 0->1 edge.
 *
 * @param periph Peripheral to acquire.
 * @return 0 on success, negative errno on failure (rail enable failed).
 */
int ambiq_pwrctrl_acquire(enum ambiq_pwrctrl_periph periph);

/**
 * @brief Release a reference on a peripheral, powering it down on the 1->0 edge.
 *
 * Releasing a peripheral that was never acquired by this caller is a bug; the
 * implementation will keep the count at zero and return -EINVAL but the
 * caller's HAL state may be inconsistent.
 *
 * @param periph Peripheral to release.
 * @return 0 on success, negative errno on failure.
 */
int ambiq_pwrctrl_release(enum ambiq_pwrctrl_periph periph);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_AMBIQ_PWRCTRL_H_ */
