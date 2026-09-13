/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Ambiq CC312 Ed25519 device API
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_ED25519_H_
#define ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_ED25519_H_

#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>

#define CRYPTO_AMBIQ_ED25519_SEED_SIZE       32
#define CRYPTO_AMBIQ_ED25519_PUBLIC_KEY_SIZE 32
#define CRYPTO_AMBIQ_ED25519_SIGNATURE_SIZE  64

/** Ambiq Ed25519 driver API (``ambiq,crypto-ed25519``). */
struct crypto_ambiq_ed25519_driver_api {
	int (*public_key)(const struct device *dev, const uint8_t *seed, uint8_t *public_key);
	int (*sign)(const struct device *dev, const uint8_t *seed, const uint8_t *message,
		    uint32_t message_len, uint8_t *signature);
	int (*verify)(const struct device *dev, const uint8_t *public_key, const uint8_t *message,
		      uint32_t message_len, const uint8_t *signature);
};

static inline int crypto_ambiq_ed25519_public_key(const struct device *dev, const uint8_t *seed,
						  uint8_t *public_key)
{
	const struct crypto_ambiq_ed25519_driver_api *api;

	api = (const struct crypto_ambiq_ed25519_driver_api *)dev->api;

	if (api->public_key == NULL) {
		return -ENOTSUP;
	}

	return api->public_key(dev, seed, public_key);
}

static inline int crypto_ambiq_ed25519_sign(const struct device *dev, const uint8_t *seed,
					    const uint8_t *message, uint32_t message_len,
					    uint8_t *signature)
{
	const struct crypto_ambiq_ed25519_driver_api *api;

	api = (const struct crypto_ambiq_ed25519_driver_api *)dev->api;

	if (api->sign == NULL) {
		return -ENOTSUP;
	}

	return api->sign(dev, seed, message, message_len, signature);
}

static inline int crypto_ambiq_ed25519_verify(const struct device *dev, const uint8_t *public_key,
					      const uint8_t *message, uint32_t message_len,
					      const uint8_t *signature)
{
	const struct crypto_ambiq_ed25519_driver_api *api;

	api = (const struct crypto_ambiq_ed25519_driver_api *)dev->api;

	if (api->verify == NULL) {
		return -ENOTSUP;
	}

	return api->verify(dev, public_key, message, message_len, signature);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_ED25519_H_ */
