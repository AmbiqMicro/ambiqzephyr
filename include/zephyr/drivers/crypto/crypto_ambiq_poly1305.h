/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Ambiq CC312 Poly1305 device API
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_POLY1305_H_
#define ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_POLY1305_H_

#include <errno.h>
#include <zephyr/device.h>

/**
 * @brief Ambiq Poly1305 driver API.
 *
 * Bind to a device with compatible ``ambiq,crypto-poly1305``.
 */
struct crypto_ambiq_poly1305_driver_api {
	int (*mac)(const struct device *dev, const uint8_t *key, const uint8_t *input,
		   uint32_t length, uint8_t *mac_out);
	int (*mac_aead)(const struct device *dev, const uint8_t *key, const uint8_t *aad,
			uint32_t aad_len, const uint8_t *input, uint32_t length, uint8_t *mac_out);
};

/**
 * @brief Compute a Poly1305 authenticator.
 */
static inline int crypto_ambiq_poly1305_mac(const struct device *dev, const uint8_t *key,
					    const uint8_t *input, uint32_t length,
					    uint8_t *mac_out)
{
	const struct crypto_ambiq_poly1305_driver_api *api;

	api = (const struct crypto_ambiq_poly1305_driver_api *)dev->api;

	if (api->mac == NULL) {
		return -ENOTSUP;
	}

	return api->mac(dev, key, input, length, mac_out);
}

/**
 * @brief Compute a ChaCha20-Poly1305 AEAD authenticator (RFC 8439 layout).
 */
static inline int crypto_ambiq_poly1305_mac_aead(const struct device *dev, const uint8_t *key,
						 const uint8_t *aad, uint32_t aad_len,
						 const uint8_t *input, uint32_t length,
						 uint8_t *mac_out)
{
	const struct crypto_ambiq_poly1305_driver_api *api;

	api = (const struct crypto_ambiq_poly1305_driver_api *)dev->api;

	if (api->mac_aead == NULL) {
		return -ENOTSUP;
	}

	return api->mac_aead(dev, key, aad, aad_len, input, length, mac_out);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_POLY1305_H_ */
