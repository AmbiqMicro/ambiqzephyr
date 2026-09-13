/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Crypto key agreement APIs
 */

#ifndef ZEPHYR_INCLUDE_CRYPTO_KEY_AGREEMENT_H_
#define ZEPHYR_INCLUDE_CRYPTO_KEY_AGREEMENT_H_

#include <zephyr/device.h>

/**
 * @addtogroup crypto_key_agreement
 * @{
 */

/** Key agreement algorithm */
enum key_agreement_algo {
	CRYPTO_KEY_AGREEMENT_ALGO_X25519 = 1,
};

/** X25519 shared secret request */
struct key_agreement_x25519_pkt {
	const uint8_t *private_key;
	const uint8_t *public_key;
	uint8_t *shared_secret;
};

/** X25519 driver API (device-specific compatibles). */
struct key_agreement_driver_api {
	int (*x25519_shared_secret)(const struct device *dev, struct key_agreement_x25519_pkt *pkt);
	int (*x25519_public_key)(const struct device *dev, const uint8_t *private_key,
				 uint8_t *public_key);
};

/**
 * @brief Compute an X25519 shared secret.
 *
 * @param dev Pointer to the X25519 device (for example ``ambiq,crypto-x25519``).
 * @param pkt Request describing private key, peer public key, and output buffer.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int key_agreement_x25519(const struct device *dev,
				       struct key_agreement_x25519_pkt *pkt)
{
	const struct key_agreement_driver_api *api;

	api = (const struct key_agreement_driver_api *)dev->api;

	if (api->x25519_shared_secret == NULL) {
		return -ENOTSUP;
	}

	return api->x25519_shared_secret(dev, pkt);
}

/**
 * @brief Derive an X25519 public key from a private scalar.
 *
 * @param dev Pointer to the X25519 device.
 * @param private_key 32-byte private scalar.
 * @param public_key 32-byte output buffer for the public u-coordinate.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int key_agreement_x25519_public_key(const struct device *dev,
						    const uint8_t *private_key,
						    uint8_t *public_key)
{
	const struct key_agreement_driver_api *api;

	api = (const struct key_agreement_driver_api *)dev->api;

	if (api->x25519_public_key == NULL) {
		return -ENOTSUP;
	}

	return api->x25519_public_key(dev, private_key, public_key);
}

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_CRYPTO_KEY_AGREEMENT_H_ */
