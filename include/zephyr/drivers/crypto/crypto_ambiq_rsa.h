/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Ambiq CC312 RSA device API
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_RSA_H_
#define ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_RSA_H_

#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>

/** Raw modular exponentiation request (one-shot). */
struct crypto_ambiq_rsa_mod_exp_pkt {
	const uint8_t *modulus;
	uint32_t modulus_bytes;
	const uint8_t *exponent;
	const uint8_t *base;
	uint8_t *result;
};

/** CRT private-key operation request (one-shot). */
struct crypto_ambiq_rsa_mod_exp_crt_pkt {
	const uint8_t *p;
	const uint8_t *q;
	const uint8_t *dP;
	const uint8_t *dQ;
	const uint8_t *qInv;
	uint32_t pq_bytes;
	const uint8_t *base;
	uint8_t *result;
};

/** Ambiq RSA driver API (``ambiq,crypto-rsa``). */
struct crypto_ambiq_rsa_driver_api {
	int (*mod_exp)(const struct device *dev, struct crypto_ambiq_rsa_mod_exp_pkt *pkt);
	int (*mod_exp_crt)(const struct device *dev, struct crypto_ambiq_rsa_mod_exp_crt_pkt *pkt);
};

static inline int crypto_ambiq_rsa_mod_exp(const struct device *dev,
					   struct crypto_ambiq_rsa_mod_exp_pkt *pkt)
{
	const struct crypto_ambiq_rsa_driver_api *api;

	api = (const struct crypto_ambiq_rsa_driver_api *)dev->api;

	if (api->mod_exp == NULL) {
		return -ENOTSUP;
	}

	return api->mod_exp(dev, pkt);
}

static inline int crypto_ambiq_rsa_mod_exp_crt(const struct device *dev,
						 struct crypto_ambiq_rsa_mod_exp_crt_pkt *pkt)
{
	const struct crypto_ambiq_rsa_driver_api *api;

	api = (const struct crypto_ambiq_rsa_driver_api *)dev->api;

	if (api->mod_exp_crt == NULL) {
		return -ENOTSUP;
	}

	return api->mod_exp_crt(dev, pkt);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_RSA_H_ */
