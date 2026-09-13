/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Ambiq CC312 SRP-6a device API
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_SRP_H_
#define ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_SRP_H_

#include <errno.h>
#include <stdint.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/device.h>

/** SRP group parameters passed to the driver. */
struct crypto_ambiq_srp_group {
	const uint8_t *n;
	const uint8_t *g;
	uint32_t modulus_bytes;
	enum hash_algo hash;
};

/** Ambiq SRP driver API (``ambiq,crypto-srp``). */
struct crypto_ambiq_srp_driver_api {
	int (*derive_x)(const struct device *dev, const struct crypto_ambiq_srp_group *group,
			const uint8_t *salt, uint32_t salt_len, const uint8_t *user,
			uint32_t user_len, const uint8_t *password, uint32_t pass_len,
			uint8_t *x);
	int (*derive_k)(const struct device *dev, const struct crypto_ambiq_srp_group *group,
			uint8_t *k);
	int (*derive_u)(const struct device *dev, const struct crypto_ambiq_srp_group *group,
			const uint8_t *a_pub, const uint8_t *b_pub, uint8_t *u);
	int (*derive_verifier)(const struct device *dev, const struct crypto_ambiq_srp_group *group,
			       const uint8_t *x, uint32_t x_len, uint8_t *v);
	int (*client_secret)(const struct device *dev, const struct crypto_ambiq_srp_group *group,
			     const uint8_t *b_pub, const uint8_t *k, uint32_t k_len,
			     const uint8_t *x, uint32_t x_len, const uint8_t *a_priv,
			     uint32_t a_len, const uint8_t *u, uint32_t u_len, uint8_t *secret);
	int (*server_secret)(const struct device *dev, const struct crypto_ambiq_srp_group *group,
			     const uint8_t *a_pub, const uint8_t *v, const uint8_t *u,
			     uint32_t u_len, const uint8_t *b_priv, uint32_t b_len,
			     uint8_t *secret);
};

static inline int crypto_ambiq_srp_derive_x(const struct device *dev,
					    const struct crypto_ambiq_srp_group *group,
					    const uint8_t *salt, uint32_t salt_len,
					    const uint8_t *user, uint32_t user_len,
					    const uint8_t *password, uint32_t pass_len,
					    uint8_t *x)
{
	const struct crypto_ambiq_srp_driver_api *api =
		(const struct crypto_ambiq_srp_driver_api *)dev->api;

	if (api->derive_x == NULL) {
		return -ENOTSUP;
	}

	return api->derive_x(dev, group, salt, salt_len, user, user_len, password, pass_len, x);
}

static inline int crypto_ambiq_srp_derive_k(const struct device *dev,
					    const struct crypto_ambiq_srp_group *group, uint8_t *k)
{
	const struct crypto_ambiq_srp_driver_api *api =
		(const struct crypto_ambiq_srp_driver_api *)dev->api;

	if (api->derive_k == NULL) {
		return -ENOTSUP;
	}

	return api->derive_k(dev, group, k);
}

static inline int crypto_ambiq_srp_derive_u(const struct device *dev,
					    const struct crypto_ambiq_srp_group *group,
					    const uint8_t *a_pub, const uint8_t *b_pub, uint8_t *u)
{
	const struct crypto_ambiq_srp_driver_api *api =
		(const struct crypto_ambiq_srp_driver_api *)dev->api;

	if (api->derive_u == NULL) {
		return -ENOTSUP;
	}

	return api->derive_u(dev, group, a_pub, b_pub, u);
}

static inline int crypto_ambiq_srp_derive_verifier(const struct device *dev,
						   const struct crypto_ambiq_srp_group *group,
						   const uint8_t *x, uint32_t x_len, uint8_t *v)
{
	const struct crypto_ambiq_srp_driver_api *api;

	api = (const struct crypto_ambiq_srp_driver_api *)dev->api;

	if (api->derive_verifier == NULL) {
		return -ENOTSUP;
	}

	return api->derive_verifier(dev, group, x, x_len, v);
}

static inline int crypto_ambiq_srp_client_secret(const struct device *dev,
						 const struct crypto_ambiq_srp_group *group,
						 const uint8_t *b_pub, const uint8_t *k,
						 uint32_t k_len, const uint8_t *x,
						 uint32_t x_len, const uint8_t *a_priv,
						 uint32_t a_len, const uint8_t *u,
						 uint32_t u_len, uint8_t *secret)
{
	const struct crypto_ambiq_srp_driver_api *api;

	api = (const struct crypto_ambiq_srp_driver_api *)dev->api;

	if (api->client_secret == NULL) {
		return -ENOTSUP;
	}

	return api->client_secret(dev, group, b_pub, k, k_len, x, x_len, a_priv, a_len, u, u_len,
				  secret);
}

static inline int crypto_ambiq_srp_server_secret(const struct device *dev,
						 const struct crypto_ambiq_srp_group *group,
						 const uint8_t *a_pub, const uint8_t *v,
						 const uint8_t *u, uint32_t u_len,
						 const uint8_t *b_priv, uint32_t b_len,
						 uint8_t *secret)
{
	const struct crypto_ambiq_srp_driver_api *api;

	api = (const struct crypto_ambiq_srp_driver_api *)dev->api;

	if (api->server_secret == NULL) {
		return -ENOTSUP;
	}

	return api->server_secret(dev, group, a_pub, v, u, u_len, b_priv, b_len, secret);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_CRYPTO_CRYPTO_AMBIQ_SRP_H_ */
