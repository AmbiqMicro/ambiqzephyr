/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Crypto key derivation APIs
 */

#ifndef ZEPHYR_INCLUDE_CRYPTO_KDF_H_
#define ZEPHYR_INCLUDE_CRYPTO_KDF_H_

#include <zephyr/device.h>
#include <zephyr/crypto/hash.h>

/**
 * @addtogroup crypto_kdf
 * @{
 */

/** Key derivation algorithm */
enum kdf_algo {
	CRYPTO_KDF_ALGO_HKDF = 1,
};

/**
 * HKDF derive request.
 */
struct kdf_hkdf_pkt {
	enum hash_algo hash;
	const uint8_t *salt;
	size_t salt_len;
	const uint8_t *ikm;
	size_t ikm_len;
	const uint8_t *info;
	size_t info_len;
	uint8_t *okm;
	size_t okm_len;
};

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_CRYPTO_KDF_H_ */
