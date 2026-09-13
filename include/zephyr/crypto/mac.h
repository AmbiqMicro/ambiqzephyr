/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Crypto MAC APIs
 */

#ifndef ZEPHYR_INCLUDE_CRYPTO_MAC_H_
#define ZEPHYR_INCLUDE_CRYPTO_MAC_H_

#include <zephyr/device.h>

/**
 * @addtogroup crypto_mac
 * @{
 */

/** Message authentication algorithm */
enum mac_algo {
	CRYPTO_MAC_ALGO_HMAC_SHA1 = 1,
	CRYPTO_MAC_ALGO_HMAC_SHA224 = 2,
	CRYPTO_MAC_ALGO_HMAC_SHA256 = 3,
	CRYPTO_MAC_ALGO_AES_CMAC = 4,
};

struct mac_ctx;
struct mac_pkt;

typedef int (*mac_op_t)(struct mac_ctx *ctx, struct mac_pkt *pkt, bool finish);

/**
 * MAC session context.
 */
struct mac_ctx {
	const struct device *device;
	void *drv_sessn_state;
	mac_op_t mac_hndlr;
	bool started;
	uint16_t flags;
	uint16_t keylen;
	union {
		const uint8_t *bit_stream;
		void *handle;
	} key;
};

/**
 * MAC operation packet.
 */
struct mac_pkt {
	const uint8_t *in_buf;
	size_t in_len;
	uint8_t *out_buf;
	struct mac_ctx *ctx;
};

typedef void (*mac_completion_cb)(struct mac_pkt *completed, int status);

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_CRYPTO_MAC_H_ */
