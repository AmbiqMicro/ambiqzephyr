/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/crypto/crypto.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/printk.h>

#include <string.h>

#include <soc.h>

#define X25519_DEV DEVICE_DT_GET_ONE(ambiq_crypto_x25519)

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define X25519_NOCACHE __nocache
#else
#define X25519_NOCACHE
#endif

static X25519_NOCACHE uint8_t g_shared[32] __aligned(32);

/* RFC 7748 section 5.2 vector 1 (shared secret) */
static const uint8_t alice_private[32] = {
	0x77, 0x07, 0x6d, 0x0a, 0x73, 0x18, 0xa5, 0x7d, 0x3c, 0x16, 0xc1,
	0x72, 0x51, 0xb2, 0x66, 0x45, 0xdf, 0x4c, 0x2f, 0x87, 0xeb, 0xc0,
	0x99, 0x2a, 0xb1, 0x77, 0xfb, 0xa5, 0x1d, 0xb9, 0x2c, 0x2a,
};
static const uint8_t bob_public[32] = {
	0xde, 0x9e, 0xdb, 0x7d, 0x7b, 0x7d, 0xc1, 0xb4, 0xd3, 0x5b, 0x61,
	0xc2, 0xec, 0xe4, 0x35, 0x37, 0x3f, 0x83, 0x43, 0xc8, 0x5b, 0x78,
	0x67, 0x4d, 0xad, 0xfc, 0x7e, 0x14, 0x6f, 0x88, 0x2b, 0x4f,
};
static const uint8_t shared_exp[32] = {
	0x4a, 0x5d, 0x9d, 0x5b, 0xa4, 0xce, 0x2d, 0xe1, 0x72, 0x8e, 0x3b,
	0xf4, 0x80, 0x35, 0x0f, 0x25, 0xe0, 0x7e, 0x21, 0xc9, 0x47, 0xd1,
	0x9e, 0x33, 0x76, 0xf0, 0x9b, 0x3c, 0x1e, 0x16, 0x17, 0x42,
};

int main(void)
{
	const struct device *dev = X25519_DEV;
	struct key_agreement_x25519_pkt pkt = {
		.private_key = alice_private,
		.public_key = bob_public,
		.shared_secret = g_shared,
	};
	int ret;
	bool ok;

	printk("\nAmbiq CC312 X25519 driver checks\n\n");

	if (!device_is_ready(dev)) {
		printk("X25519 device not ready\n");
		return 0;
	}

	am_hal_cc312_context_clear(g_shared, sizeof(g_shared));
	ret = key_agreement_x25519(dev, &pkt);
	ok = (ret == 0) && (memcmp(g_shared, shared_exp, sizeof(shared_exp)) == 0);

	printk("X25519 RFC7748 vector 1                 %s\n", ok ? "PASS" : "FAIL");

	if (ok) {
		printk("All driver checks passed\n");
	}

	return 0;
}
