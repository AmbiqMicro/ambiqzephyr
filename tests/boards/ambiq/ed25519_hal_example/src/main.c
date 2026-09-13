/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/crypto/crypto_ambiq_ed25519.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/printk.h>

#include <string.h>

#include <soc.h>

#define ED25519_DEV DEVICE_DT_GET_ONE(ambiq_crypto_ed25519)

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define ED25519_NOCACHE __nocache
#else
#define ED25519_NOCACHE
#endif

static ED25519_NOCACHE uint8_t g_seed[32] __aligned(32);
static ED25519_NOCACHE uint8_t g_public[32] __aligned(32);
static ED25519_NOCACHE uint8_t g_signature[64] __aligned(32);

/* RFC 8032 section 7.1 TEST 1 (empty message) */
static const uint8_t seed[32] = {
	0x9d, 0x61, 0xb1, 0x9d, 0xef, 0xfd, 0x5a, 0x60, 0xba, 0x84, 0x4a,
	0xf4, 0x92, 0xec, 0x2c, 0xc4, 0x44, 0x49, 0xc5, 0x69, 0x7b, 0x32,
	0x69, 0x19, 0x70, 0x3b, 0xac, 0x03, 0x1c, 0xae, 0x7f, 0x60,
};
static const uint8_t public_exp[32] = {
	0xd7, 0x5a, 0x98, 0x01, 0x82, 0xb1, 0x0a, 0xb7, 0xd5, 0x4b, 0xfe,
	0xd3, 0xc9, 0x64, 0x07, 0x3a, 0x0e, 0xe1, 0x72, 0xf3, 0xda, 0xa6,
	0x23, 0x25, 0xaf, 0x02, 0x1a, 0x68, 0xf7, 0x07, 0x51, 0x1a,
};
static const uint8_t signature_exp[64] = {
	0xe5, 0x56, 0x43, 0x00, 0xc3, 0x60, 0xac, 0x72, 0x90, 0x86, 0xe2, 0xcc, 0x80,
	0x6e, 0x82, 0x8a, 0x84, 0x87, 0x7f, 0x1e, 0xb8, 0xe5, 0xd9, 0x74, 0xd8, 0x73,
	0xe0, 0x65, 0x22, 0x49, 0x01, 0x55, 0x5f, 0xb8, 0x82, 0x15, 0x90, 0xa3, 0x3b,
	0xac, 0xc6, 0x1e, 0x39, 0x70, 0x1c, 0xf9, 0xb4, 0x6b, 0xd2, 0x5b, 0xf5, 0xf0,
	0x59, 0x5b, 0xbe, 0x24, 0x65, 0x51, 0x41, 0x43, 0x8e, 0x7a, 0x10, 0x0b,
};

int main(void)
{
	const struct device *dev = ED25519_DEV;
	int ret;
	bool pub_ok;
	bool sig_ok;
	bool ver_ok;

	printk("\nAmbiq CC312 Ed25519 driver checks\n\n");

	if (!device_is_ready(dev)) {
		printk("Ed25519 device not ready\n");
		return 0;
	}

	(void)am_hal_cc312_memcpy(g_seed, (const uint8_t *)seed, sizeof(seed));
	am_hal_cc312_context_clear(g_public, sizeof(g_public));
	ret = crypto_ambiq_ed25519_public_key(dev, g_seed, g_public);
	pub_ok = (ret == 0) && (memcmp(g_public, public_exp, sizeof(public_exp)) == 0);
	printk("Ed25519 public key RFC8032 test 1      %s\n", pub_ok ? "PASS" : "FAIL");

	am_hal_cc312_context_clear(g_signature, sizeof(g_signature));
	ret = crypto_ambiq_ed25519_sign(dev, g_seed, NULL, 0U, g_signature);
	sig_ok = (ret == 0) && (memcmp(g_signature, signature_exp, sizeof(signature_exp)) == 0);
	printk("Ed25519 sign RFC8032 test 1            %s\n", sig_ok ? "PASS" : "FAIL");

	ret = crypto_ambiq_ed25519_verify(dev, g_public, NULL, 0U, g_signature);
	ver_ok = (ret == 0);
	printk("Ed25519 verify RFC8032 test 1          %s\n", ver_ok ? "PASS" : "FAIL");

	if (pub_ok && sig_ok && ver_ok) {
		printk("All driver checks passed\n");
	}

	return 0;
}
