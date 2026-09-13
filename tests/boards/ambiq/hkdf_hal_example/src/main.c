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

#define KDF_DEV DEVICE_DT_GET_ONE(ambiq_crypto_kdf)

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define HKDF_NOCACHE __nocache
#else
#define HKDF_NOCACHE
#endif

static HKDF_NOCACHE uint8_t g_ikm[22] __aligned(32);
static HKDF_NOCACHE uint8_t g_salt[13] __aligned(32);
static HKDF_NOCACHE uint8_t g_info[10] __aligned(32);
static HKDF_NOCACHE uint8_t g_okm[42] __aligned(32);

/* RFC 5869 test case 1 (SHA-256), 42-byte OKM */
static const uint8_t ikm[] = {
	0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
	0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
};
static const uint8_t salt[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c,
};
static const uint8_t info[] = {
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9,
};
static const uint8_t okm_exp[42] = {
	0x3c, 0xb2, 0x5f, 0x25, 0xfa, 0xac, 0xd5, 0x7a, 0x90, 0x43, 0x4f, 0x64, 0xd0, 0x36,
	0x2f, 0x2a, 0x2d, 0x2d, 0x0a, 0x90, 0xcf, 0x1a, 0x5a, 0x4c, 0x5d, 0xb0, 0x2d, 0x56,
	0xec, 0xc4, 0xc5, 0xbf, 0x34, 0x00, 0x72, 0x08, 0xd5, 0xb8, 0x87, 0x18, 0x58, 0x65,
};

int main(void)
{
	const struct device *dev = KDF_DEV;
	struct kdf_hkdf_pkt pkt = {
		.hash = CRYPTO_HASH_ALGO_SHA256,
		.salt = g_salt,
		.salt_len = sizeof(g_salt),
		.ikm = g_ikm,
		.ikm_len = sizeof(g_ikm),
		.info = g_info,
		.info_len = sizeof(g_info),
		.okm = g_okm,
		.okm_len = sizeof(g_okm),
	};
	int ret;
	bool ok;

	printk("\nAmbiq CC312 HKDF driver checks\n\n");

	if (!device_is_ready(dev)) {
		printk("HKDF device not ready\n");
		return 0;
	}

	(void)am_hal_cc312_memcpy(g_ikm, ikm, sizeof(ikm));
	(void)am_hal_cc312_memcpy(g_salt, salt, sizeof(salt));
	(void)am_hal_cc312_memcpy(g_info, info, sizeof(info));
	am_hal_cc312_context_clear(g_okm, sizeof(g_okm));

	ret = kdf_hkdf(dev, &pkt);
	ok = (ret == 0) && (memcmp(g_okm, okm_exp, sizeof(okm_exp)) == 0);

	printk("HKDF-SHA256 RFC5869 test 1           %s\n", ok ? "PASS" : "FAIL");

	if (ok) {
		printk("All driver checks passed\n");
	}

	return 0;
}
