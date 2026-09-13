/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/crypto/crypto_ambiq_poly1305.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/printk.h>

#include <string.h>

#include <soc.h>

#define POLY1305_DEV DEVICE_DT_GET_ONE(ambiq_crypto_poly1305)

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define POLY_NOCACHE __nocache
#else
#define POLY_NOCACHE
#endif

static POLY_NOCACHE uint8_t g_key[32] __aligned(32);
static POLY_NOCACHE uint8_t g_msg[64] __aligned(32);
static POLY_NOCACHE uint8_t g_mac[16] __aligned(32);

/* RFC 8439 (ex-7539) Appendix A.3 test vector #1 */
static const uint8_t key[32] = {
	0x85, 0xd6, 0xbe, 0x78, 0x57, 0x55, 0x6d, 0x33, 0x7f, 0x44, 0x52,
	0xfe, 0x42, 0xd5, 0x06, 0xa8, 0x01, 0x03, 0x80, 0x8a, 0xfb, 0x0d,
	0xb2, 0xfd, 0x4a, 0xbf, 0xf6, 0xaf, 0x41, 0x49, 0xf5, 0x1b,
};
static const char msg[] = "Cryptographic Forum Research Group";
static const uint8_t mac_exp[16] = {
	0xa8, 0x06, 0x1d, 0xc1, 0x30, 0x51, 0x36, 0xc6,
	0xc2, 0x2b, 0x8b, 0xaf, 0x0c, 0x01, 0x27, 0xa9,
};

int main(void)
{
	const struct device *dev = POLY1305_DEV;
	size_t msg_len = strlen(msg);
	int ret;
	bool ok;

	printk("\nAmbiq CC312 Poly1305 driver checks\n\n");

	if (!device_is_ready(dev)) {
		printk("Poly1305 device not ready\n");
		return 0;
	}

	(void)am_hal_cc312_memcpy(g_key, (const uint8_t *)key, sizeof(key));
	(void)am_hal_cc312_memcpy(g_msg, (const uint8_t *)msg, msg_len);
	am_hal_cc312_context_clear(g_mac, sizeof(g_mac));
	ret = crypto_ambiq_poly1305_mac(dev, g_key, g_msg, (uint32_t)msg_len, g_mac);
	ok = (ret == 0) && (memcmp(g_mac, mac_exp, sizeof(mac_exp)) == 0);

	printk("Poly1305 RFC8439 A.3                  %s\n", ok ? "PASS" : "FAIL");

	if (ok) {
		printk("All driver checks passed\n");
	}

	return 0;
}
