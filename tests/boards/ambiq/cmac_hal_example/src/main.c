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

#define CMAC_DEV DEVICE_DT_GET_ONE(ambiq_crypto_cmac)

#define MAC_CAPS (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define CMAC_NOCACHE __nocache
#else
#define CMAC_NOCACHE
#endif

static CMAC_NOCACHE uint8_t g_key[16] __aligned(32);
static CMAC_NOCACHE uint8_t g_msg[16] __aligned(32);
static CMAC_NOCACHE uint8_t g_mac[16] __aligned(32);

/* NIST SP 800-38B Appendix D.1 Example 2 (AES-128), Mlen = 128 bits */
static const uint8_t key[16] = {
	0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
	0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
};
static const uint8_t msg[16] = {
	0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
	0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
};
static const uint8_t mac_exp[16] = {
	0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44,
	0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a, 0x28, 0x7c,
};

int main(void)
{
	const struct device *dev = CMAC_DEV;
	struct mac_ctx ctx = {
		.flags = MAC_CAPS,
		.keylen = sizeof(g_key),
		.key.bit_stream = g_key,
	};
	struct mac_pkt pkt;
	int ret;
	bool ok;

	printk("\nAmbiq CC312 AES-CMAC driver checks\n\n");

	if (!device_is_ready(dev)) {
		printk("CMAC device not ready\n");
		return 0;
	}

	(void)am_hal_cc312_memcpy(g_key, (const uint8_t *)key, sizeof(key));
	(void)am_hal_cc312_memcpy(g_msg, (const uint8_t *)msg, sizeof(msg));
	am_hal_cc312_context_clear(g_mac, sizeof(g_mac));
	pkt.in_buf = g_msg;
	pkt.in_len = sizeof(msg);
	pkt.out_buf = g_mac;

	ret = mac_begin_session(dev, &ctx, CRYPTO_MAC_ALGO_AES_CMAC);
	if (ret == 0) {
		ret = mac_compute(&ctx, &pkt);
		(void)mac_free_session(dev, &ctx);
	}

	ok = (ret == 0) && (memcmp(g_mac, mac_exp, sizeof(mac_exp)) == 0);
	printk("AES-CMAC-128 NIST example 2          %s\n", ok ? "PASS" : "FAIL");

	if (ok) {
		printk("All driver checks passed\n");
	}

	return 0;
}
