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

#define HMAC_DEV DEVICE_DT_GET_ONE(ambiq_crypto_hmac)

#define MAC_CAPS (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define HMAC_NOCACHE __nocache
#else
#define HMAC_NOCACHE
#endif

static HMAC_NOCACHE uint8_t g_key[20] __aligned(32);
static HMAC_NOCACHE uint8_t g_data[8] __aligned(32);
static HMAC_NOCACHE uint8_t g_mac[32] __aligned(32);

/* RFC 4231 test case 1: HMAC-SHA256, key 20 x 0x0b, data "Hi There" */
static const uint8_t tc1_key[20] = {
	0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
	0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
};
static const char tc1_data[] = "Hi There";
static const uint8_t tc1_mac[32] = {
	0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53, 0x5c, 0xa8, 0xaf,
	0xce, 0xaf, 0x0b, 0xf1, 0x2b, 0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83,
	0x3d, 0xa7, 0x26, 0xe9, 0x37, 0x6c, 0x2e, 0x32, 0xcf, 0xf7,
};

static int g_pass;
static int g_fail;

static void report(const char *name, bool ok)
{
	printk("%-40s %s\n", name, ok ? "PASS" : "FAIL");
	if (ok) {
		g_pass++;
	} else {
		g_fail++;
	}
}

static bool run_hmac_sha256(void)
{
	struct mac_ctx ctx = {
		.flags = MAC_CAPS,
		.keylen = sizeof(g_key),
		.key.bit_stream = g_key,
	};
	struct mac_pkt pkt = {
		.in_buf = g_data,
		.in_len = sizeof(g_data),
		.out_buf = g_mac,
	};
	const struct device *dev = HMAC_DEV;
	int ret;

	(void)am_hal_cc312_memcpy(g_key, tc1_key, sizeof(tc1_key));
	(void)am_hal_cc312_memcpy(g_data, (const uint8_t *)tc1_data, sizeof(g_data));
	am_hal_cc312_context_clear(g_mac, sizeof(g_mac));

	ret = mac_begin_session(dev, &ctx, CRYPTO_MAC_ALGO_HMAC_SHA256);
	if (ret != 0) {
		return false;
	}

	ret = mac_compute(&ctx, &pkt);
	(void)mac_free_session(dev, &ctx);

	return (ret == 0) && (memcmp(g_mac, tc1_mac, 32) == 0);
}

int main(void)
{
	const struct device *dev = HMAC_DEV;

	printk("\nAmbiq CC312 HMAC driver checks\n\n");

	if (!device_is_ready(dev)) {
		printk("HMAC device not ready\n");
		return 0;
	}

	report("HMAC-SHA256 RFC4231 test 1", run_hmac_sha256());

	printk("\n%d passed, %d failed\n", g_pass, g_fail);

	if (g_fail == 0) {
		printk("All driver checks passed\n");
	}

	return 0;
}
