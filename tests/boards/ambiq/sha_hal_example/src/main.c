/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/crypto/crypto.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <string.h>

#define SHA_DEV DEVICE_DT_GET_ONE(ambiq_crypto_sha)

#define SHA1_LEN   20
#define SHA256_LEN 32

#define SHA_CAPS (CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

/* Spans more than one DMA chunk, so the driver's DLLI splitting is exercised. */
#define PATTERN_LEN 100000U

static uint8_t g_pattern[PATTERN_LEN] __aligned(32);
static uint8_t g_digest[SHA256_LEN] __aligned(32);

static const char msg_abc[] = "abc";
static const char msg_448[] = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";

static const uint8_t kat_sha1_empty[SHA1_LEN] = {
	0xda, 0x39, 0xa3, 0xee, 0x5e, 0x6b, 0x4b, 0x0d, 0x32, 0x55,
	0xbf, 0xef, 0x95, 0x60, 0x18, 0x90, 0xaf, 0xd8, 0x07, 0x09};

static const uint8_t kat_sha1_abc[SHA1_LEN] = {
	0xa9, 0x99, 0x3e, 0x36, 0x47, 0x06, 0x81, 0x6a, 0xba, 0x3e,
	0x25, 0x71, 0x78, 0x50, 0xc2, 0x6c, 0x9c, 0xd0, 0xd8, 0x9d};

static const uint8_t kat_sha1_448[SHA1_LEN] = {
	0x84, 0x98, 0x3e, 0x44, 0x1c, 0x3b, 0xd2, 0x6e, 0xba, 0xae,
	0x4a, 0xa1, 0xf9, 0x51, 0x29, 0xe5, 0xe5, 0x46, 0x70, 0xf1};

static const uint8_t kat_sha1_pattern[SHA1_LEN] = {
	0x0b, 0x2d, 0xbe, 0x11, 0xb2, 0x51, 0xd5, 0x95, 0xb3, 0x5e,
	0xb4, 0x2f, 0xe8, 0x5b, 0x3a, 0x3e, 0x15, 0x76, 0x7a, 0xa0};

static const uint8_t kat_sha256_empty[SHA256_LEN] = {
	0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4,
	0xc8, 0x99, 0x6f, 0xb9, 0x24, 0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b,
	0x93, 0x4c, 0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55};

static const uint8_t kat_sha256_abc[SHA256_LEN] = {
	0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40,
	0xde, 0x5d, 0xae, 0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17,
	0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad};

static const uint8_t kat_sha256_448[SHA256_LEN] = {
	0x24, 0x8d, 0x6a, 0x61, 0xd2, 0x06, 0x38, 0xb8, 0xe5, 0xc0, 0x26,
	0x93, 0x0c, 0x3e, 0x60, 0x39, 0xa3, 0x3c, 0xe4, 0x59, 0x64, 0xff,
	0x21, 0x67, 0xf6, 0xec, 0xed, 0xd4, 0x19, 0xdb, 0x06, 0xc1};

static const uint8_t kat_sha256_pattern[SHA256_LEN] = {
	0xdb, 0x8f, 0x1d, 0x69, 0x25, 0x1d, 0x95, 0xe2, 0xc8, 0x82, 0x68,
	0xd3, 0xc5, 0x40, 0x53, 0x3c, 0xc5, 0x18, 0x2e, 0x0e, 0x33, 0x06,
	0x5a, 0x6f, 0x3f, 0x32, 0x2f, 0x60, 0x6a, 0x57, 0x44, 0x89};

static int g_pass;
static int g_fail;

static void report(const char *name, bool ok)
{
	printk("%-46s %s\n", name, ok ? "PASS" : "FAIL");
	if (ok) {
		g_pass++;
	} else {
		g_fail++;
	}
}

static size_t digest_len(enum hash_algo algo)
{
	return (algo == CRYPTO_HASH_ALGO_SHA1) ? SHA1_LEN : SHA256_LEN;
}

static bool run_oneshot(const struct device *dev, enum hash_algo algo, const uint8_t *in,
			size_t in_len, const uint8_t *expected)
{
	struct hash_ctx ctx = {.flags = SHA_CAPS};
	struct hash_pkt pkt = {.in_buf = (uint8_t *)in, .in_len = in_len, .out_buf = g_digest};
	bool ok;
	int ret;

	memset(g_digest, 0, sizeof(g_digest));

	ret = hash_begin_session(dev, &ctx, algo);
	if (ret != 0) {
		printk("  begin_session failed: %d\n", ret);
		return false;
	}

	ret = hash_compute(&ctx, &pkt);
	ok = (ret == 0) && (memcmp(g_digest, expected, digest_len(algo)) == 0);
	if (ret != 0) {
		printk("  hash_compute failed: %d\n", ret);
	}

	(void)hash_free_session(dev, &ctx);
	return ok;
}

/* Feeds the message one byte per update() to prove block-counter continuation. */
static bool run_streaming(const struct device *dev, enum hash_algo algo, const uint8_t *in,
			  size_t in_len, const uint8_t *expected)
{
	struct hash_ctx ctx = {.flags = SHA_CAPS};
	struct hash_pkt pkt;
	bool ok;
	int ret;

	memset(g_digest, 0, sizeof(g_digest));

	ret = hash_begin_session(dev, &ctx, algo);
	if (ret != 0) {
		printk("  begin_session failed: %d\n", ret);
		return false;
	}

	for (size_t i = 0; i < in_len; i++) {
		pkt.in_buf = (uint8_t *)&in[i];
		pkt.in_len = 1U;
		pkt.out_buf = NULL;

		ret = hash_update(&ctx, &pkt);
		if (ret != 0) {
			printk("  hash_update[%u] failed: %d\n", (unsigned int)i, ret);
			(void)hash_free_session(dev, &ctx);
			return false;
		}
	}

	pkt.in_buf = NULL;
	pkt.in_len = 0U;
	pkt.out_buf = g_digest;

	ret = hash_compute(&ctx, &pkt);
	ok = (ret == 0) && (memcmp(g_digest, expected, digest_len(algo)) == 0);
	if (ret != 0) {
		printk("  hash_compute failed: %d\n", ret);
	}

	(void)hash_free_session(dev, &ctx);
	return ok;
}

static bool run_unsupported_algo(const struct device *dev)
{
	struct hash_ctx ctx = {.flags = SHA_CAPS};

	return hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA512) == -ENOTSUP;
}

int main(void)
{
	const struct device *dev = SHA_DEV;

	printk("\nAmbiq CC312 SHA driver checks\n\n");

	if (!device_is_ready(dev)) {
		printk("SHA device not ready\n");
		return 0;
	}

	for (size_t i = 0; i < PATTERN_LEN; i++) {
		g_pattern[i] = (uint8_t)(i & 0xFFU);
	}

	report("SHA-1   one-shot, empty message",
	       run_oneshot(dev, CRYPTO_HASH_ALGO_SHA1, NULL, 0U, kat_sha1_empty));
	report("SHA-1   one-shot, \"abc\"",
	       run_oneshot(dev, CRYPTO_HASH_ALGO_SHA1, (const uint8_t *)msg_abc,
			   strlen(msg_abc), kat_sha1_abc));
	report("SHA-1   one-shot, 448-bit message",
	       run_oneshot(dev, CRYPTO_HASH_ALGO_SHA1, (const uint8_t *)msg_448,
			   strlen(msg_448), kat_sha1_448));
	report("SHA-1   streaming, byte at a time",
	       run_streaming(dev, CRYPTO_HASH_ALGO_SHA1, (const uint8_t *)msg_448,
			     strlen(msg_448), kat_sha1_448));
	report("SHA-1   multi-chunk DMA, 100000 bytes",
	       run_oneshot(dev, CRYPTO_HASH_ALGO_SHA1, g_pattern, PATTERN_LEN,
			   kat_sha1_pattern));

	report("SHA-256 one-shot, empty message",
	       run_oneshot(dev, CRYPTO_HASH_ALGO_SHA256, NULL, 0U, kat_sha256_empty));
	report("SHA-256 one-shot, \"abc\"",
	       run_oneshot(dev, CRYPTO_HASH_ALGO_SHA256, (const uint8_t *)msg_abc,
			   strlen(msg_abc), kat_sha256_abc));
	report("SHA-256 one-shot, 448-bit message",
	       run_oneshot(dev, CRYPTO_HASH_ALGO_SHA256, (const uint8_t *)msg_448,
			   strlen(msg_448), kat_sha256_448));
	report("SHA-256 streaming, byte at a time",
	       run_streaming(dev, CRYPTO_HASH_ALGO_SHA256, (const uint8_t *)msg_448,
			     strlen(msg_448), kat_sha256_448));
	report("SHA-256 multi-chunk DMA, 100000 bytes",
	       run_oneshot(dev, CRYPTO_HASH_ALGO_SHA256, g_pattern, PATTERN_LEN,
			   kat_sha256_pattern));

	report("SHA-512 rejected as unsupported", run_unsupported_algo(dev));

	printk("\n%d passed, %d failed\n", g_pass, g_fail);

	if (g_fail == 0) {
		printk("All driver checks passed\n");
	}

	return 0;
}
