/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>

#include <string.h>

#define CRYPTO_DEV_COMPAT      ambiq_crypto_chacha
#define CHACHA_DMA_ALIGNMENT   32U
#ifndef CHACHA_BLOCK_SIZE
#define CHACHA_BLOCK_SIZE      64U
#endif
#define CHACHA_TEST_ITERATIONS 1000

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define CHACHA_SAMPLE_NOCACHE __nocache
#else
#define CHACHA_SAMPLE_NOCACHE
#endif

#if IS_ENABLED(CONFIG_CHACHA_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
#define CHACHA_TEST_BUF_FLAGS        (CAP_RAW_KEY | CAP_SYNC_OPS | CAP_INPLACE_OPS)
#define CHACHA_TEST_OUT_BUF(buf)     NULL
#define CHACHA_TEST_OUT_BUF_MAX(len) 0U
#define CHACHA_TEST_BUFFER_MODE      "in-place"
#else
#define CHACHA_TEST_BUF_FLAGS        (CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS)
#define CHACHA_TEST_OUT_BUF(buf)     (buf)
#define CHACHA_TEST_OUT_BUF_MAX(len) (len)
#define CHACHA_TEST_BUFFER_MODE      "separate-io"
#endif

static const struct device *crypto_dev;

/* RFC 8439 Section 2.4.2 known-answer test vector */

static const uint8_t g_key[32] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
	0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
};

static const uint8_t g_nonce_init[12] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x00, 0x00, 0x00, 0x00,
};

#define CHACHA_TEST_INITIAL_COUNTER 1U

static const uint8_t g_plaintext[114] =
	"Ladies and Gentlemen of the class of '99: If I could offer you only "
	"one tip for the future, sunscreen would be it.";

static const uint8_t g_expected_ct[114] = {
	0x6e, 0x2e, 0x35, 0x9a, 0x25, 0x68, 0xf9, 0x80, 0x41, 0xba, 0x07, 0x28, 0xdd, 0x0d, 0x69,
	0x81, 0xe9, 0x7e, 0x7a, 0xec, 0x1d, 0x43, 0x60, 0xc2, 0x0a, 0x27, 0xaf, 0xcc, 0xfd, 0x9f,
	0xae, 0x0b, 0xf9, 0x1b, 0x65, 0xc5, 0x52, 0x47, 0x33, 0xab, 0x8f, 0x59, 0x3d, 0xab, 0xcd,
	0x62, 0xb3, 0x57, 0x16, 0x39, 0xd6, 0x24, 0xe6, 0x51, 0x52, 0xab, 0x8f, 0x53, 0x0c, 0x35,
	0x9f, 0x08, 0x61, 0xd8, 0x07, 0xca, 0x0d, 0xbf, 0x50, 0x0d, 0x6a, 0x61, 0x56, 0xa3, 0x8e,
	0x08, 0x8a, 0x22, 0xb6, 0x5e, 0x52, 0xbc, 0x51, 0x4d, 0x16, 0xcc, 0xf8, 0x06, 0x81, 0x8c,
	0xe9, 0x1a, 0xb7, 0x79, 0x37, 0x36, 0x5a, 0xf9, 0x0b, 0xbf, 0x74, 0xa3, 0x5b, 0xe6, 0xb4,
	0x0b, 0x8e, 0xed, 0xf2, 0x78, 0x5e, 0x42, 0x87, 0x4d,
};

#define CHACHA_TEST_LEN ((uint32_t)sizeof(g_plaintext))
#define CHACHA_TEST_BUF 128U

static CHACHA_SAMPLE_NOCACHE uint8_t g_in[CHACHA_TEST_BUF] __aligned(CHACHA_DMA_ALIGNMENT);
static CHACHA_SAMPLE_NOCACHE uint8_t g_out[CHACHA_TEST_BUF] __aligned(CHACHA_DMA_ALIGNMENT);
static CHACHA_SAMPLE_NOCACHE uint8_t g_nonce[12] __aligned(CHACHA_DMA_ALIGNMENT);

static void print_mode_header(const char *mode, unsigned int iterations)
{
	printk("\n[%s] %u iteration(s)\n", mode, iterations);
}

static void print_mode_summary(const char *mode, unsigned int passed, unsigned int total)
{
	printk("[%s] %s (%u/%u)\n", mode, (passed == total) ? "PASS" : "FAIL", passed, total);
}

static bool run_encrypt_kat(void)
{
	int rc;
	struct cipher_ctx ctx = {
		.keylen = sizeof(g_key),
		.key.bit_stream = (uint8_t *)g_key,
		.mode_params.ctr_info = {.ctr_len = CHACHA_TEST_INITIAL_COUNTER},
		.flags = CHACHA_TEST_BUF_FLAGS,
	};

	memset(g_in, 0, sizeof(g_in));
	memset(g_out, 0, sizeof(g_out));
	memcpy(g_in, g_plaintext, CHACHA_TEST_LEN);
	memcpy(g_nonce, g_nonce_init, sizeof(g_nonce_init));

	struct cipher_pkt pkt = {
		.in_buf = g_in,
		.in_len = CHACHA_TEST_LEN,
		.out_buf = CHACHA_TEST_OUT_BUF(g_out),
		.out_buf_max = CHACHA_TEST_OUT_BUF_MAX(CHACHA_TEST_BUF),
	};

	rc = cipher_begin_session(crypto_dev, &ctx, CRYPTO_CIPHER_ALGO_CHACHA20,
				  CRYPTO_CIPHER_MODE_CHACHA20, CRYPTO_CIPHER_OP_ENCRYPT);
	if (rc != 0) {
		printk("  begin_session failed rc=%d\n", rc);
		return false;
	}

	rc = cipher_chacha20_op(&ctx, &pkt, g_nonce);
	cipher_free_session(crypto_dev, &ctx);
	if (rc != 0) {
		printk("  crypt_op failed rc=%d\n", rc);
		return false;
	}

#if IS_ENABLED(CONFIG_CHACHA_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	return memcmp(g_in, g_expected_ct, CHACHA_TEST_LEN) == 0;
#else
	return memcmp(g_out, g_expected_ct, CHACHA_TEST_LEN) == 0;
#endif
}

static bool run_decrypt_roundtrip(void)
{
	int rc;
	struct cipher_ctx ctx = {
		.keylen = sizeof(g_key),
		.key.bit_stream = (uint8_t *)g_key,
		.mode_params.ctr_info = {.ctr_len = CHACHA_TEST_INITIAL_COUNTER},
		.flags = CHACHA_TEST_BUF_FLAGS,
	};

	memset(g_in, 0, sizeof(g_in));
	memset(g_out, 0, sizeof(g_out));
	memcpy(g_in, g_expected_ct, CHACHA_TEST_LEN);
	memcpy(g_nonce, g_nonce_init, sizeof(g_nonce_init));

	struct cipher_pkt pkt = {
		.in_buf = g_in,
		.in_len = CHACHA_TEST_LEN,
		.out_buf = CHACHA_TEST_OUT_BUF(g_out),
		.out_buf_max = CHACHA_TEST_OUT_BUF_MAX(CHACHA_TEST_BUF),
	};

	rc = cipher_begin_session(crypto_dev, &ctx, CRYPTO_CIPHER_ALGO_CHACHA20,
				  CRYPTO_CIPHER_MODE_CHACHA20, CRYPTO_CIPHER_OP_DECRYPT);
	if (rc != 0) {
		printk("  begin_session failed rc=%d\n", rc);
		return false;
	}

	rc = cipher_chacha20_op(&ctx, &pkt, g_nonce);
	cipher_free_session(crypto_dev, &ctx);
	if (rc != 0) {
		printk("  crypt_op failed rc=%d\n", rc);
		return false;
	}

#if IS_ENABLED(CONFIG_CHACHA_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	return memcmp(g_in, g_plaintext, CHACHA_TEST_LEN) == 0;
#else
	return memcmp(g_out, g_plaintext, CHACHA_TEST_LEN) == 0;
#endif
}

/*
 * Split streaming: encrypt in two separate HAL calls to verify that
 * the block-counter continuation works.  The driver re-inits per call,
 * so we manually advance the counter (ctr_len) for the second chunk.
 */
static bool run_split_streaming(void)
{
	int rc;

	/* First chunk: blocks starting at CHACHA_TEST_INITIAL_COUNTER */
	struct cipher_ctx ctx1 = {
		.keylen = sizeof(g_key),
		.key.bit_stream = (uint8_t *)g_key,
		.mode_params.ctr_info = {.ctr_len = CHACHA_TEST_INITIAL_COUNTER},
		.flags = CHACHA_TEST_BUF_FLAGS,
	};
	/* Second chunk: starts one block later */
	struct cipher_ctx ctx2 = {
		.keylen = sizeof(g_key),
		.key.bit_stream = (uint8_t *)g_key,
		.mode_params.ctr_info = {.ctr_len = CHACHA_TEST_INITIAL_COUNTER + 1U},
		.flags = CHACHA_TEST_BUF_FLAGS,
	};

	memset(g_in, 0, sizeof(g_in));
	memset(g_out, 0, sizeof(g_out));
	memcpy(g_in, g_plaintext, CHACHA_TEST_LEN);
	memcpy(g_nonce, g_nonce_init, sizeof(g_nonce_init));

	struct cipher_pkt pkt1 = {
		.in_buf = g_in,
		.in_len = CHACHA_BLOCK_SIZE,
		.out_buf = CHACHA_TEST_OUT_BUF(g_out),
		.out_buf_max = CHACHA_TEST_OUT_BUF_MAX(CHACHA_BLOCK_SIZE),
	};

	rc = cipher_begin_session(crypto_dev, &ctx1, CRYPTO_CIPHER_ALGO_CHACHA20,
				  CRYPTO_CIPHER_MODE_CHACHA20, CRYPTO_CIPHER_OP_ENCRYPT);
	if (rc != 0) {
		return false;
	}
	rc = cipher_chacha20_op(&ctx1, &pkt1, g_nonce);
	cipher_free_session(crypto_dev, &ctx1);
	if (rc != 0) {
		return false;
	}

#if IS_ENABLED(CONFIG_CHACHA_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	struct cipher_pkt pkt2 = {
		.in_buf = g_in + CHACHA_BLOCK_SIZE,
		.in_len = CHACHA_TEST_LEN - CHACHA_BLOCK_SIZE,
		.out_buf = NULL,
		.out_buf_max = 0U,
	};
#else
	struct cipher_pkt pkt2 = {
		.in_buf = g_in + CHACHA_BLOCK_SIZE,
		.in_len = CHACHA_TEST_LEN - CHACHA_BLOCK_SIZE,
		.out_buf = g_out + CHACHA_BLOCK_SIZE,
		.out_buf_max = CHACHA_TEST_BUF - CHACHA_BLOCK_SIZE,
	};
#endif

	rc = cipher_begin_session(crypto_dev, &ctx2, CRYPTO_CIPHER_ALGO_CHACHA20,
				  CRYPTO_CIPHER_MODE_CHACHA20, CRYPTO_CIPHER_OP_ENCRYPT);
	if (rc != 0) {
		return false;
	}
	rc = cipher_chacha20_op(&ctx2, &pkt2, g_nonce);
	cipher_free_session(crypto_dev, &ctx2);
	if (rc != 0) {
		return false;
	}

#if IS_ENABLED(CONFIG_CHACHA_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	return memcmp(g_in, g_expected_ct, CHACHA_TEST_LEN) == 0;
#else
	return memcmp(g_out, g_expected_ct, CHACHA_TEST_LEN) == 0;
#endif
}

int main(void)
{
	int fails = 0;
	bool ok;
	unsigned int passed;
	unsigned int iteration;

	crypto_dev = DEVICE_DT_GET_ONE(CRYPTO_DEV_COMPAT);
	if (crypto_dev == NULL || !device_is_ready(crypto_dev)) {
		printk("Ambiq ChaCha crypto device not ready\n");
		return -1;
	}

	printk("Ambiq ChaCha driver sample start\n");
#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
	printk("\nUsing __nocache memory\n\n");
#else
	printk("\nUsing normal cached memory\n\n");
#endif
	printk("Test buffer mode: %s\n\n", CHACHA_TEST_BUFFER_MODE);

	if (IS_ENABLED(CONFIG_CHACHA_HAL_EXAMPLE_TEST_ENCRYPT_KAT)) {
		passed = 0U;
		print_mode_header("ChaCha20-Encrypt-KAT", CHACHA_TEST_ITERATIONS);
		for (iteration = 0; iteration < CHACHA_TEST_ITERATIONS; iteration++) {
			ok = run_encrypt_kat();
			passed += ok ? 1U : 0U;
			if (!ok) {
				printk("  [%u/%u] FAIL\n", iteration + 1U, CHACHA_TEST_ITERATIONS);
			}
		}
		fails += (int)(CHACHA_TEST_ITERATIONS - passed);
		print_mode_summary("ChaCha20-Encrypt-KAT", passed, CHACHA_TEST_ITERATIONS);
	} else {
		printk("\n[ChaCha20-Encrypt-KAT] SKIPPED\n");
	}

	if (IS_ENABLED(CONFIG_CHACHA_HAL_EXAMPLE_TEST_DECRYPT_ROUNDTRIP)) {
		passed = 0U;
		print_mode_header("ChaCha20-Decrypt-RoundTrip", CHACHA_TEST_ITERATIONS);
		for (iteration = 0; iteration < CHACHA_TEST_ITERATIONS; iteration++) {
			ok = run_decrypt_roundtrip();
			passed += ok ? 1U : 0U;
			if (!ok) {
				printk("  [%u/%u] FAIL\n", iteration + 1U, CHACHA_TEST_ITERATIONS);
			}
		}
		fails += (int)(CHACHA_TEST_ITERATIONS - passed);
		print_mode_summary("ChaCha20-Decrypt-RoundTrip", passed, CHACHA_TEST_ITERATIONS);
	} else {
		printk("\n[ChaCha20-Decrypt-RoundTrip] SKIPPED\n");
	}

	if (IS_ENABLED(CONFIG_CHACHA_HAL_EXAMPLE_TEST_SPLIT_STREAMING)) {
		passed = 0U;
		print_mode_header("ChaCha20-Split-Streaming", CHACHA_TEST_ITERATIONS);
		for (iteration = 0; iteration < CHACHA_TEST_ITERATIONS; iteration++) {
			ok = run_split_streaming();
			passed += ok ? 1U : 0U;
			if (!ok) {
				printk("  [%u/%u] FAIL\n", iteration + 1U, CHACHA_TEST_ITERATIONS);
			}
		}
		fails += (int)(CHACHA_TEST_ITERATIONS - passed);
		print_mode_summary("ChaCha20-Split-Streaming", passed, CHACHA_TEST_ITERATIONS);
	} else {
		printk("\n[ChaCha20-Split-Streaming] SKIPPED\n");
	}

	if (fails == 0) {
		printk("All driver checks passed\n");
	} else {
		printk("Driver checks failed: %d\n", fails);
	}

#if IS_ENABLED(CONFIG_REBOOT)
	k_msleep(1000);
	sys_reboot(SYS_REBOOT_COLD);
#endif

	return (fails == 0) ? 0 : -1;
}
