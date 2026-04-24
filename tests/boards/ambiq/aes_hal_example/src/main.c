/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <string.h>
#include <soc.h>

#define CRYPTO_DEV_COMPAT   ambiq_crypto_aes
#define TEST_ITERATIONS     5
#define ECB_TEST_ITERATIONS TEST_ITERATIONS
#define CTR_TEST_ITERATIONS TEST_ITERATIONS
#define OFB_TEST_ITERATIONS TEST_ITERATIONS
#define CCM_TEST_ITERATIONS 5
#define GCM_TEST_ITERATIONS 5
#define XTS_TEST_ITERATIONS TEST_ITERATIONS
#define AES_DMA_ALIGNMENT   32U

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define AES_SAMPLE_NOCACHE __nocache
#else
#define AES_SAMPLE_NOCACHE
#endif

#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
#define AES_TEST_BUF_FLAGS (CAP_RAW_KEY | CAP_SYNC_OPS | CAP_INPLACE_OPS)
#define AES_TEST_OUT_BUF(buf) NULL
#define AES_TEST_OUT_BUF_MAX(len) 0U
#define AES_TEST_BUFFER_MODE "in-place"
#else
#define AES_TEST_BUF_FLAGS (CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS)
#define AES_TEST_OUT_BUF(buf) buf
#define AES_TEST_OUT_BUF_MAX(len) len
#define AES_TEST_BUFFER_MODE "separate-io"
#endif

static const struct device *crypto_dev;

static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ecb_pt_in[16];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ecb_ct[16];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ecb_pt_out[16];

static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ctr_pt_in[16];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ctr_ct[16];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ctr_pt_out[16];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ctr_iv[16];

static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ofb_pt_in[64];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ofb_ct[64];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ofb_pt_out[64];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ofb_iv[16];

static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ccm_pt_in[23];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ccm_out[31];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ccm_ct_in[31];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ccm_pt_out[23];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ccm_nonce[16];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) ccm_ad[8];

static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) gcm_pt_in[42];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) gcm_out[42];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) gcm_ct_in[42];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) gcm_pt_out[42];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) gcm_nonce[16];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) gcm_ad[20];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) gcm_tag[16];

static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) xts_pt_in[32];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) xts_ct[32];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) xts_pt_out[32];
static AES_SAMPLE_NOCACHE uint8_t __aligned(AES_DMA_ALIGNMENT) xts_data_unit[16];

static const uint8_t aes_zero_key[32] = {0};

static const uint8_t aes_ecb_expected_dec[3][16] = {
	{0x44, 0x41, 0x6a, 0xc2, 0xd1, 0xf5, 0x3c, 0x58, 0x33, 0x03, 0x91, 0x7e, 0x6b, 0xe9, 0xeb,
	 0xe0},
	{0x48, 0xe3, 0x1e, 0x9e, 0x25, 0x67, 0x18, 0xf2, 0x92, 0x29, 0x31, 0x9c, 0x19, 0xf1, 0x5b,
	 0xa4},
	{0x05, 0x8c, 0xcf, 0xfd, 0xbb, 0xcb, 0x38, 0x2d, 0x1f, 0x6f, 0x56, 0x58, 0x5d, 0x8a, 0x4a,
	 0xde},
};

static const uint8_t aes_ecb_expected_enc[3][16] = {
	{0xc3, 0x4c, 0x05, 0x2c, 0xc0, 0xda, 0x8d, 0x73, 0x45, 0x1a, 0xfe, 0x5f, 0x03, 0xbe, 0x29,
	 0x7f},
	{0xf3, 0xf6, 0x75, 0x2a, 0xe8, 0xd7, 0x83, 0x11, 0x38, 0xf0, 0x41, 0x56, 0x06, 0x31, 0xb1,
	 0x14},
	{0x8b, 0x79, 0xee, 0xcc, 0x93, 0xa0, 0xee, 0x5d, 0xff, 0x30, 0xb4, 0xea, 0x21, 0x63, 0x6d,
	 0xa4},
};

static const uint8_t aes_ofb_keys[3][32] = {
	{0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f,
	 0x3c},
	{0x8e, 0x73, 0xb0, 0xf7, 0xda, 0x0e, 0x64, 0x52, 0xc8, 0x10, 0xf3, 0x2b,
	 0x80, 0x90, 0x79, 0xe5, 0x62, 0xf8, 0xea, 0xd2, 0x52, 0x2c, 0x6b, 0x7b},
	{0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe, 0x2b, 0x73, 0xae,
	 0xf0, 0x85, 0x7d, 0x77, 0x81, 0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61,
	 0x08, 0xd7, 0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4},
};

static const uint8_t aes_ofb_iv[16] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
};

static const uint8_t aes_ofb_pt[64] = {
	0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73,
	0x93, 0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7,
	0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4,
	0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef, 0xf6, 0x9f, 0x24, 0x45,
	0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10,
};

static const uint8_t aes_ofb_ct[3][64] = {
	{0x3b, 0x3f, 0xd9, 0x2e, 0xb7, 0x2d, 0xad, 0x20, 0x33, 0x34, 0x49, 0xf8, 0xe8,
	 0x3c, 0xfb, 0x4a, 0x77, 0x89, 0x50, 0x8d, 0x16, 0x91, 0x8f, 0x03, 0xf5, 0x3c,
	 0x52, 0xda, 0xc5, 0x4e, 0xd8, 0x25, 0x97, 0x40, 0x05, 0x1e, 0x9c, 0x5f, 0xec,
	 0xf6, 0x43, 0x44, 0xf7, 0xa8, 0x22, 0x60, 0xed, 0xcc, 0x30, 0x4c, 0x65, 0x28,
	 0xf6, 0x59, 0xc7, 0x78, 0x66, 0xa5, 0x10, 0xd9, 0xc1, 0xd6, 0xae, 0x5e},
	{0xcd, 0xc8, 0x0d, 0x6f, 0xdd, 0xf1, 0x8c, 0xab, 0x34, 0xc2, 0x59, 0x09, 0xc9,
	 0x9a, 0x41, 0x74, 0xfc, 0xc2, 0x8b, 0x8d, 0x4c, 0x63, 0x83, 0x7c, 0x09, 0xe8,
	 0x17, 0x00, 0xc1, 0x10, 0x04, 0x01, 0x8d, 0x9a, 0x9a, 0xea, 0xc0, 0xf6, 0x59,
	 0x6f, 0x55, 0x9c, 0x6d, 0x4d, 0xaf, 0x59, 0xa5, 0xf2, 0x6d, 0x9f, 0x20, 0x08,
	 0x57, 0xca, 0x6c, 0x3e, 0x9c, 0xac, 0x52, 0x4b, 0xd9, 0xac, 0xc9, 0x2a},
	{0xdc, 0x7e, 0x84, 0xbf, 0xda, 0x79, 0x16, 0x4b, 0x7e, 0xcd, 0x84, 0x86, 0x98,
	 0x5d, 0x38, 0x60, 0x4f, 0xeb, 0xdc, 0x67, 0x40, 0xd2, 0x0b, 0x3a, 0xc8, 0x8f,
	 0x6a, 0xd8, 0x2a, 0x4f, 0xb0, 0x8d, 0x71, 0xab, 0x47, 0xa0, 0x86, 0xe8, 0x6e,
	 0xed, 0xf3, 0x9d, 0x1c, 0x5b, 0xba, 0x97, 0xc4, 0x08, 0x01, 0x26, 0x14, 0x1d,
	 0x67, 0xf3, 0x7b, 0xe8, 0x53, 0x8f, 0x5a, 0x8b, 0xe7, 0x40, 0xe4, 0x84},
};

static const uint8_t gcm_key_128[16] = {
	0x07, 0x1b, 0x11, 0x3b, 0x0c, 0xa7, 0x43, 0xfe,
	0xcc, 0xcf, 0x3d, 0x05, 0x1f, 0x73, 0x73, 0x82,
};

static const uint8_t gcm_key_192[24] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
	0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
};

static const uint8_t gcm_key_256[32] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
	0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
};

static void print_mode_header(const char *mode, unsigned int iterations)
{
	printk("\n[%s] %u iteration(s)\n", mode, iterations);
}

static void print_mode_summary(const char *mode, unsigned int passed, unsigned int total)
{
	printk("[%s] %s (%u/%u)\n", mode, (passed == total) ? "PASS" : "FAIL", passed, total);
}

static void dump_buf(const char *label, const uint8_t *buf, size_t len)
{
	size_t i;

	printk("%s", label);
	for (i = 0; i < len; i++) {
		printk("%02x", buf[i]);
	}
	printk("\n");
}

static bool run_ecb_key_test(uint16_t keylen, const uint8_t *exp_enc, const uint8_t *exp_dec)
{
	int rc;
	uint32_t i;
	struct cipher_ctx enc = {
		.keylen = keylen,
		.key.bit_stream = (uint8_t *)aes_zero_key,
		.flags = AES_TEST_BUF_FLAGS,
	};
	struct cipher_ctx dec = {
		.keylen = keylen,
		.key.bit_stream = (uint8_t *)aes_zero_key,
		.flags = AES_TEST_BUF_FLAGS,
	};
	struct cipher_pkt pkt_enc = {
		.in_buf = ecb_pt_in,
		.in_len = sizeof(ecb_pt_in),
		.out_buf = AES_TEST_OUT_BUF(ecb_ct),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(ecb_ct)),
	};
	struct cipher_pkt pkt_dec = {
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
		.in_buf = ecb_pt_in,
#else
		.in_buf = ecb_ct,
#endif
		.in_len = sizeof(ecb_ct),
		.out_buf = AES_TEST_OUT_BUF(ecb_pt_out),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(ecb_pt_out)),
	};

	memset(ecb_pt_in, 0, sizeof(ecb_pt_in));
	memset(ecb_ct, 0, sizeof(ecb_ct));
	memset(ecb_pt_out, 0, sizeof(ecb_pt_out));

	rc = cipher_begin_session(crypto_dev, &enc, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_ECB,
				  CRYPTO_CIPHER_OP_ENCRYPT);
	if (rc != 0) {
		return false;
	}

	for (i = 0; i < 10000U; i++) {
		rc = cipher_block_op(&enc, &pkt_enc);
		if (rc != 0) {
			break;
		}
#if !IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
		memcpy(ecb_pt_in, ecb_ct, sizeof(ecb_pt_in));
#endif
	}
	cipher_free_session(crypto_dev, &enc);
	if (rc != 0 || memcmp(ecb_pt_in, exp_enc, sizeof(ecb_pt_in)) != 0) {
		return false;
	}

	memset(ecb_ct, 0, sizeof(ecb_ct));
	memset(ecb_pt_out, 0, sizeof(ecb_pt_out));
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	memset(ecb_pt_in, 0, sizeof(ecb_pt_in));
#endif

	rc = cipher_begin_session(crypto_dev, &dec, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_ECB,
				  CRYPTO_CIPHER_OP_DECRYPT);
	if (rc != 0) {
		return false;
	}

	for (i = 0; i < 10000U; i++) {
		rc = cipher_block_op(&dec, &pkt_dec);
		if (rc != 0) {
			break;
		}
#if !IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
		memcpy(ecb_ct, ecb_pt_out, sizeof(ecb_ct));
#endif
	}
	cipher_free_session(crypto_dev, &dec);
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	if (rc != 0 || memcmp(ecb_pt_in, exp_dec, sizeof(ecb_pt_in)) != 0) {
#else
	if (rc != 0 || memcmp(ecb_ct, exp_dec, sizeof(ecb_ct)) != 0) {
#endif
		return false;
	}

	return true;
}

static int run_ecb_tests(void)
{
	static const uint16_t key_lens[] = {16, 24, 32};
	unsigned int passed = 0U;
	bool iter_ok[ARRAY_SIZE(key_lens)];
	size_t iteration;
	size_t i;

	if (!IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_TEST_ECB)) {
		printk("\n[AES-ECB] SKIPPED\n");
		return 0;
	}

	print_mode_header("AES-ECB", ECB_TEST_ITERATIONS);

	for (iteration = 0; iteration < ECB_TEST_ITERATIONS; iteration++) {
		for (i = 0; i < ARRAY_SIZE(key_lens); i++) {
			iter_ok[i] = run_ecb_key_test(key_lens[i], aes_ecb_expected_enc[i],
						      aes_ecb_expected_dec[i]);
			passed += iter_ok[i] ? 1U : 0U;
		}

		printk("  [%u/%u] 128:%s 192:%s 256:%s\n", (unsigned int)iteration + 1U,
		       ECB_TEST_ITERATIONS, iter_ok[0] ? "PASS" : "FAIL",
		       iter_ok[1] ? "PASS" : "FAIL", iter_ok[2] ? "PASS" : "FAIL");
	}

	print_mode_summary("AES-ECB", passed,
			   (unsigned int)ARRAY_SIZE(key_lens) * ECB_TEST_ITERATIONS);

	return ((unsigned int)ARRAY_SIZE(key_lens) * ECB_TEST_ITERATIONS) - passed;
}

static bool run_ctr_test(void)
{
	static const uint8_t key[16] = {
		0xAE, 0x68, 0x52, 0xF8, 0x12, 0x10, 0x67, 0xCC,
		0x4B, 0xF7, 0xA5, 0x76, 0x55, 0x77, 0xF3, 0x9E,
	};
	static const uint8_t ctr_iv_init[16] = {
		0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	};
	static const uint8_t pt[16] = {
		0x53, 0x69, 0x6E, 0x67, 0x6C, 0x65, 0x20, 0x62,
		0x6C, 0x6F, 0x63, 0x6B, 0x20, 0x6D, 0x73, 0x67,
	};
	static const uint8_t ct_exp[16] = {
		0xE4, 0x09, 0x5D, 0x4F, 0xB7, 0xA7, 0xB3, 0x79,
		0x2D, 0x61, 0x75, 0xA3, 0x26, 0x13, 0x11, 0xB8,
	};

	int rc;

	struct cipher_ctx ctx = {
		.keylen = sizeof(key),
		.key.bit_stream = (uint8_t *)key,
		.mode_params.ctr_info = {.ctr_len = 32},
		.flags = AES_TEST_BUF_FLAGS,
	};
	memset(ctr_pt_in, 0, sizeof(ctr_pt_in));
	memset(ctr_ct, 0, sizeof(ctr_ct));
	memset(ctr_pt_out, 0, sizeof(ctr_pt_out));
	memset(ctr_iv, 0, sizeof(ctr_iv));
	memcpy(ctr_pt_in, pt, sizeof(ctr_pt_in));

	struct cipher_pkt pkt_enc = {
		.in_buf = ctr_pt_in,
		.in_len = sizeof(pt),
		.out_buf = AES_TEST_OUT_BUF(ctr_ct),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(ctr_ct)),
	};
	struct cipher_pkt pkt_dec = {
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
		.in_buf = ctr_pt_in,
#else
		.in_buf = ctr_ct,
#endif
		.in_len = sizeof(ctr_ct),
		.out_buf = AES_TEST_OUT_BUF(ctr_pt_out),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(ctr_pt_out)),
	};

	rc = cipher_begin_session(crypto_dev, &ctx, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_CTR,
				  CRYPTO_CIPHER_OP_ENCRYPT);
	if (rc != 0) {
		return false;
	}

	memcpy(ctr_iv, ctr_iv_init, sizeof(ctr_iv));
	rc = cipher_ctr_op(&ctx, &pkt_enc, ctr_iv);
	if (rc != 0) {
		cipher_free_session(crypto_dev, &ctx);
		return false;
	}
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	if (memcmp(ctr_pt_in, ct_exp, sizeof(ct_exp)) != 0) {
#else
	if (memcmp(ctr_ct, ct_exp, sizeof(ct_exp)) != 0) {
#endif
		cipher_free_session(crypto_dev, &ctx);
		return false;
	}

	memcpy(ctr_iv, ctr_iv_init, sizeof(ctr_iv));
	rc = cipher_ctr_op(&ctx, &pkt_dec, ctr_iv);
	cipher_free_session(crypto_dev, &ctx);
	if (rc != 0) {
		return false;
	}
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	if (memcmp(ctr_pt_in, pt, sizeof(pt)) != 0) {
#else
	if (memcmp(ctr_pt_out, pt, sizeof(pt)) != 0) {
#endif
		return false;
	}

	return true;
}

static bool run_ccm_test(void)
{
	static const uint8_t key[16] = {
		0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
		0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
	};
	static const uint8_t nonce_init[13] = {
		0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5,
	};
	static const uint8_t ad[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	static const uint8_t pt[23] = {
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
		0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
	};
	int rc;
	uint8_t *ccm_dec_in;
	uint8_t *ccm_dec_tag;

	struct cipher_ctx enc = {
		.keylen = sizeof(key),
		.key.bit_stream = (uint8_t *)key,
		.mode_params.ccm_info = {
			.nonce_len = 13,
			.tag_len = 8,
		},
		.flags = AES_TEST_BUF_FLAGS,
	};
	struct cipher_ctx dec = {
		.keylen = sizeof(key),
		.key.bit_stream = (uint8_t *)key,
		.mode_params.ccm_info = {
			.nonce_len = 13,
			.tag_len = 8,
		},
		.flags = AES_TEST_BUF_FLAGS,
	};

	memset(ccm_pt_in, 0, sizeof(ccm_pt_in));
	memset(ccm_out, 0, sizeof(ccm_out));
	memset(ccm_ct_in, 0, sizeof(ccm_ct_in));
	memset(ccm_pt_out, 0, sizeof(ccm_pt_out));
	memset(ccm_nonce, 0, sizeof(ccm_nonce));
	memset(ccm_ad, 0, sizeof(ccm_ad));
	memcpy(ccm_pt_in, pt, sizeof(ccm_pt_in));
	memcpy(ccm_nonce, nonce_init, sizeof(nonce_init));
	memcpy(ccm_ad, ad, sizeof(ccm_ad));

	struct cipher_pkt pkt_enc = {
		.in_buf = ccm_pt_in,
		.in_len = sizeof(ccm_pt_in),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(ccm_out)),
		.out_buf = AES_TEST_OUT_BUF(ccm_out),
	};
	struct cipher_aead_pkt aead_enc = {
		.ad = ccm_ad,
		.ad_len = sizeof(ccm_ad),
		.pkt = &pkt_enc,
		.tag = ccm_out + sizeof(ccm_pt_in),
	};

	rc = cipher_begin_session(crypto_dev, &enc, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_CCM,
				  CRYPTO_CIPHER_OP_ENCRYPT);
	if (rc != 0) {
		printk("CCM enc begin failed rc=%d\n", rc);
		return false;
	}

	rc = cipher_ccm_op(&enc, &aead_enc, ccm_nonce);
	cipher_free_session(crypto_dev, &enc);
	if (rc != 0) {
		printk("CCM enc op failed rc=%d\n", rc);
		return false;
	}

#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	ccm_dec_in = ccm_pt_in;
	ccm_dec_tag = ccm_out + sizeof(ccm_pt_in);
#else
	memcpy(ccm_ct_in, ccm_out, sizeof(ccm_ct_in));
	ccm_dec_in = ccm_ct_in;
	ccm_dec_tag = ccm_ct_in + sizeof(ccm_pt_in);
#endif
	memset(ccm_nonce, 0, sizeof(ccm_nonce));
	memcpy(ccm_nonce, nonce_init, sizeof(nonce_init));
	memcpy(ccm_ad, ad, sizeof(ccm_ad));

	struct cipher_pkt pkt_dec = {
		.in_buf = ccm_dec_in,
		.in_len = sizeof(ccm_pt_in),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(ccm_pt_out)),
		.out_buf = AES_TEST_OUT_BUF(ccm_pt_out),
	};
	struct cipher_aead_pkt aead_dec = {
		.ad = ccm_ad,
		.ad_len = sizeof(ccm_ad),
		.pkt = &pkt_dec,
		.tag = ccm_dec_tag,
	};

	rc = cipher_begin_session(crypto_dev, &dec, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_CCM,
				  CRYPTO_CIPHER_OP_DECRYPT);
	if (rc != 0) {
		printk("CCM dec begin failed rc=%d\n", rc);
		return false;
	}

	rc = cipher_ccm_op(&dec, &aead_dec, ccm_nonce);
	cipher_free_session(crypto_dev, &dec);
	if (rc != 0) {
		printk("CCM dec op failed rc=%d\n", rc);
		return false;
	}
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	if (memcmp(ccm_pt_in, pt, sizeof(pt)) != 0) {
#else
	if (memcmp(ccm_pt_out, pt, sizeof(pt)) != 0) {
#endif
		printk("CCM dec mismatch\n");
		return false;
	}

	return true;
}

static bool run_ofb_key_test(uint16_t keylen, const uint8_t *key, const uint8_t *ct_exp)
{
	int rc;
	struct cipher_ctx ctx = {
		.keylen = keylen,
		.key.bit_stream = (uint8_t *)key,
		.flags = AES_TEST_BUF_FLAGS,
	};
	struct cipher_pkt pkt_enc = {
		.in_buf = ofb_pt_in,
		.in_len = sizeof(ofb_pt_in),
		.out_buf = AES_TEST_OUT_BUF(ofb_ct),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(ofb_ct)),
	};
	struct cipher_pkt pkt_dec = {
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
		.in_buf = ofb_pt_in,
#else
		.in_buf = ofb_ct,
#endif
		.in_len = sizeof(ofb_ct),
		.out_buf = AES_TEST_OUT_BUF(ofb_pt_out),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(ofb_pt_out)),
	};

	memset(ofb_pt_in, 0, sizeof(ofb_pt_in));
	memset(ofb_ct, 0, sizeof(ofb_ct));
	memset(ofb_pt_out, 0, sizeof(ofb_pt_out));
	memset(ofb_iv, 0, sizeof(ofb_iv));
	memcpy(ofb_pt_in, aes_ofb_pt, sizeof(ofb_pt_in));

	rc = cipher_begin_session(crypto_dev, &ctx, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_OFB,
				  CRYPTO_CIPHER_OP_ENCRYPT);
	if (rc != 0) {
		return false;
	}

	memcpy(ofb_iv, aes_ofb_iv, sizeof(ofb_iv));
	rc = cipher_ofb_op(&ctx, &pkt_enc, ofb_iv);
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	if (rc != 0 || memcmp(ofb_pt_in, ct_exp, sizeof(ofb_pt_in)) != 0) {
#else
	if (rc != 0 || memcmp(ofb_ct, ct_exp, sizeof(ofb_ct)) != 0) {
#endif
		cipher_free_session(crypto_dev, &ctx);
		return false;
	}

	memcpy(ofb_iv, aes_ofb_iv, sizeof(ofb_iv));
	rc = cipher_ofb_op(&ctx, &pkt_dec, ofb_iv);
	cipher_free_session(crypto_dev, &ctx);
	if (rc != 0) {
		return false;
	}

#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	return memcmp(ofb_pt_in, aes_ofb_pt, sizeof(ofb_pt_in)) == 0;
#else
	return memcmp(ofb_pt_out, aes_ofb_pt, sizeof(ofb_pt_out)) == 0;
#endif
}

static int run_ofb_tests(void)
{
	static const uint16_t key_lens[] = {16, 24, 32};
	unsigned int passed = 0U;
	bool iter_ok[ARRAY_SIZE(key_lens)];
	size_t iteration;
	size_t i;

	if (!IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_TEST_OFB)) {
		printk("\n[AES-OFB] SKIPPED\n");
		return 0;
	}

	print_mode_header("AES-OFB", OFB_TEST_ITERATIONS);

	for (iteration = 0; iteration < OFB_TEST_ITERATIONS; iteration++) {
		for (i = 0; i < ARRAY_SIZE(key_lens); i++) {
			iter_ok[i] = run_ofb_key_test(key_lens[i], aes_ofb_keys[i], aes_ofb_ct[i]);
			passed += iter_ok[i] ? 1U : 0U;
		}

		printk("  [%u/%u] 128:%s 192:%s 256:%s\n", (unsigned int)iteration + 1U,
		       OFB_TEST_ITERATIONS, iter_ok[0] ? "PASS" : "FAIL",
		       iter_ok[1] ? "PASS" : "FAIL", iter_ok[2] ? "PASS" : "FAIL");
	}

	print_mode_summary("AES-OFB", passed,
			   (unsigned int)ARRAY_SIZE(key_lens) * OFB_TEST_ITERATIONS);

	return ((unsigned int)ARRAY_SIZE(key_lens) * OFB_TEST_ITERATIONS) - passed;
}

static bool run_gcm_key_test(const uint8_t *key, uint16_t keylen)
{
	static const uint8_t nonce_init[12] = {
		0xf0, 0x76, 0x1e, 0x8d, 0xcd, 0x3d, 0x00, 0x01, 0x76, 0xd4, 0x57, 0xed,
	};
	static const uint8_t ad[20] = {
		0xe2, 0x01, 0x06, 0xd7, 0xcd, 0x0d, 0xf0, 0x76, 0x1e, 0x8d,
		0xcd, 0x3d, 0x88, 0xe5, 0x4c, 0x2a, 0x76, 0xd4, 0x57, 0xed,
	};
	static const uint8_t pt[42] = {
		0x08, 0x00, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a,
		0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
		0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x00, 0x04,
	};
	int rc;
	uint8_t *gcm_dec_in;

	struct cipher_ctx enc = {
		.keylen = keylen,
		.key.bit_stream = (uint8_t *)key,
		.mode_params.gcm_info = {
			.nonce_len = 12,
			.tag_len = 16,
		},
		.flags = AES_TEST_BUF_FLAGS,
	};
	struct cipher_ctx dec = {
		.keylen = keylen,
		.key.bit_stream = (uint8_t *)key,
		.mode_params.gcm_info = {
			.nonce_len = 12,
			.tag_len = 16,
		},
		.flags = AES_TEST_BUF_FLAGS,
	};

	memset(gcm_pt_in, 0, sizeof(gcm_pt_in));
	memset(gcm_out, 0, sizeof(gcm_out));
	memset(gcm_ct_in, 0, sizeof(gcm_ct_in));
	memset(gcm_pt_out, 0, sizeof(gcm_pt_out));
	memset(gcm_nonce, 0, sizeof(gcm_nonce));
	memset(gcm_ad, 0, sizeof(gcm_ad));
	memset(gcm_tag, 0, sizeof(gcm_tag));
	memcpy(gcm_pt_in, pt, sizeof(gcm_pt_in));
	memcpy(gcm_nonce, nonce_init, sizeof(nonce_init));
	memcpy(gcm_ad, ad, sizeof(gcm_ad));

	struct cipher_pkt pkt_enc = {
		.in_buf = gcm_pt_in,
		.in_len = sizeof(gcm_pt_in),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(gcm_out)),
		.out_buf = AES_TEST_OUT_BUF(gcm_out),
	};
	struct cipher_aead_pkt aead_enc = {
		.ad = gcm_ad,
		.ad_len = sizeof(gcm_ad),
		.pkt = &pkt_enc,
		.tag = gcm_tag,
	};

	rc = cipher_begin_session(crypto_dev, &enc, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_GCM,
				  CRYPTO_CIPHER_OP_ENCRYPT);
	if (rc != 0) {
		printk("GCM enc begin failed rc=%d\n", rc);
		return false;
	}

	rc = cipher_gcm_op(&enc, &aead_enc, gcm_nonce);
	cipher_free_session(crypto_dev, &enc);
	if (rc != 0) {
		printk("GCM enc op failed rc=%d\n", rc);
		dump_buf("GCM nonce=", nonce_init, sizeof(nonce_init));
		dump_buf("GCM aad=", ad, sizeof(ad));
		dump_buf("GCM pt=", pt, sizeof(pt));
		return false;
	}

#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	gcm_dec_in = gcm_pt_in;
#else
	memcpy(gcm_ct_in, gcm_out, sizeof(gcm_ct_in));
	gcm_dec_in = gcm_ct_in;
#endif
	memset(gcm_nonce, 0, sizeof(gcm_nonce));
	memcpy(gcm_nonce, nonce_init, sizeof(nonce_init));
	memcpy(gcm_ad, ad, sizeof(gcm_ad));

	struct cipher_pkt pkt_dec = {
		.in_buf = gcm_dec_in,
		.in_len = sizeof(gcm_pt_in),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(gcm_pt_out)),
		.out_buf = AES_TEST_OUT_BUF(gcm_pt_out),
	};
	struct cipher_aead_pkt aead_dec = {
		.ad = gcm_ad,
		.ad_len = sizeof(gcm_ad),
		.pkt = &pkt_dec,
		.tag = gcm_tag,
	};

	rc = cipher_begin_session(crypto_dev, &dec, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_GCM,
				  CRYPTO_CIPHER_OP_DECRYPT);
	if (rc != 0) {
		printk("GCM dec begin failed rc=%d\n", rc);
		return false;
	}

	rc = cipher_gcm_op(&dec, &aead_dec, gcm_nonce);
	cipher_free_session(crypto_dev, &dec);
	if (rc != 0) {
		printk("GCM dec op failed rc=%d\n", rc);
		dump_buf("GCM ct=", pkt_dec.in_buf, sizeof(gcm_ct_in));
		dump_buf("GCM tag=", gcm_tag, sizeof(gcm_tag));
		dump_buf("GCM nonce=", nonce_init, sizeof(nonce_init));
		dump_buf("GCM aad=", ad, sizeof(ad));
		return false;
	}
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	if (memcmp(gcm_pt_in, pt, sizeof(pt)) != 0) {
		printk("GCM dec mismatch\n");
		dump_buf("GCM dec pt=", gcm_pt_in, sizeof(gcm_pt_in));
#else
	if (memcmp(gcm_pt_out, pt, sizeof(pt)) != 0) {
		printk("GCM dec mismatch\n");
		dump_buf("GCM dec pt=", gcm_pt_out, sizeof(gcm_pt_out));
#endif
		dump_buf("GCM exp pt=", pt, sizeof(pt));
		return false;
	}

	return true;
}

static int run_gcm_tests(void)
{
	static const struct {
		const uint8_t *key;
		uint16_t keylen;
	} gcm_cases[] = {
		{gcm_key_128, sizeof(gcm_key_128)},
		{gcm_key_192, sizeof(gcm_key_192)},
		{gcm_key_256, sizeof(gcm_key_256)},
	};
	unsigned int passed = 0U;
	bool iter_ok[ARRAY_SIZE(gcm_cases)];
	size_t iteration;
	size_t i;

	if (!IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_TEST_GCM)) {
		printk("\n[AES-GCM] SKIPPED\n");
		return 0;
	}

	print_mode_header("AES-GCM", GCM_TEST_ITERATIONS);

	for (iteration = 0; iteration < GCM_TEST_ITERATIONS; iteration++) {
		for (i = 0; i < ARRAY_SIZE(gcm_cases); i++) {
			iter_ok[i] = run_gcm_key_test(gcm_cases[i].key, gcm_cases[i].keylen);
			passed += iter_ok[i] ? 1U : 0U;
		}

		printk("  [%u/%u] 128:%s 192:%s 256:%s\n", (unsigned int)iteration + 1U,
		       GCM_TEST_ITERATIONS, iter_ok[0] ? "PASS" : "FAIL",
		       iter_ok[1] ? "PASS" : "FAIL", iter_ok[2] ? "PASS" : "FAIL");
	}

	print_mode_summary("AES-GCM", passed,
			   (unsigned int)ARRAY_SIZE(gcm_cases) * GCM_TEST_ITERATIONS);

	return ((unsigned int)ARRAY_SIZE(gcm_cases) * GCM_TEST_ITERATIONS) - passed;
}

static bool run_xts_test(void)
{
	static const uint8_t key[32] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
		0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
		0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
	};
	static const uint8_t data_unit[16] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	static const uint8_t pt[32] = {
		0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x61,
		0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c,
		0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76,
	};
	int rc;

	struct cipher_ctx enc = {
		.keylen = sizeof(key),
		.key.bit_stream = (uint8_t *)key,
		.flags = AES_TEST_BUF_FLAGS,
	};
	struct cipher_ctx dec = {
		.keylen = sizeof(key),
		.key.bit_stream = (uint8_t *)key,
		.flags = AES_TEST_BUF_FLAGS,
	};

	memset(xts_pt_in, 0, sizeof(xts_pt_in));
	memset(xts_ct, 0, sizeof(xts_ct));
	memset(xts_pt_out, 0, sizeof(xts_pt_out));
	memset(xts_data_unit, 0, sizeof(xts_data_unit));
	memcpy(xts_pt_in, pt, sizeof(xts_pt_in));
	memcpy(xts_data_unit, data_unit, sizeof(xts_data_unit));

	struct cipher_pkt pkt_enc = {
		.in_buf = xts_pt_in,
		.in_len = sizeof(xts_pt_in),
		.out_buf = AES_TEST_OUT_BUF(xts_ct),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(xts_ct)),
	};
	struct cipher_pkt pkt_dec = {
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
		.in_buf = xts_pt_in,
#else
		.in_buf = xts_ct,
#endif
		.in_len = sizeof(xts_ct),
		.out_buf = AES_TEST_OUT_BUF(xts_pt_out),
		.out_buf_max = AES_TEST_OUT_BUF_MAX(sizeof(xts_pt_out)),
	};

	rc = cipher_begin_session(crypto_dev, &enc, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_XTS,
				  CRYPTO_CIPHER_OP_ENCRYPT);
	if (rc != 0) {
		printk("XTS enc begin failed rc=%d\n", rc);
		return false;
	}
	rc = cipher_xts_op(&enc, &pkt_enc, xts_data_unit);
	cipher_free_session(crypto_dev, &enc);
	if (rc != 0) {
		printk("XTS enc op failed rc=%d\n", rc);
		return false;
	}
#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	if (memcmp(xts_pt_in, pt, sizeof(pt)) == 0) {
		printk("XTS enc output unchanged (ct[0..3]=%02x %02x %02x %02x)\n", xts_pt_in[0],
		       xts_pt_in[1], xts_pt_in[2], xts_pt_in[3]);
#else
	if (memcmp(xts_ct, pt, sizeof(pt)) == 0) {
		printk("XTS enc output unchanged (ct[0..3]=%02x %02x %02x %02x)\n", xts_ct[0],
		       xts_ct[1], xts_ct[2], xts_ct[3]);
#endif
		return false;
	}

	memset(xts_data_unit, 0, sizeof(xts_data_unit));
	memcpy(xts_data_unit, data_unit, sizeof(xts_data_unit));

	rc = cipher_begin_session(crypto_dev, &dec, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_XTS,
				  CRYPTO_CIPHER_OP_DECRYPT);
	if (rc != 0) {
		printk("XTS dec begin failed rc=%d\n", rc);
		return false;
	}
	rc = cipher_xts_op(&dec, &pkt_dec, xts_data_unit);
	cipher_free_session(crypto_dev, &dec);
	if (rc != 0) {
		printk("XTS dec op failed rc=%d\n", rc);
		return false;
	}

#if IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS)
	if (memcmp(xts_pt_in, pt, sizeof(pt)) != 0) {
#else
	if (memcmp(xts_pt_out, pt, sizeof(pt)) != 0) {
#endif
		printk("XTS dec mismatch\n");
		return false;
	}

	return true;
}

int main(void)
{
	int fails = 0;
	bool ok;
	size_t iteration;
	unsigned int passed;

	crypto_dev = DEVICE_DT_GET_ONE(CRYPTO_DEV_COMPAT);
	if (crypto_dev == NULL || !device_is_ready(crypto_dev)) {
		printk("Ambiq crypto device not ready\n");
		return -1;
	}

	printk("Ambiq AES driver sample start\n");
#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
	printk("\nUsing __nocache memory\n\n");
#else
	printk("\nUsing normal cached memory\n\n");
#endif
	printk("Test buffer mode: %s\n\n", AES_TEST_BUFFER_MODE);

	fails += run_ecb_tests();

	if (IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_TEST_CTR)) {
		passed = 0U;
		print_mode_header("AES-CTR-128", CTR_TEST_ITERATIONS);

		for (iteration = 0; iteration < CTR_TEST_ITERATIONS; iteration++) {
			ok = run_ctr_test();
			passed += ok ? 1U : 0U;
			printk("  [%u/%u] %s\n", (unsigned int)iteration + 1U, CTR_TEST_ITERATIONS,
			       ok ? "PASS" : "FAIL");
		}
		fails += (int)(CTR_TEST_ITERATIONS - passed);
		print_mode_summary("AES-CTR-128", passed, CTR_TEST_ITERATIONS);
	} else {
		printk("\n[AES-CTR-128] SKIPPED\n");
	}

	fails += run_ofb_tests();

	if (IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_TEST_CCM)) {
		passed = 0U;
		print_mode_header("AES-CCM-128", CCM_TEST_ITERATIONS);

		for (iteration = 0; iteration < CCM_TEST_ITERATIONS; iteration++) {
			ok = run_ccm_test();
			passed += ok ? 1U : 0U;
			printk("  [%u/%u] %s\n", (unsigned int)iteration + 1U, CCM_TEST_ITERATIONS,
			       ok ? "PASS" : "FAIL");
		}
		fails += (int)(CCM_TEST_ITERATIONS - passed);
		print_mode_summary("AES-CCM-128", passed, CCM_TEST_ITERATIONS);
	} else {
		printk("\n[AES-CCM-128] SKIPPED\n");
	}

	fails += run_gcm_tests();

	if (IS_ENABLED(CONFIG_AES_HAL_EXAMPLE_TEST_XTS)) {
		passed = 0U;
		print_mode_header("AES-XTS-128", XTS_TEST_ITERATIONS);

		for (iteration = 0; iteration < XTS_TEST_ITERATIONS; iteration++) {
			ok = run_xts_test();
			passed += ok ? 1U : 0U;
			printk("  [%u/%u] %s\n", (unsigned int)iteration + 1U, XTS_TEST_ITERATIONS,
			       ok ? "PASS" : "FAIL");
		}
		fails += (int)(XTS_TEST_ITERATIONS - passed);
		print_mode_summary("AES-XTS-128", passed, XTS_TEST_ITERATIONS);
	} else {
		printk("\n[AES-XTS-128] SKIPPED\n");
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
