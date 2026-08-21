/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * CC312 hardware counterpart to tests/benchmarks/mbedtls.
 *
 * The loop, buffer size and output format are taken from that benchmark so the
 * rows line up: each case runs for one second to get KiB/s, then a fixed 1024
 * iterations to get ns/byte. Row titles match the software ones wherever the
 * same primitive exists on both sides, so the two logs diff directly.
 *
 * CC312 shares one interrupt across its engines, so one engine is measured per
 * image and the overlay picks which.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/crypto/crypto.h>

#include <string.h>

#define BUFSIZE       1024
#define HEADER_FORMAT "  %-24s :  "

static uint8_t buf[BUFSIZE] __aligned(32);
static uint8_t out[BUFSIZE + 32] __aligned(32);
static uint8_t ad[16] __aligned(32);
static uint8_t iv[16] __aligned(32);
static uint8_t key[32] __aligned(32);

static volatile int alarmed;
static struct k_work_delayable alarm_work;
static struct k_work_sync work_sync;

static void alarm_timeout(struct k_work *work)
{
	ARG_UNUSED(work);
	alarmed = 1;
}

static void set_alarm(int seconds)
{
	alarmed = 0;
	k_work_schedule(&alarm_work, K_SECONDS(seconds));
}

/* Mirrors TIME_AND_TSC() in the mbedTLS benchmark. */
#define TIME_AND_TSC(TITLE, CODE)                                                                  \
	do {                                                                                       \
		unsigned long ii, jj;                                                              \
		uint32_t tsc;                                                                      \
		uint64_t delta;                                                                    \
		int ret = 0;                                                                       \
                                                                                                   \
		printk(HEADER_FORMAT, TITLE);                                                      \
                                                                                                   \
		set_alarm(1);                                                                      \
		for (ii = 1; ret == 0 && !alarmed; ii++) {                                         \
			ret = CODE;                                                                \
		}                                                                                  \
                                                                                                   \
		tsc = k_cycle_get_32();                                                            \
		for (jj = 0; ret == 0 && jj < 1024; jj++) {                                        \
			ret = CODE;                                                                \
		}                                                                                  \
                                                                                                   \
		delta = k_cycle_get_32() - tsc;                                                    \
		delta = k_cyc_to_ns_floor64(delta);                                                \
                                                                                                   \
		(void)k_work_cancel_delayable_sync(&alarm_work, &work_sync);                       \
                                                                                                   \
		if (ret != 0) {                                                                    \
			printk("FAILED: %d\n", ret);                                               \
		} else {                                                                           \
			printk("%9lu KiB/s,  %9lu ns/byte\n", ii * BUFSIZE / 1024,                 \
			       (unsigned long)(delta / (jj * BUFSIZE)));                           \
		}                                                                                  \
	} while (0)

#if DT_HAS_COMPAT_STATUS_OKAY(ambiq_crypto_sha)
#define CRYPTO_DEV_COMPAT ambiq_crypto_sha
#elif DT_HAS_COMPAT_STATUS_OKAY(ambiq_crypto_chacha)
#define CRYPTO_DEV_COMPAT ambiq_crypto_chacha
#elif DT_HAS_COMPAT_STATUS_OKAY(ambiq_crypto_aes)
#define CRYPTO_DEV_COMPAT ambiq_crypto_aes
#else
#error "Enable exactly one CC312 engine with the matching overlay"
#endif

static const struct device *dev;

#if DT_HAS_COMPAT_STATUS_OKAY(ambiq_crypto_sha)
static int hash_once(enum hash_algo algo)
{
	struct hash_ctx ctx = {.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS};
	struct hash_pkt pkt = {.in_buf = buf, .in_len = BUFSIZE, .out_buf = out};
	int rc = hash_begin_session(dev, &ctx, algo);

	if (rc != 0) {
		return rc;
	}

	rc = hash_compute(&ctx, &pkt);
	hash_free_session(dev, &ctx);

	return rc;
}

static void run_engine(void)
{
	TIME_AND_TSC("SHA-1", hash_once(CRYPTO_HASH_ALGO_SHA1));
	TIME_AND_TSC("SHA-256", hash_once(CRYPTO_HASH_ALGO_SHA256));
}
#else
static uint8_t tag[16] __aligned(32);
static int cipher_once(enum cipher_algo algo, enum cipher_mode mode, size_t keylen)
{
	struct cipher_ctx ctx = {
		.keylen = keylen,
		.key.bit_stream = key,
		.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS,
	};
	struct cipher_pkt pkt = {
		.in_buf = buf,
		.in_len = BUFSIZE,
		.out_buf = out,
		.out_buf_max = sizeof(out),
	};
	struct cipher_aead_pkt aead = {
		.pkt = &pkt,
		.ad = ad,
		.ad_len = sizeof(ad),
		.tag = tag,
	};
	int rc;

	if (mode == CRYPTO_CIPHER_MODE_CTR) {
		ctx.mode_params.ctr_info.ctr_len = 32;
	} else if (mode == CRYPTO_CIPHER_MODE_CCM) {
		ctx.mode_params.ccm_info.tag_len = sizeof(tag);
		ctx.mode_params.ccm_info.nonce_len = 13;
	} else if (mode == CRYPTO_CIPHER_MODE_GCM) {
		ctx.mode_params.gcm_info.tag_len = sizeof(tag);
		ctx.mode_params.gcm_info.nonce_len = 12;
	}

	rc = cipher_begin_session(dev, &ctx, algo, mode, CRYPTO_CIPHER_OP_ENCRYPT);
	if (rc != 0) {
		return rc;
	}

	switch (mode) {
	case CRYPTO_CIPHER_MODE_ECB:
		pkt.in_len = 16;
		rc = cipher_block_op(&ctx, &pkt);
		break;
	case CRYPTO_CIPHER_MODE_CTR:
		rc = cipher_ctr_op(&ctx, &pkt, iv);
		break;
	case CRYPTO_CIPHER_MODE_CCM:
		rc = cipher_ccm_op(&ctx, &aead, iv);
		break;
	case CRYPTO_CIPHER_MODE_GCM:
		rc = cipher_gcm_op(&ctx, &aead, iv);
		break;
	case CRYPTO_CIPHER_MODE_CHACHA20:
		rc = cipher_chacha20_op(&ctx, &pkt, iv);
		break;
	default:
		rc = -ENOTSUP;
		break;
	}

	cipher_free_session(dev, &ctx);

	return rc;
}

#if DT_HAS_COMPAT_STATUS_OKAY(ambiq_crypto_chacha)
static void run_engine(void)
{
	TIME_AND_TSC("ChaCha20",
		     cipher_once(CRYPTO_CIPHER_ALGO_CHACHA20, CRYPTO_CIPHER_MODE_CHACHA20, 32));
}
#else
static void run_engine(void)
{
	static const size_t keylens[] = {16, 24, 32};
	char title[32];

	for (size_t i = 0; i < ARRAY_SIZE(keylens); i++) {
		unsigned int bits = (unsigned int)keylens[i] * 8U;

		snprintk(title, sizeof(title), "AES-ECB-%u", bits);
		TIME_AND_TSC(title, cipher_once(CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_ECB,
						keylens[i]));

		snprintk(title, sizeof(title), "AES-CTR-%u", bits);
		TIME_AND_TSC(title, cipher_once(CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_CTR,
						keylens[i]));

		snprintk(title, sizeof(title), "AES-GCM-%u", bits);
		TIME_AND_TSC(title, cipher_once(CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_GCM,
						keylens[i]));

		snprintk(title, sizeof(title), "AES-CCM-%u", bits);
		TIME_AND_TSC(title, cipher_once(CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_CCM,
						keylens[i]));
	}
}
#endif /* ambiq_crypto_chacha */
#endif /* ambiq_crypto_sha */

int main(void)
{
	k_work_init_delayable(&alarm_work, alarm_timeout);

	for (int i = 0; i < BUFSIZE; i++) {
		buf[i] = (uint8_t)i;
	}
	memset(key, 0x2b, sizeof(key));
	memset(iv, 0xf0, sizeof(iv));
	memset(ad, 0xad, sizeof(ad));

	dev = DEVICE_DT_GET_ONE(CRYPTO_DEV_COMPAT);
	if (!device_is_ready(dev)) {
		printk("CC312 device not ready\n");
		return 0;
	}

	printk("\nCC312 hardware benchmark\n\n");

	run_engine();

	printk("\nDone\n");

	return 0;
}
