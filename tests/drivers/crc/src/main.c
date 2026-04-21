/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/crc.h>

#include <zephyr/device.h>
#include <zephyr/ztest.h>
#include <zephyr/logging/log.h>

#define WAIT_THREAD_STACK_SIZE 1024
#define WAIT_THREAD_PRIO       -10

static void wait_thread_entry(void *a, void *b, void *c);

K_THREAD_STACK_DEFINE(wait_thread_stack_area, WAIT_THREAD_STACK_SIZE);
struct k_thread wait_thread_data;

/* Define result of CRC computation */
#define RESULT_CRC_16_THREADSAFE 0xD543

/**
 * 1) Take the semaphore
 * 2) Sleep for 50 ms (to allow ztest main thread to attempt to acquire semaphore)
 * 3) Give the semaphore
 */
static void wait_thread_entry(void *a, void *b, void *c)
{
	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data[8] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4};

	struct crc_ctx ctx = {
		.type = CRC16,
		.polynomial = CRC16_POLY,
		.seed = CRC16_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	crc_begin(dev, &ctx);

	k_sleep(K_MSEC(50));

	crc_update(dev, &ctx, data, sizeof(data));
	crc_finish(dev, &ctx);
	zassert_equal(crc_verify(&ctx, RESULT_CRC_16_THREADSAFE), 0);
}

/* Define result of CRC computation */
#define RESULT_CRC_8 0xB2

/**
 * @brief Test that crc8 works
 */
ZTEST(crc, test_crc_8)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC8)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data[8] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4};

	struct crc_ctx ctx = {
		.type = CRC8,
		.polynomial = CRC8_POLY,
		.seed = CRC8_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	crc_begin(dev, &ctx);

	k_sleep(K_MSEC(50));

	crc_update(dev, &ctx, data, sizeof(data));
	crc_finish(dev, &ctx);
	zassert_equal(crc_verify(&ctx, RESULT_CRC_8), 0);
}

/* Define result of CRC computation */
#define RESULT_CRC_16 0xD543

/**
 * @brief Test that crc16 works
 */
ZTEST(crc, test_crc_16)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC16)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data[8] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4};

	struct crc_ctx ctx = {
		.type = CRC16,
		.polynomial = CRC16_POLY,
		.seed = CRC16_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_update(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_finish(dev, &ctx), 0);

	zassert_equal(crc_verify(&ctx, RESULT_CRC_16), 0);
}

/* Define result of CRC computation */
#define RESULT_CRC_CCITT 0x445C

/**
 * @brief Test that crc_16_ccitt works
 */
ZTEST(crc, test_crc_16_ccitt)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC16_CCITT)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data[8] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4};

	struct crc_ctx ctx = {
		.type = CRC16_CCITT,
		.polynomial = CRC16_CCITT_POLY,
		.seed = CRC16_CCITT_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_update(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_finish(dev, &ctx), 0);

	zassert_equal(crc_verify(&ctx, RESULT_CRC_CCITT), 0);
}

/* Define result of CRC computation */
#define RESULT_CRC_32_C 0xBB19ECB2

/**
 * @brief Test that crc_32_c works
 */
ZTEST(crc, test_crc_32_c)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC32_C)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data[8] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4};

	struct crc_ctx ctx = {
		.type = CRC32_C,
		.polynomial = CRC32C_POLY,
		.seed = CRC32_C_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_update(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_finish(dev, &ctx), 0);

	zassert_equal(crc_verify(&ctx, RESULT_CRC_32_C), 0);
}

/* Define result of CRC computation */
#define RESULT_CRC_32_IEEE 0xCEA4A6C2

/**
 * @brief Test that crc_32_ieee works
 */
ZTEST(crc, test_crc_32_ieee)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC32_IEEE)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data[8] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4};
	struct crc_ctx ctx = {
		.type = CRC32_IEEE,
		.polynomial = CRC32_IEEE_POLY,
		.seed = CRC32_IEEE_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_update(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_finish(dev, &ctx), 0);
	zassert_equal(crc_verify(&ctx, RESULT_CRC_32_IEEE), 0);
}

/* Define result of CRC computation */
#define RESULT_CRC_8_REMAIN_3 0xBB

/**
 * @brief Test that crc_8_remain_3 works
 */
ZTEST(crc, test_crc_8_remain_3)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC8)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data[11] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4, 0x3D, 0x4D, 0x51};

	struct crc_ctx ctx = {
		.type = CRC8,
		.polynomial = CRC8_POLY,
		.seed = CRC8_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_update(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_finish(dev, &ctx), 0);
	zassert_equal(crc_verify(&ctx, RESULT_CRC_8_REMAIN_3), 0);
}

/* Define result of CRC computation */
#define RESULT_CRC_16_REMAIN_1 0x2055

/**
 * @brief Test that crc_16_remain_1 works
 */
ZTEST(crc, test_crc_16_remain_1)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC16)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data[9] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4, 0x3D};

	struct crc_ctx ctx = {
		.type = CRC16,
		.polynomial = CRC16_POLY,
		.seed = CRC16_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_update(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_finish(dev, &ctx), 0);

	zassert_equal(crc_verify(&ctx, RESULT_CRC_16_REMAIN_1), 0);
}

/* Define result of CRC computation */
#define RESULT_CRC_CCITT_REMAIN_2 0x24BD

/**
 * @brief Test that crc_16_ccitt works
 */
ZTEST(crc, test_crc_16_ccitt_remain_2)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC16_CCITT)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data[10] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4, 0xFF, 0xA0};

	struct crc_ctx ctx = {
		.type = CRC16_CCITT,
		.polynomial = CRC16_CCITT_POLY,
		.seed = CRC16_CCITT_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_update(dev, &ctx, data, sizeof(data)), 0);
	zassert_equal(crc_finish(dev, &ctx), 0);

	zassert_equal(crc_verify(&ctx, RESULT_CRC_CCITT_REMAIN_2), 0);
}

/* Define result of CRC computation */
#define RESULT_DISCONTINUOUS_BUFFER 0x75

/**
 * @brief Test CRC calculation with discontinuous buffers (CRC8).
 */
ZTEST(crc, test_discontinuous_buf)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC8)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data1[5] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E};
	uint8_t data2[5] = {0x49, 0x00, 0xC4, 0x3B, 0x78};

	struct crc_ctx ctx = {
		.type = CRC8,
		.polynomial = CRC8_POLY,
		.seed = CRC8_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_INPUT | CRC_FLAG_REVERSE_OUTPUT,
	};

	zassert_equal(crc_begin(dev, &ctx), 0);
	zassert_equal(crc_update(dev, &ctx, data1, sizeof(data1)), 0);
	zassert_equal(crc_update(dev, &ctx, data2, sizeof(data2)), 0);
	zassert_equal(crc_finish(dev, &ctx), 0);
	zassert_equal(crc_verify(&ctx, RESULT_DISCONTINUOUS_BUFFER), 0);
}

/**
 * @brief Test CRC calculation with discontinuous buffers (CRC32 IEEE).
 *
 * Verifies that two sequential crc_update() calls produce the same result
 * as a single crc_update() over the combined data.
 */
ZTEST(crc, test_discontinuous_buf_crc32)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC32_IEEE)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	static const uint8_t data_full[8] __aligned(32) = {
		0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4
	};
	static const uint8_t data_half1[4] __aligned(32) = {0x0A, 0x2B, 0x4C, 0x6D};
	static const uint8_t data_half2[4] __aligned(32) = {0x8E, 0x49, 0x00, 0xC4};

	struct crc_ctx ctx_single = {
		.type = CRC32_IEEE,
		.polynomial = CRC32_IEEE_POLY,
		.seed = CRC32_IEEE_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_INPUT | CRC_FLAG_REVERSE_OUTPUT,
	};
	struct crc_ctx ctx_split = ctx_single;

	/* Single-shot */
	zassert_equal(crc_begin(dev, &ctx_single), 0);
	zassert_equal(crc_update(dev, &ctx_single, data_full, sizeof(data_full)), 0);
	zassert_equal(crc_finish(dev, &ctx_single), 0);

	/* Split into two updates */
	zassert_equal(crc_begin(dev, &ctx_split), 0);
	zassert_equal(crc_update(dev, &ctx_split, data_half1, sizeof(data_half1)), 0);
	zassert_equal(crc_update(dev, &ctx_split, data_half2, sizeof(data_half2)), 0);
	zassert_equal(crc_finish(dev, &ctx_split), 0);

	zassert_equal(ctx_single.result, ctx_split.result,
		      "Discontinuous CRC32 mismatch: 0x%08x vs 0x%08x",
		      ctx_single.result, ctx_split.result);
}

/* Define result of CRC computation */
#define RESULT_CRC_8_REMAIN_3_THREADSAFE 0xBB

/**
 * @brief Test CRC function thread safety (CRC8).
 */
ZTEST(crc, test_crc_threadsafe)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC8)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	uint8_t data[11] = {0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4, 0x3D, 0x4D, 0x51};

	struct crc_ctx ctx = {
		.type = CRC8,
		.polynomial = CRC8_POLY,
		.seed = CRC8_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	k_thread_create(&wait_thread_data, wait_thread_stack_area,
			K_THREAD_STACK_SIZEOF(wait_thread_stack_area), wait_thread_entry, NULL,
			NULL, NULL, WAIT_THREAD_PRIO, 0, K_NO_WAIT);

	k_sleep(K_MSEC(10));

	crc_begin(dev, &ctx);
	crc_update(dev, &ctx, data, sizeof(data));
	crc_finish(dev, &ctx);
	zassert_equal(crc_verify(&ctx, RESULT_CRC_8_REMAIN_3_THREADSAFE), 0);
}

/**
 * @brief Test CRC function thread safety (CRC32 IEEE).
 *
 * A second thread holds the CRC mutex for 50 ms.  The main thread's
 * crc_begin() blocks until the mutex is released, then computes its
 * own CRC and verifies the result.
 */
static void wait_thread_entry_crc32(void *a, void *b, void *c)
{
	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	static const uint8_t data[8] __aligned(32) = {
		0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4
	};

	struct crc_ctx ctx = {
		.type = CRC32_IEEE,
		.polynomial = CRC32_IEEE_POLY,
		.seed = CRC32_IEEE_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	crc_begin(dev, &ctx);

	k_sleep(K_MSEC(50));

	crc_update(dev, &ctx, data, sizeof(data));
	crc_finish(dev, &ctx);
	zassert_equal(crc_verify(&ctx, RESULT_CRC_32_IEEE), 0);
}

ZTEST(crc, test_crc_threadsafe_crc32)
{
	if (!IS_ENABLED(CONFIG_CRC_DRIVER_HAS_CRC32_IEEE)) {
		ztest_test_skip();
	}

	static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crc));

	static const uint8_t data[8] __aligned(32) = {
		0x0A, 0x2B, 0x4C, 0x6D, 0x8E, 0x49, 0x00, 0xC4
	};

	struct crc_ctx ctx = {
		.type = CRC32_IEEE,
		.polynomial = CRC32_IEEE_POLY,
		.seed = CRC32_IEEE_INIT_VAL,
		.reversed = CRC_FLAG_REVERSE_OUTPUT | CRC_FLAG_REVERSE_INPUT,
	};

	k_thread_create(&wait_thread_data, wait_thread_stack_area,
			K_THREAD_STACK_SIZEOF(wait_thread_stack_area),
			wait_thread_entry_crc32, NULL, NULL, NULL,
			WAIT_THREAD_PRIO, 0, K_NO_WAIT);

	k_sleep(K_MSEC(10));

	crc_begin(dev, &ctx);
	crc_update(dev, &ctx, data, sizeof(data));
	crc_finish(dev, &ctx);
	zassert_equal(crc_verify(&ctx, RESULT_CRC_32_IEEE), 0);
}

ZTEST_SUITE(crc, NULL, NULL, NULL, NULL, NULL);
