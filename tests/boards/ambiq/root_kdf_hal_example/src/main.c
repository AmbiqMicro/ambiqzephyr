/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/printk.h>

#include <string.h>

#include <soc.h>
#include <cc312_arbiter.h>

/*
 * CMAC-KDF demo with a compile-time key source:
 *
 * - CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_TEST_KEY (default):
 *   SP 800-108 CMAC-KDF with a fixed USER_KEY — no HUK required.
 * - CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_HUK:
 *   am_hal_cc312_root_kdf_cmac() with RKEK — needs HUK/KDR usable in LCS.
 */

#define CC312_PD DEVICE_DT_GET(DT_NODELABEL(crypto_pd))

#define KEK_LEN       32U
#define CMAC_LEN      16U
#define KDF_MAX_LABEL 64U
#define KDF_MAX_CTX   64U

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define ROOT_KDF_NOCACHE __nocache
#else
#define ROOT_KDF_NOCACHE
#endif

static ROOT_KDF_NOCACHE uint8_t g_kek_a[KEK_LEN] __aligned(32);
static ROOT_KDF_NOCACHE uint8_t g_kek_b[KEK_LEN] __aligned(32);
static ROOT_KDF_NOCACHE uint8_t g_kek_a2[KEK_LEN] __aligned(32);

/* Stable product label + per-purpose / generation context (domain separation). */
static const uint8_t label[] = "product/p256-wrap/v1";
static const uint8_t ctx_slot7_gen1[] = {
	0x00, 0x00, 0x00, 0x07, /* key id 7 */
	0x00, 0x00, 0x00, 0x01, /* generation 1 */
};
static const uint8_t ctx_slot7_gen2[] = {
	0x00, 0x00, 0x00, 0x07,
	0x00, 0x00, 0x00, 0x02,
};

#if IS_ENABLED(CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_TEST_KEY)
/* Arbitrary non-secret test key for the USER_KEY path only. */
static const uint8_t test_aes_key[32] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
	0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b,
	0x1c, 0x1d, 0x1e, 0x1f,
};
#endif

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

static int cc312_acquire(void)
{
	int ret;

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	ret = pm_device_runtime_get(CC312_PD);
	if (ret < 0) {
		k_mutex_unlock(ambiq_cc312_arbiter_lock());
	}
	return ret;
}

static void cc312_release(void)
{
	(void)pm_device_runtime_put(CC312_PD);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());
}

#if IS_ENABLED(CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_TEST_KEY)
/*
 * Same SP 800-108 counter CMAC-KDF construction as am_hal_cc312_root_kdf_cmac(),
 * but Ki is a software AES key (USER_KEY) so DM boards can exercise the path.
 */
static uint32_t derive_user_key(const uint8_t *ctx, size_t ctx_len, uint8_t *out, size_t out_len)
{
	uint32_t status;
	uint32_t i;
	uint32_t data_size;
	uint32_t n_iter;
	uint32_t iter;
	uint32_t to_copy;
	uint32_t remaining;
	uint32_t length_bits;
	am_hal_cc312_aes_context_t aes_ctx;
	uint8_t data_in[1U + KDF_MAX_LABEL + 1U + KDF_MAX_CTX + 2U];
	uint8_t cmac[CMAC_LEN];

	if ((out == NULL) || (out_len == 0U) || (out_len > 4080U) || (ctx == NULL) ||
	    (ctx_len == 0U) || (ctx_len > KDF_MAX_CTX)) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	length_bits = (uint32_t)out_len * 8U;
	i = 1U;
	memcpy(&data_in[i], label, sizeof(label) - 1U);
	i += sizeof(label) - 1U;
	data_in[i++] = 0x00;
	memcpy(&data_in[i], ctx, ctx_len);
	i += (uint32_t)ctx_len;

	if (length_bits > 0xFFU) {
		data_in[i++] = (uint8_t)((length_bits >> 8) & 0xFFU);
		data_in[i++] = (uint8_t)(length_bits & 0xFFU);
	} else {
		data_in[i++] = (uint8_t)length_bits;
	}

	data_size = i;
	n_iter = ((uint32_t)out_len + CMAC_LEN - 1U) / CMAC_LEN;
	remaining = (uint32_t)out_len;

	status = am_hal_cc312_aes_context_init(&aes_ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	status = am_hal_cc312_aes_setkey_enc(&aes_ctx, test_aes_key, 256U);
	if (status != AM_HAL_STATUS_SUCCESS) {
		(void)am_hal_cc312_aes_free(&aes_ctx);
		return status;
	}

	am_hal_cc312_context_clear(out, out_len);

	for (iter = 0U; iter < n_iter; iter++) {
		data_in[0] = (uint8_t)(iter + 1U);
		status = am_hal_aes_cmac(&aes_ctx, data_size, data_in, cmac);
		if (status != AM_HAL_STATUS_SUCCESS) {
			am_hal_cc312_context_clear(cmac, sizeof(cmac));
			am_hal_cc312_context_clear(data_in, sizeof(data_in));
			(void)am_hal_cc312_aes_free(&aes_ctx);
			return status;
		}

		to_copy = (remaining > CMAC_LEN) ? CMAC_LEN : remaining;
		memcpy(&out[iter * CMAC_LEN], cmac, to_copy);
		remaining -= to_copy;
	}

	am_hal_cc312_context_clear(cmac, sizeof(cmac));
	am_hal_cc312_context_clear(data_in, sizeof(data_in));
	(void)am_hal_cc312_aes_free(&aes_ctx);

	return AM_HAL_STATUS_SUCCESS;
}
#endif

static uint32_t derive(const uint8_t *ctx, size_t ctx_len, uint8_t *out, size_t out_len)
{
#if IS_ENABLED(CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_HUK)
	am_hal_cc312_context_clear(out, out_len);
	return am_hal_cc312_root_kdf_cmac(label, sizeof(label) - 1U, ctx, (uint32_t)ctx_len, out,
					  (uint32_t)out_len);
#else
	return derive_user_key(ctx, ctx_len, out, out_len);
#endif
}

int main(void)
{
	uint32_t status;
	int ret;
	bool ok;

#if IS_ENABLED(CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_HUK)
	printk("\nAmbiq CC312 CMAC-KDF (RKEK / HUK)\n\n");
#else
	printk("\nAmbiq CC312 CMAC-KDF (USER_KEY test key)\n\n");
#endif

	if (!device_is_ready(CC312_PD)) {
		printk("crypto_pd not ready\n");
		return 0;
	}

	ret = cc312_acquire();
	if (ret < 0) {
		printk("Failed to power CC312: %d\n", ret);
		return 0;
	}

	status = derive(ctx_slot7_gen1, sizeof(ctx_slot7_gen1), g_kek_a, sizeof(g_kek_a));
	if (status != AM_HAL_STATUS_SUCCESS) {
		cc312_release();
#if IS_ENABLED(CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_HUK)
		printk("HUK/RKEK KDF failed (status 0x%x) — HUK/KDR must be programmed "
		       "and usable in this LCS.\n",
		       status);
		printk("Rebuild with CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_TEST_KEY=y for DM bring-up.\n");
#else
		printk("USER_KEY CMAC-KDF failed (status 0x%x)\n", status);
#endif
		return 0;
	}

	status = derive(ctx_slot7_gen2, sizeof(ctx_slot7_gen2), g_kek_b, sizeof(g_kek_b));
	ok = (status == AM_HAL_STATUS_SUCCESS) && (memcmp(g_kek_a, g_kek_b, KEK_LEN) != 0);
	report("Domain separation (gen1 vs gen2)", ok);

	status = derive(ctx_slot7_gen1, sizeof(ctx_slot7_gen1), g_kek_a2, sizeof(g_kek_a2));
	ok = (status == AM_HAL_STATUS_SUCCESS) && (memcmp(g_kek_a, g_kek_a2, KEK_LEN) == 0);
	report("Deterministic re-derive (same label/ctx)", ok);

	am_hal_cc312_context_clear(g_kek_a, sizeof(g_kek_a));
	am_hal_cc312_context_clear(g_kek_b, sizeof(g_kek_b));
	am_hal_cc312_context_clear(g_kek_a2, sizeof(g_kek_a2));

	cc312_release();

	printk("\n%d passed, %d failed\n", g_pass, g_fail);
	if (g_fail == 0) {
		printk("All driver checks passed\n");
	}

	return 0;
}
