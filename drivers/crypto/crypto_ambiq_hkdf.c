/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <soc.h>
#include <cc312_arbiter.h>

LOG_MODULE_REGISTER(crypto_ambiq_hkdf, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_kdf

static int ambiq_hkdf_hal_status_to_errno(uint32_t status)
{
	switch (status) {
	case AM_HAL_STATUS_SUCCESS:
		return 0;
	case AM_HAL_STATUS_INVALID_ARG:
		return -EINVAL;
	case AM_HAL_STATUS_TIMEOUT:
		return -ETIMEDOUT;
	default:
		return -EIO;
	}
}

static am_hal_cc312_sha_mode_e ambiq_hkdf_hal_mode(enum hash_algo algo)
{
	switch (algo) {
	case CRYPTO_HASH_ALGO_SHA1:
		return AM_HAL_CC312_SHA1;
	case CRYPTO_HASH_ALGO_SHA224:
		return AM_HAL_CC312_SHA224;
	default:
		return AM_HAL_CC312_SHA256;
	}
}

static bool ambiq_hkdf_hash_supported(enum hash_algo algo)
{
	return (algo == CRYPTO_HASH_ALGO_SHA1) || (algo == CRYPTO_HASH_ALGO_SHA224) ||
	       (algo == CRYPTO_HASH_ALGO_SHA256);
}

static int ambiq_hkdf_derive(const struct device *dev, struct kdf_hkdf_pkt *pkt)
{
	uint32_t status;
	int ret;

	if (dev == NULL || pkt == NULL || pkt->okm == NULL || pkt->okm_len == 0U) {
		return -EINVAL;
	}

	if (pkt->ikm == NULL || pkt->ikm_len == 0U) {
		return -EINVAL;
	}

	if (!ambiq_hkdf_hash_supported(pkt->hash)) {
		return -ENOTSUP;
	}

	if (pkt->okm_len > AM_HAL_HKDF_MAX_OUTPUT_BYTES) {
		return -ERANGE;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);

	status = am_hal_cc312_hkdf(ambiq_hkdf_hal_mode(pkt->hash), pkt->salt,
				   (uint32_t)pkt->salt_len, pkt->ikm, (uint32_t)pkt->ikm_len,
				   pkt->info, (uint32_t)pkt->info_len, pkt->okm,
				   (uint32_t)pkt->okm_len);

	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_hkdf_hal_status_to_errno(status);
}

static int ambiq_hkdf_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return CAP_SYNC_OPS;
}

static int ambiq_hkdf_pm_action(const struct device *dev, enum pm_device_action action)
{
	ARG_UNUSED(dev);

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
	case PM_DEVICE_ACTION_SUSPEND:
	case PM_DEVICE_ACTION_TURN_ON:
	case PM_DEVICE_ACTION_TURN_OFF:
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int ambiq_hkdf_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return pm_device_runtime_enable(dev);
}

static DEVICE_API(crypto, ambiq_hkdf_crypto_api) = {
	.query_hw_caps = ambiq_hkdf_query_hw_caps,
	.kdf_hkdf = ambiq_hkdf_derive,
};

#define AMBIQ_HKDF_DEVICE_DEFINE(inst)                                                             \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_hkdf_pm_action);                                      \
	DEVICE_DT_INST_DEFINE(inst, ambiq_hkdf_init, PM_DEVICE_DT_INST_GET(inst), NULL, NULL,      \
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, &ambiq_hkdf_crypto_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_HKDF_DEVICE_DEFINE)
