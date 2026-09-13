/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/crypto/crypto_ambiq_srp.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <soc.h>
#include <cc312_arbiter.h>

LOG_MODULE_REGISTER(crypto_ambiq_srp, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_srp

static int ambiq_srp_hal_status_to_errno(uint32_t status)
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

static am_hal_cc312_sha_mode_e ambiq_srp_hal_mode(enum hash_algo hash)
{
	switch (hash) {
	case CRYPTO_HASH_ALGO_SHA1:
		return AM_HAL_CC312_SHA1;
	case CRYPTO_HASH_ALGO_SHA224:
		return AM_HAL_CC312_SHA224;
	default:
		return AM_HAL_CC312_SHA256;
	}
}

static int ambiq_srp_group_to_hal(const struct crypto_ambiq_srp_group *group,
				  am_hal_cc312_srp_group_t *hal)
{
	if (group == NULL || hal == NULL || group->n == NULL || group->g == NULL ||
	    group->modulus_bytes == 0U) {
		return -EINVAL;
	}

	hal->pN = group->n;
	hal->pG = group->g;
	hal->modulusBytes = group->modulus_bytes;
	hal->hashMode = ambiq_srp_hal_mode(group->hash);

	return 0;
}

static int ambiq_srp_derive_x(const struct device *dev, const struct crypto_ambiq_srp_group *group,
			      const uint8_t *salt, uint32_t salt_len, const uint8_t *user,
			      uint32_t user_len, const uint8_t *password, uint32_t pass_len,
			      uint8_t *x)
{
	am_hal_cc312_srp_group_t hal;
	uint32_t status;
	int ret;

	if (x == NULL) {
		return -EINVAL;
	}

	ret = ambiq_srp_group_to_hal(group, &hal);
	if (ret != 0) {
		return ret;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	status = am_hal_cc312_srp_derive_x(&hal, salt, salt_len, user, user_len, password, pass_len,
					   x);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_srp_hal_status_to_errno(status);
}

static int ambiq_srp_derive_k(const struct device *dev, const struct crypto_ambiq_srp_group *group,
			      uint8_t *k)
{
	am_hal_cc312_srp_group_t hal;
	uint32_t status;
	int ret;

	if (k == NULL) {
		return -EINVAL;
	}

	ret = ambiq_srp_group_to_hal(group, &hal);
	if (ret != 0) {
		return ret;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	status = am_hal_cc312_srp_derive_k(&hal, k);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_srp_hal_status_to_errno(status);
}

static int ambiq_srp_derive_u(const struct device *dev, const struct crypto_ambiq_srp_group *group,
			      const uint8_t *a_pub, const uint8_t *b_pub, uint8_t *u)
{
	am_hal_cc312_srp_group_t hal;
	uint32_t status;
	int ret;

	if (a_pub == NULL || b_pub == NULL || u == NULL) {
		return -EINVAL;
	}

	ret = ambiq_srp_group_to_hal(group, &hal);
	if (ret != 0) {
		return ret;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	status = am_hal_cc312_srp_derive_u(&hal, a_pub, b_pub, u);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_srp_hal_status_to_errno(status);
}

static int ambiq_srp_derive_verifier(const struct device *dev,
				     const struct crypto_ambiq_srp_group *group, const uint8_t *x,
				     uint32_t x_len, uint8_t *v)
{
	am_hal_cc312_srp_group_t hal;
	uint32_t status;
	int ret;

	if (x == NULL || v == NULL) {
		return -EINVAL;
	}

	ret = ambiq_srp_group_to_hal(group, &hal);
	if (ret != 0) {
		return ret;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	status = am_hal_cc312_srp_derive_verifier(&hal, x, x_len, v);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_srp_hal_status_to_errno(status);
}

static int ambiq_srp_client_secret(const struct device *dev,
				   const struct crypto_ambiq_srp_group *group,
				   const uint8_t *b_pub, const uint8_t *k, uint32_t k_len,
				   const uint8_t *x, uint32_t x_len, const uint8_t *a_priv,
				   uint32_t a_len, const uint8_t *u, uint32_t u_len,
				   uint8_t *secret)
{
	am_hal_cc312_srp_group_t hal;
	uint32_t status;
	int ret;

	if (b_pub == NULL || k == NULL || x == NULL || a_priv == NULL || u == NULL ||
	    secret == NULL) {
		return -EINVAL;
	}

	ret = ambiq_srp_group_to_hal(group, &hal);
	if (ret != 0) {
		return ret;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	status = am_hal_cc312_srp_client_secret(&hal, b_pub, k, k_len, x, x_len, a_priv, a_len, u,
						u_len, secret);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_srp_hal_status_to_errno(status);
}

static int ambiq_srp_server_secret(const struct device *dev,
				   const struct crypto_ambiq_srp_group *group,
				   const uint8_t *a_pub, const uint8_t *v, const uint8_t *u,
				   uint32_t u_len, const uint8_t *b_priv, uint32_t b_len,
				   uint8_t *secret)
{
	am_hal_cc312_srp_group_t hal;
	uint32_t status;
	int ret;

	if (a_pub == NULL || v == NULL || u == NULL || b_priv == NULL || secret == NULL) {
		return -EINVAL;
	}

	ret = ambiq_srp_group_to_hal(group, &hal);
	if (ret != 0) {
		return ret;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	status = am_hal_cc312_srp_server_secret(&hal, a_pub, v, u, u_len, b_priv, b_len, secret);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_srp_hal_status_to_errno(status);
}

static int ambiq_srp_pm_action(const struct device *dev, enum pm_device_action action)
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

static int ambiq_srp_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return pm_device_runtime_enable(dev);
}

static const struct crypto_ambiq_srp_driver_api ambiq_srp_api = {
	.derive_x = ambiq_srp_derive_x,
	.derive_k = ambiq_srp_derive_k,
	.derive_u = ambiq_srp_derive_u,
	.derive_verifier = ambiq_srp_derive_verifier,
	.client_secret = ambiq_srp_client_secret,
	.server_secret = ambiq_srp_server_secret,
};

#define AMBIQ_SRP_DEVICE_DEFINE(inst)                                                              \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_srp_pm_action);                                       \
	DEVICE_DT_INST_DEFINE(inst, ambiq_srp_init, PM_DEVICE_DT_INST_GET(inst), NULL, NULL,       \
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, &ambiq_srp_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_SRP_DEVICE_DEFINE)
