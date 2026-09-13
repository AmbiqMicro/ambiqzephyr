/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/crypto/crypto_ambiq_rsa.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <soc.h>
#include <cc312_arbiter.h>

LOG_MODULE_REGISTER(crypto_ambiq_rsa, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_rsa

static int ambiq_rsa_hal_status_to_errno(uint32_t status)
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

static int ambiq_rsa_mod_exp(const struct device *dev, struct crypto_ambiq_rsa_mod_exp_pkt *pkt)
{
	uint32_t status;
	int ret;

	if (dev == NULL || pkt == NULL || pkt->modulus == NULL || pkt->exponent == NULL ||
	    pkt->base == NULL || pkt->result == NULL) {
		return -EINVAL;
	}

	if (pkt->modulus_bytes == 0U || (pkt->modulus_bytes % 4U) != 0U) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	status = am_hal_cc312_rsa_mod_exp(pkt->modulus, pkt->modulus_bytes, pkt->exponent,
					  pkt->base, pkt->result);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_rsa_hal_status_to_errno(status);
}

static int ambiq_rsa_mod_exp_crt(const struct device *dev,
				 struct crypto_ambiq_rsa_mod_exp_crt_pkt *pkt)
{
	uint32_t status;
	int ret;

	if (dev == NULL || pkt == NULL || pkt->p == NULL || pkt->q == NULL || pkt->dP == NULL ||
	    pkt->dQ == NULL || pkt->qInv == NULL || pkt->base == NULL || pkt->result == NULL) {
		return -EINVAL;
	}

	if (pkt->pq_bytes == 0U || (pkt->pq_bytes % 4U) != 0U) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	status = am_hal_cc312_rsa_mod_exp_crt(pkt->p, pkt->q, pkt->dP, pkt->dQ, pkt->qInv,
					      pkt->pq_bytes, pkt->base, pkt->result);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_rsa_hal_status_to_errno(status);
}

static int ambiq_rsa_pm_action(const struct device *dev, enum pm_device_action action)
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

static int ambiq_rsa_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return pm_device_runtime_enable(dev);
}

static const struct crypto_ambiq_rsa_driver_api ambiq_rsa_api = {
	.mod_exp = ambiq_rsa_mod_exp,
	.mod_exp_crt = ambiq_rsa_mod_exp_crt,
};

#define AMBIQ_RSA_DEVICE_DEFINE(inst)                                                              \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_rsa_pm_action);                                       \
	DEVICE_DT_INST_DEFINE(inst, ambiq_rsa_init, PM_DEVICE_DT_INST_GET(inst), NULL, NULL,       \
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, &ambiq_rsa_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_RSA_DEVICE_DEFINE)
