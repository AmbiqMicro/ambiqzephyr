/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/crypto/key_agreement.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <soc.h>
#include <cc312_arbiter.h>

LOG_MODULE_REGISTER(crypto_ambiq_x25519, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_x25519

static int ambiq_x25519_hal_status_to_errno(uint32_t status)
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

static int ambiq_x25519_shared_secret(const struct device *dev,
				      struct key_agreement_x25519_pkt *pkt)
{
	uint32_t status;
	int ret;

	if (pkt == NULL || pkt->private_key == NULL || pkt->public_key == NULL ||
	    pkt->shared_secret == NULL) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	status = am_hal_cc312_x25519_scalar_mult(pkt->private_key, pkt->public_key,
						 pkt->shared_secret);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_x25519_hal_status_to_errno(status);
}

static int ambiq_x25519_public_key(const struct device *dev, const uint8_t *private_key,
				   uint8_t *public_key)
{
	uint32_t status;
	int ret;

	if (private_key == NULL || public_key == NULL) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	status = am_hal_cc312_x25519_base_mult(private_key, public_key);
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	(void)pm_device_runtime_put(dev);

	return ambiq_x25519_hal_status_to_errno(status);
}

static int ambiq_x25519_pm_action(const struct device *dev, enum pm_device_action action)
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

static int ambiq_x25519_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return pm_device_runtime_enable(dev);
}

static const struct key_agreement_driver_api ambiq_x25519_api = {
	.x25519_shared_secret = ambiq_x25519_shared_secret,
	.x25519_public_key = ambiq_x25519_public_key,
};

#define AMBIQ_X25519_DEVICE_DEFINE(inst)                                                           \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_x25519_pm_action);                                    \
	DEVICE_DT_INST_DEFINE(inst, ambiq_x25519_init, PM_DEVICE_DT_INST_GET(inst), NULL, NULL,    \
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, &ambiq_x25519_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_X25519_DEVICE_DEFINE)
