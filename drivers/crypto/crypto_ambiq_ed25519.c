/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/crypto/crypto_ambiq_ed25519.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/util.h>

#include <soc.h>
#include <cc312_arbiter.h>

LOG_MODULE_REGISTER(crypto_ambiq_ed25519, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_ed25519

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define AMBIQ_ED25519_NOCACHE __nocache
#else
#define AMBIQ_ED25519_NOCACHE
#endif

#define AMBIQ_ED25519_SEED_SIZE    32U
#define AMBIQ_ED25519_PUB_SIZE     32U
#define AMBIQ_ED25519_SIG_SIZE     64U
#define AMBIQ_ED25519_MSG_SCRATCH  64U

struct ambiq_ed25519_dma {
	uint8_t seed[AMBIQ_ED25519_SEED_SIZE] __aligned(32);
	uint8_t public_key[AMBIQ_ED25519_PUB_SIZE] __aligned(32);
	uint8_t signature[AMBIQ_ED25519_SIG_SIZE] __aligned(32);
	uint8_t msg[AMBIQ_ED25519_MSG_SCRATCH] __aligned(32);
};

struct ambiq_ed25519_data {
	struct ambiq_ed25519_dma *dma;
};

struct ambiq_ed25519_config {
	struct ambiq_ed25519_dma *dma;
};

static int ambiq_ed25519_hal_status_to_errno(uint32_t status)
{
	switch (status) {
	case AM_HAL_STATUS_SUCCESS:
		return 0;
	case AM_HAL_STATUS_FAIL:
		return -EBADMSG;
	case AM_HAL_STATUS_INVALID_ARG:
		return -EINVAL;
	case AM_HAL_STATUS_TIMEOUT:
		return -ETIMEDOUT;
	default:
		return -EIO;
	}
}

static int ambiq_ed25519_public_key(const struct device *dev, const uint8_t *seed,
				    uint8_t *public_key)
{
	struct ambiq_ed25519_data *data = dev->data;
	uint32_t status;
	int ret;

	if (seed == NULL || public_key == NULL || data == NULL || data->dma == NULL) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);

	(void)am_hal_cc312_memcpy(data->dma->seed, seed, AMBIQ_ED25519_SEED_SIZE);

	status = am_hal_cc312_ed25519_public_key(data->dma->seed, data->dma->public_key);
	if (status == AM_HAL_STATUS_SUCCESS) {
		(void)am_hal_cc312_memcpy(public_key, data->dma->public_key,
					  AMBIQ_ED25519_PUB_SIZE);
	} else {
		am_hal_cc312_context_clear(public_key, AMBIQ_ED25519_PUB_SIZE);
	}
	ret = ambiq_ed25519_hal_status_to_errno(status);

	am_hal_cc312_context_clear(data->dma->seed, sizeof(data->dma->seed));
	am_hal_cc312_context_clear(data->dma->public_key, sizeof(data->dma->public_key));

	k_mutex_unlock(ambiq_cc312_arbiter_lock());
	(void)pm_device_runtime_put(dev);

	return ret;
}

static int ambiq_ed25519_sign(const struct device *dev, const uint8_t *seed,
			      const uint8_t *message, uint32_t message_len, uint8_t *signature)
{
	struct ambiq_ed25519_data *data = dev->data;
	const uint8_t *msg_ptr;
	uint32_t status;
	int ret;

	if (seed == NULL || signature == NULL || (message_len != 0U && message == NULL) ||
	    data == NULL || data->dma == NULL) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);

	(void)am_hal_cc312_memcpy(data->dma->seed, seed, AMBIQ_ED25519_SEED_SIZE);

	if (message_len == 0U) {
		msg_ptr = NULL;
	} else if (message_len <= AMBIQ_ED25519_MSG_SCRATCH) {
		(void)am_hal_cc312_memcpy(data->dma->msg, message, message_len);
		msg_ptr = data->dma->msg;
	} else {
		msg_ptr = message;
	}

	status = am_hal_cc312_ed25519_sign(data->dma->seed, msg_ptr, message_len,
					   data->dma->signature);
	if (status == AM_HAL_STATUS_SUCCESS) {
		(void)am_hal_cc312_memcpy(signature, data->dma->signature, AMBIQ_ED25519_SIG_SIZE);
	} else {
		am_hal_cc312_context_clear(signature, AMBIQ_ED25519_SIG_SIZE);
	}
	ret = ambiq_ed25519_hal_status_to_errno(status);

	am_hal_cc312_context_clear(data->dma->seed, sizeof(data->dma->seed));
	am_hal_cc312_context_clear(data->dma->signature, sizeof(data->dma->signature));
	if (message_len != 0U && message_len <= AMBIQ_ED25519_MSG_SCRATCH) {
		am_hal_cc312_context_clear(data->dma->msg, message_len);
	}

	k_mutex_unlock(ambiq_cc312_arbiter_lock());
	(void)pm_device_runtime_put(dev);

	return ret;
}

static int ambiq_ed25519_verify(const struct device *dev, const uint8_t *public_key,
				const uint8_t *message, uint32_t message_len,
				const uint8_t *signature)
{
	struct ambiq_ed25519_data *data = dev->data;
	const uint8_t *msg_ptr;
	uint32_t status;
	int ret;

	if (public_key == NULL || signature == NULL || data == NULL || data->dma == NULL ||
	    (message_len != 0U && message == NULL)) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);

	(void)am_hal_cc312_memcpy(data->dma->public_key, public_key, AMBIQ_ED25519_PUB_SIZE);

	(void)am_hal_cc312_memcpy(data->dma->signature, signature, AMBIQ_ED25519_SIG_SIZE);

	if (message_len == 0U) {
		msg_ptr = NULL;
	} else if (message_len <= AMBIQ_ED25519_MSG_SCRATCH) {
		(void)am_hal_cc312_memcpy(data->dma->msg, message, message_len);
		msg_ptr = data->dma->msg;
	} else {
		msg_ptr = message;
	}

	status = am_hal_cc312_ed25519_verify(data->dma->public_key, msg_ptr, message_len,
					     data->dma->signature);
	ret = ambiq_ed25519_hal_status_to_errno(status);

	am_hal_cc312_context_clear(data->dma->public_key, sizeof(data->dma->public_key));
	am_hal_cc312_context_clear(data->dma->signature, sizeof(data->dma->signature));
	if (message_len != 0U && message_len <= AMBIQ_ED25519_MSG_SCRATCH) {
		am_hal_cc312_context_clear(data->dma->msg, message_len);
	}

	k_mutex_unlock(ambiq_cc312_arbiter_lock());
	(void)pm_device_runtime_put(dev);

	return ret;
}

static int ambiq_ed25519_pm_action(const struct device *dev, enum pm_device_action action)
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

static int ambiq_ed25519_init(const struct device *dev)
{
	const struct ambiq_ed25519_config *cfg = dev->config;
	struct ambiq_ed25519_data *data = dev->data;

	data->dma = cfg->dma;
	return pm_device_runtime_enable(dev);
}

static const struct crypto_ambiq_ed25519_driver_api ambiq_ed25519_api = {
	.public_key = ambiq_ed25519_public_key,
	.sign = ambiq_ed25519_sign,
	.verify = ambiq_ed25519_verify,
};

#define AMBIQ_ED25519_DEVICE_DEFINE(inst)                                                          \
	static AMBIQ_ED25519_NOCACHE struct ambiq_ed25519_dma ambiq_ed25519_dma_##inst;            \
	static struct ambiq_ed25519_data ambiq_ed25519_data_##inst;                                \
	static const struct ambiq_ed25519_config ambiq_ed25519_cfg_##inst = {                      \
		.dma = &ambiq_ed25519_dma_##inst,                                                  \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_ed25519_pm_action);                                   \
	DEVICE_DT_INST_DEFINE(inst, ambiq_ed25519_init, PM_DEVICE_DT_INST_GET(inst),               \
			      &ambiq_ed25519_data_##inst, &ambiq_ed25519_cfg_##inst, POST_KERNEL,  \
			      CONFIG_CRYPTO_INIT_PRIORITY, &ambiq_ed25519_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_ED25519_DEVICE_DEFINE)
