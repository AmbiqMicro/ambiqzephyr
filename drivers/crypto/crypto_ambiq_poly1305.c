/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/crypto/crypto_ambiq_poly1305.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/util.h>

#include <soc.h>
#include <cc312_arbiter.h>

LOG_MODULE_REGISTER(crypto_ambiq_poly1305, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_poly1305

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define AMBIQ_POLY_NOCACHE __nocache
#else
#define AMBIQ_POLY_NOCACHE
#endif

#define AMBIQ_POLY_KEY_SIZE 32U
#define AMBIQ_POLY_MAC_SIZE 16U
/* Scratch for typical KATs; longer messages fall back to the caller's buffer. */
#define AMBIQ_POLY_MSG_SCRATCH 64U

struct ambiq_poly1305_dma {
	uint8_t key[AMBIQ_POLY_KEY_SIZE] __aligned(32);
	uint8_t mac[AMBIQ_POLY_MAC_SIZE] __aligned(32);
	uint8_t msg[AMBIQ_POLY_MSG_SCRATCH] __aligned(32);
};

struct ambiq_poly1305_data {
	struct ambiq_poly1305_dma *dma;
};

struct ambiq_poly1305_config {
	struct ambiq_poly1305_dma *dma;
};

static int ambiq_poly1305_hal_status_to_errno(uint32_t status)
{
	if (status == AM_HAL_STATUS_SUCCESS) {
		return 0;
	}
	if (status == AM_HAL_STATUS_INVALID_ARG) {
		return -EINVAL;
	}
	return -EIO;
}

static int ambiq_poly1305_mac(const struct device *dev, const uint8_t *key, const uint8_t *input,
			      uint32_t length, uint8_t *mac_out)
{
	struct ambiq_poly1305_data *data = dev->data;
	const uint8_t *msg_ptr;
	uint32_t status;
	int ret;

	if (key == NULL || mac_out == NULL || (length != 0U && input == NULL) || data == NULL ||
	    data->dma == NULL) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);

	(void)am_hal_cc312_memcpy(data->dma->key, key, AMBIQ_POLY_KEY_SIZE);

	if (length == 0U) {
		msg_ptr = NULL;
	} else if (length <= AMBIQ_POLY_MSG_SCRATCH) {
		(void)am_hal_cc312_memcpy(data->dma->msg, input, length);
		msg_ptr = data->dma->msg;
	} else {
		msg_ptr = input;
	}

	status = am_hal_cc312_poly1305(data->dma->key, msg_ptr, length, data->dma->mac);
	if (status == AM_HAL_STATUS_SUCCESS) {
		(void)am_hal_cc312_memcpy(mac_out, data->dma->mac, AMBIQ_POLY_MAC_SIZE);
	} else {
		am_hal_cc312_context_clear(mac_out, AMBIQ_POLY_MAC_SIZE);
	}
	ret = ambiq_poly1305_hal_status_to_errno(status);

	am_hal_cc312_context_clear(data->dma->key, sizeof(data->dma->key));
	am_hal_cc312_context_clear(data->dma->mac, sizeof(data->dma->mac));
	if (length != 0U && length <= AMBIQ_POLY_MSG_SCRATCH) {
		am_hal_cc312_context_clear(data->dma->msg, length);
	}

	k_mutex_unlock(ambiq_cc312_arbiter_lock());
	(void)pm_device_runtime_put(dev);

	return ret;
}

static int ambiq_poly1305_mac_aead(const struct device *dev, const uint8_t *key, const uint8_t *aad,
				   uint32_t aad_len, const uint8_t *input, uint32_t length,
				   uint8_t *mac_out)
{
	struct ambiq_poly1305_data *data = dev->data;
	uint32_t status;
	int ret;

	if (key == NULL || mac_out == NULL || data == NULL || data->dma == NULL) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);

	(void)am_hal_cc312_memcpy(data->dma->key, key, AMBIQ_POLY_KEY_SIZE);

	status = am_hal_cc312_poly1305_aead(data->dma->key, aad, aad_len, input, length,
					    data->dma->mac);
	if (status == AM_HAL_STATUS_SUCCESS) {
		(void)am_hal_cc312_memcpy(mac_out, data->dma->mac, AMBIQ_POLY_MAC_SIZE);
	} else {
		am_hal_cc312_context_clear(mac_out, AMBIQ_POLY_MAC_SIZE);
	}
	ret = ambiq_poly1305_hal_status_to_errno(status);

	am_hal_cc312_context_clear(data->dma->key, sizeof(data->dma->key));
	am_hal_cc312_context_clear(data->dma->mac, sizeof(data->dma->mac));

	k_mutex_unlock(ambiq_cc312_arbiter_lock());
	(void)pm_device_runtime_put(dev);

	return ret;
}

static int ambiq_poly1305_pm_action(const struct device *dev, enum pm_device_action action)
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

static int ambiq_poly1305_init(const struct device *dev)
{
	const struct ambiq_poly1305_config *cfg = dev->config;
	struct ambiq_poly1305_data *data = dev->data;

	data->dma = cfg->dma;
	return pm_device_runtime_enable(dev);
}

static const struct crypto_ambiq_poly1305_driver_api ambiq_poly1305_api = {
	.mac = ambiq_poly1305_mac,
	.mac_aead = ambiq_poly1305_mac_aead,
};

#define AMBIQ_POLY1305_DEVICE_DEFINE(inst)                                                         \
	static AMBIQ_POLY_NOCACHE struct ambiq_poly1305_dma ambiq_poly1305_dma_##inst;             \
	static struct ambiq_poly1305_data ambiq_poly1305_data_##inst;                              \
	static const struct ambiq_poly1305_config ambiq_poly1305_cfg_##inst = {                    \
		.dma = &ambiq_poly1305_dma_##inst,                                                 \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_poly1305_pm_action);                                  \
	DEVICE_DT_INST_DEFINE(inst, ambiq_poly1305_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &ambiq_poly1305_data_##inst, &ambiq_poly1305_cfg_##inst, POST_KERNEL,\
			      CONFIG_CRYPTO_INIT_PRIORITY, &ambiq_poly1305_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_POLY1305_DEVICE_DEFINE)
