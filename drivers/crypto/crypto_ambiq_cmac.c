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
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/util.h>

#include <soc.h>
#include <cc312_arbiter.h>

LOG_MODULE_REGISTER(crypto_ambiq_cmac, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_cmac

#define AMBIQ_CMAC_BLOCK_SIZE   16U
#define AMBIQ_CMAC_CAPS         (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define AMBIQ_CMAC_NOCACHE __nocache
#else
#define AMBIQ_CMAC_NOCACHE
#endif

struct ambiq_cmac_session {
	am_hal_cc312_aes_context_t op_ctx;
	bool in_use;
};

struct ambiq_cmac_dma_data {
	am_hal_cc312_aes_context_t op_ctx;
	uint8_t key_local[32] __aligned(AM_HAL_CC312_DMA_ALIGNMENT);
	uint8_t mac_local[AMBIQ_CMAC_BLOCK_SIZE] __aligned(AM_HAL_CC312_DMA_ALIGNMENT);
};

struct ambiq_cmac_data {
	struct ambiq_cmac_session sessions[CONFIG_CRYPTO_AMBIQ_CMAC_MAX_SESSION];
	struct ambiq_cmac_dma_data *dma;
};

struct ambiq_cmac_config {
	struct ambiq_cmac_dma_data *dma;
};

static int ambiq_cmac_hal_status_to_errno(uint32_t status)
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

static uint32_t ambiq_cmac_prepare_key_ctx(am_hal_cc312_aes_context_t *op_ctx, const uint8_t *key,
					   uint32_t key_bits)
{
	uint32_t status;

	status = am_hal_cc312_aes_context_init(op_ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	status = am_hal_cc312_aes_setkey_enc(op_ctx, key, key_bits);
	if (status != AM_HAL_STATUS_SUCCESS) {
		(void)am_hal_cc312_aes_free(op_ctx);
	}

	return status;
}

static int ambiq_cmac_handler(struct mac_ctx *mctx, struct mac_pkt *pkt, bool finish)
{
	struct ambiq_cmac_session *s = mctx->drv_sessn_state;
	const struct device *dev = mctx->device;
	struct ambiq_cmac_data *data = dev->data;
	uint32_t status;
	int ret = 0;

	if (s == NULL || data == NULL) {
		return -EINVAL;
	}

	if (!finish) {
		return -ENOTSUP;
	}

	if (pkt->out_buf == NULL) {
		return -EINVAL;
	}

	if (pkt->in_len > 0U && pkt->in_buf == NULL) {
		return -EINVAL;
	}

	if (pkt->in_len >= AM_HAL_CC312_DLLI_MAX_BUFF_SIZE) {
		return -EINVAL;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);

	(void)am_hal_cc312_memcpy(data->dma->key_local, mctx->key.bit_stream,
				  (uint32_t)mctx->keylen);

	status = ambiq_cmac_prepare_key_ctx(&data->dma->op_ctx, data->dma->key_local,
					    (uint32_t)mctx->keylen * 8U);
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_cmac_hal_status_to_errno(status);
		goto out;
	}

	status = am_hal_aes_cmac(&data->dma->op_ctx, (uint32_t)pkt->in_len, pkt->in_buf,
				 data->dma->mac_local);
	(void)am_hal_cc312_aes_free(&data->dma->op_ctx);

	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_cmac_hal_status_to_errno(status);
		goto out;
	}

	(void)am_hal_cc312_memcpy(pkt->out_buf, data->dma->mac_local, AMBIQ_CMAC_BLOCK_SIZE);
	ret = 0;

out:
	if (ret != 0) {
		am_hal_cc312_context_clear(pkt->out_buf, AMBIQ_CMAC_BLOCK_SIZE);
	}
	am_hal_cc312_context_clear(data->dma->key_local, sizeof(data->dma->key_local));
	am_hal_cc312_context_clear(data->dma->mac_local, sizeof(data->dma->mac_local));

	k_mutex_unlock(ambiq_cc312_arbiter_lock());
	return ret;
}

static int ambiq_cmac_begin_session(const struct device *dev, struct mac_ctx *ctx,
				    enum mac_algo algo)
{
	struct ambiq_cmac_data *data = dev->data;
	struct ambiq_cmac_session *s = NULL;
	int ret;

	if (dev == NULL || ctx == NULL || data == NULL) {
		return -EINVAL;
	}

	if (algo != CRYPTO_MAC_ALGO_AES_CMAC) {
		return -ENOTSUP;
	}

	if (ctx->flags & ~AMBIQ_CMAC_CAPS) {
		return -ENOTSUP;
	}

	if (ctx->key.bit_stream == NULL) {
		return -EINVAL;
	}

	if (!(ctx->keylen == 16 || ctx->keylen == 24 || ctx->keylen == 32)) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
	for (int i = 0; i < ARRAY_SIZE(data->sessions); i++) {
		if (!data->sessions[i].in_use) {
			s = &data->sessions[i];
			s->in_use = true;
			break;
		}
	}
	k_mutex_unlock(ambiq_cc312_arbiter_lock());

	if (s == NULL) {
		(void)pm_device_runtime_put(dev);
		return -ENOSPC;
	}

	ctx->device = dev;
	ctx->drv_sessn_state = s;
	ctx->mac_hndlr = ambiq_cmac_handler;
	ctx->started = false;

	return 0;
}

static int ambiq_cmac_free_session(const struct device *dev, struct mac_ctx *ctx)
{
	struct ambiq_cmac_session *s;

	if (dev == NULL || ctx == NULL) {
		return -EINVAL;
	}

	s = ctx->drv_sessn_state;
	if (s != NULL) {
		k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
		s->in_use = false;
		k_mutex_unlock(ambiq_cc312_arbiter_lock());
	}

	ctx->drv_sessn_state = NULL;
	ctx->mac_hndlr = NULL;
	ctx->started = false;

	(void)pm_device_runtime_put(dev);
	return 0;
}

static int ambiq_cmac_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return AMBIQ_CMAC_CAPS;
}

static int ambiq_cmac_pm_action(const struct device *dev, enum pm_device_action action)
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

static int ambiq_cmac_init(const struct device *dev)
{
	const struct ambiq_cmac_config *cfg = dev->config;
	struct ambiq_cmac_data *data = dev->data;

	data->dma = cfg->dma;
	return pm_device_runtime_enable(dev);
}

static DEVICE_API(crypto, ambiq_cmac_crypto_api) = {
	.query_hw_caps = ambiq_cmac_query_hw_caps,
	.mac_begin_session = ambiq_cmac_begin_session,
	.mac_free_session = ambiq_cmac_free_session,
};

#define AMBIQ_CMAC_DEVICE_DEFINE(inst)                                                             \
	static AMBIQ_CMAC_NOCACHE struct ambiq_cmac_dma_data ambiq_cmac_dma_##inst;                \
	static struct ambiq_cmac_data ambiq_cmac_data_##inst;                                      \
	static const struct ambiq_cmac_config ambiq_cmac_cfg_##inst = {                            \
		.dma = &ambiq_cmac_dma_##inst,                                                     \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_cmac_pm_action);                                      \
	DEVICE_DT_INST_DEFINE(inst, ambiq_cmac_init, PM_DEVICE_DT_INST_GET(inst),                  \
			      &ambiq_cmac_data_##inst, &ambiq_cmac_cfg_##inst, POST_KERNEL,        \
			      CONFIG_CRYPTO_INIT_PRIORITY, &ambiq_cmac_crypto_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_CMAC_DEVICE_DEFINE)
