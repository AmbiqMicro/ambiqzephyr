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
#include <zephyr/sys/util.h>

#include <soc.h>
#include <cc312_arbiter.h>

LOG_MODULE_REGISTER(crypto_ambiq_hmac, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_hmac

#define AMBIQ_HMAC_CAPS (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

/*
 * Largest buffer handed to a single am_hal_cc312_hmac_update() call. HMAC is
 * implemented on the HASH engine; CC312 DLLI size is 16 bits (LLI_SIZE_BIT_SIZE
 * in the Arm cryptocell-312-runtime), so one transfer must stay strictly below
 * AM_HAL_CC312_DLLI_MAX_BUFF_SIZE. Match the SHA driver's block-aligned chunk.
 */
#define AMBIQ_HMAC_MAX_CHUNK                                                                   \
	ROUND_DOWN(AM_HAL_CC312_DLLI_MAX_BUFF_SIZE - AM_HAL_SHA_BLOCK_SIZE, AM_HAL_SHA_BLOCK_SIZE)

struct ambiq_hmac_session {
	am_hal_cc312_hmac_context_t hal;
	enum mac_algo algo;
	bool in_use;
};

struct ambiq_hmac_data {
	struct ambiq_hmac_session sessions[CONFIG_CRYPTO_AMBIQ_HMAC_MAX_SESSION];
};

static int ambiq_hmac_hal_status_to_errno(uint32_t status)
{
	switch (status) {
	case AM_HAL_STATUS_SUCCESS:
		return 0;
	case AM_HAL_STATUS_INVALID_ARG:
		return -EINVAL;
	case AM_HAL_STATUS_TIMEOUT:
		return -ETIMEDOUT;
	case AM_HAL_STATUS_IN_USE:
		return -EBUSY;
	default:
		return -EIO;
	}
}

static am_hal_cc312_sha_mode_e ambiq_hmac_hal_mode(enum mac_algo algo)
{
	switch (algo) {
	case CRYPTO_MAC_ALGO_HMAC_SHA1:
		return AM_HAL_CC312_SHA1;
	case CRYPTO_MAC_ALGO_HMAC_SHA224:
		return AM_HAL_CC312_SHA224;
	default:
		return AM_HAL_CC312_SHA256;
	}
}

static size_t ambiq_hmac_mac_size(enum mac_algo algo)
{
	switch (algo) {
	case CRYPTO_MAC_ALGO_HMAC_SHA1:
		return AM_HAL_SHA1_DIGEST_SIZE;
	case CRYPTO_MAC_ALGO_HMAC_SHA224:
		return AM_HAL_SHA224_DIGEST_SIZE;
	default:
		return AM_HAL_SHA256_DIGEST_SIZE;
	}
}

static bool ambiq_hmac_algo_supported(enum mac_algo algo)
{
	return (algo == CRYPTO_MAC_ALGO_HMAC_SHA1) || (algo == CRYPTO_MAC_ALGO_HMAC_SHA224) ||
	       (algo == CRYPTO_MAC_ALGO_HMAC_SHA256);
}

static struct ambiq_hmac_session *ambiq_hmac_session_alloc(struct ambiq_hmac_data *data,
							   enum mac_algo algo)
{
	struct ambiq_hmac_session *s = NULL;

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);

	for (int i = 0; i < ARRAY_SIZE(data->sessions); i++) {
		if (!data->sessions[i].in_use) {
			s = &data->sessions[i];
			s->in_use = true;
			s->algo = algo;
			break;
		}
	}

	k_mutex_unlock(ambiq_cc312_arbiter_lock());
	return s;
}

static uint32_t ambiq_hmac_feed(struct ambiq_hmac_session *s, const uint8_t *in, size_t len)
{
	uint32_t status = AM_HAL_STATUS_SUCCESS;

	while (len > 0U) {
		size_t chunk = MIN(len, (size_t)AMBIQ_HMAC_MAX_CHUNK);

		status = am_hal_cc312_hmac_update(&s->hal, in, (uint32_t)chunk);
		if (status != AM_HAL_STATUS_SUCCESS) {
			break;
		}

		in += chunk;
		len -= chunk;
	}

	return status;
}

static int ambiq_hmac_handler(struct mac_ctx *mctx, struct mac_pkt *pkt, bool finish)
{
	struct ambiq_hmac_session *s = mctx->drv_sessn_state;
	uint32_t status;
	int ret = 0;

	if (s == NULL) {
		return -EINVAL;
	}

	if (pkt->in_len > 0U && pkt->in_buf == NULL) {
		return -EINVAL;
	}

	if (finish && pkt->out_buf == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);

	if (!mctx->started) {
		status = am_hal_cc312_hmac_start(&s->hal, ambiq_hmac_hal_mode(s->algo),
						 mctx->key.bit_stream, mctx->keylen);
		if (status != AM_HAL_STATUS_SUCCESS) {
			ret = ambiq_hmac_hal_status_to_errno(status);
			goto out;
		}
		mctx->started = true;
	}

	if (pkt->in_len > 0U) {
		status = ambiq_hmac_feed(s, pkt->in_buf, pkt->in_len);
		if (status != AM_HAL_STATUS_SUCCESS) {
			ret = ambiq_hmac_hal_status_to_errno(status);
			goto out;
		}
	}

	if (finish) {
		status = am_hal_cc312_hmac_finish(&s->hal, pkt->out_buf);
		if (status != AM_HAL_STATUS_SUCCESS) {
			ret = ambiq_hmac_hal_status_to_errno(status);
			goto out;
		}
		mctx->started = false;
	}

out:
	if (ret != 0) {
		(void)am_hal_cc312_hmac_free(&s->hal);
		mctx->started = false;
		if (finish && pkt->out_buf != NULL) {
			am_hal_cc312_context_clear(pkt->out_buf, ambiq_hmac_mac_size(s->algo));
		}
	}

	k_mutex_unlock(ambiq_cc312_arbiter_lock());
	return ret;
}

static int ambiq_hmac_begin_session(const struct device *dev, struct mac_ctx *ctx,
				    enum mac_algo algo)
{
	struct ambiq_hmac_session *s;
	int ret;

	if (dev == NULL || ctx == NULL || dev->data == NULL) {
		return -EINVAL;
	}

	if (!ambiq_hmac_algo_supported(algo)) {
		return -ENOTSUP;
	}

	if (ctx->flags & ~AMBIQ_HMAC_CAPS) {
		return -ENOTSUP;
	}

	if (ctx->key.bit_stream == NULL || ctx->keylen == 0U) {
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	s = ambiq_hmac_session_alloc(dev->data, algo);
	if (s == NULL) {
		(void)pm_device_runtime_put(dev);
		return -ENOSPC;
	}

	ctx->device = dev;
	ctx->drv_sessn_state = s;
	ctx->mac_hndlr = ambiq_hmac_handler;
	ctx->started = false;

	return 0;
}

static int ambiq_hmac_free_session(const struct device *dev, struct mac_ctx *ctx)
{
	struct ambiq_hmac_session *s;

	if (dev == NULL || ctx == NULL) {
		return -EINVAL;
	}

	s = ctx->drv_sessn_state;
	if (s != NULL) {
		k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
		(void)am_hal_cc312_hmac_free(&s->hal);
		s->in_use = false;
		k_mutex_unlock(ambiq_cc312_arbiter_lock());
	}

	ctx->drv_sessn_state = NULL;
	ctx->mac_hndlr = NULL;
	ctx->started = false;

	(void)pm_device_runtime_put(dev);
	return 0;
}

static int ambiq_hmac_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return AMBIQ_HMAC_CAPS;
}

static int ambiq_hmac_pm_action(const struct device *dev, enum pm_device_action action)
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

static int ambiq_hmac_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return pm_device_runtime_enable(dev);
}

static DEVICE_API(crypto, ambiq_hmac_crypto_api) = {
	.query_hw_caps = ambiq_hmac_query_hw_caps,
	.mac_begin_session = ambiq_hmac_begin_session,
	.mac_free_session = ambiq_hmac_free_session,
};

#define AMBIQ_HMAC_DEVICE_DEFINE(inst)                                                             \
	static struct ambiq_hmac_data ambiq_hmac_data_##inst;                                      \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_hmac_pm_action);                                      \
	DEVICE_DT_INST_DEFINE(inst, ambiq_hmac_init, PM_DEVICE_DT_INST_GET(inst),                \
			      &ambiq_hmac_data_##inst, NULL, POST_KERNEL,                          \
			      CONFIG_CRYPTO_INIT_PRIORITY, &ambiq_hmac_crypto_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_HMAC_DEVICE_DEFINE)
