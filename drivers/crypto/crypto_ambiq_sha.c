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

#include <string.h>

#include <soc.h>
#include <cc312_arbiter.h>

LOG_MODULE_REGISTER(crypto_ambiq_sha, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_sha

#define AMBIQ_SHA_CAPS (CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

/*
 * Largest buffer handed to a single am_hal_cc312_sha_update() call. The HAL
 * forwards whole blocks straight to the DMA, whose DLLI length field tops out
 * below AM_HAL_CC312_DLLI_MAX_BUFF_SIZE. Block-aligned so no partial block is
 * buffered between chunks.
 */
#define AMBIQ_SHA_MAX_UPDATE                                                                       \
	ROUND_DOWN(AM_HAL_CC312_DLLI_MAX_BUFF_SIZE - AM_HAL_SHA_BLOCK_SIZE, AM_HAL_SHA_BLOCK_SIZE)

struct ambiq_sha_session {
	am_hal_cc312_sha_context_t hal;
	enum hash_algo algo;
	bool in_use;
};

struct ambiq_sha_data {
	struct ambiq_sha_session sessions[CONFIG_CRYPTO_AMBIQ_SHA_MAX_SESSION];
};

static int ambiq_sha_hal_status_to_errno(uint32_t status)
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
	case AM_HAL_STATUS_OUT_OF_RANGE:
		return -ERANGE;
	default:
		return -EIO;
	}
}

static am_hal_cc312_sha_mode_e ambiq_sha_hal_mode(enum hash_algo algo)
{
	return (algo == CRYPTO_HASH_ALGO_SHA1) ? AM_HAL_CC312_SHA1 : AM_HAL_CC312_SHA256;
}

static size_t ambiq_sha_digest_size(enum hash_algo algo)
{
	if (algo == CRYPTO_HASH_ALGO_SHA1) {
		return AM_HAL_SHA1_DIGEST_SIZE;
	}
	return AM_HAL_SHA256_DIGEST_SIZE;
}

static bool ambiq_sha_algo_supported(enum hash_algo algo)
{
	return (algo == CRYPTO_HASH_ALGO_SHA1) || (algo == CRYPTO_HASH_ALGO_SHA256);
}

static struct ambiq_sha_session *ambiq_sha_session_alloc(struct ambiq_sha_data *data,
							 enum hash_algo algo)
{
	struct ambiq_sha_session *s = NULL;

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

static uint32_t ambiq_sha_feed(struct ambiq_sha_session *s, const uint8_t *in, size_t len)
{
	uint32_t status = AM_HAL_STATUS_SUCCESS;

	while (len > 0U) {
		size_t chunk = MIN(len, (size_t)AMBIQ_SHA_MAX_UPDATE);

		status = am_hal_cc312_sha_update(&s->hal, in, (uint32_t)chunk);
		if (status != AM_HAL_STATUS_SUCCESS) {
			break;
		}

		in += chunk;
		len -= chunk;
	}

	return status;
}

static int ambiq_sha_handler(struct hash_ctx *hctx, struct hash_pkt *pkt, bool finish)
{
	struct ambiq_sha_session *s;
	struct ambiq_sha_data *data;
	uint32_t status;
	int ret = 0;

	if (hctx == NULL || pkt == NULL || hctx->device == NULL) {
		return -EINVAL;
	}

	s = hctx->drv_sessn_state;
	data = hctx->device->data;
	if (s == NULL || data == NULL) {
		return -EINVAL;
	}

	if (pkt->in_len > 0U && pkt->in_buf == NULL) {
		return -EINVAL;
	}

	if (finish && pkt->out_buf == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);

	if (!hctx->started) {
		status = am_hal_cc312_sha_context_init(&s->hal, ambiq_sha_hal_mode(s->algo));
		if (status != AM_HAL_STATUS_SUCCESS) {
			ret = ambiq_sha_hal_status_to_errno(status);
			goto out;
		}
		hctx->started = true;
	}

	if (pkt->in_len > 0U) {
		status = ambiq_sha_feed(s, pkt->in_buf, pkt->in_len);
		if (status != AM_HAL_STATUS_SUCCESS) {
			ret = ambiq_sha_hal_status_to_errno(status);
			goto out;
		}
	}

	if (finish) {
		status = am_hal_cc312_sha_finish(&s->hal, pkt->out_buf);
		if (status != AM_HAL_STATUS_SUCCESS) {
			ret = ambiq_sha_hal_status_to_errno(status);
			goto out;
		}
		hctx->started = false;
	}

out:
	if (ret != 0) {
		(void)am_hal_cc312_sha_free(&s->hal);
		hctx->started = false;
		if (finish && pkt->out_buf != NULL) {
			memset(pkt->out_buf, 0, ambiq_sha_digest_size(s->algo));
		}
	}

	k_mutex_unlock(ambiq_cc312_arbiter_lock());
	return ret;
}

static int ambiq_sha_begin_session(const struct device *dev, struct hash_ctx *hctx,
				   enum hash_algo algo)
{
	struct ambiq_sha_session *s;
	int ret;

	if (dev == NULL || hctx == NULL || dev->data == NULL) {
		return -EINVAL;
	}

	if (!ambiq_sha_algo_supported(algo)) {
		return -ENOTSUP;
	}

	if (hctx->flags & ~AMBIQ_SHA_CAPS) {
		return -ENOTSUP;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	s = ambiq_sha_session_alloc(dev->data, algo);
	if (s == NULL) {
		(void)pm_device_runtime_put(dev);
		return -ENOSPC;
	}

	hctx->device = dev;
	hctx->drv_sessn_state = s;
	hctx->hash_hndlr = ambiq_sha_handler;
	hctx->started = false;

	return 0;
}

static int ambiq_sha_free_session(const struct device *dev, struct hash_ctx *hctx)
{
	struct ambiq_sha_session *s;
	struct ambiq_sha_data *data;

	if (dev == NULL || hctx == NULL || dev->data == NULL) {
		return -EINVAL;
	}

	s = hctx->drv_sessn_state;
	data = dev->data;

	if (s != NULL) {
		k_mutex_lock(ambiq_cc312_arbiter_lock(), K_FOREVER);
		(void)am_hal_cc312_sha_free(&s->hal);
		s->in_use = false;
		k_mutex_unlock(ambiq_cc312_arbiter_lock());
	}

	hctx->drv_sessn_state = NULL;
	hctx->hash_hndlr = NULL;
	hctx->started = false;

	(void)pm_device_runtime_put(dev);
	return 0;
}

static int ambiq_sha_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return AMBIQ_SHA_CAPS;
}

static int ambiq_sha_pm_action(const struct device *dev, enum pm_device_action action)
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

static int ambiq_sha_init(const struct device *dev)
{
	return pm_device_runtime_enable(dev);
}

static DEVICE_API(crypto, ambiq_sha_crypto_api) = {
	.query_hw_caps = ambiq_sha_query_hw_caps,
	.hash_begin_session = ambiq_sha_begin_session,
	.hash_free_session = ambiq_sha_free_session,
};

#define AMBIQ_SHA_DEVICE_DEFINE(inst)                                                              \
	static struct ambiq_sha_data ambiq_sha_data_##inst;                                        \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_sha_pm_action);                                       \
	DEVICE_DT_INST_DEFINE(inst, ambiq_sha_init, PM_DEVICE_DT_INST_GET(inst),                   \
			      &ambiq_sha_data_##inst, NULL, POST_KERNEL,                           \
			      CONFIG_CRYPTO_INIT_PRIORITY, &ambiq_sha_crypto_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_SHA_DEVICE_DEFINE)
