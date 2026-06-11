/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/cache.h>

#include <string.h>
#include <cmsis_gcc.h>

#include <soc.h>

LOG_MODULE_REGISTER(crypto_ambiq_chacha, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_chacha

#define AMBIQ_CHACHA_DMA_ALIGNMENT  AM_HAL_CC312_DMA_ALIGNMENT
#define AMBIQ_CHACHA_IRQ_WAIT_TIMEOUT_MS 2000
#define AMBIQ_CHACHA_CAPS \
	(CAP_RAW_KEY | CAP_INPLACE_OPS | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_NO_IV_PREFIX)

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define AMBIQ_CHACHA_NOCACHE __nocache
#else
#define AMBIQ_CHACHA_NOCACHE
#endif

/* Flush scratch buffer — one DMA alignment worth of space. */
#define AMBIQ_CHACHA_FLUSH_SIZE AMBIQ_CHACHA_DMA_ALIGNMENT

struct ambiq_chacha_dma_data {
	am_hal_cc312_chacha_context_t ctx;
	uint8_t dlli_flush_scratch[AMBIQ_CHACHA_FLUSH_SIZE]
		__aligned(AMBIQ_CHACHA_DMA_ALIGNMENT);
};

struct ambiq_chacha_data {
	struct k_mutex lock;
	struct k_sem irq_sem;
	uint32_t irq_num;
	atomic_t irq_seen;
	atomic_t irq_wait_mask;
	struct ambiq_chacha_dma_data *dma;
};

struct ambiq_chacha_config {
	uint32_t irq_num;
	void (*irq_config_func)(void);
	struct ambiq_chacha_dma_data *dma;
};

/* ---------- IRQ helpers (identical pattern to AES driver) ---------- */

static void ambiq_chacha_prepare_irq_wait(struct ambiq_chacha_data *data, uint32_t wait_mask)
{
	irq_disable(data->irq_num);
	k_sem_reset(&data->irq_sem);
	(void)atomic_set(&data->irq_seen, 0U);
	(void)atomic_set(&data->irq_wait_mask, wait_mask);
}

static void ambiq_chacha_finish_irq_wait(struct ambiq_chacha_data *data)
{
	irq_disable(data->irq_num);
	(void)atomic_set(&data->irq_wait_mask, 0U);
}

static void ambiq_cc312_chacha_isr(const void *arg)
{
	const struct device *dev = arg;
	struct ambiq_chacha_data *data;
	uint32_t irr_val;
	uint32_t clear_mask;
	uint32_t irq_seen;
	uint32_t irq_wait_mask;

	if (dev == NULL) {
		return;
	}

	data = dev->data;
	if (data == NULL) {
		return;
	}

	irr_val = CRYPTO->HOSTRGFIRR;
	if (irr_val == 0U) {
		return;
	}

	clear_mask = irr_val;
	if ((irr_val & CRYPTO_HOSTRGFIRR_AHBERRINT_Msk) != 0U) {
		clear_mask |= CRYPTO_HOSTRGFICR_AXIERRCLEAR_Msk;
	}
	CRYPTO->HOSTRGFICR = clear_mask;

	irq_seen = (uint32_t)atomic_or(&data->irq_seen, irr_val) | irr_val;
	irq_wait_mask = (uint32_t)atomic_get(&data->irq_wait_mask);

	if (((irq_seen & irq_wait_mask) != 0U) ||
	    ((irr_val & CRYPTO_HOSTRGFIRR_AHBERRINT_Msk) != 0U)) {
		k_sem_give(&data->irq_sem);
	}
}

/* ---------- Cache helpers ---------- */

static void ambiq_chacha_cache_clean_invalidate(const void *addr, size_t len)
{
	if (addr == NULL || len == 0U) {
		return;
	}
	if (!buf_in_nocache(POINTER_TO_UINT(addr), len)) {
		(void)sys_cache_data_flush_and_invd_range((void *)addr, len);
	}
}

static void ambiq_chacha_cache_invalidate(const void *addr, size_t len)
{
	if (addr == NULL || len == 0U) {
		return;
	}
	if (!buf_in_nocache(POINTER_TO_UINT(addr), len)) {
		(void)sys_cache_data_invd_range((void *)addr, len);
	}
}

/* ---------- Secure zero ---------- */

__noinline static void ambiq_chacha_secure_zero(void *buf, size_t len)
{
	volatile uint8_t *p;

	if (buf == NULL || len == 0U) {
		return;
	}
	p = (volatile uint8_t *)buf;
	while (len-- != 0U) {
		*p++ = 0U;
	}
	__DMB();
}

#define AMBIQ_CHACHA_BLOCK_BYTES 64U
#define AMBIQ_CHACHA_MAX_CHUNK                                                                     \
	((AM_HAL_CC312_DLLI_MAX_BUFF_SIZE - 1U) & ~(AMBIQ_CHACHA_BLOCK_BYTES - 1U))

/* ---------- DLLI dummy-read flush (same as AES driver) ---------- */

static uint32_t ambiq_chacha_flush_dummy_dlli(struct ambiq_chacha_data *data,
					      uint32_t src_addr, uint32_t length)
{
	uint32_t irr_mask = CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk;
	uint32_t flush_len;
	uint32_t src_tail_addr;
	uint32_t status = AM_HAL_STATUS_SUCCESS;
	int ret;

	if (data == NULL || data->dma == NULL || src_addr == 0U || length == 0U) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	ambiq_chacha_secure_zero(data->dma->dlli_flush_scratch,
				 sizeof(data->dma->dlli_flush_scratch));

	flush_len = MIN(length, (uint32_t)AMBIQ_CHACHA_FLUSH_SIZE);
	src_tail_addr = src_addr + (length - flush_len);

	ambiq_chacha_prepare_irq_wait(data, irr_mask);
	am_hal_cc312_set_dma_destination(AM_HAL_CC312_DMA_DLLI_ADDR,
					 POINTER_TO_UINT(data->dma->dlli_flush_scratch),
					 flush_len);
	am_hal_cc312_clear_interrupt(0xFFFFFFFFU);
	irq_enable(data->irq_num);
	am_hal_cc312_set_dma_source(AM_HAL_CC312_DMA_DLLI_ADDR, src_tail_addr, flush_len);

	ret = k_sem_take(&data->irq_sem, K_MSEC(AMBIQ_CHACHA_IRQ_WAIT_TIMEOUT_MS));
	if (ret != 0) {
		status = AM_HAL_STATUS_TIMEOUT;
		goto flush_exit;
	}

	if (((uint32_t)atomic_get(&data->irq_seen) & CRYPTO_HOSTRGFIRR_AHBERRINT_Msk) != 0U) {
		status = AM_HAL_STATUS_HW_ERR;
		goto flush_exit;
	}

	if (((uint32_t)atomic_get(&data->irq_seen) & irr_mask) == 0U) {
		status = AM_HAL_STATUS_TIMEOUT;
		goto flush_exit;
	}

flush_exit:
	ambiq_chacha_secure_zero(data->dma->dlli_flush_scratch,
				 sizeof(data->dma->dlli_flush_scratch));
	return status;
}

/* ---------- HAL status → errno ---------- */

static int ambiq_chacha_hal_status_to_errno(uint32_t status)
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

/* ---------- Output buffer resolution ---------- */

static int ambiq_chacha_get_output_buffer(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
					  uint8_t **out_buf)
{
	if (pkt->in_buf == NULL || pkt->in_len < 0 || pkt->out_buf_max < 0) {
		return -EINVAL;
	}

	if (pkt->out_buf == NULL) {
		if ((ctx->flags & CAP_INPLACE_OPS) == 0U) {
			return -EINVAL;
		}
		*out_buf = pkt->in_buf;
		return 0;
	}

	if ((ctx->flags & CAP_SEPARATE_IO_BUFS) == 0U) {
		return -EINVAL;
	}

	if ((size_t)pkt->out_buf_max < (size_t)pkt->in_len) {
		return -EINVAL;
	}

	*out_buf = pkt->out_buf;
	return 0;
}

/* ---------- DMA transfer contract ---------- */

static uint32_t ambiq_chacha_check_buffer(am_hal_cc312_dma_addr_type_e addr_type,
					  const void *buf, uint32_t length)
{
	if (addr_type != AM_HAL_CC312_DMA_DLLI_ADDR) {
		return AM_HAL_STATUS_SUCCESS;
	}

	if (length >= AM_HAL_CC312_DLLI_MAX_BUFF_SIZE) {
		return AM_HAL_STATUS_OUT_OF_RANGE;
	}

	if ((POINTER_TO_UINT(buf) & (AMBIQ_CHACHA_DMA_ALIGNMENT - 1U)) != 0U) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	return AM_HAL_STATUS_SUCCESS;
}

/* ---------- Core ChaCha20 process (IRQ-driven DMA) ---------- */

static uint32_t ambiq_chacha_process(struct ambiq_chacha_data *data,
				     am_hal_cc312_chacha_context_t *ctx,
				     const uint8_t *input, uint8_t *output,
				     uint32_t length)
{
	uint32_t irr_mask = CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk;
	uint32_t status;
	int ret;

	if (data == NULL || ctx == NULL || input == NULL || output == NULL || length == 0U) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	status = ambiq_chacha_check_buffer(ctx->inputDataAddrType, input, length);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	status = ambiq_chacha_check_buffer(ctx->outputDataAddrType, output, length);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	ambiq_chacha_prepare_irq_wait(data, irr_mask);

	am_hal_cc312_chacha_enable_clocks();

	status = am_hal_cc312_chacha_init(ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto process_exit;
	}

	status = am_hal_cc312_chacha_load_state(ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto process_exit;
	}

	status = am_hal_cc312_chacha_load_key(ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto process_exit;
	}

	am_hal_cc312_chacha_set_control(ctx);

	am_hal_cc312_set_buffer_security(0U, 0U);
	ambiq_chacha_cache_clean_invalidate(input, (size_t)length);
	ambiq_chacha_cache_clean_invalidate(output, (size_t)length);

	am_hal_cc312_set_dma_destination((am_hal_cc312_dma_addr_type_e)ctx->outputDataAddrType,
					 POINTER_TO_UINT(output), length);
	am_hal_cc312_clear_interrupt(0xFFFFFFFFU);
	irq_enable(data->irq_num);
	am_hal_cc312_set_dma_source((am_hal_cc312_dma_addr_type_e)ctx->inputDataAddrType,
				    POINTER_TO_UINT(input), length);

	ret = k_sem_take(&data->irq_sem, K_MSEC(AMBIQ_CHACHA_IRQ_WAIT_TIMEOUT_MS));
	if (ret != 0) {
		status = AM_HAL_STATUS_TIMEOUT;
		goto process_exit;
	}

	if (((uint32_t)atomic_get(&data->irq_seen) & CRYPTO_HOSTRGFIRR_AHBERRINT_Msk) != 0U) {
		status = AM_HAL_STATUS_HW_ERR;
		goto process_exit;
	}

	if (((uint32_t)atomic_get(&data->irq_seen) & irr_mask) == 0U) {
		status = AM_HAL_STATUS_TIMEOUT;
		goto process_exit;
	}

	status = am_hal_cc312_chacha_store_state(ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto process_exit;
	}

	if (ctx->outputDataAddrType == AM_HAL_CC312_DMA_DLLI_ADDR) {
		status = ambiq_chacha_flush_dummy_dlli(data, POINTER_TO_UINT(output), length);
		if (status != AM_HAL_STATUS_SUCCESS) {
			goto process_exit;
		}
		ambiq_chacha_cache_invalidate(output, (size_t)length);
	}

process_exit:
	ambiq_chacha_finish_irq_wait(data);
	am_hal_cc312_chacha_disable_clocks();
	return status;
}

/* ---------- Zephyr crypto op handler ---------- */

/*
 * ChaCha20 is a symmetric stream cipher — encrypt and decrypt are the same
 * keystream-XOR operation.  The Zephyr crypto API passes the nonce via the
 * cipher_pkt iv field when using CRYPTO_CIPHER_MODE_CHACHA20.  The key and
 * initial block counter are taken from the cipher_ctx.
 *
 * ctx->key.bit_stream  : 32-byte key
 * ctx->keylen          : must be 32
 * iv (nonce)           : 12 bytes (96-bit, RFC 8439)
 * ctx->mode_params.ctr_info.ctr_len : initial block counter (low 32 bits)
 */
static int ambiq_chacha_crypt_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *nonce)
{
	struct ambiq_chacha_data *data;
	struct ambiq_chacha_dma_data *dma;
	uint8_t *out_buf;
	uint32_t status;
	int ctx_init = 0;
	int ret;

	if (ctx == NULL || pkt == NULL || nonce == NULL) {
		return -EINVAL;
	}

	ret = ambiq_chacha_get_output_buffer(ctx, pkt, &out_buf);
	if (ret != 0) {
		return ret;
	}

	if (ctx->device == NULL || ctx->device->data == NULL) {
		return -ENODEV;
	}
	data = ctx->device->data;
	dma = data->dma;

	k_mutex_lock(&data->lock, K_FOREVER);

	am_hal_cc312_chacha_context_init(&dma->ctx);
	status = am_hal_cc312_chacha_setkey(&dma->ctx, ctx->key.bit_stream,
					    (uint32_t)ctx->keylen * 8U);
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_chacha_hal_status_to_errno(status);
		goto cleanup;
	}

	uint32_t remaining = (uint32_t)pkt->in_len;
	uint32_t offset = 0U;
	uint32_t counter = ctx->mode_params.ctr_info.ctr_len;

	while (remaining > 0U) {
		uint32_t chunk = MIN(remaining, AMBIQ_CHACHA_MAX_CHUNK);

		status = am_hal_cc312_chacha_set_nonce(&dma->ctx, nonce,
						       AM_HAL_CHACHA_NONCE_SIZE_96, counter);
		if (status != AM_HAL_STATUS_SUCCESS) {
			ret = ambiq_chacha_hal_status_to_errno(status);
			goto cleanup;
		}
		ctx_init = 1;

		status = ambiq_chacha_process(data, &dma->ctx, pkt->in_buf + offset,
					      out_buf + offset, chunk);
		if (status != AM_HAL_STATUS_SUCCESS) {
			ret = ambiq_chacha_hal_status_to_errno(status);
			goto cleanup;
		}

		offset += chunk;
		remaining -= chunk;
		counter += chunk / AMBIQ_CHACHA_BLOCK_BYTES;
	}

	pkt->out_len = pkt->in_len;
	ret = 0;

cleanup:
	if (ctx_init != 0) {
		(void)am_hal_cc312_chacha_free(&dma->ctx);
	}
	if (ret != 0 && out_buf != NULL) {
		ambiq_chacha_secure_zero(out_buf, (size_t)pkt->in_len);
		pkt->out_len = 0;
	}
	ambiq_chacha_secure_zero(dma->dlli_flush_scratch, sizeof(dma->dlli_flush_scratch));
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------- Session management ---------- */

static int ambiq_chacha_validate_session(const struct cipher_ctx *ctx, enum cipher_algo algo,
					 enum cipher_mode mode)
{
	if (ctx->flags & ~AMBIQ_CHACHA_CAPS) {
		return -ENOTSUP;
	}
	if ((ctx->flags & CAP_RAW_KEY) == 0U) {
		return -ENOTSUP;
	}
	if ((ctx->flags & CAP_SYNC_OPS) == 0U) {
		return -ENOTSUP;
	}
	if (ctx->key.bit_stream == NULL) {
		return -EINVAL;
	}
	if (algo != CRYPTO_CIPHER_ALGO_CHACHA20) {
		return -ENOTSUP;
	}
	if (mode != CRYPTO_CIPHER_MODE_CHACHA20) {
		return -ENOTSUP;
	}
	if (ctx->keylen != CHACHA_KEY_SIZE) {
		return -EINVAL;
	}
	return 0;
}

static int ambiq_chacha_begin_session(const struct device *dev, struct cipher_ctx *ctx,
				      enum cipher_algo algo, enum cipher_mode mode,
				      enum cipher_op op_type)
{
	int ret;

	ARG_UNUSED(op_type); /* ChaCha20 encrypt == decrypt */

	if (dev == NULL || ctx == NULL || dev->data == NULL) {
		return -EINVAL;
	}

	ret = ambiq_chacha_validate_session(ctx, algo, mode);
	if (ret != 0) {
		return ret;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	ctx->ops.chacha20_crypt_hndlr = ambiq_chacha_crypt_op;
	ctx->ops.cipher_mode = mode;
	ctx->drv_sessn_state = NULL;
	return 0;
}

static int ambiq_chacha_free_session(const struct device *dev, struct cipher_ctx *ctx)
{
	if (dev == NULL || ctx == NULL || dev->data == NULL) {
		return -EINVAL;
	}

	ctx->ops.chacha20_crypt_hndlr = NULL;
	ctx->drv_sessn_state = NULL;

	(void)pm_device_runtime_put(dev);
	return 0;
}

static int ambiq_chacha_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return AMBIQ_CHACHA_CAPS;
}

static int ambiq_chacha_callback_set(const struct device *dev, cipher_completion_cb cb)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	return -ENOTSUP;
}

/* ---------- PM ---------- */

static int ambiq_chacha_pm_action(const struct device *dev, enum pm_device_action action)
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

/* ---------- Init ---------- */

static int ambiq_chacha_init(const struct device *dev)
{
	const struct ambiq_chacha_config *cfg = dev->config;
	struct ambiq_chacha_data *data = dev->data;

	k_mutex_init(&data->lock);
	k_sem_init(&data->irq_sem, 0, 1);
	data->irq_num = cfg->irq_num;
	(void)atomic_set(&data->irq_seen, 0U);
	(void)atomic_set(&data->irq_wait_mask, 0U);
	data->dma = cfg->dma;

	cfg->irq_config_func();
	irq_disable(data->irq_num);

	return pm_device_runtime_enable(dev);
}

/* ---------- Driver API ---------- */

static DEVICE_API(crypto, ambiq_chacha_crypto_api) = {
	.query_hw_caps           = ambiq_chacha_query_hw_caps,
	.cipher_begin_session    = ambiq_chacha_begin_session,
	.cipher_free_session     = ambiq_chacha_free_session,
	.cipher_async_callback_set = ambiq_chacha_callback_set,
};

/* ---------- Instance macros ---------- */

#define AMBIQ_CHACHA_IRQ_CONFIG(inst)                                          \
	static void ambiq_chacha_irq_config_##inst(void)                       \
	{                                                                      \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),  \
			    ambiq_cc312_chacha_isr,                            \
			    DEVICE_DT_INST_GET(inst), 0);                      \
	}

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_CHACHA_IRQ_CONFIG)

#define AMBIQ_CHACHA_DEVICE_DEFINE(inst)                                                   \
	static AMBIQ_CHACHA_NOCACHE                                                        \
		struct ambiq_chacha_dma_data ambiq_chacha_dma_##inst;                      \
	static struct ambiq_chacha_data ambiq_chacha_data_##inst;                          \
	static const struct ambiq_chacha_config ambiq_chacha_cfg_##inst = {                \
		.irq_num          = DT_INST_IRQN(inst),                                    \
		.irq_config_func  = ambiq_chacha_irq_config_##inst,                        \
		.dma              = &ambiq_chacha_dma_##inst,                              \
	};                                                                                 \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_chacha_pm_action);                            \
	DEVICE_DT_INST_DEFINE(inst, ambiq_chacha_init, PM_DEVICE_DT_INST_GET(inst),        \
			      &ambiq_chacha_data_##inst, &ambiq_chacha_cfg_##inst,         \
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,                    \
			      &ambiq_chacha_crypto_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_CHACHA_DEVICE_DEFINE)
