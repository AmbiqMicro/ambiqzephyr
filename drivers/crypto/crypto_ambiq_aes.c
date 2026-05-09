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

/* ambiq-sdk includes */
#include <soc.h>

LOG_MODULE_REGISTER(crypto_ambiq_aes, CONFIG_CRYPTO_LOG_LEVEL);

#define DT_DRV_COMPAT ambiq_crypto_aes

#define AMBIQ_AES_BLOCK_SIZE    16U
#define AMBIQ_AES_DMA_ALIGNMENT 32U
#define AMBIQ_AES_CAPS                                                                             \
	(CAP_RAW_KEY | CAP_INPLACE_OPS | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_NO_IV_PREFIX)
#define AMBIQ_AES_IRQ_WAIT_TIMEOUT_MS 2000

#if IS_ENABLED(CONFIG_NOCACHE_MEMORY)
#define AMBIQ_AES_NOCACHE __nocache
#else
#define AMBIQ_AES_NOCACHE
#endif

struct ambiq_aes_dma_data {
	am_hal_cc312_aes_context_t op_ctx;
	am_hal_cc312_aes_ccm_context_t ccm_ctx;
	am_hal_cc312_aes_gcm_context_t gcm_ctx;
	am_hal_cc312_aes_context_t xts_crypt_ctx;
	am_hal_cc312_aes_context_t xts_tweak_ctx;
	uint8_t ccm_tag_local[AMBIQ_AES_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
	uint8_t xts_tweak[AMBIQ_AES_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
	uint8_t xts_prev_tweak[AMBIQ_AES_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
	uint8_t xts_tmp_in[AMBIQ_AES_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
	uint8_t xts_tmp_out[AMBIQ_AES_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
	uint8_t dlli_flush_scratch[AMBIQ_AES_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
};

struct ambiq_aes_data {
	struct k_mutex lock;
	struct k_sem irq_sem;
	uint32_t irq_num;
	atomic_t irq_seen;
	atomic_t irq_wait_mask;
	struct ambiq_aes_dma_data *dma;
};

struct ambiq_aes_config {
	uint32_t irq_num;
	void (*irq_config_func)(void);
	struct ambiq_aes_dma_data *dma;
};

static void ambiq_aes_prepare_irq_wait(struct ambiq_aes_data *data, uint32_t wait_mask)
{
	irq_disable(data->irq_num);
	k_sem_reset(&data->irq_sem);
	(void)atomic_set(&data->irq_seen, 0U);
	(void)atomic_set(&data->irq_wait_mask, wait_mask);
}

static void ambiq_aes_finish_irq_wait(struct ambiq_aes_data *data)
{
	irq_disable(data->irq_num);
	(void)atomic_set(&data->irq_wait_mask, 0U);
}

static void ambiq_cc312_isr(const void *arg)
{
	const struct device *dev = arg;
	struct ambiq_aes_data *data;
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

static int ambiq_aes_data_from_ctx(struct cipher_ctx *ctx, struct ambiq_aes_data **data_out)
{
	struct ambiq_aes_data *data;

	if (ctx == NULL || ctx->device == NULL || ctx->device->data == NULL) {
		return -ENODEV;
	}

	data = ctx->device->data;
	*data_out = data;

	return 0;
}

static int ambiq_aes_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return AMBIQ_AES_CAPS;
}

static int ambiq_hal_status_to_errno(uint32_t status)
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

static int ambiq_hal_auth_status_to_errno(uint32_t status)
{
	if (status == AM_HAL_STATUS_FAIL) {
		return -EFAULT;
	}

	return ambiq_hal_status_to_errno(status);
}

/* Toolchain-aware secure zero and constant-time diff helpers.
 */

/* Use Zephyr-provided `__noinline` when available instead of local macros. */
__noinline static void ambiq_aes_secure_zero(void *buf, size_t len)
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

/* Keep as static but force emission and prevent inlining/vectorization.
 * Returns 0 when equal, non-zero otherwise.
 */
__noinline __attribute__((used)) static uint32_t
ambiq_aes_auth_diff(const uint8_t *lhs, const uint8_t *rhs, uint32_t len)
{
	uint32_t diff = 0U;
	uint32_t i;

	if (lhs == NULL || rhs == NULL || len == 0U) {
		return diff;
	}

	for (i = 0U; i < len; i++) {
		diff |= ((uint32_t)lhs[i] ^ (uint32_t)rhs[i]);
	}

	__DMB();
	return diff;
}

static int ambiq_aes_get_output_buffer(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				       uint8_t **out_buf, size_t min_len)
{
	if (pkt->in_buf == NULL) {
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

	if ((size_t)pkt->out_buf_max < min_len) {
		return -EINVAL;
	}

	*out_buf = pkt->out_buf;
	return 0;
}

static uint32_t ambiq_aes_prepare_key_ctx(am_hal_cc312_aes_context_t *op_ctx, const uint8_t *key,
					  uint32_t key_bits, int32_t hal_mode)
{
	uint32_t status;

	status = am_hal_cc312_aes_context_init(op_ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	if (hal_mode == AM_HAL_AES_ENCRYPT) {
		status = am_hal_cc312_aes_setkey_enc(op_ctx, key, key_bits);
	} else {
		status = am_hal_cc312_aes_setkey_dec(op_ctx, key, key_bits);
	}
	if (status != AM_HAL_STATUS_SUCCESS) {
		(void)am_hal_cc312_aes_free(op_ctx);
	}

	return status;
}

static uint32_t ambiq_aes_prepare_op_ctx(am_hal_cc312_aes_context_t *op_ctx,
					 const struct cipher_ctx *ctx, int32_t hal_mode)
{
	return ambiq_aes_prepare_key_ctx(op_ctx, ctx->key.bit_stream, (uint32_t)ctx->keylen * 8U,
					 hal_mode);
}

static uint32_t ambiq_aes_prepare_ccm_ctx(am_hal_cc312_aes_ccm_context_t *ctx, const uint8_t *key,
					  uint32_t key_bits)
{
	uint32_t status;

	am_hal_aes_ccm_init(ctx);
	status = am_hal_aes_ccm_setkey(ctx, key, key_bits);
	if (status != AM_HAL_STATUS_SUCCESS) {
		am_hal_aes_ccm_free(ctx);
	}

	return status;
}

static uint32_t ambiq_aes_prepare_gcm_ctx(am_hal_cc312_aes_gcm_context_t *ctx, const uint8_t *key,
					  uint32_t key_bits)
{
	uint32_t status;

	am_hal_aes_gcm_init(ctx);
	status = am_hal_aes_gcm_setkey(ctx, key, key_bits);
	if (status != AM_HAL_STATUS_SUCCESS) {
		am_hal_aes_gcm_free(ctx);
	}

	return status;
}

static uint32_t ambiq_cc312_flush_dummy_dlli(struct ambiq_aes_data *data, uint32_t src_addr,
					     uint32_t length)
{
	uint32_t irr_mask = CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk;
	uint32_t flush_len;
	uint32_t src_tail_addr;
	uint32_t status = AM_HAL_STATUS_SUCCESS;
	int ret;

	if (data == NULL || data->dma == NULL || src_addr == 0U || length == 0U) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	ambiq_aes_secure_zero(data->dma->dlli_flush_scratch, sizeof(data->dma->dlli_flush_scratch));

	flush_len = MIN(length, AMBIQ_AES_BLOCK_SIZE);
	src_tail_addr = src_addr + (length - flush_len);

	ambiq_aes_prepare_irq_wait(data, irr_mask);
	am_hal_cc312_set_dma_destination(AM_HAL_CC312_DMA_DLLI_ADDR,
					 POINTER_TO_UINT(data->dma->dlli_flush_scratch), flush_len);
	am_hal_cc312_clear_interrupt(0xFFFFFFFFU);
	irq_enable(data->irq_num);
	am_hal_cc312_set_dma_source(AM_HAL_CC312_DMA_DLLI_ADDR, src_tail_addr, flush_len);

	ret = k_sem_take(&data->irq_sem, K_MSEC(AMBIQ_AES_IRQ_WAIT_TIMEOUT_MS));
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
	ambiq_aes_secure_zero(data->dma->dlli_flush_scratch, sizeof(data->dma->dlli_flush_scratch));

	return status;
}

static void ambiq_aes_cache_clean_invalidate_region(const void *addr, size_t length)
{
	if (addr == NULL || length == 0U) {
		return;
	}

	if (!buf_in_nocache(POINTER_TO_UINT(addr), length)) {
		(void)sys_cache_data_flush_and_invd_range((void *)addr, length);
	}
}

static void ambiq_aes_cache_invalidate_region(const void *addr, size_t length)
{
	if (addr == NULL || length == 0U) {
		return;
	}

	if (!buf_in_nocache(POINTER_TO_UINT(addr), length)) {
		(void)sys_cache_data_invd_range((void *)addr, length);
	}
}

/* Securely clear DMA scratch buffers in the per-device DMA struct. */
static void ambiq_aes_clear_dma_scratch(struct ambiq_aes_dma_data *dma)
{
	if (dma == NULL) {
		return;
	}

	ambiq_aes_secure_zero(dma->ccm_tag_local, sizeof(dma->ccm_tag_local));
	ambiq_aes_secure_zero(dma->xts_tweak, sizeof(dma->xts_tweak));
	ambiq_aes_secure_zero(dma->xts_prev_tweak, sizeof(dma->xts_prev_tweak));
	ambiq_aes_secure_zero(dma->xts_tmp_in, sizeof(dma->xts_tmp_in));
	ambiq_aes_secure_zero(dma->xts_tmp_out, sizeof(dma->xts_tmp_out));
	ambiq_aes_secure_zero(dma->dlli_flush_scratch, sizeof(dma->dlli_flush_scratch));
}

/* AES (ECB/CTR/OFB/XTS) */
static uint32_t ambiq_cc312_aes_process(struct ambiq_aes_data *data,
					am_hal_cc312_aes_context_t *ctx,
					am_hal_cc312_buffer_t *input_info,
					am_hal_cc312_buffer_t *output_info, uint32_t length)
{
	uint32_t status;
	uint32_t irr_val = 0U;
	int ret;

	if (data == NULL || ctx == NULL || input_info == NULL || output_info == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	ambiq_aes_prepare_irq_wait(data, CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk);

	am_hal_cc312_aes_enable_clocks();

	status = am_hal_cc312_aes_init(ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto process_exit;
	}

	status = am_hal_cc312_aes_load_key(ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto process_exit;
	}

	status = am_hal_cc312_aes_load_iv(ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto process_exit;
	}

	am_hal_cc312_set_buffer_security(input_info->ui8NonSecure, output_info->ui8NonSecure);
	ambiq_aes_cache_clean_invalidate_region(UINT_TO_POINTER(input_info->ui32DataAddr),
						(size_t)length);
	ambiq_aes_cache_clean_invalidate_region(UINT_TO_POINTER(output_info->ui32DataAddr),
						(size_t)length);

	am_hal_cc312_set_dma_destination((am_hal_cc312_dma_addr_type_e)ctx->outputDataAddrType,
					 output_info->ui32DataAddr, length);
	am_hal_cc312_clear_interrupt(0xFFFFFFFFU);
	irq_enable(data->irq_num);
	am_hal_cc312_set_dma_source((am_hal_cc312_dma_addr_type_e)ctx->inputDataAddrType,
				    input_info->ui32DataAddr, length);

	irr_val |= CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk;
	ret = k_sem_take(&data->irq_sem, K_MSEC(AMBIQ_AES_IRQ_WAIT_TIMEOUT_MS));
	if (ret != 0) {
		status = AM_HAL_STATUS_TIMEOUT;
		goto process_exit;
	}

	if (((uint32_t)atomic_get(&data->irq_seen) & CRYPTO_HOSTRGFIRR_AHBERRINT_Msk) != 0U) {
		status = AM_HAL_STATUS_HW_ERR;
		goto process_exit;
	}

	if (((uint32_t)atomic_get(&data->irq_seen) & irr_val) == 0U) {
		status = AM_HAL_STATUS_TIMEOUT;
		goto process_exit;
	}

	status = am_hal_cc312_aes_store_iv(ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto process_exit;
	}

	ctx->dataBlockType = MIDDLE_BLOCK;

	if (ctx->outputDataAddrType == DLLI_ADDR) {
		status = ambiq_cc312_flush_dummy_dlli(data, output_info->ui32DataAddr, length);
		if (status != AM_HAL_STATUS_SUCCESS) {
			goto process_exit;
		}

		ambiq_aes_cache_invalidate_region(UINT_TO_POINTER(output_info->ui32DataAddr),
						  (size_t)length);
	}

process_exit:
	ambiq_aes_finish_irq_wait(data);
	am_hal_cc312_aes_disable_clocks();
	return status;
}

static uint32_t ambiq_cc312_aes_crypt_ecb(struct ambiq_aes_data *data,
					  am_hal_cc312_aes_context_t *ctx, int32_t mode,
					  const uint8_t input[AMBIQ_AES_BLOCK_SIZE],
					  uint8_t output[AMBIQ_AES_BLOCK_SIZE])
{
	am_hal_cc312_buffer_t input_info = {0};
	am_hal_cc312_buffer_t output_info = {0};
	uint32_t status;

	if (ctx == NULL || input == NULL || output == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	if (mode != AM_HAL_AES_DECRYPT && mode != AM_HAL_AES_ENCRYPT) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	if ((mode == AM_HAL_AES_ENCRYPT && ctx->dir != CRYPTO_DIRECTION_ENCRYPT) ||
	    (mode == AM_HAL_AES_DECRYPT && ctx->dir != CRYPTO_DIRECTION_DECRYPT)) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	ctx->mode = CIPHER_ECB;

	status = am_hal_cc312_aes_set_data_buffers_info(input, &input_info, output, &output_info);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	return ambiq_cc312_aes_process(data, ctx, &input_info, &output_info, AMBIQ_AES_BLOCK_SIZE);
}

static uint32_t ambiq_cc312_aes_crypt_ctr_ofb(struct ambiq_aes_data *data,
					      am_hal_cc312_aes_context_t *ctx,
					      am_hal_cc312_aes_mode_e mode, uint32_t length,
					      uint32_t *iv_off, uint8_t iv[AMBIQ_AES_BLOCK_SIZE],
					      const uint8_t *input, uint8_t *output)
{
	am_hal_cc312_buffer_t input_info = {0};
	am_hal_cc312_buffer_t output_info = {0};
	uint32_t status;

	if (length == 0U) {
		return AM_HAL_STATUS_SUCCESS;
	}

	if (ctx == NULL || iv == NULL || input == NULL || output == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	if ((iv_off != NULL) && (*iv_off != 0U)) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	ctx->mode = mode;
	memcpy(ctx->ivBuf, iv, AMBIQ_AES_BLOCK_SIZE);

	status = am_hal_cc312_aes_set_data_buffers_info(input, &input_info, output, &output_info);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	status = ambiq_cc312_aes_process(data, ctx, &input_info, &output_info, length);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	memcpy(iv, ctx->ivBuf, AMBIQ_AES_BLOCK_SIZE);
	return AM_HAL_STATUS_SUCCESS;
}

static uint32_t ambiq_cc312_aes_crypt_ctr(struct ambiq_aes_data *data,
					  am_hal_cc312_aes_context_t *ctx, uint32_t length,
					  uint32_t *nc_off,
					  uint8_t nonce_counter[AMBIQ_AES_BLOCK_SIZE],
					  const uint8_t *input, uint8_t *output)
{
	return ambiq_cc312_aes_crypt_ctr_ofb(data, ctx, CIPHER_CTR, length, nc_off, nonce_counter,
					     input, output);
}

static uint32_t ambiq_cc312_aes_crypt_ofb(struct ambiq_aes_data *data,
					  am_hal_cc312_aes_context_t *ctx, uint32_t length,
					  uint32_t *iv_off, uint8_t iv[AMBIQ_AES_BLOCK_SIZE],
					  const uint8_t *input, uint8_t *output)
{
	return ambiq_cc312_aes_crypt_ctr_ofb(data, ctx, CIPHER_OFB, length, iv_off, iv, input,
					     output);
}

static int ambiq_aes_ecb_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt, int32_t hal_mode)
{
	struct ambiq_aes_data *data;
	struct ambiq_aes_dma_data *dma;
	uint32_t status;
	int op_ctx_init = 0;
	uint8_t *out_buf;
	int ret;

	if (ctx == NULL || pkt == NULL) {
		return -EINVAL;
	}

	if (pkt->in_len != AMBIQ_AES_BLOCK_SIZE) {
		return -EINVAL;
	}

	ret = ambiq_aes_get_output_buffer(ctx, pkt, &out_buf, AMBIQ_AES_BLOCK_SIZE);
	if (ret != 0) {
		return ret;
	}
	ret = ambiq_aes_data_from_ctx(ctx, &data);
	if (ret != 0) {
		return ret;
	}
	dma = data->dma;

	k_mutex_lock(&data->lock, K_FOREVER);

	status = ambiq_aes_prepare_op_ctx(&dma->op_ctx, ctx, hal_mode);
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_status_to_errno(status);
		goto cleanup;
	}
	op_ctx_init = 1;

	status = ambiq_cc312_aes_crypt_ecb(data, &dma->op_ctx, hal_mode, pkt->in_buf, out_buf);
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_status_to_errno(status);
		goto cleanup;
	}

	pkt->out_len = AMBIQ_AES_BLOCK_SIZE;
	ret = 0;

cleanup:
	if (op_ctx_init != 0) {
		(void)am_hal_cc312_aes_free(&dma->op_ctx);
	}

	/* On error, ensure any partial plaintext/ciphertext is zeroed */
	if (ret != 0 && out_buf != NULL) {
		ambiq_aes_secure_zero(out_buf, AMBIQ_AES_BLOCK_SIZE);
		if (pkt != NULL) {
			pkt->out_len = 0;
		}
	}

	/* Clear DMA scratch buffers */
	ambiq_aes_clear_dma_scratch(dma);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int ambiq_aes_ecb_encrypt_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	return ambiq_aes_ecb_op(ctx, pkt, AM_HAL_AES_ENCRYPT);
}

static int ambiq_aes_ecb_decrypt_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	return ambiq_aes_ecb_op(ctx, pkt, AM_HAL_AES_DECRYPT);
}

static int ambiq_aes_ctr_ofb_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv,
				enum cipher_mode mode)
{
	struct ambiq_aes_data *data;
	struct ambiq_aes_dma_data *dma;
	uint32_t status;
	uint32_t offset = 0;
	uint32_t ctr_len_bits = 0U;
	size_t iv_copy_len = AMBIQ_AES_BLOCK_SIZE;
	int op_ctx_init = 0;
	uint8_t iv_local[AMBIQ_AES_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
	uint8_t *out_buf;
	int ret;

	if (ctx == NULL || pkt == NULL) {
		return -EINVAL;
	}

	if (iv == NULL) {
		return -EINVAL;
	}

	ret = ambiq_aes_get_output_buffer(ctx, pkt, &out_buf, (size_t)pkt->in_len);
	if (ret != 0) {
		return ret;
	}
	ambiq_aes_secure_zero(iv_local, sizeof(iv_local));
	if (mode == CRYPTO_CIPHER_MODE_CTR) {
		ctr_len_bits = ctx->mode_params.ctr_info.ctr_len;
		if ((ctr_len_bits == 0U) || ((ctr_len_bits % 8U) != 0U) ||
		    (ctr_len_bits > (AMBIQ_AES_BLOCK_SIZE * 8U))) {
			return -EINVAL;
		}
		iv_copy_len = AMBIQ_AES_BLOCK_SIZE - (ctr_len_bits / 8U);
		/*
		 * CTR split-counter mode: caller provides only IV prefix and
		 * driver manages the counter field. Start with counter value 1.
		 */
		iv_local[AMBIQ_AES_BLOCK_SIZE - 1U] = 1U;
	}
	if (iv_copy_len > 0U) {
		memcpy(iv_local, iv, iv_copy_len);
	}

	ret = ambiq_aes_data_from_ctx(ctx, &data);
	if (ret != 0) {
		return ret;
	}
	dma = data->dma;

	k_mutex_lock(&data->lock, K_FOREVER);

	status = ambiq_aes_prepare_op_ctx(&dma->op_ctx, ctx, AM_HAL_AES_ENCRYPT);
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_status_to_errno(status);
		goto cleanup;
	}
	op_ctx_init = 1;

	if (mode == CRYPTO_CIPHER_MODE_CTR) {
		status = ambiq_cc312_aes_crypt_ctr(data, &dma->op_ctx, (uint32_t)pkt->in_len,
						   &offset, iv_local, pkt->in_buf, out_buf);
	} else {
		status = ambiq_cc312_aes_crypt_ofb(data, &dma->op_ctx, (uint32_t)pkt->in_len,
						   &offset, iv_local, pkt->in_buf, out_buf);
	}
	if (iv_copy_len > 0U) {
		memcpy(iv, iv_local, iv_copy_len);
	}
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_status_to_errno(status);
		goto cleanup;
	}

	pkt->out_len = pkt->in_len;
	ret = 0;

cleanup:
	if (op_ctx_init != 0) {
		(void)am_hal_cc312_aes_free(&dma->op_ctx);
	}

	/* Zero output on failure to avoid leaking partial data */
	if (ret != 0 && out_buf != NULL) {
		ambiq_aes_secure_zero(out_buf, (size_t)pkt->in_len);
		if (pkt != NULL) {
			pkt->out_len = 0;
		}
	}

	/* Clear DMA scratch buffers */
	ambiq_aes_clear_dma_scratch(dma);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int ambiq_aes_ctr_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	return ambiq_aes_ctr_ofb_op(ctx, pkt, iv, CRYPTO_CIPHER_MODE_CTR);
}

static int ambiq_aes_xts_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *data_unit,
			    int32_t hal_mode)
{
	struct ambiq_aes_data *data;
	struct ambiq_aes_dma_data *dma;
	const uint8_t *key1;
	const uint8_t *key2;
	uint8_t *in_ptr;
	uint8_t *out_ptr;
	uint32_t blocks;
	uint32_t leftover;
	uint32_t status;
	uint32_t i;
	int tweak_ctx_init = 0;
	int crypt_ctx_init = 0;
	uint8_t *out_buf;
	int ret;

	if (ctx == NULL || pkt == NULL) {
		return -EINVAL;
	}

	if (data_unit == NULL || pkt->in_len < AMBIQ_AES_BLOCK_SIZE) {
		return -EINVAL;
	}

	in_ptr = pkt->in_buf;

	ret = ambiq_aes_get_output_buffer(ctx, pkt, &out_buf, (size_t)pkt->in_len);
	if (ret != 0) {
		return ret;
	}

	ret = ambiq_aes_data_from_ctx(ctx, &data);
	if (ret != 0) {
		return ret;
	}

	dma = data->dma;
	k_mutex_lock(&data->lock, K_FOREVER);

	key1 = ctx->key.bit_stream;
	key2 = ctx->key.bit_stream + (ctx->keylen / 2U);

	status = ambiq_aes_prepare_key_ctx(&dma->xts_tweak_ctx, key2,
					   (uint32_t)(ctx->keylen / 2U) * 8U, AM_HAL_AES_ENCRYPT);
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_status_to_errno(status);
		goto cleanup;
	}
	tweak_ctx_init = 1;

	status = ambiq_aes_prepare_key_ctx(&dma->xts_crypt_ctx, key1,
					   (uint32_t)(ctx->keylen / 2U) * 8U, hal_mode);
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_status_to_errno(status);
		goto cleanup;
	}
	crypt_ctx_init = 1;

	status = ambiq_cc312_aes_crypt_ecb(data, &dma->xts_tweak_ctx, AM_HAL_AES_ENCRYPT, data_unit,
					   dma->xts_tweak);
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_status_to_errno(status);
		goto cleanup;
	}

	blocks = (uint32_t)pkt->in_len / AMBIQ_AES_BLOCK_SIZE;
	leftover = (uint32_t)pkt->in_len % AMBIQ_AES_BLOCK_SIZE;
	if (leftover != 0U) {
		blocks--;
	}
	out_ptr = out_buf;

	while (blocks--) {
		for (i = 0; i < AMBIQ_AES_BLOCK_SIZE; i++) {
			dma->xts_tmp_in[i] = in_ptr[i] ^ dma->xts_tweak[i];
		}

		status = ambiq_cc312_aes_crypt_ecb(data, &dma->xts_crypt_ctx, hal_mode,
						   dma->xts_tmp_in, dma->xts_tmp_out);
		if (status != AM_HAL_STATUS_SUCCESS) {
			ret = ambiq_hal_status_to_errno(status);
			goto cleanup;
		}

		for (i = 0; i < AMBIQ_AES_BLOCK_SIZE; i++) {
			out_ptr[i] = dma->xts_tmp_out[i] ^ dma->xts_tweak[i];
		}

		/* GF(2^128) multiply-by-x in little-endian representation. */
		{
			uint8_t carry = 0U;
			uint8_t next_carry;

			for (i = 0; i < AMBIQ_AES_BLOCK_SIZE; i++) {
				next_carry = (uint8_t)(dma->xts_tweak[i] >> 7);
				dma->xts_tweak[i] = (uint8_t)((dma->xts_tweak[i] << 1) | carry);
				carry = next_carry;
			}
			if (carry != 0U) {
				dma->xts_tweak[0] ^= 0x87U;
			}
		}

		in_ptr += AMBIQ_AES_BLOCK_SIZE;
		out_ptr += AMBIQ_AES_BLOCK_SIZE;
	}

	if (leftover) {
		uint8_t t_cur[AMBIQ_AES_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
		uint8_t t_next[AMBIQ_AES_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
		uint8_t c_partial[AMBIQ_AES_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
		uint8_t *in_last;
		uint8_t *in_tail;
		uint8_t *out_last;
		uint8_t *out_tail;
		uint8_t carry = 0U;
		uint8_t next_carry;

		/*
		 * For ciphertext stealing, leave one full block plus the partial tail
		 * for dedicated handling below.
		 */
		in_last = in_ptr;
		in_tail = in_ptr + AMBIQ_AES_BLOCK_SIZE;
		out_last = out_ptr;
		out_tail = out_ptr + AMBIQ_AES_BLOCK_SIZE;

		memcpy(t_cur, dma->xts_tweak, sizeof(t_cur));
		memcpy(t_next, t_cur, sizeof(t_next));
		for (i = 0; i < AMBIQ_AES_BLOCK_SIZE; i++) {
			next_carry = (uint8_t)(t_next[i] >> 7);
			t_next[i] = (uint8_t)((t_next[i] << 1) | carry);
			carry = next_carry;
		}
		if (carry != 0U) {
			t_next[0] ^= 0x87U;
		}

		ambiq_aes_secure_zero(c_partial, sizeof(c_partial));
		memcpy(c_partial, in_tail, leftover);

		if (hal_mode == AM_HAL_AES_ENCRYPT) {
			/* CC = E(P_n xor T_n) xor T_n */
			for (i = 0; i < AMBIQ_AES_BLOCK_SIZE; i++) {
				dma->xts_tmp_in[i] = in_last[i] ^ t_cur[i];
			}
			status = ambiq_cc312_aes_crypt_ecb(data, &dma->xts_crypt_ctx,
							   AM_HAL_AES_ENCRYPT, dma->xts_tmp_in,
							   dma->xts_tmp_out);
			if (status != AM_HAL_STATUS_SUCCESS) {
				ret = ambiq_hal_status_to_errno(status);
				goto cleanup;
			}
			for (i = 0; i < AMBIQ_AES_BLOCK_SIZE; i++) {
				dma->xts_tmp_out[i] ^= t_cur[i];
			}

			/* C_* = MSB_r(CC) */
			memcpy(out_tail, dma->xts_tmp_out, leftover);

			/* PP = P_* || LSB_{16-r}(CC), then C_n = E(PP xor T_{n+1}) xor T_{n+1} */
			for (i = 0; i < leftover; i++) {
				dma->xts_tmp_in[i] = c_partial[i] ^ t_next[i];
			}
			for (; i < AMBIQ_AES_BLOCK_SIZE; i++) {
				dma->xts_tmp_in[i] = dma->xts_tmp_out[i] ^ t_next[i];
			}
			status = ambiq_cc312_aes_crypt_ecb(data, &dma->xts_crypt_ctx,
							   AM_HAL_AES_ENCRYPT, dma->xts_tmp_in,
							   dma->xts_tmp_out);
			if (status != AM_HAL_STATUS_SUCCESS) {
				ret = ambiq_hal_status_to_errno(status);
				goto cleanup;
			}
			for (i = 0; i < AMBIQ_AES_BLOCK_SIZE; i++) {
				out_last[i] = dma->xts_tmp_out[i] ^ t_next[i];
			}
		} else {
			/* PP = D(C_n xor T_{n+1}) xor T_{n+1} */
			for (i = 0; i < AMBIQ_AES_BLOCK_SIZE; i++) {
				dma->xts_tmp_in[i] = in_last[i] ^ t_next[i];
			}
			status = ambiq_cc312_aes_crypt_ecb(data, &dma->xts_crypt_ctx,
							   AM_HAL_AES_DECRYPT, dma->xts_tmp_in,
							   dma->xts_tmp_out);
			if (status != AM_HAL_STATUS_SUCCESS) {
				ret = ambiq_hal_status_to_errno(status);
				goto cleanup;
			}
			for (i = 0; i < AMBIQ_AES_BLOCK_SIZE; i++) {
				dma->xts_tmp_out[i] ^= t_next[i];
			}

			/* P_* = MSB_r(PP) */
			memcpy(out_tail, dma->xts_tmp_out, leftover);

			/* CC = C_* || LSB_{16-r}(PP), then P_n = D(CC xor T_n) xor T_n */
			for (i = 0; i < leftover; i++) {
				dma->xts_tmp_in[i] = c_partial[i] ^ t_cur[i];
			}
			for (; i < AMBIQ_AES_BLOCK_SIZE; i++) {
				dma->xts_tmp_in[i] = dma->xts_tmp_out[i] ^ t_cur[i];
			}
			status = ambiq_cc312_aes_crypt_ecb(data, &dma->xts_crypt_ctx,
							   AM_HAL_AES_DECRYPT, dma->xts_tmp_in,
							   dma->xts_tmp_out);
			if (status != AM_HAL_STATUS_SUCCESS) {
				ret = ambiq_hal_status_to_errno(status);
				goto cleanup;
			}
			for (i = 0; i < AMBIQ_AES_BLOCK_SIZE; i++) {
				out_last[i] = dma->xts_tmp_out[i] ^ t_cur[i];
			}
		}
	}

	pkt->out_len = pkt->in_len;
	ret = 0;

cleanup:
	if (crypt_ctx_init != 0) {
		(void)am_hal_cc312_aes_free(&dma->xts_crypt_ctx);
	}
	if (tweak_ctx_init != 0) {
		(void)am_hal_cc312_aes_free(&dma->xts_tweak_ctx);
	}

	/* Zero output on error to avoid leaking plaintext/ciphertext */
	if (ret != 0 && out_buf != NULL) {
		ambiq_aes_secure_zero(out_buf, (size_t)pkt->in_len);
		if (pkt != NULL) {
			pkt->out_len = 0;
		}
	}

	/* Clear DMA scratch buffers */
	ambiq_aes_clear_dma_scratch(dma);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int ambiq_aes_xts_encrypt_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				    uint8_t *data_unit)
{
	return ambiq_aes_xts_op(ctx, pkt, data_unit, AM_HAL_AES_ENCRYPT);
}

static int ambiq_aes_xts_decrypt_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				    uint8_t *data_unit)
{
	return ambiq_aes_xts_op(ctx, pkt, data_unit, AM_HAL_AES_DECRYPT);
}

static int ambiq_aes_ofb_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	return ambiq_aes_ctr_ofb_op(ctx, pkt, iv, CRYPTO_CIPHER_MODE_OFB);
}

/* AES-CCM */
static uint32_t ambiq_cc312_ccm_process(struct ambiq_aes_data *data,
					am_hal_cc312_aes_ccm_context_t *ctx, const uint8_t *input,
					uint8_t *output, uint32_t length)
{
	am_hal_cc312_buffer_t input_info = {0};
	am_hal_cc312_buffer_t output_info = {0};
	uint32_t status;
	int ret;

	if (data == NULL || ctx == NULL || input == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	if ((ctx->mode != AM_HAL_CCM_MODE_CBC_MAC) && (output == NULL)) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	input_info.ui32DataAddr = POINTER_TO_UINT(input);
	input_info.ui8NonSecure = 0U;
	output_info.ui32DataAddr = POINTER_TO_UINT(output);
	output_info.ui8NonSecure = 0U;

	ambiq_aes_prepare_irq_wait(data, CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk);

	am_hal_cc312_aes_enable_clocks();

	status = am_hal_aes_ccm_init_hw(ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto process_exit;
	}

	am_hal_aes_ccm_load_key(ctx);
	am_hal_cc312_set_buffer_security(input_info.ui8NonSecure, output_info.ui8NonSecure);
	ambiq_aes_cache_clean_invalidate_region(UINT_TO_POINTER(input_info.ui32DataAddr),
						(size_t)length);
	if (output != NULL) {
		ambiq_aes_cache_clean_invalidate_region(UINT_TO_POINTER(output_info.ui32DataAddr),
							(size_t)length);
	}

	if (ctx->mode != AM_HAL_CCM_MODE_CBC_MAC) {
		am_hal_aes_ccm_load_ctr(ctx);
		am_hal_cc312_set_dma_destination(AM_HAL_CC312_DMA_DLLI_ADDR,
						 output_info.ui32DataAddr, length);
	}

	if (ctx->mode != AM_HAL_CCM_MODE_CTR) {
		am_hal_aes_ccm_load_iv(ctx);
		CRYPTO->AESCMACINIT = AM_HAL_CC312_AES_CCM_CMAC_INIT_VAL;
		CRYPTO->AESREMAININGBYTES = length;
	}

	am_hal_cc312_clear_interrupt(0xFFFFFFFFU);
	irq_enable(data->irq_num);
	am_hal_cc312_set_dma_source(AM_HAL_CC312_DMA_DLLI_ADDR, input_info.ui32DataAddr, length);

	ret = k_sem_take(&data->irq_sem, K_MSEC(AMBIQ_AES_IRQ_WAIT_TIMEOUT_MS));
	if (ret != 0) {
		status = AM_HAL_STATUS_TIMEOUT;
		goto process_exit;
	}

	if (((uint32_t)atomic_get(&data->irq_seen) & CRYPTO_HOSTRGFIRR_AHBERRINT_Msk) != 0U) {
		status = AM_HAL_STATUS_HW_ERR;
		goto process_exit;
	}

	if (((uint32_t)atomic_get(&data->irq_seen) & CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk) == 0U) {
		status = AM_HAL_STATUS_TIMEOUT;
		goto process_exit;
	}

	if (ctx->mode != AM_HAL_CCM_MODE_CBC_MAC) {
		am_hal_aes_ccm_store_ctr(ctx);
	}

	if (ctx->mode != AM_HAL_CCM_MODE_CTR) {
		am_hal_aes_ccm_store_iv(ctx);
	}

	status = AM_HAL_STATUS_SUCCESS;

	if (output != NULL) {
		status = ambiq_cc312_flush_dummy_dlli(data, output_info.ui32DataAddr, length);
		if (status != AM_HAL_STATUS_SUCCESS) {
			goto process_exit;
		}

		ambiq_aes_cache_invalidate_region(UINT_TO_POINTER(output_info.ui32DataAddr),
						  (size_t)length);
	}

process_exit:
	ambiq_aes_finish_irq_wait(data);
	am_hal_cc312_aes_disable_clocks();
	return status;
}

static uint32_t ambiq_cc312_ccm_init(struct ambiq_aes_data *data,
				     am_hal_cc312_aes_ccm_context_t *ctx, uint8_t dir,
				     uint32_t aad_len, uint32_t data_len, const uint8_t *nonce,
				     uint8_t nonce_len, uint8_t tag_len, uint32_t ccm_mode)
{
	uint8_t ctr_state_buf[AM_HAL_AES_CCM_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);
	uint8_t q_field_size = 15U - nonce_len;
	uint8_t security_level_field = 0U;
	uint8_t *temp_buff;
	uint32_t status;

	if (ctx == NULL || nonce == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	if (dir >= 2U) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	if (ccm_mode == AM_HAL_AES_CCM_MODE_CCM) {
		if ((tag_len < 4U) || (tag_len > 16U) || ((tag_len & 1U) != 0U)) {
			return AM_HAL_STATUS_INVALID_ARG;
		}
	} else if (ccm_mode == AM_HAL_AES_CCM_MODE_STAR) {
		status = am_hal_aes_ccm_get_security_level(tag_len, &security_level_field);
		if (status != AM_HAL_STATUS_SUCCESS) {
			return status;
		}

		if (nonce_len != AM_HAL_AES_CCM_STAR_NONCE_SIZE) {
			return AM_HAL_STATUS_INVALID_ARG;
		}

		if (nonce[AM_HAL_AES_CCM_STAR_NONCE_SIZE - 1U] != security_level_field) {
			return AM_HAL_STATUS_INVALID_ARG;
		}
	} else {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	if ((q_field_size < 2U) || (q_field_size > 8U)) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	if ((q_field_size < 4U) && ((data_len >> (q_field_size * 8U)) > 0U)) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	if ((nonce_len < AM_HAL_AES_CCM_NONCE_MIN_SIZE) ||
	    (nonce_len >= AM_HAL_AES_CCM_BLOCK_SIZE)) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	ctx->mode = AM_HAL_CCM_MODE_CBC_MAC;
	ctx->sizeOfN = nonce_len;
	ctx->sizeOfT = tag_len;
	ctx->dir = dir;

	ambiq_aes_secure_zero(ctx->ivBuf, sizeof(ctx->ivBuf));
	ambiq_aes_secure_zero(ctx->ctrStateBuf, sizeof(ctx->ctrStateBuf));
	ambiq_aes_secure_zero(ctx->tempBuff, sizeof(ctx->tempBuff));

	temp_buff = ctx->tempBuff;
	if (aad_len > 0U) {
		temp_buff[0] = 1U << 6;
	}

	temp_buff[0] |= (((tag_len - 2U) / 2U) << 3);
	temp_buff[0] |= (q_field_size - 1U);
	memcpy(temp_buff + 1, nonce, nonce_len);

	am_hal_aes_ccm_reverse_memcpy(temp_buff + 16U - MIN(q_field_size, 4U), (uint8_t *)&data_len,
				      MIN(q_field_size, 4U));

	status = ambiq_cc312_ccm_process(data, ctx, temp_buff, NULL, AM_HAL_AES_CCM_BLOCK_SIZE);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	ambiq_aes_secure_zero(ctr_state_buf, sizeof(ctr_state_buf));
	ctr_state_buf[0] = q_field_size - 1U;
	memcpy(ctr_state_buf + 1, nonce, nonce_len);
	ctr_state_buf[15] = 1U;
	memcpy(ctx->ctrStateBuf, ctr_state_buf, AM_HAL_AES_CCM_BLOCK_SIZE);

	return AM_HAL_STATUS_SUCCESS;
}

static uint32_t ambiq_cc312_ccm_process_aad(struct ambiq_aes_data *data,
					    am_hal_cc312_aes_ccm_context_t *ctx, const uint8_t *aad,
					    uint32_t aad_len)
{
	uint32_t first_block_rem_size;
	uint8_t a_size = 0U;
	uint8_t *temp_buff;
	uint32_t status;

	if (ctx == NULL || aad == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	temp_buff = ctx->tempBuff;
	ambiq_aes_secure_zero(temp_buff, AM_HAL_AES_CCM_BLOCK_SIZE);

	if (aad_len >= 0xFF00U) {
		temp_buff[0] = 0xFFU;
		temp_buff[1] = 0xFEU;
		am_hal_aes_ccm_reverse_memcpy(temp_buff + 2, (uint8_t *)&aad_len, 4);
		a_size = 6U;
	} else if (aad_len > 0U) {
		am_hal_aes_ccm_reverse_memcpy(temp_buff, (uint8_t *)&aad_len, 2);
		a_size = 2U;
	}

	first_block_rem_size = AM_HAL_AES_CCM_BLOCK_SIZE - a_size;

	if (aad_len > 0U) {
		uint32_t copy_size = MIN(first_block_rem_size, aad_len);

		memcpy(temp_buff + a_size, aad, copy_size);
		status = ambiq_cc312_ccm_process(data, ctx, temp_buff, NULL,
						 AM_HAL_AES_CCM_BLOCK_SIZE);
		if (status != AM_HAL_STATUS_SUCCESS) {
			return status;
		}

		if (aad_len > first_block_rem_size) {
			status = ambiq_cc312_ccm_process(data, ctx, aad + first_block_rem_size,
							 NULL, aad_len - first_block_rem_size);
			if (status != AM_HAL_STATUS_SUCCESS) {
				return status;
			}
		}
	}

	return AM_HAL_STATUS_SUCCESS;
}

static uint32_t ambiq_cc312_ccm_process_data(struct ambiq_aes_data *data,
					     am_hal_cc312_aes_ccm_context_t *ctx,
					     const uint8_t *input, uint8_t *output, uint32_t length)
{
	uintptr_t input_addr;
	uintptr_t output_addr;

	if (ctx == NULL || input == NULL || output == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	input_addr = POINTER_TO_UINT(input);
	output_addr = POINTER_TO_UINT(output);
	if ((input_addr < output_addr) && ((input_addr + length) > output_addr)) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	ctx->mode = (ctx->dir == AM_HAL_AES_CCM_DECRYPT) ? AM_HAL_CCM_MODE_CCMPD
							 : AM_HAL_CCM_MODE_CCMPE;

	return ambiq_cc312_ccm_process(data, ctx, input, output, length);
}

static uint32_t ambiq_cc312_ccm_finish(struct ambiq_aes_data *data,
				       am_hal_cc312_aes_ccm_context_t *ctx, uint8_t *tag,
				       uint32_t *tag_len)
{
	uint8_t *temp_buff;
	uint8_t q_field_size;
	uint32_t status;
	uint32_t diff = 0U;
	uint32_t i;

	if (ctx == NULL || tag == NULL || tag_len == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	q_field_size = 15U - ctx->sizeOfN;
	ctx->mode = AM_HAL_CCM_MODE_CTR;
	ambiq_aes_secure_zero((uint8_t *)ctx->ctrStateBuf + AM_HAL_AES_CCM_BLOCK_SIZE -
				      q_field_size,
			      q_field_size);

	temp_buff = ctx->tempBuff;
	ambiq_aes_secure_zero(temp_buff, sizeof(ctx->tempBuff));

	if (ctx->dir == AM_HAL_AES_CCM_ENCRYPT) {
		status = ambiq_cc312_ccm_process(data, ctx, (uint8_t *)ctx->ivBuf, temp_buff,
						 AM_HAL_AES_CCM_BLOCK_SIZE);
		if (status != AM_HAL_STATUS_SUCCESS) {
			return status;
		}

		memcpy(tag, temp_buff, ctx->sizeOfT);
	} else {
		memcpy(temp_buff, tag, ctx->sizeOfT);
		status = ambiq_cc312_ccm_process(data, ctx, temp_buff, temp_buff,
						 AM_HAL_AES_CCM_BLOCK_SIZE);
		if (status != AM_HAL_STATUS_SUCCESS) {
			return status;
		}

		for (i = 0U; i < ctx->sizeOfT; i++) {
			diff |= ((uint8_t *)ctx->ivBuf)[i] ^ temp_buff[i];
		}

		if (diff != 0U) {
			return AM_HAL_STATUS_FAIL;
		}
	}

	*tag_len = ctx->sizeOfT;
	return AM_HAL_STATUS_SUCCESS;
}

static uint32_t ambiq_cc312_ccm_auth_crypt(struct ambiq_aes_data *data,
					   am_hal_cc312_aes_ccm_context_t *ctx, const uint8_t *key,
					   uint32_t key_bits, uint32_t length, const uint8_t *iv,
					   uint32_t iv_len, const uint8_t *aad, uint32_t aad_len,
					   const uint8_t *input, uint8_t *output, uint8_t *tag,
					   uint32_t tag_len, uint8_t dir, uint32_t ccm_mode)
{
	uint32_t status;

	if (ctx == NULL || key == NULL || iv == NULL || input == NULL || output == NULL ||
	    tag == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	status = ambiq_aes_prepare_ccm_ctx(ctx, key, key_bits);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto cleanup_unlock;
	}

	status = ambiq_cc312_ccm_init(data, ctx, dir, aad_len, length, iv, (uint8_t)iv_len,
				      (uint8_t)tag_len, ccm_mode);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto cleanup;
	}

	if (aad_len != 0U) {
		status = ambiq_cc312_ccm_process_aad(data, ctx, aad, aad_len);
		if (status != AM_HAL_STATUS_SUCCESS) {
			goto cleanup;
		}
	}

	if (length != 0U) {
		status = ambiq_cc312_ccm_process_data(data, ctx, input, output, length);
		if (status != AM_HAL_STATUS_SUCCESS) {
			goto cleanup;
		}
	}

	status = ambiq_cc312_ccm_finish(data, ctx, tag, &tag_len);
	if ((status != AM_HAL_STATUS_SUCCESS) && (length != 0U)) {
		ambiq_aes_secure_zero(output, length);
	}

cleanup:
	am_hal_aes_ccm_free(ctx);
cleanup_unlock:
	k_mutex_unlock(&data->lock);
	return status;
}

static uint32_t ambiq_cc312_aes_ccm_encrypt_and_tag(
	struct ambiq_aes_data *data, am_hal_cc312_aes_ccm_context_t *ctx, const uint8_t *key,
	uint32_t key_bits, uint32_t length, const uint8_t *iv, uint32_t iv_len, const uint8_t *aad,
	uint32_t aad_len, const uint8_t *input, uint8_t *output, uint8_t *tag, uint32_t tag_len)
{
	return ambiq_cc312_ccm_auth_crypt(data, ctx, key, key_bits, length, iv, iv_len, aad,
					  aad_len, input, output, tag, tag_len,
					  AM_HAL_AES_CCM_ENCRYPT, AM_HAL_AES_CCM_MODE_CCM);
}

static uint32_t
ambiq_cc312_aes_ccm_auth_decrypt(struct ambiq_aes_data *data, am_hal_cc312_aes_ccm_context_t *ctx,
				 const uint8_t *key, uint32_t key_bits, uint32_t length,
				 const uint8_t *iv, uint32_t iv_len, const uint8_t *aad,
				 uint32_t aad_len, const uint8_t *input, uint8_t *output,
				 const uint8_t *tag, uint32_t tag_len)
{
	uint8_t local_mac_buf[AM_HAL_AES_CCM_BLOCK_SIZE] __aligned(AMBIQ_AES_DMA_ALIGNMENT);

	if (tag == NULL || tag_len > AM_HAL_AES_CCM_BLOCK_SIZE) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	memcpy(local_mac_buf, tag, tag_len);
	return ambiq_cc312_ccm_auth_crypt(data, ctx, key, key_bits, length, iv, iv_len, aad,
					  aad_len, input, output, local_mac_buf, tag_len,
					  AM_HAL_AES_CCM_DECRYPT, AM_HAL_AES_CCM_MODE_CCM);
}

static int ambiq_aes_ccm_encrypt_op(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt,
				    uint8_t *nonce)
{
	struct ambiq_aes_data *data;
	struct ambiq_aes_dma_data *dma;
	uint32_t status;
	uint8_t *out_buf;
	uint16_t tag_len;
	uint16_t nonce_len;
	size_t i;
	int ret;

	if (ctx == NULL || apkt == NULL || apkt->pkt == NULL || nonce == NULL ||
	    apkt->tag == NULL) {
		return -EINVAL;
	}

	tag_len = ctx->mode_params.ccm_info.tag_len;
	nonce_len = ctx->mode_params.ccm_info.nonce_len;

	if (tag_len < AM_HAL_AES_CCM_TAG_MIN_SIZE || tag_len > AM_HAL_AES_CCM_TAG_MAX_SIZE ||
	    ((tag_len & 1U) != 0U)) {
		return -EINVAL;
	}

	if (nonce_len < AM_HAL_AES_CCM_NONCE_MIN_SIZE ||
	    nonce_len > AM_HAL_AES_CCM_NONCE_MAX_SIZE) {
		return -EINVAL;
	}

	ret = ambiq_aes_get_output_buffer(ctx, apkt->pkt, &out_buf, (size_t)apkt->pkt->in_len);
	if (ret != 0) {
		return ret;
	}

	ret = ambiq_aes_data_from_ctx(ctx, &data);
	if (ret != 0) {
		return ret;
	}
	dma = data->dma;

	ambiq_aes_secure_zero(dma->ccm_tag_local, sizeof(dma->ccm_tag_local));

	status = ambiq_cc312_aes_ccm_encrypt_and_tag(
		data, &dma->ccm_ctx, ctx->key.bit_stream, (uint32_t)ctx->keylen * 8U,
		(uint32_t)apkt->pkt->in_len, nonce, nonce_len, apkt->ad, apkt->ad_len,
		apkt->pkt->in_buf, out_buf, dma->ccm_tag_local, tag_len);
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_status_to_errno(status);
		goto out;
	}
	for (i = 0; i < tag_len; i++) {
		apkt->tag[i] = dma->ccm_tag_local[i];
	}

	apkt->pkt->out_len = apkt->pkt->in_len + tag_len;
	ret = 0;

out:
	/* Clear DMA scratch buffers before returning */
	ambiq_aes_clear_dma_scratch(dma);
	return ret;
}

static int ambiq_aes_ccm_decrypt_op(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt,
				    uint8_t *nonce)
{
	struct ambiq_aes_data *data;
	struct ambiq_aes_dma_data *dma;
	uint32_t status;
	uint8_t *out_buf;
	uint16_t tag_len;
	uint16_t nonce_len;
	int ret;

	if (ctx == NULL || apkt == NULL || apkt->pkt == NULL || nonce == NULL ||
	    apkt->tag == NULL) {
		return -EINVAL;
	}

	tag_len = ctx->mode_params.ccm_info.tag_len;
	nonce_len = ctx->mode_params.ccm_info.nonce_len;

	if (tag_len < AM_HAL_AES_CCM_TAG_MIN_SIZE || tag_len > AM_HAL_AES_CCM_TAG_MAX_SIZE ||
	    ((tag_len & 1U) != 0U)) {
		return -EINVAL;
	}

	if (nonce_len < AM_HAL_AES_CCM_NONCE_MIN_SIZE ||
	    nonce_len > AM_HAL_AES_CCM_NONCE_MAX_SIZE) {
		return -EINVAL;
	}

	ret = ambiq_aes_get_output_buffer(ctx, apkt->pkt, &out_buf, (size_t)apkt->pkt->in_len);
	if (ret != 0) {
		return ret;
	}

	ret = ambiq_aes_data_from_ctx(ctx, &data);
	if (ret != 0) {
		return ret;
	}

	dma = data->dma;

	status = ambiq_cc312_aes_ccm_auth_decrypt(
		data, &dma->ccm_ctx, ctx->key.bit_stream, (uint32_t)ctx->keylen * 8U,
		(uint32_t)apkt->pkt->in_len, nonce, nonce_len, apkt->ad, apkt->ad_len,
		apkt->pkt->in_buf, out_buf, apkt->tag, tag_len);
	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_auth_status_to_errno(status);
		goto out;
	}

	apkt->pkt->out_len = apkt->pkt->in_len;
	ret = 0;

out:
	/* Clear DMA scratch buffers before returning */
	ambiq_aes_clear_dma_scratch(dma);
	return ret;
}

/* AES-GCM */
static uint32_t ambiq_aes_gcm_process(struct ambiq_aes_data *data, const uint8_t *input,
				      uint8_t *output, uint32_t length)
{
	am_hal_cc312_aes_gcm_context_t *ctx;
	uint32_t status;
	int ret;
	uint32_t irr_mask = CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk;

	if (data == NULL || input == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	ctx = &data->dma->gcm_ctx;

	ambiq_aes_prepare_irq_wait(data, irr_mask);

	/* Initialize hardware for this process mode */
	status = am_hal_aes_gcm_init_hw(ctx);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto process_exit;
	}

	/*
	 * HAL init masks all interrupts for hash-only GCM phases because the HAL
	 * path busy-polls IRR. The Zephyr path waits on ISR/sem, so keep only
	 * SYM_DMA_COMPLETED unmasked for every GCM phase.
	 */
	am_hal_cc312_config_dma_interrupt_mask();

	/* Configure DMA destination (only for operations with output) */
	if ((ctx->processMode == AM_HAL_AES_GCM_CALC_H) ||
	    (ctx->processMode == AM_HAL_AES_GCM_PROCESS_DATA) ||
	    (ctx->processMode == AM_HAL_AES_GCM_GCTR_FINAL)) {
		ambiq_aes_cache_clean_invalidate_region(output, (size_t)length);
		am_hal_cc312_set_dma_destination(AM_HAL_CC312_DMA_DLLI_ADDR, (uint32_t)output,
						 length);
	}

	ambiq_aes_cache_clean_invalidate_region(input, (size_t)length);

	am_hal_cc312_clear_interrupt(0xFFFFFFFFU);
	irq_enable(data->irq_num);
	/* Configure DMA source and kick operation */
	am_hal_cc312_set_dma_source(AM_HAL_CC312_DMA_DLLI_ADDR, (uint32_t)input, length);

	/* Wait for DMA completion interrupt */
	ret = k_sem_take(&data->irq_sem, K_MSEC(AMBIQ_AES_IRQ_WAIT_TIMEOUT_MS));
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

	status = AM_HAL_STATUS_SUCCESS;

	/* Post-processing: store results based on mode */
	switch (ctx->processMode) {
	case AM_HAL_AES_GCM_CALC_H:
		/* H is already in output buffer (ctx->H) */
		break;

	case AM_HAL_AES_GCM_CALC_J0_PHASE1:
		am_hal_aes_gcm_store_ghash_iv(ctx, false);
		break;

	case AM_HAL_AES_GCM_CALC_J0_PHASE2:
		am_hal_aes_gcm_store_ghash_iv(ctx, true); /* Store in J0 */
		break;

	case AM_HAL_AES_GCM_PROCESS_AAD:
		am_hal_aes_gcm_store_ghash_iv(ctx, false);
		break;

	case AM_HAL_AES_GCM_PROCESS_DATA:
		am_hal_aes_gcm_store_counter(ctx);
		am_hal_aes_gcm_store_ghash_iv(ctx, false);
		break;

	case AM_HAL_AES_GCM_PROCESS_LEN:
		am_hal_aes_gcm_store_ghash_iv(ctx, false);
		break;

	case AM_HAL_AES_GCM_GCTR_FINAL:
		/* preTagBuf already filled by DMA */
		break;

	default:
		status = AM_HAL_STATUS_INVALID_ARG;
		goto process_exit;
	}

	/*
	 * Perform DLLI-mode dummy read flush after state capture.
	 */
	if (output != NULL) {
		status = ambiq_cc312_flush_dummy_dlli(data, (uint32_t)output, length);
		if (status != AM_HAL_STATUS_SUCCESS) {
			status = AM_HAL_STATUS_TIMEOUT;
			goto process_exit;
		}

		ambiq_aes_cache_invalidate_region(output, (size_t)length);
	}

process_exit:
	ambiq_aes_finish_irq_wait(data);
	return status;
}

static uint32_t ambiq_aes_gcm_calc_h(struct ambiq_aes_data *data)
{
	am_hal_cc312_aes_gcm_context_t *ctx = &data->dma->gcm_ctx;

	ctx->processMode = AM_HAL_AES_GCM_CALC_H;
	ambiq_aes_secure_zero(ctx->tempBuf, AMBIQ_AES_BLOCK_SIZE);
	return ambiq_aes_gcm_process(data, (uint8_t *)ctx->tempBuf, (uint8_t *)ctx->H,
				     AMBIQ_AES_BLOCK_SIZE);
}

static uint32_t ambiq_aes_gcm_calc_j0(struct ambiq_aes_data *data, const uint8_t *iv)
{
	am_hal_cc312_aes_gcm_context_t *ctx = &data->dma->gcm_ctx;
	uint32_t status;

	if (ctx->ivSize == AM_HAL_AES_GCM_IV_96_BIT_SIZE) {
		memcpy(ctx->J0, iv, AM_HAL_AES_GCM_IV_96_BIT_SIZE);
		ctx->J0[3] = AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(0x00000001U);
		return AM_HAL_STATUS_SUCCESS;
	}

	ambiq_aes_secure_zero(ctx->ghashResBuf, AMBIQ_AES_BLOCK_SIZE);
	ctx->processMode = AM_HAL_AES_GCM_CALC_J0_PHASE1;
	status = ambiq_aes_gcm_process(data, iv, NULL, ctx->ivSize);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	ambiq_aes_secure_zero(ctx->tempBuf, AMBIQ_AES_BLOCK_SIZE);
	ctx->tempBuf[3] = (ctx->ivSize << 3) & AM_HAL_CC312_AES_GCM_BITMASK(32);
	ctx->tempBuf[3] = AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(ctx->tempBuf[3]);

	ctx->processMode = AM_HAL_AES_GCM_CALC_J0_PHASE2;
	return ambiq_aes_gcm_process(data, (uint8_t *)ctx->tempBuf, NULL, AMBIQ_AES_BLOCK_SIZE);
}

static uint32_t ambiq_aes_gcm_process_aad(struct ambiq_aes_data *data, const uint8_t *aad)
{
	am_hal_cc312_aes_gcm_context_t *ctx = &data->dma->gcm_ctx;

	ambiq_aes_secure_zero(ctx->ghashResBuf, AMBIQ_AES_BLOCK_SIZE);

	if (ctx->aadSize == 0U) {
		return AM_HAL_STATUS_SUCCESS;
	}

	ctx->processMode = AM_HAL_AES_GCM_PROCESS_AAD;
	return ambiq_aes_gcm_process(data, aad, NULL, ctx->aadSize);
}

static uint32_t ambiq_aes_gcm_process_cipher(struct ambiq_aes_data *data, const uint8_t *input,
					     uint8_t *output)
{
	am_hal_cc312_aes_gcm_context_t *ctx = &data->dma->gcm_ctx;

	if (ctx->dataSize == 0U) {
		return AM_HAL_STATUS_SUCCESS;
	}

	ctx->processMode = AM_HAL_AES_GCM_PROCESS_DATA;
	return ambiq_aes_gcm_process(data, input, output, ctx->dataSize);
}

static uint32_t ambiq_aes_gcm_process_len(struct ambiq_aes_data *data)
{
	am_hal_cc312_aes_gcm_context_t *ctx = &data->dma->gcm_ctx;

	ctx->tempBuf[1] = (ctx->aadSize << 3) & AM_HAL_CC312_AES_GCM_BITMASK(32);
	ctx->tempBuf[1] = AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(ctx->tempBuf[1]);
	ctx->tempBuf[0] = 0U;
	ctx->tempBuf[3] = (ctx->dataSize << 3) & AM_HAL_CC312_AES_GCM_BITMASK(32);
	ctx->tempBuf[3] = AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(ctx->tempBuf[3]);
	ctx->tempBuf[2] = 0U;

	ctx->processMode = AM_HAL_AES_GCM_PROCESS_LEN;
	return ambiq_aes_gcm_process(data, (uint8_t *)ctx->tempBuf, NULL, AMBIQ_AES_BLOCK_SIZE);
}

static uint32_t ambiq_aes_gcm_finish(struct ambiq_aes_data *data, uint8_t *tag)
{
	am_hal_cc312_aes_gcm_context_t *ctx = &data->dma->gcm_ctx;
	uint32_t status;

	ctx->processMode = AM_HAL_AES_GCM_GCTR_FINAL;
	status = ambiq_aes_gcm_process(data, (uint8_t *)ctx->tempBuf, ctx->preTagBuf,
				       AMBIQ_AES_BLOCK_SIZE);
	if (status != AM_HAL_STATUS_SUCCESS) {
		return status;
	}

	if (ctx->dir == AM_HAL_AES_GCM_ENCRYPT) {
		memcpy(tag, ctx->preTagBuf, ctx->tagSize);
	} else if (ambiq_aes_auth_diff(ctx->preTagBuf, tag, ctx->tagSize) != 0U) {
		return AM_HAL_STATUS_FAIL;
	}

	return AM_HAL_STATUS_SUCCESS;
}

static uint32_t ambiq_aes_gcm_crypt_and_tag(struct ambiq_aes_data *data, int mode,
					    const uint8_t *key, uint32_t key_bits, uint32_t length,
					    const uint8_t *iv, uint32_t iv_len, const uint8_t *aad,
					    uint32_t aad_len, const uint8_t *input, uint8_t *output,
					    uint32_t tag_len, uint8_t *tag)
{
	am_hal_cc312_aes_gcm_context_t *ctx;
	uint32_t status;

	if (data == NULL) {
		return AM_HAL_STATUS_INVALID_ARG;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ctx = &data->dma->gcm_ctx;
	status = ambiq_aes_prepare_gcm_ctx(ctx, key, key_bits);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto cleanup_unlock;
	}

	ctx->dir = mode;
	ctx->dataSize = length;
	ctx->ivSize = iv_len;
	ctx->aadSize = aad_len;
	ctx->tagSize = tag_len;
	ctx->j0Inc32Done = false;

	am_hal_cc312_aes_enable_clocks();
	am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_HASH);

	status = ambiq_aes_gcm_calc_h(data);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto cleanup;
	}

	status = ambiq_aes_gcm_calc_j0(data, iv);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto cleanup;
	}

	status = ambiq_aes_gcm_process_aad(data, aad);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto cleanup;
	}

	status = ambiq_aes_gcm_process_cipher(data, input, output);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto cleanup;
	}

	status = ambiq_aes_gcm_process_len(data);
	if (status != AM_HAL_STATUS_SUCCESS) {
		goto cleanup;
	}

	status = ambiq_aes_gcm_finish(data, tag);

cleanup:
	if ((mode == AM_HAL_AES_GCM_DECRYPT) && (status != AM_HAL_STATUS_SUCCESS) &&
	    (output != NULL) && (length != 0U)) {
		ambiq_aes_secure_zero(output, length);
	}

	am_hal_aes_gcm_free(ctx);
	am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_HASH);
	am_hal_cc312_aes_disable_clocks();
cleanup_unlock:
	k_mutex_unlock(&data->lock);

	return status;
}

static int ambiq_aes_gcm_encrypt_op(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt,
				    uint8_t *nonce)
{
	struct ambiq_aes_data *data;
	uint32_t status;
	uint8_t *out_buf;
	uint16_t tag_len;
	uint16_t nonce_len;
	int ret;

	if (ctx == NULL || apkt == NULL || apkt->pkt == NULL || nonce == NULL ||
	    apkt->tag == NULL) {
		return -EINVAL;
	}

	tag_len = ctx->mode_params.gcm_info.tag_len;
	nonce_len = ctx->mode_params.gcm_info.nonce_len;

	ret = ambiq_aes_get_output_buffer(ctx, apkt->pkt, &out_buf, (size_t)apkt->pkt->in_len);
	if (ret != 0) {
		return ret;
	}
	if (nonce_len == 0U || nonce_len > AMBIQ_AES_BLOCK_SIZE || tag_len == 0U ||
	    tag_len > AMBIQ_AES_BLOCK_SIZE) {
		return -EINVAL;
	}

	ret = ambiq_aes_data_from_ctx(ctx, &data);
	if (ret != 0) {
		return ret;
	}

	status = ambiq_aes_gcm_crypt_and_tag(
		data, AM_HAL_AES_GCM_ENCRYPT, ctx->key.bit_stream, (uint32_t)ctx->keylen * 8U,
		(uint32_t)apkt->pkt->in_len, nonce, nonce_len, apkt->ad, apkt->ad_len,
		apkt->pkt->in_buf, out_buf, tag_len, apkt->tag);

	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_status_to_errno(status);
		goto out;
	}

	apkt->pkt->out_len = apkt->pkt->in_len + tag_len;
	ret = 0;

out:
	/* Clear DMA scratch buffers before returning */
	ambiq_aes_clear_dma_scratch(data->dma);
	return ret;
}

static int ambiq_aes_gcm_decrypt_op(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt,
				    uint8_t *nonce)
{
	struct ambiq_aes_data *data;
	uint32_t status;
	uint8_t *out_buf;
	uint16_t tag_len;
	uint16_t nonce_len;
	int ret;

	if (ctx == NULL || apkt == NULL || apkt->pkt == NULL || nonce == NULL ||
	    apkt->tag == NULL) {
		return -EINVAL;
	}

	tag_len = ctx->mode_params.gcm_info.tag_len;
	nonce_len = ctx->mode_params.gcm_info.nonce_len;

	ret = ambiq_aes_get_output_buffer(ctx, apkt->pkt, &out_buf, (size_t)apkt->pkt->in_len);
	if (ret != 0) {
		return ret;
	}
	if (nonce_len == 0U || nonce_len > AMBIQ_AES_BLOCK_SIZE || tag_len == 0U ||
	    tag_len > AMBIQ_AES_BLOCK_SIZE) {
		return -EINVAL;
	}

	ret = ambiq_aes_data_from_ctx(ctx, &data);
	if (ret != 0) {
		return ret;
	}

	status = ambiq_aes_gcm_crypt_and_tag(
		data, AM_HAL_AES_GCM_DECRYPT, ctx->key.bit_stream, (uint32_t)ctx->keylen * 8U,
		(uint32_t)apkt->pkt->in_len, nonce, nonce_len, apkt->ad, apkt->ad_len,
		apkt->pkt->in_buf, out_buf, tag_len, apkt->tag);

	if (status != AM_HAL_STATUS_SUCCESS) {
		ret = ambiq_hal_auth_status_to_errno(status);
		goto out;
	}

	apkt->pkt->out_len = apkt->pkt->in_len;
	ret = 0;

out:
	/* Clear DMA scratch buffers before returning */
	ambiq_aes_clear_dma_scratch(data->dma);
	return ret;
}

static int ambiq_aes_set_session_ops(struct cipher_ctx *ctx, enum cipher_mode mode,
				     enum cipher_op op_type)
{
	switch (mode) {
	case CRYPTO_CIPHER_MODE_ECB:
		ctx->ops.block_crypt_hndlr = (op_type == CRYPTO_CIPHER_OP_ENCRYPT)
						     ? ambiq_aes_ecb_encrypt_op
						     : ambiq_aes_ecb_decrypt_op;
		return 0;
	case CRYPTO_CIPHER_MODE_CTR:
		ctx->ops.ctr_crypt_hndlr = ambiq_aes_ctr_op;
		return 0;
	case CRYPTO_CIPHER_MODE_OFB:
		ctx->ops.ofb_crypt_hndlr = ambiq_aes_ofb_op;
		return 0;
	case CRYPTO_CIPHER_MODE_CCM:
		ctx->ops.ccm_crypt_hndlr = (op_type == CRYPTO_CIPHER_OP_ENCRYPT)
						   ? ambiq_aes_ccm_encrypt_op
						   : ambiq_aes_ccm_decrypt_op;
		return 0;
	case CRYPTO_CIPHER_MODE_GCM:
		ctx->ops.gcm_crypt_hndlr = (op_type == CRYPTO_CIPHER_OP_ENCRYPT)
						   ? ambiq_aes_gcm_encrypt_op
						   : ambiq_aes_gcm_decrypt_op;
		return 0;
	case CRYPTO_CIPHER_MODE_XTS:
		ctx->ops.xts_crypt_hndlr = (op_type == CRYPTO_CIPHER_OP_ENCRYPT)
						   ? ambiq_aes_xts_encrypt_op
						   : ambiq_aes_xts_decrypt_op;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static void ambiq_aes_clear_session_ops(struct cipher_ctx *ctx)
{
	ctx->ops.block_crypt_hndlr = NULL;
	ctx->ops.ctr_crypt_hndlr = NULL;
	ctx->ops.ccm_crypt_hndlr = NULL;
	ctx->ops.gcm_crypt_hndlr = NULL;
	ctx->ops.xts_crypt_hndlr = NULL;
	ctx->ops.ofb_crypt_hndlr = NULL;
}

static int ambiq_aes_validate_session(const struct cipher_ctx *ctx, enum cipher_algo algo,
				      enum cipher_mode mode)
{
	if (ctx->flags & ~AMBIQ_AES_CAPS) {
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

	if (algo != CRYPTO_CIPHER_ALGO_AES) {
		return -ENOTSUP;
	}

	if (!(mode == CRYPTO_CIPHER_MODE_ECB || mode == CRYPTO_CIPHER_MODE_CTR ||
	      mode == CRYPTO_CIPHER_MODE_OFB || mode == CRYPTO_CIPHER_MODE_CCM ||
	      mode == CRYPTO_CIPHER_MODE_GCM || mode == CRYPTO_CIPHER_MODE_XTS)) {
		return -ENOTSUP;
	}

	if (mode == CRYPTO_CIPHER_MODE_XTS) {
		if (!(ctx->keylen == 32 || ctx->keylen == 64)) {
			return -EINVAL;
		}
	} else if (!(ctx->keylen == 16 || ctx->keylen == 24 || ctx->keylen == 32)) {
		return -EINVAL;
	}

	return 0;
}

static int ambiq_aes_begin_session(const struct device *dev, struct cipher_ctx *ctx,
				   enum cipher_algo algo, enum cipher_mode mode,
				   enum cipher_op op_type)
{
	struct ambiq_aes_data *data;
	int ret;

	if (dev == NULL || ctx == NULL || dev->data == NULL) {
		return -EINVAL;
	}

	ret = ambiq_aes_validate_session(ctx, algo, mode);
	if (ret != 0) {
		return ret;
	}

	ret = pm_device_runtime_get(dev);
	if (ret != 0) {
		return ret;
	}

	data = dev->data;
	k_mutex_lock(&data->lock, K_FOREVER);
	ambiq_aes_clear_session_ops(ctx);
	ret = ambiq_aes_set_session_ops(ctx, mode, op_type);
	if (ret != 0) {
		k_mutex_unlock(&data->lock);
		(void)pm_device_runtime_put(dev);
		return ret;
	}

	ctx->drv_sessn_state = NULL;
	ctx->ops.cipher_mode = mode;
	k_mutex_unlock(&data->lock);
	return 0;
}

static int ambiq_aes_free_session(const struct device *dev, struct cipher_ctx *ctx)
{
	struct ambiq_aes_data *data;

	if (dev == NULL || ctx == NULL || dev->data == NULL) {
		return -EINVAL;
	}

	data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	ambiq_aes_clear_session_ops(ctx);
	ctx->drv_sessn_state = NULL;
	k_mutex_unlock(&data->lock);

	(void)pm_device_runtime_put(dev);

	return 0;
}

static int ambiq_aes_callback_set(const struct device *dev, cipher_completion_cb cb)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	return -ENOTSUP;
}

static int ambiq_aes_pm_action(const struct device *dev, enum pm_device_action action)
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

static int ambiq_aes_init(const struct device *dev)
{
	const struct ambiq_aes_config *cfg = dev->config;
	struct ambiq_aes_data *data = dev->data;

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

static DEVICE_API(crypto, ambiq_aes_crypto_api) = {
	.query_hw_caps = ambiq_aes_query_hw_caps,
	.cipher_begin_session = ambiq_aes_begin_session,
	.cipher_free_session = ambiq_aes_free_session,
	.cipher_async_callback_set = ambiq_aes_callback_set,
};

#define AMBIQ_AES_IRQ_CONFIG(inst)                                                                 \
	static void ambiq_aes_irq_config_##inst(void)                                              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), ambiq_cc312_isr,      \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
	}

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_AES_IRQ_CONFIG)

#define AMBIQ_AES_DEVICE_DEFINE(inst)                                                              \
	static AMBIQ_AES_NOCACHE struct ambiq_aes_dma_data ambiq_aes_dma_##inst;                   \
	static struct ambiq_aes_data ambiq_aes_data_##inst;                                        \
	static const struct ambiq_aes_config ambiq_aes_cfg_##inst = {                              \
		.irq_num = DT_INST_IRQN(inst),                                                     \
		.irq_config_func = ambiq_aes_irq_config_##inst,                                    \
		.dma = &ambiq_aes_dma_##inst,                                                      \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(inst, ambiq_aes_pm_action);                                       \
	DEVICE_DT_INST_DEFINE(inst, ambiq_aes_init, PM_DEVICE_DT_INST_GET(inst),                   \
			      &ambiq_aes_data_##inst, &ambiq_aes_cfg_##inst, POST_KERNEL,          \
			      CONFIG_CRYPTO_INIT_PRIORITY, &ambiq_aes_crypto_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_AES_DEVICE_DEFINE)
