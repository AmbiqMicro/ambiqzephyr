/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ambiq_i2s_loopback, LOG_LEVEL_INF);

#define I2S_TX DT_ALIAS(i2s_tx)
#define I2S_RX DT_ALIAS(i2s_rx)

#define SAMPLE_BIT_WIDTH   24U
#define NUMBER_OF_CHANNELS 2U
#define SAMPLE_FREQ_HZ     16000U
#define SAMPLE_BLOCKS_MS   10U
#define TIMEOUT_MS         1000U

#define BYTES_PER_SAMPLE sizeof(int32_t)
#define SAMPLES_PER_BLOCK                                                                          \
	((SAMPLE_FREQ_HZ * SAMPLE_BLOCKS_MS / 1000U) * NUMBER_OF_CHANNELS)
#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT 4U

/*
 * Keep the verification window modest to avoid a large-buffer fault seen with
 * much larger windows on Apollo510B.
 */
#define LOOPBACK_TEST_BLOCKS 20U

K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

static uint8_t rx_temp_buf[BLOCK_SIZE * LOOPBACK_TEST_BLOCKS];

static void fill_tx_pattern(void *mem_block)
{
	int32_t *buf = mem_block;

	/*
	 * Use a full in-block index in the low bits so the sync word is unique
	 * within each block. (i & 0xFF) collides at i==0 and i==256 when the
	 * block has 320 samples.
	 */
	for (int i = 0; i < (int)SAMPLES_PER_BLOCK; i++) {
		buf[i] = 0xC00000 | i;
	}
}

static bool check_i2s_data(uint32_t rxtx_sample_num, void *rx_databuf)
{
	uint32_t *rx = rx_databuf;
	int index_0 = -1;
	int i;

	/* Require a two-word sync so a single repeated value cannot false-lock. */
	for (i = 0; i + 1 < (int)rxtx_sample_num; i++) {
		if ((rx[i] == 0xC00000U) && (rx[i + 1] == 0xC00001U)) {
			index_0 = i;
			break;
		}
	}

	if (index_0 < 0) {
		LOG_ERR("sync word not found");
		return false;
	}

	for (i = 0; i < (int)(rxtx_sample_num - index_0); i++) {
		int idx = i + index_0;
		uint32_t expected = 0xC00000U | (i % (int)SAMPLES_PER_BLOCK);

		if (rx[idx] != expected) {
			LOG_ERR("mismatch idx=%d offset=%d got=0x%x expected=0x%x", index_0, i,
				rx[idx], expected);
			return false;
		}
	}

	return true;
}

int main(void)
{
	const struct device *const i2s_tx = DEVICE_DT_GET(I2S_TX);
	const struct device *const i2s_rx = DEVICE_DT_GET(I2S_RX);
	struct i2s_config cfg;
	uint32_t rx_buf_index = 0;
	bool started = false;
	int tx_count = 0;
	int ret;

	if (!device_is_ready(i2s_tx) || !device_is_ready(i2s_rx)) {
		LOG_ERR("I2S device not ready");
		return 0;
	}

	LOG_INF("Ambiq I2S loopback sample");

	cfg.word_size = SAMPLE_BIT_WIDTH;
	cfg.channels = NUMBER_OF_CHANNELS;
	cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	cfg.frame_clk_freq = SAMPLE_FREQ_HZ;
	cfg.mem_slab = &mem_slab;
	cfg.block_size = BLOCK_SIZE;
	cfg.timeout = TIMEOUT_MS;

	LOG_INF("rate=%u Hz channels=%u block=%u", cfg.frame_clk_freq, cfg.channels,
		cfg.block_size);

	ret = i2s_configure(i2s_tx, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		LOG_ERR("TX configure failed: %d", ret);
		return 0;
	}

	cfg.options = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE;
	ret = i2s_configure(i2s_rx, I2S_DIR_RX, &cfg);
	if (ret < 0) {
		LOG_ERR("RX configure failed: %d", ret);
		return 0;
	}

	LOG_INF("Start streams");

	while (tx_count <= 1000) {
		void *mem_block;
		void *rx_buffer;
		size_t rx_size = 0;

		ret = k_mem_slab_alloc(&mem_slab, &mem_block, Z_TIMEOUT_TICKS(TIMEOUT_MS));
		if (ret < 0) {
			LOG_ERR("TX alloc failed: %d", ret);
			break;
		}

		fill_tx_pattern(mem_block);
		ret = i2s_write(i2s_tx, mem_block, BLOCK_SIZE);
		if (ret < 0) {
			LOG_ERR("TX write failed: %d", ret);
			k_mem_slab_free(&mem_slab, mem_block);
			break;
		}

		if (!started) {
			ret = i2s_trigger(i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
			if (ret < 0) {
				LOG_ERR("RX START failed: %d", ret);
				break;
			}

			ret = i2s_trigger(i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
			if (ret < 0) {
				LOG_ERR("TX START failed: %d", ret);
				break;
			}

			started = true;
			/*
			 * Queue another TX block before blocking on RX to avoid
			 * TX underrun right after START.
			 */
			tx_count++;
			continue;
		}

		ret = i2s_read(i2s_rx, &rx_buffer, &rx_size);
		if (ret < 0) {
			LOG_ERR("RX read failed: %d", ret);
			break;
		}

		if (rx_size != BLOCK_SIZE) {
			LOG_ERR("unexpected RX size %u (expected %u)", (uint32_t)rx_size,
				(uint32_t)BLOCK_SIZE);
			k_mem_slab_free(&mem_slab, rx_buffer);
			break;
		}

		if ((rx_buf_index + rx_size) > sizeof(rx_temp_buf)) {
			LOG_ERR("verify buffer overflow");
			k_mem_slab_free(&mem_slab, rx_buffer);
			break;
		}

		memcpy(&rx_temp_buf[rx_buf_index], rx_buffer, rx_size);
		rx_buf_index += rx_size;
		k_mem_slab_free(&mem_slab, rx_buffer);

		if (rx_buf_index >= (BLOCK_SIZE * LOOPBACK_TEST_BLOCKS)) {
			if (check_i2s_data(SAMPLES_PER_BLOCK * LOOPBACK_TEST_BLOCKS, rx_temp_buf)) {
				LOG_INF("%u bytes passed", BLOCK_SIZE * LOOPBACK_TEST_BLOCKS);
			} else {
				LOG_ERR("data check failed");
			}
			rx_buf_index = 0;
		}

		tx_count++;
	}

	ret = i2s_trigger(i2s_tx, I2S_DIR_TX, I2S_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("TX STOP failed: %d", ret);
	}

	ret = i2s_trigger(i2s_rx, I2S_DIR_RX, I2S_TRIGGER_DROP);
	if (ret < 0) {
		LOG_ERR("RX DROP failed: %d", ret);
	}

	LOG_INF("Streams stopped");
	return 0;
}
