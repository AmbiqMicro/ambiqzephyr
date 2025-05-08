/*
 * Copyright (c) 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/amic.h>

#include <zephyr/logging/log.h>

#include "SensoryLib.h"
#include "am_vos_thf.h"

LOG_MODULE_REGISTER(amic_sample);

#define SAMPLE_RATE  16000
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

/* AUDADC is 1 channel supported only */
#define NUMBER_OF_CHANNELS	1
#define SAMPLE_BIT_WIDTH (16 * NUMBER_OF_CHANNELS)

/* Size of a block for 15 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 200) * 3 * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define BLOCK_SIZE_BYTES   BLOCK_SIZE(SAMPLE_RATE, NUMBER_OF_CHANNELS)
#define BLOCK_COUNT      4
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE_BYTES, BLOCK_COUNT, 4);

static int do_audadc_transfer(const struct device *amic_dev,
			   struct amic_cfg *cfg,
			   size_t block_count)
{
	int ret;

	LOG_INF("PCM output rate: %u, channels: %u",
		cfg->streams[0].pcm_rate, 1);

	ret = amic_configure(amic_dev, cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure the driver: %d", ret);
		return ret;
	}

	ret = amic_trigger(amic_dev, AMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("START trigger failed: %d", ret);
		return ret;
	}

	while (1) {
		void *buffer;
		uint32_t size;
        uint8_t detected_flag = 0;

		ret = amic_read(amic_dev, 0, &buffer, &size, READ_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Read failed: %d", ret);
			return ret;
		}

        if(detected_flag == 0)
        {
            am_vos_engine_process(buffer, size, &detected_flag);
        }
        else
        {
            detected_flag++;
            if(detected_flag > 100)
            {
                detected_flag = 0;
            }
        }

		k_mem_slab_free(&mem_slab, buffer);
	}

	ret = amic_trigger(amic_dev, AMIC_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("STOP trigger failed: %d", ret);
		return ret;
	}

	return ret;
}

int main(void)
{
	const struct device *const amic_dev = DEVICE_DT_GET(DT_NODELABEL(amic_dev));
	int ret;

	LOG_INF("AMIC THF sample");

	if (!device_is_ready(amic_dev)) {
		LOG_ERR("%s is not ready", amic_dev->name);
		return 0;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};
	struct amic_cfg cfg = {
		.streams = &stream,
	};

    am_vos_engine_init();

	cfg.streams[0].pcm_rate = SAMPLE_RATE;
	cfg.streams[0].block_size = BLOCK_SIZE_BYTES;

	ret = do_audadc_transfer(amic_dev, &cfg, 2 * BLOCK_COUNT);
	if (ret < 0) {
		return 0;
	}

	LOG_INF("Exiting");
	return 0;
}
