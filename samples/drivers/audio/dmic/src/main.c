/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dmic_sample);

#if defined(CONFIG_USE_SEGGER_RTT)
#include <SEGGER_RTT.h>
#endif

#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH CONFIG_SAMPLE_BIT_WIDTH
#define BYTES_PER_SAMPLE (SAMPLE_BIT_WIDTH == 24 ? (4) : (SAMPLE_BIT_WIDTH / 8))
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT      4
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 32);

#if defined(CONFIG_USE_SEGGER_RTT)
static int16_t temp_buffer[MAX_SAMPLE_RATE]           __attribute__ ((aligned (32))) __attribute__ ((section (".dtcm_data")));
static int16_t rtt_recorder_buffer[MAX_SAMPLE_RATE*5] __attribute__ ((aligned (32))) __attribute__ ((section (".dtcm_data")));
#endif

static int do_pdm_transfer(const struct device *dmic_dev,
			   struct dmic_cfg *cfg,
			   size_t block_count)
{
	int ret;

	LOG_INF("PCM output rate: %u, channels: %u",
		cfg->streams[0].pcm_rate, cfg->channel.req_num_chan);

	ret = dmic_configure(dmic_dev, cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure the driver: %d", ret);
		return ret;
	}

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("START trigger failed: %d", ret);
		return ret;
	}

	for (int i = 0; i < block_count; ++i) {
		void *buffer;
		uint32_t size;

		ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("%d - read failed: %d", i, ret);
			return ret;
		}

#if defined(CONFIG_USE_SEGGER_RTT)
		/* Dump two channels data in 16-bit format */
		uint32_t num_ch = cfg->channel.act_num_chan;
		uint32_t num_sample = size / (num_ch * BYTES_PER_SAMPLE);

		for (uint32_t j = 0; j < num_sample; j++) {
			temp_buffer[2 * j + 0] = (((uint32_t*)buffer)[num_ch * j + 0]  & 0x00FFFF00) >> 8;
			temp_buffer[2 * j + 1] = (((uint32_t*)buffer)[num_ch * j + 1]  & 0x00FFFF00) >> 8;
		}
		SEGGER_RTT_Write(1, temp_buffer, num_sample * sizeof(int16_t) * 2);
#endif

		LOG_INF("%d - got buffer %p of %u bytes", i, buffer, size);

		k_mem_slab_free(&mem_slab, buffer);
	}

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("STOP trigger failed: %d", ret);
		return ret;
	}

	return ret;
}

int main(void)
{
	#if defined(CONFIG_USE_SEGGER_RTT)
	SEGGER_RTT_Init();
	SEGGER_RTT_ConfigUpBuffer(1, "DataLogger", rtt_recorder_buffer,
			sizeof(rtt_recorder_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	/* To dump PCM data with RTT, run the following command in the terminal:
	   JLinkRTTLogger -Device 'AP510NFA-CBR' -If SWD -Usb xxxxxx -Speed 4000 \
	   		-RTTAddress 0xxxxxxxxx -RTTChannel 1 dmic_2ch_dump.raw
           where '-Usb' is followed by the J-Link serial number and 'RTTAddress'
	   is printed in the line shown below.
	*/
	LOG_INF("-RTTAddress: %p", &_SEGGER_RTT);
	#endif

	const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	int ret;

	LOG_INF("DMIC sample");

	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("%s is not ready", dmic_dev->name);
		return 0;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	cfg.channel.req_num_chan = 2;
	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
		dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size =
		BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	ret = do_pdm_transfer(dmic_dev, &cfg, 40 * BLOCK_COUNT);
	if (ret < 0) {
		return 0;
	}

	LOG_INF("Exiting");
	return 0;
}
