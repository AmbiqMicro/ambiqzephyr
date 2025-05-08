/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>

#include <zephyr/logging/log.h>

#include <arm_math.h>

LOG_MODULE_REGISTER(dmic_sample);

#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

/* Size of a block for 10 ms of audio data. */
//#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
//	(BYTES_PER_SAMPLE * (_sample_rate / 100) * _number_of_channels)
/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT      4
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

//#define PDM_FFT_SIZE (MAX_BLOCK_SIZE * 2)
#define PDM_FFT_SIZE (1024)
//
// Buffers used for FFT.
//
float g_fPDMTimeDomain[PDM_FFT_SIZE * 2];
float g_fPDMMagnitudes[PDM_FFT_SIZE * 2];

int16_t g_i16PcmBuf[PDM_FFT_SIZE * 2];

#define PRINT_PDM_DATA 0
#define PRINT_FFT_DATA 0

//*****************************************************************************
//
// Analyze and print frequency data.
//
//*****************************************************************************
void
pcm_fft_print(int16_t *pi16PDMData)
{
    float fMaxValue;
    uint32_t ui32MaxIndex;
    uint32_t ui32LoudestFrequency;
    uint32_t ui32SampleFreq = 24000000 / (2 * 8 * 2 * 48);      // 15,625 Hz

    //
    // Convert the PDM samples to floats, and arrange them in the format
    // required by the FFT function.
    //
    for (uint32_t i = 0; i < PDM_FFT_SIZE; i++)
    {
        if (PRINT_PDM_DATA)
        {
            LOG_INF("%d\n", pi16PDMData[i]);
        }

        g_fPDMTimeDomain[2 * i] = (pi16PDMData[i] / 2.0);
//        g_fPDMTimeDomain[2 * i] = (pi16PDMData[i] << 8) / 65536.0;
        g_fPDMTimeDomain[2 * i + 1] = 0.0;
    }

    if (PRINT_PDM_DATA)
    {
        LOG_INF("END\n");
    }

    //
    // Perform the FFT.
    //
    arm_cfft_radix4_instance_f32 S;
    arm_cfft_radix4_init_f32(&S, PDM_FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&S, g_fPDMTimeDomain);
    arm_cmplx_mag_f32(g_fPDMTimeDomain, g_fPDMMagnitudes, PDM_FFT_SIZE);

    if (PRINT_FFT_DATA)
    {
        for (uint32_t i = 0; i < PDM_FFT_SIZE / 2; i++)
        {
            LOG_INF("%f\n", g_fPDMMagnitudes[i]);
        }

        LOG_INF("END\n");
    }

    //
    // Find the frequency bin with the largest magnitude.
    //
    arm_max_f32(g_fPDMMagnitudes, PDM_FFT_SIZE / 2, &fMaxValue, &ui32MaxIndex);

    ui32LoudestFrequency = (ui32SampleFreq * ui32MaxIndex) / PDM_FFT_SIZE;

    if (PRINT_FFT_DATA)
    {
        LOG_INF("Loudest frequency bin: %d\n", ui32MaxIndex);
    }
    LOG_INF("Loudest frequency: %d\n", ui32LoudestFrequency);
}

static int do_pdm_transfer(const struct device *dmic_dev,
			   struct dmic_cfg *cfg,
			   size_t block_count)
{
	int ret;
    uint8_t *pBuf = (uint8_t *)g_i16PcmBuf;
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

//	for (int i = 0; i < block_count; ++i) {
	while (1) {
		void *buffer;
		uint32_t size;

		ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
		// if (ret < 0) {
		// 	LOG_ERR("%d - read failed: %d", i, ret);
		// 	return ret;
		// }

		//LOG_INF("%d - got buffer %p of %u bytes", i, buffer, size);
        pcm_fft_print(buffer);
        //memcpy(pBuf, buffer, size);
        //pBuf += size;
		k_mem_slab_free(&mem_slab, buffer);
	}

    LOG_INF("Pass");

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("STOP trigger failed: %d", ret);
		return ret;
	}

    //pcm_fft_print(g_i16PcmBuf);

	return ret;
}

int main(void)
{
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

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size =
		BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	ret = do_pdm_transfer(dmic_dev, &cfg, 2 * BLOCK_COUNT);
	if (ret < 0) {
		return 0;
	}

	cfg.channel.req_num_chan = 2;
	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
		dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size =
		BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	ret = do_pdm_transfer(dmic_dev, &cfg, 2 * BLOCK_COUNT);
	if (ret < 0) {
		return 0;
	}

	LOG_INF("Exiting");
	return 0;
}
