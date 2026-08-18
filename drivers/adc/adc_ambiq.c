/*
 * Copyright (c) 2024 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_adc

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>

#ifdef CONFIG_ADC_AMBIQ_STREAM
#include <zephyr/rtio/rtio.h>
#include <zephyr/rtio/work.h>
#endif /* CONFIG_ADC_AMBIQ_STREAM */

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

/* ambiq-sdk includes */
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_ambiq, CONFIG_ADC_LOG_LEVEL);

#if defined(CONFIG_ADC_AMBIQ_DMA) && defined(CONFIG_ADC_ASYNC)
#error "ADC DMA mode does not support async mode."
#endif

/* Number of slots available. */
#define AMBIQ_ADC_SLOT_NUMBER AM_HAL_ADC_MAX_SLOTS

#ifdef CONFIG_ADC_AMBIQ_STREAM
/* Maximum number of channels that can be encoded in a single stream frame. */
#define AMBIQ_ADC_STREAM_MAX_CHANNELS AMBIQ_ADC_SLOT_NUMBER

/*
 * Encoded header prepended to every RTIO frame produced by the driver (both the
 * one-shot and the streaming paths). It is self-describing so that the decoder
 * can convert the raw samples that immediately follow it without any external
 * state.
 */
struct adc_ambiq_stream_header {
	uint64_t timestamp_ns;
	uint16_t vref_mv;
	uint16_t frame_count; /* number of channel scans stored in the payload */
	uint8_t num_channels; /* number of samples per scan */
	uint8_t resolution;
	/* channel id for each slot, in the order samples are stored */
	uint8_t channel_ids[AMBIQ_ADC_STREAM_MAX_CHANNELS];
	/* payload: frame_count * num_channels little-endian uint16_t samples */
	uint16_t samples[];
} __packed;
#endif /* CONFIG_ADC_AMBIQ_STREAM */

#ifdef CONFIG_ADC_AMBIQ_DMA
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
#define AMBIQ_ADC_DMA_INT (AM_HAL_ADC_INT_DERR | AM_HAL_ADC_INT_DCMP)
#else
#define AMBIQ_ADC_DMA_INT (AM_HAL_ADC_INT_DERR | AM_HAL_ADC_INT_DCMP | AM_HAL_ADC_INT_FIFOOVR1)
#endif
#endif

struct adc_ambiq_config {
	uint32_t base;
	int size;
	uint8_t num_channels;
	void (*irq_config_func)(void);
	const struct pinctrl_dev_config *pin_cfg;
};

struct adc_ambiq_data {
	struct adc_context ctx;
	void *adcHandle;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t active_channels;
#ifdef CONFIG_ADC_AMBIQ_DMA
	am_hal_adc_dma_config_t dma_cfg;
	am_hal_adc_sample_t *sample_buf;
	bool dma_mode; /* Device tree configuration: DMA enabled */
	bool use_dma;  /* Runtime decision: actually use DMA for this sequence */
#endif
#ifdef CONFIG_ADC_AMBIQ_STREAM
	struct rtio_iodev_sqe *sqe;
	uint8_t stream_channel_ids[AMBIQ_ADC_STREAM_MAX_CHANNELS];
	uint8_t stream_num_channels;
	uint8_t stream_resolution;
	uint16_t stream_vref_mv;
	bool streaming;
#endif
	const struct device *dev;
};

#ifdef CONFIG_ADC_AMBIQ_STREAM
static void adc_ambiq_stream_isr(const struct device *dev, uint32_t int_mask);
#endif

static int adc_ambiq_set_resolution(am_hal_adc_slot_prec_e *prec, uint8_t adc_resolution)
{
	switch (adc_resolution) {
	case 8:
		*prec = AM_HAL_ADC_SLOT_8BIT;
		break;
	case 10:
		*prec = AM_HAL_ADC_SLOT_10BIT;
		break;
	case 12:
		*prec = AM_HAL_ADC_SLOT_12BIT;
		break;
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	case 14:
		*prec = AM_HAL_ADC_SLOT_14BIT;
		break;
#endif
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int adc_ambiq_config(const struct device *dev)
{
	struct adc_ambiq_data *data = dev->data;
	am_hal_adc_config_t ADCConfig;

	/* Set up the ADC configuration parameters. These settings are reasonable
	 *  for accurate measurements at a low sample rate.
	 */
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	ADCConfig.eClock = AM_HAL_ADC_CLKSEL_HFRC;
	ADCConfig.eReference = AM_HAL_ADC_REFSEL_INT_1P5;
#else
	ADCConfig.eClock = AM_HAL_ADC_CLKSEL_HFRC_24MHZ;
	ADCConfig.eRepeatTrigger = AM_HAL_ADC_RPTTRIGSEL_INT;
#endif
	ADCConfig.ePolarity = AM_HAL_ADC_TRIGPOL_RISING;
	ADCConfig.eTrigger = AM_HAL_ADC_TRIGSEL_SOFTWARE;
	ADCConfig.eClockMode = AM_HAL_ADC_CLKMODE_LOW_LATENCY;
	ADCConfig.ePowerMode = AM_HAL_ADC_LPMODE0;
#ifdef CONFIG_ADC_AMBIQ_DMA
	if (data->use_dma) {
		ADCConfig.eRepeat = AM_HAL_ADC_REPEATING_SCAN;
	} else {
		ADCConfig.eRepeat = AM_HAL_ADC_SINGLE_SCAN;
	}
#else
	ADCConfig.eRepeat = AM_HAL_ADC_SINGLE_SCAN;
#endif
	if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure(data->adcHandle, &ADCConfig)) {
		LOG_ERR("configuring ADC failed.\n");
		return -ENODEV;
	}

	return 0;
}

static int adc_ambiq_slot_config(const struct device *dev, const struct adc_sequence *sequence,
				 am_hal_adc_slot_chan_e channel, uint32_t ui32SlotNumber)
{
	struct adc_ambiq_data *data = dev->data;
	am_hal_adc_slot_config_t ADCSlotConfig;

	if (adc_ambiq_set_resolution(&ADCSlotConfig.ePrecisionMode, sequence->resolution) != 0) {
		LOG_ERR("unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}

	/* Set up an ADC slot */
	ADCSlotConfig.eMeasToAvg = AM_HAL_ADC_SLOT_AVG_1;
	ADCSlotConfig.eChannel = channel;
	ADCSlotConfig.bWindowCompare = false;
	ADCSlotConfig.bEnabled = true;
#if !defined(CONFIG_SOC_SERIES_APOLLO3X)
	ADCSlotConfig.ui32TrkCyc = AM_HAL_ADC_MIN_TRKCYC;
#endif
	if (AM_HAL_STATUS_SUCCESS !=
	    am_hal_adc_configure_slot(data->adcHandle, ui32SlotNumber, &ADCSlotConfig)) {
		LOG_ERR("configuring ADC Slot 0 failed.\n");
		return -ENODEV;
	}

	return 0;
}

static void adc_ambiq_stop_sampling(const struct device *dev)
{
#ifdef CONFIG_ADC_AMBIQ_DMA
	struct adc_ambiq_data *data = dev->data;

	if (data->use_dma) {
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
		/* Stop the ADC repetitive sample timer A3 */
		am_hal_ctimer_stop(3, AM_HAL_CTIMER_TIMERA);
		am_hal_ctimer_adc_trigger_disable();
#else
		/* Disable internal repeat trigger timer */
		am_hal_adc_irtt_disable(data->adcHandle);
#endif
		/* Disable DMA */
		ADCn(0)->DMACFG_b.DMAEN = 0;
	}
#endif
}

static void adc_ambiq_start_sampling(const struct device *dev)
{
	struct adc_ambiq_data *data = dev->data;

	LOG_DBG("ADC: sequence.options=%p, extra_samplings=%u", data->ctx.sequence.options,
		data->ctx.sequence.options ? data->ctx.options.extra_samplings : 0);
	/* Enable the ADC for safety. */
	am_hal_adc_enable(data->adcHandle);
#ifdef CONFIG_ADC_AMBIQ_DMA
	if (data->use_dma) {
		ADCn(0)->DMACFG_b.DMAEN = 1;

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
		/* Enable the ADC repetitive sample timer A3 */
		am_hal_ctimer_adc_trigger_enable();
		am_hal_ctimer_start(3, AM_HAL_CTIMER_TIMERA);
#else
		/* Enable internal repeat trigger timer */
		am_hal_adc_irtt_enable(data->adcHandle);
#endif
	}
#endif
	/* Trigger the ADC */
	am_hal_adc_sw_trigger(data->adcHandle);
}

#ifdef CONFIG_ADC_AMBIQ_DMA
static int adc_ambiq_dma_config(const struct device *dev, uint8_t active_channels)
{
	struct adc_ambiq_data *data = dev->data;
	am_hal_adc_dma_config_t ADCDmaConfig = data->dma_cfg;

	if (data->dma_cfg.ui32SampleCount < active_channels) {
		LOG_ERR("Not enough DMA buffer.\n");
		return -EOVERFLOW;
	}

	ADCDmaConfig.ui32SampleCount = active_channels;

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	/* Start a timer to trigger the ADC periodically. */
	am_hal_ctimer_config_single(3, AM_HAL_CTIMER_TIMERA,
				    AM_HAL_CTIMER_HFRC_3MHZ | AM_HAL_CTIMER_FN_REPEAT);
	am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA3);
	am_hal_ctimer_period_set(3, AM_HAL_CTIMER_TIMERA, 10, 5);
	/* Enable the timer A3 to trigger the ADC directly */
	am_hal_ctimer_adc_trigger_enable();
#else
	am_hal_adc_irtt_config_t ADCIrttConfig;

	/* Set up internal repeat trigger timer */
	ADCIrttConfig.bIrttEnable = true;
	ADCIrttConfig.eClkDiv = AM_HAL_ADC_RPTT_CLK_DIV16; /* 24MHz / 16  = 1.5MHz */
	ADCIrttConfig.ui32IrttCountMax = 750;              /* 1.5MHz / 750 = 2kHz */
	am_hal_adc_configure_irtt(data->adcHandle, &ADCIrttConfig);
#endif

	/* Configure DMA */
	if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure_dma(data->adcHandle, &ADCDmaConfig)) {
		LOG_ERR("Error - configuring DMA failed.\n");
		return -EINVAL;
	}

	am_hal_adc_interrupt_clear(data->adcHandle, AMBIQ_ADC_DMA_INT);
	am_hal_adc_interrupt_enable(data->adcHandle, AMBIQ_ADC_DMA_INT);

	return 0;
}
#endif /* CONFIG_ADC_AMBIQ_DMA */

static void adc_ambiq_isr(const struct device *dev)
{
	struct adc_ambiq_data *data = dev->data;
	uint32_t ui32IntMask;
	uint32_t ui32NumSamples;
	am_hal_adc_sample_t Sample;

	/* Read the interrupt status. */
	am_hal_adc_interrupt_status(data->adcHandle, &ui32IntMask, true);

#ifdef CONFIG_ADC_AMBIQ_STREAM
	if (data->streaming) {
		adc_ambiq_stream_isr(dev, ui32IntMask);
		return;
	}
#endif /* CONFIG_ADC_AMBIQ_STREAM */

	/*
	 * If we got a conversion completion interrupt (which should be our only
	 * ADC interrupt), go ahead and read the data.
	 */
	if (ui32IntMask & AM_HAL_ADC_INT_CNVCMP) {
		for (uint32_t i = 0; i < data->active_channels; i++) {
			/* Read the value from the FIFO. */
			ui32NumSamples = 1;
			am_hal_adc_samples_read(data->adcHandle, false, NULL, &ui32NumSamples,
						&Sample);
			*data->buffer++ = Sample.ui32Sample;
		}
		/* Stop sampling but keep ADC enabled for next sampling */
		adc_ambiq_stop_sampling(dev);
		/* Clear the ADC interrupt.*/
		am_hal_adc_interrupt_clear(data->adcHandle, AM_HAL_ADC_INT_CNVCMP);
		adc_context_on_sampling_done(&data->ctx, dev);
	}

#ifdef CONFIG_ADC_AMBIQ_DMA
	if (data->use_dma) {
#if defined(CONFIG_SOC_SERIES_APOLLO3X)
		if (ui32IntMask & AM_HAL_ADC_INT_DCMP) {
#else
		if (((ui32IntMask & AM_HAL_ADC_INT_FIFOOVR1) && (ADCn(0)->DMASTAT_b.DMACPL)) ||
		    (ui32IntMask & AM_HAL_ADC_INT_DCMP)) {
#endif
			/* Clear the DMA interrupt first to avoid race condition */
			am_hal_adc_interrupt_clear(data->adcHandle, ui32IntMask);

#if CONFIG_ADC_AMBIQ_HANDLE_CACHE
			if (!buf_in_nocache((uintptr_t)data->dma_cfg.ui32TargetAddress,
					    data->active_channels * sizeof(uint32_t))) {
				/* Invalidate Dcache after DMA read */
				sys_cache_data_invd_range((void *)data->dma_cfg.ui32TargetAddress,
							  data->active_channels * sizeof(uint32_t));
			}
#endif /* CONFIG_ADC_AMBIQ_HANDLE_CACHE */

			/* Read the value from the DMA buffer */
			am_hal_adc_samples_read(
				data->adcHandle, false, (uint32_t *)data->dma_cfg.ui32TargetAddress,
				(uint32_t *)&data->active_channels, data->sample_buf);

			/* Copy data to user buffer */
			for (uint32_t i = 0; i < data->active_channels; i++) {
				*data->buffer++ = data->sample_buf[i].ui32Sample;
			}

			/* Stop sampling but keep ADC enabled for next sampling */
			adc_ambiq_stop_sampling(dev);

			/* Let framework handle repeated samplings */
			adc_context_on_sampling_done(&data->ctx, dev);
		} else {
			/* Clear other DMA-related interrupts */
			am_hal_adc_interrupt_clear(data->adcHandle, ui32IntMask);
		}
	}
#endif /* CONFIG_ADC_AMBIQ_DMA */
}

static int adc_ambiq_check_buffer_size(const struct adc_sequence *sequence, uint8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(uint16_t);

	if (sequence->options) {
		needed_buffer_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		LOG_DBG("Provided buffer is too small (%u/%u)", sequence->buffer_size,
			needed_buffer_size);
		return -ENOMEM;
	}

	return 0;
}

static int adc_ambiq_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_ambiq_data *data = dev->data;
	const struct adc_ambiq_config *cfg = dev->config;
	uint8_t channel_id = 0;
	uint32_t channels = 0;
	uint8_t active_channels = 0;
	uint8_t slot_index;

	int error = 0;

	if (sequence->channels & ~BIT_MASK(cfg->num_channels)) {
		LOG_ERR("Incorrect channels, bitmask 0x%x", sequence->channels);
		return -EINVAL;
	}

	if (sequence->channels == 0UL) {
		LOG_ERR("No channel selected");
		return -EINVAL;
	}

	active_channels = POPCOUNT(sequence->channels);
	if (active_channels > AMBIQ_ADC_SLOT_NUMBER) {
		LOG_ERR("Too many channels for sequencer. Max: %d", AMBIQ_ADC_SLOT_NUMBER);
		return -ENOTSUP;
	}

	error = adc_ambiq_check_buffer_size(sequence, active_channels);
	if (error < 0) {
		return error;
	}

#ifdef CONFIG_ADC_AMBIQ_DMA
	/* Calculate total samples and decide whether to use DMA */
	if (data->dma_mode) {
		uint32_t total_samples = active_channels;

		if (sequence->options) {
			total_samples *= (1 + sequence->options->extra_samplings);
		}
		data->use_dma = (total_samples >= CONFIG_ADC_AMBIQ_DMA_MIN_SAMPLES);
		LOG_DBG("DMA mode: %s (total_samples=%u, threshold=%d)",
			data->use_dma ? "enabled" : "disabled", total_samples,
			CONFIG_ADC_AMBIQ_DMA_MIN_SAMPLES);
	} else {
		data->use_dma = false;
	}
#endif

	error = adc_ambiq_config(dev);
	if (error < 0) {
		return error;
	}

	channels = sequence->channels;
	for (slot_index = 0; slot_index < active_channels; slot_index++) {
		channel_id = find_lsb_set(channels) - 1;
		error = adc_ambiq_slot_config(dev, sequence, channel_id, slot_index);
		if (error < 0) {
			return error;
		}
		channels &= ~BIT(channel_id);
	}
	__ASSERT_NO_MSG(channels == 0);

#ifdef CONFIG_ADC_AMBIQ_DMA
	if (data->use_dma) {
		error = adc_ambiq_dma_config(dev, active_channels);
		if (error < 0) {
			return error;
		}
	} else {
		am_hal_adc_interrupt_enable(data->adcHandle, AM_HAL_ADC_INT_CNVCMP);
	}
#else
	am_hal_adc_interrupt_enable(data->adcHandle, AM_HAL_ADC_INT_CNVCMP);
#endif

	data->active_channels = active_channels;
	data->buffer = sequence->buffer;
	/* Start ADC conversion */
	adc_context_start_read(&data->ctx, sequence);

	/* Wait for all samplings to complete (handled by framework in ISR) */
	error = adc_context_wait_for_completion(&data->ctx);

	return error;
}

static int adc_ambiq_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_ambiq_data *data = dev->data;
	int error = 0;

	adc_context_lock(&data->ctx, false, NULL);

	error = pm_device_runtime_get(dev);
	if (error < 0) {
		LOG_ERR("Failed to get device runtime PM state");
		adc_context_release(&data->ctx, error);
		return error;
	}

	error = adc_ambiq_start_read(dev, sequence);
	if (error < 0) {
		pm_device_runtime_put(dev);
	}

	adc_context_release(&data->ctx, error);

	return error;
}

static int adc_ambiq_channel_setup(const struct device *dev, const struct adc_channel_cfg *chan_cfg)
{
	const struct adc_ambiq_config *cfg = dev->config;

	if (chan_cfg->channel_id >= cfg->num_channels) {
		LOG_ERR("unsupported channel id '%d'", chan_cfg->channel_id);
		return -ENOTSUP;
	}

	if (chan_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not valid");
		return -ENOTSUP;
	}

	if (chan_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Reference is not valid");
		return -ENOTSUP;
	}

	if (chan_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("unsupported acquisition_time '%d'", chan_cfg->acquisition_time);
		return -ENOTSUP;
	}

	if (chan_cfg->differential) {
		LOG_ERR("Differential sampling not supported");
		return -ENOTSUP;
	}

	return 0;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_ambiq_data *data = CONTAINER_OF(ctx, struct adc_ambiq_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_ambiq_data *data = CONTAINER_OF(ctx, struct adc_ambiq_data, ctx);

	data->repeat_buffer = data->buffer;
	adc_ambiq_start_sampling(data->dev);
}

static int adc_ambiq_init(const struct device *dev)
{
	struct adc_ambiq_data *data = dev->data;
	const struct adc_ambiq_config *cfg = dev->config;

	int ret;

	/* Initialize the ADC and get the handle*/
	if (AM_HAL_STATUS_SUCCESS != am_hal_adc_initialize(0, &data->adcHandle)) {
		ret = -ENODEV;
		LOG_ERR("Failed to initialize ADC, code:%d", ret);
		return ret;
	}

	/* power on ADC*/
	ret = am_hal_adc_power_control(data->adcHandle, AM_HAL_SYSCTRL_WAKE, false);
	if (ret != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("Failed to power on ADC, code: %d", ret);
		return -ENODEV;
	}

	ret = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Enable the ADC interrupts in the ADC. */
	cfg->irq_config_func();
	adc_context_unlock_unconditionally(&data->ctx);

	data->dev = dev;
#ifdef CONFIG_ADC_AMBIQ_DMA
	/* Initialize use_dma to false, will be set dynamically in adc_ambiq_start_read */
	data->use_dma = false;
#endif

	return 0;
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct adc_ambiq_data *data = CONTAINER_OF(ctx, struct adc_ambiq_data, ctx);

#ifdef CONFIG_ADC_AMBIQ_DMA
	data->use_dma = false;
#endif

	/* Disable all ADC interrupts */
	am_hal_adc_interrupt_disable(data->adcHandle, 0xFF);
	/* All sampling is truly complete, disable ADC and put device to sleep */
	am_hal_adc_disable(data->adcHandle);
	pm_device_runtime_put_async(data->dev, K_MSEC(1));
}

#ifdef CONFIG_ADC_ASYNC
static int adc_ambiq_read_async(const struct device *dev, const struct adc_sequence *sequence,
				struct k_poll_signal *async)
{
	struct adc_ambiq_data *data = dev->data;
	int error = 0;

	adc_context_lock(&data->ctx, true, async);

	error = pm_device_runtime_get(dev);
	if (error < 0) {
		adc_context_release(&data->ctx, error);
		return error;
	}

	error = adc_ambiq_start_read(dev, sequence);
	if (error < 0) {
		pm_device_runtime_put(dev);
	}

	adc_context_release(&data->ctx, error);

	return error;
}
#endif

#ifdef CONFIG_PM_DEVICE
static int adc_ambiq_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct adc_ambiq_data *data = dev->data;
	uint32_t ret = 0;
	am_hal_sysctrl_power_state_e status;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		status = AM_HAL_SYSCTRL_WAKE;
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		status = AM_HAL_SYSCTRL_DEEPSLEEP;
		break;
	default:
		return -ENOTSUP;
	}

	ret = am_hal_adc_power_control(data->adcHandle, status, true);

	if (ret != AM_HAL_STATUS_SUCCESS) {
		return -EPERM;
	} else {
		return 0;
	}
}
#endif /* CONFIG_PM_DEVICE */

#ifdef CONFIG_ADC_AMBIQ_STREAM

/* Number of bits required to represent the vref, used as the q31 shift value. */
static inline uint8_t adc_ambiq_stream_shift(uint16_t vref_mv)
{
	return 32 - __builtin_clz(vref_mv);
}

static inline void adc_ambiq_stream_convert_q31(q31_t *out, uint16_t sample, uint16_t vref_mv,
						uint8_t resolution, uint8_t shift)
{
	uint32_t scale = BIT(resolution);
	/* micro-volts per LSB */
	uint32_t sensitivity = (vref_mv * (scale - 1)) / scale * 1000 / scale;

	*out = BIT(31 - shift) * sensitivity / 1000000 * sample;
}

static int adc_ambiq_decoder_get_frame_count(const uint8_t *buffer, uint32_t channel,
					     uint16_t *frame_count)
{
	const struct adc_ambiq_stream_header *hdr = (const struct adc_ambiq_stream_header *)buffer;

	ARG_UNUSED(channel);
	*frame_count = hdr->frame_count;

	return 0;
}

static int adc_ambiq_decoder_get_size_info(struct adc_dt_spec adc_spec, uint32_t channel,
					   size_t *base_size, size_t *frame_size)
{
	ARG_UNUSED(adc_spec);
	ARG_UNUSED(channel);

	__ASSERT_NO_MSG(base_size != NULL);
	__ASSERT_NO_MSG(frame_size != NULL);

	*base_size = sizeof(struct adc_data);
	*frame_size = sizeof(struct adc_sample_data);

	return 0;
}

static int adc_ambiq_decoder_decode(const uint8_t *buffer, uint32_t channel, uint32_t *fit,
				    uint16_t max_count, void *data_out)
{
	const struct adc_ambiq_stream_header *hdr = (const struct adc_ambiq_stream_header *)buffer;
	struct adc_data *out = (struct adc_data *)data_out;
	uint8_t slot = hdr->num_channels;
	uint8_t shift;
	uint16_t count = 0;

	/* Map the requested channel id to its slot within a scan. */
	for (uint8_t i = 0; i < hdr->num_channels; i++) {
		if (hdr->channel_ids[i] == channel) {
			slot = i;
			break;
		}
	}
	if (slot == hdr->num_channels) {
		return -EINVAL;
	}

	if (*fit >= hdr->frame_count) {
		return 0;
	}

	shift = adc_ambiq_stream_shift(hdr->vref_mv);
	out->header.base_timestamp_ns = hdr->timestamp_ns;
	out->shift = shift;

	while (count < max_count && *fit < hdr->frame_count) {
		uint16_t sample = hdr->samples[(*fit) * hdr->num_channels + slot];

		out->readings[count].timestamp_delta = 0;
		adc_ambiq_stream_convert_q31(&out->readings[count].value, sample, hdr->vref_mv,
					     hdr->resolution, shift);
		count++;
		(*fit)++;
	}

	out->header.reading_count = count;

	return count;
}

static const struct adc_decoder_api adc_ambiq_decoder_api = {
	.get_frame_count = adc_ambiq_decoder_get_frame_count,
	.get_size_info = adc_ambiq_decoder_get_size_info,
	.decode = adc_ambiq_decoder_decode,
};

static int adc_ambiq_get_decoder(const struct device *dev, const struct adc_decoder_api **api)
{
	ARG_UNUSED(dev);
	*api = &adc_ambiq_decoder_api;

	return 0;
}

/*
 * One-shot RTIO path: performed on the RTIO work queue because the classic
 * am_hal ADC read is blocking. The resulting frame uses the same encoded
 * header as the streaming path so a single decoder serves both.
 */
static void adc_ambiq_submit_oneshot_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	const struct adc_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = cfg->adc;
	const struct adc_dt_spec *adc_spec = cfg->adc_spec;
	uint8_t num_channels = cfg->adc_spec_cnt;
	uint16_t raw[AMBIQ_ADC_STREAM_MAX_CHANNELS];
	uint32_t channels = 0;
	struct adc_ambiq_stream_header *hdr;
	uint8_t *buf;
	uint32_t buf_len;
	size_t frame_len;
	int rc;

	if (num_channels == 0 || num_channels > AMBIQ_ADC_STREAM_MAX_CHANNELS) {
		rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
		return;
	}

	for (uint8_t i = 0; i < num_channels; i++) {
		channels |= BIT(adc_spec[i].channel_id);
	}

	struct adc_sequence sequence = {
		.channels = channels,
		.buffer = raw,
		.buffer_size = num_channels * sizeof(uint16_t),
		.resolution = adc_spec[0].resolution,
		.oversampling = adc_spec[0].oversampling,
	};

	rc = adc_read(dev, &sequence);
	if (rc != 0) {
		LOG_WRN("Failed to read ADC samples (%d)", rc);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	frame_len = sizeof(struct adc_ambiq_stream_header) + num_channels * sizeof(uint16_t);
	rc = rtio_sqe_rx_buf(iodev_sqe, frame_len, frame_len, &buf, &buf_len);
	if (rc != 0) {
		LOG_WRN("Failed to get a read buffer of size %zu bytes", frame_len);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	hdr = (struct adc_ambiq_stream_header *)buf;
	hdr->timestamp_ns = k_ticks_to_ns_floor64(k_uptime_ticks());
	hdr->vref_mv = adc_spec[0].vref_mv;
	hdr->frame_count = 1;
	hdr->num_channels = num_channels;
	hdr->resolution = adc_spec[0].resolution;
	for (uint8_t i = 0; i < num_channels; i++) {
		hdr->channel_ids[i] = adc_spec[i].channel_id;
		hdr->samples[i] = raw[i];
	}

	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

static void adc_ambiq_submit_oneshot(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_work_req *req = rtio_work_req_alloc();

	ARG_UNUSED(dev);

	if (req == NULL) {
		LOG_ERR("RTIO work item allocation failed. Consider increasing "
			"CONFIG_RTIO_WORKQ_POOL_ITEMS.");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	rtio_work_req_submit(req, iodev_sqe, adc_ambiq_submit_oneshot_sync);
}

/* Re-arm the DMA engine for the next scan while the repeat-trigger timer keeps running. */
static void adc_ambiq_stream_rearm(const struct device *dev)
{
	struct adc_ambiq_data *data = dev->data;
	am_hal_adc_dma_config_t dma_cfg = data->dma_cfg;

	dma_cfg.ui32SampleCount = data->stream_num_channels;

	ADCn(0)->DMACFG_b.DMAEN = 0;
	am_hal_adc_configure_dma(data->adcHandle, &dma_cfg);
	am_hal_adc_interrupt_clear(data->adcHandle, AMBIQ_ADC_DMA_INT);
	am_hal_adc_interrupt_enable(data->adcHandle, AMBIQ_ADC_DMA_INT);
	ADCn(0)->DMACFG_b.DMAEN = 1;
}

static void adc_ambiq_stop_stream(const struct device *dev)
{
	struct adc_ambiq_data *data = dev->data;

	adc_ambiq_stop_sampling(dev);
	am_hal_adc_interrupt_disable(data->adcHandle, 0xFF);
	am_hal_adc_disable(data->adcHandle);
	data->streaming = false;
	data->use_dma = false;
	pm_device_runtime_put_async(dev, K_MSEC(1));
}

static int adc_ambiq_start_stream(const struct device *dev, const struct adc_read_config *cfg)
{
	struct adc_ambiq_data *data = dev->data;
	const struct adc_ambiq_config *dcfg = dev->config;
	const struct adc_dt_spec *adc_spec = cfg->adc_spec;
	uint8_t num_channels = cfg->adc_spec_cnt;
	uint32_t channels = 0;
	uint8_t slot = 0;
	struct adc_sequence sequence = {0};
	int error;

	if (num_channels == 0 || num_channels > AMBIQ_ADC_STREAM_MAX_CHANNELS) {
		return -EINVAL;
	}

	if (!data->dma_mode) {
		LOG_ERR("ADC streaming requires 'dma-mode' to be enabled in devicetree");
		return -ENOTSUP;
	}

	for (uint8_t i = 0; i < num_channels; i++) {
		if (adc_spec[i].channel_id >= dcfg->num_channels) {
			LOG_ERR("Invalid channel id %d", adc_spec[i].channel_id);
			return -EINVAL;
		}
		channels |= BIT(adc_spec[i].channel_id);
	}

	sequence.resolution = adc_spec[0].resolution;
	sequence.channels = channels;

	/* Streaming always uses the DMA + internal repeat trigger timer engine. */
	data->use_dma = true;

	error = adc_ambiq_config(dev);
	if (error < 0) {
		return error;
	}

	/* Configure one slot per channel; record the channel id in scan order so the
	 * decoder can map a channel id back to its sample offset.
	 */
	while (channels != 0) {
		uint8_t channel_id = find_lsb_set(channels) - 1;

		error = adc_ambiq_slot_config(dev, &sequence, channel_id, slot);
		if (error < 0) {
			return error;
		}
		data->stream_channel_ids[slot] = channel_id;
		channels &= ~BIT(channel_id);
		slot++;
	}

	data->stream_num_channels = num_channels;
	data->stream_resolution = adc_spec[0].resolution;
	data->stream_vref_mv = adc_spec[0].vref_mv;
	data->active_channels = num_channels;

	error = adc_ambiq_dma_config(dev, num_channels);
	if (error < 0) {
		return error;
	}

	adc_ambiq_start_sampling(dev);

	return 0;
}

static void adc_ambiq_stream_isr(const struct device *dev, uint32_t int_mask)
{
	struct adc_ambiq_data *data = dev->data;
	struct rtio_iodev_sqe *iodev_sqe;
	struct adc_ambiq_stream_header *hdr;
	uint32_t num_samples = data->stream_num_channels;
	uint8_t *buf;
	uint32_t buf_len;
	size_t frame_len;
	bool dma_complete;
	int rc;

#if defined(CONFIG_SOC_SERIES_APOLLO3X)
	dma_complete = (int_mask & AM_HAL_ADC_INT_DCMP);
#else
	dma_complete = ((int_mask & AM_HAL_ADC_INT_FIFOOVR1) && (ADCn(0)->DMASTAT_b.DMACPL)) ||
		       (int_mask & AM_HAL_ADC_INT_DCMP);
#endif

	am_hal_adc_interrupt_clear(data->adcHandle, int_mask);

	if (!dma_complete) {
		return;
	}

#if CONFIG_ADC_AMBIQ_HANDLE_CACHE
	if (!buf_in_nocache((uintptr_t)data->dma_cfg.ui32TargetAddress,
			    num_samples * sizeof(uint32_t))) {
		sys_cache_data_invd_range((void *)data->dma_cfg.ui32TargetAddress,
					  num_samples * sizeof(uint32_t));
	}
#endif /* CONFIG_ADC_AMBIQ_HANDLE_CACHE */

	am_hal_adc_samples_read(data->adcHandle, false,
				(uint32_t *)data->dma_cfg.ui32TargetAddress, &num_samples,
				data->sample_buf);

	iodev_sqe = data->sqe;
	data->sqe = NULL;

	/* No consumer is waiting for this frame: stop sampling to save power. The
	 * next submit will restart the stream.
	 */
	if (iodev_sqe == NULL) {
		adc_ambiq_stop_stream(dev);
		return;
	}

	frame_len = sizeof(struct adc_ambiq_stream_header) +
		    data->stream_num_channels * sizeof(uint16_t);

	rc = rtio_sqe_rx_buf(iodev_sqe, frame_len, frame_len, &buf, &buf_len);
	if (rc != 0) {
		rtio_iodev_sqe_err(iodev_sqe, rc);
		adc_ambiq_stop_stream(dev);
		return;
	}

	hdr = (struct adc_ambiq_stream_header *)buf;
	hdr->timestamp_ns = k_ticks_to_ns_floor64(k_uptime_ticks());
	hdr->vref_mv = data->stream_vref_mv;
	hdr->frame_count = 1;
	hdr->num_channels = data->stream_num_channels;
	hdr->resolution = data->stream_resolution;
	for (uint8_t i = 0; i < data->stream_num_channels; i++) {
		hdr->channel_ids[i] = data->stream_channel_ids[i];
		hdr->samples[i] = (uint16_t)data->sample_buf[i].ui32Sample;
	}

	rtio_iodev_sqe_ok(iodev_sqe, 0);

	/* Keep the stream running for the next frame. */
	adc_ambiq_stream_rearm(dev);
}

static void adc_ambiq_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct adc_read_config *cfg = iodev_sqe->sqe.iodev->data;
	struct adc_ambiq_data *data = dev->data;
	bool need_start;
	unsigned int key;
	int error;

	if (!cfg->is_streaming) {
		adc_ambiq_submit_oneshot(dev, iodev_sqe);
		return;
	}

	key = irq_lock();
	data->sqe = iodev_sqe;
	need_start = !data->streaming;
	if (need_start) {
		data->streaming = true;
	}
	irq_unlock(key);

	if (!need_start) {
		/* Stream already running; the ISR will complete this sqe with
		 * the next frame.
		 */
		return;
	}

	error = pm_device_runtime_get(dev);
	if (error < 0) {
		data->streaming = false;
		data->sqe = NULL;
		rtio_iodev_sqe_err(iodev_sqe, error);
		return;
	}

	error = adc_ambiq_start_stream(dev, cfg);
	if (error < 0) {
		data->streaming = false;
		data->sqe = NULL;
		pm_device_runtime_put(dev);
		rtio_iodev_sqe_err(iodev_sqe, error);
	}
}

#define ADC_AMBIQ_STREAM_API                                                                       \
	.submit = adc_ambiq_submit, .get_decoder = adc_ambiq_get_decoder,
#else
#define ADC_AMBIQ_STREAM_API
#endif /* CONFIG_ADC_AMBIQ_STREAM */

#ifdef CONFIG_ADC_ASYNC
#define ADC_AMBIQ_DRIVER_API(n)                                                                    \
	static DEVICE_API(adc, adc_ambiq_driver_api_##n) = {                                       \
		.channel_setup = adc_ambiq_channel_setup,                                          \
		.read = adc_ambiq_read,                                                            \
		.read_async = adc_ambiq_read_async,                                                \
		.ref_internal = DT_INST_PROP(n, internal_vref_mv),                                 \
		ADC_AMBIQ_STREAM_API                                                               \
	};
#else
#define ADC_AMBIQ_DRIVER_API(n)                                                                    \
	static DEVICE_API(adc, adc_ambiq_driver_api_##n) = {                                       \
		.channel_setup = adc_ambiq_channel_setup,                                          \
		.read = adc_ambiq_read,                                                            \
		.ref_internal = DT_INST_PROP(n, internal_vref_mv),                                 \
		ADC_AMBIQ_STREAM_API                                                               \
	};
#endif

#if CONFIG_ADC_AMBIQ_HANDLE_CACHE
#define __ADC_NOCACHE(n)                                                                           \
	__attribute__((section(DT_INST_PROP_OR(n, dma_buffer_location, ".nocache"))))
#else
#define __ADC_NOCACHE(n)
#endif /* CONFIG_ADC_AMBIQ_HANDLE_CACHE */

#ifdef CONFIG_ADC_AMBIQ_DMA

#define ADC_DMA_CFG(n, buf, size)                                                                  \
	{                                                                                          \
		.bDynamicPriority = true,                                                          \
		.ePriority = AM_HAL_ADC_PRIOR_SERVICE_IMMED,                                       \
		.bDMAEnable = true,                                                                \
		.ui32SampleCount = size,                                                           \
		.ui32TargetAddress = (uint32_t)buf,                                                \
	}

#define ADC_AMBIQ_DMA_BUF_DEFINE(n)                                                                \
	IF_ENABLED(DT_INST_PROP(n, dma_mode), (                                                \
		static uint32_t adc_ambiq_dma_buf##n[DT_INST_PROP_OR(n, dma_buffer_size, 128)] \
			__ADC_NOCACHE(n);                                                         \
		static am_hal_adc_sample_t                                                         \
			adc_sample_buf##n[DT_INST_PROP_OR(n, dma_buffer_size, 128)]; \
	))

#define ADC_AMBIQ_DMA_CFG_ENABLED(n)                                                               \
	ADC_DMA_CFG(n, adc_ambiq_dma_buf##n, DT_INST_PROP_OR(n, dma_buffer_size, 128))

#define ADC_AMBIQ_DMA_CFG_DISABLED ADC_DMA_CFG(0, NULL, 0)

#define ADC_AMBIQ_DMA_INIT(n)                                                                      \
	.dma_cfg = COND_CODE_1(DT_INST_PROP(n, dma_mode),                                      \
		(ADC_AMBIQ_DMA_CFG_ENABLED(n)),                                                   \
		(ADC_AMBIQ_DMA_CFG_DISABLED)),                      \
		 .dma_mode = DT_INST_PROP(n, dma_mode),                                            \
		 .sample_buf = COND_CODE_1(DT_INST_PROP(n, dma_mode),                              \
		(adc_sample_buf##n), (NULL)),

#else
#define ADC_AMBIQ_DMA_BUF_DEFINE(n)
#define ADC_AMBIQ_DMA_INIT(n)
#endif

#define ADC_AMBIQ_INIT(n)                                                                          \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	ADC_AMBIQ_DRIVER_API(n);                                                                   \
	static void adc_irq_config_func_##n(void)                                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), adc_ambiq_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};                                                                                         \
	ADC_AMBIQ_DMA_BUF_DEFINE(n)                                                                \
	static struct adc_ambiq_data adc_ambiq_data_##n = {                                        \
		ADC_CONTEXT_INIT_TIMER(adc_ambiq_data_##n, ctx),                                   \
		ADC_CONTEXT_INIT_LOCK(adc_ambiq_data_##n, ctx),                                    \
		ADC_CONTEXT_INIT_SYNC(adc_ambiq_data_##n, ctx), ADC_AMBIQ_DMA_INIT(n)};            \
	const static struct adc_ambiq_config adc_ambiq_config_##n = {                              \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.size = DT_INST_REG_SIZE(n),                                                       \
		.num_channels = DT_INST_PROP(n, channel_count),                                    \
		.irq_config_func = adc_irq_config_func_##n,                                        \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                      \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(n, adc_ambiq_pm_action);                                          \
	DEVICE_DT_INST_DEFINE(n, adc_ambiq_init, PM_DEVICE_DT_INST_GET(n), &adc_ambiq_data_##n,    \
			      &adc_ambiq_config_##n, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,        \
			      &adc_ambiq_driver_api_##n);

DT_INST_FOREACH_STATUS_OKAY(ADC_AMBIQ_INIT)
