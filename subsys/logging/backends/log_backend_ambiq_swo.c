/*
 * Copyright (c) 2018 Piotr Mienkowski
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Serial Wire Output (SWO) backend implementation for Ambiq SOCs.
 *
 * SWO/SWV has been developed by ARM. The following code works only on ARM
 * architecture.
 *
 * An SWO viewer program will typically set-up the SWO port including its
 * frequency when connected to the debug probe. Such configuration can persist
 * only until the MCU reset. The SWO backend initialization function will
 * re-configure the SWO port upon boot and set the frequency as specified by
 * the LOG_BACKEND_AMBIQ_SWO_FREQ_HZ Kconfig option. To ensure flawless operation
 * this frequency should much the one set by the SWO viewer program.
 *
 */

#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_backend_std.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
/* ambiq-sdk includes */
#include <am_mcu_apollo.h>

/* If ITM has pin control properties, apply them for SWO pins */
#if DT_NODE_HAS_PROP(DT_NODELABEL(itm), pinctrl_0)
PINCTRL_DT_DEFINE(DT_NODELABEL(itm));
#endif

/* Set TPIU prescaler for the current debug trace clock frequency. */
#ifndef CONFIG_LOG_BACKEND_AMBIQ_SWO_FREQ_HZ
#error "Missing CONFIG_LOG_BACKEND_AMBIQ_SWO_FREQ_HZ"
#else

/* Set reference frequency which can be custom or cpu frequency. */
#if DT_NODE_HAS_PROP(DT_NODELABEL(itm), swo_ref_frequency)
#define SWO_REF_FREQ DT_PROP(DT_NODELABEL(itm), swo_ref_frequency)
#elif DT_NODE_HAS_PROP(DT_PATH(cpus, cpu_0), clock_frequency)
#define SWO_REF_FREQ DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)
#else
#error "Missing DT 'clock-frequency' property on cpu@0 node"
#endif

#define SWO_FREQ_DIV \
	((SWO_REF_FREQ + (CONFIG_LOG_BACKEND_AMBIQ_SWO_FREQ_HZ / 2)) / \
		CONFIG_LOG_BACKEND_AMBIQ_SWO_FREQ_HZ)

#if SWO_FREQ_DIV > 0xFFFF
#error CONFIG_LOG_BACKEND_AMBIQ_SWO_FREQ_HZ is too low. SWO clock divider is 16-bit. \
	Minimum supported SWO clock frequency is \
	[Reference Clock Frequency]/2^16.
#endif

#endif
#if defined(CONFIG_SOC_SERIES_APOLLO4X)
#define DCU_SWO (0x00000010UL |  \
				 0x00000004UL )
#elif defined(CONFIG_SOC_SERIES_APOLLO5X)
#define DCU_SWO (0x00000010UL |  \
				 0x00000004UL |  \
				 0x00000400UL )
#else
#error "Soc series not supported."
#endif

static uint8_t buf[1];
static uint32_t log_format_current = CONFIG_LOG_BACKEND_AMBIQ_SWO_OUTPUT_DEFAULT;

static int char_out(uint8_t *data, size_t length, void *ctx)
{
	ARG_UNUSED(ctx);

	for (size_t i = 0; i < length; i++) {
		ITM_SendChar(data[i]);
	}

	return length;
}

LOG_OUTPUT_DEFINE(log_output_swo, char_out, buf, sizeof(buf));

static void log_backend_swo_process(const struct log_backend *const backend,
				    union log_msg_generic *msg)
{
	uint32_t flags = log_backend_std_get_flags();

	log_format_func_t log_output_func = log_format_func_t_get(log_format_current);

	log_output_func(&log_output_swo, &msg->log, flags);
}

static int format_set(const struct log_backend *const backend, uint32_t log_type)
{
	log_format_current = log_type;
	return 0;
}

static void log_backend_swo_init(struct log_backend const *const backend)
{
	uint32_t ui32dcuVal;

    /* Need to make sure that SWO is enabled */
    {
        if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 1) && (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 1))
        {
            am_hal_dcu_get(&ui32dcuVal);
            /* Enable SWO */
            if ( ((ui32dcuVal & DCU_SWO) != DCU_SWO) &&
                 (am_hal_dcu_update(true, DCU_SWO) != AM_HAL_STATUS_SUCCESS) )
            {
                /* Cannot enable SWO */
                return;
            }
        }
        else
        {
            /* If DCU is not accessible, we cannot determine if ITM can be safely enabled!! */
            return;
        }
    }

    /* Enable the ITM interface */
    am_hal_itm_enable();

    am_hal_tpiu_enable(CONFIG_LOG_BACKEND_AMBIQ_SWO_FREQ_HZ);

	/* Initialize pin control settings, if any are defined */
#if DT_NODE_HAS_PROP(DT_NODELABEL(itm), pinctrl_0)
	const struct pinctrl_dev_config *pincfg =
		PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(itm));

	pinctrl_apply_state(pincfg, PINCTRL_STATE_DEFAULT);
#endif
}

static void log_backend_swo_panic(struct log_backend const *const backend)
{
}

static void dropped(const struct log_backend *const backend, uint32_t cnt)
{
	ARG_UNUSED(backend);

	log_backend_std_dropped(&log_output_swo, cnt);
}

const struct log_backend_api log_backend_swo_api = {
	.process = log_backend_swo_process,
	.panic = log_backend_swo_panic,
	.init = log_backend_swo_init,
	.dropped = IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE) ? NULL : dropped,
	.format_set = format_set,
};

LOG_BACKEND_DEFINE(log_backend_swo, log_backend_swo_api, true);
