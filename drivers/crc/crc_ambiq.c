/*
 * Copyright (c) 2025 Ambiq Micro, Inc.
 * Author: Richard S Wheatley
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_hw_crc32

#include "soc.h"
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/crc/crc.h>

LOG_MODULE_REGISTER(ambiq_hw_crc32, CONFIG_KERNEL_LOG_LEVEL);

static int crc_ambiq_get_crc32(const struct device *dev, const void *startAddr, uint32_t sizeBytes,
			       uint32_t *pCrc)
{
	ARG_UNUSED(dev);

	/*
	 * Validate input parameters
	 */
	if (sizeBytes == 0 || startAddr == NULL || pCrc == NULL) {
		return -EINVAL;
	}

	/*
	 * If the CRC is already running then return an error
	 */
	if (SECURITY->CTRL_b.ENABLE) {
		return -EBUSY;
	}

	/*
	 * Generate the CRC
	 */
	am_hal_crc32((uint32_t)startAddr, sizeBytes, pCrc);

	return 0;
}

static int crc_ambiq_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Any hardware init can be performed here if needed. */
	return 0;
}

static const struct crc_driver_api crc_ambiq_api_funcs = {
	.get_crc = crc_ambiq_get_crc32,
};

DEVICE_DT_INST_DEFINE(0, crc_ambiq_init, NULL, NULL, NULL, POST_KERNEL,
					  CONFIG_CRC_INIT_PRIORITY,
					  &crc_ambiq_api_funcs);
