/*
 * Copyright (c) 2025 Ambiq Micro, Inc.
 * Author: Richard S Wheatley
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/crc/crc.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(crc_sample);

/* CRC device array following test_counter.c pattern */
static const struct device *const crc_devices[] = {
#ifdef CONFIG_CRC_AMBIQ
	DEVICE_DT_GET(DT_NODELABEL(crc32)),
#endif
	/* Add other CRC devices here as they become available */
};

static void test_crc_basic(const struct device *dev)
{
	int ret;
	uint32_t crc = 0;

	/* Initialize test data for CRC calculation */
	uint32_t test_data[64];

	for (int i = 0; i < 64; i++) {
		test_data[i] = i; /* Safe multiplier to avoid overflow */
	}

	LOG_INF("Testing CRC device: %s", dev->name);

	if (!device_is_ready(dev)) {
		LOG_ERR("CRC device %s is not ready", dev->name);
		return;
	}

	/* Calculate CRC over test data */
	ret = crc_driver_get_crc(dev, test_data, sizeof(test_data), &crc);
	if (ret < 0) {
		LOG_ERR("CRC calculation failed: %d", ret);
	} else {
		LOG_INF("CRC32 of %d bytes: 0x%08x", sizeof(test_data), crc);
	}

	/* Test with different data patterns */

	/* Test pattern 1: All zeros */
	memset(test_data, 0, sizeof(test_data));
	ret = crc_driver_get_crc(dev, test_data, sizeof(test_data), &crc);
	if (ret < 0) {
		LOG_ERR("CRC calculation failed: %d", ret);
	} else {
		LOG_INF("CRC32 of zeros: 0x%08x", crc);
	}

	/* Test pattern 2: All 0xFF */
	memset(test_data, 0xFF, sizeof(test_data));
	ret = crc_driver_get_crc(dev, test_data, sizeof(test_data), &crc);
	if (ret < 0) {
		LOG_ERR("CRC calculation failed: %d", ret);
	} else {
		LOG_INF("CRC32 of 0xFF pattern: 0x%08x", crc);
	}

	/* Test pattern 3: Reversed incremental */
	for (int i = 0; i < 64; i++) {
		test_data[i] = 63 - i;
	}
	ret = crc_driver_get_crc(dev, test_data, sizeof(test_data), &crc);
	if (ret < 0) {
		LOG_ERR("CRC calculation failed: %d", ret);
	} else {
		LOG_INF("CRC32 of reversed incremental pattern: 0x%08x", crc);
	}
}

static void test_all_crc_instances(void)
{
	if (ARRAY_SIZE(crc_devices) == 0) {
		LOG_WRN("No CRC devices found");
		return;
	}

	LOG_INF("Found %d CRC device(s)", ARRAY_SIZE(crc_devices));

	for (int i = 0; i < ARRAY_SIZE(crc_devices); i++) {
		test_crc_basic(crc_devices[i]);
		/* Allow logs to be printed */
		k_msleep(100);
	}
}

int main(void)
{
	LOG_INF("CRC Sample starting...");

	/* Test all available CRC devices */
	test_all_crc_instances();

	LOG_INF("CRC test completed successfully");

	return 0;
}
