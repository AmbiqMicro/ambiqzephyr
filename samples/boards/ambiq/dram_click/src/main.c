/*
 * Copyright (c) 2025, Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dram_click, CONFIG_SPI_LOG_LEVEL);

/*
 * Get the APS6404L PSRAM device via the spi-psram alias.
 * This alias is provided by the mikroe_dram_click shield overlay.
 *
 * A build error here means the shield was not specified with --shield
 * or the board does not define a mikrobus_spi node.
 */
static const struct device *const psram_dev = DEVICE_DT_GET(DT_ALIAS(spi_psram));

#define TEST_ADDR        0x0000
#define TRANSFER_SIZE    128

static uint8_t tx_buf[TRANSFER_SIZE];
static uint8_t rx_buf[TRANSFER_SIZE];

extern int aps6404l_write(const struct device *dev, uint8_t *pui8TxBuffer,
			  uint32_t ui32WriteAddress, uint32_t ui32NumBytes);
extern int aps6404l_read(const struct device *dev, uint8_t *pui8RxBuffer,
			 uint32_t ui32ReadAddress, uint32_t ui32NumBytes);

static bool verify_buffers(const uint8_t *tx, const uint8_t *rx, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (tx[i] != rx[i]) {
			LOG_ERR("Mismatch at byte %zu: wrote 0x%02x, read 0x%02x",
				i, tx[i], rx[i]);
			return false;
		}
	}
	return true;
}

int main(void)
{
	int ret;

	if (!device_is_ready(psram_dev)) {
		LOG_ERR("APS6404L device not ready");
		return -ENODEV;
	}

	printk("DRAM Click sample - APS6404L 64Mbit PSRAM\n");
	printk("==========================================\n");

	/* Fill TX buffer with a known pattern */
	for (int i = 0; i < TRANSFER_SIZE; i++) {
		tx_buf[i] = (uint8_t)(i & 0xFF);
	}
	memset(rx_buf, 0, sizeof(rx_buf));

	/* Write to PSRAM */
	ret = aps6404l_write(psram_dev, tx_buf, TEST_ADDR, TRANSFER_SIZE);
	if (ret != 0) {
		LOG_ERR("APS6404L Write FAILED (err %d)", ret);
		return ret;
	}
	printk("APS6404L Write PASSED\n");

	/* Read back from PSRAM */
	ret = aps6404l_read(psram_dev, rx_buf, TEST_ADDR, TRANSFER_SIZE);
	if (ret != 0) {
		LOG_ERR("APS6404L Read FAILED (err %d)", ret);
		return ret;
	}

	/* Verify */
	if (!verify_buffers(tx_buf, rx_buf, TRANSFER_SIZE)) {
		LOG_ERR("APS6404L data verification FAILED");
		return -EIO;
	}

	printk("APS6404L Read PASSED\n");
	printk("All %d bytes verified successfully.\n", TRANSFER_SIZE);

	return 0;
}
