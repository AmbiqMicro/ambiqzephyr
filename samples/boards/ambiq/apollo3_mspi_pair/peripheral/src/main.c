/*
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Apollo3 IOS SPI peripheral paired with apollo3_mspi_pair_controller.
 *
 * The Ambiq IOS protocol uses byte 0 as a command (R/W flag in bit 7 plus a
 * 7-bit LRAM offset). When the controller writes (cmd 0x80|off) the trailing
 * payload bytes land in the peripheral's LRAM at the requested offset; when
 * the controller reads (cmd 0x00|off) it pulls bytes back from LRAM.
 *
 * This sample uses the Zephyr SPI peripheral API: it queues a known response
 * pattern (the one's complement of whatever the controller wrote during the
 * previous cycle) so the controller can validate end-to-end data integrity.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(apollo3_mspi_pair_peripheral, LOG_LEVEL_INF);

#define SPI_PERIPH_NODE DT_ALIAS(mspi_peer)

#define PAYLOAD_LEN 16

static const struct device *spi_dev = DEVICE_DT_GET(SPI_PERIPH_NODE);

static const struct spi_config periph_cfg = {
	.frequency = 1000000,
	.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave     = 0,
};

static uint8_t rx_buf[PAYLOAD_LEN];
static uint8_t tx_buf[PAYLOAD_LEN];

int main(void)
{
	int ret;

	if (!device_is_ready(spi_dev)) {
		LOG_ERR("IOS SPI peripheral not ready");
		return -ENODEV;
	}

	/* Seed the response so the very first read after power-up still
	 * returns deterministic content if the controller reads before
	 * writing.
	 */
	for (int i = 0; i < PAYLOAD_LEN; i++) {
		tx_buf[i] = 0x00;
	}

	printk("apollo3_mspi_pair_peripheral: ready\n");

	while (1) {
		struct spi_buf tx = {
			.buf = tx_buf,
			.len = sizeof(tx_buf),
		};
		struct spi_buf rx = {
			.buf = rx_buf,
			.len = sizeof(rx_buf),
		};
		const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
		const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

		ret = spi_transceive(spi_dev, &periph_cfg, &tx_set, &rx_set);
		if (ret < 0) {
			LOG_WRN("spi_transceive: %d", ret);
			k_msleep(10);
			continue;
		}

		LOG_HEXDUMP_INF(rx_buf, sizeof(rx_buf), "RX from controller");

		/* Build next response = one's complement of received pattern. */
		for (int i = 0; i < PAYLOAD_LEN; i++) {
			tx_buf[i] = (uint8_t)~rx_buf[i];
		}
	}

	return 0;
}
