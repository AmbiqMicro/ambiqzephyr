/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/spi.h>

#define TESE_SPI DT_NODELABEL(spi0)

#if DT_NODE_HAS_STATUS(TESE_SPI, okay)
static const struct device *spi_dev = DEVICE_DT_GET(TESE_SPI);
#else
#error "Node is disabled"
#endif

#define TEST_FREQ_HZ 8000000U // 48000000U

#define XOR_BYTE 0

#define DATA_SIZE 10

static struct spi_config spi_cfg = {
	.frequency = TEST_FREQ_HZ,
	.operation = (SPI_HALF_DUPLEX | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) |
		      SPI_LINES_SINGLE),
};

static int ambiq_spi_rdsr(const struct device *dev, uint8_t *status)
{
	uint8_t rdsr = 0x05;
	uint8_t sr[2];
	int err;
	const struct spi_buf tx_buf = {
		.buf = &rdsr,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf = {
		.buf = sr,
		.len = sizeof(sr),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	err = spi_transceive(dev, &spi_cfg, &tx, &rx);
	if (!err) {
		*status = sr[1];
	}

	return err;
}

static int ambiq_spi_write(const struct device *dev, off_t offset, const void *buf, size_t len)
{
	uint8_t cmd[3] = {0x02, 0, 0};
	uint8_t *paddr;
	int err;
	const struct spi_buf tx_bufs[2] = {
		{
			.buf = cmd,
			.len = 3,
		},
		{
			.buf = (void *)buf,
			.len = len,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};
	paddr = &cmd[1];
	*paddr++ = offset >> 8;
	*paddr = offset;
	err = spi_transceive(dev, &spi_cfg, &tx, NULL);
	if (err) {
		return err;
	}

	return err;
}

static int ambiq_spi_read(const struct device *dev, off_t offset, void *buf, size_t len, bool bFd,
			  bool bCont)
{
	uint8_t cmd[3] = {0x03, 0, 0};
	uint8_t *paddr;
	int err;
	paddr = &cmd[1];
	*paddr++ = offset >> 8;
	*paddr = offset;

	const struct spi_buf rx_buf = {
		.buf = buf,
		.len = len,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	if (bCont) {
		spi_cfg.operation |= SPI_HOLD_ON_CS;
	} else {
		spi_cfg.operation &= ~SPI_HOLD_ON_CS;
	}

	if (bFd) {
		spi_cfg.operation &= ~SPI_HALF_DUPLEX;
		void *cmd_tx = NULL;
		cmd_tx = k_malloc(len);
		memset(cmd_tx, 0, len);
		memcpy(cmd_tx, cmd, 3);
		const struct spi_buf tx_buf = {
			.buf = cmd_tx,
			.len = len,
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buf,
			.count = 1,
		};
		err = spi_transceive(dev, &spi_cfg, &tx, &rx);
		if (err < 0) {
			return err;
		}
	} else {
		spi_cfg.operation |= SPI_HALF_DUPLEX;
		const struct spi_buf tx_buf = {
			.buf = cmd,
			.len = 3,
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buf,
			.count = 1,
		};
		err = spi_transceive(dev, &spi_cfg, &tx, &rx);
		if (err < 0) {
			return err;
		}
	}

	return err;
}

static int ambiq_spi_read_continue(const struct device *dev, void *buf, size_t len, bool bCont)
{
	int err;
	const struct spi_buf rx_buf = {
		.buf = buf,
		.len = len,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	if (bCont) {
		spi_cfg.operation |= SPI_HOLD_ON_CS;
	} else {
		spi_cfg.operation &= ~SPI_HOLD_ON_CS;
	}
	err = spi_transceive(dev, &spi_cfg, NULL, &rx);
	if (err < 0) {
		return err;
	}

	return err;
}

int main(void)
{
	int ret;
	uint8_t status = 0;

	if (!device_is_ready(spi_dev)) {
		printf("SPI 0 device not ready!\n");
		return -1;
	}

	uint8_t buff[DATA_SIZE] = {0};
	// Initialize Test Data
	for (uint32_t i = 0; i < DATA_SIZE; i++) {
		buff[i] = (i & 0xFF) ^ XOR_BYTE;
	}
	uint8_t rxdata[10][DATA_SIZE];

	/* Read register */
	ret = ambiq_spi_rdsr(spi_dev, &status);

	/* Write memory */
	ret = ambiq_spi_write(spi_dev, 0x1FFF, buff, DATA_SIZE);

	/* Read memory */
	ret = ambiq_spi_read(spi_dev, 0x1FFF, rxdata[0], DATA_SIZE, false, false);

	/* Read memory continue */
	ret = ambiq_spi_read(spi_dev, 0x1FFF, rxdata[0], DATA_SIZE, false, true);
	for (uint8_t i = 1; i < 10; i++) {
		ret = ambiq_spi_read_continue(spi_dev, rxdata[i], DATA_SIZE,
					      (i == 9 ? false : true));
	}

	/* Fullduplex */
	ret = ambiq_spi_read(spi_dev, 0x1FFF, rxdata[0], DATA_SIZE, true, false);

	return 0;
}
