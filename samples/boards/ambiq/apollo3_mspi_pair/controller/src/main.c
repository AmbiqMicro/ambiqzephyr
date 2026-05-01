/*
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Apollo3 MSPI bus-controller sample (paired with apollo3_mspi_pair_peripheral
 * running on a second EVB). Drives MSPI0 in single-bit single-rate mode and
 * exchanges a known data pattern with the peripheral via the IOS register
 * protocol (1-byte command = R/W flag in bit7 + 7-bit LRAM offset).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(apollo3_mspi_pair_controller, LOG_LEVEL_INF);

#define MSPI_BUS    DT_BUS(DT_ALIAS(mspi_peer))
#define MSPI_TARGET DT_ALIAS(mspi_peer)

#define LRAM_OFFSET 0x00
#define PAYLOAD_LEN 16

static const struct gpio_dt_spec peer_int =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), peer_int_gpios);

static uint8_t tx_buf[PAYLOAD_LEN];
static uint8_t rx_buf[PAYLOAD_LEN];

static int do_write(const struct device *bus, const struct mspi_dev_id *id)
{
	struct mspi_xfer_packet pkt = {
		.dir       = MSPI_TX,
		.cmd       = 0x80 | LRAM_OFFSET, /* IOS write flag + offset */
		.address   = 0,
		.num_bytes = PAYLOAD_LEN,
		.data_buf  = tx_buf,
		.cb_mask   = MSPI_BUS_NO_CB,
	};
	struct mspi_xfer xfer = {
		.async       = false,
		.xfer_mode   = MSPI_PIO,
		.cmd_length  = 1,
		.addr_length = 0,
		.priority    = 1,
		.packets     = &pkt,
		.num_packet  = 1,
		.timeout     = 1000,
	};

	return mspi_transceive(bus, id, &xfer);
}

static int do_read(const struct device *bus, const struct mspi_dev_id *id)
{
	struct mspi_xfer_packet pkt = {
		.dir       = MSPI_RX,
		.cmd       = 0x00 | LRAM_OFFSET, /* IOS read flag + offset */
		.address   = 0,
		.num_bytes = PAYLOAD_LEN,
		.data_buf  = rx_buf,
		.cb_mask   = MSPI_BUS_NO_CB,
	};
	struct mspi_xfer xfer = {
		.async       = false,
		.xfer_mode   = MSPI_PIO,
		.cmd_length  = 1,
		.addr_length = 0,
		.priority    = 1,
		.packets     = &pkt,
		.num_packet  = 1,
		.timeout     = 1000,
	};

	return mspi_transceive(bus, id, &xfer);
}

static int wait_peer_ready(k_timeout_t timeout)
{
	int64_t deadline = k_uptime_get() + k_ticks_to_ms_floor64(timeout.ticks);

	while (k_uptime_get() < deadline) {
		if (gpio_pin_get_dt(&peer_int) == 1) {
			return 0;
		}
		k_msleep(1);
	}
	return -ETIMEDOUT;
}

int main(void)
{
	const struct device *bus = DEVICE_DT_GET(MSPI_BUS);
	struct mspi_dev_id id = MSPI_DEVICE_ID_DT(MSPI_TARGET);
	int ret;

	if (!device_is_ready(bus)) {
		LOG_ERR("MSPI bus not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&peer_int)) {
		LOG_ERR("Peer INT GPIO not ready");
		return -ENODEV;
	}
	(void)gpio_pin_configure_dt(&peer_int, GPIO_INPUT);

	ret = mspi_dev_config(bus, &id, MSPI_DEVICE_CONFIG_ALL, NULL);
	if (ret) {
		LOG_ERR("mspi_dev_config failed: %d", ret);
		return ret;
	}

	for (int i = 0; i < PAYLOAD_LEN; i++) {
		tx_buf[i] = (uint8_t)(0xA0 + i);
	}

	LOG_INF("Writing %d bytes to peripheral LRAM offset 0x%02x", PAYLOAD_LEN, LRAM_OFFSET);
	ret = do_write(bus, &id);
	if (ret) {
		LOG_ERR("write transceive failed: %d", ret);
		return ret;
	}

	/* Give the peripheral time to populate response buffer. */
	if (wait_peer_ready(K_MSEC(500)) != 0) {
		LOG_WRN("Peer INT not asserted within timeout, reading anyway");
	}

	LOG_INF("Reading %d bytes back from peripheral LRAM offset 0x%02x",
		PAYLOAD_LEN, LRAM_OFFSET);
	memset(rx_buf, 0, sizeof(rx_buf));
	ret = do_read(bus, &id);
	if (ret) {
		LOG_ERR("read transceive failed: %d", ret);
		return ret;
	}

	bool match = true;
	for (int i = 0; i < PAYLOAD_LEN; i++) {
		/* Peripheral echoes back ~tx_buf[i] (one's complement) so we can
		 * tell echo from a stuck bus.
		 */
		uint8_t expected = (uint8_t)~tx_buf[i];

		if (rx_buf[i] != expected) {
			LOG_ERR("byte %d: got 0x%02x, expected 0x%02x",
				i, rx_buf[i], expected);
			match = false;
		}
	}

	if (match) {
		printk("apollo3_mspi_pair_controller: PASS\n");
	} else {
		printk("apollo3_mspi_pair_controller: FAIL\n");
	}
	return match ? 0 : -EIO;
}
