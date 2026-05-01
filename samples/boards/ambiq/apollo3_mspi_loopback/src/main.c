/*
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Apollo3 MSPI single-board self-test. Exercises the ambiq,mspi-controller
 * driver against a stub MSPI device. Reports PASS if the driver completes a
 * fixed number of TX transactions without error.
 *
 * No external memory chip is required. With a logic analyzer or scope on
 * MSPI0_D0 (P22), MSPI0_SCK (P24), and NCE19 (P19) the generated waveforms
 * can be observed.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(apollo3_mspi_loopback, LOG_LEVEL_INF);

#define MSPI_BUS    DT_BUS(DT_ALIAS(mspi_stub))
#define MSPI_TARGET DT_ALIAS(mspi_stub)

#define ITERATIONS  8
#define PAYLOAD_LEN 32

static uint8_t tx_buf[PAYLOAD_LEN];

static int run_tx(const struct device *bus, const struct mspi_dev_id *id, uint32_t addr)
{
	struct mspi_xfer_packet pkt = {
		.dir       = MSPI_TX,
		.cmd       = 0x02,
		.address   = addr,
		.num_bytes = PAYLOAD_LEN,
		.data_buf  = tx_buf,
		.cb_mask   = MSPI_BUS_NO_CB,
	};
	struct mspi_xfer xfer = {
		.async       = false,
		.xfer_mode   = MSPI_PIO,
		.cmd_length  = 1,
		.addr_length = 3,
		.priority    = 1,
		.packets     = &pkt,
		.num_packet  = 1,
		.timeout     = 1000,
	};

	return mspi_transceive(bus, id, &xfer);
}

int main(void)
{
	const struct device *bus = DEVICE_DT_GET(MSPI_BUS);
	struct mspi_dev_id id = MSPI_DEVICE_ID_DT(MSPI_TARGET);
	int errors = 0;

	if (!device_is_ready(bus)) {
		LOG_ERR("MSPI bus not ready");
		printk("apollo3_mspi_loopback: FAIL (bus not ready)\n");
		return -ENODEV;
	}

	int ret = mspi_dev_config(bus, &id, MSPI_DEVICE_CONFIG_ALL, NULL);
	if (ret) {
		LOG_ERR("mspi_dev_config failed: %d", ret);
		printk("apollo3_mspi_loopback: FAIL (dev_config=%d)\n", ret);
		return ret;
	}

	for (int i = 0; i < PAYLOAD_LEN; i++) {
		tx_buf[i] = (uint8_t)(0x55 ^ i);
	}

	int64_t t0 = k_uptime_get();

	for (int i = 0; i < ITERATIONS; i++) {
		ret = run_tx(bus, &id, i * PAYLOAD_LEN);
		if (ret) {
			LOG_ERR("iter %d: mspi_transceive=%d", i, ret);
			errors++;
		}
	}

	int64_t elapsed = k_uptime_get() - t0;
	uint32_t total_bytes = (uint32_t)ITERATIONS * (PAYLOAD_LEN + 1 + 3);

	LOG_INF("Completed %d transactions, %u bytes, %lld ms (errors=%d)",
		ITERATIONS, total_bytes, elapsed, errors);

	if (errors == 0) {
		printk("apollo3_mspi_loopback: PASS\n");
		return 0;
	}

	printk("apollo3_mspi_loopback: FAIL (%d transaction errors)\n", errors);
	return -EIO;
}
