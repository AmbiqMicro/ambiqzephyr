/*
 * Copyright (c) 2024 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Some Ambiq Apollox Blue SOC (e.g. Apollo3 Blue) uses internal designed BLEIF module which is
 * different from the general IOM module for SPI transceiver. The called HAL API will also be
 * independent. This driver is implemented for the BLEIF module usage scenarios.
 */

#define DT_DRV_COMPAT ambiq_spi_bleif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_ambiq_bleif);

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device_runtime.h>
#include <stdlib.h>
#include <errno.h>
#include "spi_context.h"
#include <am_mcu_apollo.h>

#define PWRCTRL_MAX_WAIT_US 5

/* The pins for the SPI transceiver are not exposed from the chips and no need for user to confiugre
 * them. But the chips design the configurable BLEIF timing observation functions on some exposed
 * pins. The user can set the macro SPI_BLEIF_TIMING_OBSERVATION to 1 to configure the pins for
 * timing trace purpose.
 */
#define SPI_BLEIF_TIMING_OBSERVATION 0

typedef int (*ambiq_spi_pwr_func_t)(void);

struct spi_ambiq_config {
	uint32_t base;
	int size;
	const struct pinctrl_dev_config *pcfg;
	ambiq_spi_pwr_func_t pwr_func;
};

struct spi_ambiq_data {
	struct spi_context ctx;
	am_hal_ble_config_t ble_cfg;
	void *BLEhandle;
};

#define SPI_BASE      (((const struct spi_ambiq_config *)(dev)->config)->base)
#define REG_STAT      0x268
#define SPI_STAT(dev) (SPI_BASE + REG_STAT)
#define SPI_WORD_SIZE 8

static int spi_config(const struct device *dev, const struct spi_config *config)
{
	struct spi_ambiq_data *data = dev->data;
	struct spi_context *ctx = &(data->ctx);

	int ret = 0;

	if (spi_context_configured(ctx, config)) {
		/* Already configured. No need to do it again. */
		return 0;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != SPI_WORD_SIZE) {
		LOG_ERR("Word size must be %d", SPI_WORD_SIZE);
		return -ENOTSUP;
	}

	if ((config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only supports single mode");
		return -ENOTSUP;
	}

	if (config->operation & SPI_LOCK_ON) {
		LOG_ERR("Lock On not supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("LSB first not supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}
	if (config->operation & SPI_MODE_LOOP) {
		LOG_ERR("Loopback mode not supported");
		return -ENOTSUP;
	}

	/* We consider only the default configuration defined in HAL is tested and stable. */
	data->ble_cfg = am_hal_ble_default_config;

	ctx->config = config;

	ret = am_hal_ble_config(data->BLEhandle, &data->ble_cfg);

	return ret;
}

static int spi_ambiq_xfer(const struct device *dev, const struct spi_config *config)
{
	struct spi_ambiq_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret = 0;

	am_hal_ble_transfer_t trans = {0};

	if (ctx->tx_len) {
		trans.ui8Command = AM_HAL_BLE_WRITE;
		trans.pui32Data = (uint32_t *)ctx->tx_buf;
		trans.ui16Length = ctx->tx_len;
		trans.bContinue = false;
	} else {
		trans.ui8Command = AM_HAL_BLE_READ;
		trans.pui32Data = (uint32_t *)ctx->rx_buf;
		trans.ui16Length = ctx->rx_len;
		trans.bContinue = false;
	}

	ret = am_hal_ble_blocking_transfer(data->BLEhandle, &trans);
	spi_context_complete(ctx, dev, 0);

	return ret;
}

static int spi_ambiq_transceive(const struct device *dev, const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	struct spi_ambiq_data *data = dev->data;
	int ret;

	ret = spi_config(dev, config);

	if (ret) {
		return ret;
	}

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

#if defined(CONFIG_PM_DEVICE_RUNTIME)
	int rc = pm_device_runtime_get(dev);

	if (rc < 0) {
		LOG_ERR("pm_device_runtime_get failed: %d", rc);
	}
#endif

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	ret = spi_ambiq_xfer(dev, config);

#if defined(CONFIG_PM_DEVICE_RUNTIME)
	rc = pm_device_runtime_put(dev);

	if (rc < 0) {
		LOG_ERR("pm_device_runtime_put failed: %d", rc);
	}
#endif

	return ret;
}

static int spi_ambiq_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_ambiq_data *data = dev->data;

	if (!sys_read32(SPI_STAT(dev))) {
		return -EBUSY;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static struct spi_driver_api spi_ambiq_driver_api = {
	.transceive = spi_ambiq_transceive,
	.release = spi_ambiq_release,
};

static int spi_ambiq_init(const struct device *dev)
{
	struct spi_ambiq_data *data = dev->data;
	const struct spi_ambiq_config *cfg = dev->config;
	int ret;

#if (SPI_BLEIF_TIMING_OBSERVATION)
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}
#endif /* SPI_BLEIF_TIMING_OBSERVATION */

	ret = am_hal_ble_initialize((cfg->base - REG_BLEIF_BASEADDR) / cfg->size, &data->BLEhandle);
	if (ret) {
		return ret;
	}

	ret = am_hal_ble_power_control(data->BLEhandle, AM_HAL_BLE_POWER_ACTIVE);
	if (ret) {
		return ret;
	}

	ret = cfg->pwr_func();

	return ret;
}

#ifdef CONFIG_PM_DEVICE
static int spi_ambiq_pm_action(const struct device *dev,
			       enum pm_device_action action)
{
	printk("spi_ambiq_pm_action, action %d\n", action);
	struct spi_ambiq_data *data = dev->data;
	uint32_t ret;
	am_hal_ble_power_state_e state;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		state = AM_HAL_BLE_POWER_ACTIVE;
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		state = AM_HAL_BLE_POWER_OFF;
		break;
	default:
		return -ENOTSUP;
	}

	ret = am_hal_ble_power_control(data->BLEhandle, state);

	if (ret != AM_HAL_STATUS_SUCCESS) {
		return -EPERM;
	} else {
		return 0;
	}
}
#endif /* CONFIG_PM_DEVICE */

#define AMBIQ_SPI_BLEIF_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static int pwr_on_ambiq_spi_##n(void)                                                      \
	{                                                                                          \
		uint32_t addr = DT_REG_ADDR(DT_INST_PHANDLE(n, ambiq_pwrcfg)) +                    \
				DT_INST_PHA(n, ambiq_pwrcfg, offset);                              \
		sys_write32((sys_read32(addr) | DT_INST_PHA(n, ambiq_pwrcfg, mask)), addr);        \
		k_busy_wait(PWRCTRL_MAX_WAIT_US);                                                  \
		return 0;                                                                          \
	}                                                                                          \
	static struct spi_ambiq_data spi_ambiq_data##n = {                                         \
		SPI_CONTEXT_INIT_LOCK(spi_ambiq_data##n, ctx),                                     \
		SPI_CONTEXT_INIT_SYNC(spi_ambiq_data##n, ctx)};                                    \
	static const struct spi_ambiq_config spi_ambiq_config##n = {                               \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.size = DT_INST_REG_SIZE(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.pwr_func = pwr_on_ambiq_spi_##n};                                                 \
	PM_DEVICE_DT_INST_DEFINE(n, spi_ambiq_pm_action);                                      \
	DEVICE_DT_INST_DEFINE(n, spi_ambiq_init, PM_DEVICE_DT_INST_GET(n), &spi_ambiq_data##n, &spi_ambiq_config##n,   \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_SPI_BLEIF_INIT)
