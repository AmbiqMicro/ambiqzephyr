/*
 * Copyright (c) 2024 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_sdmmc

#include <zephyr/types.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include "am_mcu_apollo.h"

LOG_MODULE_REGISTER(apollo_sdmmc, CONFIG_SDMMC_LOG_LEVEL);

typedef void (*irq_config_func_t)(const struct device *port);

struct apollo_sdmmc_config {
	const struct pinctrl_dev_config *pcfg;
	const struct gpio_dt_spec reset;
	irq_config_func_t irq_config;
	uint32_t port;
};

struct apollo_sdmmc_priv {
	struct k_sem thread_lock;
	struct k_sem sync;
	struct k_work work;

	am_hal_card_host_t *host;
	am_hal_card_t card;

	struct k_mutex lock;
};

static int sdmmc_block_read(struct apollo_sdmmc_priv *priv, uint8_t *buff,
		uint32_t sector, uint32_t count)
{
	uint32_t status;

	k_mutex_lock(&priv->lock, K_FOREVER);

	status = am_hal_card_block_read_sync(&priv->card, sector, count, buff);

	k_mutex_unlock(&priv->lock);

	if ((status & 0xFFFF) != 0 || (status >> 16) != count) {
		LOG_ERR("Sector %" PRIu32 " read error, status %u", sector, status);
		return -EIO;
	}

	return 0;
}

static int sdmmc_block_write(struct apollo_sdmmc_priv *priv, const uint8_t *buff,
		uint32_t sector, uint32_t count)
{
	uint32_t status;

	k_mutex_lock(&priv->lock, K_FOREVER);

	status = am_hal_card_block_write_sync(&priv->card, sector, count, (uint8_t *)buff);

	k_mutex_unlock(&priv->lock);

	if ((status & 0xFFFF) != 0 || (status >> 16) != count) {
		LOG_ERR("Sector %" PRIu32 " write error, status %u", sector, status);
		return -EIO;
	}

	return 0;
}

static int apollo_sdmmc_access_status(struct disk_info *disk)
{
	return DISK_STATUS_OK;
}

static int apollo_sdmmc_access_init(struct disk_info *disk)
{
	return 0;
}

static int apollo_sdmmc_access_read(struct disk_info *disk, uint8_t *buff,
				uint32_t sector, uint32_t count)
{
	const struct device *dev = disk->dev;
	struct apollo_sdmmc_priv *priv = dev->data;
	uint32_t last_sector = sector + count;
	int status;

	LOG_DBG("%s sector %x, count %d\r\n", __func__, sector, count);

	if (last_sector < sector || last_sector > priv->card.ui32MaxBlks) {
		LOG_ERR("Sector %" PRIu32 " is outside the range %u",
				last_sector, priv->card.ui32MaxBlks);
		return -EIO;
	}

	pm_device_busy_set(dev);
	status = sdmmc_block_read(priv, buff, sector, count);
	pm_device_busy_clear(dev);

	return status;
}

static int apollo_sdmmc_access_write(struct disk_info *disk, const uint8_t *buff,
				 uint32_t sector, uint32_t count)
{
	const struct device *dev = disk->dev;
	struct apollo_sdmmc_priv *priv = dev->data;
	uint32_t last_sector = sector + count;
	int status;

	LOG_DBG("%s sector %x, count %d\r\n", __func__, sector, count);

	if (last_sector < sector || last_sector > priv->card.ui32MaxBlks) {
		LOG_ERR("Sector %" PRIu32 " is outside the range %u",
				last_sector, priv->card.ui32MaxBlks);
		return -EIO;
	}

	pm_device_busy_set(dev);
	status = sdmmc_block_write(priv, buff, sector, count);
	pm_device_busy_clear(dev);

	return status;
}

static int apollo_sdmmc_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	const struct device *dev = disk->dev;
	struct apollo_sdmmc_priv *priv = dev->data;

	LOG_DBG("%s cmd %d\r\n", __func__, cmd);

	switch (cmd) {
	case DISK_IOCTL_CTRL_SYNC:
		break;
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(uint32_t *)buff = priv->card.ui32MaxBlks;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(uint32_t *)buff = priv->card.ui32BlkSize;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(uint32_t *)buff  = priv->card.ui32BlkSize;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct disk_operations apollo_sdmmc_ops = {
	.init = apollo_sdmmc_access_init,
	.status = apollo_sdmmc_access_status,
	.read = apollo_sdmmc_access_read,
	.write = apollo_sdmmc_access_write,
	.ioctl = apollo_sdmmc_access_ioctl,
};

static struct disk_info apollo_sdmmc_disk = {
	.name = CONFIG_SDMMC_VOLUME_NAME,
	.ops = &apollo_sdmmc_ops,
};

static void apollo_sdmmc_isr(const struct device *dev)
{
	uint32_t ui32IntStatus;
	struct apollo_sdmmc_priv *priv = dev->data;

	am_hal_sdhc_intr_status_get(priv->host->pHandle, &ui32IntStatus, true);
	am_hal_sdhc_intr_status_clear(priv->host->pHandle, ui32IntStatus);
	am_hal_sdhc_interrupt_service(priv->host->pHandle, ui32IntStatus);
}


static int disk_apollo_sdmmc_init(const struct device *dev)
{
	int err;
	struct apollo_sdmmc_priv *priv = dev->data;
	const struct apollo_sdmmc_config *cfg = dev->config;

	LOG_INF("Device %s initialized", dev->name);

	/* Configure dt provided device signals when available */
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	err = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_ACTIVE);
	if (err) {
		return err;
	}

	priv->host = am_hal_get_card_host(AM_HAL_SDHC_CARD_HOST + cfg->port, true);

	if (priv->host == NULL) {
		LOG_ERR("No such card host and stop");
		return -ENODEV;
	}

	if (cfg->irq_config) {
		cfg->irq_config(dev);
	}

	//
	// check if card is present
	//
	while (am_hal_card_host_find_card(priv->host, &priv->card) != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("No card is present now\n");
		k_sleep(K_MSEC(1000));
		LOG_ERR("Checking if card is available again\n");
	}

	while (am_hal_card_init(&priv->card, NULL,
				AM_HAL_CARD_PWR_CTRL_NONE) != AM_HAL_STATUS_SUCCESS) {
		k_sleep(K_MSEC(1000));
		LOG_ERR("card init failed, try again\n");
	}

	while (am_hal_card_cfg_set(&priv->card, AM_HAL_CARD_TYPE_EMMC,
				AM_HAL_HOST_BUS_WIDTH_4, 48000000, AM_HAL_HOST_BUS_VOLTAGE_1_8,
				AM_HAL_HOST_UHS_SDR50) != AM_HAL_STATUS_SUCCESS) {
		k_sleep(K_MSEC(100));
		LOG_ERR("setting card cfg failed\n");
	}

	k_mutex_init(&priv->lock);

	apollo_sdmmc_disk.dev = dev;

	return disk_access_register(&apollo_sdmmc_disk);
}

#define APOLLO_SDMMC_INIT(n)								\
											\
	PINCTRL_DT_INST_DEFINE(n);							\
											\
	static void apollo_sdmmc_irq_##n##_config(const struct device *dev)		\
	{										\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),			\
				apollo_sdmmc_isr, DEVICE_DT_INST_GET(n), 0);		\
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));				\
	}										\
											\
	static const struct apollo_sdmmc_config apollo_sdmmc_config_##n = {		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(n, reset_gpios),			\
		(									\
			.reset = GPIO_DT_SPEC_INST_GET(n, reset_gpios),			\
		),(									\
			.reset = NULL,							\
		))									\
		.irq_config = apollo_sdmmc_irq_##n##_config,				\
		.port = DT_INST_PROP_OR(n, port, 0),					\
	};										\
											\
	static struct apollo_sdmmc_priv apollo_sdmmc_priv_##n;				\
											\
	DEVICE_DT_INST_DEFINE(n, &disk_apollo_sdmmc_init, NULL,				\
		    &apollo_sdmmc_priv_##n,						\
		    &apollo_sdmmc_config_##n, POST_KERNEL,				\
		    CONFIG_SD_INIT_PRIORITY,						\
		    NULL);

DT_INST_FOREACH_STATUS_OKAY(APOLLO_SDMMC_INIT)
