/*
 * Copyright (c) 2024 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_sdio

#include <zephyr/drivers/sdhc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include "am_mcu_apollo.h"

LOG_MODULE_REGISTER(ambiq_sdio, CONFIG_SDHC_LOG_LEVEL);

#define AMBIQ_SDIO_FREQ_MAX MHZ(96)
#define AMBIQ_SDIO_FREQ_MIN KHZ(375)

struct ambiq_sdio_config {
	SDIO_Type *pSDHC;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(const struct device *dev);
	uint32_t inst;
};

struct ambiq_sdio_data {
	am_hal_card_t card;
	am_hal_card_host_t *host;
	struct k_sem transfer_sem;
	struct k_mutex access_mutex;
};

static void ambiq_sdio_isr(const struct device *dev)
{
	uint32_t ui32IntStatus;
	struct ambiq_sdio_data *data = dev->data;

	am_hal_sdhc_intr_status_get(data->host->pHandle, &ui32IntStatus, true);
	am_hal_sdhc_intr_status_clear(data->host->pHandle, ui32IntStatus);
	am_hal_sdhc_interrupt_service(data->host->pHandle, ui32IntStatus);
}

static int ambiq_sdio_reset(const struct device *dev)
{
	const struct ambiq_sdio_config *config = dev->config;
	uint32_t ui32Status = 0;

	ui32Status = am_hal_sdhc_software_reset(config->pSDHC, AM_HAL_SDHC_SW_RESET_ALL);
	if ( ui32Status )
	{
		LOG_ERR("SDHC software reset failed, ui32Status = %u", ui32Status);
		return -EIO;
	}

	return 0;
}

static int ambiq_sdio_get_host_props(const struct device *dev,
	struct sdhc_host_props *props)
{
	memset(props, 0, sizeof(*props));
	props->f_max = AMBIQ_SDIO_FREQ_MAX;
	props->f_min = AMBIQ_SDIO_FREQ_MIN;
	props->power_delay = 500;
	props->host_caps.high_spd_support = true;
	props->host_caps.suspend_res_support = true;
	props->host_caps.vol_330_support = true;
	props->host_caps.bus_8_bit_support = true;
	props->max_current_330 = 1024;
	return 0;
}

static int ambiq_sdio_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct ambiq_sdio_data *data = dev->data;

	LOG_DBG("%s(clock=%d, bus_width=%d, timing=%d, mode=%d)", __func__, ios->clock,
		ios->bus_width, ios->timing, ios->bus_mode);

	if (ios->clock != 0 && (ios->clock <= AMBIQ_SDIO_FREQ_MAX) && (ios->clock >= AMBIQ_SDIO_FREQ_MIN))
	{
		data->card.cfg.ui32Clock = ios->clock;
	}
	else if (ios->clock != 0)
	{
		return -ENOTSUP;
	}

	if (ios->bus_mode != SDHC_BUSMODE_PUSHPULL)
	{
		return -ENOTSUP;
	}

	switch (ios->bus_width)
	{
		case SDHC_BUS_WIDTH1BIT:
			data->card.cfg.eBusWidth = AM_HAL_HOST_BUS_WIDTH_1;
			break;
		case SDHC_BUS_WIDTH4BIT:
			data->card.cfg.eBusWidth = AM_HAL_HOST_BUS_WIDTH_4;
			break;
		case SDHC_BUS_WIDTH8BIT:
			data->card.cfg.eBusWidth = AM_HAL_HOST_BUS_WIDTH_8;
			break;
		default:
			return -ENOTSUP;
	}

	switch (ios->signal_voltage)
	{
		case SD_VOL_3_3_V:
			data->card.cfg.eIoVoltage = AM_HAL_HOST_BUS_VOLTAGE_3_3;
			break;
		case SD_VOL_3_0_V:
			data->card.cfg.eIoVoltage = AM_HAL_HOST_BUS_VOLTAGE_3_0;
			break;
		case SD_VOL_1_8_V:
			data->card.cfg.eIoVoltage = AM_HAL_HOST_BUS_VOLTAGE_1_8;
			break;
		default:
			return -ENOTSUP;
	}

	if (ios->signal_voltage != SD_VOL_1_8_V)
	{
		return -ENOTSUP;
	}

    if ( am_hal_card_set_speed(&data->card, data->card.cfg.ui32Clock) != AM_HAL_STATUS_SUCCESS )
    {
        LOG_ERR("Failed to change SDIO bus speed\n");
        return -ENOTSUP;
    }

    if ( am_hal_card_set_bus_width(&data->card, data->card.cfg.eBusWidth) != AM_HAL_STATUS_SUCCESS )
    {
        LOG_ERR("Failed to change SDIO bus width\n");
        return -ENOTSUP;
    }

	return 0;
}

static int ambiq_sdio_init(const struct device *dev)
{
	const struct ambiq_sdio_config *config = dev->config;
	struct ambiq_sdio_data *data = dev->data;

	int ret;

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	data->host = am_hal_get_card_host(AM_HAL_SDHC_CARD_HOST + config->inst, true);

	if (data->host == NULL) {
		LOG_ERR("No such card host and stop");
		return -ENODEV;
	}

	config->irq_config_func(dev);

	while (am_hal_card_host_find_card(data->host, &data->card) != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("No card is present now\n");
		k_sleep(K_MSEC(1000));
		LOG_ERR("Checking if card is available again\n");
	}

	while (am_hal_card_init(&data->card, NULL,
				AM_HAL_CARD_PWR_CTRL_NONE) != AM_HAL_STATUS_SUCCESS) {
		k_sleep(K_MSEC(1000));
		LOG_ERR("card init failed, try again\n");
	}

	while (am_hal_card_cfg_set(&data->card, AM_HAL_CARD_TYPE_EMMC,
				AM_HAL_HOST_BUS_WIDTH_4, 48000000, AM_HAL_HOST_BUS_VOLTAGE_1_8,
				AM_HAL_HOST_UHS_SDR50) != AM_HAL_STATUS_SUCCESS) {
		k_sleep(K_MSEC(100));
		LOG_ERR("setting card cfg failed\n");
	}

	k_mutex_init(&data->access_mutex);
	k_sem_init(&data->transfer_sem, 0, 1);
	return 0;
}

static int ambiq_sdio_get_card_present(const struct device *dev)
{
	struct ambiq_sdio_data *data = dev->data;

	return am_hal_sdhc_get_cd(data->host->pHandle);
}


static int ambiq_sdio_card_busy(const struct device *dev)
{
	struct ambiq_sdio_data *data = dev->data;
	uint32_t ui32Status;

	ui32Status = data->host->ops->card_busy(data->host, DEFAULT_GET_STATUS_TIMEOUT_MS);

	return (ui32Status != AM_HAL_STATUS_SUCCESS) ? 1 : 0;
}


static int ambiq_sdio_request(const struct device *dev,
			struct sdhc_command *cmd,
			struct sdhc_data *data)
{
	int ret;
	struct ambiq_sdio_data *dev_data = dev->data;
    uint32_t ui32Status = 0;

    am_hal_card_cmd_t sdio_cmd = {0};
    am_hal_card_cmd_data_t cmd_data = {0};

	if (cmd)
	{
		sdio_cmd.ui8Idx = cmd->opcode;
		sdio_cmd.ui32Arg = cmd->arg;
		sdio_cmd.ui32RespType = cmd->response_type;
		sdio_cmd.bASync = false;
	}
	else
	{
		LOG_ERR("Invalid CMD");
		return -EINVAL;
	}

	if (data)
	{
		cmd_data.ui32BlkCnt = data->blocks;
		cmd_data.ui32BlkSize = data->block_size;
		cmd_data.pui8Buf = data->data;
	}

	if (cmd_data.ui32BlkCnt > 1)
	{
		sdio_cmd.bAutoCMD12 = true;
	}

	if (sdio_cmd.ui8Idx == MMC_CMD_WRITE_SINGLE_BLOCK || sdio_cmd.ui8Idx == MMC_CMD_WRITE_MULTIPLE_BLOCK )
	{
		cmd_data.dir = AM_HAL_DATA_DIR_WRITE;
	}
	else if ( (sdio_cmd.ui8Idx == SDIO_CMD_IO_RW_EXTENDED) && (sdio_cmd.ui32Arg & BIT(SDIO_CMD_ARG_RW_SHIFT)))
	{
		cmd_data.dir = AM_HAL_DATA_DIR_WRITE;
	}
	else
	{
		cmd_data.dir = AM_HAL_DATA_DIR_READ;
	}

	ret = k_mutex_lock(&dev_data->access_mutex, K_MSEC(cmd->timeout_ms));
	if (ret) {
		LOG_ERR("Could not access card");
		return -EBUSY;
	}

	ui32Status = dev_data->host->ops->execute_cmd(&dev_data->host->pHandle, &sdio_cmd, &cmd_data);
	if (ui32Status != AM_HAL_STATUS_SUCCESS)
	{
		LOG_ERR("Failed to send command");
		return -EBUSY;
	}

	k_mutex_unlock(&dev_data->access_mutex);

	memcpy(cmd->response, sdio_cmd.ui32Resp, sizeof(cmd->response));

	LOG_DBG("Resp0 = 0x%x, Resp1 = 0x%x, Resp2 = 0x%x, Resp3 = 0x%x", cmd->response[0],
		cmd->response[1], cmd->response[2], cmd->response[3]);

	if (data)
	{
		data->bytes_xfered = (ui32Status >> 16) & 0xFFFF;
	}

	return ret;
}

static const struct sdhc_driver_api ambiq_sdio_api = {
	.reset = ambiq_sdio_reset,
	.get_host_props = ambiq_sdio_get_host_props,
	.set_io = ambiq_sdio_set_io,
	.get_card_present = ambiq_sdio_get_card_present,
	.request = ambiq_sdio_request,
	.card_busy = ambiq_sdio_card_busy,
};


#define AMBIQ_SDIO_INIT(n)							\
	static void sdio_##n##_irq_config_func(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			ambiq_sdio_isr, DEVICE_DT_INST_GET(n), 0);		\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	PINCTRL_DT_INST_DEFINE(n);						\
										\
	static const struct ambiq_sdio_config ambiq_sdio_config_##n = {		\
		.pSDHC = (SDIO_Type *) DT_INST_REG_ADDR(n),			\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.irq_config_func = sdio_##n##_irq_config_func,			\
		.inst = n,                                              \
	};									\
										\
	static struct ambiq_sdio_data ambiq_sdio_data_##n;          \
	DEVICE_DT_INST_DEFINE(n,						\
		&ambiq_sdio_init,						\
		NULL,								\
		&ambiq_sdio_data_##n,               \
		&ambiq_sdio_config_##n,                  \
		POST_KERNEL,							\
		CONFIG_SDHC_INIT_PRIORITY,					\
		&ambiq_sdio_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_SDIO_INIT)
