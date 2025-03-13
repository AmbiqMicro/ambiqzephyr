/*
 * Copyright (c) 2025 Ambiq Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_apollo_mipi_dsi

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <am_mcu_apollo.h>
#include <nema_dc_hal_ambiq.h>
#include <nema_dc_regs.h>
#include <nema_dc_hal.h>
#include <nema_dc_mipi.h>
#include <nema_dc_intern.h>
#include <nema_dc.h>
#include <nema_dc_dsi.h>

LOG_MODULE_REGISTER(dsi_ambiq, CONFIG_MIPI_DSI_LOG_LEVEL);

struct mipi_dsi_apollo_config {
	uint32_t data_lanes;
	uint32_t dbi_width;
	uint32_t phy_clock;
	uint32_t pix_fmt;
	bool disp_te;
	const struct pinctrl_dev_config *te_cfg;
	void (*irq_config_func)(const struct device *dev);
};

struct mipi_dsi_apollo_data {
	nemadc_layer_t dc_layer;
	nemadc_initial_config_t dc_config;
};

static int mipi_dsi_apollo_attach(const struct device *dev, uint8_t channel,
				  const struct mipi_dsi_device *mdev)
{
	const struct mipi_dsi_apollo_config *config = dev->config;
	struct mipi_dsi_apollo_data *data = dev->data;
	LOG_DBG("mipi_dsi_apollo_attach");

	if (mdev->data_lanes > config->data_lanes) {
		LOG_ERR("DSI data lane config conflict!\n");
	}

	if (mdev->pixfmt != config->pix_fmt) {
		LOG_ERR("DSI pixel format config conflict!\n");
	}

	if (am_hal_dsi_para_config(mdev->data_lanes, config->dbi_width, config->phy_clock, false) !=
	    0) {
		LOG_ERR("DSI config failed!\n");
	}

	switch (mdev->pixfmt) {
	case MIPI_DSI_PIXFMT_RGB888:
		if (config->dbi_width == 16) {
			data->dc_config.ui32PixelFormat = MIPICFG_16RGB888_OPT0;
		} else if (config->dbi_width == 8) {
			data->dc_config.ui32PixelFormat = MIPICFG_8RGB888_OPT0;
		}
		data->dc_layer.format = NEMADC_RGB24;
		break;

	case MIPI_DSI_PIXFMT_RGB565:
		if (config->dbi_width == 16) {
			data->dc_config.ui32PixelFormat = MIPICFG_16RGB565_OPT0;
		} else if (config->dbi_width == 8) {
			data->dc_config.ui32PixelFormat = MIPICFG_8RGB565_OPT0;
		}
		data->dc_layer.format = NEMADC_RGB565;
		break;
	default:
		LOG_ERR("Invalid color coding!\n");
		break;
	}

	data->dc_config.ui16ResX = mdev->timings.hactive;
	data->dc_config.ui32FrontPorchX = mdev->timings.hfp;
	data->dc_config.ui32BackPorchX = mdev->timings.hbp;
	data->dc_config.ui32BlankingX = mdev->timings.hsync;

	data->dc_config.ui16ResY = mdev->timings.vactive;
	data->dc_config.ui32FrontPorchY = mdev->timings.vfp;
	data->dc_config.ui32BackPorchY = mdev->timings.vbp;
	data->dc_config.ui32BlankingY = mdev->timings.vsync;

	data->dc_config.bTEEnable = config->disp_te;
	data->dc_config.eInterface = DISP_INTERFACE_DBIDSI;

	nemadc_configure(&data->dc_config);

	data->dc_layer.resx = data->dc_config.ui16ResX;
	data->dc_layer.resy = data->dc_config.ui16ResY;
	data->dc_layer.buscfg = 0;
	data->dc_layer.blendmode = NEMADC_BL_SRC;
	// data->dc_layer.format        = NEMADC_RGB24;
	data->dc_layer.stride = nemadc_stride_size(data->dc_layer.format, data->dc_layer.resx);
	data->dc_layer.startx = 0;
	data->dc_layer.starty = 0;
	data->dc_layer.sizex = data->dc_layer.resx;
	data->dc_layer.sizey = data->dc_layer.resy;
	data->dc_layer.alpha = 0xFF;
	data->dc_layer.flipx_en = 0;
	data->dc_layer.flipy_en = 0;
	data->dc_layer.extra_bits = 0;

	return 0;
}

static ssize_t mipi_dsi_apollo_transfer(const struct device *dev, uint8_t channel,
					struct mipi_dsi_msg *msg)
{
	struct mipi_dsi_apollo_data *data = dev->data;
	ssize_t len;
	uint8_t *pointer;
	int ret;
	LOG_DBG("mipi_dsi_apollo_transfer");
	switch (msg->type) {
	case MIPI_DSI_DCS_READ:
		ret = nemadc_mipi_cmd_read(msg->cmd, NULL, 0, (uint32_t *)msg->rx_buf,
					   (uint8_t)msg->rx_len, true, false);
		len = msg->rx_len;
		break;
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_DCS_LONG_WRITE:
		if ((msg->cmd == MIPI_DCS_WRITE_MEMORY_START) ||
		    (msg->cmd == MIPI_DCS_WRITE_MEMORY_CONTINUE)) {
			nemadc_timing(data->dc_layer.resx, 1, 10, 1, data->dc_layer.resy, 1, 1, 1);
			data->dc_layer.baseaddr_virt = (void *)msg->tx_buf;
			data->dc_layer.baseaddr_phys = (unsigned)(data->dc_layer.baseaddr_virt);
			nemadc_set_layer(0, &data->dc_layer);
			nemadc_transfer_frame_prepare(data->dc_config.bTEEnable);
			if (!data->dc_config.bTEEnable) {
				//
				// It's necessary to launch frame manually when TE is disabled.
				//
				nemadc_transfer_frame_launch();
			}
			nemadc_wait_vsync();
		} else {
			nemadc_mipi_cmd_write(msg->cmd, (uint8_t *)msg->tx_buf,
					      (uint8_t)msg->tx_len, true, false);
			len = msg->tx_len;
			pointer = (uint8_t *)msg->tx_buf;
			if (msg->cmd == MIPI_DCS_SET_COLUMN_ADDRESS) {
				data->dc_layer.resx = (int32_t)((pointer[2] << 8) | pointer[3]) +
						      1 - (int32_t)((pointer[0] << 8) | pointer[1]);
				data->dc_layer.stride = nemadc_stride_size(data->dc_layer.format,
									   data->dc_layer.resx);
			} else if (msg->cmd == MIPI_DCS_SET_PAGE_ADDRESS) {
				data->dc_layer.resy = (int32_t)((pointer[2] << 8) | pointer[3]) +
						      1 - (int32_t)((pointer[0] << 8) | pointer[1]);
			}
		}
		break;
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		ret = nemadc_mipi_cmd_read(NULL, (uint8_t *)msg->tx_buf, (uint8_t)msg->tx_len,
					   (uint32_t *)msg->rx_buf, (uint8_t)msg->rx_len, false,
					   false);
		len = msg->rx_len;
		break;
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_GENERIC_LONG_WRITE:
		nemadc_mipi_cmd_write(NULL, (uint8_t *)msg->tx_buf, (uint8_t)msg->tx_len, false,
				      false);
		len = msg->tx_len;
		break;
	default:
		LOG_ERR("Unsupported message type (%d)", msg->type);
		return -ENOTSUP;
	}

	return len;
}

static struct mipi_dsi_driver_api dsi_apollo_api = {
	.attach = mipi_dsi_apollo_attach,
	.transfer = mipi_dsi_apollo_transfer,
};

static int mipi_dsi_apollo_init(const struct device *dev)
{
	const struct mipi_dsi_apollo_config *config = dev->config;
	int ret;
	LOG_DBG("mipi_dsi_apollo_init");
	/* Select "default" state at initialization time */
	ret = pinctrl_apply_state(config->te_cfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}
	//
	// Global interrupt enable
	//
	am_hal_interrupt_master_enable();

	//
	// VDD18 control callback function
	//
	am_hal_dsi_register_external_vdd18_callback(NULL);

	//
	// Enable DSI power and configure DSI clock.
	//
	am_hal_dsi_init();

	//
	// Power up DC
	//
	am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISP);

	//
	// Initialize NemaDC
	//
	if (nemadc_init() != 0) {
		LOG_ERR("DC init failed!\n");
	}

	config->irq_config_func(dev);
	return 0;
}

/*
 * Ambiq DC interrupt service routine
 */
extern void am_disp_isr();

static void ambiq_disp_isr(const struct device *dev)
{
	am_disp_isr();
}

#define APOLLO_MIPI_DSI_DEVICE(inst)                                                               \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static void disp_##inst##_irq_config_func(const struct device *dev)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), ambiq_disp_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static struct mipi_dsi_apollo_data apollo_dsi_data_##inst = {                              \
		.dc_layer = {0},                                                                   \
		.dc_config = {0},                                                                  \
	};                                                                                         \
	static const struct mipi_dsi_apollo_config apollo_dsi_config_##inst = {                    \
		.data_lanes = DT_INST_PROP(inst, data_lanes),                                      \
		.dbi_width = DT_INST_PROP(inst, dbi_width),                                        \
		.phy_clock = DT_INST_PROP(inst, phy_clock),                                        \
		.pix_fmt = DT_INST_PROP(inst, pix_fmt),                                            \
		.disp_te = DT_INST_PROP(inst, disp_te),                                            \
		.te_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.irq_config_func = disp_##inst##_irq_config_func,                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &mipi_dsi_apollo_init, NULL, &apollo_dsi_data_##inst,          \
			      &apollo_dsi_config_##inst, POST_KERNEL,                              \
			      CONFIG_MIPI_DSI_INIT_PRIORITY, &dsi_apollo_api);

DT_INST_FOREACH_STATUS_OKAY(APOLLO_MIPI_DSI_DEVICE)
