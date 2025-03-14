/*
 * Copyright (c) 2025 Ambiq Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT raydium_rm69330

#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rm69330, CONFIG_DISPLAY_LOG_LEVEL);

struct rm69330_config {
	const struct device *mipi_dsi;
	const struct gpio_dt_spec reset;
	const struct gpio_dt_spec enable;
	const struct gpio_dt_spec ifsel1;
	const struct gpio_dt_spec ifsel2;
	uint8_t channel;
	uint32_t data_lanes;
	uint32_t pixel_format;
	uint32_t height;
	uint32_t width;
	uint32_t rotation;
};
struct rm69330_data {
	uint16_t xstart;
	uint16_t ystart;
	uint16_t width;
	uint16_t height;
	enum display_pixel_format pixel_format;
	enum display_orientation orientation;
};

static inline int rm69330_dcs_write(const struct device *dev, uint8_t cmd, const void *buf,
				    size_t len)
{
	const struct rm69330_config *cfg = dev->config;
	int ret;
	LOG_DBG("rm69330_dcs_write 0x%x", cmd);

	ret = mipi_dsi_dcs_write(cfg->mipi_dsi, cfg->channel, cmd, buf, len);
	if (ret < 0) {
		LOG_ERR("DCS 0x%x write failed! (%d)", cmd, ret);
		return ret;
	}

	return 0;
}

static inline int rm69330_generic_write(const struct device *dev, const void *buf, size_t len)
{
	const struct rm69330_config *cfg = dev->config;
	int ret;
	LOG_DBG("rm69330_generic_write.");
	ret = mipi_dsi_generic_write(cfg->mipi_dsi, cfg->channel, buf, len);
	if (ret < 0) {
		LOG_ERR("Generic write failed!");
		return ret;
	}

	return 0;
}

static int rm69330_blanking_on(const struct device *dev)
{
	LOG_DBG("rm69330_blanking_on");
	rm69330_dcs_write(dev, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
	return 0;
}

static int rm69330_blanking_off(const struct device *dev)
{
	LOG_DBG("rm69330_blanking_off");
	rm69330_dcs_write(dev, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
	return 0;
}

static int rm69330_write(const struct device *dev, uint16_t x, uint16_t y,
			 const struct display_buffer_descriptor *desc, const void *buf)
{
	struct rm69330_data *data = dev->data;
	uint8_t cmd[4];
	if (data->xstart != x || data->width != desc->width) {
		data->xstart = x;
		data->width = desc->width;

		cmd[0] = data->xstart >> 8U;
		cmd[1] = data->xstart & 0xFFU;
		cmd[2] = (data->xstart + data->width - 1) >> 8U;
		cmd[3] = (data->xstart + data->width - 1) & 0xFFU;
		rm69330_dcs_write(dev, MIPI_DCS_SET_COLUMN_ADDRESS, cmd, 4);
	}
	if (data->ystart != y || data->height != desc->height) {
		data->ystart = y;
		data->height = desc->height;

		cmd[0] = data->ystart >> 8U;
		cmd[1] = data->ystart & 0xFFU;
		cmd[2] = (data->ystart + data->height - 1) >> 8U;
		cmd[3] = (data->ystart + data->height - 1) & 0xFFU;
		rm69330_dcs_write(dev, MIPI_DCS_SET_PAGE_ADDRESS, cmd, 4);
	}

	rm69330_dcs_write(dev, MIPI_DCS_WRITE_MEMORY_START, buf, desc->buf_size);
	return 0;
}

static int rm69330_set_brightness(const struct device *dev, uint8_t brightness)
{
	uint8_t cmd[4];
	LOG_DBG("rm69330_set_brightness");
	/*
	 * brightness is generic command for ambiq
	 */
	cmd[0] = MIPI_DCS_SET_DISPLAY_BRIGHTNESS;
	cmd[1] = brightness;
	rm69330_generic_write(dev, cmd, 2);
	return 0;
}

static void rm69330_get_capabilities(const struct device *dev,
				     struct display_capabilities *capabilities)
{
	const struct rm69330_config *config = dev->config;
	LOG_DBG("rm69330_get_capabilities");
	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = config->width;
	capabilities->y_resolution = config->height;
	capabilities->supported_pixel_formats = config->pixel_format;
	capabilities->current_pixel_format = config->pixel_format;
	capabilities->current_orientation = config->rotation;
	return;
}

static int rm69330_configure(const struct device *dev)
{
	const struct rm69330_config *config = dev->config;
	struct rm69330_data *data = dev->data;
	uint8_t cmd[4];
	int ret;
	LOG_DBG("rm69330_configure");
	/*
	 * Sending brightness command.
	 */
	cmd[0] = MIPI_DCS_SET_DISPLAY_BRIGHTNESS;
	cmd[1] = 0xff;
	ret = rm69330_generic_write(dev, cmd, 2);

	/*
	 * Set MIPI Panel Pixel Format
	 */
	if (config->pixel_format == PANEL_PIXEL_FORMAT_RGB_888) {
		cmd[0] = MIPI_DCS_PIXEL_FORMAT_24BIT;
	} else if (config->pixel_format == PANEL_PIXEL_FORMAT_RGB_565) {
		cmd[0] = MIPI_DCS_PIXEL_FORMAT_16BIT;
	}
	ret = rm69330_dcs_write(dev, MIPI_DCS_SET_PIXEL_FORMAT, cmd, 1);

	const int MIPI_set_hsifopctr = 0x0A;
	const int MIPI_set_cmd_page = 0xFE;
	cmd[0] = MIPI_set_cmd_page;
	cmd[1] = 0x01; // MCS
	ret = rm69330_generic_write(dev, cmd, 2);

	cmd[0] = MIPI_set_hsifopctr;
	cmd[1] = 0xF8; // set N565 to 1
	ret = rm69330_generic_write(dev, cmd, 2);

	cmd[0] = MIPI_set_cmd_page;
	cmd[1] = 0x00; // UCS
	ret = rm69330_generic_write(dev, cmd, 2);

	cmd[0] = 0x00;
	ret = rm69330_dcs_write(dev, MIPI_DCS_SET_ADDRESS_MODE, cmd, 1);

#if defined(CONFIG_SOC_SERIES_APOLLO5X)
	cmd[0] = MIPI_set_cmd_page;
	cmd[1] = 0x20; // RFE 20
	ret = rm69330_generic_write(dev, cmd, 2);

	cmd[0] = 0xF4;
	cmd[1] = 0x5A; // RF4 5A
	ret = rm69330_generic_write(dev, cmd, 2);

	cmd[0] = 0xF5;
	cmd[1] = 0x59; // RF5 59
	ret = rm69330_generic_write(dev, cmd, 2);

	cmd[0] = MIPI_set_cmd_page;
	cmd[1] = 0x80; // RFE 80
	ret = rm69330_generic_write(dev, cmd, 2);

	cmd[0] = 0x00;
	cmd[1] = 0xF8; // R00 F8
	ret = rm69330_generic_write(dev, cmd, 2);

	cmd[0] = MIPI_set_cmd_page;
	cmd[1] = 0x00; // UCS
	ret = rm69330_generic_write(dev, cmd, 2);
#endif

	ret = rm69330_dcs_write(dev, MIPI_DCS_EXIT_SLEEP_MODE, cmd, 0);

	ret = rm69330_dcs_write(dev, MIPI_DCS_SET_DISPLAY_ON, cmd, 0);

	cmd[0] = 0x02; // TE output active at refresh frame
	ret = rm69330_dcs_write(dev, MIPI_DCS_SET_TEAR_ON, cmd, 1);

	cmd[0] = data->xstart >> 8U;
	cmd[1] = data->xstart & 0xFFU;
	cmd[2] = (data->xstart + data->width - 1) >> 8U;
	cmd[3] = (data->xstart + data->width - 1) & 0xFFU;
	ret = rm69330_dcs_write(dev, MIPI_DCS_SET_COLUMN_ADDRESS, cmd, 4);

	cmd[0] = data->ystart >> 8U;
	cmd[1] = data->ystart & 0xFFU;
	cmd[2] = (data->ystart + data->height - 1) >> 8U;
	cmd[3] = (data->ystart + data->height - 1) & 0xFFU;
	ret = rm69330_dcs_write(dev, MIPI_DCS_SET_PAGE_ADDRESS, cmd, 4);

	return 0;
}

static int rm69330_init(const struct device *dev)
{
	const struct rm69330_config *cfg = dev->config;
	struct rm69330_data *data = dev->data;
	struct mipi_dsi_device mdev;
	int ret;
	LOG_DBG("rm69330_configure");
#if !defined(CONFIG_SOC_SERIES_APOLLO5X)
	if (cfg->enable.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->enable)) {
			LOG_ERR("enable GPIO device is not ready!");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->enable, GPIO_OUTPUT_HIGH);
		if (ret < 0) {
			LOG_ERR("Could not pull enable high! (%d)", ret);
			return ret;
		}
		ret = gpio_pin_set_dt(&cfg->enable, 1);
		if (ret < 0) {
			LOG_ERR("Could not pull enable high! (%d)", ret);
			return ret;
		}
	}
	if (cfg->ifsel1.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->ifsel1)) {
			LOG_ERR("ifsel1 GPIO device is not ready!");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->ifsel1, GPIO_OUTPUT_LOW);
		if (ret < 0) {
			LOG_ERR("Could not pull ifsel1 low! (%d)", ret);
			return ret;
		}
		ret = gpio_pin_set_dt(&cfg->ifsel1, 0);
		if (ret < 0) {
			LOG_ERR("Could not pull enable low! (%d)", ret);
			return ret;
		}
	}
	if (cfg->ifsel2.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->ifsel2)) {
			LOG_ERR("ifsel2 GPIO device is not ready!");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->ifsel2, GPIO_OUTPUT_LOW);
		if (ret < 0) {
			LOG_ERR("Could not pull ifsel2 low! (%d)", ret);
			return ret;
		}
		ret = gpio_pin_set_dt(&cfg->ifsel2, 0);
		if (ret < 0) {
			LOG_ERR("Could not pull enable low! (%d)", ret);
			return ret;
		}
	}
#endif
	if (cfg->reset.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			LOG_ERR("Reset GPIO device is not ready!");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_HIGH);
		if (ret < 0) {
			LOG_ERR("Could not pull reset high! (%d)", ret);
			return ret;
		}
		k_sleep(K_MSEC(5));
		ret = gpio_pin_set_dt(&cfg->reset, 0);
		if (ret < 0) {
			LOG_ERR("Could not pull reset low! (%d)", ret);
			return ret;
		}
		k_sleep(K_MSEC(20));
		ret = gpio_pin_set_dt(&cfg->reset, 1);
		if (ret < 0) {
			LOG_ERR("Could not toggle reset pin from low to high! (%d)", ret);
			return ret;
		}
		k_sleep(K_MSEC(150));
	}

	/* attach to MIPI-DSI host */
	mdev.data_lanes = cfg->data_lanes;
	if (cfg->pixel_format == PANEL_PIXEL_FORMAT_RGB_888) {
		mdev.pixfmt = MIPI_DSI_PIXFMT_RGB888;
	} else if (cfg->pixel_format == PANEL_PIXEL_FORMAT_RGB_565) {
		mdev.pixfmt = MIPI_DSI_PIXFMT_RGB565;
	}

	mdev.mode_flags = MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_LPM;

	mdev.timings.hactive = data->width;
	mdev.timings.hbp = 1;
	mdev.timings.hfp = 1;
	mdev.timings.hsync = 10;
	mdev.timings.vactive = data->height;
	mdev.timings.vbp = 1;
	mdev.timings.vfp = 1;
	mdev.timings.vsync = 1;

	ret = mipi_dsi_attach(cfg->mipi_dsi, cfg->channel, &mdev);
	if (ret < 0) {
		LOG_ERR("MIPI-DSI attach failed! (%d)", ret);
		return ret;
	}

	ret = rm69330_configure(dev);
	if (ret) {
		LOG_ERR("DSI init sequence failed! (%d)", ret);
		return ret;
	}

	return 0;
}

static int rm69330_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format pixel_format)
{
	const struct rm69330_config *config = dev->config;
	struct rm69330_data *data = dev->data;
	uint8_t param;

	switch (pixel_format) {
	case PIXEL_FORMAT_RGB_565:
		data->pixel_format = MIPI_DSI_PIXFMT_RGB565;
		param = MIPI_DCS_PIXEL_FORMAT_16BIT;
		break;
	case PIXEL_FORMAT_RGB_888:
		data->pixel_format = MIPI_DSI_PIXFMT_RGB888;
		param = MIPI_DCS_PIXEL_FORMAT_24BIT;
		break;
	default:
		/* Other display formats not implemented */
		return -ENOTSUP;
	}
	return mipi_dsi_dcs_write(config->mipi_dsi, config->channel, MIPI_DCS_SET_PIXEL_FORMAT,
				  &param, 1);
}

static int rm69330_set_orientation(const struct device *dev,
				   const enum display_orientation orientation)
{
	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}
	LOG_ERR("Changing display orientation not implemented");
	return -ENOTSUP;
}

#ifdef CONFIG_PM_DEVICE

static int rm69330_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct rm69330_config *config = dev->config;
	struct rm69330_data *data = dev->data;
	struct mipi_dsi_device mdev = {0};

	mdev.data_lanes = config->data_lanes;
	mdev.pixfmt = data->pixel_format;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/* Detach from the MIPI DSI controller */
		return mipi_dsi_detach(config->mipi_dsi, config->channel, &mdev);
	case PM_DEVICE_ACTION_RESUME:
		return mipi_dsi_attach(config->mipi_dsi, config->channel, &mdev);
	default:
		return -ENOTSUP;
	}
}

#endif /* CONFIG_PM_DEVICE */

static DEVICE_API(display, rm69330_api) = {
	.blanking_on = rm69330_blanking_on,
	.blanking_off = rm69330_blanking_off,
	.write = rm69330_write,
	.set_brightness = rm69330_set_brightness,
	.get_capabilities = rm69330_get_capabilities,
	.set_pixel_format = rm69330_set_pixel_format,
	.set_orientation = rm69330_set_orientation,
};

#define RM69330_PANEL(id)                                                                          \
	static const struct rm69330_config rm69330_config_##id = {                                 \
		.mipi_dsi = DEVICE_DT_GET(DT_INST_BUS(id)),                                        \
		.reset = GPIO_DT_SPEC_INST_GET_OR(id, reset_gpios, {0}),                           \
		.enable = GPIO_DT_SPEC_INST_GET_OR(id, enable_gpios, {0}),                         \
		.ifsel1 = GPIO_DT_SPEC_INST_GET_OR(id, ifsel1_gpios, {0}),                         \
		.ifsel2 = GPIO_DT_SPEC_INST_GET_OR(id, ifsel2_gpios, {0}),                         \
		.channel = 0,                                                                      \
		.data_lanes = DT_INST_PROP_BY_IDX(id, data_lanes, 0),                              \
		.pixel_format = DT_INST_PROP(id, pixel_format),                                    \
		.height = DT_INST_PROP(id, height),                                                \
		.width = DT_INST_PROP(id, width),                                                  \
		.rotation = DT_INST_PROP(id, rotation),                                            \
	};                                                                                         \
	static struct rm69330_data rm69330_data_##id = {                                           \
		.xstart = 0,                                                                       \
		.ystart = 0,                                                                       \
		.width = DT_INST_PROP(id, width),                                                  \
		.height = DT_INST_PROP(id, height),                                                \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(id, rm69330_pm_action);                                           \
	DEVICE_DT_INST_DEFINE(id, &rm69330_init, NULL, &rm69330_data_##id, &rm69330_config_##id,   \
			      POST_KERNEL, CONFIG_DISPLAY_RM69330_INIT_PRIORITY, &rm69330_api);

DT_INST_FOREACH_STATUS_OKAY(RM69330_PANEL)
