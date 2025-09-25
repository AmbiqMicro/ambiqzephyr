/*
 * Copyright (c) 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sharp_ls014b7dd01

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/jdi.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>

LOG_MODULE_REGISTER(ls014b7dd01, CONFIG_DISPLAY_LOG_LEVEL);

struct ls014b7dd01_config {
	const struct device *jdi;
	const struct jdi_device device;
	uint32_t rotation;
};

struct ls014b7dd01_data {
	uint16_t xstart;
	uint16_t ystart;
	uint16_t width;
	uint16_t height;
	enum display_pixel_format pixel_format;
	enum display_orientation orientation;
};

static int ls014b7dd01_blanking_on(const struct device *dev)
{
	const struct ls014b7dd01_config *config = dev->config;
	struct ls014b7dd01_data *data = dev->data;
	int ret;

	LOG_DBG("Turning display blanking on");

	return 0;
}

static int ls014b7dd01_blanking_off(const struct device *dev)
{
	const struct ls014b7dd01_config *config = dev->config;
	struct ls014b7dd01_data *data = dev->data;
	int ret;

	LOG_DBG("Turning display blanking off");

	return 0;
}

static int ls014b7dd01_write(const struct device *dev, const uint16_t x, const uint16_t y,
			     const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct ls014b7dd01_config *config = dev->config;
	struct ls014b7dd01_data *data = dev->data;
	struct jdi_msg message;
	int ret;

	message.startx = x;
	message.starty = y;
	message.tx_buf = (void *)buf;

	ret = jdi_transfer(dev, &message);
	if (ret < 0) {
		LOG_ERR("Failed to transfer: %d", ret);
		return ret;
	}

	return 0;
}

static void ls014b7dd01_get_capabilities(const struct device *dev,
					 struct display_capabilities *capabilities)
{
	struct ls014b7dd01_data *data = dev->data;
}

static int ls014b7dd01_set_orientation(const struct device *dev,
				       const enum display_orientation orientation)
{
	if (orientation != DISPLAY_ORIENTATION_NORMAL &&
	    orientation != DISPLAY_ORIENTATION_ROTATED_180) {
		LOG_ERR("Unsupported orientation");
		return -ENOTSUP;
	}

	return 0;
}

/* Display Driver API Structure */
static DEVICE_API(display, ls014b7dd01_api) = {
	.blanking_on = ls014b7dd01_blanking_on,
	.blanking_off = ls014b7dd01_blanking_off,
	.write = ls014b7dd01_write,
	.get_capabilities = ls014b7dd01_get_capabilities,
	.set_orientation = ls014b7dd01_set_orientation,
};

static int ls014b7dd01_init(const struct device *dev)
{
	const struct ls014b7dd01_config *config = dev->config;
	struct ls014b7dd01_data *data = dev->data;
	int ret;

	/* Attach to JDI */
	ret = jdi_attach(config->jdi, &config->device);
	if (ret < 0) {
		LOG_ERR("Failed to attach to JDI Host: %d", ret);
		return ret;
	}

	return 0;
}
/* Device Definition Macros */
#define LS014B7DD01_DEVICE(n)                                                                      \
	static struct ls014b7dd01_data ls014b7dd01_data_##n;                                       \
	static const struct ls014b7dd01_config ls014b7dd01_config_##n = {                          \
		.jdi = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
		.rotation = DT_INST_PROP_OR(n, rotation, 0),                                       \
		.device =                                                                          \
			{                                                                          \
				.input_pixfmt =                                                    \
					DT_INST_PROP_OR(n, input_pixel_format, JDI_PIXFMT_RGB565), \
				.width = DT_INST_PROP_OR(n, width, 280),                           \
				.height = DT_INST_PROP_OR(n, height, 280),                         \
				.mode_flags = DT_INST_PROP_OR(n, mode_flags, 0),                   \
			},                                                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, ls014b7dd01_init, NULL, &ls014b7dd01_data_##n,                    \
			      &ls014b7dd01_config_##n, POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,  \
			      &ls014b7dd01_api);

DT_INST_FOREACH_STATUS_OKAY(LS014B7DD01_DEVICE)
