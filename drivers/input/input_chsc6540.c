/*
 * Copyright (c) 2026 Your Name
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT chipsemi_chsc6540

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_touch.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

LOG_MODULE_REGISTER(chsc6540, CONFIG_INPUT_LOG_LEVEL);

struct chsc6540_config {
	struct input_touchscreen_common_config common;
	struct i2c_dt_spec i2c;
	const struct gpio_dt_spec int_gpio;
	const struct gpio_dt_spec reset_gpio;
};

struct chsc6540_data {
	const struct device *dev;
	struct k_work work;
	struct gpio_callback int_gpio_cb;
	uint32_t touch_count;
};

#define CHSC6540_BASE_ADDR1         0x20
#define CHSC6540_BASE_ADDR2         0x00
#define CHSC6540_BASE_ADDR3         0x00
#define CHSC6540_ADDRESS_TOUCH_DATA 0x00
#define CHSC6540_TOUCH_DATA_LEN     16

static void chsc6540_work_handler(struct k_work *work)
{
	struct chsc6540_data *data = CONTAINER_OF(work, struct chsc6540_data, work);
	const struct device *dev = data->dev;
	const struct chsc6540_config *cfg = dev->config;

	uint8_t touch_data[CHSC6540_TOUCH_DATA_LEN] = {0};
	const uint8_t write_buffer[] = {
		CHSC6540_ADDRESS_TOUCH_DATA,
		CHSC6540_BASE_ADDR3,
		CHSC6540_BASE_ADDR2,
		CHSC6540_BASE_ADDR1,
	};
	int ret = i2c_write_read_dt(&cfg->i2c, write_buffer, sizeof(write_buffer), touch_data,
				    CHSC6540_TOUCH_DATA_LEN);

	if (ret < 0) {
		LOG_ERR("CHSC6540: Could not read touch data: %d", ret);
		return;
	}

	uint8_t num_points = touch_data[2] & 0x0F;
	bool is_pressed = num_points > 0;

	if (num_points >= 1) {
		uint16_t x1 = ((uint16_t)touch_data[3] << 8) | touch_data[4];
		uint16_t y1 = ((uint16_t)touch_data[5] << 8) | touch_data[6];

		data->touch_count++;
		if ((data->touch_count % 5) == 0) {
			LOG_INF("Touch: x=%d, y=%d, pressed=%d (count=%d)", x1, y1, is_pressed,
				data->touch_count);
		}
		input_touchscreen_report_pos(dev, x1, y1, K_FOREVER);
	}

	input_report_key(dev, INPUT_BTN_TOUCH, is_pressed, true, K_FOREVER);
}

static void chsc6540_isr_handler(const struct device *dev, struct gpio_callback *cb, uint32_t mask)
{
	struct chsc6540_data *data = CONTAINER_OF(cb, struct chsc6540_data, int_gpio_cb);

	k_work_submit(&data->work);
}

static int chsc6540_reset(const struct device *dev)
{
	const struct chsc6540_config *config = dev->config;
	int ret = 0;

	if (config->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			LOG_ERR("GPIO port %s not ready", config->reset_gpio.port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO (%d)", ret);
			return ret;
		}

		k_usleep(500);
		ret = gpio_pin_set_dt(&config->reset_gpio, 0);
		if (ret < 0) {
			LOG_ERR("Could not pull reset low (%d)", ret);
			return ret;
		}

		/* Wait for controller to boot up after reset */
		k_msleep(50);
	}

	return 0;
}

static int chsc6540_init(const struct device *dev)
{
	const struct chsc6540_config *config = dev->config;
	struct chsc6540_data *data = dev->data;
	int ret;

	LOG_INF("Initializing CHSC6540 touch controller");

	data->dev = dev;
	k_work_init(&data->work, chsc6540_work_handler);

	ret = chsc6540_reset(dev);
	if (ret < 0) {
		LOG_ERR("Failed to reset (%d)", ret);
		return ret;
	}

	LOG_INF("Reset complete");

	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("GPIO port %s not ready", config->int_gpio.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure interrupt GPIO pin: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure interrupt GPIO interrupt: %d", ret);
		return ret;
	}

	gpio_init_callback(&data->int_gpio_cb, chsc6540_isr_handler, BIT(config->int_gpio.pin));
	ret = gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
	if (ret < 0) {
		LOG_ERR("Could not set gpio callback: %d", ret);
		return ret;
	}

	LOG_INF("CHSC6540 initialized successfully, waiting for touch events...");

	return 0;
}

#define CHSC6540_DEFINE(index)                                                                     \
	static const struct chsc6540_config chsc6540_config_##index = {                            \
		.common = INPUT_TOUCH_DT_INST_COMMON_CONFIG_INIT(index),                           \
		.i2c = I2C_DT_SPEC_INST_GET(index),                                                \
		.int_gpio = GPIO_DT_SPEC_INST_GET(index, int_gpios),                               \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(index, reset_gpios, {0}),                   \
	};                                                                                         \
	static struct chsc6540_data chsc6540_data_##index;                                         \
	DEVICE_DT_INST_DEFINE(index, chsc6540_init, NULL, &chsc6540_data_##index,                  \
			      &chsc6540_config_##index, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,   \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(CHSC6540_DEFINE)
