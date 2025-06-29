/*
 * Copyright (c) 2022 Intel Corporation
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_icm42688

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/icm42688.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>

#include "icm42688.h"
#include "icm42688_decoder.h"
#include "icm42688_reg.h"
#include "icm42688_rtio.h"
#include "icm42688_spi.h"
#include "icm42688_trigger.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ICM42688, CONFIG_SENSOR_LOG_LEVEL);

static void icm42688_convert_accel(struct sensor_value *val, int16_t raw_val,
				   struct icm42688_cfg *cfg)
{
	icm42688_accel_ms(cfg, (int32_t)raw_val, &val->val1, &val->val2);
}

static void icm42688_convert_gyro(struct sensor_value *val, int16_t raw_val,
				  struct icm42688_cfg *cfg)
{
	icm42688_gyro_rads(cfg, (int32_t)raw_val, &val->val1, &val->val2);
}

static inline void icm42688_convert_temp(struct sensor_value *val, int16_t raw_val)
{
	icm42688_temp_c((int32_t)raw_val, &val->val1, &val->val2);
}

int icm42688_channel_parse_readings(enum sensor_channel chan, int16_t readings[7],
				    struct icm42688_cfg *cfg, struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm42688_convert_accel(&val[0],
			cfg->axis_align[0].sign*readings[cfg->axis_align[0].index + 1], cfg);
		icm42688_convert_accel(&val[1],
			cfg->axis_align[1].sign*readings[cfg->axis_align[1].index + 1], cfg);
		icm42688_convert_accel(&val[2],
			cfg->axis_align[2].sign*readings[cfg->axis_align[2].index + 1], cfg);
		break;
	case SENSOR_CHAN_ACCEL_X:
		icm42688_convert_accel(val,
			cfg->axis_align[0].sign*readings[cfg->axis_align[0].index + 1], cfg);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm42688_convert_accel(val,
			cfg->axis_align[1].sign*readings[cfg->axis_align[1].index + 1], cfg);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm42688_convert_accel(val,
			cfg->axis_align[2].sign*readings[cfg->axis_align[2].index + 1], cfg);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm42688_convert_gyro(&val[0],
			cfg->axis_align[0].sign*readings[cfg->axis_align[0].index + 4], cfg);
		icm42688_convert_gyro(&val[1],
			cfg->axis_align[1].sign*readings[cfg->axis_align[1].index + 4], cfg);
		icm42688_convert_gyro(&val[2],
			cfg->axis_align[2].sign*readings[cfg->axis_align[2].index + 4], cfg);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm42688_convert_gyro(val,
			cfg->axis_align[0].sign*readings[cfg->axis_align[0].index + 4], cfg);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm42688_convert_gyro(val,
			cfg->axis_align[1].sign*readings[cfg->axis_align[1].index + 4], cfg);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm42688_convert_gyro(val,
			cfg->axis_align[2].sign*readings[cfg->axis_align[2].index + 4], cfg);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm42688_convert_temp(val, readings[0]);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int icm42688_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct icm42688_dev_data *data = dev->data;

	return icm42688_channel_parse_readings(chan, data->readings, &data->cfg, val);
}

static int icm42688_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	uint8_t status;
	struct icm42688_dev_data *data = dev->data;
	const struct icm42688_dev_cfg *cfg = dev->config;

	int res = icm42688_spi_read(&cfg->spi, REG_INT_STATUS, &status, 1);

	if (res) {
		return res;
	}

	if (!FIELD_GET(BIT_DATA_RDY_INT, status)) {
		return -EBUSY;
	}

	uint8_t readings[14];

	res = icm42688_read_all(dev, readings);

	if (res) {
		return res;
	}

	for (int i = 0; i < 7; i++) {
		data->readings[i] = sys_le16_to_cpu((readings[i * 2] << 8) | readings[i * 2 + 1]);
	}

	return 0;
}

static int icm42688_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct icm42688_dev_data *data = dev->data;
	struct icm42688_cfg new_config = data->cfg;
	int res = 0;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			new_config.accel_odr = icm42688_accel_hz_to_reg(val->val1);
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			new_config.accel_fs = icm42688_accel_fs_to_reg(sensor_ms2_to_g(val));
		} else {
			LOG_ERR("Unsupported attribute");
			res = -ENOTSUP;
		}
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			new_config.gyro_odr = icm42688_gyro_odr_to_reg(val->val1);
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			new_config.gyro_fs = icm42688_gyro_fs_to_reg(sensor_rad_to_degrees(val));
		} else {
			LOG_ERR("Unsupported attribute");
			res = -EINVAL;
		}
		break;
	case SENSOR_CHAN_ALL:
		if (attr == SENSOR_ATTR_BATCH_DURATION) {
			if (val->val1 < 0) {
				return -EINVAL;
			}
			new_config.batch_ticks = val->val1;
		} else if ((enum sensor_attribute_icm42688)attr ==
			   SENSOR_ATTR_ICM42688_PIN9_FUNCTION) {
			if (val->val1 != ICM42688_PIN9_FUNCTION_INT2 &&
			    val->val1 != ICM42688_PIN9_FUNCTION_FSYNC &&
			    val->val1 != ICM42688_PIN9_FUNCTION_CLKIN) {
				LOG_ERR("Unknown pin function");
				return -EINVAL;
			}

			if (val->val2 < 31000 || val->val2 > 50000) {
				LOG_ERR("RTC frequency must be between 31kHz and 50kHz");
				return -EINVAL;
			}

			/* TODO: Allow this if FSYNC is configurable later. */
			if (val->val1 == ICM42688_PIN9_FUNCTION_FSYNC) {
				LOG_ERR("FSYNC is disabled, PIN9_FUNCTION should not be set to "
					"FSYNC");
				return -ENOTSUP;
			}

			new_config.pin9_function = val->val1;
			new_config.rtc_freq = val->val2;
		} else {
			LOG_ERR("Unsupported attribute");
			res = -EINVAL;
		}
		break;
	default:
		LOG_ERR("Unsupported channel");
		res = -EINVAL;
		break;
	}

	if (res) {
		return res;
	}
	return icm42688_safely_configure(dev, &new_config);
}

static int icm42688_attr_get(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	const struct icm42688_dev_data *data = dev->data;
	const struct icm42688_cfg *cfg = &data->cfg;
	int res = 0;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			icm42688_accel_reg_to_hz(cfg->accel_odr, val);
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			icm42688_accel_reg_to_fs(cfg->accel_fs, val);
		} else {
			LOG_ERR("Unsupported attribute");
			res = -EINVAL;
		}
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			icm42688_gyro_reg_to_odr(cfg->gyro_odr, val);
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			icm42688_gyro_reg_to_fs(cfg->gyro_fs, val);
		} else {
			LOG_ERR("Unsupported attribute");
			res = -EINVAL;
		}
		break;
	case SENSOR_CHAN_ALL:
		if (attr == SENSOR_ATTR_BATCH_DURATION) {
			val->val1 = cfg->batch_ticks;
			val->val2 = 0;
		} else if ((enum sensor_attribute_icm42688)attr ==
			   SENSOR_ATTR_ICM42688_PIN9_FUNCTION) {
			val->val1 = cfg->pin9_function;
			val->val2 = cfg->rtc_freq;
		} else {
			LOG_ERR("Unsupported attribute");
			res = -EINVAL;
		}
		break;
	default:
		LOG_ERR("Unsupported channel");
		res = -EINVAL;
		break;
	}

	return res;
}

static DEVICE_API(sensor, icm42688_driver_api) = {
	.sample_fetch = icm42688_sample_fetch,
	.channel_get = icm42688_channel_get,
	.attr_set = icm42688_attr_set,
	.attr_get = icm42688_attr_get,
#ifdef CONFIG_ICM42688_TRIGGER
	.trigger_set = icm42688_trigger_set,
#endif
	.get_decoder = icm42688_get_decoder,
#ifdef CONFIG_SENSOR_ASYNC_API
	.submit = icm42688_submit,
#endif
};

int icm42688_init(const struct device *dev)
{
	struct icm42688_dev_data *data = dev->data;
	const struct icm42688_dev_cfg *cfg = dev->config;
	int res;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	if (icm42688_reset(dev)) {
		LOG_ERR("could not initialize sensor");
		return -EIO;
	}

#ifdef CONFIG_ICM42688_TRIGGER
	res = icm42688_trigger_init(dev);
	if (res != 0) {
		LOG_ERR("Failed to initialize triggers");
		return res;
	}
#endif

	res = icm42688_configure(dev, &data->cfg);
	if (res != 0) {
		LOG_ERR("Failed to configure");
		return res;
	}

	return 0;
}

#ifndef CONFIG_ICM42688_TRIGGER
void icm42688_lock(const struct device *dev)
{
	ARG_UNUSED(dev);
}
void icm42688_unlock(const struct device *dev)
{
	ARG_UNUSED(dev);
}
#endif

/* device defaults to spi mode 0/3 support */
#define ICM42688_SPI_CFG                                                                           \
	SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB

#define ICM42688_RTIO_DEFINE(inst)                                                                 \
	SPI_DT_IODEV_DEFINE(icm42688_spi_iodev_##inst, DT_DRV_INST(inst), ICM42688_SPI_CFG, 0U);   \
	RTIO_DEFINE(icm42688_rtio_##inst, 8, 4);

#define ICM42688_DT_CONFIG_INIT(inst)						\
	{									\
		.accel_pwr_mode = DT_INST_PROP(inst, accel_pwr_mode),		\
		.accel_fs = DT_INST_PROP(inst, accel_fs),			\
		.accel_odr = DT_INST_PROP(inst, accel_odr),			\
		.gyro_pwr_mode = DT_INST_PROP(inst, gyro_pwr_mode),		\
		.gyro_fs = DT_INST_PROP(inst, gyro_fs),				\
		.gyro_odr = DT_INST_PROP(inst, gyro_odr),			\
		.temp_dis = false,						\
		.fifo_en = IS_ENABLED(CONFIG_ICM42688_STREAM),			\
		.batch_ticks = 0,						\
		.fifo_hires = DT_INST_PROP(inst, fifo_hires),			\
		.interrupt1_drdy = false,					\
		.interrupt1_fifo_ths = false,					\
		.interrupt1_fifo_full = false,					\
		.pin9_function = ICM42688_PIN9_FUNCTION_INT2,			\
		.rtc_freq = 32000,						\
		.axis_align[0].index = DT_INST_PROP(inst, axis_align_x),	\
		.axis_align[1].index = DT_INST_PROP(inst, axis_align_y),	\
		.axis_align[2].index = DT_INST_PROP(inst, axis_align_z),	\
		.axis_align[0].sign = DT_INST_PROP(inst, axis_align_x_sign)-1,	\
		.axis_align[1].sign = DT_INST_PROP(inst, axis_align_y_sign)-1,	\
		.axis_align[2].sign = DT_INST_PROP(inst, axis_align_z_sign)-1	\
	}

#define ICM42688_DEFINE_DATA(inst)                                                                 \
	IF_ENABLED(CONFIG_ICM42688_STREAM, (ICM42688_RTIO_DEFINE(inst)));                          \
	static struct icm42688_dev_data icm42688_driver_##inst = {                                 \
		.cfg = ICM42688_DT_CONFIG_INIT(inst),                                              \
		IF_ENABLED(CONFIG_ICM42688_STREAM, (.r = &icm42688_rtio_##inst,                    \
						    .spi_iodev = &icm42688_spi_iodev_##inst,))     \
	};

#define ICM42688_INIT(inst)                                                                        \
	ICM42688_DEFINE_DATA(inst);                                                                \
                                                                                                   \
	static const struct icm42688_dev_cfg icm42688_cfg_##inst = {                               \
		.spi = SPI_DT_SPEC_INST_GET(inst, ICM42688_SPI_CFG, 0U),                           \
		.gpio_int1 = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                       \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icm42688_init, NULL, &icm42688_driver_##inst,           \
				     &icm42688_cfg_##inst, POST_KERNEL,                            \
				     CONFIG_SENSOR_INIT_PRIORITY, &icm42688_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM42688_INIT)
