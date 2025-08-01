/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM4268X_SPI_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM4268X_SPI_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

/**
 * @brief perform a single SPI write to a ICM4268X register
 *
 * this functions wraps all logic necessary to write to any of the ICM4268X registers, regardless
 * of which memory bank the register belongs to.
 *
 * @param bus SPI bus pointer
 * @param reg address of ICM4268X register to write to
 * @param data data byte to write to register
 * @return int 0 on success, negative error code otherwise
 */
int icm4268x_spi_single_write(const struct spi_dt_spec *bus, uint16_t reg, uint8_t data);

/**
 * @brief update a single ICM4268X register value
 *
 * this functions wraps all logic necessary to update any of the ICM4268X registers, regardless
 * of which memory bank the register belongs to.
 *
 * @param bus SPI bus pointer
 * @param reg address of ICM4268X register to update
 * @param mask bitmask defining which bits of the register to update
 * @param data new value to update register with, respecting the bitmask
 * @return int 0 on success, negative error code otherwise
 */
int icm4268x_spi_update_register(const struct spi_dt_spec *bus, uint16_t reg, uint8_t mask,
				 uint8_t data);

/**
 * @brief read from one or more ICM4268X registers
 *
 * this functions wraps all logic necessary to read from any of the ICM4268X registers, regardless
 * of which memory bank the register belongs to.
 *
 * @param bus SPI bus pointer
 * @param reg start address of ICM4268X register(s) to read from
 * @param data pointer to byte array to read register values to
 * @param len number of bytes to read from the device
 * @return int 0 on success, negative error code otherwise
 */
int icm4268x_spi_read(const struct spi_dt_spec *bus, uint16_t reg, uint8_t *data, size_t len);

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM4268X_SPI_H_ */
