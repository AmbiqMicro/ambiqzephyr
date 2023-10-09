/*
 * Copyright (c) 2019 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __CUSTOM_MSPI_AMBIQ_H__
#define __CUSTOM_MSPI_AMBIQ_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <am_mcu_apollo.h>

#define PINCTRL_STATE_QUAD PINCTRL_STATE_PRIV_START

/**
 * @brief SPI buffer structure
 *
 * @param send_addr Set true to indicate sending address before buf.
 * @param instr is instruction sent before address.
 * @param buf is a valid pointer on a data buffer, or NULL otherwise.
 * @param len is the length of the buffer or, if buf is NULL, will be the
 *    length which as to be sent as dummy bytes (as TX buffer) or
 *    the length of bytes that should be skipped (as RX buffer).
 */
struct mspi_buf {
	bool send_addr;
	uint16_t instr;
	uint32_t addr;
	size_t len;
	void *buf;
};

struct mspi_config {
	bool dqs_en;
	uint32_t dummy_cycles;
	struct spi_config generic_config;
};
/**
 * @typedef spi_api_io
 * @brief Callback API for write
 * See mspi_write() for argument descriptions
 */
typedef int (*mspi_api_wr)(const struct device *dev,
			  const struct mspi_buf *tx_bufs);

/**
 * @typedef spi_api_io
 * @brief Callback API for read
 * See mspi_read() for argument descriptions
 */
typedef int (*mspi_api_rd)(const struct device *dev,
			  const struct mspi_buf *rx_bufs);

/**
 * @typedef mspi_api_config
 * @brief Callback API for configuring SPI device.
 * See mspi_config() for argument descriptions
 */
typedef int (*mspi_api_config)(const struct device *dev,
			       const struct mspi_config *config);

/**
 * @typedef mspi_api_release
 * @brief Callback API for unlocking SPI device.
 * See mspi_release() for argument descriptions
 */
typedef int (*mspi_api_release)(const struct device *dev);


/**
 * @brief SPI driver API
 * This is the mandatory API any SPI driver needs to expose.
 */
__subsystem struct mspi_driver_api {
	mspi_api_wr write;
	mspi_api_rd read;
	mspi_api_config config;
	mspi_api_release release;
};

/**
 * @brief Write the specified amount of data from the SPI driver.
 *
 * @note This function is synchronous.
 *
 * @note This function is an helper function calling write.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param tx_bufs Buffer array where data to be sent originates from.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int custom_mspi_write(const struct device *dev,
			     const struct mspi_buf *tx_bufs);

static inline int z_impl_custom_mspi_write(const struct device *dev,
					const struct mspi_buf *tx_bufs)
{
	const struct mspi_driver_api *api =
		(const struct mspi_driver_api *)dev->api;
	int ret;

	ret = api->write(dev, tx_bufs);

	return ret;
}

/**
 * @brief Read the specified amount of data from the SPI driver.
 *
 * @note This function is synchronous.
 *
 * @note This function is an helper function calling read.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param rx_bufs Buffer array where data to be read will be written to.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int custom_mspi_read(const struct device *dev,
			     const struct mspi_buf *rx_bufs);

static inline int z_impl_custom_mspi_read(const struct device *dev,
					const struct mspi_buf *rx_bufs)
{
	const struct mspi_driver_api *api =
		(const struct mspi_driver_api *)dev->api;
	int ret;

	ret = api->read(dev, rx_bufs);

	return ret;
}

/**
 * @brief Configure the SPI device
 *
 * Note: This synchronous function is used to configure  SPI device, if
 * the device has been configured, this function disable it and re-configure
 * it again with new configuration.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid spi_config structure instance.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int custom_mspi_config(const struct device *dev,
			  const struct mspi_config *config);

static inline int z_impl_custom_mspi_config(const struct device *dev,
				     const struct mspi_config *config)
{
	const struct mspi_driver_api *api =
		(const struct mspi_driver_api *)dev->api;

	return api->config(dev, config);
}

/**
 * @brief Release the SPI device locked on and/or the CS by the current config
 *
 * Note: This synchronous function is used to release either the lock on the
 *       SPI device and/or the CS line that was kept if, and if only,
 *       given config parameter was the last one to be used (in any of the
 *       above functions) and if it has the SPI_LOCK_ON bit set and/or the
 *       SPI_HOLD_ON_CS bit set into its operation bits field.
 *       This can be used if the caller needs to keep its hand on the SPI
 *       device for consecutive transactions and/or if it needs the device to
 *       stay selected. Usually both bits will be used along each other, so the
 *       the device is locked and stays on until another operation is necessary
 *       or until it gets released with the present function.
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to a valid spi_config structure instance.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int custom_mspi_release(const struct device *dev);

static inline int z_impl_custom_mspi_release(const struct device *dev)
{
	const struct mspi_driver_api *api =
		(const struct mspi_driver_api *)dev->api;

	return api->release(dev);
}
#include <syscalls/custom_mspi_ambiq.h>

int custom_mspi_enable_xip(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_MSPI_AMBIQ_H__ */
