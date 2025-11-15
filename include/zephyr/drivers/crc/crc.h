/*
 * Copyright (c) 2025, Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CRC_H_
#define ZEPHYR_INCLUDE_DRIVERS_CRC_H_

/**
 * @brief CRC Interface
 * @defgroup crc_interface CRC Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/device.h>
#include <stdint.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Driver callback function for computing CRC32 over a memory region
 *
 * @param dev CRC device instance
 * @param start_addr Starting address of the memory region
 * @param size_bytes Size of the memory region in bytes
 * @param out_crc Pointer to store the computed CRC32 value
 *
 * @retval 0 On success
 * @retval -EINVAL Invalid parameters
 * @retval -EBUSY CRC engine is busy
 * @retval -ENOSYS Function not implemented
 */
typedef int (*crc_get_fn)(const struct device *dev,
			  const void *start_addr,
			  uint32_t size_bytes,
			  uint32_t *out_crc);

/**
 * @brief CRC driver API structure
 */
struct crc_driver_api {
	crc_get_fn get_crc;
};

/**
 * @brief Compute CRC32 checksum over a memory region
 *
 * This function computes a CRC32 checksum over the specified memory region
 * using hardware acceleration when available.
 *
 * @param dev Pointer to the CRC device
 * @param start_addr Starting address of the memory region
 * @param size_bytes Size of the memory region in bytes
 * @param out_crc Pointer to store the computed CRC32 value
 *
 * @retval 0 On success
 * @retval -EINVAL Invalid parameters (NULL pointers, invalid size)
 * @retval -EBUSY CRC engine is currently busy
 * @retval -ENOSYS Function not implemented by the driver
 */
static inline int crc_driver_get_crc(const struct device *dev,
				     const void *start_addr,
				     uint32_t size_bytes,
				     uint32_t *out_crc)
{
	const struct crc_driver_api *api;

	if (dev == NULL) {
		return -EINVAL;
	}

	api = (const struct crc_driver_api *)dev->api;
	if (!api || !api->get_crc) {
		return -ENOSYS;
	}

	return api->get_crc(dev, start_addr, size_bytes, out_crc);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_CRC_H_ */
