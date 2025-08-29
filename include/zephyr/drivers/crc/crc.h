/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public CRC driver API used by Ambiq CRC driver.
 * Matches: int crc_ambiq_get_crc32(const struct device *dev,
 *                                 uint32_t startAddr,
 *                                 uint32_t sizeBytes,
 *                                 uint32_t *pCrc)
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CRC_H_
#define ZEPHYR_INCLUDE_DRIVERS_CRC_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Driver callback: compute CRC32 over a memory region */
typedef int (*crc_get_fn)(const struct device *dev,
			  uint32_t start_addr,
			  uint32_t size_bytes,
			  uint32_t *out_crc);

/* Public driver API struct */
struct crc_driver_api {
	crc_get_fn get_crc;
};

/* Convenience inline wrapper that calls the device API */
static inline int crc_driver_get_crc(const struct device *dev,
				     uint32_t start_addr,
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

#endif /* ZEPHYR_INCLUDE_DRIVERS_CRC_H_ */
