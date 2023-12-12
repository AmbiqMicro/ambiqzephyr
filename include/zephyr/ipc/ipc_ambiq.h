/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_IPC_IPC_AMBIQ_H_
#define ZEPHYR_INCLUDE_IPC_IPC_AMBIQ_H_

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/ipc/ipc_service.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IPC
 * @defgroup ipc IPC
 * @ingroup os_services
 * @{
 * @}
 */

/**
 * @brief IPC Service API
 * @defgroup ipc_ambiq_api IPC service APIs
 * @ingroup ipc
 * @{
 */

/** @brief Open an instance
 *
 *  Function to be used to open an instance before being able to register a new
 *  endpoint on it.
 *
 *  @param[in] instance Instance to open.
 *
 *  @retval -EINVAL when instance configuration is invalid.
 *  @retval -EIO when no backend is registered.
 *  @retval -EALREADY when the instance is already opened (or being opened).
 *
 *  @retval 0 on success or when not implemented on the backend (not needed).
 *  @retval other errno codes depending on the implementation of the backend.
 */
int ipc_ambiq_open_instance(const struct device *instance);

/** @brief Close an instance
 *
 *  Function to be used to close an instance. All bounded endpoints must be
 *  deregistered using ipc_ambiq_deregister_endpoint before this
 *  is called.
 *
 *  @param[in] instance Instance to close.
 *
 *  @retval -EINVAL when instance configuration is invalid.
 *  @retval -EIO when no backend is registered.
 *  @retval -EALREADY when the instance is not already opened.
 *  @retval -EBUSY when an endpoint exists that hasn't been
 *           deregistered
 *
 *  @retval 0 on success or when not implemented on the backend (not needed).
 *  @retval other errno codes depending on the implementation of the backend.
 */
int ipc_ambiq_close_instance(const struct device *instance);

/** @brief Register IPC endpoint onto an instance.
 *
 *  Registers IPC endpoint onto an instance to enable communication with a
 *  remote device.
 *
 *  The same function registers endpoints for both host and remote devices.
 *
 *  @param[in] instance Instance to register the endpoint onto.
 *  @param[in] ept Endpoint object.
 *  @param[in] cfg Endpoint configuration.
 *
 *  @note Keep the variable pointed by @p cfg alive when endpoint is in use.
 *
 *  @retval -EIO when no backend is registered.
 *  @retval -EINVAL when instance, endpoint or configuration is invalid.
 *  @retval -EBUSY when the instance is busy.
 *
 *  @retval 0 on success.
 *  @retval other errno codes depending on the implementation of the backend.
 */
int ipc_ambiq_register_endpoint(const struct device *instance, struct ipc_ept *ept,
				const struct ipc_ept_cfg *cfg);

/** @brief Deregister an IPC endpoint from its instance.
 *
 *  Deregisters an IPC endpoint from its instance.
 *
 *  The same function deregisters endpoints for both host and remote devices.
 *
 *  @param[in] ept Endpoint object.
 *
 *  @retval -EIO when no backend is registered.
 *  @retval -EINVAL when instance, endpoint or configuration is invalid.
 *  @retval -ENOENT when the endpoint is not registered with the instance.
 *  @retval -EBUSY when the instance is busy.
 *
 *  @retval 0 on success.
 *  @retval other errno codes depending on the implementation of the backend.
 */
int ipc_ambiq_deregister_endpoint(struct ipc_ept *ept);

/** @brief Send data using given IPC endpoint.
 *
 *  @param[in] ept Registered endpoint by @ref ipc_ambiq_register_endpoint.
 *  @param[in] data Pointer to the buffer to send.
 *  @param[in] len Number of bytes to send.
 *
 *  @retval -EIO when no backend is registered or send hook is missing from
 *               backend.
 *  @retval -EINVAL when instance or endpoint is invalid.
 *  @retval -ENOENT when the endpoint is not registered with the instance.
 *  @retval -EBADMSG when the data is invalid (i.e. invalid data format,
 *		     invalid length, ...)
 *  @retval -EBUSY when the instance is busy.
 *  @retval -ENOMEM when no memory / buffers are available.
 *
 *  @retval bytes number of bytes sent.
 *  @retval other errno codes depending on the implementation of the backend.
 */
int ipc_ambiq_send(struct ipc_ept *ept, const void *data, size_t len);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_IPC_IPC_AMBIQ_H_ */
