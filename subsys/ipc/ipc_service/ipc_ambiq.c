/*
 * Copyright (c) 2023 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ipc/ipc_service.h>
#include <zephyr/ipc/ipc_service_backend.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

LOG_MODULE_DECLARE(ipc_service, CONFIG_IPC_SERVICE_LOG_LEVEL);

#define WQ_STACK_SIZE         (512)
#define IPC_AMBIQ_WQ_PRIORITY (K_PRIO_COOP(1))
K_THREAD_STACK_DEFINE(ipc_ambiq_stack, WQ_STACK_SIZE);

struct ipc_ambiq_data {
	/* IPC work */
	struct k_work ipc_ambiq_work;
	struct k_work_q ipc_ambiq_wq;
};

static struct ipc_ambiq_data ipc_data;

int ipc_ambiq_open_instance(const struct device *instance)
{
	return ipc_service_open_instance(instance);
}

int ipc_ambiq_close_instance(const struct device *instance)
{
	return ipc_service_close_instance(instance);
}

int ipc_ambiq_register_endpoint(const struct device *instance, struct ipc_ept *ept,
				const struct ipc_ept_cfg *cfg)
{
	return ipc_service_register_endpoint(instance, ept, cfg);
}

int ipc_ambiq_deregister_endpoint(struct ipc_ept *ept)
{
	return ipc_service_deregister_endpoint(ept);
	;
}

int ipc_ambiq_send(struct ipc_ept *ept, const void *data, size_t len)
{
	return ipc_service_send(ept, data, len);
}

