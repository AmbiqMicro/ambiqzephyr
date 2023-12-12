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

#define STATE_READY	(0)
#define STATE_BUSY	(1)
#define STATE_INITED	(2)

#define WQ_STACK_SIZE         (512)
#define IPC_AMBIQ_WQ_PRIORITY (K_PRIO_COOP(1))
K_THREAD_STACK_DEFINE(ipc_ambiq_stack, WQ_STACK_SIZE);

struct ipc_ambiq_data {
	/* IPC work */
	struct k_work ipc_tx_work;
	struct k_work ipc_rx_work;
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
	// struct ipc_ambiq_data *data = CONTAINER_OF(cb, struct ipc_ambiq_data, irq_gpio_cb);
	// err = k_work_submit_to_queue(&ipc_data.ipc_ambiq_wq, &(ipc_data.ipc_tx_work));
	// if (err < 0) {
	// 	LOG_ERR("IPC work submit failed=%d\n", err);
	// }
	return ipc_service_send(ept, data, len);
}

static void ipc_tx_work_process(struct k_work *item)
{
	// struct ipc_ambiq_data *data = CONTAINER_OF(item, struct ipc_ambiq_data, ipc_tx_work);
	// ipc_service_send(ept, data, len);
}

static void ipc_rx_work_process(struct k_work *item)
{
	struct ipc_ambiq_data *data = CONTAINER_OF(item, struct ipc_ambiq_data, ipc_rx_work);
	// ipc_service_send(ept, data, len);
}

static void mbox_callback(const struct device *instance, uint32_t channel,
			  void *user_data, struct mbox_msg *msg_data)
{
	struct ipc_ambiq_data *data = user_data;

	k_work_submit_to_queue(&ipc_data.ipc_ambiq_wq, &ipc_data.ipc_rx_work);
}

int ipc_ambiq_config_instance(const struct device *instance, struct mbox_channel *mbox_rx)
{
	int err;
	if(!device_is_ready(instance))
	{
		LOG_ERR("Instance is not ready");
		return -ENODEV;
	}

	err = mbox_register_callback(mbox_rx, mbox_callback, &ipc_data);
	if (err != 0) {
		LOG_ERR("Overload mbox isr failed:%d", err);
		return err;
	}
	/**
	 * Initialize work item for handling IPC serialized TX/RX
	 */
	if (!atomic_cas(&data->state, STATE_READY, STATE_BUSY)) {
		return -EALREADY;
	}
	err = k_work_queue_init(&ipc_data.ipc_ambiq_wq);
	if (err != 0) {
		LOG_ERR("Init ipc WQ failed:%d", err);
		return err;
	}
	err = k_work_queue_start(&ipc_data.ipc_ambiq_wq, ipc_ambiq_stack, WQ_STACK_SIZE,
			   IPC_AMBIQ_WQ_PRIORITY, NULL);
	if (err != 0) {
		LOG_ERR("Start ipc WQ failed:%d", err);
		return err;
	}
	err = k_work_init(&ipc_data.ipc_tx_work, ipc_tx_work_process);
	if (err != 0) {
		LOG_ERR("Init ipc work item failed:%d", err);
		return err;
	}
	err = k_work_init(&ipc_data.ipc_rx_work, ipc_rx_work_process);
	if (err != 0) {
		LOG_ERR("Init ipc work item failed:%d", err);
		return err;
	}
	atomic_set(&data->state, STATE_INITED);
}
