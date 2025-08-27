/*
 * Copyright (c) 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Ambiq radio subsystem manager functions. The radio subsystem is running
 * at another network processing core, which is only included in Apollo510L series
 * SoC now.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/logging/log.h>
#include <am_mcu_apollo.h>
#include <am_rss_mgr.h>

LOG_MODULE_REGISTER(rss_mgr);

BUILD_ASSERT(IS_ENABLED(CONFIG_DT_HAS_VND_MBOX_CONSUMER_ENABLED),
	     "Apollo510L radio subsystem management needs vendor mbox consumer support");

#define IPC_TX_POOL_BUF_SIZE (256)
#define IPC_TX_POOL_BUF_NUM  (1)

NET_BUF_POOL_FIXED_DEFINE(ipc_tx_pool, IPC_TX_POOL_BUF_NUM, IPC_TX_POOL_BUF_SIZE, 0, NULL);

static const struct mbox_dt_spec vnd_mbox_tx = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), tx);
static const struct mbox_dt_spec vnd_mbox_rx = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), rx);
static K_SEM_DEFINE(sem_ipc_cfg, 0, 1);
static bool ipc_shm_configured = false;

static void mbox_callback(const struct device *dev, mbox_channel_id_t channel_id, void *user_data,
			  struct mbox_msg *msg)
{
	uint32_t rxdata = 0;

	if (msg->size != sizeof(uint32_t)) {
		LOG_ERR("Received mbox message size is not expected");
		return;
	}

	if (channel_id != vnd_mbox_rx.channel_id) {
		LOG_ERR("Received channel id is incorrect");
		return;
	}

	memcpy(&rxdata, msg->data, msg->size);
	am_hal_ipc_mbox_service(rxdata);
}

static void ipc_shm_config_rsp_handler(void *pArg)
{
	k_sem_give(&sem_ipc_cfg);
}

static int vnd_mbox_init(void)
{
	int ret;

	am_hal_ipc_mbox_handler_register(AM_HAL_IPC_MBOX_SIGNAL_MSG_IPC_SHM_CONFIG_RSP,
					 ipc_shm_config_rsp_handler, NULL);

	ret = mbox_register_callback_dt(&vnd_mbox_rx, mbox_callback, NULL);
	if (ret < 0) {
		LOG_ERR("Could not register callback (%d)\n", ret);
		return ret;
	}

	ret = mbox_set_enabled_dt(&vnd_mbox_rx, true);
	if ((ret < 0) && (ret != -EALREADY)) {
		LOG_ERR("Could not enable RX channel %d (%d)\n", vnd_mbox_rx.channel_id, ret);
		return ret;
	}

	return 0;
}

int am_rss_mgr_ipc_shm_config(void)
{
	int ret = 0;
	struct mbox_msg rsp_msg = {0};
	uint32_t config[] = {AM_HAL_IPC_MBOX_SIGNAL_MSG_IPC_SHM_CONFIG_REQ, 2,
			     DT_REG_ADDR(DT_NODELABEL(sram_ipc)),
			     DT_REG_SIZE(DT_NODELABEL(sram_ipc))};

	/* Skip if the IPC share memory has been already configured */
	if (ipc_shm_configured) {
		return 0;
	}

	rsp_msg.size = sizeof(uint32_t);
	for (uint8_t i = 0; i < sizeof(config) / sizeof(uint32_t); i++) {
		rsp_msg.data = (void *)&config[i];
		mbox_send_dt(&vnd_mbox_tx, &rsp_msg);
	}

	ret = k_sem_take(&sem_ipc_cfg, K_SECONDS(2));
	if (ret) {
		LOG_ERR("IPC share memory configuration fail with %d", ret);
		return ret;
	}

	ipc_shm_configured = true;

	return 0;
}

struct net_buf *am_rss_mgr_opmode_config(am_rss_opmode_e opmode)
{
	struct net_buf *buf;
	struct am_ipc_sys_set_rss_opmode_req_t req;
	req.hdr.type = AM_IPC_PACKET_TYPE_SYS_REQ;
	req.hdr.opcode = AM_IPC_SYS_OP_SET_RSS_OPMODE;
	req.hdr.size = 1;
	req.mode = opmode;

	buf = net_buf_alloc(&ipc_tx_pool, K_FOREVER);
	if (!buf) {
		LOG_ERR("Unable to allocate a IPC TX buffer");
		return NULL;
	}

	net_buf_add_mem(buf, &req, sizeof(req));
	k_busy_wait(100);

	return buf;
}

int am_rss_mgr_rss_enable(bool on)
{
	int ret = 0;

	if (on) {
		/* Power up the radio subsystem */
		ret = am_hal_pwrctrl_rss_bootup();
		if ((ret != AM_HAL_STATUS_SUCCESS) && (ret != AM_HAL_STATUS_IN_USE)) {
			LOG_ERR("RSS bootup failed %d", ret);
			return -EIO;
		}

		ret = vnd_mbox_init();
	} else {
		uint32_t user_count = 0;

		/* Check if crystal in radio subsystem is used by other peripherals */
		ret = am_hal_clkmgr_clock_status_get(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, &user_count);
		if (ret != AM_HAL_STATUS_SUCCESS) {
			LOG_ERR("Clock status getting failed %d", ret);
			return -EIO;
		}

		if (user_count == 0) {
			ret = am_hal_pwrctrl_rss_pwroff();
			if (ret != AM_HAL_STATUS_SUCCESS) {
				LOG_ERR("RSS pwroff failed %d", ret);
				return -EIO;
			}
			ret = mbox_set_enabled_dt(&vnd_mbox_rx, false);

			ipc_shm_configured = false;
		}
	}

	return ret;
}
