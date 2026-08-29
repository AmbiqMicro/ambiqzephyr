/*
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/logging/log.h>

#include <am_mcu_apollo.h>
#include <am_rss_mgr.h>

LOG_MODULE_REGISTER(rss_ipc, LOG_LEVEL_INF);

static const struct mbox_dt_spec rss_rx = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), rx);

static const char *const signal_names[] = {
	"M2D",
	"D2M",
	"RFXTAL_ON_REQ",
	"RFXTAL_ON_RSP",
	"RFXTAL_OFF_REQ",
	"RFXTAL_OFF_RSP",
	"RFXTAL_CONFIG_REQ",
	"RFXTAL_CONFIG_RSP",
	"IPC_SHM_CONFIG_REQ",
	"IPC_SHM_CONFIG_RSP",
	"RSS_SLEEP_DURATION_NTF",
};

static void rss_mbox_callback(const struct device *dev, mbox_channel_id_t channel_id,
			      void *user_data, struct mbox_msg *msg)
{
	uint32_t word = 0;

	if ((msg == NULL) || (msg->size != sizeof(word))) {
		return;
	}
	memcpy(&word, msg->data, sizeof(word));

	if ((word >= AM_HAL_IPC_MBOX_SIGNAL_MSG_START) && (word < AM_HAL_IPC_MBOX_SIGNAL_MSG_END)) {
		LOG_INF("radio says 0x%08x %s", word,
			signal_names[word - AM_HAL_IPC_MBOX_SIGNAL_MSG_START]);
	} else {
		LOG_INF("radio says 0x%08x", word);
	}

	am_hal_ipc_mbox_service(word);
}

int main(void)
{
	uint32_t status;
	int64_t t0;
	int ret;

	LOG_INF("Ambiq RSS IPC sample on %s", CONFIG_BOARD_TARGET);

	ret = mbox_register_callback_dt(&rss_rx, rss_mbox_callback, NULL);
	if (ret < 0) {
		LOG_ERR("callback registration failed (%d)", ret);
		return 0;
	}

	t0 = k_uptime_get();
	ret = am_rss_mgr_ipc_shm_config();
	if (ret != 0) {
		LOG_ERR("shm handshake failed (%d)", ret);
		return 0;
	}
	LOG_INF("shm handshake answered in %lld ms", k_uptime_get() - t0);

	t0 = k_uptime_get();
	status = am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_XTAL_HS,
					     AM_HAL_CLKMGR_USER_ID_RESV0);
	if (status == AM_HAL_STATUS_SUCCESS) {
		LOG_INF("crystal granted in %lld ms", k_uptime_get() - t0);
		status = am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_XTAL_HS,
						     AM_HAL_CLKMGR_USER_ID_RESV0);
		LOG_INF("crystal released (status %u)", status);
	} else {
		LOG_INF("crystal request not available (status %u)", status);
	}

	LOG_INF("RSS IPC sample complete");

	return 0;
}
