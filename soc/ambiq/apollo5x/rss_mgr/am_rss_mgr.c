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
#include <string.h>

#include <am_mcu_apollo.h>
#include <hal/am_hal_info.h>
#include <am_rss_mgr.h>

LOG_MODULE_REGISTER(rss_mgr);

BUILD_ASSERT(IS_ENABLED(CONFIG_DT_HAS_VND_MBOX_CONSUMER_ENABLED),
	     "Apollo510L radio subsystem management needs vendor mbox consumer support");

#define IPC_TX_POOL_BUF_SIZE (256)
#define IPC_TX_POOL_BUF_NUM  (1)

/*
 * For TrimVer >= 2.4, RF trim data is stored in two separate INFO1 sections
 * with a reserved gap between them.
 */
#define AM_RSS_RF_TRIM_SECTION0_SIZE     0x90
#define AM_RSS_RF_TRIM_RESERVED_GAP_SIZE 0xb0
#define AM_RSS_RF_TRIM_SECTION1_SIZE     0x50
#define AM_RSS_RF_TRIM_SECTION1_OFFSET                                                             \
	(AM_RSS_RF_TRIM_SECTION0_SIZE + AM_RSS_RF_TRIM_RESERVED_GAP_SIZE)
#define AM_RSS_RF_TRIM_SIZE (AM_RSS_RF_TRIM_SECTION0_SIZE + AM_RSS_RF_TRIM_SECTION1_SIZE)

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
	uint32_t config[] = {AM_HAL_IPC_MBOX_SIGNAL_MSG_IPC_SHM_CONFIG_REQ, 3,
			     DT_REG_ADDR(DT_NODELABEL(sram_ipc)),
			     DT_REG_SIZE(DT_NODELABEL(sram_ipc)),
			     DT_PROP(DT_NODELABEL(ipc0), zephyr_buffer_size)};
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
	k_busy_wait(200);

	return buf;
}

struct am_ipc_sys_set_rftrim_req {
	struct am_ipc_packet_sys_common_hdr hdr;
	uint8_t data[AM_RSS_RF_TRIM_SIZE];
} __packed;

#if defined(CONFIG_BT_AMBIQ_RFXTAL_CUSTOM_CONFIG)
/* Offsets within the RF trim blob for XOCONFIG / OSCTRIM / RADIOTUNE */
#define AM_RSS_RF_TRIM_XTAL_CONFIG_OFFSET 0x04
#define AM_RSS_RF_TRIM_XTAL_TRIM_OFFSET   0x08
#define AM_RSS_RF_TRIM_RF_TUNE_OFFSET     0x0c

static void rss_rftrim_apply_custom_rfxtal(uint8_t *rf_trim)
{
	uint32_t xo_config = CONFIG_BT_AMBIQ_RFXTAL_XO_CONFIG;
	uint32_t radio_tune = CONFIG_BT_AMBIQ_RFXTAL_RADIO_TUNE;

	/*
	 * Overlay board RFXTAL values onto the Info1 RF-trim blob before
	 * SET_RFTRIM so later IPC trim programming does not undo
	 * AM_HAL_CLKMGR_CONFIGURE_RFXTAL.
	 */
	rf_trim[AM_RSS_RF_TRIM_XTAL_TRIM_OFFSET] = (uint8_t)CONFIG_BT_AMBIQ_RFXTAL_XO_FREQ_TRIM;
	memcpy(&rf_trim[AM_RSS_RF_TRIM_XTAL_CONFIG_OFFSET], &xo_config, sizeof(xo_config));
	memcpy(&rf_trim[AM_RSS_RF_TRIM_RF_TUNE_OFFSET], &radio_tune, sizeof(radio_tune));
}
#endif

static int rss_rftrim_info_read(uint8_t *rf_trim)
{
	am_hal_status_e st;
	uint32_t trim_main_ver = 0;
	uint32_t trim_sub_ver = 0;
	bool otp_powered = false;
	int ret = -ENOENT;

	if (PWRCTRL->DEVPWRSTATUS_b.PWRSTOTP == 0) {
		am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
		otp_powered = true;
	}

	st = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, AM_REG_OTP_INFO1_TRIM_REV_O / 4,
			       1, &trim_main_ver);
	st |= am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1,
				AM_REG_OTP_INFO1_PGM_INFO_O / 4, 1, &trim_sub_ver);
	if (st != AM_HAL_STATUS_SUCCESS) {
		ret = -EIO;
	} else if ((trim_main_ver > 2) || ((trim_main_ver == 2) && ((trim_sub_ver & 0x0f) >= 4))) {
		/* RF trim is available for Info1 TrimVer >= 2.4 */
		st = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1,
				       AM_REG_OTP_INFO1_RFTRIM_TRIM_VER_O / 4,
				       AM_RSS_RF_TRIM_SECTION0_SIZE / 4, (uint32_t *)rf_trim);
		if (st != AM_HAL_STATUS_SUCCESS) {
			ret = -EIO;
		} else if (rf_trim[0] != 0) {
			st = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1,
					       (AM_REG_OTP_INFO1_RFTRIM_TRIM_VER_O +
						AM_RSS_RF_TRIM_SECTION1_OFFSET) /
						       4,
					       AM_RSS_RF_TRIM_SECTION1_SIZE / 4,
					       (uint32_t *)&rf_trim[AM_RSS_RF_TRIM_SECTION0_SIZE]);
			if (st != AM_HAL_STATUS_SUCCESS) {
				ret = -EIO;
			} else {
#if defined(CONFIG_BT_AMBIQ_RFXTAL_CUSTOM_CONFIG)
				rss_rftrim_apply_custom_rfxtal(rf_trim);
#endif
				ret = 0;
			}
		}
	}

	if (otp_powered) {
		am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
	}

	return ret;
}

static void rss_rfxtal_configure(void)
{
	uint32_t status;

	/*
	 * Program RFXTAL before RSS boot so the HAL can push the config via
	 * mailbox after power-up.
	 */
#if defined(CONFIG_BT_AMBIQ_RFXTAL_CUSTOM_CONFIG)
	static const am_hal_clkmgr_rfxtal_config_t rfxtal_cfg = {
		.ui32XOFreqTrim = CONFIG_BT_AMBIQ_RFXTAL_XO_FREQ_TRIM,
		.ui32XOConfig = CONFIG_BT_AMBIQ_RFXTAL_XO_CONFIG,
		.ui32RadioTune = CONFIG_BT_AMBIQ_RFXTAL_RADIO_TUNE,
	};

	status = am_hal_clkmgr_control(AM_HAL_CLKMGR_CONFIGURE_RFXTAL, (void *)&rfxtal_cfg);
	if (status != AM_HAL_STATUS_SUCCESS) {
		/*
		 * HAL may still report a prior Info1 read status even after the
		 * custom config was cached; do not fail RSS bring-up.
		 */
		LOG_WRN("RFXTAL custom config status 0x%08x", status);
	} else {
		LOG_INF("RFXTAL custom config applied");
	}

#else
	status = am_hal_clkmgr_control(AM_HAL_CLKMGR_CONFIGURE_RFXTAL, NULL);
	if (status != AM_HAL_STATUS_SUCCESS) {
		LOG_WRN("RFXTAL Info1 trim unavailable (0x%08x); using RSS defaults", status);
	} else {
		LOG_INF("RFXTAL Info1 trim applied");
	}
#endif
}

struct net_buf *am_rss_mgr_rftrim_config(void)
{
	struct net_buf *buf;
	struct am_ipc_sys_set_rftrim_req req;
	int ret;

	memset(&req, 0, sizeof(req));

	ret = rss_rftrim_info_read(req.data);
	if (ret) {
		LOG_WRN("RF trim Info1 read failed (%d)", ret);
		return NULL;
	}

	req.data[1] = (uint8_t)MCUCTRL->CHIPREV;
	req.hdr.type = AM_IPC_PACKET_TYPE_SYS_REQ;
	req.hdr.opcode = AM_IPC_SYS_OP_SET_RFTRIM;
	req.hdr.size = AM_RSS_RF_TRIM_SIZE;

	buf = net_buf_alloc(&ipc_tx_pool, K_FOREVER);
	if (!buf) {
		LOG_ERR("Unable to allocate IPC TX buffer for RF trim");
		return NULL;
	}

	net_buf_add_mem(buf, &req, sizeof(req));
	k_busy_wait(200);

	return buf;
}

int am_rss_mgr_rss_enable(bool on)
{
	int ret = 0;

	if (on) {
		rss_rfxtal_configure();

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
