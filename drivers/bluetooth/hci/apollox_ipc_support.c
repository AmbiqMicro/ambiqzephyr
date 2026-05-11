/*
 * Copyright (c) 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
#include <zephyr/kernel.h>
#endif
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/net_buf.h>
#include <string.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>

#include <am_rss_mgr.h>

LOG_MODULE_REGISTER(bt_hci_apollox_ipc_support);

#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
/* CEVA/RSS 510L vendor opcodes (AmbiqSuite hci_vs_510L_radio / hci_drv_510L_radio) */
#define AMBIQ_HCI_OP_VS_510L_UPDATE_NVDS       BT_OP(BT_OGF_VS, 0x0080)
#define AMBIQ_HCI_OP_VS_510L_UPDATE_LL_FEATURE BT_OP(BT_OGF_VS, 0x0082)
#define AMBIQ_HCI_OP_VS_510L_SET_TX_POWER      BT_OP(BT_OGF_VS, 0x0083)

#define AMBIQ_NVDS_CFG_PAYLOAD_LEN    240U
#define AMBIQ_510L_LPCLK_DRIFT_PPM    500U
#define AMBIQ_510L_EXT_WAKEUP_TIME_US 800U
#define AMBIQ_510L_OSC_WAKEUP_TIME_US 800U
#define AMBIQ_510L_RM_WAKEUP_TIME_US  800U
#define AMBIQ_510L_POST_NVDS_DELAY_MS 1U
#define AMBIQ_510L_LL_FEATURE_MASK    0ULL
#define AMBIQ_510L_VS_TX_POWER_LEVEL  0

/* NVDS parameter tags (CEVA/RSS 510L; align with AmbiqSuite NVDS / hci_vs_510L_radio) */
#define AMBIQ_NVDS_PARAM_ID_LPCLK_DRIFT     0x07
#define AMBIQ_NVDS_PARAM_ID_EXT_WAKEUP_TIME 0x0d
#define AMBIQ_NVDS_PARAM_ID_OSC_WAKEUP_TIME 0x0e
#define AMBIQ_NVDS_PARAM_ID_RM_WAKEUP_TIME  0x0f
#define AMBIQ_NVDS_PARAM_ID_SLEEP_ENABLE    0x11
#define AMBIQ_NVDS_PARAM_ID_TRACER_CONFIG   0x2f
#define AMBIQ_NVDS_PARAM_ID_LE_QOS_ENABLE   0x51

static void ambiq_510l_nvds_add_u8(uint8_t *p, size_t *o, uint8_t param_id, uint8_t value)
{
	p[(*o)++] = param_id;
	p[(*o)++] = 0x06;
	p[(*o)++] = sizeof(value);
	p[(*o)++] = value;
}

static void ambiq_510l_nvds_add_le16(uint8_t *p, size_t *o, uint8_t param_id, uint16_t value)
{
	p[(*o)++] = param_id;
	p[(*o)++] = 0x06;
	p[(*o)++] = sizeof(value);
	sys_put_le16(value, &p[*o]);
	*o += sizeof(value);
}

static void ambiq_510l_nvds_add_le32(uint8_t *p, size_t *o, uint8_t param_id, uint32_t value)
{
	p[(*o)++] = param_id;
	p[(*o)++] = 0x06;
	p[(*o)++] = sizeof(value);
	sys_put_le32(value, &p[*o]);
	*o += sizeof(value);
}

static void ambiq_510l_build_nvds_sleep_cfg(uint8_t *p)
{
	size_t o = 0;

	/*
	 * NVDS payload signature: same byte sequence as NVDS_PARAMETER_MAGIC_NUMBER in
	 * Ambiq applet_configuration.c (0x4e, 0x56, 0x44, 0x53) = ASCII "NVDS".
	 * Marks the start of a controller NVDS configuration block for vendor HCI
	 * update (here: 510L VS 0xFC80; AmbiqSuite: HCI_DBG_UPDATE_NVDS_CFG_CMD_OPCODE).
	 * Tags that follow carry boot/runtime parameters (e.g. LP drift, wakeup times,
	 * sleep enable) consumed by the CEVA controller firmware. Full NVDS images in
	 * other tools can also include items such as BD address; this builder only
	 * emits the sleep-configuration tags below.
	 */
	p[o++] = 0x4e;
	p[o++] = 0x56;
	p[o++] = 0x44;
	p[o++] = 0x53;
	ambiq_510l_nvds_add_le16(p, &o, AMBIQ_NVDS_PARAM_ID_LPCLK_DRIFT,
				 AMBIQ_510L_LPCLK_DRIFT_PPM);
	ambiq_510l_nvds_add_le16(p, &o, AMBIQ_NVDS_PARAM_ID_EXT_WAKEUP_TIME,
				 AMBIQ_510L_EXT_WAKEUP_TIME_US);
	ambiq_510l_nvds_add_le16(p, &o, AMBIQ_NVDS_PARAM_ID_OSC_WAKEUP_TIME,
				 AMBIQ_510L_OSC_WAKEUP_TIME_US);
	ambiq_510l_nvds_add_le16(p, &o, AMBIQ_NVDS_PARAM_ID_RM_WAKEUP_TIME,
				 AMBIQ_510L_RM_WAKEUP_TIME_US);
	ambiq_510l_nvds_add_u8(p, &o, AMBIQ_NVDS_PARAM_ID_SLEEP_ENABLE, 1);
	ambiq_510l_nvds_add_u8(p, &o, AMBIQ_NVDS_PARAM_ID_LE_QOS_ENABLE, 0);
	ambiq_510l_nvds_add_le32(p, &o, AMBIQ_NVDS_PARAM_ID_TRACER_CONFIG, 0);

	__ASSERT_NO_MSG(o <= AMBIQ_NVDS_CFG_PAYLOAD_LEN);
	memset(&p[o], 0, AMBIQ_NVDS_CFG_PAYLOAD_LEN - o);
}

static int hci_vs_cmd_send_sync(uint16_t opcode, const void *param, size_t param_len)
{
	struct net_buf *buf = NULL;
	struct net_buf *rsp = NULL;
	int err;

	if (param_len != 0U) {
		buf = bt_hci_cmd_alloc(K_FOREVER);
		if (!buf) {
			return -ENOBUFS;
		}
		net_buf_add_mem(buf, param, param_len);
	}

	err = bt_hci_cmd_send_sync(opcode, buf, &rsp);

	if (rsp != NULL) {
		net_buf_unref(rsp);
	}

	return err;
}

static int ambiq_510l_send_nvds_and_pre_reset_vs(const struct device *dev)
{
	uint8_t nvds[AMBIQ_NVDS_CFG_PAYLOAD_LEN];
	struct net_buf *buf;
	struct net_buf *rsp = NULL;
	int err;

	ARG_UNUSED(dev);

	ambiq_510l_build_nvds_sleep_cfg(nvds);

	buf = bt_hci_cmd_alloc(K_FOREVER);
	if (!buf) {
		return -ENOBUFS;
	}

	net_buf_add_mem(buf, nvds, sizeof(nvds));
	err = bt_hci_cmd_send_sync(AMBIQ_HCI_OP_VS_510L_UPDATE_NVDS, buf, &rsp);
	if (rsp != NULL) {
		net_buf_unref(rsp);
	}
	if (err != 0) {
		return err;
	}
	k_sleep(K_MSEC(AMBIQ_510L_POST_NVDS_DELAY_MS));

	/*
	 * Mirror hci_vs_510L_radio ordering before HCI_Reset. Payloads must match
	 * AmbiqSuite hci_vs_510L_radio.c if the controller rejects these defaults.
	 */
	uint8_t ll_feat[8];

	sys_put_le64(AMBIQ_510L_LL_FEATURE_MASK, ll_feat);

	err = hci_vs_cmd_send_sync(AMBIQ_HCI_OP_VS_510L_UPDATE_LL_FEATURE, ll_feat,
				   sizeof(ll_feat));
	if (err != 0) {
		LOG_ERR("510L VS 0xFC82 failed: %d", err);
		return err;
	}

	const int8_t tx_pwr = AMBIQ_510L_VS_TX_POWER_LEVEL;

	err = hci_vs_cmd_send_sync(AMBIQ_HCI_OP_VS_510L_SET_TX_POWER, &tx_pwr, sizeof(tx_pwr));
	if (err != 0) {
		LOG_ERR("510L VS 0xFC83 failed: %d", err);
		return err;
	}

	return 0;
}
#endif /* CONFIG_SOC_APOLLO510L or CONFIG_SOC_APOLLO330P */

int bt_hci_transport_teardown(const struct device *dev)
{
	ARG_UNUSED(dev);

	int ret;

	/* Disable the radio subsystem */
	ret = am_rss_mgr_rss_enable(false);

	return ret;
}

int bt_hci_transport_setup(const struct device *dev)
{
	ARG_UNUSED(dev);

	int ret = 0;

	/* Enable the radio subsystem */
	ret = am_rss_mgr_rss_enable(true);
	if (ret != 0) {
		return ret;
	}

	ret = am_rss_mgr_ipc_shm_config();

	return ret;
}

int bt_ipc_setup(const struct device *dev, const struct bt_hci_setup_params *params)
{
	ARG_UNUSED(params);

	int ret;
	struct net_buf *buf;

#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
	buf = am_rss_mgr_rftrim_config();
	if (buf != NULL) {
		ret = bt_hci_send(dev, buf);
		if (ret != 0) {
			return ret;
		}
	} else {
		LOG_WRN("RF trim IPC skipped (no Info1 trim)");
	}
#endif

	buf = am_rss_mgr_opmode_config(AM_RSS_OPMODE_NP);
	if (!buf) {
		return -ENOBUFS;
	}

	ret = bt_hci_send(dev, buf);
	if (ret != 0) {
		return ret;
	}

#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
	ret = ambiq_510l_send_nvds_and_pre_reset_vs(dev);
	if (ret != 0) {
		LOG_ERR("510L CEVA sleep/VS sequence failed: %d", ret);
		return ret;
	}
#endif

	return 0;
}
