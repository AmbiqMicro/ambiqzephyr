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
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/net_buf.h>
#include <string.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>

#include <am_rss_mgr.h>
#include <am_mcu_apollo.h>

#include "apollox_ipc_support.h"

LOG_MODULE_REGISTER(bt_hci_apollox_ipc_support);

#if defined(CONFIG_SOC_APOLLO510L) || defined(CONFIG_SOC_APOLLO330P)
/* CEVA/RSS 510L vendor opcodes (AmbiqSuite hci_vs_510L_radio / hci_drv_510L_radio) */
#define AMBIQ_HCI_OP_VS_510L_UPDATE_NVDS       BT_OP(BT_OGF_VS, 0x0080)
#define AMBIQ_HCI_OP_VS_510L_SET_BD_ADDR       BT_OP(BT_OGF_VS, 0x0081)
#define AMBIQ_HCI_OP_VS_510L_UPDATE_LL_FEATURE BT_OP(BT_OGF_VS, 0x0082)
#define AMBIQ_HCI_OP_VS_510L_SET_TX_POWER      BT_OP(BT_OGF_VS, 0x0083)
#define AMBIQ_HCI_OP_VS_510L_GET_CON_EVT_CNT   BT_OP(BT_OGF_VS, 0x0085)
#define AMBIQ_HCI_OP_VS_510L_GET_DTM_RSSI      BT_OP(BT_OGF_VS, 0x0086)

#define AMBIQ_NVDS_CFG_PAYLOAD_LEN          240U
#define APOLLO_330P_510L_LPCLK_DRIFT_PPM    500U
#define APOLLO_330P_510L_EXT_WAKEUP_TIME_US 800U
#define APOLLO_330P_510L_OSC_WAKEUP_TIME_US 800U
#define APOLLO_330P_510L_RM_WAKEUP_TIME_US  800U
#define APOLLO_330P_510L_POST_NVDS_DELAY_MS 1U

/* NVDS parameter tags (CEVA/RSS 510L; align with AmbiqSuite NVDS / hci_vs_510L_radio) */
#define AMBIQ_NVDS_PARAM_ID_BD_ADDRESS       0x01
#define AMBIQ_NVDS_PARAM_ID_LPCLK_DRIFT      0x07
#define AMBIQ_NVDS_PARAM_ID_EXT_WAKEUP_TIME  0x0d
#define AMBIQ_NVDS_PARAM_ID_OSC_WAKEUP_TIME  0x0e
#define AMBIQ_NVDS_PARAM_ID_RM_WAKEUP_TIME   0x0f
#define AMBIQ_NVDS_PARAM_ID_SLEEP_ENABLE     0x11
#define AMBIQ_NVDS_PARAM_ID_TRACER_CONFIG    0x2f
#define AMBIQ_NVDS_PARAM_ID_MEM_WRITE_ENABLE 0x50
#define AMBIQ_NVDS_PARAM_ID_LE_QOS_ENABLE    0x51

#define APOLLO_330P_510L_TRACER_CONFIG 0x00080000U

static void apollo_330p_510l_nvds_add_u8(uint8_t *p, size_t *o, uint8_t param_id, uint8_t value)
{
	p[(*o)++] = param_id;
	p[(*o)++] = 0x06;
	p[(*o)++] = sizeof(value);
	p[(*o)++] = value;
}

static void apollo_330p_510l_nvds_add_le16(uint8_t *p, size_t *o, uint8_t param_id, uint16_t value)
{
	p[(*o)++] = param_id;
	p[(*o)++] = 0x06;
	p[(*o)++] = sizeof(value);
	sys_put_le16(value, &p[*o]);
	*o += sizeof(value);
}

static void apollo_330p_510l_nvds_add_le32(uint8_t *p, size_t *o, uint8_t param_id, uint32_t value)
{
	p[(*o)++] = param_id;
	p[(*o)++] = 0x06;
	p[(*o)++] = sizeof(value);
	sys_put_le32(value, &p[*o]);
	*o += sizeof(value);
}

static void apollo_330p_510l_nvds_add_addr(uint8_t *p, size_t *o, uint8_t param_id,
					   const bt_addr_t *addr)
{
	p[(*o)++] = param_id;
	p[(*o)++] = 0x06;
	p[(*o)++] = sizeof(addr->val);
	memcpy(&p[*o], addr->val, sizeof(addr->val));
	*o += sizeof(addr->val);
}

static int apollo_330p_510l_get_bd_address(const struct bt_hci_setup_params *params,
					   bt_addr_t *addr)
{
	am_hal_mcuctrl_device_t device;
	uint32_t status;

	if (!bt_addr_eq(&params->public_addr, BT_ADDR_ANY)) {
		bt_addr_copy(addr, &params->public_addr);
		return 0;
	}

	status = am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &device);
	if (status != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("Failed to read MCU device ID: 0x%08x", status);
		return -EIO;
	}

	memcpy(&addr->val[0], &device.ui32ChipID1, sizeof(device.ui32ChipID1));
	addr->val[4] = (device.ui32ChipID0 >> 8) & 0xff;
	addr->val[5] = (device.ui32ChipID0 >> 16) & 0xff;

	/*
	 * Match AmbiqSuite: clear U/L and I/G bits for Android compatibility.
	 */
	addr->val[5] &= 0xfc;

	return 0;
}

static void apollo_330p_510l_build_nvds_cfg(uint8_t *p, const bt_addr_t *addr)
{
	size_t o = 0;

	/*
	 * NVDS payload signature: same byte sequence as NVDS_PARAMETER_MAGIC_NUMBER in
	 * Ambiq applet_configuration.c (0x4e, 0x56, 0x44, 0x53) = ASCII "NVDS".
	 * Marks the start of a controller NVDS configuration block for vendor HCI
	 * update (here: 510L VS 0xFC80; AmbiqSuite: HCI_DBG_UPDATE_NVDS_CFG_CMD_OPCODE).
	 * Tags that follow carry boot/runtime parameters (e.g. LP drift, wakeup times,
	 * sleep enable) consumed by the CEVA controller firmware.
	 */
	p[o++] = 0x4e;
	p[o++] = 0x56;
	p[o++] = 0x44;
	p[o++] = 0x53;
	apollo_330p_510l_nvds_add_addr(p, &o, AMBIQ_NVDS_PARAM_ID_BD_ADDRESS, addr);
	apollo_330p_510l_nvds_add_u8(p, &o, AMBIQ_NVDS_PARAM_ID_MEM_WRITE_ENABLE, 1);
	apollo_330p_510l_nvds_add_le16(p, &o, AMBIQ_NVDS_PARAM_ID_LPCLK_DRIFT,
				       APOLLO_330P_510L_LPCLK_DRIFT_PPM);
	apollo_330p_510l_nvds_add_le16(p, &o, AMBIQ_NVDS_PARAM_ID_EXT_WAKEUP_TIME,
				       APOLLO_330P_510L_EXT_WAKEUP_TIME_US);
	apollo_330p_510l_nvds_add_le16(p, &o, AMBIQ_NVDS_PARAM_ID_OSC_WAKEUP_TIME,
				       APOLLO_330P_510L_OSC_WAKEUP_TIME_US);
	apollo_330p_510l_nvds_add_le16(p, &o, AMBIQ_NVDS_PARAM_ID_RM_WAKEUP_TIME,
				       APOLLO_330P_510L_RM_WAKEUP_TIME_US);
	apollo_330p_510l_nvds_add_u8(p, &o, AMBIQ_NVDS_PARAM_ID_SLEEP_ENABLE, 1);
	apollo_330p_510l_nvds_add_u8(p, &o, AMBIQ_NVDS_PARAM_ID_LE_QOS_ENABLE, 0);
	apollo_330p_510l_nvds_add_le32(p, &o, AMBIQ_NVDS_PARAM_ID_TRACER_CONFIG,
				       APOLLO_330P_510L_TRACER_CONFIG);

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

static int apollo_330p_510l_vsc_update_nvds_param(const uint8_t nvds[AMBIQ_NVDS_CFG_PAYLOAD_LEN])
{
	return hci_vs_cmd_send_sync(AMBIQ_HCI_OP_VS_510L_UPDATE_NVDS, nvds,
				    AMBIQ_NVDS_CFG_PAYLOAD_LEN);
}

static void apollo_330p_510l_build_ll_features(uint8_t ll_features[8])
{
	ll_features[0] = (uint8_t)APOLLO_330P_510L_LL_FEATURES_BYTE0;
	ll_features[1] = (uint8_t)(APOLLO_330P_510L_LL_FEATURES_BYTE1 >> 8);
	ll_features[2] = (uint8_t)(APOLLO_330P_510L_LL_FEATURES_BYTE2 >> 16);
	ll_features[3] = (uint8_t)(APOLLO_330P_510L_LL_FEATURES_BYTE3 >> 24);
	ll_features[4] = (uint8_t)(APOLLO_330P_510L_LL_FEATURES_BYTE4 >> 32);
	ll_features[5] = (uint8_t)(APOLLO_330P_510L_LL_FEATURES_BYTE5 >> 40);
	ll_features[6] = (uint8_t)(APOLLO_330P_510L_LL_FEATURES_BYTE6 >> 48);
	ll_features[7] = (uint8_t)(APOLLO_330P_510L_LL_FEATURES_BYTE7 >> 56);
}

static int apollo_330p_510l_vsc_update_link_layer_feature(void)
{
	uint8_t ll_features[8];

	apollo_330p_510l_build_ll_features(ll_features);

	return hci_vs_cmd_send_sync(AMBIQ_HCI_OP_VS_510L_UPDATE_LL_FEATURE, ll_features,
				    sizeof(ll_features));
}

static int
apollo_330p_510l_vsc_update_tx_power_level(apollo_330p_510l_tx_power_level_t tx_power_level)
{
	const int8_t tx_pwr = tx_power_level;

	return hci_vs_cmd_send_sync(AMBIQ_HCI_OP_VS_510L_SET_TX_POWER, &tx_pwr, sizeof(tx_pwr));
}

int apollo_330p_510l_vsc_set_bd_address(const bt_addr_t *addr)
{
	return hci_vs_cmd_send_sync(AMBIQ_HCI_OP_VS_510L_SET_BD_ADDR, addr->val, sizeof(addr->val));
}

int apollo_330p_510l_vsc_get_conn_event_counter(uint16_t conn_handle)
{
	uint8_t param[sizeof(conn_handle)];

	sys_put_le16(conn_handle, param);

	return hci_vs_cmd_send_sync(AMBIQ_HCI_OP_VS_510L_GET_CON_EVT_CNT, param, sizeof(param));
}

int apollo_330p_510l_vsc_get_dtm_rssi(void)
{
	return hci_vs_cmd_send_sync(AMBIQ_HCI_OP_VS_510L_GET_DTM_RSSI, NULL, 0);
}

static int apollo_330p_510l_send_nvds_and_pre_reset_vs(const struct device *dev,
						       const struct bt_hci_setup_params *params)
{
	uint8_t nvds[AMBIQ_NVDS_CFG_PAYLOAD_LEN];
	bt_addr_t bd_addr;
	int err;

	ARG_UNUSED(dev);

	err = apollo_330p_510l_get_bd_address(params, &bd_addr);
	if (err != 0) {
		return err;
	}

	apollo_330p_510l_build_nvds_cfg(nvds, &bd_addr);

	err = apollo_330p_510l_vsc_update_nvds_param(nvds);
	if (err != 0) {
		return err;
	}
	k_sleep(K_MSEC(APOLLO_330P_510L_POST_NVDS_DELAY_MS));

	/*
	 * Mirror hci_vs_510L_radio ordering before HCI_Reset. Payloads must match
	 * AmbiqSuite hci_vs_510L_radio.c if the controller rejects these defaults.
	 */
	err = apollo_330p_510l_vsc_update_link_layer_feature();
	if (err != 0) {
		LOG_ERR("510L VS 0xFC82 failed: %d", err);
		return err;
	}

	err = apollo_330p_510l_vsc_update_tx_power_level(APOLLO_330P_510L_TX_POWER_LEVEL_DEFAULT);
	if (err != 0) {
		LOG_ERR("510L VS 0xFC83 failed: %d", err);
		return err;
	}

	err = apollo_330p_510l_vsc_set_bd_address(&bd_addr);
	if (err != 0) {
		LOG_ERR("510L VS 0xFC81 failed: %d", err);
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
	ret = apollo_330p_510l_send_nvds_and_pre_reset_vs(dev, params);
	if (ret != 0) {
		LOG_ERR("510L CEVA sleep/VS sequence failed: %d", ret);
		return ret;
	}
#endif

	return 0;
}
