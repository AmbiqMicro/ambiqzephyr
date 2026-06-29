/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file em9305_ll_features.h
 * @brief LE Local Supported Features bit table for the EM9305 BLE radio.
 *
 * Layout, naming, per-byte grouping and BT_53 / BT_54 guards mirror the
 * AmbiqSuite Cordio EM9305 driver reference
 * (third_party/cordio/ble-host/sources/hci/ambiq/em9305/hci_drv_em9305.h)
 * so that this header can be compared 1:1 with the upstream source.
 *
 * Each ``HCI_LE_SUP_FEAT_*`` macro is the absolute bit position in the
 * 64-bit LE feature field per Bluetooth Core Spec 5.4 Vol 6 Part B,
 * Table 4-5. ``UINT64_C`` is used so the ``>>32`` / ``>>40`` shifts in
 * the caller's array assembly are well-defined regardless of ``int``
 * width (AmbiqSuite relies on Cordio's ``hci_defs.h`` providing 64-bit
 * literals for the same reason).
 *
 * To opt in to BT 5.3 / 5.4 host-support feature bits when the EM9305
 * firmware in use advertises them, define ``EM9305_BT_53`` and / or
 * ``EM9305_BT_54`` to 1 before including this header (or globally via
 * the build system).
 */

#ifndef ZEPHYR_DRIVERS_BLUETOOTH_HCI_EM9305_LL_FEATURES_H_
#define ZEPHYR_DRIVERS_BLUETOOTH_HCI_EM9305_LL_FEATURES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef EM9305_BT_53
#define EM9305_BT_53 0
#endif
#ifndef EM9305_BT_54
#define EM9305_BT_54 0
#endif

/* Individual LE feature bits (absolute position in the 64-bit feature
 * field). Names mirror Cordio hci_defs.h.
 */
#define HCI_LE_SUP_FEAT_ENCRYPTION           UINT64_C(0x0000000000000001) /* bit  0 */
#define HCI_LE_SUP_FEAT_CONN_PARAM_REQ_PROC  UINT64_C(0x0000000000000002) /* bit  1 */
#define HCI_LE_SUP_FEAT_EXT_REJECT_IND       UINT64_C(0x0000000000000004) /* bit  2 */
#define HCI_LE_SUP_FEAT_SLV_INIT_FEAT_EXCH   UINT64_C(0x0000000000000008) /* bit  3 */
#define HCI_LE_SUP_FEAT_LE_PING              UINT64_C(0x0000000000000010) /* bit  4 */
#define HCI_LE_SUP_FEAT_DATA_LEN_EXT         UINT64_C(0x0000000000000020) /* bit  5 */
#define HCI_LE_SUP_FEAT_PRIVACY              UINT64_C(0x0000000000000040) /* bit  6 */
#define HCI_LE_SUP_FEAT_EXT_SCAN_FILT_POLICY UINT64_C(0x0000000000000080) /* bit  7 */

#define HCI_LE_SUP_FEAT_LE_2M_PHY               UINT64_C(0x0000000000000100) /* bit  8 */
#define HCI_LE_SUP_FEAT_STABLE_MOD_IDX_RECEIVER UINT64_C(0x0000000000000400) /* bit 10 */
#define HCI_LE_SUP_FEAT_LE_CODED_PHY            UINT64_C(0x0000000000000800) /* bit 11 */
#define HCI_LE_SUP_FEAT_LE_EXT_ADV              UINT64_C(0x0000000000001000) /* bit 12 */
#define HCI_LE_SUP_FEAT_LE_PER_ADV              UINT64_C(0x0000000000002000) /* bit 13 */
#define HCI_LE_SUP_FEAT_CH_SEL_2                UINT64_C(0x0000000000004000) /* bit 14 */

#define HCI_LE_SUP_FEAT_MIN_NUN_USED_CHAN UINT64_C(0x0000000000010000) /* bit 16 */

#define HCI_LE_SUP_FEAT_PAST_SENDER               UINT64_C(0x0000000001000000) /* bit 24 */
#define HCI_LE_SUP_FEAT_PAST_RECIPIENT            UINT64_C(0x0000000002000000) /* bit 25 */
#define HCI_LE_SUP_FEAT_SCA_UPDATE                UINT64_C(0x0000000004000000) /* bit 26 */
#define HCI_LE_SUP_FEAT_REMOTE_PUB_KEY_VALIDATION UINT64_C(0x0000000008000000) /* bit 27 */

#define HCI_LE_SUP_FEAT_POWER_CONTROL_REQUEST  UINT64_C(0x0000000200000000) /* bit 33 */
#define HCI_LE_SUP_FEAT_POWER_CHANGE_IND       UINT64_C(0x0000000400000000) /* bit 34 */
#define HCI_LE_SUP_FEAT_PATH_LOSS_MONITOR      UINT64_C(0x0000000800000000) /* bit 35 */
#define HCI_LE_SUP_FEAT_PRR_ADV_ADI            UINT64_C(0x0000001000000000) /* bit 36 */
#define HCI_LE_SUP_FEAT_SUBRATING              UINT64_C(0x0000002000000000) /* bit 37 */
#define HCI_LE_SUP_FEAT_SUBRATING_HOST_SUPPORT UINT64_C(0x0000004000000000) /* bit 38 */
#define HCI_LE_SUP_FEAT_CHAN_CLASSIFICATION    UINT64_C(0x0000008000000000) /* bit 39 */

#define HCI_LE_SUP_FEAT_ADV_CODING_SELECTION UINT64_C(0x0000010000000000) /* bit 40 */
#define HCI_LE_SUP_FEAT_ADV_CODING_SELECTION_HOST_SUPPORT                                          \
	UINT64_C(0x0000020000000000) /* bit 41                                                     \
				      */
/*
 * Bit 42, HCI_LE_SUP_DECISION_BASE_ADV_FILTER, is marked as Reserved in
 * Bluetooth Spec v5.4.
 */
#define HCI_LE_SUP_FEAT_PER_ADV_WITH_RESP_ADVERTISER UINT64_C(0x0000080000000000) /* bit 43 */
#define HCI_LE_SUP_FEAT_PER_ADV_WITH_RESP_SCANNER    UINT64_C(0x0000100000000000) /* bit 44 */

/* Per-byte feature composites. Mirror AmbiqSuite hci_drv_em9305.h. */
#define LL_FEATURES_BYTE0                                                                          \
	(HCI_LE_SUP_FEAT_ENCRYPTION | HCI_LE_SUP_FEAT_CONN_PARAM_REQ_PROC |                        \
	 HCI_LE_SUP_FEAT_EXT_REJECT_IND | HCI_LE_SUP_FEAT_SLV_INIT_FEAT_EXCH |                     \
	 HCI_LE_SUP_FEAT_LE_PING | HCI_LE_SUP_FEAT_DATA_LEN_EXT | HCI_LE_SUP_FEAT_PRIVACY |        \
	 HCI_LE_SUP_FEAT_EXT_SCAN_FILT_POLICY)

#define LL_FEATURES_BYTE1                                                                          \
	(HCI_LE_SUP_FEAT_LE_2M_PHY | HCI_LE_SUP_FEAT_STABLE_MOD_IDX_RECEIVER |                     \
	 HCI_LE_SUP_FEAT_LE_CODED_PHY | HCI_LE_SUP_FEAT_LE_EXT_ADV | HCI_LE_SUP_FEAT_LE_PER_ADV |  \
	 HCI_LE_SUP_FEAT_CH_SEL_2)

#define LL_FEATURES_BYTE2 (HCI_LE_SUP_FEAT_MIN_NUN_USED_CHAN)

#define LL_FEATURES_BYTE3                                                                          \
	(HCI_LE_SUP_FEAT_PAST_SENDER | HCI_LE_SUP_FEAT_PAST_RECIPIENT |                            \
	 HCI_LE_SUP_FEAT_SCA_UPDATE | HCI_LE_SUP_FEAT_REMOTE_PUB_KEY_VALIDATION)

#if ((EM9305_BT_53) || (EM9305_BT_54))
#define LL_FEATURES_BYTE4                                                                          \
	(HCI_LE_SUP_FEAT_POWER_CONTROL_REQUEST | HCI_LE_SUP_FEAT_POWER_CHANGE_IND |                \
	 HCI_LE_SUP_FEAT_PATH_LOSS_MONITOR | HCI_LE_SUP_FEAT_PRR_ADV_ADI |                         \
	 HCI_LE_SUP_FEAT_SUBRATING | HCI_LE_SUP_FEAT_SUBRATING_HOST_SUPPORT |                      \
	 HCI_LE_SUP_FEAT_CHAN_CLASSIFICATION)
#else
#define LL_FEATURES_BYTE4                                                                          \
	(HCI_LE_SUP_FEAT_POWER_CONTROL_REQUEST | HCI_LE_SUP_FEAT_POWER_CHANGE_IND |                \
	 HCI_LE_SUP_FEAT_PATH_LOSS_MONITOR | HCI_LE_SUP_FEAT_PRR_ADV_ADI |                         \
	 HCI_LE_SUP_FEAT_SUBRATING | HCI_LE_SUP_FEAT_CHAN_CLASSIFICATION)
#endif /* EM9305_BT_53 */

#if (EM9305_BT_54)
#define LL_FEATURES_BYTE5                                                                          \
	(HCI_LE_SUP_FEAT_ADV_CODING_SELECTION |                                                    \
	 HCI_LE_SUP_FEAT_ADV_CODING_SELECTION_HOST_SUPPORT |                                       \
	 HCI_LE_SUP_FEAT_PER_ADV_WITH_RESP_ADVERTISER | HCI_LE_SUP_FEAT_PER_ADV_WITH_RESP_SCANNER)
#endif /* EM9305_BT_54 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_BLUETOOTH_HCI_EM9305_LL_FEATURES_H_ */
