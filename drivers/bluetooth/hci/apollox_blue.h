/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Header file of Ambiq Apollox Blue SoC extended driver
 * for SPI based HCI.
 */
#ifndef ZEPHYR_DRIVERS_BLUETOOTH_HCI_APOLLOX_BLUE_H_
#define ZEPHYR_DRIVERS_BLUETOOTH_HCI_APOLLOX_BLUE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @typedef bt_spi_transceive_fun
 * @brief SPI transceive function for Bluetooth packet.
 *
 * @param tx Pointer of transmission packet.
 * @param tx_len Length of transmission packet.
 * @param rx Pointer of reception packet.
 * @param rx_len Length of reception packet.
 *
 * @return 0 on success or negative error number on failure.
 */
typedef int (*bt_spi_transceive_fun)(void *tx, uint32_t tx_len, void *rx, uint32_t rx_len);

/**
 * @typedef spi_transmit_fun
 * @brief Define the SPI transmission function.
 *
 * @param data Pointer of transmission packet.
 * @param len Length of transmission packet.
 *
 * @return 0 on success or negative error number on failure.
 */
typedef int (*spi_transmit_fun)(uint8_t *data, uint16_t len);

/**
 * @brief Initialize the required devices for HCI driver.
 *
 * The devices mainly include the required gpio (e.g. reset-gpios,
 * irq-gpios).
 *
 * @return 0 on success or negative error number on failure.
 */
int bt_apollo_dev_init(void);

/**
 * @brief Send the packets to BLE controller from host via SPI.
 *
 * @param data Pointer of transmission packet.
 * @param len  Length of transmission packet.
 * @param transceive SPI transceive function for Bluetooth packet.
 *
 * @return 0 on success or negative error number on failure.
 */
int bt_apollo_spi_send(uint8_t *data, uint16_t len, bt_spi_transceive_fun transceive);

/**
 * @brief Receive the packets sent from BLE controller to host via SPI.
 *
 * @param data Pointer of reception packet.
 * @param len  Pointer of reception packet length.
 * @param transceive  SPI transceive function for Bluetooth packet.
 *
 * @return 0 on success or negative error number on failure.
 */
int bt_apollo_spi_rcv(uint8_t *data, uint16_t *len, bt_spi_transceive_fun transceive);

/**
 * @brief Initialize the BLE controller.
 *
 * This step may do the necessary handshaking with the controller before
 * @param transmit SPI transmit function
 *
 * @return 0 on success or negative error number on failure.
 */
int bt_apollo_controller_init(spi_transmit_fun transmit);

/**
 * @brief Deinitialize the BLE controller.
 *
 * @return 0 on success or negative error number on failure.
 */
int bt_apollo_controller_deinit(void);

/**
 * @brief Vendor specific setup before general HCI command sequence for
 * Bluetooth application.
 *
 * @return 0 on success or negative error number on failure.
 */
int bt_apollo_vnd_setup(void);

/**
 * @brief Set the controller public BD address via a vendor-specific HCI
 * command.
 *
 * The address bytes follow HCI / Zephyr ``bt_addr_t`` convention: little
 * endian, byte[0] is the least-significant octet.
 *
 * Currently implemented for the EM9305-over-SPI path (SOC_APOLLO510B,
 * VSC opcode 0xFC43); other Apollo SoCs return -ENOTSUP.
 *
 * @param addr Pointer to a 6-byte BD address (LE order).
 *
 * @return 0 on success or negative error number on failure.
 */
int bt_apollo_set_public_addr(const uint8_t addr[6]);

/**
 * @brief Set the advertising TX power level via a vendor-specific HCI command.
 *
 * Applies to all advertising sets on the controller. Implemented for the
 * EM9305-over-SPI path (SOC_APOLLO510B, VSC opcode 0xFFF5); other Apollo SoCs
 * return -ENOTSUP.
 *
 * @param txpower_dbm Requested TX power in dBm. Valid range is greater than
 *                    -20 up to +6 inclusive; out-of-range values return
 *                    -EINVAL.
 *
 * @return 0 on success or negative error number on failure.
 */
int bt_apollo_set_adv_tx_power(int8_t txpower_dbm);

/**
 * @brief Set a connection's TX power level via a vendor-specific HCI command.
 *
 * Implemented for the EM9305-over-SPI path (SOC_APOLLO510B, VSC opcode
 * 0xFFF6); other Apollo SoCs return -ENOTSUP.
 *
 * @param conn_handle Controller connection handle.
 * @param txpower_dbm Requested TX power in dBm. Valid range is greater than
 *                    -20 up to +6 inclusive; out-of-range values return
 *                    -EINVAL.
 *
 * @return 0 on success or negative error number on failure.
 */
int bt_apollo_set_conn_tx_power(uint16_t conn_handle, int8_t txpower_dbm);

/**
 * @brief Check if vendor specific receiving handling is ongoing.
 *
 * @param data Pointer of received packet.
 *
 * @return true indicates if vendor specific receiving handling is ongoing.
 */
bool bt_apollo_vnd_rcv_ongoing(uint8_t *data, uint16_t len);

/**
 * @brief Report a received HCI ``Command Complete`` event to the driver.
 *
 * Called by the HCI driver from the RX path when a ``BT_HCI_EVT_CMD_COMPLETE``
 * is parsed. In ``CONFIG_BT_HCI_RAW`` builds on EM9305 (SOC_APOLLO510B) this
 * unblocks the driver's internal command pacing for vendor-specific commands
 * sent from ``.setup`` (e.g. set-public-addr / LL-features / TX-power), which
 * otherwise have no host-stack wait facility available.
 *
 * In all other configurations this function is a no-op.
 *
 * @param opcode HCI opcode contained in the Command Complete event (host
 *               byte order).
 * @param status Status byte from the Command Complete payload (0 = success).
 */
void bt_apollo_vsc_cc_observe(uint16_t opcode, uint8_t status);

/**
 * @brief Do the specific preprocessing in HCI packet receiving ISR if needed,
 * for example, clear the interrupt status.
 */
void bt_apollo_rcv_isr_preprocess(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_BLUETOOTH_HCI_APOLLOX_BLUE_H_ */
