/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Ambiq Apollox Blue SoC extended driver for SPI based HCI.
 */

#define DT_DRV_COMPAT ambiq_bt_hci_spi

#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_raw.h>
#include <zephyr/bluetooth/bluetooth.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_apollox_driver);

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_ambiq.h>

#include <soc.h>
#include "apollox_blue.h"
#if (CONFIG_SOC_SERIES_APOLLO4X)
#include "am_devices_cooper.h"
#elif (CONFIG_SOC_SERIES_APOLLO3X)
#include "am_apollo3_bt_support.h"
#elif (CONFIG_SOC_APOLLO510B)
#include "am_devices_em9305.h"
/* Apollo510B supports BT 5.3 and BT 5.4 */
#define EM9305_BT_53 1
#define EM9305_BT_54 1
#include "em9305_ll_features.h"
#endif /* CONFIG_SOC_SERIES_APOLLO4X */

#define HCI_SPI_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ambiq_bt_hci_spi)
#define SPI_DEV_NODE DT_BUS(HCI_SPI_NODE)

#if (CONFIG_SOC_APOLLO510B)
#define CLK_32M_NODE DT_NODELABEL(xo32m_xtal)
#define CLK_32K_NODE DT_NODELABEL(xo32k_xtal)
#else
#define CLK_32M_NODE DT_NODELABEL(xo32m)
#define CLK_32K_NODE DT_NODELABEL(xo32k)
#endif /* CONFIG_SOC_APOLLO510B */
/* Command/response for SPI operation */
#define SPI_WRITE   0x80
#define SPI_READ    0x04
#define READY_BYTE0 0x68
#define READY_BYTE1 0xA8

/* Maximum attempts of SPI write */
#define SPI_WRITE_TIMEOUT 200

#define SPI_MAX_RX_MSG_LEN 258

#if (CONFIG_SOC_APOLLO510B)
#define EM9305_SPI_T_RDY_US              1U
#define EM9305_SPI_RDY_LOW_DETECT_MAX_US 20U

#define EM9305_CM_TIMER         11U
#define EM9305_CM_PAD_CT_FNCSEL 6U
#define EM9305_CM_PWM_COMPARE0  50U /* HFRC/64 = 1.5 MHz -> 30 kHz period */
#define EM9305_CM_PWM_COMPARE1  25U /* 50 % duty cycle                    */

/* EM9305 vendor-specific HCI commands sent during controller bring-up.
 */
#define HCI_VSC_SET_LOCAL_SUP_FEAT_CMD_OPCODE 0xFFF2U
#define HCI_VSC_SET_LOCAL_SUP_FEAT_CMD_LENGTH 8U
#define HCI_VSC_SET_TX_POWER_LEVEL_CMD_OPCODE 0xFCC4U
#define HCI_VSC_SET_TX_POWER_LEVEL_CMD_LENGTH 1U
#define HCI_VSC_SET_DEV_PUB_ADDR_CMD_OPCODE   0xFC43U
#define HCI_VSC_SET_DEV_PUB_ADDR_CMD_LENGTH   6U
#define HCI_VSC_SET_SLEEP_OPTION_CMD_OPCODE   0xFC49U
#define HCI_VSC_SET_SLEEP_OPTION_CMD_LENGTH   1U
#define HCI_VSC_SET_ADV_TX_POWER_CMD_OPCODE   0xFFF5U
#define HCI_VSC_SET_ADV_TX_POWER_CMD_LENGTH   1U
#define HCI_VSC_SET_CONN_TX_POWER_CMD_OPCODE  0xFFF6U
#define HCI_VSC_SET_CONN_TX_POWER_CMD_LENGTH  3U

/* Default radio TX power for EM9305 (0 dBm) */
#define EM9305_TX_POWER_DEFAULT 0x00

/* Valid EM9305 TX power range in dBm: greater than -20, up to +6 inclusive.
 */
#define EM9305_TX_POWER_MIN_DBM  (-20)
#define EM9305_TX_POWER_MAX_DBM  (6)
#define EM9305_VSC_CC_TIMEOUT_MS 2000U

/* HCI heartbeat: send a benign HCI command every 10 s when the EM9305 is
 * active to prevent it from going silent and to detect a hung controller
 * early.
 */
#define EM9305_HEARTBEAT_INTERVAL_MS 10000U

#if (CONFIG_SOC_APOLLO510B) && !defined(CONFIG_BT_HCI_RAW)
#define EM9305_HEARTBEAT_ENABLED 1
#else
#define EM9305_HEARTBEAT_ENABLED 0
#endif

#if !defined(CONFIG_BT_TRANSMIT_POWER_CONTROL)
#define EM9305_LL_FEAT_BYTE4_NO_POWER_CTRL ((uint8_t)((LL_FEATURES_BYTE4 >> 32) & ~0x07U))
#else
#define EM9305_LL_FEAT_BYTE4_NO_POWER_CTRL ((uint8_t)(LL_FEATURES_BYTE4 >> 32))
#endif

/* 8-byte mask actually sent via VSC 0xFFF2 right after HCI_Reset */
static const uint8_t em9305_ll_feats[HCI_VSC_SET_LOCAL_SUP_FEAT_CMD_LENGTH] = {
	(uint8_t)(LL_FEATURES_BYTE0),
	(uint8_t)(LL_FEATURES_BYTE1 >> 8),
	(uint8_t)(LL_FEATURES_BYTE2 >> 16),
	(uint8_t)(LL_FEATURES_BYTE3 >> 24),
	EM9305_LL_FEAT_BYTE4_NO_POWER_CTRL,
	(uint8_t)(LL_FEATURES_BYTE5 >> 40),
	0,
	0,
};

static const struct gpio_dt_spec irq_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, irq_gpios);
static const struct gpio_dt_spec rst_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, reset_gpios);
static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET(SPI_DEV_NODE, cs_gpios);
static const struct gpio_dt_spec cm_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, cm_gpios);
/* AP5_12M_CLKREQ: output pin that requests the EM9305 to drive its 12 MHz
 * reference clock onto the Apollo5 RF subsystem clock input.  Must be
 * asserted before radio use and de-asserted on shutdown.
 */
static const struct gpio_dt_spec clkreq_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, clkreq_gpios);

static struct gpio_callback irq_gpio_cb;

#if EM9305_HEARTBEAT_ENABLED
/* Heartbeat work item: sends a benign HCI_Read_Local_Version_Information
 * command every EM9305_HEARTBEAT_INTERVAL_MS to keep the EM9305 active and
 * detect a hung controller before it corrupts the BLE link.
 */
static void bt_em9305_heartbeat_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(em9305_heartbeat_work, bt_em9305_heartbeat_work_handler);

static void bt_em9305_heartbeat_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	struct net_buf *buf = bt_hci_cmd_alloc(K_NO_WAIT);

	if (buf) {
		bt_hci_cmd_send(BT_HCI_OP_READ_LOCAL_VERSION_INFO, buf);
	}
	k_work_reschedule(&em9305_heartbeat_work, K_MSEC(EM9305_HEARTBEAT_INTERVAL_MS));
}

void bt_apollo_heartbeat_restart(void)
{
	/* Defer the next heartbeat command by a full interval from now.
	 * Called on every successful SPI TX or RX so the heartbeat fires
	 * only after EM9305_HEARTBEAT_INTERVAL_MS of genuine HCI silence.
	 */
	k_work_reschedule(&em9305_heartbeat_work, K_MSEC(EM9305_HEARTBEAT_INTERVAL_MS));
}
#else
void bt_apollo_heartbeat_restart(void)
{
}
#endif /* EM9305_HEARTBEAT_ENABLED */

#if EM9305_HEARTBEAT_ENABLED
/* Radio-level recovery work item.
 *
 * Runs from the system workqueue so it executes in thread context — required
 * by bt_disable() / bt_enable() which must not be called from an ISR or from
 * bt_spi_rx_thread itself (bt_disable() aborts that thread).
 *
 * Flow:
 *   1. bt_disable()  — closes HCI transport (bt_apollo_close → deinit EM9305,
 *                      stop heartbeat, de-assert CLKREQ), disconnects all
 *                      connections, triggers application disconnected callbacks.
 *   2. bt_enable()   — re-opens HCI transport (bt_apollo_open → reinit EM9305,
 *                      restart RX thread), re-runs HCI reset sequence.
 *
 * The application layer handles re-advertising / re-scanning through its
 * normal disconnected/ready callbacks — no MCU reboot required.
 */
static void bt_em9305_radio_recovery_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_ERR("EM9305: starting radio-level recovery (bt_disable + bt_enable)");

	int err = bt_disable();

	if (err) {
		LOG_ERR("EM9305: bt_disable failed (%d)", err);
		return;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("EM9305: bt_enable failed (%d)", err);
	} else {
		LOG_INF("EM9305: radio recovery complete");
	}
}

static K_WORK_DEFINE(em9305_recovery_work, bt_em9305_radio_recovery_handler);

void bt_apollo_schedule_radio_recovery(void)
{
	/* Cancel any pending heartbeat — it will be restarted after bt_enable
	 * completes and bt_apollo_vnd_setup() runs again.
	 */
	k_work_cancel_delayable(&em9305_heartbeat_work);
	/* Submit to system workqueue; safe to call from any thread/ISR. */
	k_work_submit(&em9305_recovery_work);
}
#else
void bt_apollo_schedule_radio_recovery(void)
{
	/* NO-OP */
}
#endif /* EM9305_HEARTBEAT_ENABLED */

static void bt_em9305_hsclk_req(bool enable)
{
	gpio_pin_set_dt(&clkreq_gpio, enable ? 1 : 0);
}

extern void bt_packet_irq_isr(const struct device *unused1, struct gpio_callback *unused2,
			      uint32_t unused3);

/* GPIO wrapper functions for EM9305 device driver */
static bool irq_pin_state(void)
{
	int pin_state = gpio_pin_get_dt(&irq_gpio);

	return pin_state > 0;
}

static void bt_em9305_set_reset(bool state)
{
	gpio_pin_set_dt(&rst_gpio, state);
}

static bool bt_em9305_get_reset(void)
{
	int pin_state = gpio_pin_get_dt(&rst_gpio);

	return pin_state > 0;
}

static void bt_em9305_cs_set(void)
{
	gpio_pin_set_dt(&cs_gpio, 1);
}

static void bt_em9305_cs_release(void)
{
	gpio_pin_set_dt(&cs_gpio, 0);
}

static void bt_em9305_set_cm(bool state)
{
	gpio_pin_set_dt(&cm_gpio, state ? 1 : 0);
}

static void bt_em9305_cm_pwm_ctrl(bool enable)
{
	if (enable) {
		am_hal_timer_config_t timer_cfg;
		am_hal_gpio_pincfg_t ct_cfg = am_hal_gpio_pincfg_output;

		am_hal_timer_default_config_set(&timer_cfg);
		timer_cfg.eFunction = AM_HAL_TIMER_FN_PWM;
		timer_cfg.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV64;
		timer_cfg.ui32Compare0 = EM9305_CM_PWM_COMPARE0;
		timer_cfg.ui32Compare1 = EM9305_CM_PWM_COMPARE1;
		am_hal_timer_config(EM9305_CM_TIMER, &timer_cfg);

		/* Route the chosen CTIMER's OUT0 to the CM pad, then switch the
		 * pad's pinmux to its CT (timer output) function.
		 */
		am_hal_timer_output_config(cm_gpio.pin, AM_HAL_TIMER_OUTPUT_TMR11_OUT0);

		ct_cfg.GP.cfg_b.uFuncSel = EM9305_CM_PAD_CT_FNCSEL;
		am_hal_gpio_pinconfig(cm_gpio.pin, ct_cfg);

		am_hal_timer_enable(EM9305_CM_TIMER);
	} else {
		am_hal_timer_disable(EM9305_CM_TIMER);
		gpio_pin_configure_dt(&cm_gpio, GPIO_INPUT);
	}
}

static bool em9305_poll_rdy(bool high)
{
	for (uint32_t i = 0; i < WAIT_EM9305_RDY_TIMEOUT; i++) {
		if (irq_pin_state() == high) {
			return true;
		}

		k_busy_wait(100);
	}

	return false;
}

static bool em9305_wait_spi_ready_for_header(void)
{
	if (irq_pin_state()) {
		for (uint8_t t = 0; t < EM9305_SPI_RDY_LOW_DETECT_MAX_US; t++) {
			k_busy_wait(1);
			if (!irq_pin_state()) {
				break;
			}
		}
	}

	return em9305_poll_rdy(true);
}

static bool em9305_spi_begin(void)
{
	bt_em9305_cs_set();
	k_busy_wait(EM9305_SPI_T_RDY_US);

	if (em9305_wait_spi_ready_for_header()) {
		return true;
	}

	bt_em9305_cs_release();
	return false;
}

bool bt_apollo_irq_pending(void)
{
	return irq_pin_state();
}

int bt_apollo_spi_send(uint8_t *pui8Values, uint16_t ui32NumBytes, bt_spi_transceive_fun transceive)
{
	return am_devices_em9305_blocking_write(pui8Values, ui32NumBytes, transceive);
}
#endif /* CONFIG_SOC_APOLLO510B */

#if (CONFIG_SOC_SERIES_APOLLO4X)
static const struct gpio_dt_spec irq_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, irq_gpios);
static const struct gpio_dt_spec rst_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, reset_gpios);
static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET(SPI_DEV_NODE, cs_gpios);
static const struct gpio_dt_spec clkreq_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, clkreq_gpios);

static struct gpio_callback irq_gpio_cb;
static struct gpio_callback clkreq_gpio_cb;

static const struct device *clk32m_dev = DEVICE_DT_GET(CLK_32M_NODE);
static const struct device *clk32k_dev = DEVICE_DT_GET(CLK_32K_NODE);
#endif /* CONFIG_SOC_SERIES_APOLLO4X */

extern void bt_packet_irq_isr(const struct device *unused1, struct gpio_callback *unused2,
			      uint32_t unused3);

void bt_apollo_rcv_isr_preprocess(void)
{
#if (CONFIG_SOC_SERIES_APOLLO3X)
	am_apollo3_bt_isr_pre();
#endif /* CONFIG_SOC_SERIES_APOLLO3X */
}

#if (CONFIG_SOC_SERIES_APOLLO4X)
static bool irq_pin_state(void)
{
	int pin_state;

	pin_state = gpio_pin_get_dt(&irq_gpio);
	LOG_DBG("IRQ Pin: %d", pin_state);
	return pin_state > 0;
}

static bool clkreq_pin_state(void)
{
	int pin_state;

	pin_state = gpio_pin_get_dt(&clkreq_gpio);
	LOG_DBG("CLKREQ Pin: %d", pin_state);
	return pin_state > 0;
}

static void bt_clkreq_isr(const struct device *unused1, struct gpio_callback *unused2,
			  uint32_t unused3)
{
	if (clkreq_pin_state()) {
		/* Enable XO32MHz */
		clock_control_on(clk32m_dev,
				 (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE);
		gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_FALLING);
	} else {
		/* Disable XO32MHz */
		clock_control_off(clk32m_dev,
				  (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE);
		gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_RISING);
	}
}

static void bt_apollo_controller_ready_wait(void)
{
	gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);
	k_busy_wait(200);
	PINCTRL_DT_DEFINE(SPI_DEV_NODE);
	pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(SPI_DEV_NODE), PINCTRL_STATE_DEFAULT);
	k_busy_wait(2000);
}

static void bt_apollo_controller_reset(void)
{
	/* Reset the controller*/
	gpio_pin_set_dt(&rst_gpio, 1);

	/* Take controller out of reset */
	k_sleep(K_MSEC(10));
	gpio_pin_set_dt(&rst_gpio, 0);

	/* Give the controller some time to boot */
	k_sleep(K_MSEC(500));
}
#endif /* CONFIG_SOC_SERIES_APOLLO4X */

#if !(CONFIG_SOC_APOLLO510B)
int bt_apollo_spi_send(uint8_t *data, uint16_t len, bt_spi_transceive_fun transceive)
{
	int ret = -ENOTSUP;

#if (CONFIG_SOC_SERIES_APOLLO4X)
	uint8_t command[1] = {SPI_WRITE};
	uint8_t response[2] = {0, 0};
	uint16_t fail_count = 0;
	bool sent = false;

	do {
		/* Check if the controller is ready to receive the HCI packets. */
		ret = transceive(command, 1, response, 2);
		if ((response[0] != READY_BYTE0) || (response[1] != READY_BYTE1) || ret) {
			bt_apollo_controller_ready_wait();
		} else {
			/* Transmit the message */
			ret = transceive(data, len, NULL, 0);
			if (ret) {
				LOG_ERR("SPI write error %d", ret);
			}
			sent = (ret == 0);
			break;
		}
	} while (fail_count++ < SPI_WRITE_TIMEOUT);

	if (!sent && (ret == 0)) {
		LOG_ERR("SPI write timeout waiting for controller ready");
		ret = -ETIMEDOUT;
	}
#elif (CONFIG_SOC_SERIES_APOLLO3X)
	ret = transceive(data, len, NULL, 0);
	if ((ret) && (ret != AM_HAL_BLE_STATUS_SPI_NOT_READY)) {
		LOG_ERR("SPI write error %d", ret);
	}
#endif /* CONFIG_SOC_SERIES_APOLLO4X */

	return ret;
}
#endif /* CONFIG_SOC_APOLLO510B */

int bt_apollo_spi_rcv(uint8_t *data, uint16_t *len, bt_spi_transceive_fun transceive)
{
#if (CONFIG_SOC_APOLLO510B)
	{
		uint8_t sCommand[2] = {EM9305_SPI_HEADER_RX, 0x0};
		uint8_t sStas[2] = {0};
		uint8_t ui8RxBytes = 0;
		uint8_t ret = 0;

		*len = 0;
		/* Check if the SPI is free */
		if (am_devices_em9305_get_spi_tx_status()) {
			/* TX in progress -> Ignore RDY interrupt */
			LOG_ERR("EM9305 SPI TX in progress");
			return AM_DEVICES_EM9305_TX_BUSY;
		}

		/* Check if they are still data to read */
		if (!irq_pin_state()) {
			/* No data */
			return AM_DEVICES_EM9305_NO_DATA_TX;
		}

		uint32_t read_packet_count = 0U;

		do {
			for (uint32_t i = 0; i < EM9305_STS_CHK_CNT_MAX; i++) {
				if (!em9305_spi_begin()) {
					if (*len != 0) {
						return AM_DEVICES_EM9305_STATUS_SUCCESS;
					}
					return AM_DEVICES_EM9305_NOT_READY;
				}

				ret = transceive(sCommand, 2, sStas, 2);

				if (ret != AM_HAL_STATUS_SUCCESS) {
					bt_em9305_cs_release();
					return AM_DEVICES_EM9305_CMD_TRANSFER_ERROR;
				}

				/* Check if the EM9305 is ready and the tx data size. */
				if ((sStas[0] == EM9305_STS1_READY_VALUE) && (sStas[1] != 0x00)) {
					break;
				}
				bt_em9305_cs_release();
			}

			/* Check that the EM9305 is ready or the receive FIFO is not full. */
			if ((sStas[0] != EM9305_STS1_READY_VALUE) || (sStas[1] == 0x00)) {
				bt_em9305_cs_release();
				if (*len != 0) {
					return AM_DEVICES_EM9305_STATUS_SUCCESS;
				}
				if ((sStas[0] == EM9305_STS1_READY_VALUE) && (sStas[1] == 0x00)) {
					LOG_DBG("EM9305 RX not ready yet (0x%02x 0x%02x)", sStas[0],
						sStas[1]);
				} else if ((sStas[0] == 0xff) && (sStas[1] == 0xff)) {
					LOG_DBG("EM9305 RX status not valid yet (0x%02x 0x%02x)",
						sStas[0], sStas[1]);
				} else {
					LOG_WRN("EM9305 unexpected RX status (0x%02x 0x%02x)",
						sStas[0], sStas[1]);
				}
				return AM_DEVICES_EM9305_NOT_READY;
			}

			/* Set the number of bytes to receive. */
			ui8RxBytes = sStas[1];

			if (irq_pin_state() && (ui8RxBytes != 0)) {
				/* Controller -> host RX path: worst case is one HCI Event
				 * (1 H4 + 2 hdr + 255 param = 258).
				 */
				if ((*len + ui8RxBytes) > EM9305_HCI_MAX_RX_LEN) {
					bt_em9305_cs_release();
					if (*len != 0) {
						LOG_DBG("EM9305 RX buffer full (have %u, +%u > "
							"%u); deferring remainder",
							(unsigned int)*len,
							(unsigned int)ui8RxBytes,
							(unsigned int)EM9305_HCI_MAX_RX_LEN);
						return AM_DEVICES_EM9305_STATUS_SUCCESS;
					}
					LOG_ERR("HCI RX packet too large: have %u, +%u > %u (%02x "
						"%02x)",
						(unsigned int)*len, (unsigned int)ui8RxBytes,
						(unsigned int)EM9305_HCI_MAX_RX_LEN, sStas[0],
						sStas[1]);
					return AM_DEVICES_EM9305_DATA_LENGTH_ERROR;
				}

				/* Read to the IOM. */
				ret = transceive(NULL, 0, data + *len, ui8RxBytes);

				if (ret != AM_HAL_STATUS_SUCCESS) {
					bt_em9305_cs_release();
					LOG_ERR(" bt_apollo_spi_rcv ret =%d\n", ret);
					return AM_DEVICES_EM9305_DATA_TRANSFER_ERROR;
				}
				*len += ui8RxBytes;
			}
			/* Deselect the EM9305 */
			bt_em9305_cs_release();

		} while (irq_pin_state() && (++read_packet_count < 4U));

		return AM_DEVICES_EM9305_STATUS_SUCCESS;
	}
#else
	int ret = -ENOTSUP;
	uint8_t response[2] = {0, 0};
	uint16_t read_size = 0;

	do {
#if (CONFIG_SOC_SERIES_APOLLO4X)
		/* Skip if the IRQ pin is not in high state */
		if (!irq_pin_state()) {
			ret = -1;
			break;
		}

		/* Check the available packet bytes */
		uint8_t command[1] = {SPI_READ};
		ret = transceive(command, 1, response, 2);
		if (ret) {
			break;
		}
#elif (CONFIG_SOC_SERIES_APOLLO3X)
		/* Skip if the IRQ bit is not set */
		if (!BLEIFn(0)->BSTATUS_b.BLEIRQ) {
			ret = -1;
			break;
		}

		/* Check the available packet bytes */
		ret = transceive(NULL, 0, response, 2);
		if (ret) {
			break;
		}
#else
		break;
#endif /* CONFIG_SOC_SERIES_APOLLO4X */

		/* Check if the read size is acceptable */
		read_size = (uint16_t)(response[0] | response[1] << 8);
		if ((read_size == 0) || (read_size > SPI_MAX_RX_MSG_LEN)) {
			ret = -1;
			break;
		}

		*len = read_size;

		/* Read the HCI data from controller */
		ret = transceive(NULL, 0, data, read_size);

		if (ret) {
			LOG_ERR("SPI read error %d", ret);
			break;
		}
	} while (0);

	return ret;
#endif
}

bool bt_apollo_vnd_rcv_ongoing(uint8_t *data, uint16_t len)
{
#if (CONFIG_SOC_SERIES_APOLLO4X)
	/* The vendor specific handshake command/response is incompatible with
	 * standard Bluetooth HCI format, need to handle the received packets
	 * specifically.
	 */
	if (am_devices_cooper_get_initialize_state() != AM_DEVICES_COOPER_STATE_INITIALIZED) {
		am_devices_cooper_handshake_recv(data, len);
		return true;
	} else {
		return false;
	}
#elif (CONFIG_SOC_APOLLO510B)
	{
		/* Match the 4-byte active-state vendor event {0x04,0xFF,0x01,0x01}. */
		static const uint8_t active_state_evt[] = {0x04, 0xFF, 0x01, 0x01};

		if ((len < sizeof(active_state_evt)) ||
		    (memcmp(data, active_state_evt, sizeof(active_state_evt)) != 0)) {
			/* Not an active-state event — forward to host stack. */
			return false;
		}

		/* Let the HAL update its init flag and determine whether this is the
		 * expected first boot event or a spontaneous mid-session reset.
		 */
		bool was_init = am_devices_em9305_check_active_state_event(data, len);

		if (!was_init) {
			LOG_ERR("EM9305 reset mid-session; scheduling radio recovery");
			bt_apollo_schedule_radio_recovery();
		}

		return true;
	}
#else
	return false;
#endif /* CONFIG_SOC_SERIES_APOLLO4X */
}

int bt_hci_transport_setup(const struct device *dev)
{
	ARG_UNUSED(dev);
	int ret = 0;

#if (CONFIG_SOC_APOLLO510B)
	/* Register GPIO operations for EM9305 device driver */
	am_devices_em9305_register_gpio_ops(bt_em9305_set_reset, bt_em9305_get_reset, irq_pin_state,
					    bt_em9305_cs_set, bt_em9305_cs_release);
	am_devices_em9305_register_cm_gpio(bt_em9305_set_cm);
	am_devices_em9305_register_cm_pwm_ops(bt_em9305_cm_pwm_ctrl);

	/* Configure RST pin and hold BLE in Reset */
	ret = gpio_pin_configure_dt(&rst_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&cm_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	/* Configure IRQ pin and register the callback */
	ret = gpio_pin_configure_dt(&irq_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&irq_gpio_cb, bt_packet_irq_isr, BIT(irq_gpio.pin));
	ret = gpio_add_callback(irq_gpio.port, &irq_gpio_cb);
	if (ret) {
		return ret;
	}

	/* Configure the interrupt edge for IRQ pin */
	gpio_pin_interrupt_configure_dt(&irq_gpio, GPIO_INT_EDGE_RISING);

	bt_em9305_hsclk_req(true);
#elif (CONFIG_SOC_SERIES_APOLLO4X)
	/* Configure the XO32MHz and XO32kHz clocks.*/
	clock_control_configure(clk32k_dev, NULL, NULL);
	clock_control_configure(clk32m_dev, NULL, NULL);

	/* Enable XO32kHz for Controller */
	clock_control_on(clk32k_dev, (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_LFXTAL);

	/* Enable XO32MHz for Controller */
	clock_control_on(clk32m_dev, (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE);

	/* Configure RST pin and hold BLE in Reset */
	ret = gpio_pin_configure_dt(&rst_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		return ret;
	}

	/* Configure IRQ pin and register the callback */
	ret = gpio_pin_configure_dt(&irq_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&irq_gpio_cb, bt_packet_irq_isr, BIT(irq_gpio.pin));
	ret = gpio_add_callback(irq_gpio.port, &irq_gpio_cb);
	if (ret) {
		return ret;
	}

	/* Configure CLKREQ pin and register the callback */
	ret = gpio_pin_configure_dt(&clkreq_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&clkreq_gpio_cb, bt_clkreq_isr, BIT(clkreq_gpio.pin));
	ret = gpio_add_callback(clkreq_gpio.port, &clkreq_gpio_cb);
	if (ret) {
		return ret;
	}

	/* Configure the interrupt edge for CLKREQ pin */
	gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_RISING);

	/* Take controller out of reset */
	k_sleep(K_MSEC(10));
	gpio_pin_set_dt(&rst_gpio, 0);

	/* Give the controller some time to boot */
	k_sleep(K_MSEC(500));

	/* Configure the interrupt edge for IRQ pin */
	gpio_pin_interrupt_configure_dt(&irq_gpio, GPIO_INT_EDGE_RISING);
#elif (CONFIG_SOC_SERIES_APOLLO3X)
	IRQ_CONNECT(DT_IRQN(SPI_DEV_NODE), DT_IRQ(SPI_DEV_NODE, priority), bt_packet_irq_isr, 0, 0);
#endif /* CONFIG_SOC_APOLLO510B */

	return ret;
}

int bt_apollo_controller_init(spi_transmit_fun transmit, bt_spi_transceive_fun transceive)
{
	int ret = 0;

#if (CONFIG_SOC_APOLLO510B)
	ARG_UNUSED(transmit);

	am_devices_em9305_callback_t cb = {
		.reset = am_devices_em9305_controller_reset,
		.transceive = transceive,
	};

	/* Initialize the BLE controller */
	ret = am_devices_em9305_init(&cb);

	if (ret == AM_DEVICES_EM9305_STATUS_SUCCESS) {
		LOG_DBG("bt controller initialized\r\n");
	} else {
		LOG_DBG("bt controller initialization fail\r\n");
	}
#elif (CONFIG_SOC_SERIES_APOLLO4X)
	ARG_UNUSED(transceive);

	am_devices_cooper_callback_t cb = {
		.write = transmit,
		.reset = bt_apollo_controller_reset,
	};

	/* Initialize the BLE controller */
	ret = am_devices_cooper_init(&cb);
	if (ret == AM_DEVICES_COOPER_STATUS_SUCCESS) {
		if (am_devices_cooper_upgrade_performed()) {
#if defined(CONFIG_REBOOT)
			LOG_INF("BLE controller update applied, rebooting host for clean restart");
			sys_reboot(SYS_REBOOT_COLD);
#else
			LOG_INF("BLE controller update applied, continue to main application");
#endif
		}

		am_devices_cooper_set_initialize_state(AM_DEVICES_COOPER_STATE_INITIALIZED);
		LOG_INF("BT controller initialized");
	} else {
		am_devices_cooper_set_initialize_state(AM_DEVICES_COOPER_STATE_INITIALIZE_FAIL);
		LOG_ERR("BT controller initialization fail");
	}
#elif (CONFIG_SOC_SERIES_APOLLO3X)
	ARG_UNUSED(transceive);

	ret = am_apollo3_bt_controller_init();
	if (ret == AM_HAL_STATUS_SUCCESS) {
		LOG_INF("BT controller initialized");
	} else {
		LOG_ERR("BT controller initialization fail");
	}

	irq_enable(DT_IRQN(SPI_DEV_NODE));
#endif /* CONFIG_SOC_APOLLO510B */

	return ret;
}

int bt_apollo_controller_deinit(void)
{
	int ret = 0;

#if (CONFIG_SOC_APOLLO510B)
	/* Deinitialize the BLE controller driver */
	ret = am_devices_em9305_deinit();
	if (ret == AM_DEVICES_EM9305_STATUS_SUCCESS) {
#if EM9305_HEARTBEAT_ENABLED
		/* Stop the heartbeat timer */
		k_work_cancel_delayable(&em9305_heartbeat_work);
#endif
		bt_em9305_hsclk_req(false);
		/* Disable GPIOs */
		gpio_pin_configure_dt(&irq_gpio, GPIO_DISCONNECTED);
		gpio_pin_configure_dt(&rst_gpio, GPIO_DISCONNECTED);
		gpio_remove_callback(irq_gpio.port, &irq_gpio_cb);
		LOG_INF("BT controller deinitialized");
	} else {
		ret = -EPERM;
		LOG_ERR("BT controller deinitialization fails");
	}
#elif (CONFIG_SOC_SERIES_APOLLO4X)
	/* Keep the Controller in resetting state */
	gpio_pin_set_dt(&rst_gpio, 1);

	/* Disable XO32MHz */
	clock_control_off(clk32m_dev, (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE);
	/* Disable XO32kHz  */
	clock_control_off(clk32k_dev, (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_LFXTAL);

	/* Disable GPIOs */
	gpio_pin_configure_dt(&irq_gpio, GPIO_DISCONNECTED);
	gpio_pin_configure_dt(&clkreq_gpio, GPIO_DISCONNECTED);
	gpio_remove_callback(clkreq_gpio.port, &clkreq_gpio_cb);
	gpio_remove_callback(irq_gpio.port, &irq_gpio_cb);
#elif (CONFIG_SOC_SERIES_APOLLO3X)
	irq_disable(DT_IRQN(SPI_DEV_NODE));

	ret = am_apollo3_bt_controller_deinit();
	if (ret == AM_HAL_STATUS_SUCCESS) {
		LOG_INF("BT controller deinitialized");
	} else {
		ret = -EPERM;
		LOG_ERR("BT controller deinitialization fails");
	}
#else
	ret = -ENOTSUP;
#endif /* CONFIG_SOC_APOLLO510B */

	return ret;
}

#if (CONFIG_SOC_SERIES_APOLLO4X)
static int bt_apollo_set_nvds(void)
{
	int ret;
	struct net_buf *buf;

#if defined(CONFIG_BT_HCI_RAW)
	struct bt_hci_cmd_hdr hdr;

	hdr.opcode = sys_cpu_to_le16(HCI_VSC_UPDATE_NVDS_CFG_CMD_OPCODE);
	hdr.param_len = HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH;
	buf = bt_buf_get_tx(BT_BUF_CMD, K_NO_WAIT, &hdr, sizeof(hdr));
	if (!buf) {
		return -ENOBUFS;
	}

	net_buf_add_mem(buf, &am_devices_cooper_nvds[0], HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH);
	ret = bt_send(buf);

	if (!ret) {
		/* Give some time to make NVDS take effect in BLE controller */
		k_sleep(K_MSEC(5));

		/* Need to send reset command to make the NVDS take effect */
		hdr.opcode = sys_cpu_to_le16(BT_HCI_OP_RESET);
		hdr.param_len = 0;
		buf = bt_buf_get_tx(BT_BUF_CMD, K_NO_WAIT, &hdr, sizeof(hdr));
		if (!buf) {
			return -ENOBUFS;
		}

		ret = bt_send(buf);
	}
#else
	uint8_t *p;

	buf = bt_hci_cmd_alloc(K_FOREVER);
	if (!buf) {
		return -ENOBUFS;
	}

	p = net_buf_add(buf, HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH);
	memcpy(p, &am_devices_cooper_nvds[0], HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH);
	ret = bt_hci_cmd_send_sync(HCI_VSC_UPDATE_NVDS_CFG_CMD_OPCODE, buf, NULL);

	if (!ret) {
		/* Give some time to make NVDS take effect in BLE controller */
		k_sleep(K_MSEC(5));
	}
#endif /* defined(CONFIG_BT_HCI_RAW) */

	return ret;
}
#endif /* CONFIG_SOC_SERIES_APOLLO4X */

#if (CONFIG_SOC_APOLLO510B) && defined(CONFIG_BT_HCI_RAW)
static uint16_t g_vsc_pending_opcode;
static uint8_t g_vsc_cc_status;
static K_SEM_DEFINE(g_vsc_cc_done, 0, 1);

void bt_apollo_vsc_cc_observe(uint16_t opcode, uint8_t status)
{
	if ((opcode != 0U) && (opcode == g_vsc_pending_opcode)) {
		g_vsc_cc_status = status;
		g_vsc_pending_opcode = 0U;
		k_sem_give(&g_vsc_cc_done);
	}
}
#else
void bt_apollo_vsc_cc_observe(uint16_t opcode, uint8_t status)
{
	ARG_UNUSED(opcode);
	ARG_UNUSED(status);
}
#endif /* CONFIG_SOC_APOLLO510B && CONFIG_BT_HCI_RAW */

#if (CONFIG_SOC_APOLLO510B)
static int bt_em9305_send_vsc(uint16_t opcode, uint8_t param_len, const uint8_t *param)
{
#if defined(CONFIG_BT_HCI_RAW)
	struct net_buf *buf;
	struct bt_hci_cmd_hdr hdr;
	int ret;

	hdr.opcode = sys_cpu_to_le16(opcode);
	hdr.param_len = param_len;
	buf = bt_buf_get_tx(BT_BUF_CMD, K_NO_WAIT, &hdr, sizeof(hdr));
	if (!buf) {
		return -ENOBUFS;
	}
	if (param_len > 0U) {
		net_buf_add_mem(buf, param, param_len);
	}

	g_vsc_cc_status = 0xFFU;
	g_vsc_pending_opcode = opcode;
	k_sem_reset(&g_vsc_cc_done);

	ret = bt_send(buf);
	if (ret) {
		g_vsc_pending_opcode = 0U;
		return ret;
	}

	if (k_sem_take(&g_vsc_cc_done, K_MSEC(EM9305_VSC_CC_TIMEOUT_MS)) != 0) {
		LOG_ERR("EM9305: VSC 0x%04x command-complete timeout", opcode);
		g_vsc_pending_opcode = 0U;
		return -ETIMEDOUT;
	}

	if (g_vsc_cc_status != 0U) {
		LOG_ERR("EM9305: VSC 0x%04x rejected, status=0x%02x", opcode, g_vsc_cc_status);
		return -EIO;
	}

	return 0;
#else
	struct net_buf *buf;

	buf = bt_hci_cmd_alloc(K_FOREVER);
	if (!buf) {
		return -ENOBUFS;
	}
	if (param_len > 0U) {
		net_buf_add_mem(buf, param, param_len);
	}
	return bt_hci_cmd_send_sync(opcode, buf, NULL);
#endif /* defined(CONFIG_BT_HCI_RAW) */
}

/*
 * Push the EM9305 LE local supported features mask.
 */
static int bt_em9305_set_ll_features(void)
{
	int ret = bt_em9305_send_vsc(HCI_VSC_SET_LOCAL_SUP_FEAT_CMD_OPCODE,
				     HCI_VSC_SET_LOCAL_SUP_FEAT_CMD_LENGTH, em9305_ll_feats);
	if (ret) {
		LOG_ERR("EM9305: set LL features VSC failed (%d)", ret);
	}
	return ret;
}

/*
 * Set the EM9305 default radio transmit power.
 */
static int bt_em9305_set_tx_power(int8_t dbm)
{
	uint8_t param = (uint8_t)dbm;
	int ret = bt_em9305_send_vsc(HCI_VSC_SET_TX_POWER_LEVEL_CMD_OPCODE,
				     HCI_VSC_SET_TX_POWER_LEVEL_CMD_LENGTH, &param);
	if (ret) {
		LOG_ERR("EM9305: set TX power VSC failed (%d)", ret);
	}
	return ret;
}
#endif /* CONFIG_SOC_APOLLO510B */

int bt_apollo_set_public_addr(const uint8_t addr[6])
{
#if (CONFIG_SOC_APOLLO510B)
	int ret;

	if (addr == NULL) {
		return -EINVAL;
	}
	ret = bt_em9305_send_vsc(HCI_VSC_SET_DEV_PUB_ADDR_CMD_OPCODE,
				 HCI_VSC_SET_DEV_PUB_ADDR_CMD_LENGTH, addr);
	if (ret) {
		LOG_ERR("EM9305: set public BD addr VSC failed (%d)", ret);
	}
	return ret;
#else
	ARG_UNUSED(addr);
	return -ENOTSUP;
#endif /* CONFIG_SOC_APOLLO510B */
}

int bt_apollo_set_adv_tx_power(int8_t txpower_dbm)
{
#if (CONFIG_SOC_APOLLO510B)
	uint8_t param = (uint8_t)txpower_dbm;
	int ret;

	if ((txpower_dbm <= EM9305_TX_POWER_MIN_DBM) || (txpower_dbm > EM9305_TX_POWER_MAX_DBM)) {
		LOG_ERR("EM9305: adv TX power %d dBm out of range (%d..%d]", txpower_dbm,
			EM9305_TX_POWER_MIN_DBM, EM9305_TX_POWER_MAX_DBM);
		return -EINVAL;
	}

	ret = bt_em9305_send_vsc(HCI_VSC_SET_ADV_TX_POWER_CMD_OPCODE,
				 HCI_VSC_SET_ADV_TX_POWER_CMD_LENGTH, &param);
	if (ret) {
		LOG_ERR("EM9305: set adv TX power VSC failed (%d)", ret);
	}
	return ret;
#else
	ARG_UNUSED(txpower_dbm);
	return -ENOTSUP;
#endif /* CONFIG_SOC_APOLLO510B */
}

int bt_apollo_set_conn_tx_power(uint16_t conn_handle, int8_t txpower_dbm)
{
#if (CONFIG_SOC_APOLLO510B)
	uint8_t param[HCI_VSC_SET_CONN_TX_POWER_CMD_LENGTH];
	int ret;

	if ((txpower_dbm <= EM9305_TX_POWER_MIN_DBM) || (txpower_dbm > EM9305_TX_POWER_MAX_DBM)) {
		LOG_ERR("EM9305: conn TX power %d dBm out of range (%d..%d]", txpower_dbm,
			EM9305_TX_POWER_MIN_DBM, EM9305_TX_POWER_MAX_DBM);
		return -EINVAL;
	}

	param[0] = (uint8_t)(conn_handle >> 8);
	param[1] = (uint8_t)(conn_handle & 0xFFU);
	param[2] = (uint8_t)txpower_dbm;

	ret = bt_em9305_send_vsc(HCI_VSC_SET_CONN_TX_POWER_CMD_OPCODE,
				 HCI_VSC_SET_CONN_TX_POWER_CMD_LENGTH, param);
	if (ret) {
		LOG_ERR("EM9305: set conn TX power VSC failed (%d)", ret);
	}
	return ret;
#else
	ARG_UNUSED(conn_handle);
	ARG_UNUSED(txpower_dbm);
	return -ENOTSUP;
#endif /* CONFIG_SOC_APOLLO510B */
}

int bt_apollo_vnd_setup(void)
{
	int ret = 0;

#if (CONFIG_SOC_SERIES_APOLLO4X)
	/* Set the NVDS parameters to BLE controller */
	ret = bt_apollo_set_nvds();
#elif (CONFIG_SOC_APOLLO510B)
	ret = bt_em9305_set_ll_features();
	if (ret == 0) {
		ret = bt_em9305_set_tx_power(EM9305_TX_POWER_DEFAULT);
	}
	if (ret == 0) {
		const uint8_t sleep_disable = 0x00;

		ret = bt_em9305_send_vsc(HCI_VSC_SET_SLEEP_OPTION_CMD_OPCODE,
					 HCI_VSC_SET_SLEEP_OPTION_CMD_LENGTH, &sleep_disable);
		if (ret != 0) {
			LOG_WRN("EM9305: sleep disable VSC failed (%d), continuing", ret);
			ret = 0; /* non-fatal — proceed without sleep disable */
		}
	}
	if (ret == 0) {
#if EM9305_HEARTBEAT_ENABLED
		k_work_reschedule(&em9305_heartbeat_work, K_MSEC(EM9305_HEARTBEAT_INTERVAL_MS));
#endif
	}
#endif /* CONFIG_SOC_SERIES_APOLLO4X */

	return ret;
}

int bt_apollo_dev_init(void)
{
#if (CONFIG_SOC_SERIES_APOLLO4X)
	if (!gpio_is_ready_dt(&irq_gpio)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&rst_gpio)) {
		LOG_ERR("Reset GPIO device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&clkreq_gpio)) {
		LOG_ERR("CLKREQ GPIO device not ready");
		return -ENODEV;
	}
#elif (CONFIG_SOC_APOLLO510B)
	if (!gpio_is_ready_dt(&irq_gpio)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&rst_gpio)) {
		LOG_ERR("Reset GPIO device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cm_gpio)) {
		LOG_ERR("CM GPIO device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&clkreq_gpio)) {
		LOG_ERR("CLKREQ GPIO device not ready");
		return -ENODEV;
	}

	/* Drive CLKREQ low (de-asserted) until bt_hci_transport_setup asserts
	 * it; configure the pin as output before any SPI activity.
	 */
	{
		int ret = gpio_pin_configure_dt(&clkreq_gpio, GPIO_OUTPUT_INACTIVE);

		if (ret) {
			return ret;
		}
	}
#endif /* CONFIG_SOC_SERIES_APOLLO4X */

	return 0;
}
