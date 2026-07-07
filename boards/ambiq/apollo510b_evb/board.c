/*
 * Copyright 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/byteorder.h>
#include <am_mcu_apollo.h>

#if IS_ENABLED(CONFIG_SOC_APOLLO510B_EM9305_EXTREF_INIT)
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <am_devices_em9305.h>

LOG_MODULE_REGISTER(apollo510b_evb, CONFIG_LOG_DEFAULT_LEVEL);

#define EM9305_HCI_CMD_PKT       0x01U
#define EM9305_HCI_VSC_SET_SLEEP 0xFD09U
#define EM9305_ACTIVE_TIMEOUT_MS 2000U
#endif

#if defined(CONFIG_AMBIQ_HAL_USE_USB)
#include <am_hal_usb.h>
#endif

#define APOLLO510B_SIP_EXTREF_ENABLED                                                             \
	(IS_ENABLED(CONFIG_SOC_APOLLO510B_EM9305_EXTREF_INIT) || IS_ENABLED(CONFIG_BT_AMBIQ_HCI))

#if DT_HAS_CHOSEN(ambiq_xo32m)
#define XTAL_HS_FREQ DT_PROP(DT_CHOSEN(ambiq_xo32m), clock_frequency)
#if DT_SAME_NODE(DT_CHOSEN(ambiq_xo32m), DT_NODELABEL(xo32m_xtal))
#define XTAL_HS_MODE AM_HAL_CLKMGR_XTAL_HS_MODE_XTAL
#elif DT_SAME_NODE(DT_CHOSEN(ambiq_xo32m), DT_NODELABEL(xo32m_ext))
#define XTAL_HS_MODE AM_HAL_CLKMGR_XTAL_HS_MODE_EXT
#endif
#else
#define XTAL_HS_FREQ 0
#define XTAL_HS_MODE AM_HAL_CLKMGR_XTAL_HS_MODE_XTAL
#endif

#if DT_HAS_CHOSEN(ambiq_xo32k)
#define XTAL_LS_FREQ DT_PROP(DT_CHOSEN(ambiq_xo32k), clock_frequency)
#if DT_SAME_NODE(DT_CHOSEN(ambiq_xo32k), DT_NODELABEL(xo32k_xtal))
#define XTAL_LS_MODE AM_HAL_CLKMGR_XTAL_LS_MODE_XTAL
#elif DT_SAME_NODE(DT_CHOSEN(ambiq_xo32k), DT_NODELABEL(xo32k_ext))
#define XTAL_LS_MODE AM_HAL_CLKMGR_XTAL_LS_MODE_EXT
#endif
#else
#define XTAL_LS_FREQ 0
#define XTAL_LS_MODE AM_HAL_CLKMGR_XTAL_LS_MODE_XTAL
#endif

#if DT_HAS_CHOSEN(ambiq_extrefclk)
#define EXTREFCLK_FREQ DT_PROP(DT_CHOSEN(ambiq_extrefclk), clock_frequency)
#else
#define EXTREFCLK_FREQ 0
#endif

#if IS_ENABLED(CONFIG_SOC_APOLLO510B_EM9305_EXTREF_INIT)

#define AP5_EM9305_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ambiq_bt_hci_spi)

static const struct gpio_dt_spec em9305_irq_gpio = GPIO_DT_SPEC_GET(AP5_EM9305_NODE, irq_gpios);
static const struct gpio_dt_spec em9305_rst_gpio = GPIO_DT_SPEC_GET(AP5_EM9305_NODE, reset_gpios);
static const struct gpio_dt_spec em9305_cm_gpio = GPIO_DT_SPEC_GET(AP5_EM9305_NODE, cm_gpios);
static const struct gpio_dt_spec em9305_cs_gpio =
	GPIO_DT_SPEC_GET(DT_BUS(AP5_EM9305_NODE), cs_gpios);

static struct spi_dt_spec em9305_spi = SPI_DT_SPEC_GET(AP5_EM9305_NODE,
						       SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
							       SPI_WORD_SET(8));

static struct spi_buf em9305_spi_tx_buf;
static struct spi_buf em9305_spi_rx_buf;
static const struct spi_buf_set em9305_spi_tx = {.buffers = &em9305_spi_tx_buf, .count = 1};
static const struct spi_buf_set em9305_spi_rx = {.buffers = &em9305_spi_rx_buf, .count = 1};

static bool em9305_irq_pin_state(void)
{
	return gpio_pin_get_dt(&em9305_irq_gpio) > 0;
}

static void em9305_set_reset(bool state)
{
	(void)gpio_pin_set_dt(&em9305_rst_gpio, state);
}

static bool em9305_get_reset(void)
{
	return gpio_pin_get_dt(&em9305_rst_gpio) > 0;
}

static void em9305_cs_set(void)
{
	(void)gpio_pin_set_dt(&em9305_cs_gpio, 1);
}

static void em9305_cs_release(void)
{
	(void)gpio_pin_set_dt(&em9305_cs_gpio, 0);
}

static int em9305_spi_transceive(void *tx, uint32_t tx_len, void *rx, uint32_t rx_len)
{
	em9305_spi_tx_buf.buf = tx;
	em9305_spi_tx_buf.len = tx_len;
	em9305_spi_rx_buf.buf = rx;
	em9305_spi_rx_buf.len = rx_len;

	if (tx_len && rx_len) {
		em9305_spi.config.operation |= SPI_HOLD_ON_CS;
	} else {
		em9305_spi.config.operation &= ~SPI_HOLD_ON_CS;
	}

	return spi_transceive_dt(&em9305_spi, &em9305_spi_tx, &em9305_spi_rx);
}

static int em9305_spi_rcv(uint8_t *data, uint16_t size_max, uint16_t *len)
{
	uint8_t cmd[2] = {EM9305_SPI_HEADER_RX, 0x00};
	uint8_t sts[2] = {0};
	uint8_t rx_bytes;
	int ret;

	*len = 0;

	if (am_devices_em9305_get_spi_tx_status()) {
		return -EBUSY;
	}
	if (!em9305_irq_pin_state()) {
		return -ENODATA;
	}

	do {
		for (uint32_t i = 0; i < EM9305_STS_CHK_CNT_MAX; i++) {
			em9305_cs_set();
			ret = em9305_spi_transceive(cmd, sizeof(cmd), sts, sizeof(sts));
			if (ret) {
				em9305_cs_release();
				return -EIO;
			}
			if ((sts[0] == EM9305_STS1_READY_VALUE) && (sts[1] != 0x00)) {
				break;
			}
			em9305_cs_release();
		}

		if ((sts[0] != EM9305_STS1_READY_VALUE) || (sts[1] == 0x00)) {
			em9305_cs_release();
			return (*len != 0) ? 0 : -EAGAIN;
		}

		rx_bytes = sts[1];
		if (em9305_irq_pin_state() && (rx_bytes != 0)) {
			if ((*len + rx_bytes) > size_max) {
				em9305_cs_release();
				return -EMSGSIZE;
			}

			ret = em9305_spi_transceive(NULL, 0, data + *len, rx_bytes);
			if (ret) {
				em9305_cs_release();
				return -EIO;
			}
			*len += rx_bytes;
		}
		em9305_cs_release();
	} while (em9305_irq_pin_state());

	return 0;
}

static int em9305_wait_active(void)
{
	uint8_t buf[EM9305_BUFFER_SIZE];
	uint16_t plen;
	uint32_t t0;

	am_devices_em9305_controller_reset();

	t0 = k_uptime_get_32();
	while ((k_uptime_get_32() - t0) < EM9305_ACTIVE_TIMEOUT_MS) {
		if (em9305_irq_pin_state()) {
			plen = 0;
			if ((em9305_spi_rcv(buf, sizeof(buf), &plen) == 0) && (plen > 0) &&
			    am_devices_em9305_check_active_state_event(buf, plen)) {
				return 0;
			}
		}
		k_msleep(1);
	}

	return -ETIMEDOUT;
}

static int em9305_enable_sleep(void)
{
	uint8_t cmd[5];
	int st;

	cmd[0] = EM9305_HCI_CMD_PKT;
	sys_put_le16(EM9305_HCI_VSC_SET_SLEEP, &cmd[1]);
	cmd[3] = 1U;
	cmd[4] = 1U;

	st = am_devices_em9305_blocking_write(cmd, sizeof(cmd), em9305_spi_transceive);

	return (st == AM_DEVICES_EM9305_STATUS_SUCCESS) ? 0 : -EIO;
}

static int board_em9305_extref_init(void)
{
	if (!device_is_ready(em9305_spi.bus)) {
		LOG_ERR("EM9305 EXTREF init: SPI bus not ready");
		return -ENODEV;
	}

	am_devices_em9305_register_gpio_ops(em9305_set_reset, em9305_get_reset,
					    em9305_irq_pin_state, em9305_cs_set,
					    em9305_cs_release);

	if (gpio_pin_configure_dt(&em9305_rst_gpio, GPIO_OUTPUT_ACTIVE) != 0) {
		return -EIO;
	}
	if (gpio_pin_configure_dt(&em9305_cm_gpio, GPIO_OUTPUT_INACTIVE) != 0) {
		return -EIO;
	}
	if (gpio_pin_configure_dt(&em9305_irq_gpio, GPIO_INPUT) != 0) {
		return -EIO;
	}

	if (em9305_wait_active() != 0) {
		LOG_ERR("EM9305 init failed: no active-state event");
		return -EIO;
	}

	if (em9305_enable_sleep() != 0) {
		LOG_WRN("EM9305 sleep enable failed");
	}

	LOG_INF("EM9305 ready for EXTREF");

	return 0;
}

#endif /* CONFIG_SOC_APOLLO510B_EM9305_EXTREF_INIT */

void board_early_init_hook(void)
{
	/* Set board related info into clock manager */
	am_hal_clkmgr_board_info_t sClkmgrBoardInfo = {.sXtalHs.eXtalHsMode = XTAL_HS_MODE,
						       .sXtalHs.ui32XtalHsFreq = XTAL_HS_FREQ,
						       .sXtalLs.eXtalLsMode = XTAL_LS_MODE,
						       .sXtalLs.ui32XtalLsFreq = XTAL_LS_FREQ,
						       .ui32ExtRefClkFreq = EXTREFCLK_FREQ,
#if APOLLO510B_SIP_EXTREF_ENABLED
						       .bIsSipEnabled = true,
#endif
	};
	am_hal_clkmgr_board_info_set(&sClkmgrBoardInfo);

#if APOLLO510B_SIP_EXTREF_ENABLED
	am_hal_gpio_pinconfig(136, am_hal_gpio_pincfg_output);
	am_hal_gpio_output_set(136);
#endif

	/* Default HFRC and HFRC2 to Free Running clocks */
	am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_HFRC,
				   AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ, NULL);
	am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_HFRC2,
				   AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ, NULL);

#if defined(CONFIG_AMBIQ_HAL_USE_USB)
	/* USB PHY electrical tuning. Mirrors what the AmbiqSuite BSP does in
	 * am_bsp_low_power_init() — sets the on-die termination resistance so
	 * the high/full-speed signaling matches the board trace impedance.
	 * Must run before am_hal_usb_initialize().
	 */
	am_hal_usb_phy_elec_tuning_param_val_t usb_rodt = {
		.eRodtTuning = AM_HAL_USB_PHY_R_ODT_VAL_10,
	};
	(void)am_hal_usb_phy_electrical_tuning_set(0,
		AM_HAL_USB_PHY_ELEC_TUNING_PARAM_R_ODT, &usb_rodt);
#endif /* CONFIG_AMBIQ_HAL_USE_USB */
}

#if IS_ENABLED(CONFIG_SOC_APOLLO510B_EM9305_EXTREF_INIT)
SYS_INIT(board_em9305_extref_init, POST_KERNEL,
	 CONFIG_SOC_APOLLO510B_EM9305_EXTREF_INIT_PRIORITY);
#endif

#if defined(CONFIG_BOARD_ENABLE_GPU_ASSET_RELOCATION)

/* Symbols defined in the board-level linker.ld */
extern char __gfx_assets_start[];
extern char __gfx_assets_load_start[];
extern char __gfx_assets_size[];

void board_late_init_hook(void)
{
#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(ambiq_psram)) &&                                             \
	DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(ambiq_external_ram_region))
	pm_device_runtime_get(DEVICE_DT_GET(DT_CHOSEN(ambiq_psram)));
#endif

	memcpy(__gfx_assets_start, __gfx_assets_load_start, (size_t)&__gfx_assets_size);

	sys_cache_data_flush_range(__gfx_assets_start, (size_t)&__gfx_assets_size);
}
#endif /* CONFIG_BOARD_ENABLE_GPU_ASSET_RELOCATION */
