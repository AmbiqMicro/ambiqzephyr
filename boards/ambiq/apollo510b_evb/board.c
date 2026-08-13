/*
 * Copyright 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Apollo510B EXTREF (SIP EM9305) handling on this board:
 *
 * - soc_early_init_hook() runs am_hal_pwrctrl_low_power_init() first.
 * - board_early_init_hook() runs next (see kernel/init.c): programs CLKMGR board
 *   info (including EXTREF frequency), HFRC/HFRC2 defaults, SIP GPIO 136, and
 *   optional USB PHY tuning.
 * - When CONFIG_SOC_APOLLO510B_EM9305_EXTREF_INIT is enabled, POST_KERNEL
 *   SYS_INIT wakes the SIP EM9305 over SPI and sends HCI VSC 0xFC49 (set sleep option)
 *   so CLKMGR can use the ~12 MHz EXTREF without CONFIG_BT.
 * - GPIO 136 (AP5_12M_CLKREQ) is preconfigured output-low in board_early_init_hook;
 *   clkmgr asserts it only when EXTREF_CLK is requested (not at boot).
 */

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/byteorder.h>
#include <am_mcu_apollo.h>

#if IS_ENABLED(CONFIG_SOC_APOLLO510B_EM9305_EXTREF_INIT)
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <am_devices_em9305.h>

LOG_MODULE_REGISTER(apollo510b_evb, CONFIG_LOG_DEFAULT_LEVEL);

#define EM9305_HCI_CMD_PKT       0x01U
#define EM9305_HCI_VSC_SET_SLEEP 0xFC49U
#endif

#if defined(CONFIG_AMBIQ_HAL_USE_USB)
#include <am_hal_usb.h>
#endif

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

#if !IS_ENABLED(CONFIG_BT)
#define EM9305_CM_TIMER         11U
#define EM9305_CM_PAD_CT_FNCSEL 6U
#define EM9305_CM_PWM_COMPARE0  50U /* HFRC/64 = 1.5 MHz -> 30 kHz period */
#define EM9305_CM_PWM_COMPARE1  25U /* 50 % duty cycle                    */
#endif

#define AP5_EM9305_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ambiq_bt_hci_spi)

static const struct gpio_dt_spec em9305_irq_gpio = GPIO_DT_SPEC_GET(AP5_EM9305_NODE, irq_gpios);
static const struct gpio_dt_spec em9305_rst_gpio = GPIO_DT_SPEC_GET(AP5_EM9305_NODE, reset_gpios);
static const struct gpio_dt_spec em9305_cm_gpio = GPIO_DT_SPEC_GET(AP5_EM9305_NODE, cm_gpios);
static const struct gpio_dt_spec em9305_clkreq_gpio =
	GPIO_DT_SPEC_GET(AP5_EM9305_NODE, clkreq_gpios);
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

static void em9305_set_cm(bool state)
{
	(void)gpio_pin_set_dt(&em9305_cm_gpio, state ? 1 : 0);
}

#if !IS_ENABLED(CONFIG_BT)
static void em9305_cm_pwm_ctrl(bool enable)
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

		am_hal_timer_output_config(em9305_cm_gpio.pin, AM_HAL_TIMER_OUTPUT_TMR11_OUT0);

		ct_cfg.GP.cfg_b.uFuncSel = EM9305_CM_PAD_CT_FNCSEL;
		am_hal_gpio_pinconfig(em9305_cm_gpio.pin, ct_cfg);

		am_hal_timer_enable(EM9305_CM_TIMER);
	} else {
		am_hal_timer_disable(EM9305_CM_TIMER);
		gpio_pin_configure_dt(&em9305_cm_gpio, GPIO_INPUT);
	}
}
#endif

static void em9305_hsclk_req(bool enable)
{
	(void)gpio_pin_set_dt(&em9305_clkreq_gpio, enable ? 1 : 0);
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
#if IS_ENABLED(CONFIG_BT)
	/* HCI driver owns EM9305 when Bluetooth is enabled. */
	return 0;
#endif

	am_devices_em9305_callback_t cb;
	uint32_t st;

	if (!device_is_ready(em9305_spi.bus)) {
		LOG_ERR("EM9305 EXTREF init: SPI bus not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&em9305_clkreq_gpio)) {
		LOG_ERR("EM9305 EXTREF init: CLKREQ GPIO not ready");
		return -ENODEV;
	}

	am_devices_em9305_register_gpio_ops(em9305_set_reset, em9305_get_reset,
					    em9305_irq_pin_state, em9305_cs_set,
					    em9305_cs_release);
	am_devices_em9305_register_cm_gpio(em9305_set_cm);
#if !IS_ENABLED(CONFIG_BT)
	am_devices_em9305_register_cm_pwm_ops(em9305_cm_pwm_ctrl);
#endif

	if (gpio_pin_configure_dt(&em9305_clkreq_gpio, GPIO_OUTPUT_INACTIVE) != 0) {
		return -EIO;
	}
	em9305_hsclk_req(true);

	if (gpio_pin_configure_dt(&em9305_rst_gpio, GPIO_OUTPUT_ACTIVE) != 0) {
		return -EIO;
	}
	if (gpio_pin_configure_dt(&em9305_cm_gpio, GPIO_INPUT) != 0) {
		return -EIO;
	}
	if (gpio_pin_configure_dt(&em9305_irq_gpio, GPIO_INPUT) != 0) {
		return -EIO;
	}

	cb.reset = am_devices_em9305_controller_reset;
	cb.transceive = em9305_spi_transceive;
	cb.skip_fw_update = IS_ENABLED(CONFIG_BT);

	st = am_devices_em9305_init(&cb);
	if (st != AM_DEVICES_EM9305_STATUS_SUCCESS) {
		LOG_ERR("EM9305 init failed (%u)", st);
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
	am_hal_clkmgr_board_info_t sClkmgrBoardInfo = {
		.sXtalHs = {.eXtalHsMode = XTAL_HS_MODE, .ui32XtalHsFreq = XTAL_HS_FREQ},
		.sXtalLs = {.eXtalLsMode = XTAL_LS_MODE, .ui32XtalLsFreq = XTAL_LS_FREQ},
		.ui32ExtRefClkFreq = EXTREFCLK_FREQ,
#if IS_ENABLED(CONFIG_SOC_APOLLO510B_EM9305_EXTREF_INIT)
		.bIsSipEnabled = true,
#endif
	};

	am_hal_clkmgr_board_info_set(&sClkmgrBoardInfo);

#if IS_ENABLED(CONFIG_SOC_APOLLO510B_EM9305_EXTREF_INIT)
	/*
	 * Preconfigure AP5_12M_CLKREQ (GPIO 136) as output low. Clkmgr asserts it
	 * only when EXTREF_CLK is requested; keeping it high forces EM9305 12 MHz
	 * on continuously (see am_hal_clkmgr_request_EXTREF_CLK()).
	 */
	am_hal_gpio_pinconfig(136, am_hal_gpio_pincfg_output);
	am_hal_gpio_output_clear(136);
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
	(void)am_hal_usb_phy_electrical_tuning_set(0, AM_HAL_USB_PHY_ELEC_TUNING_PARAM_R_ODT,
						   &usb_rodt);
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
