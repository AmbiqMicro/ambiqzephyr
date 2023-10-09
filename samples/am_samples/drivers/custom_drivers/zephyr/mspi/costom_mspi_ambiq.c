/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT custom_ambiq_qspi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ambiq_qspi);

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/syscall_handler.h>

#include "custom_mspi_ambiq.h"

#define SPI_WORD_SIZE        8
#define MSPI_MAX_FREQ        96000000
#define MSPI_TIMEOUT_US      1000000
#define PWRCTRL_MAX_WAIT_US  5
#define MSPI_BUSY            BIT(2)
#define AM_DEVICES_MSPI_AMBT53_READ_BL4           0xA2
#define AM_DEVICES_MSPI_AMBT53_WRITE_UL           0x7F

#define MSPI_AMBT53_TRANSFER_TIMEOUT   10

typedef int (*ambiq_mspi_pwr_func_t)(void);

struct mspi_ambiq_config {
	uint32_t base;
	uint32_t xipmm_offset;
	int size;
	const struct pinctrl_dev_config *pcfg;
	ambiq_mspi_pwr_func_t pwr_func;
};

struct mspi_ambiq_data {
	void *mspiHandle;
	am_hal_mspi_dev_config_t mspi_dev_cfg;
};

am_hal_mspi_dqs_t AMBT53DqsCfg[] =
{
  {
    .bDQSEnable             = 1,
    .bDQSSyncNeg            = 0,
    .bEnableFineDelay       = 0,
    .ui8TxDQSDelay          = 0,
    .ui8RxDQSDelay          = 16,
    .ui8RxDQSDelayNeg       = 0,
    .bRxDQSDelayNegEN       = 0,
    .ui8RxDQSDelayHi        = 0,
    .ui8RxDQSDelayNegHi     = 0,
    .bRxDQSDelayHiEN        = 0,
  },
  {
    .bDQSEnable             = 1,
    .bDQSSyncNeg            = 0,
    .bEnableFineDelay       = 0,
    .ui8TxDQSDelay          = 0,
    .ui8RxDQSDelay          = 16,
    .ui8RxDQSDelayNeg       = 0,
    .bRxDQSDelayNegEN       = 0,
    .ui8RxDQSDelayHi        = 0,
    .ui8RxDQSDelayNegHi     = 0,
    .bRxDQSDelayHiEN        = 0,
  },
  {
    .bDQSEnable             = 1,
    .bDQSSyncNeg            = 0,
    .bEnableFineDelay       = 0,
    .ui8TxDQSDelay          = 0,
    .ui8RxDQSDelay          = 16,
    .ui8RxDQSDelayNeg       = 0,
    .bRxDQSDelayNegEN       = 0,
    .ui8RxDQSDelayHi        = 0,
    .ui8RxDQSDelayNegHi     = 0,
    .bRxDQSDelayHiEN        = 0,
  }
};

am_hal_mspi_dqs_t AMBT53DqsDisable =
{
    .bDQSEnable             = 0,
    .bDQSSyncNeg            = 0,
    .bEnableFineDelay       = 0,
    .ui8TxDQSDelay          = 0,
    .ui8RxDQSDelay          = 0,
    .ui8RxDQSDelayNeg       = 0,
    .bRxDQSDelayNegEN       = 0,
    .ui8RxDQSDelayHi        = 0,
    .ui8RxDQSDelayNegHi     = 0,
    .bRxDQSDelayHiEN        = 0,
};

am_hal_mspi_xip_config_t AMBT53XipConfig[] =
{
  {
    .ui32APBaseAddr       = MSPI0_APERTURE_START_ADDR,
    .eAPMode              = AM_HAL_MSPI_AP_READ_WRITE,
    .eAPSize              = AM_HAL_MSPI_AP_SIZE64M,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
  },
  {
    .ui32APBaseAddr       = MSPI1_APERTURE_START_ADDR,
    .eAPMode              = AM_HAL_MSPI_AP_READ_WRITE,
    .eAPSize              = AM_HAL_MSPI_AP_SIZE64M,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
  },
  {
    .ui32APBaseAddr       = MSPI2_APERTURE_START_ADDR,
    .eAPMode              = AM_HAL_MSPI_AP_READ_WRITE,
    .eAPSize              = AM_HAL_MSPI_AP_SIZE64M,
    .scramblingStartAddr  = 0,
    .scramblingEndAddr    = 0,
  }
};

#if defined(CONFIG_SOC_APOLLO4P) || defined(CONFIG_SOC_APOLLO4L) || defined(CONFIG_SOC_APOLLO5)
am_hal_mspi_xip_misc_t gXipMiscCfg[] =
{
  {
    .ui32CEBreak        = 10,
    .bXIPBoundary       = true,
    .bXIPOdd            = false,
    .bAppndOdd          = false,
    .bBEOn              = false,
    .eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,
  },
  {
    .ui32CEBreak        = 10,
    .bXIPBoundary       = true,
    .bXIPOdd            = false,
    .bAppndOdd          = false,
    .bBEOn              = false,
    .eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,
  },
  {
    .ui32CEBreak        = 10,
    .bXIPBoundary       = true,
    .bXIPOdd            = false,
    .bAppndOdd          = false,
    .bBEOn              = false,
    .eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,
  },
#if defined(CONFIG_SOC_APOLLO5)
  {
    .ui32CEBreak        = 10,
    .bXIPBoundary       = true,
    .bXIPOdd            = false,
    .bAppndOdd          = false,
    .bBEOn              = false,
    .eBEPolarity        = AM_HAL_MSPI_BE_LOW_ENABLE,
  }
#endif
};

/*
 * The Apollo4 Plus/Lite and Apollo5 design the MSPI DEV0CFG1 register which can be configured
 * more options for RX mode. With the default value 0x00008200 (DQSTURN0 = 2, RXSMP0 = 1),
 * we can see there is one additional cycle of SPI CS active low after reading the SPI data.
 * It will cause the xSPI slave (ambt53) reporting burst error and invalid length error.
 * With the specific configuration 0x00008000 (DQSTURN0 = 2, RXSMP0 = 0), there is no more
 * cycle of CS active after reading, which can be the same as Apollo4 RevB MSPI RX behavior.
 * And the burst error and invalid length error can be cleaned. We have not understood all
 * the behavior/performance of this register with different configurations. Let's keep monitoring
 * this and may reconfigure it if we find other better options for the RX mode.
*/
am_hal_mspi_rxcfg_t gMspiRxCfg =
{
    .ui8DQSturn         = 2,
    .bRxHI              = 0,
    .bTaForth           = 0,
    .bHyperIO           = 0,
    .ui8RxSmp           = 0,
    .bRBX               = 0,
    .bWBX               = 0,
    .bSCLKRxHalt        = 0,
    .bRxCapEXT          = 0,
    .ui8Sfturn          = 0,
};
#endif

uint32_t custom_mspi_get_module_idx(const struct device *dev)
{
  const struct mspi_ambiq_config *cfg = dev->config;
  return (cfg->base - REG_MSPI_BASEADDR) / (cfg->size * 4);
}

static int mspi_set_freq(uint32_t freq)
{
	uint32_t d = MSPI_MAX_FREQ / freq;

	switch (d) {
	case AM_HAL_MSPI_CLK_96MHZ:
	case AM_HAL_MSPI_CLK_48MHZ:
	case AM_HAL_MSPI_CLK_32MHZ:
	case AM_HAL_MSPI_CLK_24MHZ:
	case AM_HAL_MSPI_CLK_16MHZ:
	case AM_HAL_MSPI_CLK_12MHZ:
	case AM_HAL_MSPI_CLK_8MHZ:
	case AM_HAL_MSPI_CLK_6MHZ:
	case AM_HAL_MSPI_CLK_4MHZ:
	case AM_HAL_MSPI_CLK_3MHZ:
		break;
	default:
		LOG_ERR("Frequency not supported!");
		d = AM_HAL_MSPI_CLK_INVALID;
		break;
	}

	return d;
}

static int mspi_config(const struct device *dev, const struct mspi_config *config)
{
	struct mspi_ambiq_data *data = dev->data;
	int ret;
	am_hal_mspi_dev_config_t *mspi_dev_cfg = &data->mspi_dev_cfg;
	const struct mspi_ambiq_config *cfg = dev->config;

	if (config->generic_config.operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(config->generic_config.operation) != 8) {
		LOG_ERR("Word size must be %d", SPI_WORD_SIZE);
		return -ENOTSUP;
	}

	if (config->generic_config.operation & SPI_LOCK_ON) {
		LOG_ERR("Lock On not supported");
		return -ENOTSUP;
	}

	if (config->generic_config.operation & SPI_TRANSFER_LSB) {
		LOG_ERR("LSB first not supported");
		return -ENOTSUP;
	}

	if (config->generic_config.operation & SPI_MODE_CPOL) {
		if (config->generic_config.operation & SPI_MODE_CPHA) {
			mspi_dev_cfg->eSpiMode = AM_HAL_IOM_SPI_MODE_3;
			} else {
			mspi_dev_cfg->eSpiMode = AM_HAL_IOM_SPI_MODE_2;
			}
	} else {
		if (config->generic_config.operation & SPI_MODE_CPHA) {
			mspi_dev_cfg->eSpiMode = AM_HAL_IOM_SPI_MODE_1;
		} else {
			mspi_dev_cfg->eSpiMode = AM_HAL_IOM_SPI_MODE_0;
		}
	}

	mspi_dev_cfg->eClockFreq = mspi_set_freq(config->generic_config.frequency);
	if (mspi_dev_cfg->eClockFreq == AM_HAL_MSPI_CLK_INVALID) {
		return -ENOTSUP;
	}

	switch (config->generic_config.operation & SPI_LINES_MASK)
	{
	case SPI_LINES_SINGLE:
		mspi_dev_cfg->eDeviceConfig = AM_HAL_MSPI_FLASH_SERIAL_CE0;
		// Set corresponding pin mode
		pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		break;
	case SPI_LINES_QUAD:
		mspi_dev_cfg->eDeviceConfig = AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4;
		// Set corresponding pin mode
		pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_QUAD);
		break;
	
	default:
		LOG_ERR("SPI mode is not supported");
		break;
	}

	ret = am_hal_mspi_disable(data->mspiHandle);
	if (ret) {
		return ret;
	}

  mspi_dev_cfg->eAddrCfg             = AM_HAL_MSPI_ADDR_4_BYTE;
  mspi_dev_cfg->eInstrCfg            = AM_HAL_MSPI_INSTR_1_BYTE;
  mspi_dev_cfg->bSendInstr           = true;
  mspi_dev_cfg->bSendAddr            = true;
  if (config->dummy_cycles)
  {
    mspi_dev_cfg->bTurnaround          = true;
    mspi_dev_cfg->ui8TurnAround        = config->dummy_cycles;
  }
  else
  {
    mspi_dev_cfg->bTurnaround          = false;
    mspi_dev_cfg->ui8TurnAround        = 0;
  }
  mspi_dev_cfg->ui16ReadInstr        = AM_DEVICES_MSPI_AMBT53_READ_BL4;
  mspi_dev_cfg->ui16WriteInstr       = AM_DEVICES_MSPI_AMBT53_WRITE_UL;
  mspi_dev_cfg->ui8WriteLatency      = 0;
  mspi_dev_cfg->bEnWriteLatency      = false;
  mspi_dev_cfg->bEmulateDDR          = false;
  mspi_dev_cfg->ui16DMATimeLimit     = 0;
  mspi_dev_cfg->eDMABoundary         = AM_HAL_MSPI_BOUNDARY_NONE;
	
	ret = am_hal_mspi_device_configure(data->mspiHandle, mspi_dev_cfg);
	if (ret) {
		return ret;
	}

	am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_PIOMIXED_CONFIG, &mspi_dev_cfg->eDeviceConfig);
	
	ret = am_hal_mspi_enable(data->mspiHandle);

#if defined(CONFIG_SOC_APOLLO4P) || defined(CONFIG_SOC_APOLLO4L) || defined(CONFIG_SOC_APOLLO5)
    am_hal_mspi_rxcfg_t RxCfg = gMspiRxCfg;
    ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_RXCFG, &RxCfg);
#endif

  if (config->dqs_en)
  {
    // Note: DQS pin should be properly enabled for this feature
    am_hal_mspi_dqs_t dqsCfg = AMBT53DqsCfg[custom_mspi_get_module_idx(dev)];
    ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_DQS, &dqsCfg);
  }

	return ret;
}

static int mspi_pio_write(const struct device *dev,
                        const struct mspi_buf *tx_buf)
{
	struct mspi_ambiq_data *data = dev->data;
	int ret;
    am_hal_mspi_pio_transfer_t transaction = {0};
    uint32_t timeout = 0;
	unsigned int key = 0;
    //
    // Create the transaction.
    //
    transaction.bScrambling        = false;
    transaction.eDirection         = AM_HAL_MSPI_TX;
    transaction.bSendInstr         = true;
    transaction.bTurnaround        = false;
    transaction.bEnWRLatency       = false;
    transaction.bDCX               = false;
    transaction.bSendAddr          = tx_buf->send_addr;
    transaction.ui32DeviceAddr     = tx_buf->addr;
    transaction.ui16DeviceInstr    = tx_buf->instr;
    transaction.pui32Buffer        = tx_buf->buf;
    transaction.ui32NumBytes       = tx_buf->len;

    // Execute the transction over MSPI.
    do
    {
        /* ==== ENTER CRITICAL SECTION ==== */
		key = irq_lock();
        ret = am_hal_mspi_blocking_transfer(data->mspiHandle, &transaction, MSPI_AMBT53_TRANSFER_TIMEOUT);
        /* ==== EXIT CRITICAL SECTION ==== */
		irq_unlock(key);
        // Some non-blocking transfer is not finished, wait a while
        if ((ret == AM_HAL_STATUS_INVALID_OPERATION) && (timeout < MSPI_AMBT53_TRANSFER_TIMEOUT))
        {
            timeout++;
        }
        else
        {
            break;
        }
    }while(1);

    return ret;
}

static int mspi_pio_read(const struct device *dev,
                        const struct mspi_buf *rx_buf)
{
	int ret;
    uint32_t timeout = 0;
	unsigned int key = 0;
    am_hal_mspi_pio_transfer_t transaction = {0};
	struct mspi_ambiq_data *data = dev->data;

    //
    // Create the transaction.
    //
    transaction.bScrambling        = false;
    transaction.bSendInstr         = true;
    transaction.bEnWRLatency       = false;
    transaction.bDCX               = false;
    transaction.bSendAddr          = rx_buf->send_addr;
    transaction.bTurnaround        = true;
    transaction.ui32DeviceAddr     = rx_buf->addr;
    transaction.eDirection         = AM_HAL_MSPI_RX;
    transaction.ui16DeviceInstr    = rx_buf->instr;
    transaction.pui32Buffer        = rx_buf->buf;
    transaction.ui32NumBytes       = rx_buf->len;

    //
    // Execute the transaction over MSPI.
    //
    do
    {
        /* ==== ENTER CRITICAL SECTION ==== */
		key = irq_lock();
        ret = am_hal_mspi_blocking_transfer(data->mspiHandle, &transaction, MSPI_AMBT53_TRANSFER_TIMEOUT);
        /* ==== EXIT CRITICAL SECTION ==== */
		irq_unlock(key);
        // Some non-blocking transfer is not finished, wait a while
        if ((ret == AM_HAL_STATUS_INVALID_OPERATION) && (timeout < MSPI_AMBT53_TRANSFER_TIMEOUT))
        {
            timeout++;
        }
        else
        {
            break;
        }
    }while(1);

    return ret;
}

static int mspi_ambiq_release(const struct device *dev)
{
	const struct mspi_ambiq_config *cfg = dev->config;
	struct mspi_ambiq_data *data = dev->data;
	int ret;

	if (sys_read32(cfg->base) & MSPI_BUSY) {
		return -EBUSY;
	}

	ret = am_hal_mspi_disable(data->mspiHandle);
	
	return ret;
}

static struct mspi_driver_api mspi_ambiq_driver_api = {
	.write = mspi_pio_write,
	.read = mspi_pio_read,
	.config = mspi_config,
	.release = mspi_ambiq_release,
};

//*****************************************************************************
//
//  Sets up the MSPI into XIP mode.
//
//*****************************************************************************
int custom_mspi_enable_xip(const struct device *dev)
{
  int ret;
  struct mspi_ambiq_data *data = dev->data;
  uint32_t module_idx = custom_mspi_get_module_idx(dev);

	do
	{
	//
	// Set Aperture XIP range
	//
	ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_XIP_CONFIG, &AMBT53XipConfig[module_idx]);
	if (AM_HAL_STATUS_SUCCESS != ret)
	{
		break;
	}

	#if ((CONFIG_SOC_APOLLO4P) || (CONFIG_SOC_APOLLO4L) || (CONFIG_SOC_APOLLO5))
		am_hal_mspi_xip_misc_t    xipMiscCfg = gXipMiscCfg[module_idx];
		ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_XIP_MISC_CONFIG, &xipMiscCfg);
		if (AM_HAL_STATUS_SUCCESS != ret)
		{
			break;
		}
	#endif
	//
	// Enable XIP on the MSPI.
	//
	ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_XIP_EN, &AMBT53XipConfig[module_idx]);

	} while (0);

	return ret;
}

//*****************************************************************************
//
//  Removes the MSPI from XIP mode.
//
//*****************************************************************************
int am_devices_mspi_ambt53_disable_xip(const struct device *dev)
{
  int ret;
  struct mspi_ambiq_data *data = dev->data;
  uint32_t module_idx = custom_mspi_get_module_idx(dev);

  //
  // Disable XIP on the MSPI.
  //
  ret = am_hal_mspi_control(data->mspiHandle, AM_HAL_MSPI_REQ_XIP_DIS, &AMBT53XipConfig[module_idx]);

  return ret;
}

#ifdef CONFIG_USERSPACE
static inline void z_vrfy_custom_mspi_write(const struct device *dev,
			     const struct mspi_buf *tx_bufs)
{
	z_impl_custom_mspi_write(dev, tx_bufs);
}
#include <syscalls/custom_mspi_write_mrsh.c>

static inline void z_vrfy_custom_mspi_read(const struct device *dev,
			     const struct mspi_buf *rx_bufs)
{
	z_impl_custom_mspi_read(dev, rx_bufs);
}
#include <syscalls/custom_mspi_read_mrsh.c>

static inline void z_vrfy_custom_mspi_config(const struct device *dev,
				     const struct mspi_config *config)
{
	z_impl_custom_mspi_config(dev, config);
}
#include <syscalls/custom_mspi_config_mrsh.c>

static inline void z_vrfy_custom_mspi_release(const struct device *dev)
{
	z_impl_custom_mspi_release(dev, config);
}
#include <syscalls/custom_mspi_release_mrsh.c>
#endif /* CONFIG_USERSPACE */

static int mspi_ambiq_init(const struct device *dev)
{
	struct mspi_ambiq_data *data = dev->data;
	const struct mspi_ambiq_config *cfg = dev->config;
	am_hal_mspi_config_t mspiCfg = {0};

	mspiCfg.pTCB = NULL;

	int ret = am_hal_mspi_initialize(custom_mspi_get_module_idx(dev),
					 &data->mspiHandle);
	if (ret) {
		return ret;
	}

	ret = cfg->pwr_func();

	// ret = am_hal_mspi_power_control(data->mspiHandle, AM_HAL_SYSCTRL_WAKE, false);
	// if (ret) {
	// 	return ret;
	// }

	ret = am_hal_mspi_configure(data->mspiHandle, &mspiCfg);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

	return ret;
}

#define AMBIQ_MSPI_DEFINE(n)                                                                       \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static int pwr_on_ambiq_mspi_##n(void)                                                     \
	{                                                                                          \
		uint32_t addr = DT_REG_ADDR(DT_INST_PHANDLE(n, ambiq_pwrcfg)) +                    \
				DT_INST_PHA(n, ambiq_pwrcfg, offset);                              \
		sys_write32((sys_read32(addr) | DT_INST_PHA(n, ambiq_pwrcfg, mask)), addr);        \
		k_busy_wait(PWRCTRL_MAX_WAIT_US);                                                  \
		return 0;                                                                          \
	}                                                                                          \
	static struct mspi_ambiq_data mspi_ambiq_data##n = {                                       \
		0};                                   \
	static const struct mspi_ambiq_config mspi_ambiq_config##n = {                             \
		.base = DT_INST_REG_ADDR_BY_NAME(n,qspi),                                                       \
		.xipmm_offset = DT_INST_REG_ADDR_BY_NAME(n,qspi_mm),                                                       \
		.size = DT_INST_REG_SIZE(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.pwr_func = pwr_on_ambiq_mspi_##n,                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, mspi_ambiq_init, NULL, &mspi_ambiq_data##n,                       \
			      &mspi_ambiq_config##n, POST_KERNEL, CONFIG_CUSTOM_SPI_INIT_PRIORITY,        \
			      &mspi_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_MSPI_DEFINE)
