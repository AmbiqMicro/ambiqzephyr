/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_mspi_controller

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_instance.h>
LOG_MODULE_REGISTER(mspi_ambiq_ap3);
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys_clock.h>
#include <zephyr/irq.h>

#include <am_mcu_apollo.h>

#include "mspi_ambiq_ap3.h"

#define MSPI_MAX_FREQ        48000000
#define MSPI_MAX_DEVICE      2
#define MSPI_TIMEOUT_US      1000000
#define PWRCTRL_MAX_WAIT_US  5
#define MSPI_BUSY            BIT(2)

typedef int (*mspi_ambiq_pwr_func_t)(void);
typedef void (*irq_config_func_t)(void);

struct mspi_context {
    const struct mspi_dev_id      *owner;

    const struct mspi_xfer_packet *xfer;
    uint32_t                      xfer_count;

    mspi_callback_handler_t       callback;
    struct mspi_callback_context  *callback_ctx;
    bool asynchronous;

    struct k_sem lock;
    int sync_status;
};

struct mspi_ambiq_config {
    uint32_t                        reg_base;
    uint32_t                        reg_size;

    struct mspi_cfg                 mspicfg;

    const struct pinctrl_dev_config *pcfg;
    mspi_ambiq_pwr_func_t           pwr_func;
    irq_config_func_t               irq_cfg_func;

    LOG_INSTANCE_PTR_DECLARE(log);
};

struct mspi_ambiq_data {
    void                            *mspiHandle;
    am_hal_mspi_dev_config_t        hal_dev_cfg;

    struct mspi_dev_id              *dev_id;
    struct k_mutex                  lock;

    struct mspi_dev_cfg             dev_cfg;
    struct mspi_xip_cfg             xip_cfg;
    struct mspi_scramble_cfg        scramble_cfg;

    struct mspi_context             ctx;
};

static int mspi_set_freq(const struct mspi_ambiq_config *cfg, uint32_t freq)
{
    uint32_t d = MSPI_MAX_FREQ / freq;

    switch (d) {
    case AM_HAL_MSPI_CLK_48MHZ:
    case AM_HAL_MSPI_CLK_24MHZ:
    case AM_HAL_MSPI_CLK_16MHZ:
    case AM_HAL_MSPI_CLK_12MHZ:
    case AM_HAL_MSPI_CLK_8MHZ:
    case AM_HAL_MSPI_CLK_6MHZ:
    case AM_HAL_MSPI_CLK_4MHZ:
    case AM_HAL_MSPI_CLK_3MHZ:
        break;
    default:
        LOG_INST_ERR(cfg->log, "%u,Frequency not supported!", __LINE__);
        d = 0;
        break;
    }

    return d;
}

static am_hal_mspi_device_e mspi_set_line(const struct mspi_ambiq_config *cfg,
                                          enum mspi_io_mode eIOMode,
                                          enum mspi_data_rate eDataRate,
                                          uint32_t ui32CENum)
{
    if (eDataRate != MSPI_SINGLE_DATA_RATE) {
        LOG_INST_ERR(cfg->log, "%u, mspi_data_rate incorrect, only SDR is supported",
                                __LINE__);
        return AM_HAL_MSPI_FLASH_MAX;
    }

    if (ui32CENum == 0) {
        switch (eIOMode) {
            case MSPI_IO_MODE_SINGLE:
                return AM_HAL_MSPI_FLASH_SERIAL_CE0;
            case MSPI_IO_MODE_DUAL:
                return AM_HAL_MSPI_FLASH_DUAL_CE0;
            case MSPI_IO_MODE_DUAL_1_1_2:
                return AM_HAL_MSPI_FLASH_DUAL_CE0_1_1_2;
            case MSPI_IO_MODE_DUAL_1_2_2:
                return AM_HAL_MSPI_FLASH_DUAL_CE0_1_2_2;
            case MSPI_IO_MODE_QUAD:
                return AM_HAL_MSPI_FLASH_QUAD_CE0;
            case MSPI_IO_MODE_QUAD_1_1_4:
                return AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4;
            case MSPI_IO_MODE_QUAD_1_4_4:
                return AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4;
            case MSPI_IO_MODE_OCTAL:
                return AM_HAL_MSPI_FLASH_OCTAL_CE0;
            default:
                return AM_HAL_MSPI_FLASH_MAX;
        }
    }else if (ui32CENum == 1) {
        switch (eIOMode) {
            case MSPI_IO_MODE_SINGLE:
                return AM_HAL_MSPI_FLASH_SERIAL_CE1;
            case MSPI_IO_MODE_DUAL:
                return AM_HAL_MSPI_FLASH_DUAL_CE1;
            case MSPI_IO_MODE_DUAL_1_1_2:
                return AM_HAL_MSPI_FLASH_DUAL_CE1_1_1_2;
            case MSPI_IO_MODE_DUAL_1_2_2:
                return AM_HAL_MSPI_FLASH_DUAL_CE1_1_2_2;
            case MSPI_IO_MODE_QUAD:
                return AM_HAL_MSPI_FLASH_QUAD_CE1;
            case MSPI_IO_MODE_QUAD_1_1_4:
                return AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4;
            case MSPI_IO_MODE_QUAD_1_4_4:
                return AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4;
            case MSPI_IO_MODE_OCTAL:
                return AM_HAL_MSPI_FLASH_OCTAL_CE1;
            default:
                return AM_HAL_MSPI_FLASH_MAX;
        }
    }else {
        return AM_HAL_MSPI_FLASH_MAX;
    }
}

static inline void mspi_context_ce_control(struct mspi_context *ctx, bool on)
{
 if (ctx->owner && ctx->xfer) {
        if (ctx->xfer->bHoldCE &&
            ctx->xfer->sCE.gpio.port != NULL) {
            if (on) {
                gpio_pin_set_dt(&ctx->xfer->sCE.gpio, 1);
                k_busy_wait(ctx->xfer->sCE.delay);
            } else {
                k_busy_wait(ctx->xfer->sCE.delay);
                gpio_pin_set_dt(&ctx->xfer->sCE.gpio, 0);
            }
        }
    }
}

static inline void mspi_context_unlock_unconditionally(struct mspi_context *ctx)
{
    mspi_context_ce_control(ctx, false);

    if (!k_sem_count_get(&ctx->lock)) {
        ctx->owner = NULL;
        k_sem_give(&ctx->lock);
    }
}

static inline void mspi_context_lock(struct mspi_context *ctx,
                                     const struct mspi_dev_id *req,
                                     const struct mspi_xfer_packet *xfer,
                                     bool asynchronous,
                                     mspi_callback_handler_t callback,
                                     struct mspi_callback_context *callback_ctx,
                                     bool lockon)
{
    if ((k_sem_count_get(&ctx->lock) == 0) && !lockon &&
        (ctx->owner == req)) {
            ctx->xfer_count++;
            return;
    }

    k_sem_take(&ctx->lock, K_FOREVER);
    ctx->owner = req;
    ctx->xfer = xfer;
    ctx->xfer_count = 1;
    ctx->asynchronous = asynchronous;
    if (callback) {
        ctx->callback = callback;
        ctx->callback_ctx = callback_ctx;
    }
    if (!ctx->asynchronous) {
        ctx->sync_status = ~0;
    }
}

static inline void mspi_context_release(struct mspi_context *ctx, int status)
{
    ctx->owner = NULL;
    ctx->xfer = NULL;
    ctx->xfer_count = 0;
    k_sem_give(&ctx->lock);
}

static inline void mspi_context_complete(struct mspi_context *ctx, int status)
{
    if (!ctx->asynchronous) {
        ctx->sync_status = status;
    }
}

static inline bool mspi_is_inp(const struct device *controller)
{
    struct mspi_ambiq_data *data = controller->data;
    return (k_sem_count_get(&data->ctx.lock) == 0);
}

static inline int mspi_verify_device(const struct device *controller,
                                     const struct mspi_dev_id *dev_id)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    int device_index = cfg->mspicfg.ui32SlaveNum;
    int ret = 0;

    for (int i = 0; i < cfg->mspicfg.ui32SlaveNum; i++) {
        if (dev_id->ce.port == cfg->mspicfg.pCE[i].port &&
            dev_id->ce.pin == cfg->mspicfg.pCE[i].pin &&
            dev_id->ce.dt_flags == cfg->mspicfg.pCE[i].dt_flags)
        {
            device_index = i;
        }
    }

    if (device_index >= cfg->mspicfg.ui32SlaveNum ||
        device_index != dev_id->dev_idx) {
        LOG_INST_ERR(cfg->log, "%u, unable to find the specific mspi device", __LINE__);
        return -ENODEV;
    }

    return ret;
}

static int mspi_xfer_config(const struct device *controller,
                            const struct mspi_xfer_packet *xfer)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;
    am_hal_mspi_dev_config_t hal_dev_cfg = data->hal_dev_cfg;
    am_hal_mspi_request_e eRequest;
    int ret = 0;

    if (xfer->bScrambling) {
        eRequest = AM_HAL_MSPI_REQ_SCRAMB_EN;
    } else {
        eRequest = AM_HAL_MSPI_REQ_SCRAMB_DIS;
    }

    ret = am_hal_mspi_disable(data->mspiHandle);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to disable MSPI, code:%d",
                        __LINE__, ret);
        return -EHOSTDOWN;
    }

    ret = am_hal_mspi_control(data->mspiHandle, eRequest, NULL);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u,Unable to complete scramble config:%d",
                    __LINE__, xfer->bScrambling);
        return -EHOSTDOWN;
    }

    hal_dev_cfg.bSendAddr           = xfer->bSendAddr;
    hal_dev_cfg.bSendInstr          = xfer->bSendInstr;
    if (xfer->eDirection == MSPI_RX) {
        hal_dev_cfg.ui8ReadInstr    = xfer->ui16DeviceInstr;
        hal_dev_cfg.bTurnaround     = xfer->bRXDummy;
    } else {
        hal_dev_cfg.ui8WriteInstr   = xfer->ui16DeviceInstr;
        hal_dev_cfg.bEnWriteLatency = xfer->bTXDummy;
    }

    ret = am_hal_mspi_device_configure(data->mspiHandle, &hal_dev_cfg);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to configure MSPI, code:%d",
                        __LINE__, ret);
        return -EHOSTDOWN;
    }

    ret = am_hal_mspi_enable(data->mspiHandle);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to enable MSPI, code:%d",
                        __LINE__, ret);
        return -EHOSTDOWN;
    }

    data->hal_dev_cfg = hal_dev_cfg;
    data->scramble_cfg.bEnable = xfer->bScrambling;
    return ret;
}

static int mspi_ambiq_config(const struct device *controller,
                             const struct mspi_cfg *config)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;

    int ret = 0;

    if (config->eOPMode != MSPI_OP_MODE_MASTER) {
        LOG_INST_ERR(cfg->log, "%u, only support MSPI master", __LINE__);
        return -ENOTSUP;
    }

    if (config->ui32MaxFreq > MSPI_MAX_FREQ) {
        LOG_INST_ERR(cfg->log, "%u, ui32MaxFreq too large", __LINE__);
        return -ENOTSUP;
    }

    if (config->eDuplex != MSPI_HALF_DUPLEX) {
        LOG_INST_ERR(cfg->log, "%u, only support half duplex mode", __LINE__);
        return -ENOTSUP;
    }

    ret = am_hal_mspi_initialize(config->ui8MSPIChannel, &data->mspiHandle);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to initialize MSPI, code:%d",
                     __LINE__, ret);
        return -EPERM;
    }

    ret = cfg->pwr_func();
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to power on MSPI, code:%d",
                     __LINE__, ret);
        return -EHOSTDOWN;
    }

    ret = am_hal_mspi_enable(data->mspiHandle);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to Enable MSPI, code:%d",
                     __LINE__, ret);
        return -EHOSTDOWN;
    }

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret) {
        return ret;
    }

    ret = am_hal_mspi_interrupt_clear(data->mspiHandle, AM_HAL_MSPI_INT_CQUPD |
                                                        AM_HAL_MSPI_INT_ERR);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to clear interrupt, code:%d",
                     __LINE__, ret);
        return -EHOSTDOWN;
    }

    ret = am_hal_mspi_interrupt_enable(data->mspiHandle, AM_HAL_MSPI_INT_CQUPD |
                                                         AM_HAL_MSPI_INT_ERR);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to turn on interrupt, code:%d",
                     __LINE__, ret);
        return -EHOSTDOWN;
    }

    cfg->irq_cfg_func();

    mspi_context_unlock_unconditionally(&data->ctx);

    return ret;
}


static int mspi_ambiq_dev_config(const struct device *controller,
                                 const struct mspi_dev_id *dev_id,
                                 const struct mspi_dev_cfg *dev_cfg)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;
    am_hal_mspi_dev_config_t hal_dev_cfg = data->hal_dev_cfg;
    int ret = 0;

    if (data->dev_id != dev_id) {
        k_mutex_lock(&data->lock, K_FOREVER);

        ret = mspi_verify_device(controller, dev_id);
        if (ret) {
            goto e_return;
        }
    }

    if (mspi_is_inp(controller)) {
        ret = -EBUSY;
        goto e_return;
    }

    if (data->dev_id != dev_id) {
        ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_PRIV_START + dev_id->dev_idx);
        if (ret) {
            goto e_return;
        }
    }

    if (dev_cfg->eEndian != MSPI_XFER_LITTLE_ENDIAN) {
        LOG_INST_ERR(cfg->log, "%u, only support MSB first", __LINE__);
        ret = -ENOTSUP;
        goto e_return;
    }

    hal_dev_cfg.eSpiMode = dev_cfg->eCPP;

    hal_dev_cfg.ui8WriteLatency = dev_cfg->ui32TXDummy;
    hal_dev_cfg.ui8TurnAround = dev_cfg->ui32RXDummy;

    hal_dev_cfg.eClockFreq = mspi_set_freq(cfg, dev_cfg->ui32Freq);
    if (hal_dev_cfg.eClockFreq == 0) {
        ret = -ENOTSUP;
        goto e_return;
    }

    hal_dev_cfg.eDeviceConfig = mspi_set_line(cfg, dev_cfg->eIOMode,
                                                    dev_cfg->eDataRate,
                                                    dev_cfg->ui32CENum);
    if (hal_dev_cfg.eDeviceConfig == AM_HAL_MSPI_FLASH_MAX) {
        ret = -ENOTSUP;
        goto e_return;
    }

    if (dev_cfg->ui16InstrLength > AM_HAL_MSPI_INSTR_2_BYTE) {
        LOG_INST_ERR(cfg->log, "%u, ui16InstrLength too large", __LINE__);
        ret = -ENOTSUP;
        goto e_return;
    }
    hal_dev_cfg.eInstrCfg = dev_cfg->ui16InstrLength;

    if (dev_cfg->ui16AddrLength > AM_HAL_MSPI_ADDR_4_BYTE) {
        LOG_INST_ERR(cfg->log, "%u, ui16AddrLength too large", __LINE__);
        ret = -ENOTSUP;
        goto e_return;
    }
    hal_dev_cfg.eAddrCfg = dev_cfg->ui16AddrLength;

    hal_dev_cfg.ui8ReadInstr = (uint8_t)dev_cfg->ui32ReadInstr;
    hal_dev_cfg.ui8WriteInstr = (uint8_t)dev_cfg->ui32WriteInstr;

    if (dev_cfg->ui32MemBoundary > AM_HAL_MSPI_BOUNDARY_MAX) {
        LOG_INST_ERR(cfg->log, "%u, ui32MemBoundary too large", __LINE__);
        ret = -ENOTSUP;
        goto e_return;
    }
    hal_dev_cfg.eDMABoundary = dev_cfg->ui32MemBoundary;

    hal_dev_cfg.ui16DMATimeLimit = dev_cfg->ui32BreakTimeLimit;

    ret = am_hal_mspi_disable(data->mspiHandle);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to disable MSPI, code:%d",
                        __LINE__, ret);
        ret = -EHOSTDOWN;
        goto e_return;
    }

    ret = am_hal_mspi_device_configure(data->mspiHandle, &hal_dev_cfg);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to configure MSPI, code:%d",
                        __LINE__, ret);
        ret = -EHOSTDOWN;
        goto e_return;
    }

    ret = am_hal_mspi_enable(data->mspiHandle);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to enable MSPI, code:%d",
                        __LINE__, ret);
        ret = -EHOSTDOWN;
        goto e_return;
    }
    data->dev_cfg = *dev_cfg;
    data->hal_dev_cfg = hal_dev_cfg;
    data->dev_id = (struct mspi_dev_id *)dev_id;
    return ret;

e_return:
    k_mutex_unlock(&data->lock);
    return ret;
}

static int mspi_ambiq_xip_config(const struct device *controller,
                                 const struct mspi_dev_id *dev_id,
                                 const struct mspi_xip_cfg *xip_cfg)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;
    am_hal_mspi_request_e eRequest;
    int ret = 0;

    if (dev_id != data->dev_id) {
        LOG_INST_ERR(cfg->log, "%u, dev_id don't match", __LINE__);
        return -ESTALE;
    }

    if (xip_cfg->bEnable) {
        eRequest = AM_HAL_MSPI_REQ_XIP_EN;
    } else {
        eRequest = AM_HAL_MSPI_REQ_XIP_DIS;
    }

    ret = am_hal_mspi_control(data->mspiHandle, eRequest, NULL);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u,Unable to complete xip config:%d",
                    __LINE__, xip_cfg->bEnable);
        return -EHOSTDOWN;
    }

    data->xip_cfg = *xip_cfg;
    return ret;
}

static int mspi_ambiq_scramble_config(const struct device *controller,
                                      const struct mspi_dev_id *dev_id,
                                      const struct mspi_scramble_cfg *scramble_cfg)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;
    am_hal_mspi_dev_config_t hal_dev_cfg = data->hal_dev_cfg;
    am_hal_mspi_request_e eRequest;
    int ret = 0;

    if (mspi_is_inp(controller)) {
        return -EBUSY;
    }

    if (dev_id != data->dev_id) {
        LOG_INST_ERR(cfg->log, "%u, dev_id don't match", __LINE__);
        return -ESTALE;
    }

    if (scramble_cfg->bEnable) {
        eRequest = AM_HAL_MSPI_REQ_SCRAMB_EN;
    } else {
        eRequest = AM_HAL_MSPI_REQ_SCRAMB_DIS;
    }

    ret = am_hal_mspi_disable(data->mspiHandle);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to disable MSPI, code:%d",
                        __LINE__, ret);
        return -EHOSTDOWN;
    }

    ret = am_hal_mspi_control(data->mspiHandle, eRequest, NULL);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u,Unable to complete scramble config:%d",
                    __LINE__, scramble_cfg->bEnable);
        return -EHOSTDOWN;
    }

    hal_dev_cfg.scramblingStartAddr = 0 + scramble_cfg->ui32AddrOffset;
    hal_dev_cfg.scramblingEndAddr = hal_dev_cfg.scramblingStartAddr + scramble_cfg->ui32Size;

    ret = am_hal_mspi_device_configure(data->mspiHandle, &hal_dev_cfg);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to configure MSPI, code:%d",
                        __LINE__, ret);
        return -EHOSTDOWN;
    }

    ret = am_hal_mspi_enable(data->mspiHandle);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to enable MSPI, code:%d",
                        __LINE__, ret);
        return -EHOSTDOWN;
    }

    data->scramble_cfg = *scramble_cfg;
    data->hal_dev_cfg = hal_dev_cfg;
    return ret;
}

static int mspi_ambiq_timing_config(const struct device *controller,
                                    const struct mspi_dev_id *dev_id,
                                    void *timing_cfg)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;
    am_hal_mspi_dev_config_t hal_dev_cfg = data->hal_dev_cfg;
    struct mspi_ambiq_ap3_timing_cfg *time_cfg = timing_cfg;
    int ret = 0;

    if (mspi_is_inp(controller)) {
        return -EBUSY;
    }

    if (dev_id != data->dev_id) {
        LOG_INST_ERR(cfg->log, "%u, dev_id don't match", __LINE__);
        return -ESTALE;
    }

    hal_dev_cfg.ui8TurnAround = time_cfg->ui8TurnAround;
    hal_dev_cfg.ui8WriteLatency = time_cfg->ui8WriteLatency;
    hal_dev_cfg.bEnWriteLatency = time_cfg->bWriteLatency;
    hal_dev_cfg.bTurnaround = time_cfg->bTurnaround;

    hal_dev_cfg.bSendInstr = time_cfg->bSendInstr;
    hal_dev_cfg.bSendAddr = time_cfg->bSendAddr;

    ret = am_hal_mspi_disable(data->mspiHandle);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to disable MSPI, code:%d",
                        __LINE__, ret);
        return -EHOSTDOWN;
    }

    ret = am_hal_mspi_device_configure(data->mspiHandle, &hal_dev_cfg);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to configure MSPI, code:%d",
                        __LINE__, ret);
        return -EHOSTDOWN;
    }

    ret = am_hal_mspi_enable(data->mspiHandle);
    if (ret) {
        LOG_INST_ERR(cfg->log, "%u, Fail to enable MSPI, code:%d",
                        __LINE__, ret);
        return -EHOSTDOWN;
    }

    data->hal_dev_cfg = hal_dev_cfg;
    return ret;
}

static int mspi_ambiq_get_channel_status(const struct device *controller, uint8_t ch)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;
    int ret = 0;

    if (sys_read32(cfg->reg_base) & MSPI_BUSY) {
        ret = -EBUSY;
    }

    if (mspi_is_inp(controller)) {
        return -EBUSY;
    }

    k_mutex_unlock(&data->lock);

    return ret;
}

static void mspi_ambiq_isr(const struct device *dev)
{
    struct mspi_ambiq_data *data = dev->data;
    uint32_t      ui32Status;
    am_hal_mspi_interrupt_status_get(data->mspiHandle, &ui32Status, false);
    am_hal_mspi_interrupt_clear(data->mspiHandle, ui32Status);
    am_hal_mspi_interrupt_service(data->mspiHandle, ui32Status);
}

static void hal_mspi_callback(void *pCallbackCtxt, uint32_t status)
{
    const struct device *controller = pCallbackCtxt;
    struct mspi_ambiq_data *data = controller->data;
    if (data->ctx.callback != NULL) {
        data->ctx.callback_ctx->mspi_evt.evt_type = MSPI_BUS_XFER_COMPLETE;
        data->ctx.callback_ctx->mspi_evt.evt_data.xfer = data->ctx.xfer;
        data->ctx.callback_ctx->mspi_evt.evt_data.status = status;
        data->ctx.callback(controller, data->ctx.callback_ctx);
        mspi_context_release(&data->ctx, 0);
    } else {
        mspi_context_complete(&data->ctx, status);
    }
}

static int mspi_pio_transceive(const struct device *controller,
                               const struct mspi_xfer_packet *xfer,
                               bool asynchronous,
                               mspi_callback_handler_t cb,
                               struct mspi_callback_context *cb_context)
{
    struct mspi_ambiq_data *data = controller->data;
    am_hal_mspi_pio_transfer_t trans;
    int ret = 0;

    trans.eDirection          = xfer->eDirection;
    trans.ui32NumBytes        = xfer->ui32NumBytes;
    trans.bScrambling         = xfer->bScrambling;
    trans.bSendAddr           = xfer->bSendAddr;
    trans.ui32DeviceAddr      = xfer->ui32DeviceAddr;
    trans.bSendInstr          = xfer->bSendInstr;
    trans.ui16DeviceInstr     = xfer->ui16DeviceInstr;
    trans.bTurnaround         = xfer->bRXDummy;
    trans.bEnWRLatency        = xfer->bTXDummy;
    trans.pui32Buffer         = xfer->pui32Buffer;
    trans.bDCX                = false;
    trans.bQuadCmd            = false;
    trans.bContinue           = false;

    if (!asynchronous) {
        mspi_context_lock(&data->ctx, data->dev_id, xfer, asynchronous,
                          NULL, NULL, true);

        ret = am_hal_mspi_blocking_transfer(data->mspiHandle, &trans, MSPI_TIMEOUT_US);

        mspi_context_release(&data->ctx, ret);
    } else {

        mspi_context_lock(&data->ctx, data->dev_id, xfer, asynchronous,
                          cb, cb_context, false);
        //enable interrupt here
        ret = am_hal_mspi_nonblocking_transfer(data->mspiHandle, &trans, MSPI_PIO,
                                               hal_mspi_callback, (void *)controller);
        if (ret == AM_HAL_STATUS_OUT_OF_RANGE) {
            return -ENOMEM;
        }
    }

    if (ret)
    {
        return -EIO;
    }

    return ret;
}

static int mspi_dma_transceive(const struct device *controller,
                               const struct mspi_xfer_packet *xfer,
                               bool asynchronous,
                               mspi_callback_handler_t cb,
                               struct mspi_callback_context *cb_context)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;
    am_hal_mspi_dma_transfer_t trans;
    int ret = 0;

    trans.ui8Priority           = xfer->ui8Priority;
    trans.eDirection            = xfer->eDirection;
    trans.ui32TransferCount     = xfer->ui32NumBytes;
    trans.ui32DeviceAddress     = xfer->ui32DeviceAddr;
    trans.ui32SRAMAddress       = (uint32_t)xfer->pui32Buffer;
    trans.ui32PauseCondition    = xfer->ui32PauseCondition;
    trans.ui32StatusSetClr      = xfer->ui32StatusSetClr;

    if(!asynchronous) {
        mspi_context_lock(&data->ctx, data->dev_id, xfer, asynchronous,
                          NULL, NULL, true);

        ret = mspi_xfer_config(controller, xfer);
        //disable interrupt here
        ret = am_hal_mspi_nonblocking_transfer(data->mspiHandle,
                &trans, MSPI_DMA, hal_mspi_callback, (void *)controller);
        while (data->ctx.sync_status) {
            uint32_t      ui32Status;
            am_hal_mspi_interrupt_status_get(data->mspiHandle, &ui32Status, false);
            am_hal_mspi_interrupt_clear(data->mspiHandle, ui32Status);
            am_hal_mspi_interrupt_service(data->mspiHandle, ui32Status);
            k_busy_wait(10);
        }

        mspi_context_release(&data->ctx, ret);
    } else {

        mspi_context_lock(&data->ctx, data->dev_id, xfer, asynchronous,
                          cb, cb_context, false);

        if (data->ctx.xfer_count > 1) {
            LOG_INST_DBG(cfg->log, "%u, ignoring xfer config params", __LINE__);
        } else {
            mspi_xfer_config(controller, xfer);
        }
        // enable interrupt here
        ret = am_hal_mspi_nonblocking_transfer(data->mspiHandle,
                &trans, MSPI_DMA, hal_mspi_callback, (void *)controller);
        if (ret == AM_HAL_STATUS_OUT_OF_RANGE) {
            return -ENOMEM;
        }
    }

    if (ret)
    {
        return -EIO;
    }

    return ret;
}

static int mspi_ambiq_transceive(const struct device *controller,
                                 const struct mspi_dev_id *dev_id,
                                 const struct mspi_xfer_packet *xfer)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;

    if (dev_id != data->dev_id)
    {
        LOG_INST_ERR(cfg->log, "%u, dev_id don't match", __LINE__);
        return -ESTALE;
    }

    if (xfer->eMode == MSPI_PIO) {
        return mspi_pio_transceive(controller, xfer, false, NULL, NULL);
    } else if (xfer->eMode == MSPI_DMA) {
        return mspi_dma_transceive(controller, xfer, false, NULL, NULL);
    } else {
        return -EIO;
    }

}
#if CONFIG_MSPI_ASYNC
static int mspi_ambiq_transceive_async(const struct device *controller,
                                       const struct mspi_dev_id *dev_id,
                                       const struct mspi_xfer_packet *xfer,
                                       mspi_callback_handler_t cb,
                                       struct mspi_callback_context *ctx)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;

    if (dev_id != data->dev_id)
    {
        LOG_INST_ERR(cfg->log, "%u, dev_id don't match", __LINE__);
        return -ESTALE;
    }

    if (xfer->eMode == MSPI_PIO) {
        return mspi_pio_transceive(controller, xfer, true, cb, ctx);
    } else if (xfer->eMode == MSPI_DMA) {
        return mspi_dma_transceive(controller, xfer, true, cb, ctx);
    } else {
        return -EIO;
    }
}
#endif

#if CONFIG_PM_DEVICE
static int mspi_ambiq_pm_action(const struct device *controller,
                                enum pm_device_action action)
{
    const struct mspi_ambiq_config *cfg = controller->config;
    struct mspi_ambiq_data *data = controller->data;
    int ret = 0;

    if (mspi_is_inp(controller)) {
        return -EBUSY;
    }

    switch (action) {
        case PM_DEVICE_ACTION_TURN_ON:
            ret = am_hal_mspi_power_control(data->mspiHandle, AM_HAL_SYSCTRL_WAKE, true);
            if (ret) {
                LOG_INST_ERR(cfg->log, "%u, Fail to power on MSPI, code:%d",
                             __LINE__, ret);
                return -EHOSTDOWN;
            }
            break;

        case PM_DEVICE_ACTION_TURN_OFF:
            ret = am_hal_mspi_power_control(data->mspiHandle, AM_HAL_SYSCTRL_DEEPSLEEP, true);
            if (ret) {
                LOG_INST_ERR(cfg->log, "%u, Fail to power off MSPI, code:%d",
                             __LINE__, ret);
                return -EHOSTDOWN;
            }
            break;

        default:
            return -ENOTSUP;
    }

    return 0;
}
#endif

static int mspi_ambiq_init(const struct device *controller)
{
    const struct mspi_ambiq_config *cfg = controller->config;

    return mspi_ambiq_config(controller, &cfg->mspicfg);
}

static struct mspi_driver_api mspi_ambiq_driver_api = {
    .config                = mspi_ambiq_config,
    .dev_config            = mspi_ambiq_dev_config,
    .xip_config            = mspi_ambiq_xip_config,
    .scramble_config       = mspi_ambiq_scramble_config,
    .timing_config         = mspi_ambiq_timing_config,
    .get_channel_status    = mspi_ambiq_get_channel_status,
    .transceive            = mspi_ambiq_transceive,
#if CONFIG_MSPI_ASYNC
    .transceive_async      = mspi_ambiq_transceive_async,
#endif
};

#define MSPI_PINCTRL_STATE_INIT(state_idx, node_id)                                         \
    COND_CODE_1(Z_PINCTRL_SKIP_STATE(state_idx, node_id), (),                               \
    ({                                                                                      \
        .id = state_idx,                                                                    \
        .pins = Z_PINCTRL_STATE_PINS_NAME(state_idx, node_id),                              \
        .pin_cnt = ARRAY_SIZE(Z_PINCTRL_STATE_PINS_NAME(state_idx,                          \
                                node_id))                                                   \
    }))

#define MSPI_PINCTRL_STATES_DEFINE(node_id)                                                 \
    static const struct pinctrl_state                                                       \
    Z_PINCTRL_STATES_NAME(node_id)[] = {                                                    \
        LISTIFY(DT_NUM_PINCTRL_STATES(node_id),                                             \
                MSPI_PINCTRL_STATE_INIT, (,), node_id)                                      \
    };

#define MSPI_PINCTRL_DT_DEFINE(node_id)                                                     \
    LISTIFY(DT_NUM_PINCTRL_STATES(node_id),                                                 \
                Z_PINCTRL_STATE_PINS_DEFINE, (;), node_id);                                 \
    MSPI_PINCTRL_STATES_DEFINE(node_id)                                                     \
    Z_PINCTRL_DEV_CONFIG_STATIC Z_PINCTRL_DEV_CONFIG_CONST                                  \
    struct pinctrl_dev_config Z_PINCTRL_DEV_CONFIG_NAME(node_id) =                          \
    Z_PINCTRL_DEV_CONFIG_INIT(node_id)

#define MSPI_CONFIG(n)                                                                      \
    {                                                                                       \
        .ui8MSPIChannel      = (DT_INST_REG_ADDR(n) - MSPI_REG_BASEADDR) /                  \
                               (DT_INST_REG_SIZE(n) * 4),                                   \
        .eOPMode             = MSPI_OP_MODE_MASTER,                                         \
        .eDuplex             = MSPI_HALF_DUPLEX,                                            \
        .ui32MaxFreq         = MSPI_MAX_FREQ,                                               \
    }

#define MSPI_HAL_DEVICE_CONFIG(n, cmdq, cmdq_size)                                          \
    {                                                                                       \
        .ui8WriteLatency     = 0,                                                           \
        .ui8TurnAround       = 0,                                                           \
        .eAddrCfg            = 0,                                                           \
        .eInstrCfg           = 0,                                                           \
        .ui8ReadInstr        = 0,                                                           \
        .ui8WriteInstr       = 0,                                                           \
        .eDeviceConfig       = AM_HAL_MSPI_FLASH_SERIAL_CE0,                                \
        .eSpiMode            = AM_HAL_MSPI_SPI_MODE_0,                                      \
        .eClockFreq          = MSPI_MAX_FREQ /                                              \
                               DT_INST_PROP_OR(n, clock_frequency, MSPI_MAX_FREQ),          \
        .bEnWriteLatency     = false,                                                       \
        .bSendAddr           = false,                                                       \
        .bSendInstr          = false,                                                       \
        .bTurnaround         = false,                                                       \
        .bEmulateDDR         = false,                                                       \
        .ui16DMATimeLimit    = 0,                                                           \
        .eDMABoundary        = AM_HAL_MSPI_BOUNDARY_NONE,                                   \
        .ui32TCBSize         = cmdq_size,                                                   \
        .pTCB                = cmdq,                                                        \
        .scramblingStartAddr = 0,                                                           \
        .scramblingEndAddr   = 0,                                                           \
    }

#define AMBIQ_MSPI_DEFINE(n)                                                                \
    LOG_INSTANCE_REGISTER(DT_DRV_INST(n), mspi##n, CONFIG_MSPI_LOG_LEVEL);                  \
    MSPI_PINCTRL_DT_DEFINE(DT_DRV_INST(n));                                                 \
    static void mspi_ambiq_irq_cfg_func_##n(void)                                           \
    {                                                                                       \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                              \
            mspi_ambiq_isr, DEVICE_DT_INST_GET(n), 0);                                      \
        irq_enable(DT_INST_IRQN(n));                                                        \
    }                                                                                       \
    static int pwr_on_mspi_ambiq_##n(void)                                                  \
    {                                                                                       \
        uint32_t addr = DT_REG_ADDR(DT_INST_PHANDLE(n, ambiq_pwrcfg)) +                     \
                DT_INST_PHA(n, ambiq_pwrcfg, offset);                                       \
        sys_write32((sys_read32(addr) | DT_INST_PHA(n, ambiq_pwrcfg, mask)), addr);         \
        k_busy_wait(PWRCTRL_MAX_WAIT_US);                                                   \
        return 0;                                                                           \
    }                                                                                       \
    static uint32_t mspi_ambiq_cmdq##n[DT_INST_PROP_OR(n, cmdq_buffer_size, 1024) / 4]      \
    __attribute__((section(DT_INST_PROP_OR(n, cmdq_buffer_location, "RAM"))));              \
    static struct gpio_dt_spec ce_gpios##n[] = {                                            \
        COND_CODE_1(DT_NODE_HAS_PROP(DT_DRV_INST(n), ce_gpios),                             \
        (DT_INST_FOREACH_PROP_ELEM_SEP(n, ce_gpios, GPIO_DT_SPEC_GET_BY_IDX, (,))),         \
        ())                                                                                 \
    };                                                                                      \
    static struct mspi_ambiq_data mspi_ambiq_data##n = {                                    \
        .hal_dev_cfg = MSPI_HAL_DEVICE_CONFIG(n, mspi_ambiq_cmdq##n,                        \
                                        DT_INST_PROP_OR(n, cmdq_buffer_size, 1024)),        \
        .dev_id = 0,                                                                        \
        .lock = Z_MUTEX_INITIALIZER(mspi_ambiq_data##n.lock),                               \
        .dev_cfg = {0},                                                                     \
        .xip_cfg = {0},                                                                     \
        .scramble_cfg = {0},                                                                \
        .ctx.lock = Z_SEM_INITIALIZER(mspi_ambiq_data##n.ctx.lock, 0, 1),                   \
    };                                                                                      \
    static const struct mspi_ambiq_config mspi_ambiq_config##n = {                          \
        .reg_base              = DT_INST_REG_ADDR(n),                                       \
        .reg_size              = DT_INST_REG_SIZE(n),                                       \
        .mspicfg               = MSPI_CONFIG(n),                                            \
        .mspicfg.pCE           = (struct gpio_dt_spec *)ce_gpios##n,                        \
        .mspicfg.ui32SlaveNum  = sizeof(ce_gpios##n) / sizeof(struct gpio_dt_spec),         \
        .pcfg                  = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                         \
        .pwr_func              = pwr_on_mspi_ambiq_##n,                                     \
        .irq_cfg_func          = mspi_ambiq_irq_cfg_func_##n,                               \
        LOG_INSTANCE_PTR_INIT(log, DT_DRV_INST(n), mspi##n)                                 \
    };                                                                                      \
    PM_DEVICE_DT_INST_DEFINE(n, mspi_ambiq_pm_action);                                      \
    DEVICE_DT_INST_DEFINE(n,                                                                \
                          mspi_ambiq_init,                                                  \
                          PM_DEVICE_DT_INST_GET(n),                                         \
                          &mspi_ambiq_data##n,                                              \
                          &mspi_ambiq_config##n,                                            \
                          POST_KERNEL,                                                      \
                          CONFIG_MSPI_INIT_PRIORITY,                                        \
                          &mspi_ambiq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_MSPI_DEFINE)
