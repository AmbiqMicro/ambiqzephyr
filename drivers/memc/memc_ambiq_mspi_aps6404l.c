/*
 * Copyright 2024 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_mspi_aps6404l
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mspi.h>
#include "mspi_ambiq_ap3.h"

LOG_MODULE_REGISTER(memc_ambiq_mspi_aps6404l, CONFIG_MEMC_LOG_LEVEL);

#define APM_VENDOR_ID 0xD

#define APS6404L_WRITE             0x02
#define APS6404L_READ              0x03
#define APS6404L_FAST_READ         0x0B
#define APS6404L_QUAD_MODE_ENTER   0x35
#define APS6404L_QUAD_WRITE        0x38
#define APS6404L_RESET_ENABLE      0x66
#define APS6404L_RESET_MEMORY      0x99
#define APS6404L_READ_ID           0x9F
#define APS6404L_HALF_SLEEP_ENTER  0xC0
#define APS6404L_QUAD_READ         0xEB
#define APS6404L_QUAD_MODE_EXIT    0xF5

struct memc_mspi_aps6404l_config {
    uint32_t                            port;
    uint32_t                            mem_size;

    const struct device                 *bus;
    struct mspi_dev_id                  dev_id;
    struct mspi_dev_cfg                 serial_cfg;
    struct mspi_dev_cfg                 quad_cfg;
    struct mspi_dev_cfg                 tar_dev_cfg;
    struct mspi_xip_cfg                 tar_xip_cfg;
    struct mspi_scramble_cfg            tar_scramble_cfg;
    struct mspi_ambiq_ap3_timing_cfg    tar_timing_cfg;
};

struct memc_mspi_aps6404l_data {
    struct mspi_dev_cfg                 dev_cfg;
    struct mspi_xip_cfg                 xip_cfg;
    struct mspi_scramble_cfg            scramble_cfg;
    struct mspi_ambiq_ap3_timing_cfg    timing_cfg;
    struct mspi_xfer_packet             trans;

    struct k_sem                        lock;
};

static int memc_mspi_aps6404l_command_write(const struct device *psram,
    uint8_t cmd, uint32_t addr, uint8_t *wdata, uint32_t length)
{
    const struct memc_mspi_aps6404l_config *cfg = psram->config;
    struct memc_mspi_aps6404l_data *data = psram->data;
    int ret;
    uint8_t    buffer[16];

    data->trans.eMode               = MSPI_PIO;
    data->trans.eDirection          = MSPI_TX;
    data->trans.pui32Buffer         = (uint32_t *)buffer;
    data->trans.ui32NumBytes        = length;
    data->trans.ui16DeviceInstr     = cmd;
    data->trans.bSendInstr          = true;
    data->trans.ui32DeviceAddr      = addr;
    data->trans.bSendAddr           = false;
    data->trans.bScrambling         = false;
    data->trans.bRXDummy            = false;
    data->trans.bTXDummy            = false;
    data->trans.bHoldCE             = false;

    if (wdata != NULL) {
        memcpy(buffer, wdata, length);
    }

    ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer_packet *)&data->trans);
    if (ret) {
        LOG_ERR("MSPI write transaction failed with code: %d/%u",
            ret, __LINE__);
        return -EIO;
    }
    return ret;
}

static int memc_mspi_aps6404l_command_read(const struct device *psram,
    uint8_t cmd, uint32_t addr, uint8_t *rdata, uint32_t length)
{
    const struct memc_mspi_aps6404l_config *cfg = psram->config;
    struct memc_mspi_aps6404l_data *data = psram->data;

    int ret;
    uint8_t    buffer[16];

    data->trans.eMode               = MSPI_PIO;
    data->trans.eDirection          = MSPI_RX;
    data->trans.pui32Buffer         = (uint32_t *)buffer;
    data->trans.ui32NumBytes        = length;
    data->trans.ui16DeviceInstr     = cmd;
    data->trans.bSendInstr          = true;
    data->trans.ui32DeviceAddr      = addr;
    data->trans.bSendAddr           = true;
    data->trans.bScrambling         = false;
    data->trans.bRXDummy            = false;
    data->trans.bTXDummy            = false;
    data->trans.bHoldCE             = false;

    ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer_packet *)&data->trans);
    if (ret) {
        LOG_ERR("MSPI read transaction failed with code: %d/%u",
            ret, __LINE__);
        return -EIO;
    }
    memcpy(rdata, buffer, length);
    return ret;
}

static void acquire(const struct device *psram)
{
    const struct memc_mspi_aps6404l_config *cfg = psram->config;
    struct memc_mspi_aps6404l_data *data = psram->data;

    k_sem_take(&data->lock, K_FOREVER);
    while (mspi_dev_config(cfg->bus, &cfg->dev_id, (const struct mspi_dev_cfg *)&data->dev_cfg));
}

static void release(const struct device *psram)
{
    const struct memc_mspi_aps6404l_config *cfg = psram->config;
    struct memc_mspi_aps6404l_data *data = psram->data;

    while (mspi_get_channel_status(cfg->bus, cfg->port));

    k_sem_give(&data->lock);
}

static int memc_mspi_aps6404l_reset(const struct device *psram)
{
    int ret;

    LOG_DBG("Resetting aps6404l/%u", __LINE__);

    ret = memc_mspi_aps6404l_command_write(psram, APS6404L_RESET_ENABLE, 0, NULL, 0);
    if (ret) {
        return ret;
    }
    ret = memc_mspi_aps6404l_command_write(psram, APS6404L_RESET_MEMORY, 0, NULL, 0);
    if (ret) {
        return ret;
    }
    /** We need to delay 5 ms to allow aps6404L pSRAM to reinitialize */
    k_busy_wait(5000);
    return ret;
}

static int memc_mspi_aps6404l_get_vendor_id(const struct device *psram,
            uint8_t *vendor_id)
{
    uint16_t buffer = 0;
    int ret;

    ret = memc_mspi_aps6404l_command_read(psram, APS6404L_READ_ID,
                                          0, (uint8_t *)&buffer, 2);
    LOG_DBG("Read ID buff: %x/%u", buffer, __LINE__);
    *vendor_id = buffer & 0xff;

    return ret;
}

#if CONFIG_PM_DEVICE
static int memc_ambiq_mspi_aps6404l_half_sleep_enter(const struct device *psram)
{
    int ret;

    LOG_DBG("Putting aps6404l to half sleep/%u", __LINE__);
    ret = memc_mspi_aps6404l_command_write(psram, APS6404L_HALF_SLEEP_ENTER, 0, NULL, 0);
    if (ret) {
        LOG_ERR("Failed to enter half sleep/%u", __LINE__);
        return ret;
    }
    /** Minimum half sleep duration tHS time */
    k_busy_wait(4);

    return ret;
}

static int memc_ambiq_mspi_aps6404l_half_sleep_exit(const struct device *psram)
{
    const struct memc_mspi_aps6404l_config *cfg = psram->config;
    struct memc_mspi_aps6404l_data *data = psram->data;
    struct mspi_dev_cfg bkp = data->dev_cfg;
    int ret = 0;

    data->dev_cfg.ui32Freq = 48000000;

    mspi_dev_config(cfg->bus, &cfg->dev_id, (const struct mspi_dev_cfg *)&data->dev_cfg);

    LOG_DBG("Waking up aps6404l from half sleep/%u", __LINE__);
    ret = memc_mspi_aps6404l_command_write(psram, 0, 0 , NULL, 0);
    if (ret) {
        LOG_ERR("Failed to exit from half sleep/%u", __LINE__);
        return ret;
    }
    /** Minimum half sleep exit CE to CLK setup time  */
    k_busy_wait(100);

    data->dev_cfg = bkp;

    ret = mspi_dev_config(cfg->bus, &cfg->dev_id, (const struct mspi_dev_cfg *)&data->dev_cfg);
    if (ret) {
        LOG_ERR("Failed to reconfigure MSPI after exiting half sleep/%u", __LINE__);
        return ret;
    }

    return ret;
}

static int memc_ambiq_mspi_aps6404l_pm_action(const struct device *psram,
                                              enum pm_device_action action)
{
    switch (action) {
        case PM_DEVICE_ACTION_RESUME:
            acquire(psram);
            memc_ambiq_mspi_aps6404l_half_sleep_exit(psram);
            release(psram);
            break;

        case PM_DEVICE_ACTION_SUSPEND:
            acquire(psram);
            memc_ambiq_mspi_aps6404l_half_sleep_enter(psram);
            release(psram);
            break;

        default:
            return -ENOTSUP;
    }

    return 0;
}
#endif /** IS_ENABLED(CONFIG_PM_DEVICE) */

static int memc_mspi_aps6404l_init(const struct device *psram)
{
    const struct memc_mspi_aps6404l_config *cfg = psram->config;
    struct memc_mspi_aps6404l_data *data = psram->data;
    uint8_t vendor_id;

    if (!device_is_ready(cfg->bus)) {
        LOG_ERR("Controller device not ready/%u", __LINE__);
        return -ENODEV;
    }

    switch (cfg->tar_dev_cfg.eIOMode) {
        case MSPI_IO_MODE_SINGLE:
        case MSPI_IO_MODE_QUAD:
            break;
        default:
            LOG_ERR("bus mode %d not supported/%u", cfg->tar_dev_cfg.eIOMode, __LINE__);
            return -EIO;
    }

    if (cfg->tar_dev_cfg.ui32CENum > 1) {
        LOG_ERR("config has incorrect CE number/%u", __LINE__);
        return -EIO;
    }

    if (data->dev_cfg.eIOMode == MSPI_IO_MODE_QUAD) {
        if (mspi_dev_config(cfg->bus, &cfg->dev_id, &cfg->quad_cfg)) {
            LOG_ERR("Failed to config mspi controller/%u", __LINE__);
            return -EIO;
        }
        data->dev_cfg = cfg->quad_cfg;
        if (memc_mspi_aps6404l_reset(psram)) {
            LOG_ERR("Could not reset pSRAM/%u", __LINE__);
            return -EIO;
        }
        if (memc_mspi_aps6404l_command_write(psram, APS6404L_QUAD_MODE_EXIT, 0, NULL, 0)) {
            LOG_ERR("Could not exit quad mode/%u", __LINE__);
            return -EIO;
        }
    }

    if (mspi_dev_config(cfg->bus, &cfg->dev_id, &cfg->serial_cfg)) {
        LOG_ERR("Failed to config mspi controller/%u", __LINE__);
        return -EIO;
    }
    data->dev_cfg = cfg->serial_cfg;

    if (memc_mspi_aps6404l_reset(psram)) {
        LOG_ERR("Could not reset pSRAM/%u", __LINE__);
        return -EIO;
    }

    if (memc_mspi_aps6404l_get_vendor_id(psram, &vendor_id)) {
        LOG_ERR("Could not read vendor id/%u", __LINE__);
        return -EIO;
    }
    LOG_DBG("Vendor id: 0x%0x", vendor_id);
    if (vendor_id != APM_VENDOR_ID) {
        LOG_WRN("Vendor ID does not match expected value of 0x%0x/%u",
            APM_VENDOR_ID,
            __LINE__);
    }

    if (cfg->tar_dev_cfg.eIOMode == MSPI_IO_MODE_QUAD) {
        if (memc_mspi_aps6404l_command_write(psram, APS6404L_QUAD_MODE_ENTER, 0, NULL, 0)) {
            return -EIO;
        }
    }

    if (mspi_dev_config(cfg->bus, &cfg->dev_id, &cfg->tar_dev_cfg)) {
        LOG_ERR("Failed to config mspi controller/%u", __LINE__);
        return -EIO;
    }
    data->dev_cfg = cfg->tar_dev_cfg;

    if (mspi_timing_config(cfg->bus, &cfg->dev_id, (void *)&cfg->tar_timing_cfg)) {
        LOG_ERR("Failed to config mspi timing/%u", __LINE__);
        return -EIO;
    }
    data->timing_cfg = cfg->tar_timing_cfg;

    if (cfg->tar_xip_cfg.bEnable) {
        if (mspi_xip_config(cfg->bus, &cfg->dev_id, &cfg->tar_xip_cfg)) {
            LOG_ERR("Failed to enable XIP/%u", __LINE__);
            return -EIO;
        }
        data->xip_cfg = cfg->tar_xip_cfg;
    }

    if (cfg->tar_scramble_cfg.bEnable) {
        if (mspi_scramble_config(cfg->bus, &cfg->dev_id, &cfg->tar_scramble_cfg)) {
            LOG_ERR("Failed to enable scrambling/%u", __LINE__);
            return -EIO;
        }
        data->scramble_cfg = cfg->tar_scramble_cfg;
    }

    release(psram);

    return 0;
}

#define MSPI_DEVICE_CONFIG_SERIAL(n)                                                              \
    {                                                                                             \
        .ui32CENum          = DT_INST_PROP(n, hardware_ce_num),                                   \
        .ui32Freq           = 12000000,                                                           \
        .eIOMode            = MSPI_IO_MODE_SINGLE,                                                \
        .eDataRate          = MSPI_SINGLE_DATA_RATE,                                              \
        .eCPP               = MSPI_CPP_MODE_0,                                                    \
        .eEndian            = MSPI_XFER_LITTLE_ENDIAN,                                            \
        .eCEPolarity        = MSPI_CE_ACTIVE_LOW,                                                 \
        .ui32RXDummy        = 8,                                                                  \
        .ui32TXDummy        = 0,                                                                  \
        .ui32ReadInstr      = APS6404L_FAST_READ,                                                 \
        .ui32WriteInstr     = APS6404L_WRITE,                                                     \
        .ui16InstrLength    = 0,                                                                  \
        .ui16AddrLength     = 2,                                                                  \
        .ui32MemBoundary    = 0x6,                                                                \
        .ui32BreakTimeLimit = 80,                                                                 \
    }

#define MSPI_DEVICE_CONFIG_QUAD(n)                                                                \
    {                                                                                             \
        .ui32CENum          = DT_INST_PROP(n, hardware_ce_num),                                   \
        .ui32Freq           = 12000000,                                                           \
        .eIOMode            = MSPI_IO_MODE_QUAD,                                                  \
        .eDataRate          = MSPI_SINGLE_DATA_RATE,                                              \
        .eCPP               = MSPI_CPP_MODE_0,                                                    \
        .eEndian            = MSPI_XFER_LITTLE_ENDIAN,                                            \
        .eCEPolarity        = MSPI_CE_ACTIVE_LOW,                                                 \
        .ui32RXDummy        = 6,                                                                  \
        .ui32TXDummy        = 0,                                                                  \
        .ui32ReadInstr      = APS6404L_QUAD_READ,                                                 \
        .ui32WriteInstr     = APS6404L_QUAD_WRITE,                                                \
        .ui16InstrLength    = 0,                                                                  \
        .ui16AddrLength     = 2,                                                                  \
        .ui32MemBoundary    = 0x6,                                                                \
        .ui32BreakTimeLimit = 30,                                                                 \
    }

#define MSPI_DEVICE_CONFIG(n)                                                                     \
    {                                                                                             \
        .ui32CENum          = DT_INST_PROP(n, hardware_ce_num),                                   \
        .ui32Freq           = DT_INST_PROP(n, mspi_max_frequency),                                \
        .eIOMode            = DT_INST_ENUM_IDX(n, mspi_io_mode),                                  \
        .eDataRate          = DT_INST_ENUM_IDX(n, mspi_data_rate),                                \
        .eCPP               = DT_INST_ENUM_IDX_OR(n, mspi_cpp_mode, MSPI_CPP_MODE_0),             \
        .eEndian            = DT_INST_ENUM_IDX_OR(n, mspi_endian, MSPI_XFER_LITTLE_ENDIAN),       \
        .eCEPolarity        = DT_INST_ENUM_IDX_OR(n, mspi_ce_polarity, MSPI_CE_ACTIVE_LOW),       \
        .ui32RXDummy        = DT_INST_PROP(n, rx_dummy),                                          \
        .ui32TXDummy        = DT_INST_PROP(n, tx_dummy),                                          \
        .ui32ReadInstr      = DT_INST_PROP(n, read_instruction),                                  \
        .ui32WriteInstr     = DT_INST_PROP(n, write_instruction),                                 \
        .ui16InstrLength    = DT_INST_ENUM_IDX(n, instruction_length),                            \
        .ui16AddrLength     = DT_INST_ENUM_IDX(n, address_length),                                \
        .ui32MemBoundary    = DT_INST_PROP_BY_IDX(n, ce_break_config, 0),                         \
        .ui32BreakTimeLimit = DT_INST_PROP_BY_IDX(n, ce_break_config, 1),                         \
    }

#define MSPI_XIP_CONFIG(n)                                                                        \
    {                                                                                             \
        .bEnable            = DT_INST_PROP_BY_IDX(n, xip_config, 0),                              \
        .ui32AddrOffset     = 0,                                                                  \
        .ui32Size           = 0,                                                                  \
        .ePermission        = MSPI_XIP_READ_WRITE,                                                \
    }

#define MSPI_SCRAMBLE_CONFIG(n)                                                                   \
    {                                                                                             \
        .bEnable            = DT_INST_PROP_BY_IDX(n, scramble_config, 0),                         \
        .ui32AddrOffset     = DT_INST_PROP_BY_IDX(n, scramble_config, 1),                         \
        .ui32Size           = DT_INST_PROP_BY_IDX(n, scramble_config, 2),                         \
    }

#define MSPI_TIMING_CONFIG(n)                                                                     \
    {                                                                                             \
        .bSendInstr         = DT_INST_PROP_BY_IDX(n, timing_config, 0),                           \
        .bSendAddr          = DT_INST_PROP_BY_IDX(n, timing_config, 1),                           \
        .bWriteLatency      = DT_INST_PROP_BY_IDX(n, timing_config, 2),                           \
        .bTurnaround        = DT_INST_PROP_BY_IDX(n, timing_config, 3),                           \
        .ui8WriteLatency    = DT_INST_PROP_BY_IDX(n, timing_config, 4),                           \
        .ui8TurnAround      = DT_INST_PROP_BY_IDX(n, timing_config, 5),                           \
    }

#define MSPI_DEVICE_ID(n)                                                                         \
    {                                                                                             \
        .ce                 = GPIO_DT_SPEC_GET_BY_IDX(DT_INST_BUS(n),                             \
                                                      ce_gpios,                                   \
                                                      DT_INST_REG_ADDR(n)),                       \
        .dev_idx            = DT_INST_REG_ADDR(n),                                                \
    }

#define MEMC_AMBIQ_MSPI_APS6404L(n)                                                               \
    static const struct memc_mspi_aps6404l_config                                                 \
        memc_mspi_aps6404l_config_##n = {                                                         \
        .port               = (DT_REG_ADDR(DT_INST_BUS(n)) - MSPI_REG_BASEADDR) /                 \
                              (DT_REG_SIZE(DT_INST_BUS(n)) * 4),                                  \
        .mem_size           = DT_INST_PROP(n, size) / 8,                                          \
        .bus                = DEVICE_DT_GET(DT_INST_BUS(n)),                                      \
        .dev_id             = MSPI_DEVICE_ID(n),                                                  \
        .serial_cfg         = MSPI_DEVICE_CONFIG_SERIAL(n),                                       \
        .quad_cfg           = MSPI_DEVICE_CONFIG_QUAD(n),                                         \
        .tar_dev_cfg        = MSPI_DEVICE_CONFIG(n),                                              \
        .tar_xip_cfg        = MSPI_XIP_CONFIG(n),                                                 \
        .tar_scramble_cfg   = MSPI_SCRAMBLE_CONFIG(n),                                            \
        .tar_timing_cfg     = MSPI_TIMING_CONFIG(n),                                              \
    };                                                                                            \
    static struct memc_mspi_aps6404l_data                                                         \
        memc_mspi_aps6404l_data_##n = {                                                           \
        .lock = Z_SEM_INITIALIZER(memc_mspi_aps6404l_data_##n.lock, 0, 1),                        \
    };                                                                                            \
    PM_DEVICE_DT_INST_DEFINE(n, memc_ambiq_mspi_aps6404l_pm_action);                              \
    DEVICE_DT_INST_DEFINE(n,                                                                      \
                          memc_mspi_aps6404l_init,                                                \
                          PM_DEVICE_DT_INST_GET(n),                                               \
                          &memc_mspi_aps6404l_data_##n,                                           \
                          &memc_mspi_aps6404l_config_##n,                                         \
                          POST_KERNEL,                                                            \
                          CONFIG_MEMC_INIT_PRIORITY,                                              \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(MEMC_AMBIQ_MSPI_APS6404L)
