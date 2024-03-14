/*
 * Copyright (c) 2019 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public APIs for MSPI driver
 */

#ifndef ZEPHYR_INCLUDE_MSPI_H_
#define ZEPHYR_INCLUDE_MSPI_H_

#include <errno.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MSPI Driver APIs
 * @defgroup mspi_interface MSPI Driver APIs
 * @ingroup io_interfaces
 * @{
 */

/**
 * @code
 * here contains introduction of MSPI architecture
 * @endcode
 */

/**
 * @name MSPI operational mode
 * @{
 */
enum mspi_op_mode {
    MSPI_OP_MODE_MASTER         = 0,
    MSPI_OP_MODE_SLAVE          = 1,
};

/**
 * @name MSPI duplex mode
 * @{
 */
enum mspi_duplex {
    MSPI_HALF_DUPLEX            = 0,
    MSPI_FULL_DUPLEX            = 1,
};

/**
 * @brief MSPI I/O mode capabilities
 */
enum mspi_io_mode {
    MSPI_IO_MODE_SINGLE         = 0,
    MSPI_IO_MODE_DUAL           = 1,
    MSPI_IO_MODE_DUAL_1_1_2     = 2,
    MSPI_IO_MODE_DUAL_1_2_2     = 3,
    MSPI_IO_MODE_QUAD           = 4,
    MSPI_IO_MODE_QUAD_1_1_4     = 5,
    MSPI_IO_MODE_QUAD_1_4_4     = 6,
    MSPI_IO_MODE_OCTAL          = 7,
    MSPI_IO_MODE_OCTAL_1_1_8    = 8,
    MSPI_IO_MODE_OCTAL_1_8_8    = 9,
    MSPI_IO_MODE_HEX            = 10,
};

/**
 * @brief MSPI data rate capabilities
 */
enum mspi_data_rate {
    MSPI_SINGLE_DATA_RATE       = 0,
    MSPI_DUAL_DATA_RATE         = 1,
};

/**
 * @name MSPI Polarity & Phase Modes
 * @{
 */
enum mspi_cpp_mode {
    MSPI_CPP_MODE_0             = 0,
    MSPI_CPP_MODE_1             = 1,
    MSPI_CPP_MODE_2             = 2,
    MSPI_CPP_MODE_3             = 3,
};

/**
 * @name MSPI Endian
 * @{
 */
enum mspi_endian {
    MSPI_XFER_LITTLE_ENDIAN     = 0,
    MSPI_XFER_BIG_ENDIAN        = 1,
};


/**
 * @name MSPI chip enable polarity
 * @{
 */
enum mspi_ce_polarity {
    MSPI_CE_ACTIVE_LOW          = 0,
    MSPI_CE_ACTIVE_HIGH         = 1,
};

/**
 * @brief MSPI Chip Select control structure
 *
 * This can be used to control a CE line via a GPIO line, instead of
 * using the controller inner CE logic.
 *
 */
struct mspi_ce_control {
    /**
     * GPIO devicetree specification of CE GPIO.
     * The device pointer can be set to NULL to fully inhibit CE control if
     * necessary. The GPIO flags GPIO_ACTIVE_LOW/GPIO_ACTIVE_HIGH should be
     * the same as in MSPI configuration.
     */
    struct gpio_dt_spec gpio;
    /**
     * Delay in microseconds to wait before starting the
     * transmission and before releasing the CE line.
     */
    uint32_t delay;
};

/**
 * @brief Get a <tt>struct gpio_dt_spec</tt> for a MSPI device's chip select pin
 *
 * Example devicetree fragment:
 *
 * @code{.devicetree}
 *     gpio1: gpio@abcd0001 { ... };
 *
 *     gpio2: gpio@abcd0002 { ... };
 *
 *     mspi@abcd0003 {
 *             compatible = "ambiq,mspi";
 *             ce-gpios = <&gpio1 10 GPIO_ACTIVE_LOW>,
 *                        <&gpio2 20 GPIO_ACTIVE_LOW>;
 *
 *             a: mspi-dev-a@0 {
 *                     reg = <0>;
 *             };
 *
 *             b: mspi-dev-b@1 {
 *                     reg = <1>;
 *             };
 *     };
 * @endcode
 *
 * Example usage:
 *
 * @code{.c}
 *     MSPI_CE_GPIOS_DT_SPEC_GET(DT_NODELABEL(a)) \
 *           // { DEVICE_DT_GET(DT_NODELABEL(gpio1)), 10, GPIO_ACTIVE_LOW }
 *     MSPI_CE_GPIOS_DT_SPEC_GET(DT_NODELABEL(b)) \
 *           // { DEVICE_DT_GET(DT_NODELABEL(gpio2)), 20, GPIO_ACTIVE_LOW }
 * @endcode
 *
 * @param mspi_dev a MSPI device node identifier
 * @return #gpio_dt_spec struct corresponding with mspi_dev's chip select
 */
#define MSPI_CE_GPIOS_DT_SPEC_GET(mspi_dev)                       \
        GPIO_DT_SPEC_GET_BY_IDX_OR(DT_BUS(mspi_dev), ce_gpios,    \
                   DT_REG_ADDR(mspi_dev), {})

/**
 * @brief Get a <tt>struct gpio_dt_spec</tt> for a MSPI device's chip select pin
 *
 * This is equivalent to
 * <tt>MSPI_CE_GPIOS_DT_SPEC_GET(DT_DRV_INST(inst))</tt>.
 *
 * @param inst Devicetree instance number
 * @return #gpio_dt_spec struct corresponding with mspi_dev's chip select
 */
#define MSPI_CE_GPIOS_DT_SPEC_INST_GET(inst) \
        MSPI_CE_GPIOS_DT_SPEC_GET(DT_DRV_INST(inst))

/**
 * @brief Initialize and get a pointer to a @p mspi_ce_control from a
 *        devicetree node identifier
 *
 * This helper is useful for initializing a device on a MSPI bus. It
 * initializes a struct mspi_ce_control and returns a pointer to it.
 * Here, @p node_id is a node identifier for a MSPI device, not a MSPI
 * controller.
 *
 * Example devicetree fragment:
 *
 * @code{.devicetree}
 *     mspi@abcd0001 {
 *             ce-gpios = <&gpio0 1 GPIO_ACTIVE_LOW>;
 *             mspidev: mspi-device@0 { ... };
 *     };
 * @endcode
 *
 * Example usage:
 *
 * @code{.c}
 *     struct mspi_ce_control ctrl =
 *             MSPI_CE_CONTROL_INIT(DT_NODELABEL(mspidev), 2);
 * @endcode
 *
 * This example is equivalent to:
 *
 * @code{.c}
 *     struct mspi_ce_control ctrl = {
 *             .gpio = MSPI_CE_GPIOS_DT_SPEC_GET(DT_NODELABEL(mspidev)),
 *             .delay = 2,
 *     };
 * @endcode
 *
 * @param node_id Devicetree node identifier for a device on a MSPI bus
 * @param delay_ The @p delay field to set in the @p mspi_ce_control
 * @return a pointer to the @p mspi_ce_control structure
 */
#define MSPI_CE_CONTROL_INIT(node_id, delay_)              \
    {                                                      \
        .gpio = MSPI_CE_GPIOS_DT_SPEC_GET(node_id),        \
        .delay = (delay_),                                 \
    }

/**
 * @brief Get a pointer to a @p mspi_ce_control from a devicetree node
 *
 * This is equivalent to
 * <tt>MSPI_CE_CONTROL_INIT(DT_DRV_INST(inst), delay)</tt>.
 *
 * Therefore, @p DT_DRV_COMPAT must already be defined before using
 * this macro.
 *
 * @param inst Devicetree node instance number
 * @param delay_ The @p delay field to set in the @p mspi_ce_control
 * @return a pointer to the @p mspi_ce_control structure
 */
#define MSPI_CE_CONTROL_INIT_INST(inst, delay_)            \
        MSPI_CE_CONTROL_INIT(DT_DRV_INST(inst), delay_)

/**
 * @brief MSPI device ID
 */
struct mspi_dev_id {
    /** @brief device gpio ce */
    struct gpio_dt_spec     ce;
    /** @brief device index on DT */
    unsigned int            dev_idx;
};

/**
 * @brief MSPI controller configuration
 */
struct mspi_cfg {
    /** @brief mspi channel number */
    uint8_t                 ui8MSPIChannel;
    /** @brief Configure operaton mode */
    enum mspi_op_mode       eOPMode;
    /** @brief Configure duplex mode */
    enum mspi_duplex        eDuplex;
    /** @brief DQS support flag */
    bool                    bDQS;
    /** @brief GPIO chip-select line (optional) */
    struct gpio_dt_spec     *pCE;
    /** @brief Slave number from 0 to host controller slave limit. */
    uint32_t                ui32SlaveNum;
    /** @brief Maximum supported frequency in MHz */
    uint32_t                ui32MaxFreq;
    /** @brief Whether to re-initialize controller */
    bool                    bReinit;
};

/**
 * @brief MSPI DT information
 */
struct mspi_dt_spec {
	/** @brief MSPI bus */
	const struct device     *bus;
	/** @brief MSPI hardware specific configuration */
	struct mspi_cfg         config;
};

/**
 * @brief MSPI controller device specific configuration mask
 */
enum mspi_dev_cfg_mask {
    MSPI_DEVICE_CONFIG_CE_NUM       = BIT(0),
    MSPI_DEVICE_CONFIG_FREQUENCY    = BIT(1),
    MSPI_DEVICE_CONFIG_IO_MODE      = BIT(2),
    MSPI_DEVICE_CONFIG_DATA_RATE    = BIT(3),
    MSPI_DEVICE_CONFIG_CPP          = BIT(4),
    MSPI_DEVICE_CONFIG_ENDIAN       = BIT(5),
    MSPI_DEVICE_CONFIG_CE_POL       = BIT(6),
    MSPI_DEVICE_CONFIG_DQS          = BIT(7),
    MSPI_DEVICE_CONFIG_RX_DUMMY     = BIT(8),
    MSPI_DEVICE_CONFIG_TX_DUMMY     = BIT(9),
    MSPI_DEVICE_CONFIG_READ_INSTR   = BIT(10),
    MSPI_DEVICE_CONFIG_WRITE_INSTR  = BIT(11),
    MSPI_DEVICE_CONFIG_INSTR_LEN    = BIT(12),
    MSPI_DEVICE_CONFIG_ADDR_LEN     = BIT(13),
    MSPI_DEVICE_CONFIG_MEM_BOUND    = BIT(14),
    MSPI_DEVICE_CONFIG_BREAK_TIME   = BIT(15),
    MSPI_DEVICE_CONFIG_ALL          = -1,
};

/**
 * @brief MSPI controller device specific configuration
 */
struct mspi_dev_cfg {
    /** @brief Configure CE0 or CE1 */
    uint32_t                ui32CENum;
    /** @brief Configure frequency */
    uint32_t                ui32Freq;
    /** @brief Configure I/O mode */
    enum mspi_io_mode       eIOMode;
    /** @brief Configure data rate */
    enum mspi_data_rate     eDataRate;
    /** @brief Configure clock polarity and phase*/
    enum mspi_cpp_mode      eCPP;
    /** @brief Configure transfer endian */
    enum mspi_endian        eEndian;
    /** @brief Configure chip enable polarity */
    enum mspi_ce_polarity   eCEPolarity;
    /** @brief Configure DQS mode */
    bool                    bDQSEnable;
    /** @brief Configure number of clock cycles between
     * addr and data in RX direction
     */
    uint32_t                ui32RXDummy;
    /** @brief Configure number of clock cycles between
     * addr and data in TX direction
     */
    uint32_t                ui32TXDummy;
    /** @brief Configure read instruction */
    uint32_t                ui32ReadInstr;
    /** @brief Configure write instruction */
    uint32_t                ui32WriteInstr;
    /** @brief Configure instruction length */
    uint16_t                ui16InstrLength;
    /** @brief Configure address length */
    uint16_t                ui16AddrLength;
    /** @brief Configure memory boundary */
    uint32_t                ui32MemBoundary;
    /** @brief Configure break time */
    uint32_t                ui32BreakTimeLimit;
};

/**
 * @brief MSPI XIP access permissions
 */
enum mspi_xip_permit {
    MSPI_XIP_READ_WRITE     = 0,
    MSPI_XIP_READ_ONLY      = 1,
};

/**
 * @brief MSPI controller XIP configuration
 */
struct mspi_xip_cfg {
    /** @brief XIP enable */
    bool                    bEnable;
    /** @brief XIP region start address =
     * hardware default + address offset
    */
    uint32_t                ui32AddrOffset;
    /** @brief XIP region size */
    uint32_t                ui32Size;
    /** @brief XIP access permission */
    enum mspi_xip_permit    ePermission;
};

/**
 * @brief MSPI controller scramble configuration
 */
struct mspi_scramble_cfg {
    /** @brief scramble enable */
    bool                    bEnable;
    /** @brief scramble region start address =
     * hardware default + address offset
    */
    uint32_t                ui32AddrOffset;
    /** @brief scramble region size */
    uint32_t                ui32Size;
};

/**
 * @brief MSPI bus event.
 */
enum mspi_bus_event {
    MSPI_BUS_RESET                      = 0,
    MSPI_BUS_ERROR                      = 1,
    MSPI_BUS_XFER_COMPLETE              = 2,
    MSPI_BUS_EVENT_MAX,
};

/**
 * @brief MSPI bus event callback mask
 */
enum mspi_bus_event_cb_mask {
    MSPI_BUS_NO_CB                      = 0,
    MSPI_BUS_RESET_CB                   = BIT(0),
    MSPI_BUS_ERROR_CB                   = BIT(1),
    MSPI_BUS_XFER_COMPLETE_CB           = BIT(2),
};

/**
 * @brief MSPI transfer modes
 */
enum eTransMode{
    MSPI_PIO,
    MSPI_DMA,
};

/**
 * @brief MSPI transfer directions
 */
enum eTransDirection{
    MSPI_RX,
    MSPI_TX,
};

/**
 * @brief MSPI transfer buffer
 */
struct mspi_buf {
    /** @brief  Device Instruction           */
    uint16_t                    ui16DeviceInstr;
    /** @brief  Device Address               */
    uint32_t                    ui32DeviceAddr;
    /** @brief  Number of bytes to transfer  */
    uint32_t                    ui32NumBytes;
    /** @brief  Buffer                       */
    uint32_t                    *pui32Buffer;
};

/**
 * @brief MSPI peripheral xfer packet format
 */
struct mspi_xfer_packet {
    /** @brief  Transfer Mode                */
    enum eTransMode             eMode;
    /** @brief  Direction (Transmit/Receive) */
    enum eTransDirection        eDirection;
    /** @brief  Enable scrambling            */
    bool                        bScrambling;
    /** @brief  Configure TX dummy cycles    */
    uint32_t                    ui32TXDummy;
    /** @brief  Configure RX dummy cycles    */
    uint32_t                    ui32RXDummy;
    /** @brief Configure instruction length  */
    uint16_t                    ui16InstrLength;
    /** @brief Configure address length      */
    uint16_t                    ui16AddrLength;
    /** @brief  Hold CE active after xfer    */
    bool                        bHoldCE;
    /** @brief  Software CE control          */
    struct mspi_ce_control      sCE;
    /** @brief  Priority 0 = Low (best effort)
     *                   1 = High (service immediately)
    */
    uint8_t                     ui8Priority;
    /** @brief  Transfer buffers             */
    struct mspi_buf             *pPayload;
    /** @brief  Number of transfer buffers   */
    uint32_t                    ui32NumPayload;
    /** @brief  Bus event callback masks     */
    enum mspi_bus_event_cb_mask eCBMask;
};

/**
 * @brief MSPI event data
 */
struct mspi_event_data {
    /** @brief Pointer to the slave device ID */
    const struct mspi_dev_id *dev_id;
    /** @brief Pointer to the transfer packet */
    const struct mspi_xfer_packet *xfer;
    /** @brief MSPI event status */
    uint32_t status;
};

/**
 * @brief MSPI event
 */
struct mspi_event {
    /** Event type */
    enum mspi_bus_event     evt_type;
    /** Data associated to the event */
    struct mspi_event_data  evt_data;
};

/**
 * @brief MSPI callback context
 */
struct mspi_callback_context {
    /** @brief MSPI event  */
    struct mspi_event mspi_evt;
    /** @brief user defined context */
    void              *ctx;
};

struct mspi_timing_cfg;
enum mspi_timing_param;

/**
 * @typedef mspi_callback_handler_t
 * @brief Define the application callback handler function signature.
 *
 * @param controller Device struct for the MSPI device.
 * @param cb Original struct mspi_callback owning this handler.
 * @param mspi_evt event details that trigger the callback handler.
 *
 */
typedef void (*mspi_callback_handler_t) (const struct device *controller,
                     struct mspi_callback_context *mspi_cb_ctx);

/**
 * MSPI driver API definition and system call entry points
 */
typedef int (*mspi_api_config)(const struct mspi_dt_spec *spec);

typedef int (*mspi_api_dev_config)(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const enum mspi_dev_cfg_mask param_mask,
                    const struct mspi_dev_cfg *cfg);

typedef int (*mspi_api_get_channel_status)(const struct device *controller,
                    uint8_t ch);

typedef int (*mspi_api_transceive)(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const struct mspi_xfer_packet *req);

typedef int (*mspi_api_transceive_async)(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const struct mspi_xfer_packet *req);

typedef int (*mspi_api_register_callback)(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const enum mspi_bus_event evt_type,
                    mspi_callback_handler_t cb,
                    struct mspi_callback_context *ctx);

typedef int (*mspi_api_xip_config)(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const struct mspi_xip_cfg *xip_cfg);

typedef int (*mspi_api_scramble_config)(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const struct mspi_scramble_cfg *scramble_cfg);

typedef int (*mspi_api_timing_config)(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const uint32_t param_mask,
                    void *timing_cfg);

__subsystem struct mspi_driver_api {
    mspi_api_config                 config;
    mspi_api_dev_config             dev_config;
    mspi_api_get_channel_status     get_channel_status;
    mspi_api_transceive             transceive;
    mspi_api_transceive_async       transceive_async;
    mspi_api_register_callback      register_callback;
    mspi_api_xip_config             xip_config;
    mspi_api_scramble_config        scramble_config;
    mspi_api_timing_config          timing_config;
};

/**
 * @brief Configure a MSPI controller.
 *
 * This routine provides a generic interface to override MSPI controller
 * capabilities.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param cfg The controller configuration for MSPI.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error, failed to configure device.
 * @retval -EINVAL invalid capabilities, failed to configure device.
 * @retval -ENOTSUP capability not supported by MSPI slave.
 */
__syscall int mspi_config(const struct mspi_dt_spec *spec);

static inline int z_impl_mspi_config(const struct mspi_dt_spec *spec)
{
    const struct mspi_driver_api *api =
        (const struct mspi_driver_api *)spec->bus->api;

    return api->config(spec);
}

/**
 * @brief Configure a MSPI controller with device specific parameters.
 *
 * This routine provides a generic interface to override MSPI controller
 * capabilities.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param param_mask Macro definition of what to be configured in cfg.
 * @param cfg The device runtime configuration for the MSPI controller.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error, failed to configure device.
 * @retval -EINVAL invalid capabilities, failed to configure device.
 * @retval -ENOTSUP capability not supported by MSPI slave.
 */
__syscall int mspi_dev_config(const struct device *controller,
                     const struct mspi_dev_id *dev_id,
                     const enum mspi_dev_cfg_mask param_mask,
                     const struct mspi_dev_cfg *cfg);

static inline int z_impl_mspi_dev_config(const struct device *controller,
                     const struct mspi_dev_id *dev_id,
                     const enum mspi_dev_cfg_mask param_mask,
                     const struct mspi_dev_cfg *cfg)
{
    const struct mspi_driver_api *api =
        (const struct mspi_driver_api *)controller->api;

    return api->dev_config(controller, dev_id, param_mask, cfg);
}

/**
 * @brief Query to see if it a channel is ready.
 *
 * This routine allows to check if logical channel is ready before use.
 * Note that queries for channels not supported will always return false.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param ch the MSPI channel for which status is to be retrieved.
 *
 * @retval 0 If MSPI channel is ready.
 */
__syscall int mspi_get_channel_status(const struct device *controller,
                        uint8_t ch);

static inline int z_impl_mspi_get_channel_status(const struct device *controller,
                        uint8_t ch)
{
    const struct mspi_driver_api *api =
        (const struct mspi_driver_api *)controller->api;

    return api->get_channel_status(controller, ch);
}

/**
 * @brief Transfer request over MSPI.
 *
 * This routines provides a generic interface to transfer a request synchronously.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param req Content of the request and request specific settings.
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP
 * @retval -EIO General input / output error, failed to send over the bus.
 */
__syscall int mspi_transceive(const struct device *controller,
                       const struct mspi_dev_id *dev_id,
                       const struct mspi_xfer_packet *req);

static inline int z_impl_mspi_transceive(const struct device *controller,
                       const struct mspi_dev_id *dev_id,
                       const struct mspi_xfer_packet *req)
{
    const struct mspi_driver_api *api =
        (const struct mspi_driver_api *)controller->api;

    if (!api->transceive) {
        return -ENOTSUP;
    }

    return api->transceive(controller, dev_id, req);
}

/**
 * @brief Transfer request over MSPI.
 *
 * This routines provides a generic interface to transfer a request asynchronously.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param req Content of the request and request specific settings.
 * @param
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP
 * @retval -EINVAL General input / output error, failed to send over the bus.
 */
__syscall int mspi_transceive_async(const struct device *controller,
                        const struct mspi_dev_id *dev_id,
                        const struct mspi_xfer_packet *req);

static inline int z_impl_mspi_transceive_async(const struct device *controller,
                        const struct mspi_dev_id *dev_id,
                        const struct mspi_xfer_packet *req)
{
    const struct mspi_driver_api *api =
        (const struct mspi_driver_api *)controller->api;

    if (!api->transceive_async) {
        return -ENOTSUP;
    }

    return api->transceive_async(controller, dev_id, req);
}

/**
 * @brief Register the mspi callback functions.
 *
 * This routines provides a generic interface to register mspi callback functions.
 * In generall it should be called before mspi_transceive_async.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param evt_type The event type associated the callback.
 * @param cb Pointer to the user implemented callback function.
 * @param ctx Pointer to the callback context.
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP
 */
__syscall int mspi_register_callback(const struct device *controller,
                        const struct mspi_dev_id *dev_id,
                        const enum mspi_bus_event evt_type,
                        mspi_callback_handler_t cb,
                        struct mspi_callback_context *ctx);

static inline int z_impl_mspi_register_callback(const struct device *controller,
                        const struct mspi_dev_id *dev_id,
                        const enum mspi_bus_event evt_type,
                        mspi_callback_handler_t cb,
                        struct mspi_callback_context *ctx)
{
    const struct mspi_driver_api *api =
        (const struct mspi_driver_api *)controller->api;

    if (!api->register_callback) {
        return -ENOTSUP;
    }

    return api->register_callback(controller, dev_id, evt_type, cb, ctx);
}

/**
 * @brief Configure a MSPI XIP settings.
 *
 * This routine provides a generic interface to override MSPI controller
 * capabilities.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param cfg The controller XIP configuration for MSPI.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error, failed to configure device.
 * @retval -EINVAL invalid capabilities, failed to configure device.
 * @retval -ENOTSUP capability not supported by MSPI slave.
 */
__syscall int mspi_xip_config(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const struct mspi_xip_cfg *cfg);

static inline int z_impl_mspi_xip_config(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const struct mspi_xip_cfg *cfg)
{
    const struct mspi_driver_api *api =
        (const struct mspi_driver_api *)controller->api;

    return api->xip_config(controller, dev_id, cfg);
}

/**
 * @brief Configure a MSPI scrambling settings.
 *
 * This routine provides a generic interface to override MSPI controller
 * capabilities.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param cfg The controller scramble configuration for MSPI.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error, failed to configure device.
 * @retval -EINVAL invalid capabilities, failed to configure device.
 * @retval -ENOTSUP capability not supported by MSPI slave.
 */
__syscall int mspi_scramble_config(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const struct mspi_scramble_cfg *cfg);

static inline int z_impl_mspi_scramble_config(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const struct mspi_scramble_cfg *cfg)
{
    const struct mspi_driver_api *api =
        (const struct mspi_driver_api *)controller->api;

    return api->scramble_config(controller, dev_id, cfg);
}

/**
 * @brief Configure a MSPI timing settigs.
 *
 * This routine provides a generic interface to override MSPI controller
 * capabilities.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param param_mask The macro defintion of what should be configured in cfg.
 * @param cfg The controller timing configuration for MSPI.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error, failed to configure device.
 * @retval -EINVAL invalid capabilities, failed to configure device.
 * @retval -ENOTSUP capability not supported by MSPI slave.
 */
__syscall int mspi_timing_config(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const uint32_t param_mask,
                    void *cfg);

static inline int z_impl_mspi_timing_config(const struct device *controller,
                    const struct mspi_dev_id *dev_id,
                    const uint32_t param_mask,
                    void *cfg)
{
    const struct mspi_driver_api *api =
        (const struct mspi_driver_api *)controller->api;

    return api->timing_config(controller, dev_id, param_mask, cfg);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#include <syscalls/mspi.h>
#endif /* ZEPHYR_INCLUDE_MSPI_H_ */
