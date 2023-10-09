/*
 * Copyright (c) 2019 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __AMBT53_XIP_DRIVER_H__
#define __AMBT53_XIP_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/device.h>

// ********************************XSPI********************************

//! @defgroup XSPI XSPI
//! @{

#define REG_XSPI_BASE_ADDR                          0x03000B00

#define XSPI_TRANSFER_STATUS_ADDR                   (REG_XSPI_BASE_ADDR + 0x0)
#define XSPI_TRANSFER_STATUS_RD_MASK                ((uint32_t)0xFFFFBFFF)
#define XSPI_TRANSFER_STATUS_RW_MASK                ((uint32_t)0x000000FC)
#define XSPI_TRANSFER_STATUS_RESET                  0xA5FF0000

#define XSPI_ADDR_REG_ADDR                          (REG_XSPI_BASE_ADDR + 0xC)
#define XSPI_ADDR_REG_RD_MASK                       ((uint32_t)0xFFFFFFFF)
#define XSPI_ADDR_REG_RESET                         0x00000000

#define XSPI_RX_DATA0_ADDR                          (REG_XSPI_BASE_ADDR + 0x10)
#define XSPI_RX_DATA0_RD_MASK                       ((uint32_t)0xFFFFFFFF)
#define XSPI_RX_DATA0_RESET                         0x00000000

#define XSPI_RX_DATA1_ADDR                          (REG_XSPI_BASE_ADDR + 0x14)
#define XSPI_RX_DATA1_RD_MASK                       ((uint32_t)0xFFFFFFFF)
#define XSPI_RX_DATA1_RESET                         0x00000000

#define XSPI_TX_DATA0_ADDR                          (REG_XSPI_BASE_ADDR + 0x20)
#define XSPI_TX_DATA0_RD_MASK                       ((uint32_t)0xFFFFFFFF)
#define XSPI_TX_DATA0_RESET                         0x00000000

#define XSPI_TX_DATA1_ADDR                          (REG_XSPI_BASE_ADDR + 0x24)
#define XSPI_TX_DATA1_RD_MASK                       ((uint32_t)0xFFFFFFFF)
#define XSPI_TX_DATA1_RESET                         0x00000000


//*****************************************************************************
//
//! @name Global definitions for the commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_AMBT53_SOFT_RESET         0x99
#define AM_DEVICES_MSPI_AMBT53_WRITE_1BYTE        0x60
#define AM_DEVICES_MSPI_AMBT53_WRITE_2BYTES       0x61
// 0x60~0x6F used for 1~16 bytes writing
#define AM_DEVICES_MSPI_AMBT53_WRITE_BL1          0x70
#define AM_DEVICES_MSPI_AMBT53_WRITE_BL2          0x71
#define AM_DEVICES_MSPI_AMBT53_WRITE_BL4          0x72
#define AM_DEVICES_MSPI_AMBT53_WRITE_BL8          0x73
#define AM_DEVICES_MSPI_AMBT53_WRITE_BL16         0x74
#define AM_DEVICES_MSPI_AMBT53_WRITE_BL32         0x75
#define AM_DEVICES_MSPI_AMBT53_WRITE_BL64         0x76
#define AM_DEVICES_MSPI_AMBT53_WRITE_BL128        0x77
#define AM_DEVICES_MSPI_AMBT53_WRITE_BL256        0x78
#define AM_DEVICES_MSPI_AMBT53_WRITE_UL           0x7F
#define AM_DEVICES_MSPI_AMBT53_READ_1BYTE         0x80
#define AM_DEVICES_MSPI_AMBT53_READ_2BYTES        0x81
// 0x80~0x8F used for 1~16 bytes reading
#define AM_DEVICES_MSPI_AMBT53_READ_BL1           0xA0
#define AM_DEVICES_MSPI_AMBT53_READ_BL2           0xA1
#define AM_DEVICES_MSPI_AMBT53_READ_BL4           0xA2
#define AM_DEVICES_MSPI_AMBT53_READ_BL8           0xA3
#define AM_DEVICES_MSPI_AMBT53_READ_BL16          0xA4
#define AM_DEVICES_MSPI_AMBT53_READ_BL32          0xA5
#define AM_DEVICES_MSPI_AMBT53_READ_BL64          0xA6
#define AM_DEVICES_MSPI_AMBT53_READ_BL128         0xA7
#define AM_DEVICES_MSPI_AMBT53_READ_BL256         0xA8
#define AM_DEVICES_MSPI_AMBT53_READ_UL            0xC0
#define AM_DEVICES_MSPI_AMBT53_READ_STS           0x10
#define AM_DEVICES_MSPI_AMBT53_READ_ADR           0x11
#define AM_DEVICES_MSPI_AMBT53_READ_RXD           0x12
#define AM_DEVICES_MSPI_AMBT53_READ_TXD           0x13
#define AM_DEVICES_MSPI_AMBT53_READ_CFG           0x15
#define AM_DEVICES_MSPI_AMBT53_READ_VER           0x16
#define AM_DEVICES_MSPI_AMBT53_WRITE_STS          0x14
#define AM_DEVICES_MSPI_AMBT53_SOPM_1_1_1         0x20
#define AM_DEVICES_MSPI_AMBT53_SOPM_1_2_2         0x21
#define AM_DEVICES_MSPI_AMBT53_SOPM_1_4_4         0x22
#define AM_DEVICES_MSPI_AMBT53_SOPM_1_8_8         0x23
#define AM_DEVICES_MSPI_AMBT53_SOPM_2_2_2         0x24
#define AM_DEVICES_MSPI_AMBT53_SOPM_4_4_4         0x25
#define AM_DEVICES_MSPI_AMBT53_SOPM_8_8_8         0x26
#define AM_DEVICES_MSPI_AMBT53_SOPM_1_4_4_DQS     0x27
#define AM_DEVICES_MSPI_AMBT53_SOPM_1_8_8_DQS     0x28
#define AM_DEVICES_MSPI_AMBT53_SOPM_4_4_4_DQS     0x29
#define AM_DEVICES_MSPI_AMBT53_SOPM_8_8_8_DQS     0x2A
#define AM_DEVICES_MSPI_AMBT53_SET_DUMMY          0x40

#define AM_DEVICES_MSPI_AMBT53_MAX_PACKET_SIZE    0x4000000 //64M for UL

typedef union
{
    struct
    {
        uint32_t BUSY       : 1;
        uint32_t WRITE      : 1;
        uint32_t INV_CMD    : 1;
        uint32_t INV_LEN    : 1;
        uint32_t BUS_ERR    : 1;
        uint32_t UNDERRUN   : 1;
        uint32_t OVERRUN    : 1;
        uint32_t BURST_ERR  : 1;
        uint32_t DMODE      : 2;
        uint32_t AMODE      : 2;
        uint32_t CMODE      : 2;
        uint32_t RESERVED   : 1;
        uint32_t DQS_EN     : 1;
        uint32_t DUMMY      : 8;
        uint32_t SYNC       : 8;
    }STATUS_b;
    uint32_t STATUS;
}ambt53_status_reg_t;

/*
 * This 'Hello World' driver has a 'print' syscall that prints the
 * famous 'Hello World!' string.
 *
 * The string is formatted with some internal driver data to
 * demonstrate that drivers are initialized during the boot process.
 *
 * The driver exists to demonstrate (and test) custom drivers that are
 * maintained outside of Zephyr.
 */
struct ambt53_xip_driver_api {
	/* This struct has a member called 'print'. 'print' is function
	 * pointer to a function that takes 'struct device *dev' as an
	 * argument and returns 'void'.
	 */
	void (*print)(const struct device *dev);
};

// void        ambt53_xip_print(const struct device *dev);
void ambt53_status_get(const struct device *xip_dev, ambt53_status_reg_t *pStatus);
void ambt53_status_clear(const struct device *xip_dev, ambt53_status_reg_t *pStatus);
#ifdef __cplusplus
}
#endif

#endif /* __AMBT53_XIP_DRIVER_H__ */
