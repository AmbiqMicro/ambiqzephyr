/* mspi_ambiq.h - MSPI driver private definitions */

/*
 * Copyright (c) 2023 Ambiq, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MSPI_AMBIQ_H_
#define MSPI_AMBIQ_H_

#define MSPI_REG_BASEADDR   (0x50014000UL)

struct mspi_ambiq_timing_cfg {
    uint8_t     ui8WriteLatency;
    uint8_t     ui8TurnAround;
    bool        bTxNeg;
    bool        bRxNeg;
    bool        bRxCap;
    uint32_t    ui32TxDQSDelay;
    uint32_t    ui32RxDQSDelay;
    uint32_t    ui32RXDQSDelayEXT;
};

enum mspi_ambiq_timing_param {
    MSPI_AMBIQ_SET_WLC          = BIT(0),
    MSPI_AMBIQ_SET_RLC          = BIT(1),
    MSPI_AMBIQ_SET_TXNEG        = BIT(2),
    MSPI_AMBIQ_SET_RXNEG        = BIT(3),
    MSPI_AMBIQ_SET_RXCAP        = BIT(4),
    MSPI_AMBIQ_SET_TXDQSDLY     = BIT(5),
    MSPI_AMBIQ_SET_RXDQSDLY     = BIT(6),
    MSPI_AMBIQ_SET_RXDQSDLYEXT  = BIT(7),
};

#endif