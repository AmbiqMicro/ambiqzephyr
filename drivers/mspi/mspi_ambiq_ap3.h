/* mspi_ambiq_ap3.h - MSPI driver private definitions */

/*
 * Copyright (c) 2023 Ambiq, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MSPI_AMBIQ_AP3_H_
#define MSPI_AMBIQ_AP3_H_

#define MSPI_REG_BASEADDR   (0x50014000UL)

struct mspi_ambiq_ap3_timing_cfg {
    bool        bSendInstr;
    bool        bSendAddr;
    bool        bWriteLatency;
    bool        bTurnaround;
    uint8_t     ui8WriteLatency;
    uint8_t     ui8TurnAround;
};

#endif