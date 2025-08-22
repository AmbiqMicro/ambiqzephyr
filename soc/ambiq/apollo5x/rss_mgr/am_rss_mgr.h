/*
 * Copyright (c) 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Ambiq radio subsystem manager functions. The radio subsystem is running
 * at another network processing core, which is only included in Apollo510L series
 * SoC now.
 */

#ifndef AM_RSS_MGR_H__
#define AM_RSS_MGR_H__

#include <zephyr/net_buf.h>

/** @brief Radio Subsystem Operational Modes */
typedef enum
{
    /* CM4-only mode - controller/processor on */
    AM_RSS_OPMODE_CP = 0,
    /* Network processor mode - CM4 and Radio are on */
    AM_RSS_OPMODE_NP,
    /* Network processor mode - CM4 Radio and XO are on */
    AM_RSS_OPMODE_NP_XO,
    /* Crystal-only mode - CM4 is on (active or retention) */
    AM_RSS_OPMODE_XO_CP_ON,
    /* Crystal-only mode - CM4 is off */
    AM_RSS_OPMODE_XO_CP_OFF,
    /* Subsystem is off */
    AM_RSS_OPMODE_OFF,
    /* Invalid RSS mode */
    AM_RSS_OPMODE_INVALID
} am_rss_opmode_e;

/*
 * The system control packets are transmitted via IPC message and the types are
 * Ambiq specific, their general data format is defined as:
 * ---------------------------------------------------------------------------
 * 1-byte Packet Type | 1-byte Opcode | 2-byte Data Size | N-byte Data Payload
 * ---------------------------------------------------------------------------
 */
#define AM_IPC_PACKET_TYPE_SYS_REQ 0xe0 /* System Request packet */
#define AM_IPC_PACKET_TYPE_SYS_RSP 0xe1 /* System Response packet*/
#define AM_IPC_PACKET_TYPE_SYS_NTF 0xe2 /* System Data/Status Notification packet */

/**
 * @brief Ambiq IPC System Packet Common Header structure
 */
struct am_ipc_packet_sys_common_hdr
{
    uint8_t type;   /* System Packet type */
    uint8_t opcode; /* System Packet Opcode */
    uint16_t size;  /* System Packet data payload length */
} __packed;

/**
 * @brief Ambiq IPC System Packet Opcode definition
 */
typedef enum
{
    AM_IPC_SYS_OP_SET_RSS_OPMODE = 0x00,
    AM_IPC_SYS_OP_READ_REG = 0x01,
    AM_IPC_SYS_OP_WRITE_REG = 0x02,
    AM_IPC_SYS_OP_READ_MEM = 0x03,
    AM_IPC_SYS_OP_WRITE_MEM = 0x04,
    AM_IPC_SYS_OP_SET_RFTRIM = 0x05,
} am_ipc_sys_opcode_e;

/**
 * @brief Ambiq IPC System Set Radio Subsystem Opmoode Request structure
 */
struct am_ipc_sys_set_rss_opmode_req_t
{
    struct am_ipc_packet_sys_common_hdr hdr;
    uint8_t mode;
} __packed;

/**
 * @brief Ambiq IPC System Set Radio Subsystem Opmoode Response structure
 */
struct am_ipc_sys_set_rss_opmode_rsp_t
{
    struct am_ipc_packet_sys_common_hdr hdr;
    uint8_t status;
} __packed;

/**
 * @brief Configure the IPC share memory information for the apollo510L cores
 * communication.
 *
 * The IPC share memory information is defined in the Apollo510L application core
 * and configured to the radio core via vendor mailbox command.
 *
 * @return 0 on success, or a negative error code on failure.
 */
int am_rss_mgr_ipc_shm_config(void);

/**
 * @brief Configure the radio subsystem operational mode via IPC message.
 *
 * The IPC share memory information is defined in the Apollo510L application core
 * and configured to the radio core via vendor mailbox command.
 * 
 * @param opmode request radio subsystem operational mode
 *
 * @return net_buf pointer of buffer that needs to send via IPC.
 */
struct net_buf *am_rss_mgr_opmode_config(am_rss_opmode_e opmode);

/**
 * @brief Enable/disable the radio subsystem of Apollo SoC.
 *
 * This function can be used to power on or off the radio subsystem.
 * 
 * @param on true indicates to power on the radio subsystem, otherwise power off
 *
 * @return 0 on success, or a negative error code on failure.
 */
int am_rss_mgr_rss_enable(bool on);

#endif /* AM_RSS_MGR_H__ */
