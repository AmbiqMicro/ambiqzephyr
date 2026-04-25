# Copyright (c) 2025 Renesas Electronics Corporation
# Copyright (c) 2025 Ambiq Micro, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/crc.h)

zephyr_library()

# zephyr-keep-sorted-start
zephyr_library_sources_ifdef(CONFIG_CRC_AMBIQ crc_ambiq.c)
zephyr_library_sources_ifdef(CONFIG_CRC_DRIVER_RENESAS_RA crc_renesas_ra.c)
# zephyr-keep-sorted-stop
