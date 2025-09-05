# Copyright (c) 2025 Ambiq Micro Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOOTLOADER_MCUBOOT)
board_runner_args(jlink "--device=AP510NFA-CBR" "--iface=swd" "--speed=4000")
else()
board_runner_args(jlink "--device=AMAP54KK-KBR" "--iface=swd" "--speed=4000" "--erase" "--reset")
endif()

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
