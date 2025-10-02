# Copyright (c) 2025 Ambiq Micro Inc. <www.ambiq.com>
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=AMAP42KP-KBR" "--iface=swd" "--speed=1000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
