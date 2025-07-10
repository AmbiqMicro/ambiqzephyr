# Copyright (c) 2025 Ambiq Micro Inc.
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_sources(soc.c)
zephyr_include_directories(.)
zephyr_sources_ifdef(CONFIG_PM power.c)
zephyr_include_directories(${ZEPHYR_BASE}/soc/arm/common/cortex_m)

zephyr_linker_sources(SECTIONS shared_ram.ld)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

if(CONFIG_SOC_HAS_TIMING_FUNCTIONS AND NOT CONFIG_BOARD_HAS_TIMING_FUNCTIONS)
  if(CONFIG_TIMING_FUNCTIONS)
    if(NOT CONFIG_CORTEX_M_DWT)
      zephyr_library_sources(timing.c)
    endif()
  endif()
endif()
