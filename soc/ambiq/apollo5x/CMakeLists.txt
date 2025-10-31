# Copyright (c) 2025 Ambiq Micro Inc.
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_sources(soc.c)
zephyr_include_directories(.)
zephyr_sources_ifdef(CONFIG_PM power.c)
zephyr_sources_ifdef(CONFIG_GPU_AMBIQ gpu.c)
zephyr_sources_ifdef(CONFIG_CPU_FREQ cpu_freq.c)
zephyr_include_directories(${ZEPHYR_BASE}/soc/arm/common/cortex_m)

zephyr_linker_sources(SECTIONS shared_ram.ld)

if(CONFIG_SOC_APOLLO510)
  zephyr_linker_sources(ITCM_SECTION hal_internal.ld)
endif()

if(CONFIG_SOC_APOLLO510L)
  if(CONFIG_COREMARK)
    zephyr_linker_sources(RAMFUNC_SECTION hal_internal.ld)
  else()
    zephyr_linker_sources(DTCM_SECTION hal_internal.ld)
  endif()
endif()

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

add_subdirectory_ifdef(CONFIG_SOC_AMBIQ_RSS_MGR rss_mgr)
