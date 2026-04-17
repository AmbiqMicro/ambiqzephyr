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

if(CONFIG_SOC_APOLLO510 OR CONFIG_SOC_APOLLO510B)
  zephyr_linker_sources(ITCM_SECTION hal_internal.ld)
endif()

if(CONFIG_SOC_APOLLO510L OR CONFIG_SOC_APOLLO330P)
  if(CONFIG_COREMARK)
    zephyr_linker_sources(RAMFUNC_SECTION hal_internal.ld)
  else()
    zephyr_linker_sources(DTCM_SECTION hal_internal.ld)
  endif()
  # Expose the composite part macro used by NemaDC/graphics HAL so that
  # headers guarded by AM_PART_APOLLO330P_510L are visible to drivers
  # when building for Apollo510L or Apollo330P.
  zephyr_compile_definitions(AM_PART_APOLLO330P_510L)
endif()

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

add_subdirectory_ifdef(CONFIG_SOC_AMBIQ_RSS_MGR rss_mgr)
