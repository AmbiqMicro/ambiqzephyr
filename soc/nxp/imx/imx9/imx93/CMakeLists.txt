# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_MIMX9352_A55)
  zephyr_include_directories(.)
  zephyr_include_directories(a55)

  zephyr_sources_ifdef(CONFIG_ARM_MMU a55/mmu_regions.c)

  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm64/scripts/linker.ld CACHE INTERNAL "")
elseif(CONFIG_SOC_MIMX9352_M33)
  zephyr_include_directories(.)
  zephyr_include_directories(m33)
  zephyr_sources(m33/soc.c)
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
  zephyr_library_sources_ifdef(CONFIG_SOC_RESET_HOOK m33/imx93_m33_startup.S)
endif()
