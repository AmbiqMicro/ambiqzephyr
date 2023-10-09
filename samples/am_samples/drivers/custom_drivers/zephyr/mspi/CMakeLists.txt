# SPDX-License-Identifier: Apache-2.0

if (CONFIG_CUSTOM_MSPI_DRIVER)
  # Add header file to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    costom_mspi_ambiq.c
    )
endif()
