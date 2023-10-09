# SPDX-License-Identifier: Apache-2.0

if(CONFIG_AMBT53_XIP_DRIVER)
  # Add ambt53_xip_driver.h to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    ambt53_xip_driver.c
    )
endif()
