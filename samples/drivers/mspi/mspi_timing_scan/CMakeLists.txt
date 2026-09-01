# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mspi_timing_scan)

# MSPI is not usable on the Apollo330P and Apollo510L EVBs. Fail at configure
# time rather than produce an image that cannot work. The boards are excluded
# from this sample's twister scenarios as well; this catches a manual build.
if(CONFIG_BOARD_APOLLO330MP_EVB OR CONFIG_BOARD_APOLLO510DL_EVB)
  message(FATAL_ERROR
    "MSPI is not supported on ${BOARD}. The Apollo330P and Apollo510L EVBs are "
    "quarantined for MSPI; build this sample on another Ambiq board.")
endif()


FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

if(CONFIG_MSPI)
  target_include_directories(app PRIVATE ${ZEPHYR_BASE}/include/zephyr/drivers)
  target_include_directories(app PRIVATE ${ZEPHYR_BASE}/drivers/mspi)
endif()
