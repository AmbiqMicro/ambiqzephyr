# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(amic)

function(add_prebuilt_library lib_name prebuilt_path)
  add_library(${lib_name} STATIC IMPORTED GLOBAL)
  set_target_properties(${lib_name} PROPERTIES
    IMPORTED_LOCATION ${APPLICATION_SOURCE_DIR}/${prebuilt_path}
  )
  zephyr_link_libraries(${lib_name})
endfunction()

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/am_vos_thf.c)
target_sources(app PRIVATE src/appAudio.c)

target_include_directories(app PRIVATE lib)

add_prebuilt_library(sensorylib  lib/sensorylib.a)
