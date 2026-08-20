# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nemagfx_benchmarks)

target_include_directories(app PRIVATE
  src
  ${ZEPHYR_HAL_AMBIQ_MODULE_DIR}/components/graphics/NemaGFX_SDK/headers/include/tsi/common
)

target_sources(app PRIVATE
  src/main.c
  src/utils.c
  src/nema_utils_port.c
  src/bench_blit.c
  src/bench_draw_line.c
  src/bench_draw_rect.c
  src/bench_draw_string.c
  src/bench_fill_quad.c
  src/bench_fill_rect.c
  src/bench_fill_tri.c
  src/bench_stretch_blit.c
  src/bench_stretch_blit_rotate.c
  src/bench_textured_quad.c
  src/bench_textured_tri.c
  src/DejaVuSerif12pt8b.c
)
