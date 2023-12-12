# SPDX-License-Identifier: Apache-2.0

zephyr_sources(ipc_service.c ipc_ambiq.c)

add_subdirectory(backends)
add_subdirectory(lib)
