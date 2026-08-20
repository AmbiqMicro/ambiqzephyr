/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>

#include "nema_utils.h"

float nema_get_time(void)
{
	return (float)k_cycle_get_32() / (float)sys_clock_hw_cycles_per_sec();
}

float nema_get_wall_time(void)
{
	return nema_get_time();
}

void *nema_memcpy(void *destination, const void *source, size_t num)
{
	memcpy(destination, source, num);
	sys_cache_data_flush_range(destination, num);

	return destination;
}

void nema_calculate_fps(void)
{
}

unsigned int nema_rand(void)
{
	return (unsigned int)rand();
}

nema_buffer_t nema_load_file(const char *filename, int length, void *buffer)
{
	ARG_UNUSED(filename);
	ARG_UNUSED(length);
	ARG_UNUSED(buffer);

	nema_buffer_t bo = {0};

	return bo;
}

int nema_save_file(const char *filename, int length, void *buffer)
{
	ARG_UNUSED(filename);
	ARG_UNUSED(length);
	ARG_UNUSED(buffer);

	return 0;
}
