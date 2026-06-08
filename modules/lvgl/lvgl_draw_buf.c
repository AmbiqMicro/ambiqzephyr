/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvgl_draw_buf.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/sys_heap.h>

#include <lvgl.h>
#include "draw/lv_draw_buf_private.h"

static char lvgl_draw_buf_heap_mem[CONFIG_LV_Z_DRAW_BUF_HEAP_SIZE]
	Z_GENERIC_SECTION(CONFIG_LV_Z_DRAW_BUF_ZEPHYR_REGION_NAME) __aligned(8);

static struct sys_heap lvgl_draw_buf_heap;
static struct k_spinlock lvgl_draw_buf_heap_lock;
static lv_draw_buf_handlers_t lvgl_draw_buf_handlers;

static void *lvgl_draw_buf_malloc(size_t size, lv_color_format_t color_format)
{
	k_spinlock_key_t key;
	void *ret;

	ARG_UNUSED(color_format);

	key = k_spin_lock(&lvgl_draw_buf_heap_lock);
	ret = sys_heap_alloc(&lvgl_draw_buf_heap, size);
	k_spin_unlock(&lvgl_draw_buf_heap_lock, key);

	return ret;
}

static void lvgl_draw_buf_free(void *buf)
{
	k_spinlock_key_t key;

	key = k_spin_lock(&lvgl_draw_buf_heap_lock);
	sys_heap_free(&lvgl_draw_buf_heap, buf);
	k_spin_unlock(&lvgl_draw_buf_heap_lock, key);
}

static void *lvgl_draw_buf_align(void *buf, lv_color_format_t color_format)
{
	uint8_t *buf_u8 = buf;

	ARG_UNUSED(color_format);

	if (buf_u8) {
		buf_u8 = (uint8_t *)LV_ROUND_UP((lv_uintptr_t)buf_u8, LV_DRAW_BUF_ALIGN);
	}

	return buf_u8;
}

static uint32_t lvgl_draw_buf_width_to_stride(uint32_t w, lv_color_format_t color_format)
{
	uint32_t width_byte;

	width_byte = w * lv_color_format_get_bpp(color_format);
	width_byte = (width_byte + 7) >> 3;

	return LV_ROUND_UP(width_byte, LV_DRAW_BUF_STRIDE_ALIGN);
}

void lvgl_draw_buf_init(void)
{
	sys_heap_init(&lvgl_draw_buf_heap, lvgl_draw_buf_heap_mem,
		      CONFIG_LV_Z_DRAW_BUF_HEAP_SIZE);

	lv_draw_buf_handlers_init(&lvgl_draw_buf_handlers, lvgl_draw_buf_malloc,
				  lvgl_draw_buf_free, lvgl_draw_buf_align, NULL, NULL,
				  lvgl_draw_buf_width_to_stride);
}

const lv_draw_buf_handlers_t *lvgl_draw_buf_get_handlers(void)
{
	return &lvgl_draw_buf_handlers;
}
