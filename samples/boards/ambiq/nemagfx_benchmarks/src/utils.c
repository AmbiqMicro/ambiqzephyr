/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "nemagfx_benchmarks.h"
#include "utils.h"
#include "Ambiq200x104.rgba.h"

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nemagfx_bench_utils, LOG_LEVEL_INF);

float start_wall;
float stop_wall;

TLS_VAR img_obj_t g_sAmbiqLogo = {{0}, 200, 104, -1, 0, NEMA_RGBA8888, 0};
TLS_VAR img_obj_t g_sFB = {{0}, (FB_RESX / 4) * 4, (FB_RESY / 4) * 4, -1, 0, NEMA_RGB24, 0};

static const struct device *display_dev;

static void print_test(int32_t i32TestNo)
{
	const char *pcTestsStr[TEST_MAX + 1] = {
		"_",
		"Fill_Triangle              ",
		"Fill_Triangle_Blend        ",
		"Fill_Rectangle             ",
		"Fill_Rectangle_Blend       ",
		"Fill_Quad                  ",
		"Fill_Quad_Blend            ",
		"Draw_String                ",
		"Draw_Line                  ",
		"Draw_Line_Blend            ",
		"Draw_Rectangle            ",
		"Draw_Rectangle_Blend      ",
		"Blit                      ",
		"Blit_Colorize             ",
		"Blit_Blend                ",
		"Blit_Blend_Colorize       ",
		"Blit_90                   ",
		"Blit_180                  ",
		"Blit_270                  ",
		"Blit_Vertical_Flip        ",
		"Blit_Horizontal_Flip      ",
		"Blit_SRC_Colorkeyed       ",
		"Blit_DST_Colorkeyed       ",
		"Stretch_Blit_PS           ",
		"Stretch_Blit_Blend_PS     ",
		"Stretch_Blit_BL           ",
		"Stretch_Blit_Blend_BL     ",
		"Stretch_Blit_Rotate       ",
		"Stretch_Blit_Rotate_BL    ",
		"Textured_Triangle_PS      ",
		"Textured_Triangle_Blend_PS",
		"Textured_Triangle_BL      ",
		"Textured_Triangle_Blend_BL",
		"Textured_Quad_PS          ",
		"Textured_Quad_Blend_PS    ",
		"Textured_Quad_BL          ",
		"Textured_Quad_Blend_BL    "
	};

	if (i32TestNo <= 0) {
		for (i32TestNo = 1; i32TestNo <= TEST_MAX; ++i32TestNo) {
			am_util_stdio_printf("%d    %s\r\n", i32TestNo, pcTestsStr[i32TestNo]);
		}
	} else if (i32TestNo > TEST_MAX) {
		am_util_stdio_printf("%d is not a valid test\n", i32TestNo);
	} else {
		am_util_stdio_printf("%d: %s    ", i32TestNo, pcTestsStr[i32TestNo]);
	}
}

static void clear_background(uint32_t col)
{
	static TLS_VAR nema_cmdlist_t cl;

	cl = nema_cl_create();
	nema_cl_bind(&cl);
	nema_bind_dst_tex(g_sFB.bo.base_phys, g_sFB.w, g_sFB.h,
			  (nema_tex_format_t)(g_sFB.format), g_sFB.stride);
	nema_set_clip(0, 0, FB_RESX, FB_RESY);
	nema_clear(col);
	nema_cl_unbind();
	nema_cl_submit(&cl);
	nema_cl_wait(&cl);
	nema_cl_destroy(&cl);
}

int display_setup(void)
{
	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (!device_is_ready(display_dev)) {
		LOG_ERR("Display device %s not ready", display_dev->name);
		return -ENODEV;
	}

	return display_blanking_off(display_dev);
}

void suite_init(void)
{
	g_sAmbiqLogo.bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS,
						  g_sAmbiqLogo.w * g_sAmbiqLogo.h * 4);
	if (g_sAmbiqLogo.bo.base_virt == NULL) {
		am_util_stdio_printf("Failed to create logo buffer!\n");
		k_panic();
	}
	nema_memcpy(g_sAmbiqLogo.bo.base_virt, ui8Ambiq200x104,
		    g_sAmbiqLogo.w * g_sAmbiqLogo.h * 4);
	nema_buffer_flush(&g_sAmbiqLogo.bo);

	g_sFB.bo = nema_buffer_create_pool(NEMA_MEM_POOL_FB, g_sFB.w * g_sFB.h * 4);
	if (g_sFB.bo.base_virt == NULL) {
		am_util_stdio_printf("Failed to create FB!\n");
		k_panic();
	}
	nema_buffer_map(&g_sFB.bo);
}

void suite_terminate(void)
{
	nema_buffer_destroy(&g_sFB.bo);
	nema_buffer_destroy(&g_sAmbiqLogo.bo);
}

void bench_start(int32_t i32TestNo)
{
	clear_background(0x0);
	print_test(i32TestNo);
}

static void bench_report(float perf, char *pcMesUnit)
{
	am_util_stdio_printf("%.2f %s\n", (double)perf, pcMesUnit);
}

void bench_stop(int i32TestNo, int i32PixCount)
{
	struct display_buffer_descriptor desc = {
		.buf_size = g_sFB.w * g_sFB.h * 3,
		.width = g_sFB.w,
		.height = g_sFB.h,
		.pitch = g_sFB.w,
		.frame_incomplete = false,
	};

	nema_buffer_flush(&g_sFB.bo);

	if (display_dev != NULL && device_is_ready(display_dev)) {
		display_write(display_dev, 0, 0, &desc, g_sFB.bo.base_virt);
	}

	float total_cpu_s = stop_wall - start_wall;

	if (i32TestNo == 7) {
		bench_report(i32PixCount / 1000.f / total_cpu_s, " KChars/sec");
	} else if (i32TestNo == 37 || i32TestNo == 38) {
		bench_report(i32PixCount / 1000.f / total_cpu_s, " KCLs/sec");
	} else {
		bench_report(i32PixCount / 1000.f / 1000.f / total_cpu_s, " MPixels/sec");
	}
}
