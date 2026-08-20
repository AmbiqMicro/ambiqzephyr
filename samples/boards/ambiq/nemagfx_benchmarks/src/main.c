/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief NemaGFX performance benchmarking suite for Zephyr.
 *
 * Port of the AmbiqSuite nemagfx_benchmarks example. Measures GPU throughput
 * for shape rendering, texture operations, and transformations, reporting
 * results on the console and pushing each result frame to the display.
 */

#include "nemagfx_benchmarks.h"
#include "bench.h"
#include "utils.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nemagfx_benchmarks, LOG_LEVEL_INF);

#ifndef DEFAULT_EXEC_MODE
#define DEFAULT_EXEC_MODE CPU_GPU
#endif

TLS_VAR nema_cmdlist_t *g_psCLCur, g_sCL0, g_sCL1, g_sContextCL;
ExecutionMode_e eExecMode = DEFAULT_EXEC_MODE;

typedef enum {
	GPU_BURST_LENGTH_16 = 4,
	GPU_BURST_LENGTH_32 = 5,
	GPU_BURST_LENGTH_64 = 6,
	GPU_BURST_LENGTH_128 = 7,
} nemagfx_burst_length_t;

static nemagfx_burst_length_t tex_burst_length = GPU_BURST_LENGTH_16;
static nemagfx_burst_length_t fb_burst_length = GPU_BURST_LENGTH_16;

uint32_t nema_burst_reg_value;

static int i32RenderFrame(void)
{
	return 0;
}

void run_bench(int32_t i32TestNo)
{
	int32_t i32Result = 0;

	suite_init();

	switch (i32TestNo) {
	case 1:
		bench_start(i32TestNo);
		i32Result = bench_fill_tri(0);
		bench_stop(i32TestNo, i32Result);
		break;
	case 2:
		bench_start(i32TestNo);
		i32Result = bench_fill_tri(1);
		bench_stop(i32TestNo, i32Result);
		break;
	case 3:
		bench_start(i32TestNo);
		i32Result = bench_fill_rect(0);
		bench_stop(i32TestNo, i32Result);
		break;
	case 4:
		bench_start(i32TestNo);
		i32Result = bench_fill_rect(1);
		bench_stop(i32TestNo, i32Result);
		break;
	case 5:
		bench_start(i32TestNo);
		i32Result = bench_fill_quad(0);
		bench_stop(i32TestNo, i32Result);
		break;
	case 6:
		bench_start(i32TestNo);
		i32Result = bench_fill_quad(1);
		bench_stop(i32TestNo, i32Result);
		break;
	case 7:
		bench_start(i32TestNo);
		i32Result = bench_draw_string(NEMA_BL_SRC);
		bench_stop(i32TestNo, i32Result);
		break;
	case 8:
		bench_start(i32TestNo);
		i32Result = bench_draw_line(0);
		bench_stop(i32TestNo, i32Result);
		break;
	case 9:
		bench_start(i32TestNo);
		i32Result = bench_draw_line(1);
		bench_stop(i32TestNo, i32Result);
		break;
	case 10:
		bench_start(i32TestNo);
		i32Result = bench_draw_rect(0);
		bench_stop(i32TestNo, i32Result);
		break;
	case 11:
		bench_start(i32TestNo);
		i32Result = bench_draw_rect(1);
		bench_stop(i32TestNo, i32Result);
		break;
	case 12:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SRC, NEMA_ROT_000_CCW);
		bench_stop(i32TestNo, i32Result);
		break;
	case 13:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SRC | NEMA_BLOP_MODULATE_RGB, NEMA_ROT_000_CCW);
		bench_stop(i32TestNo, i32Result);
		break;
	case 14:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SIMPLE, NEMA_ROT_000_CCW);
		bench_stop(i32TestNo, i32Result);
		break;
	case 15:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SIMPLE | NEMA_BLOP_MODULATE_RGB, NEMA_ROT_000_CCW);
		bench_stop(i32TestNo, i32Result);
		break;
	case 16:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SRC, NEMA_ROT_090_CW);
		bench_stop(i32TestNo, i32Result);
		break;
	case 17:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SRC, NEMA_ROT_180_CW);
		bench_stop(i32TestNo, i32Result);
		break;
	case 18:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SRC, NEMA_ROT_270_CW);
		bench_stop(i32TestNo, i32Result);
		break;
	case 19:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SRC, NEMA_MIR_VERT);
		bench_stop(i32TestNo, i32Result);
		break;
	case 20:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SRC, NEMA_MIR_HOR);
		bench_stop(i32TestNo, i32Result);
		break;
	case 21:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SRC | NEMA_BLOP_SRC_CKEY, NEMA_ROT_000_CCW);
		bench_stop(i32TestNo, i32Result);
		break;
	case 22:
		bench_start(i32TestNo);
		i32Result = bench_blit(NEMA_BL_SRC | NEMA_BLOP_DST_CKEY, NEMA_ROT_000_CCW);
		bench_stop(i32TestNo, i32Result);
		break;
	case 23:
		bench_start(i32TestNo);
		i32Result = bench_stretch_blit(NEMA_BL_SRC, 1.5, NEMA_FILTER_PS);
		bench_stop(i32TestNo, i32Result);
		break;
	case 24:
		bench_start(i32TestNo);
		i32Result = bench_stretch_blit(NEMA_BL_SIMPLE, 1.5, NEMA_FILTER_PS);
		bench_stop(i32TestNo, i32Result);
		break;
	case 25:
		bench_start(i32TestNo);
		i32Result = bench_stretch_blit(NEMA_BL_SRC, 1.5, NEMA_FILTER_BL);
		bench_stop(i32TestNo, i32Result);
		break;
	case 26:
		bench_start(i32TestNo);
		i32Result = bench_stretch_blit(NEMA_BL_SIMPLE, 1.5, NEMA_FILTER_BL);
		bench_stop(i32TestNo, i32Result);
		break;
	case 27:
		bench_start(i32TestNo);
		i32Result = bench_stretch_blit_rotate(NEMA_BL_SRC, 0.75, NEMA_FILTER_PS);
		bench_stop(i32TestNo, i32Result);
		break;
	case 28:
		bench_start(i32TestNo);
		i32Result = bench_stretch_blit_rotate(NEMA_BL_SRC, 0.75, NEMA_FILTER_BL);
		bench_stop(i32TestNo, i32Result);
		break;
	case 29:
		bench_start(i32TestNo);
		i32Result = bench_textured_tri(NEMA_BL_SRC, NEMA_FILTER_PS);
		bench_stop(i32TestNo, i32Result);
		break;
	case 30:
		bench_start(i32TestNo);
		i32Result = bench_textured_tri(NEMA_BL_SIMPLE, NEMA_FILTER_PS);
		bench_stop(i32TestNo, i32Result);
		break;
	case 31:
		bench_start(i32TestNo);
		i32Result = bench_textured_tri(NEMA_BL_SRC, NEMA_FILTER_BL);
		bench_stop(i32TestNo, i32Result);
		break;
	case 32:
		bench_start(i32TestNo);
		i32Result = bench_textured_tri(NEMA_BL_SIMPLE, NEMA_FILTER_BL);
		bench_stop(i32TestNo, i32Result);
		break;
	case 33:
		bench_start(i32TestNo);
		i32Result = bench_textured_quad(NEMA_BL_SRC, NEMA_FILTER_PS);
		bench_stop(i32TestNo, i32Result);
		break;
	case 34:
		bench_start(i32TestNo);
		i32Result = bench_textured_quad(NEMA_BL_SIMPLE, NEMA_FILTER_PS);
		bench_stop(i32TestNo, i32Result);
		break;
	case 35:
		bench_start(i32TestNo);
		i32Result = bench_textured_quad(NEMA_BL_SRC, NEMA_FILTER_BL);
		bench_stop(i32TestNo, i32Result);
		break;
	case 36:
		bench_start(i32TestNo);
		i32Result = bench_textured_quad(NEMA_BL_SIMPLE, NEMA_FILTER_BL);
		bench_stop(i32TestNo, i32Result);
		break;
	default:
		return;
	}

	suite_terminate();
}

static void run_benchmark_pass(void)
{
	srand(0xffffff00);

	nema_burst_reg_value = 0x0UL | (fb_burst_length << 4) | (tex_burst_length);
	am_util_stdio_printf("FB burst: %d\n", fb_burst_length);
	am_util_stdio_printf("TEX burst: %d\n", tex_burst_length);
	am_util_stdio_printf("Burst size register value: %08X\n", nema_burst_reg_value);

	for (uint32_t i32Test = 1; i32Test <= TEST_MAX; ++i32Test) {
		run_bench(i32Test);
	}
}

int main(void)
{
	int ret;

	LOG_INF("NemaGFX benchmarks starting");

	ret = nema_init();
	if (ret != 0) {
		LOG_ERR("nema_init failed: %d", ret);
		return ret;
	}

	ret = display_setup();
	if (ret != 0) {
		LOG_ERR("Display setup failed: %d", ret);
		return ret;
	}

	am_util_stdio_printf("\nNemaGFX benchmark suite (FB %dx%d)\n", FB_RESX, FB_RESY);

#if defined(CONFIG_TEST)
	run_benchmark_pass();
	am_util_stdio_printf("NemaGFX benchmarks complete\n");
	return 0;
#else
	while (1) {
		run_benchmark_pass();

		tex_burst_length++;
		if (tex_burst_length > GPU_BURST_LENGTH_128) {
			tex_burst_length = GPU_BURST_LENGTH_16;
			fb_burst_length++;
			if (fb_burst_length > GPU_BURST_LENGTH_128) {
				fb_burst_length = GPU_BURST_LENGTH_16;
			}
		}
	}
#endif
}
