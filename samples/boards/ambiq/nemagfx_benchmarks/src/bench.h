/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BENCH_H__
#define __BENCH_H__

#include "nema_hal.h"
#include "nema_cmdlist.h"
#include "nema_utils.h"
#include "nema_regs.h"

#include "utils.h"
#include <stdlib.h>

#ifndef ITEMS_PER_CL
#define ITEMS_PER_CL 90
#endif

extern ExecutionMode_e eExecMode;
extern TLS_VAR img_obj_t g_sAmbiqLogo;
extern TLS_VAR nema_cmdlist_t *g_psCLCur, g_sCL0, g_sCL1, g_sContextCL;

extern float start_wall, stop_wall;

extern uint32_t nema_burst_reg_value;

static int i32RenderFrame(void);

int bench_fill_tri(int i32Blend);
int bench_fill_rect(int i32Blend);
int bench_fill_quad(int i32Blend);
int bench_draw_string(int blendmode);
int bench_draw_line(int i32Blend);
int bench_draw_rect(int i32Blend);
int bench_blit(int i32BlendMode, int i32Rotation);
int bench_stretch_blit(int i32BlendMode, float scale, int i32NemaTexMode);
int bench_stretch_blit_rotate(int i32BlendMode, float fscale, int i32NemaTexMode);
int bench_textured_tri(int i32BlendMode, int i32NemaTexMode);
int bench_textured_quad(int i32BlendMode, int i32NemaTexMode);

#define PRINTF(...)

#define swap_cmd_lists() \
	(g_psCLCur = (g_psCLCur == &g_sCL0) ? &g_sCL1 : &g_sCL0)

static inline int CL_CHECK_SUBMIT(int i32ItemsPerCL)
{
	int _items_per_cl = i32ItemsPerCL;

	if (_items_per_cl == 0) {
		_items_per_cl = ITEMS_PER_CL;
	}

	int items = 0;
	int i32PixCount = 0;

	start_wall = nema_get_time();
	stop_wall = start_wall;

	nema_cl_bind(g_psCLCur);
	nema_cl_rewind(g_psCLCur);
	nema_cl_branch(&g_sContextCL);

	if (eExecMode != GPU_BOUND) {
		do {
			i32PixCount += i32RenderFrame();

			++items;
			if ((items % _items_per_cl) == 0) {
				if (eExecMode != CPU_BOUND) {
					nema_cl_return();
					nema_cl_submit(g_psCLCur);
				}

				swap_cmd_lists();
				nema_cl_wait(g_psCLCur);

				nema_cl_bind(g_psCLCur);
				nema_cl_rewind(g_psCLCur);
				nema_cl_branch(&g_sContextCL);

				stop_wall = nema_get_time();
				if ((stop_wall - start_wall) > TIMEOUT_S) {
					break;
				}
			}
		} while (1);

		nema_cl_wait(g_psCLCur);
		swap_cmd_lists();
		nema_cl_wait(g_psCLCur);
	} else {
		int i;
		int pix_count_cl = 0;

		for (i = 0; i < _items_per_cl; i++) {
			pix_count_cl += i32RenderFrame();
		}
		while (1) {
			nema_cl_return();
			nema_cl_submit(g_psCLCur);
			nema_cl_wait(g_psCLCur);
			items += _items_per_cl;
			i32PixCount += pix_count_cl;
			stop_wall = nema_get_time();
			if ((stop_wall - start_wall) > TIMEOUT_S) {
				break;
			}
		}
	}

	stop_wall = nema_get_time();

	return i32PixCount;
}

#endif /* __BENCH_H__ */
