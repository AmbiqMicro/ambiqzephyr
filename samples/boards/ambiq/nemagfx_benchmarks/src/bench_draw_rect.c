/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 * Copyright (c) 2019 Think Silicon S.A.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nema_core.h>

#include "bench.h"

#define min2(a, b) ((a) < (b) ? (a) : (b))

static int g_i32PremultiplyColor;

static int i32RenderFrame(void)
{
	int size = rand() % (min2(FB_RESX, FB_RESY) / 2);

	int x = rand() % (FB_RESX - size);
	uint32_t col = rand();
	int y = rand() % (FB_RESY - size);

	if (g_i32PremultiplyColor != 0) {
		col = nema_premultiply_rgba(col);
	}

	nema_draw_rect(x, y, size, size, col);

	return 4 * size - 4;
}

/****************************************************************************** */
/* */
/*! @brief draw rectangle */
/*! */
/*! @param i32Blend - blend mode selection. */
/*! */
/*! This function could draw numerous lines with selected blend mode. */
/*! */
/*! @return the total count of pixels. */
/* */
/****************************************************************************** */
int bench_draw_rect(int i32Blend)
{
	g_sContextCL = nema_cl_create();
	g_sCL0 = nema_cl_create();
	g_sCL1 = nema_cl_create();
	g_psCLCur = &g_sCL0;

	nema_cl_bind(&g_sContextCL);
	/* */
	/* Bind Framebuffer */
	/* */
	nema_bind_dst_tex(g_sFB.bo.base_phys, g_sFB.w, g_sFB.h, (nema_tex_format_t)(g_sFB.format),
			  -1);
	nema_cl_add_cmd(NEMA_BURST_SIZE, nema_burst_reg_value);
	/* */
	/* Set Clipping Rectangle */
	/* */
	nema_set_clip(0, 0, FB_RESX, FB_RESY);
	/* */
	/* Set Blending Mode */
	/* */
	nema_set_blend_fill(i32Blend ? NEMA_BL_SRC_OVER : NEMA_BL_SRC);
	g_i32PremultiplyColor = i32Blend;

	int i32PixCount = 0;

	i32PixCount += CL_CHECK_SUBMIT(0);

	nema_cl_destroy(&g_sContextCL);
	nema_cl_destroy(&g_sCL0);
	nema_cl_destroy(&g_sCL1);

	return i32PixCount;
}
