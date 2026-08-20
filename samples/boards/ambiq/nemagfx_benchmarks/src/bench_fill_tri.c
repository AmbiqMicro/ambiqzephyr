/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 * Copyright (c) 2019 Think Silicon S.A.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nema_core.h>

#include "bench.h"

static int g_i32PremultiplyColor;

static int i32RenderFrame(void)
{
	int x0 = rand() % (FB_RESX);
	int y2 = rand() % (FB_RESY);
	int x1 = rand() % (FB_RESX);
	uint32_t ui32Col = rand();
	int y0 = rand() % (FB_RESY);
	int x2 = rand() % (FB_RESX);
	int y1 = rand() % (FB_RESY);

	if (g_i32PremultiplyColor != 0) {
		ui32Col = nema_premultiply_rgba(ui32Col);
	}
	nema_fill_triangle(x0, y0, x1, y1, x2, y2, ui32Col);

	int i32Area = (int)(0.5f * nema_abs(x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1)));

	return i32Area;
}

/****************************************************************************** */
/* */
/*! @brief fill patterns with triangles */
/*! */
/*! @param i32Blend - blend mode selection. */
/*! */
/*! This function fill large number of triangles with blend mode NEMA_BL_SRC_OVER */
/*! or NEMA_BL_SRC */
/*! */
/*! @return the total count of pixels. */
/* */
/****************************************************************************** */
int bench_fill_tri(int i32Blend)
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
	/*----------------------------------------------------------------------- */
	g_i32PremultiplyColor = i32Blend;

	int i32PixCount = 0;

	i32PixCount += CL_CHECK_SUBMIT(0);

	nema_cl_destroy(&g_sContextCL);
	nema_cl_destroy(&g_sCL0);
	nema_cl_destroy(&g_sCL1);

	return i32PixCount;
}
