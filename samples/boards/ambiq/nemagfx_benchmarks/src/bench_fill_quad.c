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
	int i32BBoxW = rand() % (FB_RESX - 50) + 50;
	int i32BBoxH = rand() % (FB_RESY - 50) + 50;
	uint32_t ui32Col = rand();
	int i32BBoxX = rand() % (FB_RESX - i32BBoxW);
	int i32BBoxY = rand() % (FB_RESY - i32BBoxH);

	int x0 = (int)(i32BBoxX + (float)(rand() % (100)) / 100.f * i32BBoxW);
	int y0 = i32BBoxY;
	int x1 = i32BBoxX + i32BBoxW;
	int y1 = (int)(i32BBoxY + (float)(rand() % (100)) / 100.f * i32BBoxH);
	int x2 = (int)(i32BBoxX + (float)(rand() % (100)) / 100.f * i32BBoxW);
	int y2 = i32BBoxY + i32BBoxH;
	int x3 = i32BBoxX;
	int y3 = (int)(i32BBoxY + (float)(rand() % (100)) / 100.f * i32BBoxH);

	if (g_i32PremultiplyColor != 0) {
		ui32Col = nema_premultiply_rgba(ui32Col);
	}

	nema_fill_quad(x0, y0, x1, y1, x2, y2, x3, y3, ui32Col);

	int area = (int)(0.5f * nema_abs(x0 * (y1 - y3) + x1 * (y2 - y0) + x2 * (y3 - y1) +
					 x3 * (y0 - y2)));
	return area;
}

/****************************************************************************** */
/* */
/*! @brief fill patterns with quadrilaterals */
/*! */
/*! @param i32Blend - blend mode selection. */
/*! */
/*! This function fill large number of quadrilaterals with blend mode NEMA_BL_SRC_OVER */
/*! or NEMA_BL_SRC */
/*! */
/*! @return the total count of pixels. */
/* */
/****************************************************************************** */
int bench_fill_quad(int i32Blend)
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
