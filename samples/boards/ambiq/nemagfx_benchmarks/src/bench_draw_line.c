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
	int x1 = rand() % (FB_RESX);
	uint32_t col = rand();
	int y0 = rand() % (FB_RESY);
	int y1 = rand() % (FB_RESY);

	if (g_i32PremultiplyColor != 0) {
		col = nema_premultiply_rgba(col);
	}

	nema_draw_line(x0, y0, x1, y1, col);

	if (nema_abs(y0 - y1) > nema_abs(x0 - x1)) {
		return nema_abs(y0 - y1);
	} else {
		return nema_abs(x0 - x1);
	}
}

/****************************************************************************** */
/* */
/*! @brief draw lines */
/*! */
/*! @param i32Blend - blend mode selection. */
/*! */
/*! This function could draw numerous lines with selected blend mode. */
/*! */
/*! @return the total count of pixels. */
/* */
/****************************************************************************** */
int bench_draw_line(int i32Blend)
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
