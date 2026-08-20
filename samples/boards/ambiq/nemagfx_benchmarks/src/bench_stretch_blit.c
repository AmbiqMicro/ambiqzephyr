/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 * Copyright (c) 2019 Think Silicon S.A.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nema_core.h>

#include "bench.h"

#define PI 3.14159

static int SZX;
static int SZY;

static int i32RenderFrame(void)
{
	int x = 0, y = 0;

	if (FB_RESX - SZX > 0) {
		x = rand() % (FB_RESX - SZX);
	}
	if (FB_RESY - SZY > 0) {
		y = rand() % (FB_RESY - SZY);
	}

	nema_blit_rect_fit(x, y, SZX, SZY);

	return SZX * SZY;
}

/****************************************************************************** */
/* */
/*! @brief draw and stretch numerous company logos with selected blend mode */
/*! */
/*! @param i32BlendMode     - blend mode selection. */
/*! @param scale            - target zoom scale. */
/*! @param i32NemaTexMode   - GPU with NEMA_FILTER_PS or NEMA_FILTER_BL. */
/*! */
/*! This function could draw & stretch numerous logos with random positions. */
/*! */
/*! @return the total count of pixels. */
/* */
/****************************************************************************** */
int bench_stretch_blit(int i32BlendMode, float scale, int i32NemaTexMode)
{

	SZX = (int)(g_sAmbiqLogo.w * scale);
	SZY = (int)(g_sAmbiqLogo.h * scale);

	g_sContextCL = nema_cl_create();
	g_sCL0 = nema_cl_create();
	g_sCL1 = nema_cl_create();
	g_psCLCur = &g_sCL0;

	nema_cl_bind(&g_sContextCL);
	/* */
	/* Set Clipping Rectangle */
	/* */
	nema_set_clip(0, 0, FB_RESX, FB_RESY);
	/* */
	/* Bind Framebuffer */
	/* */
	nema_bind_dst_tex(g_sFB.bo.base_phys, g_sFB.w, g_sFB.h, (nema_tex_format_t)(g_sFB.format),
			  -1);
	/* */
	/* Set Blending Mode */
	/* */
	nema_bind_src_tex(g_sAmbiqLogo.bo.base_phys, g_sAmbiqLogo.w, g_sAmbiqLogo.h,
			  (nema_tex_format_t)(g_sAmbiqLogo.format), g_sAmbiqLogo.stride,
			  (nema_tex_mode_t)i32NemaTexMode);
	nema_cl_add_cmd(NEMA_BURST_SIZE, nema_burst_reg_value);
	nema_set_blend_blit(i32BlendMode);

	int i32PixCount = 0;

	i32PixCount += CL_CHECK_SUBMIT(0);

	nema_cl_destroy(&g_sContextCL);
	nema_cl_destroy(&g_sCL0);
	nema_cl_destroy(&g_sCL1);

	return i32PixCount;
}
