/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 * Copyright (c) 2019 Think Silicon S.A.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nema_core.h>
#include <nema_font.h>

#include "DejaVuSerif12pt8b.h"
#include "string.h"
#include "bench.h"

static const char str[] = "Think Silicon\nUltra-low power | vivid graphics";
static int w, h;

static int i32RenderFrame(void)
{
	int x = 0;
	int y = 0;
	uint32_t col = rand();

	if (w < FB_RESX) {
		x = rand() % (FB_RESX - w);
	}

	if (h < FB_RESY) {
		y = rand() % (FB_RESY - h);
	}

	nema_print(str, x, y, w, h, col, NEMA_ALIGNX_CENTER | NEMA_TEXT_WRAP | NEMA_ALIGNY_CENTER);

	return sizeof(str) - 1;
}

/****************************************************************************** */
/* */
/*! @brief draw strings */
/*! */
/*! @param blendmode - blend mode selection. */
/*! */
/*! This function load font to memory SSRAM,draw test string with selected blend */
/*! mode. */
/*! */
/*! @return the total count of pixels. */
/* */
/****************************************************************************** */
int bench_draw_string(int blendmode)
{
	g_sDejaVuSerif12pt8b.bo =
		nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, g_sDejaVuSerif12pt8b.bitmap_size);
	/*nema_buffer_map(&g_sDejaVuSerif12pt8b.bo); */
	nema_memcpy(g_sDejaVuSerif12pt8b.bo.base_virt, g_sDejaVuSerif12pt8b.bitmap,
		    g_sDejaVuSerif12pt8b.bitmap_size);
	nema_buffer_flush(&g_sDejaVuSerif12pt8b.bo);

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
	nema_set_blend_blit(blendmode);
	nema_bind_font(&g_sDejaVuSerif12pt8b);
	nema_string_get_bbox(str, &w, &h, FB_RESX, 1);

	int i32PixCount = 0;

	/*nema_bind_font(&g_sDejaVuSerif12pt8b); */
	i32PixCount += CL_CHECK_SUBMIT(ITEMS_PER_CL <= 5 ? 1 : ITEMS_PER_CL / 5);

	nema_cl_destroy(&g_sContextCL);
	nema_cl_destroy(&g_sCL0);
	nema_cl_destroy(&g_sCL1);

	nema_buffer_destroy(&g_sDejaVuSerif12pt8b.bo);

	return i32PixCount;
}
