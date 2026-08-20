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

static float scale;

typedef struct _quad_t {
	float x0, y0;
	float x1, y1;
	float x2, y2;
	float x3, y3;
} Quad_t;

static void transform_quad(Quad_t *psQ, nema_matrix3x3_t sM)
{
	psQ->x0 = 0;
	psQ->y0 = 0;
	psQ->x1 = g_sAmbiqLogo.w;
	psQ->y1 = 0;
	psQ->x2 = g_sAmbiqLogo.w;
	psQ->y2 = g_sAmbiqLogo.h;
	psQ->x3 = 0;
	psQ->y3 = g_sAmbiqLogo.h;

	nema_mat3x3_mul_vec(sM, &psQ->x0, &psQ->y0);
	nema_mat3x3_mul_vec(sM, &psQ->x1, &psQ->y1);
	nema_mat3x3_mul_vec(sM, &psQ->x2, &psQ->y2);
	nema_mat3x3_mul_vec(sM, &psQ->x3, &psQ->y3);
}

static int i32RenderFrame(void)
{
	static int i32Rotation;
	Quad_t sQuad;
	nema_matrix3x3_t sM;

	nema_mat3x3_load_identity(sM);
	nema_mat3x3_scale(sM, scale, scale);
	nema_mat3x3_rotate(sM, --i32Rotation);
	nema_mat3x3_translate(sM, FB_RESX / 2, FB_RESY / 2);

	transform_quad(&sQuad, sM);

	nema_blit_quad_fit(sQuad.x0, sQuad.y0, sQuad.x1, sQuad.y1, sQuad.x2, sQuad.y2, sQuad.x3,
			   sQuad.y3);

	return SZX * SZY;
}

/****************************************************************************** */
/* */
/*! @brief stretch and rotate numerous company logos with selected blend mode */
/*! */
/*! @param i32BlendMode         - blend mode selection. */
/*! @param fscale               - target zoom scale. */
/*! @param i32NemaTexMode       - GPU with NEMA_FILTER_PS or NEMA_FILTER_BL. */
/*! */
/*! This function could rotate & stretch numerous logos with random positions. */
/*! */
/*! @return the total count of pixels. */
/* */
/****************************************************************************** */
int bench_stretch_blit_rotate(int i32BlendMode, float fscale, int i32NemaTexMode)
{

	SZX = (int)(g_sAmbiqLogo.w * fscale);
	SZY = (int)(g_sAmbiqLogo.h * fscale);

	scale = fscale;

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
