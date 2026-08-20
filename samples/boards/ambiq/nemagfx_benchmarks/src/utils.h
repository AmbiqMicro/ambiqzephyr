/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __UTILS_H_
#define __UTILS_H_

#include "nema_hal.h"
#include "nema_utils.h"

#ifndef FB_RESX
#define FB_RESX 384
#endif

#ifndef FB_RESY
#define FB_RESY 384
#endif

#define TEST_MAX 36

#ifndef TIMEOUT_S
#define TIMEOUT_S 1.f
#endif

extern TLS_VAR img_obj_t g_sFB;

typedef enum {
	CPU_GPU = 0,
	CPU_BOUND = 1,
	GPU_BOUND = 2
} ExecutionMode_e;

int display_setup(void);

void suite_init(void);
void suite_terminate(void);

void bench_start(int32_t testno);
void bench_stop(int testno, int i32PixCount);

#endif /* __UTILS_H_ */
