/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_LVGL_DRAW_BUF_H_
#define ZEPHYR_MODULES_LVGL_DRAW_BUF_H_

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the PSRAM (or other region) heap and draw-buffer handlers. */
void lvgl_draw_buf_init(void);

/** Handlers for lv_draw_buf_create_ex(); NULL if not configured. */
const lv_draw_buf_handlers_t *lvgl_draw_buf_get_handlers(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODULES_LVGL_DRAW_BUF_H_ */
