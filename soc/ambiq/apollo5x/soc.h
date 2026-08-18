/*
 * Copyright (c) 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOC_H__
#define __SOC_H__

#include <am_mcu_apollo.h>

int apollo5x_set_performance_mode(uint32_t mode);

bool buf_in_nocache(uintptr_t buf, size_t len_bytes);

#if defined(CONFIG_SOC_AMBIQ_APOLLO5X_BLE_LP) && defined(CONFIG_SOC_APOLLO510B)
/*
 * fit_lp-style peripheral gating after bt_enable() (see peripheral_hr main.c).
 * Do not use DIS_PERIPHS_ALL here — IOM6 is already active via the Zephyr SPI
 * driver; fit_lp runs DIS_PERIPHS before radio init, which is not our boot order.
 */
void ambiq_apollo510b_ble_lp_runtime_init(void);
#endif

/* Return true if the buffer intersects the DTCM address range. */
static inline bool ambiq_buf_in_dtcm(uintptr_t buf, size_t len_bytes)
{
	if (buf == 0 || len_bytes == 0) {
		return false;
	}
	return ((buf <= (DTCM_BASEADDR + DTCM_MAX_SIZE - 1)) &&
		((buf + len_bytes - 1) >= DTCM_BASEADDR));
}

#endif /* __SOC_H__ */
