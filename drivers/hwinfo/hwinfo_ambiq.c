/*
 * Copyright (c) 2024 Ambiq
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <am_mcu_apollo.h>
#include <zephyr/drivers/hwinfo.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

struct ambiq_hwinfo {
	uint32_t ui32ChipID0;
	uint32_t ui32ChipID1;
	uint32_t ui32TrimVer;
};

struct ambiq_hwinfo sAmbiqHwInfo = {0};

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	/* Contains the HAL hardware information about the device. */
	am_hal_mcuctrl_device_t sMcuCtrlDevice;

	am_hal_mram_info_read(1, AM_REG_INFO1_TRIM_REV_O / 4, 1, &sAmbiqHwInfo.ui32TrimVer);
	am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &sMcuCtrlDevice);

	sAmbiqHwInfo.ui32ChipID0 = sMcuCtrlDevice.ui32ChipID0;
	sAmbiqHwInfo.ui32ChipID1 = sMcuCtrlDevice.ui32ChipID1;

	if (length > sizeof(sAmbiqHwInfo)) {
		length = sizeof(sAmbiqHwInfo);
	}

	memcpy(buffer, &sAmbiqHwInfo, length);

	return length;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint32_t ui32Flags = 0;
	uint32_t ui32ResetStatus = 0;
	am_hal_reset_status_t sStatus = {0};

	/* Print out reset status register upon entry */
	am_hal_reset_status_get(&sStatus);
	ui32ResetStatus = sStatus.eStatus;

	/* EXTERNAL PIN */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_EXTERNAL) {
		ui32Flags |= RESET_PIN;
	}

	/* POWER CYCLE */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_POR) {
		ui32Flags |= RESET_POR;
	}

	/* BROWNOUT DETECTOR */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_BOD) {
		ui32Flags |= RESET_BROWNOUT;
	}

	/* SOFTWARE POR */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_SWPOR) {
		ui32Flags |= RESET_SOFTWARE;
	}

	/* SOFTWARE POI */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_SWPOI) {
		ui32Flags |= RESET_SOFTWARE;
	}

	/* DEBUGGER */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_DEBUGGER) {
		ui32Flags |= RESET_DEBUG;
	}

	/* WATCHDOG */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_WDT) {
		ui32Flags |= RESET_WATCHDOG;
	}

	/* BOUNREG */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_BOUNREG) {
		ui32Flags |= RESET_HARDWARE;
	}

	/* BOCORE */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_BOCORE) {
		ui32Flags |= RESET_HARDWARE;
	}

	/* BOMEM */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_BOMEM) {
		ui32Flags |= RESET_HARDWARE;
	}

	/* BOHPMEM */
	if (ui32ResetStatus & AM_HAL_RESET_STATUS_BOHPMEM) {
		ui32Flags |= RESET_HARDWARE;
	}

	*cause = ui32Flags;
	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	/* SBL maintains the RSTGEN->STAT register in
	 * INFO1 space even upon clearing RSTGEN->STAT
	 * register.
	 * - INFO1_RESETSTATUS
	 */

	return -ENOSYS;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = RESET_PIN
			| RESET_SOFTWARE
			| RESET_POR
			| RESET_WATCHDOG
			| RESET_HARDWARE
			| RESET_BROWNOUT;
	return 0;
}
