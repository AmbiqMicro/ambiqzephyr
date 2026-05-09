/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_crypto_power_domain

#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>

#include <soc.h>

LOG_MODULE_REGISTER(pd_ambiq_crypto, CONFIG_POWER_DOMAIN_LOG_LEVEL);

static int pd_crypto_pm_action(const struct device *dev, enum pm_device_action action)
{
	uint32_t status;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
		if (status != AM_HAL_STATUS_SUCCESS) {
			LOG_ERR("CRYPTO enable failed: 0x%x", status);
			return -EIO;
		}
		pm_device_children_action_run(dev, PM_DEVICE_ACTION_TURN_ON, NULL);
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		pm_device_children_action_run(dev, PM_DEVICE_ACTION_TURN_OFF, NULL);
		/*
		 * The CC312 engine requires its own dedicated power-down
		 * sequence rather than the generic periph_disable path.
		 */
		status = am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN, NULL);
		if (status != AM_HAL_STATUS_SUCCESS) {
			LOG_ERR("CRYPTO powerdown failed: 0x%x", status);
			return -EIO;
		}
		break;

	case PM_DEVICE_ACTION_TURN_ON:
	case PM_DEVICE_ACTION_TURN_OFF:
		/*
		 * Received from the parent OTP domain. The CRYPTO domain itself
		 * is still suspended; it will RESUME on demand via runtime PM.
		 */
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int pd_crypto_init(const struct device *dev)
{
	return pm_device_driver_init(dev, pd_crypto_pm_action);
}

#define AMBIQ_CRYPTO_PD_DEFINE(inst)                                                               \
	PM_DEVICE_DT_INST_DEFINE(inst, pd_crypto_pm_action);                                       \
	DEVICE_DT_INST_DEFINE(inst, pd_crypto_init, PM_DEVICE_DT_INST_GET(inst), NULL, NULL,       \
			      PRE_KERNEL_2, CONFIG_POWER_DOMAIN_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_CRYPTO_PD_DEFINE)
