/*
 * Copyright (c) 2019 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include "ambt53_xip_driver.h"
#include <stdio.h>
#include <zephyr/kernel.h>


const struct device *dev;

static void user_entry(void *p1, void *p2, void *p3)
{
	// hello_world_print(dev);
}

int main(void)
{
	printk("Hello World from the app!\n");

	dev = DEVICE_DT_GET(DT_NODELABEL(ambt53));

	__ASSERT(dev, "Failed to get device binding");

	printk("device is %p, name is %s\n", dev, dev->name);
	
	ambt53_basic_config_get(dev);

	k_object_access_grant(dev, k_current_get());
	k_thread_user_mode_enter(user_entry, NULL, NULL, NULL);
	return 0;
}
