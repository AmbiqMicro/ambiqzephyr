/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mainentry, LOG_LEVEL_DBG);

int main(void)
{
    uint32_t i = 0;

    LOG_DBG("Backend_swo_example! %s\n", CONFIG_BOARD);
    while(1)
    {
        k_sleep(K_SECONDS(1));
        LOG_DBG("i: %d\n",i);
        ++i;
    }
	return 0;
}
