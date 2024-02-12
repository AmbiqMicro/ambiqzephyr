/*
 * Copyright (c) 2024 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include "soc.h"

static int arm_apollo5_init(void)
{

#if (defined (CONFIG_CPU_HAS_FPU) && defined (CONFIG_FPU)) || \
    (defined (CONFIG_ARMV8_1_M_MVEI) || defined (CONFIG_ARMV8_1_M_MVEF))
    SCB->CPACR |= ((3U << 10U*2U) |           /* enable CP10 Full Access */
                    (3U << 11U*2U)  );         /* enable CP11 Full Access */

    /* Set low-power state for PDEPU                */
    /*  0b00  | ON, PDEPU is not in low-power state */
    /*  0b01  | ON, but the clock is off            */
    /*  0b10  | RET(ention)                         */
    /*  0b11  | OFF                                 */

    /* Clear ELPSTATE, value is 0b11 on Cold reset */
    PWRMODCTL->CPDLPSTATE &= ~(PWRMODCTL_CPDLPSTATE_ELPSTATE_Msk);

    /* Favor best FP/MVE performance by default, avoid EPU switch-ON delays */
    /* PDEPU ON, Clock OFF */
    PWRMODCTL->CPDLPSTATE |= 0x1 << PWRMODCTL_CPDLPSTATE_ELPSTATE_Pos;
#endif

    /* Enable Loop and branch info cache */
    SCB->CCR |= SCB_CCR_LOB_Msk;
    __DSB();
    __ISB();

    /* Initialize for low power in the power control block */
    am_hal_pwrctrl_low_power_init();

    /* Enable SIMOBUCK for the apollo5 Family */
    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_SIMOBUCK_INIT, NULL);

    return 0;
}

SYS_INIT(arm_apollo5_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
