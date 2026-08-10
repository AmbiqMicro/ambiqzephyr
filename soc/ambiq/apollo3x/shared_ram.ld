/*
 * Copyright (c) 2024 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Gate on the driver, not the devicetree. The address below comes from
 * MSPI_AMBIQ_BUFF_RAM_LOCATION, which lives inside "if MSPI_AMBIQ_CONTROLLER"
 * in drivers/mspi/Kconfig.ambiq. An enabled ambiq,mspi-controller node with the
 * driver left out leaves the symbol undefined, and the linker reads the bare
 * token as a forward reference.
 */
#if defined(CONFIG_MSPI_AMBIQ_CONTROLLER)
    SECTION_PROLOGUE (mspi_buff, CONFIG_SRAM_BASE_ADDRESS + CONFIG_MSPI_AMBIQ_BUFF_RAM_LOCATION (NOLOAD),)
    {
        __mspi_buff_start = .;
        KEEP(*(SORT_BY_NAME(".mspi_buff*")))
        . = ALIGN(CONFIG_MSPI_AMBIQ_BUFF_ALIGNMENT);
        __mspi_buff_end = .;
    } GROUP_LINK_IN(RAMABLE_REGION)
#endif
