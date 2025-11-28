============================================
© 2025 Ambiq Micro Inc. All rights reserved.
============================================

Release Notes
=============

Overview
--------

This release introduces new features, bug fixes, and performance improvements.

Release Information
===================

- **Version:** v0.3
- **Release Date:** 2025-11-28

New Features
------------

Boards & SoC Support
~~~~~~~~~~~~~~~~~~~~

- **apollo5_eb_display_8080_card**: Added support for apollo5_eb_display_8080_card.

Drivers and Sensors
~~~~~~~~~~~~~~~~~~~

- **mipi_dbi**: Added support for mipi dbi driver.
- **sdhc**: Added multiple mmcs, mmc bandwidth and sdio card tests.

Improvements
------------

Build system and Infrastructure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **arch**: Upgraded code base to zephyr v4.3.0 release.

Drivers and Sensors
~~~~~~~~~~~~~~~~~~~

- **adc**: Enhanced adc context handling and interrupt clearing.
- **mipi_dsi**: Added PM runtime device support for mipi dsi driver.

HAL
~~~

- **apollo510L**: Upgraded HAL to SDK5.2.alpha.2.
- **NemaGFX**: Updated nema_dc_hal to enhance compatibility for other ambiq platforms.

Libraries / Subsystems
~~~~~~~~~~~~~~~~~~~~~~

- **power management**:
  - Enabled ``CONFIG_PM_DEVICE_RUNTIME_DEFAULT_ENABLE`` to enable PM device runtime globally.
  - Added ``CONFIG_PM_DEVICE_RUNTIME_DISABLE_CONSOLE`` to disable runtime PM for the console UART
    by default to prevent CI issues with ISR log output.

Bug Fixes
---------

- **power management**: min-residency-us of apollo510L has to be greater than 1 tick which is 1ms.
- **soc**: Moved kconfig define to prevent leakage, and bumped common kconfig to ambiq family level.
- **uart**: Wrap the TXCMP enable logic with CONFIG_PM_DEVICE so it is only used when PM runtime is active.

Deprecations
------------

- **mspi**: Removed async PIO operations.

Known Issues
------------


Release Information
===================

- **Version:** v0.2
- **Release Date:** 2025-10-31

New Features
------------

Drivers and Sensors
~~~~~~~~~~~~~~~~~~~

- **dc**: Added support for dc_spi driver of apollo510L.
- **gpu**: Added support for NemaGFX GPU driver of apollo510L.
- **i2s**: Added support for i2s driver of apollo510L.
- **mspi**: Added support for mspi driver of apollo510L.
- **pdm**: Added support for pdm driver of apollo510L.

HAL
~~~

- **apollo510L**: Added auto generate USB clock source to am_hal_usb.
- **NemaGFX**: Refactor HAL to use Zephyr GPU driver.

Libraries / Subsystems
~~~~~~~~~~~~~~~~~~~~~~

- **cpu_freq**: Added support for cpu HP/LP mode switching of apollo510L.
- **mcuboot**: Added smp_svr configurations for apollo510L_eb board.
- **power management**: Ported the tests under tests/subsys/pm to apollo510L_eb board.

Bug Fixes
---------


Deprecations
------------


Known Issues
------------
- apollo510L_eb supports MSPI hex psram device only by default and flash devices are disabled in the DTS.
  Board rework is required to run octal psram and octal flash in parallel as the MSPI hardware interface is limited.
  The CELatency is forced to 1 for apollo510L SoCs temporarily and will be updated in the future so that
  it is configurable per device in the DTS.


Release Information
===================

- **Version:** v0.1
- **Release Date:** 2025-09-10

New Features
------------

Build system and Infrastructure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- **ci**: Added CI support for apollo510L and covered all supported mcu features and basic bluetooth tests.

Bluetooth
~~~~~~~~~

- **hci drivers**: Added support for IPC HCI driver of apollo510L.
- **controller**: Added support for the radio subsystem of apollo510L.

Boards & SoC Support
~~~~~~~~~~~~~~~~~~~~

- **apollo510L soc**: Added support for apollo510L soc.
- **apollo510L_eb**: Added support for apollo510L_eb board.
- **apollo5_eb_display_card**: Added support for apollo5_eb_display_card.

Drivers and Sensors
~~~~~~~~~~~~~~~~~~~

- **adc**: Added support for ADC driver of apollo510L.
- **counter**: Added support for Counter driver of apollo510L.
- **display**: Added support for CO5300 display driver on apollo510L_eb.
- **flash**: Added support for MRAM driver of apollo510L.
- **gpio**: Added support for GPIO driver of apollo510L.
- **hwinfo**: Added support for HWINFO driver of apollo510L.
- **i2c**: Added support for I2C driver of apollo510L.
- **mbox**: Added support for MBOX driver of apollo510L.
- **mipi-dsi**: Added support for MIPI-DSI driver of apollo510L.
- **pwm**: Added support for PWM driver of apollo510L.
- **rtc**: Added support for RTC driver of apollo510L.
- **sdhc**: Added support for SDIO driver of apollo510L.
- **sensor**: Added support for accel sensor adxl363 on apollo510L_eb.
- **serial**: Added support for UART driver of apollo510L.
- **spi**: Added support for SPI driver of apollo510L.
- **timer**: Added support for Stimer driver of apollo510L.
- **trng**: Added support for TRNG driver of apollo510L.
- **usb**: Added support for USB driver of apollo510L.
- **wdt**: Added support for WDT driver of apollo510L.

HAL
~~~

- **apollo510L**: Upgraded apollo510L HAL to sdk5.2.alpha.1.

Libraries / Subsystems
~~~~~~~~~~~~~~~~~~~~~~

- **coremark**: Added support for apollo510L coremark benchmark.
- **mcuboot**: Added mcumgr and mcuboot configurations for apollo510L_eb board.
- **power management**: Added support for apollo510L deeper sleep mode.
- **storage**: Added zms configurations for apollo510L_eb board.

Bug Fixes
---------


Deprecations
------------


Known Issues
------------
- The ``test_cpu_idle`` test under tests/kernel/context may fail when using the STIMER
  with default configurations (XTAL 32.768 kHz source, ``CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=32768``,
  ``CONFIG_SYS_CLOCK_TICKS_PER_SEC=1024``), due to tick announced from STIMER being off by one,
  which may be the result of accumulated error from single digit clock cycle delay.
