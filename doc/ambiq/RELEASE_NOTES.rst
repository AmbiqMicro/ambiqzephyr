============================================
© 2025 Ambiq Micro Inc. All rights reserved.
============================================

Release Notes
=============

Release Version: 5.2.alpha.1

Release Date: 2025-09-10

Overview
--------

This release introduces new features, bug fixes, and performance improvements.

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
