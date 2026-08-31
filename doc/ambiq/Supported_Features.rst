Supported Features
##################

This page lists the Ambiq boards in this tree and the peripheral, power
management, and third-party library support that ships with them.

See also:

- `README.rst <README.rst>`_ — Ambiq documentation index
- `How_to_Build_and_Flash.rst <How_to_Build_and_Flash.rst>`_
- `Power_Management.rst <Power_Management.rst>`_
- `../../samples/README.rst <../../samples/README.rst>`_ and
  `../../tests/README.rst <../../tests/README.rst>`_ for per-board build commands

Boards
******

Apollo5 family:

- ``apollo510_evb``
- ``apollo510b_evb``
- ``apollo510dL_evb``
- ``apollo330mP_evb``

Apollo4 family:

- ``apollo4p_evb``
- ``apollo4p_blue_kxr_evb``

Apollo3 family:

- ``apollo3_evb``
- ``apollo3p_evb``

Apollo2 family:

- ``apollo2_evb``

Display and expansion shields:

- ``ap510_disp``
- ``ap510_jdi_disp``
- ``ap4_evb_disp_shield``
- ``ambiq_mikrobus_i2c`` / ``ambiq_mikrobus_spi``

Driver Support
**************

As of now, Ambiq provides zephyr support for a set of peripherals/drivers:

This table reflects the current Ambiq board focus and PM behavior in this tree.

+--------+------------------+--------------------+-----------------------------------------------+------------------+
| Driver | PM_DEVICE        | Stable code at     | Sample/Test                                   | Board            |
+========+==================+====================+===============================================+==================+
| ADC    | PM + RUNTIME     | ambiq-stable       | samples\\drivers\\adc\\adc\_dt                | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| AUDADC | PM + RUNTIME     | ambiq-stable       | samples\\drivers\\audio\\amic                 | apollo510dL_evb  |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| BLE    | Not Included (1) | ambiq-stable       | samples\\bluetooth                            | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| COUNTER| PM + RUNTIME     | ambiq-stable       | samples\\drivers\\counter\\alarm              | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| CRC    | policy locks     | ambiq-stable       | samples\\drivers\\crc                         | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| CRYPTO | PM + RUNTIME     | ambiq-stable       | tests\\boards\\ambiq\\aes_hal_example         | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| DISPLAY| Not Included (1) | ambiq-stable       | samples\\drivers\\display                     | with disp card   |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| FLASH  | not needed       | ambiq-stable       | samples\\subsys\\mgmt\\mcumgr\\smp\_svr       | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| GPIO   | Not Included (1) | ambiq-stable       | tests\drivers\gpio\gpio\_basic\_api           | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| HWINFO | Not Included (1) | ambiq-stable       | tests\\drivers\\hwinfo\\api                   | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| I2C    | PM + RUNTIME     | ambiq-stable       | samples\\drivers\\eeprom                      | apollo510dL_evb  |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| I2S    | PM Only (2)      | ambiq-stable       | samples\\drivers\\i2s\\output                 | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| I3C    | Not Included (1) | coming soon        |                                               | apollo510dL_evb  |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| INPUT  | PM Only (2)      | ambiq-stable       | samples\\subsys\\input\\draw\_touch\_events   | with disp card   |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| JDI    | Not Included (1) | ambiq-stable       | samples\\drivers\\display                     | with disp card   |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| MBOX   | Not Included (1) | ambiq-stable       | radio subsystem IPC (no standalone sample)    | 330P / 510L      |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
|MIPI_DBI| Not Included (1) | ambiq-stable       | samples\\drivers\\display                     | with disp card   |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
|MIPI_DSI| PM + RUNTIME     | ambiq-stable       | samples\\drivers\\display                     | with disp card   |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| MSPI   | PM + RUNTIME     | ambiq-stable       | samples\\drivers\\mspi\\mspi\_flash           | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| PDM    | PM Only (2)      | ambiq-stable       | samples\\drivers\\audio\\dmic                 | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| PWR_DOM| PM Only (2)      | ambiq-stable       | exercised via CRYPTO and DISPLAY              | apollo5x         |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| PWM    | Not Included (1) | ambiq-stable       | tests\\drivers\\pwm\\pwm\_api                 | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| RTC    | always-on clk    | ambiq-stable       | samples\\drivers\\rtc                         | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| SDHC   | PM + RUNTIME     | ambiq-stable       | tests\\subsys\\sd\\sdio                       | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| SPI    | PM + RUNTIME     | ambiq-stable       | samples\\boards\\ambiq\\spi\_serial\_flash    | apollo510dL_evb  |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| TIMER  | Not Included (1) | ambiq-stable       | samples\\philosophers                         | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| TRNG   | PM + RUNTIME     | ambiq-stable       | tests\\drivers\\entropy\\api                  | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| UART   | PM Only (2)      | ambiq-stable       | samples\\drivers\\uart\\echo\_bot             | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| USB    | Not Included (1) | ambiq-stable       | samples\\subsys\\usb\\mass                    | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| WDT    | always-on clk    | ambiq-stable       | samples\\drivers\\watchdog                    | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+

PM_DEVICE column legend:

- ``PM + RUNTIME`` — driver registers a ``PM_DEVICE_DT_INST_DEFINE`` action handler
  and uses ``pm_device_runtime_get`` / ``pm_device_runtime_put`` to power-cycle the
  peripheral on demand. Set ``CONFIG_PM_DEVICE=y`` and ``CONFIG_PM_DEVICE_RUNTIME=y``
  to take advantage of this.
- ``PM Only (2)`` — driver registers a ``PM_DEVICE_DT_INST_DEFINE`` suspend/resume action
  handler (used during system-managed deep sleep) but does not use runtime PM.
  Enabled with ``CONFIG_PM_DEVICE=y``. Examples: UART (must stay powered for RX),
  PDM (active during audio capture), I2S, INPUT.
- ``policy locks`` — no PM device hooks, but operations use PM policy locks to prevent
  entering deep sleep while hardware state is active (for example, CRC sessions).
- ``always-on clk`` — peripheral uses always-on low-frequency clock sources required
  across sleep states, so device suspend/resume hooks are unnecessary (for example, RTC/WDT).
- ``not needed`` — peripheral remains memory accessible without device-level power
  gating, so PM device hooks provide no benefit (for example, MRAM FLASH access).
- ``Not Included (1)`` — no PM hooks needed. Peripheral is either always-on by design
  (for example TIMER), externally powered (BLE, USB), or has no significant
  power draw when idle (HWINFO, DISPLAY controllers).

Third-Party Library Support
***************************

+--------+----------------+--------------------+-------------------------------------------+------------------+
|   Lib  |     Status     |   Stable code at   |              Sample                       |       Board      |
+========+================+====================+===========================================+==================+
|coremark|       -        |    ambiq-stable    |  samples\\benchmarks\\coremark            |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|  fatfs |       -        |    ambiq-stable    |  samples\\subsys\\fs\\fs_sample           |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
| mbedtls|    SW only     |    ambiq-stable    |  tests\\benchmarks\\mbedtls               |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|  lvgl  |       -        |    ambiq-stable    |  samples\\modules\\lvgl\\demos            |  with disp card  |
+--------+----------------+--------------------+-------------------------------------------+------------------+

Together with generic support for ARM Cortex-M peripherals like cache,
interrupt controller, etc.

Get to Know Ambiq Components
****************************

.. code-block:: text

  zephyr/
  │
  ├── boards/
  │   ├── ambiq/
  │   │   ├── apollo2_evb/
  │   │   ├── apollo330mP_evb/
  │   │   ├── apollo3_evb/
  │   │   ├── apollo3p_evb/
  │   │   ├── apollo4p_blue_kxr_evb/
  │   │   ├── apollo4p_evb/
  │   │   ├── apollo510_evb/
  │   │   ├── apollo510b_evb/
  │   │   └── apollo510dL_evb/
  │   └── shields/
  │       ├── ambiq_mikrobus_i2c/
  │       ├── ambiq_mikrobus_spi/
  │       ├── ap4_evb_disp_shield/
  │       ├── ap510_disp/
  │       └── ap510_jdi_disp/
  ├── drivers/
  │   ├── adc/
  │   │   └── adc_ambiq.c
  │   ├── audio/
  │   │   ├── amic_ambiq_audadc.c
  │   │   └── dmic_ambiq_pdm.c
  │   ├── bluetooth/
  │   │   └── hci/
  │   │       ├── apollox_blue.c
  │   │       ├── apollox_ipc_support.c
  │   │       ├── hci_ambiq.c
  │   │       └── ipc.c
  │   ├── clock_control/
  │   │   └── clock_control_ambiq.c
  │   ├── counter/
  │   │   └── counter_ambiq_timer.c
  │   ├── crc/
  │   │   └── crc_ambiq.c
  │   ├── crypto/
  │   │   └── crypto_ambiq_aes.c
  │   ├── display/
  │   │   ├── display_co5300.c
  │   │   └── display_ls014b7dd01.c
  │   ├── entropy/
  │   │   └── entropy_ambiq_puf_trng.c
  │   ├── flash/
  │   │   └── flash_ambiq.c
  │   ├── gpio/
  │   │   └── gpio_ambiq.c
  │   ├── gpu/
  │   │   └── gpu_ambiq.c
  │   ├── hwinfo/
  │   │   └── hwinfo_ambiq.c
  │   ├── i2c/
  │   │   ├── i2c_ambiq.c
  │   │   └── i2c_ambiq_ios.c
  │   ├── i2s/
  │   │   └── i2s_ambiq.c
  │   ├── input/
  │   │   └── input_chsc5x.c
  │   ├── jdi/
  │   │   └── jdi_ambiq.c
  │   ├── mbox/
  │   │   └── mbox_ambiq.c
  │   ├── mipi_dbi/
  │   │   └── mipi_dbi_ambiq.c
  │   ├── mipi_dsi/
  │   │   └── dsi_ambiq.c
  │   ├── mspi/
  │   │   ├── mspi_ambiq.h
  │   │   ├── mspi_ambiq_ap3.c
  │   │   ├── mspi_ambiq_ap4.c
  │   │   ├── mspi_ambiq_ap5.c
  │   │   └── mspi_ambiq_timing_scan.c
  │   ├── pinctrl/
  │   │   └── pinctrl_ambiq.c
  │   ├── power_domain/
  │   │   ├── power_domain_ambiq_crypto.c
  │   │   ├── power_domain_ambiq_disp.c
  │   │   └── power_domain_ambiq_otp.c
  │   ├── pwm/
  │   │   ├── pwm_ambiq_ctimer.c
  │   │   └── pwm_ambiq_timer.c
  │   ├── rtc/
  │   │   └── rtc_ambiq.c
  │   ├── sdhc/
  │   │   └── sdhc_ambiq.c
  │   ├── serial/
  │   │   └── uart_ambiq.c
  │   ├── spi/
  │   │   ├── spi_ambiq_bleif.c
  │   │   ├── spi_ambiq_dcif.c
  │   │   ├── spi_ambiq_spic.c
  │   │   └── spi_ambiq_spid.c
  │   ├── timer/
  │   │   └── ambiq_stimer.c
  │   ├── usb/
  │   │   └── udc/
  │   │       └── udc_ambiq.c
  │   └── watchdog/
  │       └── wdt_ambiq.c
  ├── dts/
  │   └── arm/
  │       └── ambiq/
  ├── modules/
  │   └── hal_ambiq/
  └── soc/
      └── ambiq/
          ├── apollo2x/
          ├── apollo3x/
          ├── apollo4x/
          └── apollo5x/
