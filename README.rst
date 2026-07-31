.. raw:: html

   <a href="https://www.zephyrproject.org">
     <p align="center">
       <picture>
         <img src="doc/_static/images/Zephyr-support-for-Ambiq.svg">
       </picture>
     </p>
   </a>

   <a href="https://bestpractices.coreinfrastructure.org/projects/74"><img src="https://bestpractices.coreinfrastructure.org/projects/74/badge"></a>
   <a href="https://scorecard.dev/viewer/?uri=github.com/zephyrproject-rtos/zephyr"><img src="https://api.securityscorecards.dev/projects/github.com/zephyrproject-rtos/zephyr/badge"></a>
   <a href="https://github.com/zephyrproject-rtos/zephyr/actions/workflows/twister.yaml?query=branch%3Amain"><img src="https://github.com/zephyrproject-rtos/zephyr/actions/workflows/twister.yaml/badge.svg?event=push"></a>

Ambiq® is an Austin-based SoC vendor who at the forefront of enabling ambient intelligence on billions of
devices with the unique SPOT platform and extreme-low power semiconductor solutions.

Whether it's the Real Time Clock (RTC) IC, or a System-on-a-Chip (SoC), Ambiq® is committed to enabling the
lowest power consumption with the highest computing performance possible for our customers to make the most
innovative battery-power endpoint devices for their end-users. `Ambiq Products`_

Ambiq release branch
********************

Manufacturing and customer releases use ``ambiq-stable``. The release number is
in ``VERSION`` (for example ``4.3.0``) and is what the boot banner reports.

See `How_to_Setup_Toolchain.rst <doc/ambiq/How_to_Setup_Toolchain.rst>`_ for
toolchain setup and version verification.

.. code-block:: console

   git checkout ambiq-stable

Command References
******************

For command-first build references derived from the apollo510dL_evb and
apollo330mP_evb board DTS files, see:

- `samples/README.rst <samples/README.rst>`_
- `tests/README.rst <tests/README.rst>`_
- `doc/ambiq/How_to_Setup_Toolchain.rst <doc/ambiq/How_to_Setup_Toolchain.rst>`_
- `doc/ambiq/How_to_Run_Zephyr_USB_Samples.rst <doc/ambiq/How_to_Run_Zephyr_USB_Samples.rst>`_
- `doc/ambiq/How_to_Run_Zephyr_MSPI_Samples_and_Tests.rst <doc/ambiq/How_to_Run_Zephyr_MSPI_Samples_and_Tests.rst>`_
- `doc/ambiq/How_to_Run_MCUBoot_Samples_and_Tests.rst <doc/ambiq/How_to_Run_MCUBoot_Samples_and_Tests.rst>`_
- `doc/ambiq/RELEASE_NOTES.rst <doc/ambiq/RELEASE_NOTES.rst>`_


Status
******

As of now, Ambiq provides zephyr support for a set of peripherals/drivers:

This table reflects the current Ambiq board focus and PM behavior in this tree.

+--------+------------------+--------------------+-----------------------------------------------+------------------+
| Driver | PM_DEVICE        | Stable code at     | Sample/Test                                   | Board            |
+========+==================+====================+===============================================+==================+
| ADC    | PM + RUNTIME     | ambiq-stable       | samples\\drivers\\adc\\adc\_dt                | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| AUDADC | PM + RUNTIME     | ambiq-stable       | samples\\drivers\\adc\\adc\_dt                | apollo510dL_evb  |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| BLE    | Not Included (1) | ambiq-stable       | samples\\bluetooth                            | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| COUNTER| Not Included (1) | ambiq-stable       | samples\\drivers\\counter\\alarm              | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| CRC    | policy locks     | ambiq-stable       | samples\\drivers\\crc\\crc\_api               | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| CRYPTO | PM Only (2)      | ambiq-stable       | tests\\boards\\ambiq\\aes_hal_example         | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| DISPLAY| Not Included (1) | ambiq-stable       | samples\\drivers\\display                     | with disp card   |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| FLASH  | not needed       | ambiq-stable       | samples\\subsys\\mgmt\\mcumgr\\smp\_svr       | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| HWINFO | Not Included (1) | ambiq-stable       | tests\\drivers\\hwinfo\\api                   | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| I2C    | PM + RUNTIME     | ambiq-stable       | samples\\drivers\\eeprom                      | apollo510dL_evb  |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| I2S    | Not Included (1) | ambiq-stable       | samples\\drivers\\audio\\amic                 | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| I3C    | Not Included (1) | coming soon        |                                               | apollo510dL_evb  |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| INPUT  | Not Included (1) | ambiq-stable       | samples\\subsys\\input\\draw\_touch\_events   | with disp card   |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| JDI    | Not Included (1) | ambiq-stable       | samples\\drivers\\display                     | with disp card   |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
|MIPI_DBI| Not Included (1) | ambiq-stable       | samples\\drivers\\display                     | with disp card   |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
|MIPI_DSI| PM + RUNTIME     | ambiq-stable       | samples\\drivers\\display                     | with disp card   |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| MSPI   | PM + RUNTIME     | ambiq-stable       | samples\\drivers\\mspi\\mspi\_flash           | All              |
+--------+------------------+--------------------+-----------------------------------------------+------------------+
| PDM    | PM Only (2)      | ambiq-stable       | samples\\drivers\\audio\\dmic                 | All              |
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
| USB    | Not Included (1) | ambiq-stable       | samples\\drivers\\subsys\\usb\\mass           | All              |
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
  Enabled with ``CONFIG_PM_DEVICE=y``. Examples: CRYPTO (per-operation power via
  shared power rail), UART (must stay powered for RX), PDM (active during audio capture).
- ``policy locks`` — no PM device hooks, but operations use PM policy locks to prevent
  entering deep sleep while hardware state is active (for example, CRC sessions).
- ``always-on clk`` — peripheral uses always-on low-frequency clock sources required
  across sleep states, so device suspend/resume hooks are unnecessary (for example, RTC/WDT).
- ``not needed`` — peripheral remains memory accessible without device-level power
  gating, so PM device hooks provide no benefit (for example, MRAM FLASH access).
- ``Not Included (1)`` — no PM hooks needed. Peripheral is either always-on by design
  (for example COUNTER/TIMER), externally powered (BLE, USB), or has no significant
  power draw when idle (HWINFO, DISPLAY controllers).

And also there are supports for some third-party libs:

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


Together with generic support for ARM Cortex-M peripherals like cache, interrupt controller, etc.


Release Notes
*************

See the full release notes: `release_note.rst <doc/ambiq/RELEASE_NOTES.rst>`_

.. below included in doc/introduction/introduction.rst


Getting Started
***************

Welcome to Ambiq Zephyr! See the `Introduction to Zephyr`_ for a high-level overview,
and the documentation's `Getting Started Guide`_ to start developing.

Check `Ambiq SoC`_ for Ambiq Apollo SoCs documents.


Setup Development Environment
-----------------------------

Follow `Getting Started Guide`_ to install dependencies and use west to set up
the workspace. Install the Zephyr SDK version matched to your release branch
(see `How_to_Setup_Toolchain.rst <doc/ambiq/How_to_Setup_Toolchain.rst>`_) and
confirm ``ZEPHYR_BASE``, ``ZEPHYR_TOOLCHAIN_VARIANT``, and
``ZEPHYR_SDK_INSTALL_DIR`` are set in your shell.


Upstream Repository Synchronization
-----------------------------------

Execute ``git remote -v`` to check if upstream has been configured.

If not, execute ``git remote add upstream https://github.com/AmbiqMicro/sdk-zephyr-alpha`` to configure the sdk-zephyr-alpha base to your upstream repository.

Execute ``git remote -v`` again to check if it configures successfully.

Execute ``git fetch upstream`` to fetch the upstream repository.

Execute ``git checkout ambiq-stable`` for the current manufacturing release
branch, or check out a release tag (for example ``git checkout v4.3.0``).


Get to Know Ambiq Components
----------------------------

.. code-block:: text

  zephyr/
  │
  ├── boards/
  │   ├── ambiq/
  │   │   ├── apollo330mP_evb/
  │   │   └── apollo510dL_evb/
  │   └── shields/
  │       └── ap510_disp/
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
  │   ├── hwinfo/
  │   │   └── hwinfo_ambiq.c
  │   ├── i2c/
  │   │   ├── i2c_ambiq.c
  │   │   └── i2c_ambiq_ios.c
  │   ├── i2s/
  │   │   └── i2s_ambiq.c
  │   ├── jdi/
  │   │   └── jdi_ambiq.c
  │   ├── mbox/
  │   │   └── mbox_ambiq.c
  │   ├── mipi_dbi/
  │   │   └── mipi_dbi_ambiq.c
  │   ├── mipi_dsi/
  │   │   └── dsi_ambiq.c
  │   ├── misc/
  │   │   └── ambiq_pwrctrl/
  │   │       └── ambiq_pwrctrl.c
  │   ├── mspi/
  │   │   ├── mspi_ambiq.h
  │   │   ├── mspi_ambiq_ap3.c
  │   │   ├── mspi_ambiq_ap4.c
  │   │   ├── mspi_ambiq_ap5.c
  │   │   └── mspi_ambiq_timing_scan.c
  │   ├── pinctrl/
  │   │   └── pinctrl_ambiq.c
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
  │   └── hal_ambiq
  └── soc/
      └── ambiq/


Build and Flash the Samples
---------------------------

Make sure you have already installed proper version of JLINK which supports corresponding ambiq SoC, and
added the path of JLINK.exe (e.g. C:\Program Files\SEGGER\JLink) to the environment variables.

Go the Zephyr root path, execute ``west build -b <board> <samples> -p always`` to build the samples for your board.
For example, build zephyr/samples/hello_world: ``west build -b <board> ./samples/hello_world -p always``.

Execute ``west flash`` to flash the binary to the EVB if the zephyr.bin has been generated by west build.

In default we use UART COM for console, and the default baudrate is 115200, so after west flash, open the serial terminal and set proper baudrate for the UART COM of plugged EVB.

You should be able to see the logs in the serial terminal.

``*** Booting Zephyr OS build 4.3.0 ***``

``Hello World! apollo510_evb``

For those samples that require additional hardware, such as the ap510_disp shield, you need to set the shield option when building. For example:

``west build -b <board> --shield ap510_disp ./samples/drivers/display -p always``

For Bluetooth samples, you need to program the BLE Controller firmware via JLINK once before running samples. The programming script and binary locate in ambiq SDK
ambiqsuite/tools/apollo510L_scripts/firmware_updates/cm4_ble_updates/ble_v1p2. Please get the SDK from `Ambiq Content Portal`_.

For MSPI samples, please refer to `How_to_Run_Zephyr_MSPI_Samples_and_Tests.rst <doc/ambiq/How_to_Run_Zephyr_MSPI_Samples_and_Tests.rst>`_

For USB samples, please refer to `How_to_Run_Zephyr_USB_Samples.rst <doc/ambiq/How_to_Run_Zephyr_USB_Samples.rst>`_

For MCU_Boot samples, please refer to `How_to_Run_MCUBoot_Samples_and_Tests.rst <doc/ambiq/How_to_Run_MCUBoot_Samples_and_Tests.rst>`_


Power Management Instructions
-----------------------------

To achieve lowest power consumption, customer needs to follow the following process for inspection and configuration optimization:

1. According to the acutal usage of memory, adjust the memory configurations in soc_early_init_hook;

2. Make sure ``CONFIG_PM=y``, ``CONFIG_PM_DEVICE=y``, and ``CONFIG_PM_DEVICE_RUNTIME=y``, and explicitly enable ``CONFIG_PM_DEVICE_SYSTEM_MANAGED=y``
   so remaining devices are suspended automatically before deep sleep.

3. Audit the Devicetree to disable unused peripherals ``status = "disabled"`` or keep ``zephyr,pm-device-runtime-auto`` on blocks that should be power-managed automatically;
   remove the property from peripherals that must stay on.

4. In application code, call pm_device_runtime_put() / pm_device_runtime_get() around peripherals that should power-cycle on demand to ensure their usage count returns to zero after use.

Check `Zephyr Power Management`_ for more detailed information.

Special Note on MSPI
--------------------

1. The apollo510dL_evb supports MSPI hex psram device only by default and flash devices are disabled in the DTS.
   Board rework is required to run octal psram and octal flash in parallel as the MSPI hardware interface is limited.

2. The CELatency is forced to 1 for apollo510L SoCs temporarily and will be updated in the future so that it is configurable per device in the DTS.

.. start_include_here

Community Support
*****************

Community support is provided via mailing lists and Discord; see the Resources
below for details.

.. _project-resources:

Resources
*********

Here's a quick summary of resources to help you find your way around:

Getting Started
---------------

  | 📖 `Zephyr Documentation`_
  | 🚀 `Getting Started Guide`_
  | 🙋🏽 `Tips when asking for help`_
  | 💻 `Code samples`_

Code and Development
--------------------

  | 🌐 `Source Code Repository`_
  | 🌐 `Ambiq HAL Repository`_
  | 📦 `Releases`_
  | 🤝 `Contribution Guide`_

Community and Support
---------------------

  | 💬 `Discord Server`_ for real-time community discussions
  | 📧 `User mailing list (users@lists.zephyrproject.org)`_
  | 📧 `Developer mailing list (devel@lists.zephyrproject.org)`_
  | 📬 `Other project mailing lists`_
  | 📚 `Project Wiki`_

Issue Tracking and Security
---------------------------

  | 🐛 `GitHub Issues`_
  | 🔒 `Security documentation`_
  | 🛡️ `Security Advisories Repository`_
  | ⚠️ Report security vulnerabilities at vulnerabilities@zephyrproject.org

Additional Resources
--------------------
  | 🌐 `Zephyr Project Website`_
  | 📺 `Zephyr Tech Talks`_

.. _Zephyr Project Website: https://www.zephyrproject.org
.. _Discord Server: https://chat.zephyrproject.org
.. _Zephyr Documentation: https://docs.zephyrproject.org
.. _Introduction to Zephyr: https://docs.zephyrproject.org/latest/introduction/index.html
.. _Getting Started Guide: https://docs.zephyrproject.org/latest/develop/getting_started/index.html
.. _Contribution Guide: https://docs.zephyrproject.org/latest/contribute/index.html
.. _Source Code Repository: https://github.com/AmbiqMicro/ambiqzephyr
.. _GitHub Issues: https://github.com/AmbiqMicro/ambiqzephyr/issues
.. _Releases: https://github.com/zephyrproject-rtos/zephyr/releases
.. _Project Wiki: https://github.com/zephyrproject-rtos/zephyr/wiki
.. _User mailing list (users@lists.zephyrproject.org): https://lists.zephyrproject.org/g/users
.. _Developer mailing list (devel@lists.zephyrproject.org): https://lists.zephyrproject.org/g/devel
.. _Other project mailing lists: https://lists.zephyrproject.org/g/main/subgroups
.. _Code samples: https://docs.zephyrproject.org/latest/samples/index.html
.. _Security documentation: https://docs.zephyrproject.org/latest/security/index.html
.. _Security Advisories Repository: https://github.com/zephyrproject-rtos/zephyr/security
.. _Tips when asking for help: https://docs.zephyrproject.org/latest/develop/getting_started/index.html#asking-for-help
.. _Zephyr Tech Talks: https://www.zephyrproject.org/tech-talks
.. _Ambiq SoC: https://contentportal.ambiq.com/soc
.. _Ambiq Products: https://ambiq.com/products/
.. _Ambiq Content Portal: https://contentportal.ambiq.com/
.. _Ambiq HAL Repository: https://github.com/AmbiqMicro/hal_ambiq_internal
.. _Zephyr Power Management: https://docs.zephyrproject.org/latest/services/pm/index.html
