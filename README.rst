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

Status
******

As of now, Ambiq provides zephyr support for a set of peripherals/drivers:

+--------+----------------+--------------------+-------------------------------------------+------------------+
| Driver |   Apollo510L   |   Stable codes at  |              Sample                       |       Board      |
+========+================+====================+===========================================+==================+
|   ADC  |       -        |    apollo510L-dev  | samples\\drivers\\adc\\adc\_dt            |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   BLE  |       -        |    apollo510L-dev  | samples\\bluetooth                        |apollo510L_eb only|
+--------+----------------+--------------------+-------------------------------------------+------------------+
| COUNTER|       -        |    apollo510L-dev  | samples\\drivers\\counter\\alarm          |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
| CRYPTO |  coming soon   |                    |                                           |                  |
+--------+----------------+--------------------+-------------------------------------------+------------------+
| DISPLAY|       -        |    apollo510L-dev  |  samples\\drivers\\display                |  with disp card  |
+--------+----------------+--------------------+-------------------------------------------+------------------+
| FLASH  |       -        |    apollo510L-dev  |  samples\\subsys\\mgmt\\mcumgr\\smp_svr   |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
| HWINFO |       -        |    apollo510L-dev  |  tests\\drivers\\hwinfo\\api              |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   I2C  |       -        |    apollo510L-dev  |  samples\\drivers\\eeprom                 |apollo510L_eb only|
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   I2S  |       -        |    apollo510L-dev  |  samples\\drivers\\audio\\amic            |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   I3C  |  coming soon   |                    |                                           |                  |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|  INPUT |  coming soon   |                    |                                           |                  |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|MIPI_DSI|       -        |    apollo510L-dev  |  samples\\drivers\\display                |  with disp card  |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|  MSPI  |       -        |    apollo510L-dev  |   samples\\drivers\\mspi\\mspi\_flash     |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   PDM  |       -        |    apollo510L-dev  |    samples\\drivers\\audio\\dmic          |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   PM   |       -        |    apollo510L-dev  |    samples\\subsys\\pm\\device\_pm        |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   PWM  |       -        |    apollo510L-dev  |  tests\\drivers\\pwm\\pwm\_api            |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   RTC  |       -        |    apollo510L-dev  |    samples\\drivers\\rtc                  |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|  SDHC  |       -        |    apollo510L-dev  |  tests\\subsys\\sd\\sdio                  |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   SPI  |       -        |    apollo510L-dev  |    samples\\sensor\\accel_polling         |apollo510L_eb only|
+--------+----------------+--------------------+-------------------------------------------+------------------+
|  TIMER |       -        |    apollo510L-dev  |    samples\\philosophers                  |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|  TRNG  |       -        |    apollo510L-dev  |  tests\\drivers\\entropy\\api             |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|  UART  |       -        |    apollo510L-dev  |   samples\\drivers\\uart\\echo\_bot       |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   USB  |       -        |    apollo510L-dev  |  samples\\drivers\\subsys\\usb\\mass      |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|   WDT  |       -        |    apollo510L-dev  |    samples\\drivers\\watchdog             |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+

And also there are supports for some third-party libs:

+--------+----------------+--------------------+-------------------------------------------+------------------+
|   Lib  |   Apollo510L   |   Stable codes at  |              Sample                       |       Board      |
+========+================+====================+===========================================+==================+
|coremark|       -        |    apollo510L-dev  |   samples\\benchmarks\\coremark           |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|  fatfs |       -        |    apollo510L-dev  |  samples\\subsys\\fs\\fs_sample           |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
| mbedtls|       -        |    apollo510L-dev  |  tests\\benchmarks\\mbedtls               |        All       |
+--------+----------------+--------------------+-------------------------------------------+------------------+
|  lvgl  |  coming soon   |                    |                                           |                  |
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

Follow `Getting Started Guide`_ to install denpendencies and use west tool to get the Zephyr source code to local.
Install Zephyr SDK and check whether the environment variables are set properly.


Upstream Repository Synchronization
-----------------------------------

Execute ``git remote -v`` to check if upstream has been configured.

If not, execute ``git remote add upstream https://github.com/AmbiqMicro/ambiqzephyr`` to configure the ambiqzephyr base to your upstream repository.

Execute ``git remote -v`` again to check if it configures successfully.

Execute ``git fetch upstream`` to fetch the upstream repository.

Execute ``git checkout apollo510L-dev`` to get the latest apollo510L development branch.


Get to Know Ambiq Components
----------------------------

.. code-block:: text

  zephyr/
  │
  ├── boards/
  │   ├── ambiq/
  │   │   └── apollo510L_eb
  │   └── shields/
  │       └── apollo5_eb_display_card
  ├── drivers/
  │   ├── adc/
  │   │   └── adc_ambiq.c
  │   ├── audio/
  │   │   └── dmic_ambiq_pdm.c
  │   ├── bluetooth/
  │   │   └── hci/
  │   │       ├── apollox_ipc_support.c
  │   │       └── ipc.c
  │   ├── clock_control/
  │   │   └── clock_control_ambiq.c
  │   ├── counter/
  │   │   └── counter_ambiq_timer.c
  │   ├── display/
  │   │   └── display_co5300.c
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
  │   ├── mbox/
  │   │   └── mbox_ambiq.c
  │   ├── mipi_dsi/
  │   │   └── dsi_ambiq.c
  │   ├── mspi/
  │   │   ├── mspi_ambiq_ap5.c
  │   │   └── mspi_ambiq_timing_scan.c
  │   ├── pinctrl/
  │   │   └── pinctrl_ambiq.c
  │   ├── pwm/
  │   │   └── pwm_ambiq_timer.c
  │   ├── rtc/
  │   │   └── rtc_ambiq.c
  │   ├── sdhc/
  │   │   └── sdhc_ambiq.c
  │   ├── serial/
  │   │   └── uart_ambiq.c
  │   ├── spi/
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
  │           └── ambiq_apollo510L.dtsi
  ├── modules/
  │   └── hal_ambiq
  └── soc/
      └── ambiq/
          └── apollo5x


Build and Flash the Samples
---------------------------

Make sure you have already installed proper version of JLINK which supports corresponding ambiq SoC, and
added the path of JLINK.exe (e.g. C:\Program Files\SEGGER\JLink) to the environment variables.

Go the Zephyr root path, execute ``west build -b <your-board-name> <samples> -p always`` to build the samples for your board.
For example, build zephyr/samples/hello_world for apollo510_evb: ``west build -b apollo510_evb ./samples/hello_world -p always``.

Execute ``west flash`` to flash the binary to the EVB if the zephyr.bin has been generated by west build.

In default we use UART COM for console, and the default baudrate is 115200, so after west flash, open the serial terminal and set proper baudrate for the UART COM of plugged EVB.

You should be able to see the logs in the serial terminal.

``*** Booting Zephyr OS build v4.1.0-7246-gad4c3e3e9afe ***``

``Hello World! apollo510L_eb/apollo510L``

For those samples that require additional hardware, such as the ap510_disp shield, you need to set the shield option when building. For example:

``west build -b apollo510L_eb --shield ap510_disp ./samples/drivers/display -p always``

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

Currently drivers below has fully PM_DEVICE support:
ADC
I2C
MIPI-DSI
MSPI
SDHC
SPI
UART
I2S
PDM

Special Note on MSPI
--------------------

1. The apollo510L_eb supports MSPI hex psram device only by default and flash devices are disabled in the DTS.
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
.. _Ambiq HAL Repository: https://github.com/AmbiqMicro/hal-zephyr-alpha
.. _Zephyr Power Management: https://docs.zephyrproject.org/latest/services/pm/index.html
