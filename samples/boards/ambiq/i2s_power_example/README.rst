.. zephyr:code-sample:: ambiq-i2s-power-example
   :name: Ambiq I2S Power Example
   :relevant-api: i2s_interface

   Measure I2S TX running power: boot banner on UART, then suspend the console
   and apply the AmbiqSuite i2s_power memory profile in the sample.

Overview
********

This Ambiq board sample mirrors AmbiqSuite
``boards/common5/examples/audio/i2s_power``:

1. Boots with UART console and prints a short banner.
2. Prints ``Shutting down console for power measurement``, drains the UART,
   disables the log UART backend, and suspends the console UART device.
3. Applies the minimum MCU/SRAM retain profile and APBDMA hysteresis trim in
   ``main()`` (not in the I2S driver).
4. Streams a 2 kHz sine on I2S0 TX (master) at 16 kHz / 2 ch / 32-bit while
   Zephyr ``CONFIG_PM`` idle windows allow deep sleep between TX queue fills.

Board overlays disable unused peripherals (MSPI/PSRAM, USB, SDIO, GPIO banks,
crypto, etc.) while leaving ``uart0`` enabled for the banner. That is the
Zephyr stand-in for AmbiqSuite ``DIS_PERIPHS_ALL``. BLE FIT-LP SoC options and
IPC/SRAM remaps are intentionally not used.

Requirements
************

* Apollo510 EVB, Apollo510B EVB, Apollo510L EB, or Apollo330mP EVB
* Optional external connection to observe SCLK/WS/SDOUT; not required for
  current measurement on VDD_MCU

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/i2s_power_example
   :board: apollo510_evb
   :goals: build flash
   :compact:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/i2s_power_example
   :board: apollo510b_evb
   :goals: build flash
   :compact:

Sample Output
*************

.. code-block:: console

   *** Booting Zephyr OS build ... ***
   <inf> ambiq_i2s_power: Ambiq I2S power example
   <inf> ambiq_i2s_power: I2S0 TX 16000 Hz / 2 ch / 32-bit (AmbiqSuite i2s_power profile)
   <inf> ambiq_i2s_power: Unused peripherals disabled in overlay; console stays up for banner only
   Shutting down console for power measurement

After that line, the console is suspended. Measure VDD_MCU while I2S TX runs.

Pins (Apollo510 / Apollo510B EVB)
*********************************

I2S0 TX uses BSP pins GPIO5 (CLK), GPIO6 (SDOUT), GPIO7 (WS).
