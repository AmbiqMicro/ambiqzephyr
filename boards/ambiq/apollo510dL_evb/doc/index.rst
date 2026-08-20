.. zephyr:board:: apollo510dL_evb

Apollo510dL EVB is a board by Ambiq featuring their ultra-low power Apollo510dL SoC.

Hardware
********

- Apollo510dL SoC with up to 250 MHz operating frequency
- ARM® Cortex® M55 core
- Integrated 32 kB Instruction Cache and 32 kB Data Cache
- Up to 2 MB of non-volatile memory2 (NVM) for code/data
- 2 MB of TCM and system RAM for code/data
- Integrated 256 kB Instruction and Data Tightly Coupled Memory (ITCM and DTCM)

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and Debugging
=========================

.. zephyr:board-supported-runners::

Flashing an application
-----------------------

Connect your device to your host computer using the JLINK USB port.
The sample application :zephyr:code-sample:`hello_world` is used for this example.
Build the Zephyr kernel and application, then flash it to the device:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: apollo510dL_evb
   :goals: flash

.. note::
   ``west flash`` requires `SEGGER J-Link software`_ and `pylink`_ Python module
   to be installed on you host computer.

Open a serial terminal (minicom, putty, etc.) with the following settings:

- Speed: 115200
- Data: 8 bits
- Parity: None
- Stop bits: 1

Reset the board and you should be able to see on the corresponding Serial Port
the following message:

.. code-block:: console

   Hello World! apollo510dL_evb

Building other samples
----------------------

The samples and tests below are built for this board as part of the Ambiq SDK
release test. Paths are relative to the workspace root.

* ``samples/basic/blinky_pwm``
* ``samples/basic/button``
* ``samples/basic/threads``
* ``samples/benchmarks/coremark``
* ``samples/bluetooth/peripheral``
* ``samples/boards/ambiq/dram_click`` (``-- -DSHIELD=ambiq_mikrobus_spi;mikroe_dram_click``)
* ``samples/boards/ambiq/pm_peripheral_demo``
* ``samples/cpp/hello_world``
* ``samples/drivers/adc/adc_dt``
* ``samples/drivers/audio/dmic``
* ``samples/drivers/counter/alarm``
* ``samples/drivers/crc``
* ``samples/drivers/display`` (``-- -DSHIELD=ap510_disp``)
* ``samples/drivers/i2s/output``
* ``samples/drivers/led/pwm``
* ``samples/drivers/mbox_data``
* ``samples/drivers/memc``
* ``samples/drivers/mspi/mspi_timing_scan``
* ``samples/drivers/rtc``
* ``samples/drivers/watchdog``
* ``samples/hello_world``
* ``samples/modules/lvgl/demos`` (``-- -DSHIELD=ap510_disp``)
* ``samples/philosophers``
* ``samples/subsys/fs/fs_sample``
* ``samples/subsys/input/draw_touch_events`` (``-- -DSHIELD=ap510_disp``)
* ``samples/subsys/logging/logger``
* ``samples/subsys/mgmt/mcumgr/smp_svr`` (``--sysbuild -T sample.mcumgr.smp_svr.serial``)
* ``samples/subsys/usb/mass``
* ``samples/synchronization``
* ``samples/sysbuild/with_mcuboot`` (``--sysbuild``)
* ``tests/arch/arm/arm_irq_vector_table``
* ``tests/benchmarks/mbedtls``
* ``tests/boot/mcuboot_data_sharing`` (``-T bootloader.mcuboot.data.sharing``)
* ``tests/boot/test_mcuboot`` (``-T bootloader.mcuboot``)
* ``tests/boot/with_mcumgr`` (``-T boot.with_mcumgr.test_upgrade``)
* ``tests/boot/with_mcumgr`` (``-T boot.with_mcumgr.test_upgrade.swap_using_offset``)
* ``tests/drivers/adc/adc_api``
* ``tests/drivers/audio/dmic_api``
* ``tests/drivers/counter/counter_basic_api``
* ``tests/drivers/entropy/api``
* ``tests/drivers/flash/common``
* ``tests/drivers/flash/erase_blocks``
* ``tests/drivers/gpio/gpio_basic_api``
* ``tests/drivers/hwinfo/api``
* ``tests/drivers/i2c/i2c_api``
* ``tests/drivers/pwm/pwm_api``
* ``tests/drivers/retained_mem/api``
* ``tests/drivers/rtc/rtc_api``
* ``tests/kernel/common``
* ``tests/kernel/context``
* ``tests/kernel/fifo/fifo_api``
* ``tests/kernel/mutex/mutex_api``
* ``tests/kernel/queue``
* ``tests/kernel/semaphore/semaphore``
* ``tests/kernel/sleep``
* ``tests/kernel/threads/thread_apis``
* ``tests/kernel/timer/timer_api``
* ``tests/kernel/timer/timer_behavior``
* ``tests/kernel/workq/work``
* ``tests/lib/heap``
* ``tests/subsys/debug/cpu_load``
* ``tests/subsys/dfu/img_util`` (``-T dfu.image_util``)
* ``tests/subsys/logging/log_api``
* ``tests/subsys/mgmt/mcumgr/img_mgmt_slot_info``
* ``tests/subsys/pm/power_mgmt_soc``
* ``tests/subsys/pm/power_residency_time``
* ``tests/subsys/pm/power_states``
* ``tests/subsys/pm/power_wakeup_timer``
* ``tests/subsys/sd/sdio``
* ``tests/subsys/sd/sdmmc``
* ``tests/subsys/settings/retention``

Samples marked with a shield need it named on the command line, because
``west build`` does not read ``sample.yaml``:

.. code-block:: console

   west build -p always -b apollo510dL_evb <sample path> -- -DSHIELD=ap510_disp

.. _SEGGER J-Link software:
   https://www.segger.com/downloads/jlink

.. _pylink:
   https://github.com/Square/pylink
