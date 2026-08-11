.. zephyr:board:: apollo510_evb

Apollo510 EVB is a board by Ambiq featuring their ultra-low power Apollo510 SoC.

Hardware
********

- Apollo510 SoC with up to 250 MHz operating frequency
- ARM® Cortex®-M55 core
- 64 kB Instruction Cache and 64 kB Data Cache
- Up to 4 MB of non-volatile memory (NVM) for code/data
- Up to 3 MB of low leakage / low power RAM for code/data
- 256 kB Instruction Tightly Coupled RAM (ITCM)
- 512 kB Data Tightly Coupled RAM (DTCM)

For more information about the Apollo510 SoC and Apollo510 EVB board:

- `Apollo510 Website`_
- `Apollo510 Datasheet`_
- `Apollo510 EVB Website`_

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
   :board: apollo510_evb
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

   Hello World! apollo510_evb

Building other samples
----------------------

The samples and tests below are built for this board as part of the Ambiq SDK
release test. Paths are relative to the workspace root.

* ``samples/basic/blinky_pwm``
* ``samples/basic/button``
* ``samples/basic/threads``
* ``samples/benchmarks/coremark``
* ``samples/boards/ambiq/dram_click`` (``-- -DSHIELD=ambiq_mikrobus_spi;mikroe_dram_click``)
* ``samples/boards/ambiq/spi_serial_flash``
* ``samples/cpp/hello_world``
* ``samples/drivers/adc/adc_dt``
* ``samples/drivers/audio/amic``
* ``samples/drivers/audio/dmic``
* ``samples/drivers/counter/alarm``
* ``samples/drivers/crc``
* ``samples/drivers/display`` (``-- -DSHIELD=ap510_disp``)
* ``samples/drivers/eeprom``
* ``samples/drivers/i2c/target_eeprom``
* ``samples/drivers/i2s/output``
* ``samples/drivers/led/pwm``
* ``samples/drivers/memc``
* ``samples/drivers/mspi/mspi_flash``
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
* ``tests/boards/ambiq/aes_hal_example``
* ``tests/boot/mcuboot_data_sharing`` (``-T bootloader.mcuboot.data.sharing``)
* ``tests/boot/test_mcuboot`` (``-T bootloader.mcuboot``)
* ``tests/boot/with_mcumgr`` (``-T boot.with_mcumgr.test_upgrade``)
* ``tests/boot/with_mcumgr`` (``-T boot.with_mcumgr.test_upgrade.swap_using_offset``)
* ``tests/crypto/mbedtls_psa``
* ``tests/crypto/secp256r1`` (``-- -DEXTRA_CONF_FILE=mbedtls.conf``)
* ``tests/crypto/secp256r1`` (``-- -DEXTRA_CONF_FILE=p256-m_raw.conf``)
* ``tests/drivers/adc/adc_api``
* ``tests/drivers/audio/dmic_api``
* ``tests/drivers/counter/counter_basic_api``
* ``tests/drivers/disk/disk_performance``
* ``tests/drivers/entropy/api``
* ``tests/drivers/flash/common``
* ``tests/drivers/flash/erase_blocks``
* ``tests/drivers/gpio/gpio_basic_api``
* ``tests/drivers/hwinfo/api``
* ``tests/drivers/i2s/i2s_api``
* ``tests/drivers/mspi/api`` (``-T drivers.mspi.api.ambiq``)
* ``tests/drivers/mspi/flash``
* ``tests/drivers/pwm/pwm_api``
* ``tests/drivers/retained_mem/api``
* ``tests/drivers/rtc/rtc_api``
* ``tests/drivers/spi/spi_loopback``
* ``tests/drivers/watchdog/wdt_basic_api``
* ``tests/kernel/common``
* ``tests/kernel/context``
* ``tests/kernel/fifo/fifo_api``
* ``tests/kernel/mutex/mutex_api``
* ``tests/kernel/queue``
* ``tests/kernel/semaphore/semaphore``
* ``tests/kernel/sleep``
* ``tests/kernel/threads/thread_apis``
* ``tests/kernel/timer/timer_api``
* ``tests/kernel/workq/work``
* ``tests/lib/heap``
* ``tests/subsys/debug/cpu_load``
* ``tests/subsys/dfu/img_util`` (``-T dfu.image_util``)
* ``tests/subsys/logging/log_api``
* ``tests/subsys/mem_mgmt/mem_attr_heap``
* ``tests/subsys/mgmt/mcumgr/img_mgmt_slot_info``
* ``tests/subsys/pm/power_mgmt_soc``
* ``tests/subsys/pm/power_wakeup_timer``
* ``tests/subsys/sd/sdmmc``
* ``tests/subsys/settings/retention``

Samples marked with a shield need it named on the command line, because
``west build`` does not read ``sample.yaml``:

.. code-block:: console

   west build -p always -b apollo510_evb <sample path> -- -DSHIELD=ap510_disp

.. _Apollo510 Website:
   https://ambiq.com/apollo510/

.. _Apollo510 Datasheet:
   https://contentportal.ambiq.com/documents/20123/2877485/Apollo510-SoC-Datasheet.pdf

.. _Apollo510 EVB Website:
   For more information, please reach out to Sales and FAE.

.. _SEGGER J-Link software:
   https://www.segger.com/downloads/jlink

.. _pylink:
   https://github.com/Square/pylink
