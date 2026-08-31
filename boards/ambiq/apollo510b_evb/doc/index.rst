.. zephyr:board:: apollo510_evb

Apollo510 EVB is a board by Ambiq featuring their ultra-low power Apollo510 SoC.

Hardware
********

- Apollo510 SoC with up to 250 MHz operating frequency
- ARM® Cortex® M55 core
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

   Hello World! apollo510b_evb

Building other samples
----------------------

The samples and tests below are built for this board as part of the Ambiq
overnight release build set. Paths are relative to the zephyr repository.

* ``samples/basic/blinky_pwm``
* ``samples/basic/button``
* ``samples/basic/threads``
* ``samples/benchmarks/coremark``
* ``samples/bluetooth/peripheral``
* ``samples/boards/ambiq/dram_click`` (``-- -DSHIELD=ambiq_mikrobus_spi,mikroe_dram_click``)
* ``samples/cpp/hello_world``
* ``samples/drivers/adc/adc_dt``
* ``samples/drivers/audio/amic``
* ``samples/drivers/audio/dmic``
* ``samples/drivers/counter/alarm``
* ``samples/drivers/crc``
* ``samples/drivers/display`` (``-- -DSHIELD=ap510_disp``)
* ``samples/drivers/eeprom`` (``-- -DSHIELD=ambiq_mikrobus_i2c,mikroe_eeram_33v_click``)
* ``samples/drivers/i2c/target_eeprom``
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
* ``samples/subsys/usb/mass``
* ``samples/synchronization``
* ``tests/arch/arm/arm_irq_vector_table``
* ``tests/benchmarks/mbedtls``
* ``tests/boards/ambiq/aes_hal_example``
* ``tests/crypto/mbedtls_psa``
* ``tests/crypto/secp256r1`` (``-- -DEXTRA_CONF_FILE=p256-m_raw.conf``)
* ``tests/drivers/adc/adc_api``
* ``tests/drivers/audio/dmic_api``
* ``tests/drivers/counter/counter_basic_api``
* ``tests/drivers/disk/disk_performance``
* ``tests/drivers/entropy/api``
* ``tests/drivers/flash/common``
* ``tests/drivers/gpio/gpio_basic_api``
* ``tests/drivers/hwinfo/api``
* ``tests/drivers/mspi/api`` (``-T drivers.mspi.api.ambiq``)
* ``tests/drivers/pwm/pwm_api``
* ``tests/drivers/retained_mem/api``
* ``tests/drivers/rtc/rtc_api``
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
* ``tests/subsys/logging/log_api``
* ``tests/subsys/mem_mgmt/mem_attr_heap``
* ``tests/subsys/sd/mmc``
* ``tests/subsys/settings/retention``
