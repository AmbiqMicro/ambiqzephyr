.. zephyr:board:: apollo3p_evb

Apollo3 Blue Plus EVB is a board by Ambiq featuring their ultra-low power Apollo3 Blue Plus SoC.

Hardware
********

- Apollo3 Blue Plus SoC with up to 96 MHz operating frequency
- ARM® Cortex®-M4F core
- 16 kB 2-way Associative/Direct-Mapped Cache per core
- Up to 2 MB of flash memory for code/data
- Up to 768 KB of low leakage / low power RAM for code/data
- Integrated Bluetooth 5 Low-energy controller

For more information about the Apollo3 Blue Plus SoC and Apollo3 Blue Plus EVB board:

- `Apollo3 Blue Plus Website`_
- `Apollo3 Blue Plus Datasheet`_
- `Apollo3 Blue Plus EVB Website`_

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
   :board: apollo3p_evb
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

   Hello World! apollo3p_evb

Building other samples
----------------------

The samples and tests below are built for this board as part of the Ambiq SDK
release test. Paths are relative to the workspace root.

* ``samples/basic/blinky_pwm``
* ``samples/basic/button``
* ``samples/basic/threads``
* ``samples/cpp/hello_world``
* ``samples/drivers/adc/adc_dt``
* ``samples/drivers/counter/alarm``
* ``samples/drivers/led/pwm``
* ``samples/drivers/memc``
* ``samples/drivers/mspi/mspi_async``
* ``samples/drivers/mspi/mspi_flash``
* ``samples/drivers/rtc``
* ``samples/drivers/watchdog``
* ``samples/hello_world``
* ``samples/philosophers``
* ``samples/subsys/logging/logger``
* ``samples/synchronization``
* ``tests/drivers/adc/adc_api``
* ``tests/drivers/counter/counter_basic_api``
* ``tests/drivers/gpio/gpio_basic_api``
* ``tests/drivers/hwinfo/api``
* ``tests/drivers/i2c/i2c_api``
* ``tests/drivers/mspi/api`` (``-T drivers.mspi.api.ambiq``)
* ``tests/drivers/mspi/flash``
* ``tests/drivers/pwm/pwm_api``
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
* ``tests/subsys/logging/log_api``

.. _Apollo3 Blue Plus Website:
   https://ambiq.com/apollo3-blue-plus/

.. _Apollo3 Blue Plus Datasheet:
   https://contentportal.ambiq.com/documents/20123/388390/Apollo3-Blue-Plus-SoC-Datasheet.pdf

.. _Apollo3 Blue Plus EVB Website:
   https://www.ambiq.top/en/apollo3-blue-plus-soc-eval-board

.. _SEGGER J-Link software:
   https://www.segger.com/downloads/jlink

.. _pylink:
   https://github.com/Square/pylink
