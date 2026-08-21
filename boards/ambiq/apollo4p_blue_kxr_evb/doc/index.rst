.. zephyr:board:: apollo4p_blue_kxr_evb

Apollo4 Blue Plus KXR EVB is a board by Ambiq featuring their ultra-low power Apollo4 Blue Plus SoC.

Hardware
********

- Apollo4 Blue Plus SoC with upto 192 MHz operating frequency
- ARM® Cortex®-M4F core
- 64 kB 2-way Associative/Direct-Mapped Cache per core
- Up to 2 MB of non-volatile memory (NVM) for code/data
- Up to 2.75 MB of low leakage / low power RAM for code/data
- 384 kB Tightly Coupled RAM
- 384 kB Extended RAM
- Bluetooth 5.1 Low Energy

For more information about the Apollo4 Blue Plus SoC and Apollo4 Blue Plus KXR EVB board:

- `Apollo4 Blue Plus Website`_
- `Apollo4 Blue Plus Datasheet`_
- `Apollo4 Blue Plus KXR EVB Website`_

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
   :board: apollo4p_blue_kxr_evb
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

   Hello World! apollo4p_blue_kxr_evb

Building other samples
----------------------

The samples and tests below are built for this board as part of the Ambiq SDK
release test. Paths are relative to the workspace root.

* ``samples/basic/blinky_pwm``
* ``samples/basic/button``
* ``samples/basic/threads``
* ``samples/bluetooth/peripheral``
* ``samples/cpp/hello_world``
* ``samples/drivers/adc/adc_dt``
* ``samples/drivers/counter/alarm``
* ``samples/drivers/led/pwm``
* ``samples/drivers/watchdog``
* ``samples/hello_world``
* ``samples/philosophers``
* ``samples/subsys/logging/logger``
* ``samples/synchronization``
* ``tests/drivers/adc/adc_api``
* ``tests/drivers/counter/counter_basic_api``
* ``tests/drivers/gpio/gpio_basic_api``
* ``tests/drivers/hwinfo/api``
* ``tests/drivers/pwm/pwm_api``
* ``tests/drivers/rtc/rtc_api``
* ``tests/drivers/watchdog/wdt_basic_api``
* ``tests/drivers/watchdog/wdt_basic_api`` (``-T drivers.watchdog.pm``)
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

.. _Apollo4 Blue Plus Website:
   https://ambiq.com/apollo4-blue-plus/

.. _Apollo4 Blue Plus Datasheet:
   https://contentportal.ambiq.com/documents/20123/388410/Apollo4-Blue-Plus-SoC-Datasheet.pdf

.. _Apollo4 Blue Plus KXR EVB Website:
   https://www.ambiq.top/en/apollo4-blue-plus-kxr-soc-eval-board

.. _SEGGER J-Link software:
   https://www.segger.com/downloads/jlink

.. _pylink:
   https://github.com/Square/pylink
