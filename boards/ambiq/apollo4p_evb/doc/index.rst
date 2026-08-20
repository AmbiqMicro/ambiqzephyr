.. zephyr:board:: apollo4p_evb

Apollo4P EVB is a board by Ambiq featuring their ultra-low power Apollo4 Plus SoC.

Hardware
********

- Apollo4 Plus SoC with upto 192 MHz operating frequency
- ARM® Cortex®-M4F core
- 64 kB 2-way Associative/Direct-Mapped Cache per core
- Up to 2 MB of non-volatile memory (NVM) for code/data
- Up to 2.75 MB of low leakage / low power RAM for code/data
- 384 kB Tightly Coupled RAM
- 384 kB Extended RAM

For more information about the Apollo4 Plus SoC and Apollo4P EVB board:

- `Apollo4 Plus Website`_
- `Apollo4 Plus Datasheet`_
- `Apollo4P EVB Website`_

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
   :board: apollo4p_evb
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

   Hello World! apollo4p_evb

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
* ``samples/drivers/memc`` (``-- -DSHIELD=ap4_evb_disp_shield_rev2``)
* ``samples/drivers/mspi/mspi_flash`` (``-- -DSHIELD=ap4_evb_disp_shield_rev2``)
* ``samples/drivers/mspi/mspi_timing_scan`` (``-- -DSHIELD=ap4_evb_disp_shield_rev2``)
* ``samples/drivers/watchdog``
* ``samples/hello_world``
* ``samples/philosophers``
* ``samples/subsys/fs/fs_sample``
* ``samples/subsys/logging/logger``
* ``samples/synchronization``
* ``tests/drivers/adc/adc_api``
* ``tests/drivers/counter/counter_basic_api``
* ``tests/drivers/disk/disk_performance``
* ``tests/drivers/gpio/gpio_basic_api``
* ``tests/drivers/hwinfo/api``
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
* ``tests/subsys/sd/sdmmc``

Samples marked with a shield need it named on the command line, because
``west build`` does not read ``sample.yaml``:

.. code-block:: console

   west build -p always -b apollo4p_evb <sample path> -- -DSHIELD=ap4_evb_disp_shield_rev2

.. _Apollo4 Plus Website:
   https://ambiq.com/apollo4-plus/

.. _Apollo4 Plus Datasheet:
   https://contentportal.ambiq.com/documents/20123/388415/Apollo4-Plus-SoC-Datasheet.pdf

.. _Apollo4P EVB Website:
   https://www.ambiq.top/en/apollo4-plus-soc-eval-board

.. _SEGGER J-Link software:
   https://www.segger.com/downloads/jlink

.. _pylink:
   https://github.com/Square/pylink
