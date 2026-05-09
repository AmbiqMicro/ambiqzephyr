:orphan:

.. _general_samples_guide:

General Driver Samples and Tests
#################################

Overview
========

This guide covers general-purpose driver samples and tests for Ambiq Apollo
SoCs. Samples marked **All** build for every supported board; those with board
restrictions are noted explicitly.

Audio, I2S and PDM samples are covered in ``doc/ambiq/audio-samples-and-tests.rst``.
Hardware entropy (TRNG) samples are covered in ``doc/ambiq/crypto-samples-and-tests.rst``.

Unless stated otherwise, build commands are run from the Zephyr root directory.
Replace ``your_board_here`` with your target board name.


ADC
===

adc_dt
------

Reads voltage on each ADC channel defined in the devicetree and prints the
result in millivolts. A useful first test for pin-mux and ADC calibration.

**Path**: ``samples/drivers/adc/adc_dt``

**Boards**: All

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/adc/adc_dt

On Apollo510 EVB a second build variant enables the AUDADC:

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/adc/adc_dt \
       -T sample.drivers.adc.adc_dt.audadc

Expected console output:

.. code-block:: none

   ADC reading[0]:
   - adc@..., channel 0: 812 = 1802 mV




I2C
===

eeprom (I2C EEPROM read/write)
-------------------------------

Writes a known pattern to an I2C EEPROM and reads it back to verify data
integrity. A handy test for I2C bus speed and pull-up configuration.

**Path**: ``samples/drivers/eeprom``

**Board**: ``apollo510_eb`` only (EEPROM fitted on board)

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/eeprom

i2c_api (I2C driver test)
--------------------------

Runs the Zephyr I2C API conformance test suite.

**Path**: ``tests/drivers/i2c/i2c_api``

.. code-block:: bash

   west build -p always -b your_board_here tests/drivers/i2c/i2c_api
   west flash


SPI
===

spi_serial_flash
----------------

Performs SPI NOR flash read/write/erase cycles via the Ambiq IOM SPI controller
and prints pass/fail results.

**Path**: ``samples/boards/ambiq/spi_serial_flash``

**Board**: ``apollo510_eb`` only

.. code-block:: bash

   west build -p always -b your_board_here samples/boards/ambiq/spi_serial_flash


Flash
=====

flash_shell
-----------

Provides an interactive shell for raw flash read, write, and erase operations.
Useful for verifying flash driver functionality and partition layout.

**Path**: ``samples/drivers/flash_shell``

**Boards**: All except ``apollo3_evb``

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/flash_shell
   west flash

After flashing, open a serial terminal at 115200 baud and use the ``flash``
shell commands:

.. code-block:: none

   uart:~$ flash erase <device> <offset> <size>
   uart:~$ flash write <device> <offset> <byte> [<byte>...]
   uart:~$ flash read <device> <offset> <len>

.. note::

   For OTA-based flash update workflows (MCUmgr SMP server), see
   ``doc/ambiq/mcuboot-samples-and-tests.rst``.


Counter
=======

alarm
-----

Configures a one-shot counter alarm and prints a message when it fires.
Verifies the Zephyr counter API and the STIMER hardware.

**Path**: ``samples/drivers/counter/alarm``

**Boards**: All

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/counter/alarm

counter_basic_api (test)
------------------------

Runs the Zephyr counter API conformance test suite.

**Path**: ``tests/drivers/counter/counter_basic_api``

.. code-block:: bash

   west build -p always -b your_board_here \
       tests/drivers/counter/counter_basic_api
   west flash


RTC
===

rtc
---

Sets the RTC to a known time, sleeps, then reads back the time to verify the
elapsed interval. Also demonstrates alarm callback usage.

**Path**: ``samples/drivers/rtc``

**Boards**: All

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/rtc

rtc_api (test)
--------------

Runs the Zephyr RTC API conformance test suite.

**Path**: ``tests/drivers/rtc/rtc_api``

.. code-block:: bash

   west build -p always -b your_board_here tests/drivers/rtc/rtc_api
   west flash


PWM
===

pwm (LED PWM)
-------------

Drives an LED with a PWM waveform and cycles through different duty cycles.
Verify output on an oscilloscope or observe LED brightness change.

**Path**: ``samples/drivers/led/pwm``

**Boards**: EVB boards only (LED connected to PWM-capable pin)

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/led/pwm

pwm_api (test)
--------------

Runs the Zephyr PWM API conformance test suite.

**Path**: ``tests/drivers/pwm/pwm_api``

.. code-block:: bash

   west build -p always -b your_board_here tests/drivers/pwm/pwm_api
   west flash


UART
====

echo_bot
--------

Echoes every byte received on the UART back to the sender. Use a serial
terminal to send text and confirm it is returned verbatim.

**Path**: ``samples/drivers/uart/echo_bot``

**Boards**: All

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/uart/echo_bot

uart_basic_api (test)
---------------------

Runs the Zephyr UART API conformance test suite.

**Path**: ``tests/drivers/uart/uart_basic_api``

.. code-block:: bash

   west build -p always -b your_board_here tests/drivers/uart/uart_basic_api
   west flash


HWINFO
======

hwinfo_api
----------

Reads device identity information (chip UID) via the Zephyr HWINFO API and
prints it to the console.

**Path**: ``tests/drivers/hwinfo/api``

**Boards**: All

.. code-block:: bash

   west build -p always -b your_board_here tests/drivers/hwinfo/api
   west flash


Power Management
================

power_wakeup_timer
------------------

Puts the SoC into deep sleep and wakes it via a timer interrupt. Prints the
wakeup reason and elapsed time to verify PM suspend/resume.

**Path**: ``tests/subsys/pm/power_wakeup_timer``

**Boards**: All

.. code-block:: bash

   west build -p always -b your_board_here \
       tests/subsys/pm/power_wakeup_timer

Expected console output:

.. code-block:: none

   Entering deep sleep
   Wake up! Elapsed: 2000 ms

See ``README.rst`` for a full description of PM configuration options.


SDHC / SD Card
==============

mmc (SD card test)
------------------

Mounts an SD card over SDHC and performs read/write/erase verification.
Requires an SD card inserted in the board's SD slot.

**Path**: ``tests/subsys/sd/mmc``

**Boards**: All (hardware SD slot required)

.. code-block:: bash

   west build -p always -b your_board_here tests/subsys/sd/mmc
   west flash

For a filesystem-level test, use:

**Path**: ``samples/subsys/fs/fs_sample``

.. code-block:: bash

   west build -p always -b your_board_here samples/subsys/fs/fs_sample


Watchdog
========

task_wdt
--------

Demonstrates the software task watchdog, which monitors individual threads and
resets the system if any thread stalls.

**Path**: ``samples/subsys/task_wdt``

**Boards**: All

.. code-block:: bash

   west build -p always -b your_board_here samples/subsys/task_wdt

wdt_basic_api (test)
---------------------

Runs the Zephyr watchdog API conformance test suite.

**Path**: ``tests/drivers/watchdog/wdt_basic_api``

.. code-block:: bash

   west build -p always -b your_board_here \
       tests/drivers/watchdog/wdt_basic_api
   west flash


Miscellaneous
=============

philosophers
------------

Classic Dining Philosophers problem. Exercises kernel threading and mutexes;
also validates the timer driver indirectly.

**Path**: ``samples/philosophers``

**Boards**: All

.. code-block:: bash

   west build -p always -b your_board_here samples/philosophers

coremark
--------

Industry-standard CoreMark CPU benchmark. Reports CoreMark/MHz score to the
console. Useful for comparing performance between clock frequencies and
compiler optimisation flags.

**Path**: ``samples/benchmarks/coremark``

**Boards**: All

.. code-block:: bash

   west build -p always -b your_board_here samples/benchmarks/coremark
