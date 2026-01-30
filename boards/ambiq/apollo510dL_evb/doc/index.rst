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

.. _SEGGER J-Link software:
   https://www.segger.com/downloads/jlink

.. _pylink:
   https://github.com/Square/pylink

