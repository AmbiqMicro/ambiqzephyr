.. _sample_dram_click:

DRAM Click (MIKROE-5337) PSRAM Sample
######################################

Overview
********

This sample exercises the APS6404L-3SQR 64Mbit PSRAM on the MikroElektronika
`DRAM Click`_ board (MIKROE-5337) via the :ref:`mikroe_dram_click` shield.

It writes a 128-byte sequential pattern to the PSRAM and reads it back,
verifying byte-for-byte correctness. Output is sent to the console.

Requirements
************

* A board with a mikroBUS |trade| socket that exposes a ``mikrobus_spi``
  node label.
* The :ref:`mikroe_dram_click` shield.
* ``CONFIG_APS6404L=y`` (selected automatically via ``prj.conf``).

Building and Running
********************

Build with the shield for your target board, for example:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/dram_click
   :board: apollo510_evb
   :shield: ambiq_mikrobus_spi, mikroe_dram_click
   :goals: build flash
   :compact:

Sample Output
*************

.. code-block:: console

   DRAM Click sample — APS6404L 64Mbit PSRAM
   ==========================================
   APS6404L Write PASSED
   APS6404L Read PASSED
   All 128 bytes verified successfully.

.. _DRAM Click:
   https://www.mikroe.com/dram-click
