.. _sample_eeram_click:

EERAM 3.3V Click (MIKROE-2728) Sample
######################################

Overview
********

This sample demonstrates read/write access to the Microchip 47L16 2KB
I2C SRAM+EEPROM backup device on the MikroElektronika `EERAM 3.3V Click`_
board (MIKROE-2728) via the :ref:`mikroe_eeram_33v_click` shield.

The 47L16 provides 2048 bytes of SRAM accessible via I2C. SRAM writes are
immediate with no write cycle delays. The device automatically backs up
SRAM contents to internal EEPROM on power loss, preserving data across
resets.

It uses the Zephyr :ref:`eeprom_api` to:

1. Write a test pattern to the SRAM.
2. Read back the data.
3. Verify byte-for-byte correctness.

Requirements
************

* A board with a mikroBUS |trade| socket that exposes a ``mikrobus_i2c``
  node label.
* The :ref:`mikroe_eeram_33v_click` shield.
* ``CONFIG_EEPROM_MICROCHIP_47L16=y`` (selected automatically via ``prj.conf``).

Building and Running
********************

Build with the shield for your target board, for example:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/eeram_click
   :board: apollo510_evb
   :shield: ambiq_mikrobus_i2c, mikroe_eeram_33v_click
   :goals: build flash
   :compact:

Sample Output
*************

.. code-block:: console

   EERAM 3.3V Click sample - Microchip 47L16
   ==========================================
   EERAM size (from dt): 2048 bytes

   --- EEPROM Driver Test ---
   Writing 32 bytes to EERAM...
   Reading back 32 bytes...
   EEPROM driver test PASSED
   ------------------------------

.. _EERAM 3.3V Click:
   https://www.mikroe.com/eeram-3-3v-click
