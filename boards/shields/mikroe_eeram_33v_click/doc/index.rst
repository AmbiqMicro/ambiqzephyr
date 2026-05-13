.. _mikroe_eeram_33v_click:

MikroElektronika EERAM 3.3V Click
###################################

Overview
********

The MikroElektronika `EERAM 3.3V Click`_ features the `Microchip 47L16`_
16Kbit (2KB) I2C SRAM with integrated EEPROM backup in a `mikroBUS`_
|trade| form factor. The device preserves SRAM contents across power loss
events via an onboard capacitor-backed automatic or manual backup cycle.
Backup restore after power-on takes approximately 25 ms.

The primary SRAM port is accessible at I2C address ``0x50`` (A2:A1:A0 all
low as wired on the Click board). A second I2C address ``0x18`` provides
access to the control and status registers.

Requirements
************

This shield can only be used with a board that provides a mikroBUS |trade|
socket and defines a ``mikrobus_i2c`` node label for the mikroBUS |trade| I2C
interface (see :ref:`shields` for more details).

For Ambiq boards, use the :ref:`ambiq_mikrobus_i2c` adapter shield to provide
the required ``mikrobus_i2c`` alias.

Programming
***********

Set ``--shield mikroe_eeram_33v_click`` when you invoke ``west build``. For
boards requiring an I2C adapter, specify both shields. For example:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/eeram_click
   :board: apollo510_evb
   :shield: ambiq_mikrobus_i2c mikroe_eeram_33v_click
   :goals: build

Or with the generic EEPROM sample:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/eeprom
   :board: <your_board>
   :shield: mikroe_eeram_33v_click
   :goals: build

.. _EERAM 3.3V Click:
   https://www.mikroe.com/eeram-3-3v-click

.. _Microchip 47L16:
   https://ww1.microchip.com/downloads/en/DeviceDoc/47L04_47C04_47L16_47C16_DS20005371B.pdf

.. _mikroBUS:
   https://www.mikroe.com/mikrobus
