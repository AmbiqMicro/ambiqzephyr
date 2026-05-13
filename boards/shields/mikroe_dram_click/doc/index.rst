.. _mikroe_dram_click:

MikroElektronika DRAM Click
############################

Overview
********

The MikroElektronika `DRAM Click`_ features the `APS6404L-3SQR`_ 64Mbit
Pseudo-SRAM (PSRAM) in a `mikroBUS`_ |trade| form factor. The device
provides 8M x 8-bit storage with an SPI/QPI interface and a self-managed
refresh mechanism, making it suitable for low-power, low-cost portable
applications.

Requirements
************

This shield can only be used with a board that provides a mikroBUS |trade|
socket and defines a ``mikrobus_spi`` node label for the mikroBUS |trade| SPI
interface (see :ref:`shields` for more details).

The APS6404L driver must be enabled via :kconfig:option:`CONFIG_APS6404L`.

Programming
***********

Set ``--shield mikroe_dram_click`` when you invoke ``west build``. For
example:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/aps6404l
   :board: <your_board>
   :shield: mikroe_dram_click
   :goals: build

.. _DRAM Click:
   https://www.mikroe.com/dram-click

.. _APS6404L-3SQR:
   https://www.apmemory.com/wp-content/uploads/APM_PSRAM_E3_SPI_APS6404L-3SQR-v2.6-PKG.pdf

.. _mikroBUS:
   https://www.mikroe.com/mikrobus
