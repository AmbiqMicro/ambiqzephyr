MSPI API Documentation
======================

Introduction
------------
Zephyr MSPI API doc: `Multi-bit SPI Bus — Zephyr Project Documentation <https://docs.zephyrproject.org/latest/hardware/peripherals/mspi.html>`_

The MSPI device drivers are split into two parts:

- **Controller drivers**: implement the MSPI API
- **Peripheral device drivers**: use the MSPI API

Board support:

- ``apollo510_evb``
- ``apollo4p_evb``
- ``apollo3p_evb``
- etc.

Examples Overview
-----------------
The following examples may be used to check whether the device and controller are initialized properly.

MEMC Example
~~~~~~~~~~~~
- Performs XIP write and read from the target device and checks if data matches.
- **Note:** This example may run for a long time as it attempts to write the entire region defined by the ``size`` property.

**Build Command:**

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/memc

**Recommendation:**

Enable ``CONFIG_LOG_MODE_IMMEDIATE=y`` to see better log printing.

FLASH Example
~~~~~~~~~~~~~
- Performs only 4-byte write and read to a specific flash address.

**Build Command:**

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/mspi/mspi_flash

MSPI API Test
~~~~~~~~~~~~~
- Performs basic MSPI controller driver checking.

**Build Command:**

.. code-block:: bash

   west build -p always -b your_board_here tests/drivers/mspi/api

MSPI FLASH Test
~~~~~~~~~~~~~~~
- Performs single-sector and multiple-sector write and read on the target flash device for 3000 bytes.

**Build Command:**

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/mspi/mspi_flash

MSPI Timing Scan Example
~~~~~~~~~~~~~~~~~~~~~~~~
- Performs timing scan for either PSRAM or Flash memory devices.

**Build Commands:**

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/mspi/mspi_timing_scan -T sample.drivers.mspi.timing_scan.memc

   west build -p always -b your_board_here samples/drivers/mspi/mspi_timing_scan -T sample.drivers.mspi.timing_scan.flash

**Note:**

If you are using a different variant of the APS Z8 PSRAM than the one listed, you need to manually change the ``extra_configs`` in ``sample.yaml``.

Board Notes
-----------

1. The apollo510dL_evb supports MSPI hex psram device only by default and flash devices are disabled in the DTS.
   Board rework is required to run octal psram and octal flash in parallel as the MSPI hardware interface is limited.

2. The CELatency is forced to 1 for apollo510L SoCs temporarily and will be updated in the future so that it is configurable per device in the DTS.
