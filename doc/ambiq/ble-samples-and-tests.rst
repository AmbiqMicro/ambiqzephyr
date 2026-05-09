:orphan:

.. _ble_samples_guide:

BLE Samples and Tests
#####################

Overview
========

This guide covers building and running Bluetooth Low Energy (BLE) samples on
Ambiq Apollo SoCs. Bluetooth support is available on boards that include a BLE
radio (Apollo3 Blue, Apollo3P Blue, Apollo4 Blue, Apollo4P Blue, Apollo4P Blue
KXR, Apollo510B). Apollo510 (non-B) boards communicate with an external BLE
controller.

.. note::

   For boards that use an external BLE controller, or for SoCs/controllers
   that require radio-core/controller firmware provisioning, flash the BLE
   controller firmware once using SEGGER J-Link and the programming scripts
   provided in the AmbiqSuite SDK before running BLE samples. Boards with an
   integrated BLE radio, such as ``apollo510b_evb``, do not require this
   external controller firmware flashing step. Obtain the SDK from the
   `Ambiq Content Portal <https://contentportal.ambiq.com/>`_.

Board Support
=============

- ``apollo510b_evb`` (integrated BLE radio)
- ``apollo510_evb`` / ``apollo510_eb`` (external BLE controller via HCI UART)
- ``apollo4p_blue_evb`` / ``apollo4p_blue_kxr_evb``
- ``apollo4l_blue_evb``
- ``apollo3p_evb`` / ``apollo3_evb`` (Blue variants)

Prerequisites
=============

1. Flash the BLE controller firmware once via J-Link (see AmbiqSuite SDK docs).
2. Ensure ``CONFIG_BT=y`` is selected (enabled by default on Blue boards).
3. For Apollo510 (non-B) the HCI transport is UART-based; confirm the HCI UART
   pins are correctly assigned in devicetree. For ``apollo510b_evb`` the radio
   is integrated and no external firmware flashing step is required.


Basic Connectivity Samples
==========================

peripheral
----------

Advertises as a connectable peripheral. Useful as a first smoke-test for the
BLE radio.

**Path**: ``samples/bluetooth/peripheral``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/peripheral

central
-------

Scans for and connects to a peripheral device. Pair with the ``peripheral``
sample on a second board to verify two-way connectivity.

**Path**: ``samples/bluetooth/central``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/central

observer
--------

Passive scanner that prints advertising reports to the console. No connection
is established.

**Path**: ``samples/bluetooth/observer``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/observer

beacon / iBeacon / Eddystone
-----------------------------

Non-connectable advertisers that broadcast fixed payloads.

**Paths**:

- ``samples/bluetooth/beacon``
- ``samples/bluetooth/ibeacon``
- ``samples/bluetooth/eddystone``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/beacon

scan_adv
--------

Combines scanning and advertising. Useful for verifying simultaneous TX/RX.

**Path**: ``samples/bluetooth/scan_adv``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/scan_adv


GATT Profile Samples
====================

peripheral_hr
-------------

Heart Rate Profile peripheral. Streams simulated heart rate measurements. Use a
phone with nRF Connect (or similar) to subscribe and verify notifications.

**Path**: ``samples/bluetooth/peripheral_hr``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/peripheral_hr

central_hr
----------

Heart Rate Profile central. Pairs with ``peripheral_hr`` to receive and print
HR notifications.

**Path**: ``samples/bluetooth/central_hr``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/central_hr

peripheral_nus
--------------

Nordic UART Service (NUS) peripheral. Exposes a simple bidirectional serial
tunnel over BLE. Use nRF Connect app or a NUS-compatible client to send and
receive data.

**Path**: ``samples/bluetooth/peripheral_nus``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/peripheral_nus

peripheral_hids
---------------

HID-over-GATT device (keyboard). Pairs with a PC or phone and injects
keystrokes.

**Path**: ``samples/bluetooth/peripheral_hids``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/peripheral_hids


OTA / Firmware Update
=====================

peripheral_amota
----------------

Implements Ambiq's AMOTA (Ambiq Multi-image OTA) service. Use the AmbiqSuite
AMOTA test tool to perform an over-the-air firmware update.

**Path**: ``samples/bluetooth/peripheral_amota``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/peripheral_amota

For DFU over BLE using MCUboot + MCUmgr SMP, see
``doc/ambiq/mcuboot-samples-and-tests.rst``.


Mesh
====

mesh
----

Runs the Zephyr Bluetooth Mesh stack with a sample provisioner and node. Requires
at least two boards for provisioning, or a mesh provisioner app on a phone.

**Path**: ``samples/bluetooth/mesh``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/mesh

mesh_demo
---------

Pre-provisioned mesh demo with LED control. One board acts as a server, another
as a client.

**Path**: ``samples/bluetooth/mesh_demo``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/mesh_demo


Extended / Periodic Advertising
================================

extended_adv / periodic_adv
----------------------------

Exercises the BLE 5.0 extended and periodic advertising features.

**Paths**:

- ``samples/bluetooth/extended_adv``
- ``samples/bluetooth/periodic_adv``
- ``samples/bluetooth/periodic_sync``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/extended_adv
   west build -p always -b your_board_here samples/bluetooth/periodic_adv


ISO / Audio (BLE 5.2)
=====================

iso_broadcast / iso_central / iso_peripheral
---------------------------------------------

BLE LE Audio isochronous channel samples. Require BLE 5.2-capable controller
firmware.

**Paths**:

- ``samples/bluetooth/iso_broadcast``
- ``samples/bluetooth/iso_central``
- ``samples/bluetooth/iso_peripheral``
- ``samples/bluetooth/iso_receive``

.. code-block:: bash

   west build -p always -b your_board_here samples/bluetooth/iso_broadcast


Notes
=====

- All BLE samples default to ``CONFIG_BT_DEVICE_NAME="Zephyr"``; override in
  ``prj.conf`` as needed.
- Use ``CONFIG_BT_LOG_LEVEL_DBG=y`` to enable verbose HCI tracing.
- For Apollo510 with external BLE, ensure ``CONFIG_BT_HCI_VS=y`` and that the
  HCI UART baud rate matches the controller firmware setting (default 115200).
