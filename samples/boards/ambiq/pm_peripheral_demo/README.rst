.. zephyr:code-sample:: pm_peripheral_demo
   :name: PM Peripheral Demo
   :relevant-api: bluetooth flash_area_api spi_interface i2c_interface crc_interface

   Demonstrates PM-aware use of BLE, Flash, CRC32, AES, SPI, and I2C on
   the apollo330mP_evb board.

Overview
********

This sample exercises multiple peripherals on the Apollo330P SoC together
with the Zephyr Power Management (PM) subsystem.  Three ``suspend-to-idle``
CPU power states are defined in the board overlay; the PM subsystem
automatically selects the deepest state whose minimum residency requirement
is met whenever the system is idle between demo iterations.

Peripherals demonstrated
========================

+----------+-------+----------------------------------------------------------+
| Subsystem| Node  | Notes                                                    |
+==========+=======+==========================================================+
| Entropy  | trng  | 16 random bytes from the hardware TRNG                   |
|          |       | (``zephyr,entropy = &trng`` in board chosen).            |
+----------+-------+----------------------------------------------------------+
| BLE      | IPC   | Connectable peripheral advertising as "Ambiq PM Demo".   |
|          |       | No board overlay needed — transport is fully IPC-based.  |
+----------+-------+----------------------------------------------------------+
| Flash    | flash0| Erase / write / read on ``storage_partition``            |
|          |       | (offset 0x148000, 32 KB).                                |
+----------+-------+----------------------------------------------------------+
| CRC32    | crc32 | Hardware CRC32-IEEE digest (ambiq,hw-crc32) computed     |
|          |       | over the flash read-back buffer.                         |
+----------+-------+----------------------------------------------------------+
| AES      |cc312  | Hardware AES-128-ECB encrypt + decrypt via ARM CC312     |
|          |_aes   | AES accelerator (Ambiq HAL CC312 crypto driver).         |
+----------+-------+----------------------------------------------------------+
| SPI      | spi0  | Single ``spi_transceive`` on IOM0.                       |
+----------+-------+----------------------------------------------------------+
| I2C      | i2c1  | Bus scan on IOM1.                                        |
+----------+-------+----------------------------------------------------------+

Power states
============

Three ``suspend-to-idle`` substates are configured in
``boards/apollo330mP_evb.overlay``:

.. code-block:: none

   stop0  substate-id=1  min-residency=300 µs   exit-latency=50 µs
   stop1  substate-id=2  min-residency=700 µs   exit-latency=300 µs
   stop2  substate-id=3  min-residency=1000 µs  exit-latency=400 µs

After each demo iteration the main thread calls ``k_sleep(K_SECONDS(5))``.
During this idle period the PM subsystem enters the deepest available power
state, waking via the system timer.

Requirements
************

* Board: ``apollo330mP_evb``
* No external hardware is required to build and run this sample.
  The SPI transfer will report a warning (no loopback device) and the I2C
  scan will report "no devices found" unless external hardware is attached.

Building and Flashing
*********************

Default build (with console and logging):

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/pm_peripheral_demo
   :board: apollo330mP_evb
   :goals: build flash
   :compact:

BLE low-power build (no console, minimal power consumption):

.. code-block:: console

   west build -b apollo330mP_evb samples/boards/ambiq/pm_peripheral_demo \
     -- -DCONF_FILE=prj_ble_lp.conf \
        -DDTC_OVERLAY_FILE=boards/apollo330mP_evb_ble_lp.overlay
   west flash

Sample Output
*************

.. code-block:: none

   *** Booting Zephyr OS ***
   [00:00:00.003,000] <inf> pm_demo: === PM Peripheral Demo ===
   [00:00:00.003,000] <inf> pm_demo: Board: apollo330mP_evb  SoC: apollo330P
   [00:00:00.800,000] <inf> pm_demo: BLE: advertising as "Ambiq PM Demo"
   [00:00:00.801,000] <inf> pm_demo: --- Iteration 1 ---
   [00:00:00.820,000] <inf> pm_demo: Flash: write/read verified OK (64 bytes)
   [00:00:00.821,000] <inf> pm_demo: CRC32 (hardware): result=0x???????? over 64 bytes
   [00:00:00.822,000] <inf> pm_demo: AES-128-ECB (hw CC312): encrypt matches FIPS-197 vector
   [00:00:00.822,000] <inf> pm_demo: AES-128-ECB (hw CC312): decrypt round-trip verified
   [00:00:00.823,000] <wrn> pm_demo: SPI transceive: -5 (no loopback device connected)
   [00:00:00.980,000] <inf> pm_demo: I2C: no devices found (expected without external hardware)
   [00:00:00.980,000] <inf> pm_demo: Sleeping 5 s — PM may enter suspend-to-idle
   [00:00:05.981,000] <inf> pm_demo: --- Iteration 2 ---
   ...
