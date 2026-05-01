.. zephyr:code-sample:: apollo3_mspi_pair
   :name: Apollo3 MSPI controller / IOS peripheral pair
   :relevant-api: mspi_interface spi_interface

   Exercise the Apollo3 MSPI controller driver against an Apollo3 IOS SPI
   peripheral running on a second EVB.

Overview
********

The Apollo3 MSPI block is a controller-only peripheral; it can drive external
memories and other SPI followers, but it cannot itself act as a SPI follower.
To validate the ``ambiq,mspi-controller`` driver on an apollo3_evb without an
external memory chip, this sample pairs two boards:

* **controller/** runs on the EVB driving MSPI0 (single-bit, single-rate, i.e.
  classic 4-wire SPI). It writes a known pattern to the peer at LRAM offset 0
  using the Ambiq IOS register protocol (1-byte command = R/W flag in bit 7
  plus a 7-bit offset), then reads the same offset back and verifies the
  response.

* **peripheral/** runs on the second EVB. The IOS block is configured as a
  SPI peripheral via the Zephyr ``ambiq,spid`` driver. Each cycle the
  application echoes back the one's complement of the bytes that the
  controller wrote, so the controller can prove the bus is working
  end-to-end (rather than reading back stuck data).

Wiring
******

============== ================== ====================
Signal         Controller (EVB-A) Peripheral (EVB-B)
============== ================== ====================
SCK            P24 ``MSPI0_SCK``  P0  ``SLSCK``
MOSI / D0      P22 ``MSPI0_0``    P1  ``SLMOSI``
MISO / D1      P26 ``MSPI0_1``    P2  ``SLMISO``
CE             P19 ``NCE19``      P3  ``SLNCE``
INT            P28 (input)        P4  ``SLINT``
GND            GND                GND
============== ================== ====================

Building and running
********************

Flash the peripheral board first so it is waiting when the controller starts:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/apollo3_mspi_pair/peripheral
   :board: apollo3_evb
   :goals: build flash

Then flash the controller:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/apollo3_mspi_pair/controller
   :board: apollo3_evb
   :goals: build flash

Open a serial console on each EVB. The controller prints
``apollo3_mspi_pair_controller: PASS`` when the bus exchange succeeds.

Single-board self-test
**********************

If only one EVB is available, see ``samples/boards/ambiq/apollo3_mspi_loopback``
for a single-board MSPI driver smoke test that exercises the same driver
without requiring a second board.
