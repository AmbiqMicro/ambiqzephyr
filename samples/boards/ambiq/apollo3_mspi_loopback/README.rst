.. zephyr:code-sample:: apollo3_mspi_loopback
   :name: Apollo3 MSPI single-board self-test
   :relevant-api: mspi_interface

   Exercise the Apollo3 ``ambiq,mspi-controller`` driver on a single EVB
   without any external MSPI memory chip.

Overview
********

This sample is a smoke test for the Apollo3 MSPI driver. It declares a stub
``ambiq,mspi-device`` child node so the driver has a target to address,
configures MSPI0 in single-bit single-rate mode, and issues a fixed number
of TX transactions. ``PASS`` is printed if every ``mspi_transceive()`` call
returns success.

No external chip is required. With a logic analyzer or oscilloscope on the
following pins, the generated waveforms can be observed directly:

* **MSPI0_D0**  - P22
* **MSPI0_D1**  - P26
* **MSPI0_SCK** - P24
* **NCE19**     - P19

Optional wire-integrity check
=============================

Jumper P22 (D0) to P26 (D1). The MSPI block's single-bit half-duplex
transactions still drive only one direction at a time, so this jumper does
not let the controller "loop back" data into a read transaction; however a
multimeter or scope on P26 will confirm the line is being toggled by P22.

Building and running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/apollo3_mspi_loopback
   :board: apollo3_evb
   :goals: build flash

Expected console output ends with::

    apollo3_mspi_loopback: PASS

Two-board test
**************

For an end-to-end MSPI <-> SPI peripheral test using two EVBs, see
``samples/boards/ambiq/apollo3_mspi_pair``.
