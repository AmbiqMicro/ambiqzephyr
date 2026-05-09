:orphan:

.. _crypto_samples_guide:

Entropy (TRNG) and Hardware CRC Samples and Tests
###################################################

Overview
========

This guide covers hardware entropy (TRNG) and hardware CRC samples and tests
for Ambiq Apollo SoCs.

The Apollo510 SECURITY engine provides:

- **TRNG** - True Random Number Generator exposed via the Zephyr ``entropy`` API
- **HW CRC32** - Hardware-accelerated CRC-32 IEEE exposed via the Zephyr ``crc``
  driver API (``ambiq,hw-crc32`` compatible)

.. note::

   Hardware crypto acceleration (AES, SHA) is planned for a future release.
   Software crypto via mbedTLS is available on all boards.

Unless stated otherwise, build commands are run from the Zephyr root directory.
Replace ``your_board_here`` with your target board name.


Hardware Entropy / TRNG
========================

entropy_api
-----------

Reads random bytes from the True Random Number Generator via the Zephyr
``entropy`` API and verifies basic entropy properties (non-zero output,
successive calls differ).

**Path**: ``tests/drivers/entropy/api``

**Boards**: All

.. code-block:: bash

   west build -p always -b your_board_here tests/drivers/entropy/api
   west flash

Expected console output:

.. code-block:: none

   entropy_api test passed


Hardware CRC
============

crc (HW CRC driver test)
-------------------------

Runs the Zephyr CRC driver API conformance test suite against the Ambiq
``ambiq,hw-crc32`` hardware engine. Tests CRC-32 IEEE computation, thread
safety (concurrent begin/update/finish calls), and error handling.

**Path**: ``tests/drivers/crc``

**Boards**: ``apollo510_evb``, ``apollo510_eb`` (Apollo510 SECURITY engine only)

Enable hardware CRC in your project configuration:

.. code-block:: kconfig

   CONFIG_CRC_AMBIQ=y

Build and run:

.. code-block:: bash

   west build -p always -b your_board_here tests/drivers/crc
   west flash

Expected console output:

.. code-block:: none

   Running ZTEST suite crc_driver_api
   PASS - test_crc32_hw
   PASS - test_crc32_threadsafe
   PROJECT EXECUTION SUCCESSFUL
