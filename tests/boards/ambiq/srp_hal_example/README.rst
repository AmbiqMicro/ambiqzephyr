Ambiq SRP HAL Example
#####################

Overview
********

Exercises the Ambiq CC312 SRP-6a driver using RFC 5054 1024-bit group parameters
with SHA-256. Tests ``derive_k`` (multiplier) and ``derive_verifier`` (password
verifier) via ``crypto_ambiq_srp.h``.

CC312 and Cortex-M55 cache coherency
************************************

Enable ``CONFIG_NOCACHE_MEMORY`` and use 32-byte aligned ``__nocache`` buffers on
Apollo510 (Cortex-M55), or flush/invalidate cached buffers around each operation.
All CC312 engines share IRQ 24; board overlays disable sibling crypto nodes.

Building and Running
********************

.. code-block:: console

   west build -b apollo510_evb zephyr/tests/boards/ambiq/srp_hal_example
