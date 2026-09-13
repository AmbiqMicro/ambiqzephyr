Ambiq HKDF HAL Example
######################

Overview
********

Exercises the Zephyr Ambiq HKDF driver using RFC 5869 test case 1 (SHA-256).

CC312 and Cortex-M55 cache coherency
************************************

Enable ``CONFIG_NOCACHE_MEMORY`` and use 32-byte aligned ``__nocache`` buffers on
Apollo510 (Cortex-M55), or flush/invalidate cached buffers around each operation.
All CC312 engines share IRQ 24; board overlays disable sibling crypto nodes.

Building and Running
********************

.. code-block:: console

   west build -b apollo510_evb zephyr/tests/boards/ambiq/hkdf_hal_example
