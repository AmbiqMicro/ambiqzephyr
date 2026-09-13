Ambiq X25519 HAL Example
########################

Overview
********

Exercises the Ambiq CC312 X25519 driver using RFC 7748 section 5.2 vector 1.

CC312 and Cortex-M55 cache coherency
************************************

Enable ``CONFIG_NOCACHE_MEMORY`` and use 32-byte aligned ``__nocache`` buffers on
Apollo510 (Cortex-M55), or flush/invalidate cached buffers around each operation.

Building and Running
********************

.. code-block:: console

   west build -b apollo510_evb zephyr/tests/boards/ambiq/x25519_hal_example
