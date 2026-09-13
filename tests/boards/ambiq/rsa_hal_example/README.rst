Ambiq RSA HAL Example
#####################

Overview
********

Exercises the Ambiq CC312 RSA modular exponentiation driver with a 256-bit
known vector (42^65537 mod n).

CC312 and Cortex-M55 cache coherency
************************************

Enable ``CONFIG_NOCACHE_MEMORY`` and use 32-byte aligned ``__nocache`` buffers on
Apollo510 (Cortex-M55), or flush/invalidate cached buffers around each operation.

Building and Running
********************

.. code-block:: console

   west build -b apollo510_evb zephyr/tests/boards/ambiq/rsa_hal_example
