Ambiq Poly1305 HAL Example
##########################

Overview
********

Exercises the Ambiq CC312 Poly1305 driver using the RFC 8439 Appendix A.3 vector.

CC312 and Cortex-M55 cache coherency
************************************

Enable ``CONFIG_NOCACHE_MEMORY`` and use 32-byte aligned ``__nocache`` buffers on
Apollo510 (Cortex-M55), or flush/invalidate cached buffers around each operation.

Board overlays disable sibling CC312 nodes (shared IRQ 24).

Building and Running
********************

.. code-block:: console

   west build -b apollo510_evb zephyr/tests/boards/ambiq/poly1305_hal_example
