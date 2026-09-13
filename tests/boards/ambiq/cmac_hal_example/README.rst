Ambiq AES-CMAC HAL Example
##########################

Overview
********

Exercises the Zephyr Ambiq AES-CMAC driver using an NIST SP 800-38B test vector.

Enable ``CONFIG_NOCACHE_MEMORY`` and use 32-byte aligned ``__nocache`` buffers on
Cortex-M55 (Apollo510). Board overlays disable sibling CC312 nodes (shared IRQ 24).

Building and Running
********************

.. code-block:: console

   west build -b apollo510_evb zephyr/tests/boards/ambiq/cmac_hal_example
