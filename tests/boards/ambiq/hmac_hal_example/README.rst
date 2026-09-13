Ambiq HMAC HAL Example
######################

Overview
********

Exercises the Zephyr Ambiq HMAC driver (``CONFIG_CRYPTO_AMBIQ_HMAC``) using
RFC 4231 test vectors for HMAC-SHA256.

CC312 on Cortex-M55 requires ``CONFIG_NOCACHE_MEMORY`` and 32-byte aligned
``__nocache`` DMA buffers, or explicit cache maintenance. All CC312 engines
share IRQ 24; board overlays disable sibling crypto nodes.

Building and Running
********************

.. code-block:: console

   west build -b apollo510_evb zephyr/tests/boards/ambiq/hmac_hal_example
