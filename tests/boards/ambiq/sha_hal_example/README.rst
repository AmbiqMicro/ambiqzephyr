Ambiq SHA HAL Example
#####################

Overview
********

This test mirrors the ``aes_hal_example`` style but runs through the Zephyr
Ambiq SHA crypto driver (``CONFIG_CRYPTO_AMBIQ_SHA``).

The test runs known-answer and streaming checks through the ``hash_*`` APIs
for both digests the CC312 engine implements:

- SHA-1: empty message, ``abc``, the 448-bit message, the same message fed in
  successive ``hash_update()`` calls, and a long pattern
- SHA-256: the same five cases

A final check confirms that ``hash_begin_session()`` rejects SHA-512 with
``-ENOTSUP``, since the engine implements only SHA-1 and SHA-256.

Requirements
************

- Ambiq board with the ``ambiq,crypto-sha`` devicetree node enabled.

All CC312 engines share one interrupt, and the SHA HAL polls for DMA
completion, so the board overlays disable the AES and ChaCha nodes. A sibling
driver that connects an ISR would consume the status the poll waits on.

Building and Running
********************

.. code-block:: console

   west build -b apollo510_evb zephyr/tests/boards/ambiq/sha_hal_example

On boot the app prints pass/fail per check and a final count.
