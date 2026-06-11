Ambiq ChaCha HAL Example
########################

Overview
********

This test mirrors the ``aes_hal_example`` style and directly exercises the
Ambiq CC312 ChaCha20 HAL (``am_hal_cc312_chacha``) on Apollo510 hardware.

The test runs three sub-tests based on the RFC 8439 Section 2.4.2 known-answer
vector:

- **Encrypt KAT** – single-shot encryption compared against the RFC 8439 ciphertext.
- **Decrypt round-trip** – decrypts the RFC ciphertext back to the original plaintext.
- **Split/streaming** – encrypts in two successive calls (64-byte block + remainder)
  to verify block-counter continuation across calls.

Requirements
************

- Ambiq Apollo510 board (``apollo510_evb`` or ``apollo510b_evb``).

Building and Running
********************

.. code-block:: console

   west build -b apollo510_evb zephyr/tests/boards/ambiq/chacha_hal_example

On boot the app prints per-test pass/fail and a final summary line.

Per-test controls
*****************

Each test can be enabled in ``prj.conf``:

- ``CONFIG_CHACHA_HAL_EXAMPLE_TEST_ENCRYPT_KAT``
- ``CONFIG_CHACHA_HAL_EXAMPLE_TEST_DECRYPT_ROUNDTRIP``
- ``CONFIG_CHACHA_HAL_EXAMPLE_TEST_SPLIT_STREAMING``

If a config is not set to ``y`` the sample prints ``SKIPPED`` for that test.
