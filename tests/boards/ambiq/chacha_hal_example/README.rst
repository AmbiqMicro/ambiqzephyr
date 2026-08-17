Ambiq ChaCha HAL Example
########################

Overview
********

This test mirrors the ``aes_hal_example`` style but runs through the Zephyr
Ambiq ChaCha20 crypto driver (``CONFIG_CRYPTO_AMBIQ_CHACHA``).

The key, nonce and initial counter come from RFC 8439 Section 2.4.2, but the
plaintext is a synthetic 65535-byte payload -- the largest single DLLI transfer
the CC312 ChaCha engine accepts (``inDataSize < DLLI_MAX_BUFF_SIZE``). The
expected ciphertext is computed over that payload, so this is a self-generated
known-answer vector rather than the RFC's own answer.

Three sub-tests run against it:

- **Encrypt KAT** – single-shot encryption compared against the expected ciphertext.
- **Decrypt round-trip** – decrypts that ciphertext back to the original plaintext.
- **Split/streaming** – encrypts in two successive calls (64-byte block + remainder)
  to verify block-counter continuation across calls.

Requirements
************

- Ambiq board with the ``ambiq,crypto-chacha`` devicetree node enabled.

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
- ``CONFIG_CHACHA_HAL_EXAMPLE_USE_INPLACE_BUFFERS`` (use in-place buffers when ``y``)

If a config is not set to ``y`` the sample prints ``SKIPPED`` for that test.
