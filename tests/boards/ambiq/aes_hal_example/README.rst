Ambiq AES HAL Example
#####################

Overview
********

This test mimics the AmbiqSuite ``aes_hal_example`` style but runs through
the Zephyr Ambiq AES crypto driver (``CONFIG_CRYPTO_AMBIQ_AES``).

The test runs a small self-test set through ``cipher_*`` APIs for:

- AES-ECB
- AES-CTR
- AES-OFB
- AES-CCM
- AES-GCM
- AES-XTS

Requirements
************

- Ambiq board with ``ambiq,crypto-aes`` devicetree node enabled.

Building and Running
********************

.. code-block:: console

   west build -b apollo510dL_evb zephyr/tests/boards/ambiq/aes_hal_example

On boot, the app prints pass/fail per mode and a final summary.

Per-test controls
*****************

Each test can be enabled in ``prj.conf``:

- ``CONFIG_AES_HAL_EXAMPLE_TEST_ECB``
- ``CONFIG_AES_HAL_EXAMPLE_TEST_CTR``
- ``CONFIG_AES_HAL_EXAMPLE_TEST_OFB``
- ``CONFIG_AES_HAL_EXAMPLE_TEST_CCM``
- ``CONFIG_AES_HAL_EXAMPLE_TEST_GCM``
- ``CONFIG_AES_HAL_EXAMPLE_TEST_XTS``
- ``CONFIG_AES_HAL_EXAMPLE_USE_INPLACE_BUFFERS`` (use in-place buffers when ``y``)

If one of these is not set to ``y``, the sample prints ``SKIPPED`` for that
test instead of ``PASS``/``FAIL``.

For example, to skip CCM temporarily, set:

.. code-block:: none

   # CONFIG_AES_HAL_EXAMPLE_TEST_CCM is not set
