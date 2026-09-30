Ambiq CC312 CMAC-KDF HAL example
###############################

Overview
********

Exercises SP 800-108 CMAC-KDF with label/context domain separation.

Key source (Kconfig choice):

* ``CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_TEST_KEY`` (default) — USER_KEY, no HUK
* ``CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_HUK`` — ``am_hal_cc312_root_kdf_cmac()``

Building and Running
********************

.. code-block:: console

   west build -b apollo510b_evb tests/boards/ambiq/root_kdf_hal_example

   west build -b apollo510b_evb tests/boards/ambiq/root_kdf_hal_example -- \
     -DCONFIG_CC312_ROOT_KDF_EXAMPLE_USE_HUK=y
