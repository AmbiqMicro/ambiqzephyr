Ambiq CC312 Root-Key / CMAC-KDF
###############################

Overview
********

Demonstrates NIST SP 800-108 counter-mode CMAC-KDF (label + context domain
separation) with a compile-time key source:

* ``CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_TEST_KEY`` (default) — fixed AES
  **USER_KEY**. No HUK required; use on DM boards for bring-up.
* ``CONFIG_CC312_ROOT_KDF_EXAMPLE_USE_HUK`` — ``am_hal_cc312_root_kdf_cmac()``
  with **RKEK/HUK**. Needs HUK/KDR programmed and usable in the current LCS.

Typical customer flow for a P-256 identity key:

1. Derive KEK with a stable label/context (HUK path in production)
2. Wrap the P-256 private scalar; store the blob in MRAM/NVM
3. Later: re-derive KEK → unwrap → ECDSA sign → zeroize

Building and Running
********************

USER_KEY path (default)::

   west build -b apollo510b_evb samples/boards/ambiq/cc312_root_kdf

HUK/RKEK path::

   west build -b apollo510b_evb samples/boards/ambiq/cc312_root_kdf -- \
     -DCONFIG_CC312_ROOT_KDF_EXAMPLE_USE_HUK=y

Same app under ``tests/boards/ambiq/root_kdf_hal_example`` for twister.
