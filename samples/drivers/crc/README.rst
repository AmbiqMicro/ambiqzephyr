.. _crc-sample:

CRC Sample
##########

Overview
********

This sample demonstrates how to use the CRC (Cyclic Redundancy Check) driver API.
The sample calculates CRC32 checksums over test data and displays the results.

Building and Running
********************

This sample can be found under :zephyr_file:`samples/drivers/crc` in the Zephyr tree.

The sample will work on any board that has a CRC driver enabled and configured.
For Ambiq Apollo4P and Apollo510 boards, the sample uses the hardware CRC32 module.

Supported boards:
- apollo4p_evb
- apollo4p_blue_kxr_evb
- apollo510_evb
- apollo510_eb

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/crc
   :board: apollo4p_evb
   :goals: build flash
   :compact:

Sample Output
=============

.. code-block:: console

    *** Booting Zephyr OS build v3.x.x ***
    [00:00:00.000,000] <inf> crc_sample: CRC Sample starting...
    [00:00:00.000,000] <inf> crc_sample: Testing CRC device: crc32@40030030
    [00:00:00.001,000] <inf> crc_sample: CRC32 of 256 bytes: 0x12345678
    [00:00:00.002,000] <inf> crc_sample: CRC test completed successfully
