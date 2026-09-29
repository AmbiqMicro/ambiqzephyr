.. zephyr:code-sample:: ambiq-i2s-loopback
   :name: Ambiq I2S Loopback
   :relevant-api: i2s_interface

   Verify I2S0 master TX to I2S1 slave RX digital loopback on Ambiq boards.

Overview
********

This Ambiq board sample streams a sequential pattern out of I2S0 (master TX)
and verifies the same pattern on I2S1 (slave RX). It is intended for Ambiq
I2S driver bring-up and does not modify the upstream Zephyr ``dmic_i2s``
sample.

After the first START, the sample queues another TX block before blocking on
RX so the Ambiq DMA path does not underrun.

Requirements
************

- Apollo510B EVB Rev 2.0 (or equivalent with the overlay pins available)
- Three jumper wires for digital loopback
- EM9305 EXTREF ready so SYSPLL can lock (board default)

Wiring (Apollo510B EVB)
***********************

.. list-table::
   :header-rows: 1

   * - I2S0 (TX / master)
     - Signal
     - I2S1 (RX / slave)
   * - GPIO5
     - CLK
     - GPIO16
   * - GPIO7
     - WS
     - GPIO10
   * - GPIO6
     - DATA
     - GPIO17

Do not use GPIO18 for I2S1 WS on this EVB; that pad is on the DBG_UART
flow-control path.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/i2s_loopback
   :board: apollo510b_evb
   :goals: build flash
   :compact:

Sample Output
*************

.. code-block:: console

   <inf> apollo510b_evb: EM9305 ready for EXTREF
   *** Booting Zephyr OS build ... ***
   <inf> ambiq_i2s_loopback: Ambiq I2S loopback sample
   <inf> ambiq_i2s_loopback: rate=16000 Hz channels=2 block=1280
   <inf> ambiq_i2s_loopback: Start streams
   <inf> ambiq_i2s_loopback: 25600 bytes passed
