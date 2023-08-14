.. _log_backend_swo_example:

Log Backend SWO Example
###########

Overview
********

A simple sample that can be used with any :ref:`supported board <boards>` and
prints to the console every 1ms.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/debug/log_backend_swo_example
   :board: apollo4p_evb
   :goals: run
   :compact:

To build for another board, change "apollo4p_evb" above to that board's name.

Sample Output
=============

.. code-block:: console

*** Booting Zephyr OS build zephyr-v3.4.0-1347-gab68a2182f35 ***

[80597.259185] <dbg> mainentry: main: Backend_swo_example! apollo4p_evb


[80598.263946] <dbg> mainentry: main: i: 0


[80599.268157] <dbg> mainentry: main: i: 1
