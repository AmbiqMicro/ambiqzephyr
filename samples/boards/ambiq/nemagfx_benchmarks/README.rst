.. _nemagfx_benchmarks:

NemaGFX Benchmarks
##################

Overview
********

This sample is a Zephyr port of the AmbiqSuite ``nemagfx_benchmarks`` example.
It exercises the Apollo510 NemaGFX GPU with 36 graphics workloads (shape fill,
line/rect drawing, blits, stretch blits, textured triangles/quads, and string
rendering) and reports throughput on the console.

Each benchmark renders into a 384x384 off-screen framebuffer, pushes the result
to the display, and prints a performance line such as ``MPixels/sec`` or
``KChars/sec``.

On ``ap510_disp`` (468x468), the board overlay sets ``x-offset`` / ``y-offset``
on the CO5300 panel node so the driver maps the 384x384 image to the center of
the panel (same mechanism as ``chipone,co5300`` support added in the display
driver).

Requirements
************

* An Apollo510 EVB (``apollo510_evb``, ``apollo510b_evb``, or ``apollo510dL_evb``)
* The ``ap510_disp`` shield (CO5300 MIPI-DSI panel)

Building and Running
********************

Build for ``apollo510_evb`` with the display shield:

.. code-block:: console

   west build -p always -b apollo510_evb zephyr/samples/boards/ambiq/nemagfx_benchmarks -- -DSHIELD=ap510_disp
   west flash

By default ``CONFIG_TEST=y`` runs one full benchmark pass (all 36 tests) and
exits. Disable it in ``prj.conf`` to loop continuously and sweep GPU burst
length settings like the AmbiqSuite bare-metal example.

Sample Output
*************

.. code-block:: console

   NemaGFX benchmark suite (FB 384x384)
   FB burst: 4
   TEX burst: 4
   Burst size register value: 00000044
   1: Fill_Triangle                  12.34 MPixels/sec
   ...
   NemaGFX benchmarks complete

Reference
*********

AmbiqSuite source:
``AmbiqSuite/boards/apollo510_evb/examples/graphics/nemagfx_benchmarks``
