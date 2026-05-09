:orphan:

.. _display_samples_guide:

Display, LVGL and Input Samples
################################

Overview
========

This guide covers display, graphics (LVGL) and touch-input samples on Ambiq
Apollo510. Multiple display interfaces are supported via shield overlays:

.. list-table::
   :header-rows: 1

   * - Shield
     - Interface
     - Board
   * - ``ap510_disp``
     - MIPI-DSI
     - ``apollo510_evb`` / ``apollo510_eb``
   * - ``ap510_jdi_disp``
     - JDI
     - ``apollo510_evb`` / ``apollo510_eb``
   * - ``display_8080_card``
     - MIPI-DBI
     - ``apollo510_eb`` only
   * - ``ap4_evb_disp_shield_rev2``
     - SPI/parallel
     - ``apollo4p_evb``

All display samples are located under ``samples/drivers/display`` or
``samples/modules/lvgl``. The ``--shield`` flag selects the active panel.


Display Samples
===============

display
-------

Draws simple geometric shapes and color bars to verify panel initialization.

**Path**: ``samples/drivers/display``

.. code-block:: bash

   # MIPI-DSI panel (CO5300)
   west build -p always -b your_board_here --shield ap510_disp \
       samples/drivers/display

   # JDI panel (LS014B7DD01)
   west build -p always -b your_board_here --shield ap510_jdi_disp \
       samples/drivers/display

   # MIPI-DBI 8080-bus display card
   west build -p always -b your_board_here --shield display_8080_card \
       samples/drivers/display

Expected output on the console:

.. code-block:: none

   Display sample for <driver-name>
   x_res=<width>, y_res=<height>, ...

display — Apollo4P EVB
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   west build -p always -b your_board_here --shield ap4_evb_disp_shield_rev2 \
       samples/drivers/display


LVGL Samples
============

LVGL demos
----------

Runs the upstream LVGL demonstration scenes (music player, benchmark, stress
test, widgets). Select the active demo at build time with the ``-T`` testcase
flag or via ``CONFIG_LV_Z_DEMO_*`` Kconfig options.

**Path**: ``samples/modules/lvgl/demos``

.. code-block:: bash

   # Widget demo on MIPI-DSI panel
   west build -p always -b your_board_here --shield ap510_disp \
       samples/modules/lvgl/demos

   # Override to benchmark demo
   west build -p always -b your_board_here --shield ap510_disp \
       samples/modules/lvgl/demos -- -DCONFIG_LV_Z_DEMO_BENCHMARK=y

.. note::

   Enable ``CONFIG_LV_Z_FULL_REFRESH=y`` and/or increase
   ``CONFIG_LV_Z_VDB_SIZE`` to improve frame rate on high-resolution panels.

LVGL — JDI panel
~~~~~~~~~~~~~~~~

.. code-block:: bash

   west build -p always -b your_board_here --shield ap510_jdi_disp \
       samples/modules/lvgl/demos


Input Samples
=============

draw_touch_events
-----------------

Reads touch events from the panel touch controller and draws circles at each
touch point. Useful to verify the touch interrupt line and I2C address are
correctly configured.

**Path**: ``samples/subsys/input/draw_touch_events``

**Board**: ``apollo510_evb`` with ``ap510_disp`` shield only (panel has an
integrated touch controller).

.. code-block:: bash

   west build -p always -b your_board_here --shield ap510_disp \
       samples/subsys/input/draw_touch_events

Expected behaviour: a colored dot appears at the position of each finger on
the screen and fades out when lifted.


Tips and Troubleshooting
========================

Blank display
-------------

- Confirm the shield overlay is passed (``--shield``); without it no display
  node is enabled.
- Check panel reset GPIO and backlight GPIO are correctly mapped in the overlay.
- Enable debug log level for the display driver to see initialization steps.

Low frame rate
--------------

- Increase ``CONFIG_LV_Z_VDB_SIZE`` (percentage of display buffer).
- Enable ``CONFIG_LV_Z_FULL_REFRESH=y`` for panels that support full-frame DMA.
- Set ``CONFIG_DISPLAY_LOG_LEVEL_ERR=y`` in production to eliminate log overhead.

Touch not responding
--------------------

- Verify the I2C bus used by the touch controller is enabled in the devicetree.
- Check the touch interrupt GPIO line and confirm a pull-up is present.
- Use ``samples/drivers/eeprom`` (I2C loopback) to confirm the bus is working
  before debugging touch.
- There is a new touch driver that will fix any of these issues
  * To be Implemented
