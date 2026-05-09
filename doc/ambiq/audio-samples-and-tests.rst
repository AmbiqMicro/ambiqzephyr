:orphan:

.. _audio_samples_guide:

Audio Samples and Tests
########################

Overview
========

This guide covers audio driver samples and tests for Ambiq Apollo SoCs,
including analog microphone capture (AUDADC), PDM digital microphone capture,
and I2S streaming.

.. note::

   Audio samples require Apollo510 or Apollo4P. They are not supported on
   Apollo3 variants.

Unless stated otherwise, build commands are run from the Zephyr root directory.
Substitute ``your_board_here`` with the appropriate supported board for the
sample.


Analog Microphone (AUDADC)
==========================

amic
----

Captures audio from an analog microphone via the AUDADC and streams samples
through the Zephyr audio codec interface. A good first test for AUDADC
pin-mux and gain configuration.

**Path**: ``samples/drivers/audio/amic``

**Boards**: ``apollo510_evb``, ``apollo510_eb``, ``apollo4p_evb``

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/audio/amic

Expected console output:

.. code-block:: none

   AUDADC sample: capturing ...
   Block received, size = <N> bytes


PDM / Digital Microphone
=========================

dmic
----

Captures PDM audio from a digital microphone using the Zephyr DMIC API and
prints buffer statistics such as frame count and overrun events.

**Path**: ``samples/drivers/audio/dmic``

**Boards**: ``apollo510_evb``, ``apollo510_eb``, ``apollo4p_evb``

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/audio/dmic

Expected console output:

.. code-block:: none

   DMIC sample: starting stream
   Received block, size = <N>


I2S
===

echo (I2S loopback)
-------------------

Loops I2S transmit data back through receive and verifies the payload. Requires
an external loopback connection between the I2S TX and RX pins (or an external
codec configured in loopback mode).

**Path**: ``samples/drivers/i2s/echo``

**Boards**: ``apollo510_evb``, ``apollo510_eb``, ``apollo4p_evb``

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/i2s/echo

output (I2S playback)
---------------------

Generates a sine-wave tone and sends it out over I2S. Connect a codec or
logic analyzer to verify the bitstream.

**Path**: ``samples/drivers/i2s/output``

**Boards**: ``apollo510_evb``, ``apollo510_eb``, ``apollo4p_evb``

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/i2s/output

dmic_i2s (PDM capture → I2S output)
-------------------------------------

Captures PDM audio and routes it to I2S output in real-time. Useful for
verifying the full audio pipeline end-to-end.

**Path**: ``samples/drivers/i2s/dmic_i2s``

**Boards**: ``apollo510_evb``, ``apollo510_eb``

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/i2s/dmic_i2s

i2s_codec (I2S with external codec)
-------------------------------------

Streams audio through a connected external audio codec. Only available on
boards with a codec fitted.

**Path**: ``samples/drivers/i2s/i2s_codec``

**Board**: ``apollo510_eb`` only

.. code-block:: bash

   west build -p always -b your_board_here samples/drivers/i2s/i2s_codec

i2s_api (I2S driver conformance test)
--------------------------------------

Runs the Zephyr I2S API conformance test suite against the Ambiq I2S driver.

**Path**: ``tests/drivers/i2s/i2s_api``

**Boards**: ``apollo510_evb``, ``apollo510_eb``

.. code-block:: bash

   west build -p always -b your_board_here tests/drivers/i2s/i2s_api
   west flash
