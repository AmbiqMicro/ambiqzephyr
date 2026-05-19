.. zephyr:code-sample:: ambiq-pwm
   :name: Ambiq CTimer PWM Output
   :relevant-api: pwm_interface

   Generate PWM outputs using the Ambiq CTimer PWM driver.

Overview
********

This sample demonstrates how to use the Ambiq CTimer-based PWM driver to
generate PWM outputs, following the Ambiq SDK ``ctimer_pwm_output.c`` example.

Three PWM outputs are configured with different duty cycles:

.. list-table::
   :header-rows: 1

   * - PWM
     - Timer
     - Segment
     - GPIO
     - CTIM
     - Duty Cycle
   * - PWM A
     - Timer 0
     - Segment B
     - GPIO 13
     - CTIM2
     - ~10% (32/327)
   * - PWM B
     - Timer 2
     - Segment A
     - GPIO 29
     - CTIM9
     - ~50% (163/327)
   * - PWM C
     - Timer 7
     - Segment B
     - GPIO 11
     - CTIM31
     - ~70% (228/327)

The clock source is the 32.768 kHz XT oscillator with a period of 327 cycles,
resulting in approximately 100 Hz PWM frequency.

Requirements
************

The board must have the Ambiq CTimer PWM driver enabled with the following
PWM nodes: ``pwm0``, ``pwm2``, and ``pwm7``.

Supported boards:

- ``apollo3p_evb``

Building and Running
********************

Build and flash the sample as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ambiq/pwm
   :board: apollo3p_evb
   :goals: build flash
   :compact:

Sample Output
*************

.. code-block:: console

   *****************************************************
   *         Ambiq CTimer PWM Output Example           *
   *****************************************************

   Configuration (matching ctimer_pwm_output.c):
     PWM A: Timer 0, Segment B, GPIO 13 (CTIM2)
     PWM B: Timer 2, Segment A, GPIO 29 (CTIM9)
     PWM C: Timer 7, Segment B, GPIO 11 (CTIM31)
     Clock: 32.768 kHz, Period: 327 cycles (~100 Hz)

   Initializing PWM devices:
     PWM_A (Timer0/GPIO13): pwm@40008000 ready
     PWM_B (Timer2/GPIO29): pwm@40008020 ready
     PWM_C (Timer7/GPIO11): pwm@40008070 ready

   Setting PWM outputs:
   PWM_A (Timer0/GPIO13): period=327 cycles, duty=32 cycles (~10%)
   PWM_B (Timer2/GPIO29): period=327 cycles, duty=163 cycles (~50%)
   PWM_C (Timer7/GPIO11): period=327 cycles, duty=228 cycles (~70%)

   All 3 PWM outputs are now running.
   Measure PWM signals on GPIO 13, GPIO 29, and GPIO 11.

Hardware Configuration
**********************

The PWM outputs are configured via the devicetree overlay. The pins match
the Ambiq SDK ``ctimer_pwm_output.c`` example:

- GPIO 13: PWM A output (~10% duty cycle)
- GPIO 29: PWM B output (~50% duty cycle)
- GPIO 11: PWM C output (~70% duty cycle)

To observe the PWM outputs:

1. Connect an oscilloscope or logic analyzer to GPIO 13, 29, and 11
2. Verify ~100 Hz frequency on all channels
3. Verify duty cycles: ~10%, ~50%, ~70%

Customization
*************

To modify duty cycles, update the ``PWM_x_DUTY_CYCLES`` defines in
``src/main.c``. The values are in clock cycles where:

- Period = 327 cycles
- Duty cycle % = (duty_cycles / 327) * 100

For example:
- 10% duty = 32 cycles
- 50% duty = 163 cycles
- 70% duty = 228 cycles
