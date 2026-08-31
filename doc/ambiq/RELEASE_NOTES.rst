============================================
© 2026 Ambiq Micro Inc. All rights reserved.
============================================

Release Notes
=============

Release Version: v4.3.0-ambiq.v1.0.0

Release Date: 2026-08-31

Summary
-------

This is the first official public release of the Ambiq Zephyr distribution.
It introduces full support for two new parts — **Apollo510L and Apollo330P** —
and consolidates a year of driver, SoC, board and test work across the whole
Apollo family. The release is based on upstream Zephyr v4.3.0.

Apollo510L and Apollo330P are supported end to end: SoC, boards, devicetree,
drivers, HAL, samples and tests. They bring a new Bluetooth transport (radio
subsystem over an inter-processor mailbox rather than an SPI-attached
controller), a new mailbox driver, the CryptoCell-312 crypto stack (AES, SHA,
ChaCha20, and mbedTLS ECDSA routed to the PKA) and a PUF TRNG entropy source, USB device support, SDIO, PWM, and a deeper sleep
state with a debugger recovery window. Apollo510 and Apollo510B gain EM9305
Bluetooth low-power operation, the Nema GPU driver, and a core CRC API.
Apollo4x, Apollo3x and Apollo2 receive driver corrections, and Apollo2 gains
flash support for the first time.

The Ambiq-owned content of this release is 621 changed files. The release also
absorbs the upstream Zephyr v4.3.0 release, a further 11,016 changed files
across the tree, which are not enumerated here. Because the release touches
Bluetooth initialisation, power-state policy, flash locking and devicetree
addressing, it should be treated as carrying moderate integration risk and
validated per chipset family before deployment.

--------------

Comparison Range
----------------

- **From:** ``ambiq-stable-pre-v1.0.0`` (2026-07-08)
- **To:** ``v4.3.0-ambiq.v1.0.0``
- **Upstream baseline:** Zephyr v4.3.0
- **HAL:** ``ambiqhal_ambiq`` — see the accompanying HAL release notes
- **Ambiq-owned changes:** 621 files

--------------

1. Apollo510L and Apollo330P
----------------------------

**The headline of this release.** Both parts are newly supported end to end.

- **SoCs:** ``apollo510L``, ``apollo330P``
- **Boards:** ``apollo510dL_evb``, ``apollo330mP_evb``
- **Devicetree:** ``ambiq_apollo510L.dtsi``, ``ambiq_apollo330P.dtsi``

1.1 Drivers
~~~~~~~~~~~

**``drivers/audio/dmic_ambiq_pdm.c``** — Apollo330P support added. Variable PCM
rate support and a read buffer queue; packed-bit handling for 32-bit samples;
corrected OSR value. Power-management holes closed and HAL statuses surfaced
to the caller instead of being discarded.

**``drivers/bluetooth/hci/apollox_ipc_support.c``** — Added the Apollo330P /
Apollo510L IPC setup sequence. These parts reach the radio through an
inter-processor mailbox rather than an SPI-attached controller, so they take a
different HCI transport from the rest of the Apollo5 family. SIMOBUCK low-power
configuration added, IPC low-power behaviour improved, and NVDS and
vendor-setup comments reworded to use Zephyr Kconfig terminology.

**``drivers/bluetooth/hci/hci_ambiq.c``** — Apollo5X HCI paths split by SoC, so
the IPC parts and the EM9305 SPI parts no longer share one initialisation path.
Vendor-setup errors are propagated out of HCI setup rather than swallowed.
AMOTA handling aligned with the IPC transport.

**``drivers/counter/counter_ambiq_timer.c``** — Counter support added for
Apollo510L, then Apollo330P. PM device support added.

**``drivers/crypto/crypto_ambiq_aes.c``**, **``crypto_ambiq_sha.c``**,
**``crypto_ambiq_chacha.c``** — the CryptoCell-312 driver stack. AES came first,
with its devicetree binding and tests, then SHA and a ChaCha20 stream cipher on
top. The three share one CC312 interrupt and lock rather than each claiming the
block. The AES CTR counter starts at zero, matching the test vectors, and
cipher mode numbering was aligned with the upstream assignments. The stack
moved off direct ``ambiq_pwrctrl`` calls onto the Zephyr power-domain framework,
with refcounting on the shared peripheral power rails so two users of the
crypto rail cannot power it down under one another, and the crypto powerdown
sequence was corrected. See section 5A.

**``drivers/entropy/entropy_ambiq_puf_trng.c``** — PUF TRNG entropy driver added
and restructured for on-demand power management, on the Zephyr power-domain
framework with rail refcounting. The TRNG is memory-mapped in OTP and needs
only the OTP peripheral, so its node was repointed from ``crypto_pd`` to
``otp_pd``; entropy reads no longer power the whole CC312 rail or run its
dedicated powerdown sequence.

**``drivers/i2c/i2c_ambiq_ios.c``** — **Apollo510L and Apollo330P are excluded
from the IOS I2C driver.** I2C target support was added for these parts during
the cycle and subsequently withdrawn. If you were building against it, it is
gone in this release. Parts that keep the driver gain byte-mode support and
``pm_device_get`` / ``pm_device_put`` around target operations.

**``drivers/i2s/i2s_ambiq.c``** — Apollo330P support added. Full-duplex support,
PM device suspend/resume, a switch fallthrough fixed in ``configure``, 24-bit
frame length corrected to use a 32-bit slot, PLL POSTDIV reused rather than
overridden, and power-on failures now log their status.

**``drivers/mbox/mbox_ambiq.c``** — **New driver**, created for Apollo510L and
Apollo330P. This is the mailbox that carries radio-subsystem IPC.
Initialisation moved earlier in boot so the mailbox is up before its first
user. Note that the mailbox peer is the Ambiq radio subsystem running Ambiq
firmware, not a second Zephyr image, so two-core mailbox samples such as
``mbox_data`` do not apply to these boards.

**``drivers/mipi_dsi/dsi_ambiq.c``** — MIPI-DSI updated for Apollo510L. PM
support added, then the Zephyr power-domain integration removed from the
display drivers in favour of direct HAL peripheral enable calls, which resolved
display initialisation failures. ``nemadc_timing`` back-porch argument corrected.

**``drivers/mspi/mspi_ambiq_ap5.c``** — Apollo330P support added. The synchronous
DMA completion wait is now bounded and an asynchronous wait timeout guard
added, so a stalled transfer fails rather than hanging. Board-level
``TIMING_SCAN_SET`` calibration is restored after ``am_hal_mspi_device_configure()``,
which had been resetting TXNEG, RXNEG, RXCAP and the DQS delays to HAL defaults
on every transfer configuration. CE latency workaround added.

**``drivers/pwm/pwm_ambiq_ctimer.c``** — PWM support for Apollo510L via the
CTIMER back end.

**``drivers/pwm/pwm_ambiq_timer.c``** — PWM support for Apollo510L, then
Apollo330P. PWM channel support added.

**``drivers/sdhc/sdhc_ambiq.c``** — SDIO support added for Apollo510L.

**``drivers/serial/uart_ambiq.c``** — UART support added for the Apollo510L SoC,
then Apollo330P. Spurious idle wakeups prevented.

**``drivers/spi/spi_ambiq_dcif.c``** — SPI display interface updated to support
Apollo510L. Moved onto the Zephyr power domain with rail refcounting, then the
power-domain PM removed from the display path.

**``drivers/usb/udc/udc_ambiq.c``** — USB device support added for Apollo510L.
Kconfig added for SoCs carrying the CRM module, and variant-specific handling
moved down into the HAL. The OUT transfer maximum-packet-size clamp was
restored: without it an OUT transaction can start larger than the endpoint MPS,
and outside DMA1 mode the USB IP cannot detect the end of a transaction whose
last packet is not short, so the transfer never completes.

**``drivers/watchdog/wdt_ambiq.c``** — Watchdog support added for the Apollo510L
SoC, then Apollo330P. ``WDT_OPT_PAUSE_IN_SLEEP`` is now honoured, the elapsed
count is kept across a sleep pause, the debug pause option is accepted as a
no-op rather than rejected, and the callback runs on a resetting timeout.

**``drivers/input/input_chsc5x.c``** — Touch controller IC type handling now
accepts ``0x10``, ``0x16`` and ``0xa0``. Different panels on these boards report
different ids through the same shield and were previously rejected at init with
a wrong-ic-type error. An initialisation ordering problem that left the display
blank was also fixed.

1.2 SoC support
~~~~~~~~~~~~~~~

- **Radio subsystem manager** (``soc/ambiq/apollo5x/rss_mgr/``) added. Only these
  two parts select ``SOC_AMBIQ_HAS_RSS``.
- **Deeper sleep recovery window.** These parts declare a third CPU power state
  that Apollo510 does not — suspend-to-disk, where only NVM is retained. A part
  that enters it cannot be halted, and the residency policy reaches it almost
  immediately on any application that idles. An opt-in recovery window, off by
  default, samples a pin at initialisation and holds a policy lock on that state
  for a configurable period so a debugger can attach and erase. The pin is the
  ``pm-recovery`` alias where a board defines one, and ``sw0`` otherwise.

1.3 Boards and shields
~~~~~~~~~~~~~~~~~~~~~~

- ``apollo510dL_evb`` and ``apollo330mP_evb`` board support.
- Ambiq mikroBUS I2C and SPI adapter shields, and the MikroE DRAM Click and
  EERAM 33V Click shields, are available on both boards.
- Display SPI shield support including QSPI chip-select routing on
  ``apollo510dL_evb``.

1.4 Devicetree
~~~~~~~~~~~~~~

- **``hw-crc32`` base address corrected** on both parts. The node addresses
  ``SECURITY.RESULT`` at offset 0x30. Both dtsi files carried ``0x40030030``, the
  Apollo510 address; on Apollo330P and Apollo510L ``SECURITY`` is at ``0x4000e800``
  and ``TIMER`` is at ``0x40030000``. An application using ``crc0`` was reading and
  writing timer registers. Verified against the AmbiqSuite 5.2.0 SVDs.
- **PUF TRNG power domain** moved from ``crypto_pd`` to ``otp_pd``.
- CC312 AES crypto nodes added and enabled, with power-domain linkage.

1.5 Samples and tests
~~~~~~~~~~~~~~~~~~~~~

- Power-management peripheral demo exercising BLE, flash, CRC, AES, entropy,
  SPI and I2C paths where devicetree supports them.
- Bluetooth low-power configurations for power measurement.
- DRAM Click and EERAM Click samples.
- Two-core mailbox samples are excluded on these boards — the mailbox peer is
  Ambiq radio firmware, not a Zephyr image.

--------------

2. Apollo510 and Apollo510B
---------------------------

- **SoCs:** ``apollo510``, ``apollo510b``
- **Boards:** ``apollo510_evb``, ``apollo510b_evb``

.. _drivers-1:

2.1 Drivers
~~~~~~~~~~~

**``drivers/bluetooth/hci/apollox_blue.c``** — EM9305 controller integration
reworked. SPI transport updated with a recovery path; Apollo5X HCI paths split
by SoC; Apollo5 EM9305 initialisation ordering and CM GPIO setup corrected;
Apollo4 RX thread start restored ahead of controller init. For Apollo510B
low-power builds, radio recovery was decoupled from the heartbeat timer so LP
builds still recover from hardware errors and TX_PARTIAL faults, a failed
``sleep_set`` is treated as fatal, sleep is routed through the HAL, and CLKREQ is
de-asserted when EXTREF is unused. A missing ``sys/reboot.h`` include was added,
which had broken any build enabling both Bluetooth and ``CONFIG_REBOOT``.

**``drivers/crc/crc_ambiq.c``** — Hardware CRC driver, moved from
``pm_policy_state_lock`` onto the power-domain framework.

**``drivers/flash/flash_ambiq.c``** — Locking, retries and verification reworked;
cache invalidated before ``irq_unlock``.

**``drivers/gpu/gpu_ambiq.c``** — The Nema GPU driver was moved out of the SoC
tree into ``drivers/gpu/``, where it is a first-class Zephyr driver with its own
Kconfig and binding.

**``drivers/spi/spi_ambiq_spic.c``** — Locking and PM reworked; ``power_ctrl``
regressions fixed.

**``drivers/spi/spi_ambiq_spid.c``** — Two PM defects fixed. A failed
configuration returned early, skipping the runtime PM put and leaking the usage
count, so the device could never runtime-suspend again; and the transfer status
was overwritten by the return of the asynchronous put, hiding transfer errors
from the caller. Separately, the configuration is no longer cached until the
HAL accepts it, so a failed IOS configure cannot leave the context marked
configured while the IOS is disabled.

**``drivers/usb/udc/udc_ambiq.c``** — Apollo5x high-speed transfer support;
full-speed default on ``apollo510b_evb``; SDIO pinctrl corrected.

**``drivers/display/display_co5300.c``** — Driver optimised and returned to the
Ambiq implementation.

**``drivers/i2c/i2c_ambiq.c``** — Locking, PM and error handling reworked.

.. _soc-support-1:

2.2 SoC support
~~~~~~~~~~~~~~~

- Apollo510B SoC support added, including BLE low-power SoC initialisation for
  EM9305 SPI with selective peripheral gating, optional DTCM-only SRAM, and
  runtime initialisation after ``bt_enable()``.
- EM9305 EXTREF initialisation is excluded from MCUboot builds. It selects the
  Ambiq BT HAL component, which was being linked into the bootloader and
  overflowing the 256K boot partition.
- GPU halt fix.

.. _boards-and-shields-1:

2.3 Boards and shields
~~~~~~~~~~~~~~~~~~~~~~

- ``apollo510b_evb`` board support, EXTREF clock enablement, MSPI PSRAM, USB
  tuning, SDIO pinctrl.
- Display SPI shields for Apollo510 EVB variants, with the touch-controller
  node reference corrected.
- Ambiq mikroBUS adapter shields and the MikroE Click shields.

.. _devicetree-1:

2.4 Devicetree
~~~~~~~~~~~~~~

- Apollo510B devicetree added.
- SPID nodes renamed to IOS.
- Power states updated.

.. _samples-and-tests-1:

2.5 Samples and tests
~~~~~~~~~~~~~~~~~~~~~

- ``peripheral_hr`` Bluetooth low-power build configuration for ``apollo510b_evb``.
- LVGL display and demo configurations.
- Retained-memory overlays corrected for the 1.5 MB SSRAM map; the addresses
  had been copied from a 2 MB board and overlapped the non-cached region.

--------------

3. Apollo4x
-----------

- **SoCs:** ``apollo4p``, ``apollo4p_blue``
- **Boards:** ``apollo4p_evb``, ``apollo4p_blue_kxr_evb``

.. _drivers-2:

3.1 Drivers
~~~~~~~~~~~

**``drivers/mspi/mspi_ambiq_ap4.c``** — Clock limits restored to Apollo4
hardware: ``MSPI_MAX_FREQ`` back to 96 MHz, and the RX double-sampling threshold
reverted from a greater-than-or-equal test to an exact 96 MHz match. Apollo4
does not support DDR above 96 MHz, so the wider check was incorrect. Async wait
timeout guard and completion timeout tolerance fallback added.

**``drivers/display/display_co5300.c``** — ``apollo4p_evb_disp_shield_rev2``
supported at 456x456.

**``drivers/serial/uart_ambiq.c``** — Driver made compatible with Apollo4x.

**``drivers/bluetooth/hci/apollox_blue.c``** — Apollo4 RX thread start restored
ahead of controller initialisation, which Apollo5 requires in the opposite
order.

.. _devicetree-2:

3.2 Devicetree
~~~~~~~~~~~~~~

- **Apollo4P Blue MSPI controllers now bind a driver.** The three MSPI nodes
  were declared as ``spi@`` with compatible ``ambiq,mspi``, which no driver claims —
  all Ambiq MSPI drivers match ``ambiq,mspi-controller``. MSPI was silently
  unavailable on the part. The nodes are now aligned with Apollo4P, which is the
  same silicon: the XIP aperture as a second reg entry and the 48 MHz
  ``clock-frequency``.

.. _samples-and-tests-2:

3.3 Samples and tests
~~~~~~~~~~~~~~~~~~~~~

- SPI loopback, RTC and MSPI overlays added.
- Flash size expectations supplied for ``apollo4p_evb`` and
  ``apollo4p_blue_kxr_evb``, which had been running the common flash test without
  ever comparing the reported size.

--------------

4. Apollo3x
-----------

- **SoCs:** ``apollo3_blue``, ``apollo3p_blue``
- **Boards:** ``apollo3_evb``, ``apollo3p_evb``

.. _drivers-3:

4.1 Drivers
~~~~~~~~~~~

**``drivers/gpio/gpio_ambiq.c``** — Apollo3 interrupt handling hardened; writes
to input-configured pins prevented.

**``drivers/hwinfo/hwinfo_ambiq.c``** — Apollo3 series support added.

**``drivers/timer/ambiq_stimer.c``** — Apollo3 tickless jitter fixed; every
series now included in the remainder accounting.

**``drivers/counter/counter_ambiq_timer.c``** — Apollo3 and API compatibility
issues fixed.

**``drivers/serial/uart_ambiq.c``** — Apollo3 switched onto ``uart_ambiq``.

**``drivers/mspi/mspi_ambiq_ap3.c``** — Async wait timeout guard and completion
timeout tolerance fallback.

.. _soc-support-2:

4.2 SoC support
~~~~~~~~~~~~~~~

- ``mspi_buff`` is now gated on the MSPI driver rather than on the devicetree
  node. The section is placed at an offset symbol that only exists when the
  driver is selected, so enabling the node without the driver left the linker
  with an undefined symbol.

.. _devicetree-3:

4.3 Devicetree
~~~~~~~~~~~~~~

- **Apollo3 ``mspi0`` corrected.** The node carried the node name ``spi@``,
  compatible ``ambiq,mspi`` and no ``clock-frequency``, so it matched no driver at
  all. Its address ``0x40020000`` is MCUCTRL, not MSPI, and the driver derives its
  channel index from that address. Corrected to ``0x50014000`` with the right
  compatible, matching Apollo3p.

.. _samples-and-tests-3:

4.4 Samples and tests
~~~~~~~~~~~~~~~~~~~~~

- SPI loopback, I2C and RTC overlays added.
- Flash size expectations supplied for ``apollo3_evb`` and ``apollo3p_evb``.

--------------

5. Apollo2
----------

- **SoC:** ``apollo2``
- **Board:** ``apollo2_evb``

.. _drivers-4:

5.1 Drivers
~~~~~~~~~~~

**``drivers/flash/flash_ambiq.c``** — **Apollo2 flash support added.** The
Apollo2 flash HAL predates ``AM_HAL_STATUS_*``, so the driver carried no path for
it and the Apollo2 devicetree had no flash-controller node. Nothing could bind,
and the common flash test stopped at a missing flash device. Driver paths and
the controller node were added, the ``SOC_NV_FLASH_COMPAT_NODE`` and
``SOC_NV_FLASH_CHILD_NODE`` helpers moved into ``flash_priv.h`` so the driver finds
its partition child the way other Ambiq parts do, and the erase and write paths
were put on a single lock order.

**``drivers/serial/uart_ambiq.c``** — Apollo2 switched onto ``uart_ambiq``.

**``drivers/timer/ambiq_stimer.c``** — The kernel clock now runs from the STIMER
on Apollo2.

5.2 Devicetree and boards
~~~~~~~~~~~~~~~~~~~~~~~~~

- Flash controller node added, with ``zephyr,flash-controller`` and
  ``zephyr,code-partition`` chosen nodes on ``apollo2_evb`` so MCUboot can build.

.. _samples-and-tests-4:

5.3 Samples and tests
~~~~~~~~~~~~~~~~~~~~~

- Common flash test now runs, with the expected size supplied.

--------------

5A. CryptoCell-312 (Apollo510, Apollo510B, Apollo510L, Apollo330P)
------------------------------------------------------------------

The CC312 hardware crypto block is supported on all four Apollo5-class parts.

- **AES**, **SHA** and **ChaCha20** drivers, each with a devicetree node and
  binding on ``ambiq_apollo510.dtsi``, ``ambiq_apollo510b.dtsi``,
  ``ambiq_apollo510L.dtsi`` and ``ambiq_apollo330P.dtsi``.
- The three drivers share a single CC312 interrupt and lock, so concurrent
  users of the block serialise instead of racing for it.
- **mbedTLS ECDSA is routed to the CC312 PKA** through an alternate
  implementation (``modules/mbedtls/alt_ambiq_cc312_ecdsa.c``), so ECDSA sign and
  verify run on hardware rather than in software.
- Cipher mode numbering was aligned with the upstream assignments, and the AES
  CTR counter now starts at zero to match the published test vectors.
- Test coverage: the AES example runs on every CC312 part, the SHA and ChaCha
  examples are documented, a maximum-length DLLI ChaCha transfer is exercised,
  the generic Zephyr crypto suites run against Ambiq CC312, ``tests/crypto/secp256r1``
  routes its ECDSA math to the PKA, and JWT exercises the CC312 ECDSA path on
  the EVBs.
- A hardware counterpart was added to the mbedTLS benchmarks so software and
  CC312 throughput can be compared directly.

--------------

6. Zephyr APIs
--------------

These APIs exist in the Ambiq distribution. Their upstream status matters if
you intend to move an application to mainline Zephyr.

+----------------------------------------------------+-----------------------------------+
| API                                                | Status                            |
+====================================================+===================================+
| ``include/zephyr/drivers/jdi.h`` and               | Ambiq distribution only. JDI      |
| ``dt-bindings/jdi/jdi.h``                          | memory-in-pixel display bus.      |
+----------------------------------------------------+-----------------------------------+
| ``include/zephyr/audio/amic.h``                    | Ambiq distribution only. Analog   |
|                                                    | microphone / AUDADC.              |
+----------------------------------------------------+-----------------------------------+
| ``include/zephyr/drivers/gpu/gpu_ambiq.h``         | Ambiq distribution only. Nema     |
|                                                    | GPU.                              |
+----------------------------------------------------+-----------------------------------+
| ``include/zephyr/dt-bindings/power/ambiq_power.h`` | Ambiq distribution only.          |
|                                                    | Power-domain identifiers.         |
+----------------------------------------------------+-----------------------------------+
| Core CRC driver API                                | Added in this cycle with a        |
|                                                    | software implementation alongside |
|                                                    | the Ambiq hardware driver.        |
+----------------------------------------------------+-----------------------------------+

A power-domain model replaces the previous direct ``ambiq_pwrctrl`` calls across
the crypto, entropy, display, MIPI-DSI, MIPI-DBI, JDI and DCIF drivers, with
refcounting on shared peripheral rails.

--------------

7. Build, CI and Manifest
-------------------------

Getting started
~~~~~~~~~~~~~~~

Initialise the workspace from the **``ambiq-stable`` branch**:

.. code:: bash

   west init -m https://github.com/AmbiqMicro/ambiqzephyr --mr ambiq-stable
   west update

The branch is the supported entry point for this release. The
``v4.3.0-ambiq.v1.0.0`` tag marks the source state and is not intended for
workspace initialisation.

Manifest
~~~~~~~~

- **HAL:** ``west.yml`` resolves the Ambiq HAL from ``ambiqhal_ambiq``. See the
  accompanying HAL release notes for its contents.
- Manifest jobs now clone with enough history for ``git merge-base`` to resolve,
  and refresh the runner reference cache before cloning.
- Twister no longer publishes results for a run in which no test suites were
  selected.
- The kernel boot banner reports the ``VERSION`` file value rather than a tag
  inherited from fork history.
- Board documentation now records the SDK release test build set for each
  board — the samples and tests that board is expected to build, with the extra
  arguments each needs.

--------------

7A. Documentation
-----------------

- The root ``README.rst`` is now a landing page. Product material moved under
  ``doc/ambiq/``: ``README.rst`` (index), ``Supported_Features.rst``,
  ``How_to_Build_and_Flash.rst`` and ``Power_Management.rst``, with the MSPI board
  notes folded into the existing MSPI guide.
- ``Supported_Features.rst`` carries the board list, the driver and power
  management matrix, third-party library status and the component tree. GPIO,
  MBOX and the power-domain drivers were added to the matrix.
- Every Ambiq board’s documentation now records the samples and tests that
  board builds as part of the overnight release build set, with the extra
  arguments each entry needs.

--------------

8. Known Issues, Removals and Migration
---------------------------------------

Removals
~~~~~~~~

- **Apollo510L and Apollo330P no longer build the IOS I2C target driver.**
  Support was added and then withdrawn during this cycle.
- The eight ``doc/ambiq/*-samples-and-tests.rst`` guides present on the previous
  public branch are not carried forward. Equivalent content is in
  ``doc/ambiq/How_to_Run_*.rst``. ``crypto-samples-and-tests.rst`` and
  ``general-samples-and-tests.rst`` have no replacement.
- ``samples/drivers/adc/adc_dt/boards/apollo510b_evb.conf`` is not carried
  forward.
- Engineering boards and their display shields are not part of this release.

Migration
~~~~~~~~~

- Applications using ``crc0`` on Apollo330P or Apollo510L were addressing timer
  registers. Re-test after upgrading.
- MSPI is now available on Apollo3 Blue and Apollo4P Blue, where the nodes
  previously bound no driver.
- Apollo4 MSPI is constrained to 96 MHz. Configurations relying on the higher
  Apollo5-derived limit are now clamped.
- Display drivers no longer use the Zephyr power-domain framework and call the
  HAL peripheral enable directly.

Previous release line
~~~~~~~~~~~~~~~~~~~~~

The pre-1.0 public line is frozen at tag ``ambiq-stable-pre-v1.0.0`` and remains
fetchable at branch ``frozen/ambiq-stable-pre-v1.0.0-2026-07``. It receives no
further updates.

Validation
~~~~~~~~~~

Validate per chipset family. At minimum, build and run one board per family:
``apollo330mP_evb``, ``apollo510dL_evb``, ``apollo510_evb``, ``apollo510b_evb``,
``apollo4p_evb``, ``apollo3p_evb``, ``apollo2_evb``.
