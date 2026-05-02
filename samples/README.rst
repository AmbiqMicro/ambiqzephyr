Ambiq Sample Build Commands
###########################

This file lists simple sample and module build commands for the Ambiq boards
checked from these board DTS files:

- ``boards/ambiq/apollo510dL_evb/apollo510dL_evb.dts``
- ``boards/ambiq/apollo330mP_evb/apollo330mP_evb.dts``

Only sample and module paths that exist in this tree are listed here.
Where a sample depends on the selected board exposing the needed peripherals or
chosen nodes, the regular ``west build -b <board> ... -p always`` form is used.

Related Ambiq Docs
******************

- `../doc/ambiq/How_to_Run_Zephyr_USB_Samples.rst <../doc/ambiq/How_to_Run_Zephyr_USB_Samples.rst>`_
- `../doc/ambiq/How_to_Run_Zephyr_MSPI_Samples_and_Tests.rst <../doc/ambiq/How_to_Run_Zephyr_MSPI_Samples_and_Tests.rst>`_
- `../doc/ambiq/How_to_Run_MCUBoot_Samples_and_Tests.rst <../doc/ambiq/How_to_Run_MCUBoot_Samples_and_Tests.rst>`_
- `../doc/ambiq/RELEASE_NOTES.rst <../doc/ambiq/RELEASE_NOTES.rst>`_

Common Samples
**************

.. code-block:: console

   west build -b <board> samples/drivers/watchdog -p always
   west build -b <board> samples/subsys/mgmt/mcumgr/smp_svr -p always

MCUboot and DFU Samples
**********************

From the Ambiq MCUboot guide:

.. code-block:: console

   west build -b apollo510dL_evb -p always --sysbuild ./samples/sysbuild/with_mcuboot -d ../build/sysbuild/with_mcuboot
   west build -b apollo510dL_evb -p always ./samples/subsys/mgmt/mcumgr/smp_svr/ --sysbuild -d ../build/samples/subsys/mgmt-mcumgr-smp-svr/serial -T sample.mcumgr.smp_svr.serial

The MCUboot guide also lists ``samples/subsys/usb/dfu-next``, but no usable
command is provided there yet.

Audio
*****

AMIC / AUDADC:

.. code-block:: console

   west build -b apollo510dL_evb samples/drivers/audio/amic -p always

DMIC / PDM:

.. code-block:: console

   west build -b apollo510dL_evb samples/drivers/audio/dmic -p always
   west build -b apollo330mP_evb samples/drivers/audio/dmic -p always

Bluetooth
*********

``apollo510dL_evb`` and ``apollo330mP_evb`` use IPC HCI.

.. code-block:: console

   west build -b apollo510dL_evb samples/bluetooth/peripheral -p always
   west build -b apollo330mP_evb samples/bluetooth/peripheral -p always

Peripheral Samples
******************

.. code-block:: console

   west build -b <board> samples/drivers/adc/adc_dt -p always
   west build -b <board> samples/drivers/counter/alarm -p always
   west build -b <board> samples/drivers/rtc -p always
   west build -b apollo510dL_evb samples/drivers/crc -p always
   west build -b apollo510dL_evb samples/drivers/eeprom -p always
   west build -b apollo510dL_evb samples/drivers/memc -p always
   west build -b apollo510dL_evb samples/drivers/mspi/mspi_flash -p always
   west build -b apollo510dL_evb samples/drivers/mspi/mspi_timing_scan -T sample.drivers.mspi.timing_scan.memc -p always
   west build -b apollo510dL_evb samples/drivers/mspi/mspi_timing_scan -T sample.drivers.mspi.timing_scan.flash -p always
   west build -b <board> samples/boards/ambiq/spi_serial_flash -p always

Display and UI
**************

.. code-block:: console

   west build -b apollo510dL_evb --shield ap510_disp samples/drivers/display -p always
   west build -b apollo510dL_evb --shield ap510_jdi_disp samples/drivers/display -p always
   west build -b apollo510dL_evb --shield apollo5_eb_display_8080_card samples/drivers/display -p always
   west build -b apollo510dL_evb --shield apollo5_eb_display_card samples/drivers/display -p always
   west build -b apollo510dL_evb --shield ap510_disp samples/modules/lvgl/demos -p always
   west build -b <board> samples/subsys/input/draw_touch_events -p always

``samples/subsys/input/draw_touch_events`` requires both ``zephyr,display``
and ``zephyr,touch`` chosen nodes on the selected board.

Storage and USB
***************

.. code-block:: console

   west build -b apollo510dL_evb samples/subsys/fs/fs_sample -p always
   west build -b apollo330mP_evb samples/subsys/fs/fs_sample -p always
   west build -b apollo510dL_evb samples/subsys/usb/mass -p always
   west build -b apollo330mP_evb samples/subsys/usb/mass -p always

Guide Notes
***********

The Ambiq USB guide also documents these sample areas:

- ``samples/subsys/usb/cdc_acm``
- ``samples/subsys/usb/console``
- ``samples/subsys/usb/hid-keyboard``
- ``samples/subsys/usb/hid-mouse``
- ``samples/subsys/usb/webusb``

That guide currently uses older naming such as ``console-next`` and
``webusb-next``, and references ``usbd_next_prj.conf`` /
``usbd_next.overlay`` files that are not present under
``samples/subsys/usb`` in this tree. Use the guide as supplemental board
notes, but prefer the current sample directories listed above.

Modules
*******

.. code-block:: console

   west build -b apollo510dL_evb samples/benchmarks/coremark -p always
