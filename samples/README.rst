Ambiq Sample Build Commands
###########################

This file lists simple sample and module build commands. They apply to every
Ambiq board in this tree, not just the newest parts: the commands use the
generic ``west build -b your_board_here ... -p always`` form, and where a
sample depends on the board exposing particular peripherals or chosen nodes,
the selected board has to provide them.

The full board list is in `../doc/ambiq/Supported_Features.rst
<../doc/ambiq/Supported_Features.rst>`_. The commands below were checked
against these board DTS files:

- ``boards/ambiq/apollo510_evb/apollo510_evb.dts``
- ``boards/ambiq/apollo510dL_evb/apollo510dL_evb.dts``
- ``boards/ambiq/apollo510b_evb/apollo510b_evb.dts``
- ``boards/ambiq/apollo330mP_evb/apollo330mP_evb.dts``

Only sample and module paths that exist in this tree are listed here.

Related Ambiq Docs
******************

- `../README.rst <../README.rst>`_ — repository landing page
- `../doc/ambiq/README.rst <../doc/ambiq/README.rst>`_ — Ambiq documentation index
- `../doc/ambiq/Supported_Features.rst <../doc/ambiq/Supported_Features.rst>`_
- `../doc/ambiq/How_to_Build_and_Flash.rst <../doc/ambiq/How_to_Build_and_Flash.rst>`_
- `../doc/ambiq/How_to_Setup_Toolchain.rst <../doc/ambiq/How_to_Setup_Toolchain.rst>`_
- `../doc/ambiq/Bluetooth.rst <../doc/ambiq/Bluetooth.rst>`_
- `../doc/ambiq/How_to_Run_Bluetooth_Samples.rst <../doc/ambiq/How_to_Run_Bluetooth_Samples.rst>`_
- `../doc/ambiq/How_to_Run_Zephyr_USB_Samples.rst <../doc/ambiq/How_to_Run_Zephyr_USB_Samples.rst>`_
- `../doc/ambiq/How_to_Run_Zephyr_MSPI_Samples_and_Tests.rst <../doc/ambiq/How_to_Run_Zephyr_MSPI_Samples_and_Tests.rst>`_
- `../doc/ambiq/How_to_Run_MCUBoot_Samples_and_Tests.rst <../doc/ambiq/How_to_Run_MCUBoot_Samples_and_Tests.rst>`_
- `Release notes <https://github.com/AmbiqMicro/ambiqzephyr/releases>`_

Common Samples
**************

.. code-block:: console

   west build -b your_board_here samples/drivers/watchdog -p always
   west build -b your_board_here samples/subsys/mgmt/mcumgr/smp_svr -p always

MCUboot and DFU Samples
***********************

From the Ambiq MCUboot guide:

.. code-block:: console

   west build -b your_board_here -p always --sysbuild ./samples/sysbuild/with_mcuboot -d ../build/sysbuild/with_mcuboot
   west build -b your_board_here -p always ./samples/subsys/mgmt/mcumgr/smp_svr/ --sysbuild -d ../build/samples/subsys/mgmt-mcumgr-smp-svr/serial -T sample.mcumgr.smp_svr.serial

Audio
*****

AMIC / AUDADC:

.. code-block:: console

   west build -b your_board_here samples/drivers/audio/amic -p always

DMIC / PDM:

.. code-block:: console

   west build -b your_board_here samples/drivers/audio/dmic -p always

Bluetooth
*********

``apollo3_evb``, ``apollo3p_evb``, ``apollo4p_blue_kxr_evb`` and
``apollo510b_evb`` use SPI HCI. ``apollo510dL_evb`` and ``apollo330mP_evb``
use IPC HCI. All Ambiq Bluetooth boards are LE only.

Validated support per board is in `../doc/ambiq/Bluetooth.rst
<../doc/ambiq/Bluetooth.rst>`_, and the full set of build commands is in
`../doc/ambiq/How_to_Run_Bluetooth_Samples.rst
<../doc/ambiq/How_to_Run_Bluetooth_Samples.rst>`_.

.. code-block:: console

   west build -b your_board_here samples/bluetooth/beacon -p always
   west build -b your_board_here samples/bluetooth/broadcaster -p always
   west build -b your_board_here samples/bluetooth/observer -p always
   west build -b your_board_here samples/bluetooth/peripheral -p always
   west build -b your_board_here samples/bluetooth/central -p always
   west build -b your_board_here samples/bluetooth/peripheral_amota -p always

The radio subsystem mailbox IPC used by ``apollo510dL_evb`` and
``apollo330mP_evb`` has its own demonstration sample:

.. code-block:: console

   west build -b your_board_here samples/boards/ambiq/rss_ipc -p always

Peripheral Samples
******************

.. code-block:: console

   west build -b your_board_here samples/drivers/adc/adc_dt -p always
   west build -b your_board_here samples/drivers/counter/alarm -p always
   west build -b your_board_here samples/drivers/rtc -p always
   west build -b your_board_here samples/drivers/crc -p always
   west build -b your_board_here samples/drivers/eeprom -p always
   west build -b your_board_here samples/drivers/memc -p always
   west build -b your_board_here samples/drivers/mspi/mspi_flash -p always
   west build -b your_board_here samples/drivers/mspi/mspi_timing_scan -T sample.drivers.mspi.timing_scan.memc -p always
   west build -b your_board_here samples/drivers/mspi/mspi_timing_scan -T sample.drivers.mspi.timing_scan.flash -p always
   west build -b your_board_here samples/boards/ambiq/spi_serial_flash -p always
   west build -b your_board_here samples/boards/ambiq/dram_click -p always -- "-DSHIELD=ambiq_mikrobus_spi;mikroe_dram_click"

Display and UI
**************

.. code-block:: console

   west build -b your_board_here --shield ap510_disp samples/drivers/display -p always
   west build -b your_board_here --shield ap510_jdi_disp samples/drivers/display -p always
   west build -b your_board_here --shield ap510_disp samples/modules/lvgl/demos -p always
   west build -b your_board_here samples/subsys/input/draw_touch_events -p always

``samples/subsys/input/draw_touch_events`` requires both ``zephyr,display``
and ``zephyr,touch`` chosen nodes on the selected board.

Storage and USB
***************

.. code-block:: console

   west build -b your_board_here samples/subsys/fs/fs_sample -p always
   west build -b your_board_here samples/subsys/usb/mass -p always

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

   west build -b your_board_here samples/benchmarks/coremark -p always
