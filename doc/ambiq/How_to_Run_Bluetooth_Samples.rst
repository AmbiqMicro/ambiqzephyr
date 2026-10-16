How to Run Zephyr Bluetooth Samples
###################################

Build commands for the Bluetooth samples validated on Ambiq boards. The
per-board list of what is known working is in
`Bluetooth.rst <Bluetooth.rst>`_; this page covers how to build it.

See also:

- `README.rst <README.rst>`_ — Ambiq documentation index
- `Bluetooth.rst <Bluetooth.rst>`_ — validated support per board
- `How_to_Build_and_Flash.rst <How_to_Build_and_Flash.rst>`_ — toolchain,
  flashing, and Bluetooth controller firmware
- `Supported_Features.rst <Supported_Features.rst>`_

Transports
**********

``apollo3_evb``, ``apollo3p_evb``, and ``apollo4p_blue_kxr_evb`` use the Ambiq
SPI HCI transport. ``apollo510b_evb`` drives an EM9305 over the same SPI HCI
transport. ``apollo510dL_evb`` and ``apollo330mP_evb`` use the Ambiq radio
subsystem HCI over IPC/mailbox.

All of these are LE only.

Advertising and Scanning
************************

Validated on every board listed above.

.. code-block:: console

   west build -b your_board_here samples/bluetooth/beacon -p always
   west build -b your_board_here samples/bluetooth/broadcaster -p always
   west build -b your_board_here samples/bluetooth/observer -p always

Connected Peripheral and Central
********************************

Validated on ``apollo510b_evb``, ``apollo510dL_evb``, and ``apollo330mP_evb``.

.. code-block:: console

   west build -b your_board_here samples/bluetooth/peripheral -p always
   west build -b your_board_here samples/bluetooth/peripheral_hr -p always
   west build -b your_board_here samples/bluetooth/peripheral_dis -p always
   west build -b your_board_here samples/bluetooth/peripheral_ht -p always
   west build -b your_board_here samples/bluetooth/peripheral_hids -p always
   west build -b your_board_here samples/bluetooth/central -p always
   west build -b your_board_here samples/bluetooth/central_hr -p always

GATT Write
**********

Validated on ``apollo510dL_evb`` and ``apollo330mP_evb`` with the peer match
changed from RSSI filtering to service and characteristic UUID matching.

.. code-block:: console

   west build -b your_board_here samples/bluetooth/central_gatt_write -p always
   west build -b your_board_here samples/bluetooth/peripheral_gatt_write -p always

AMOTA
*****

Validated on ``apollo4p_blue_kxr_evb``, ``apollo510b_evb``,
``apollo510dL_evb``, and ``apollo330mP_evb``.

.. code-block:: console

   west build -b your_board_here samples/bluetooth/peripheral_amota -p always

Extended Advertising and PAST
*****************************

``extended_adv/advertiser`` is validated on ``apollo510b_evb``,
``apollo510dL_evb``, and ``apollo330mP_evb``. ``extended_adv/scanner`` and
``central_past`` are validated on ``apollo510b_evb``; ``peripheral_past`` is
validated on ``apollo510dL_evb`` and ``apollo330mP_evb``.

.. code-block:: console

   west build -b your_board_here samples/bluetooth/extended_adv/advertiser -p always
   west build -b your_board_here samples/bluetooth/extended_adv/scanner -p always
   west build -b your_board_here samples/bluetooth/central_past -p always
   west build -b your_board_here samples/bluetooth/peripheral_past -p always

Periodic Advertising and Sync
*****************************

Validated on ``apollo510b_evb``, ``apollo510dL_evb``, and ``apollo330mP_evb``.
These samples need larger event buffers than the sample defaults. Add to
``prj.conf`` before building::

   CONFIG_BT_BUF_EVT_DISCARDABLE_SIZE=255
   CONFIG_BT_BUF_EVT_RX_SIZE=255
   CONFIG_BT_BUF_EVT_RX_COUNT=16
   CONFIG_BT_BUF_EVT_DISCARDABLE_COUNT=6

.. code-block:: console

   west build -b your_board_here samples/bluetooth/periodic_adv -p always
   west build -b your_board_here samples/bluetooth/periodic_sync -p always
   west build -b your_board_here samples/bluetooth/periodic_adv_conn -p always
   west build -b your_board_here samples/bluetooth/periodic_sync_conn -p always

Encrypted Advertising
*********************

The peripheral role is validated against a phone on ``apollo510b_evb``,
``apollo510dL_evb``, and ``apollo330mP_evb``. It needs the same buffer
configuration as the periodic samples above.

.. code-block:: console

   west build -b your_board_here samples/bluetooth/encrypted_advertising/peripheral -p always

ISO
***

``iso_broadcast``, ``iso_receive``, and ``iso_peripheral`` are validated on
``apollo510b_evb``, ``apollo510dL_evb``, and ``apollo330mP_evb``.
``iso_central`` is validated on ``apollo510b_evb``.

.. code-block:: console

   west build -b your_board_here samples/bluetooth/iso_broadcast -p always
   west build -b your_board_here samples/bluetooth/iso_receive -p always
   west build -b your_board_here samples/bluetooth/iso_central -p always
   west build -b your_board_here samples/bluetooth/iso_peripheral -p always

LE Audio
********

Validated on ``apollo510dL_evb`` and ``apollo330mP_evb`` with synthetic data.
External audio board support for ``mic`` and ``wireless_mic_dongle`` is not
available yet.

.. code-block:: console

   west build -b your_board_here samples/bluetooth/bap_broadcast_sink -p always
   west build -b your_board_here samples/bluetooth/bap_broadcast_source -p always
   west build -b your_board_here samples/bluetooth/bap_unicast_client -p always
   west build -b your_board_here samples/bluetooth/bap_unicast_server -p always
