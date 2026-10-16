Bluetooth
#########

Validated Bluetooth support for the Ambiq boards in this tree. Each board
lists the host transport it uses and the Bluetooth samples confirmed working
on hardware. A sample that is not listed for a board was either not exercised
on that board or is not supported there.

Validation baseline: 28 August 2026 on ``ambiq-stable``.

See also:

- `README.rst <README.rst>`_ — Ambiq documentation index
- `How_to_Run_Bluetooth_Samples.rst <How_to_Run_Bluetooth_Samples.rst>`_ —
  build commands for the samples listed here
- `Supported_Features.rst <Supported_Features.rst>`_ — driver, power
  management, and third-party library support
- `../../samples/README.rst <../../samples/README.rst>`_ — sample build
  commands per board

apollo3_evb
***********

Transport: Ambiq SPI HCI. LE only.

Working:

- ``beacon``
- ``broadcaster``
- ``observer``

apollo3p_evb
************

Transport: Ambiq SPI HCI. LE only.

Working:

- ``beacon``
- ``broadcaster``
- ``observer``

apollo4p_blue_kxr_evb
*********************

Transport: Ambiq SPI HCI. LE only.

Working:

- ``beacon``
- ``broadcaster``
- ``observer``
- ``peripheral_amota``

apollo510b_evb
**************

Transport: EM9305 over Ambiq SPI HCI. LE only.

Working:

- ``beacon``
- ``broadcaster``
- ``observer``
- ``peripheral``
- ``peripheral_hr``
- ``peripheral_dis``
- ``peripheral_ht``
- ``peripheral_hids``
- ``central``
- ``central_hr``
- ``peripheral_amota``
- ``encrypted_advertising/peripheral`` — paired against a phone
- ``extended_adv/advertiser``
- ``extended_adv/scanner``
- ``central_past``
- ``periodic_*`` — with the buffer configuration below
- ``iso_broadcast``
- ``iso_receive``
- ``iso_central``
- ``iso_peripheral``

apollo510dL_evb
***************

Transport: Ambiq RSS HCI over IPC/mailbox.

Working:

- ``beacon``
- ``broadcaster``
- ``observer``
- ``peripheral``
- ``peripheral_hr``
- ``peripheral_dis``
- ``peripheral_ht``
- ``peripheral_hids``
- ``central``
- ``central_hr``
- ``peripheral_amota``
- ``central_gatt_write`` — with UUID-based matching
- ``peripheral_gatt_write`` — with UUID-based matching
- ``encrypted_advertising/peripheral`` — paired against a phone
- ``extended_adv/advertiser``
- ``peripheral_past``
- ``periodic_*`` — with the buffer configuration below
- ``iso_broadcast``
- ``iso_receive``
- ``iso_peripheral``
- ``bap_broadcast_sink`` — with synthetic data
- ``bap_broadcast_source`` — with synthetic data
- ``bap_unicast_client`` — with synthetic data
- ``bap_unicast_server`` — with synthetic data

apollo330mP_evb
***************

Transport: Ambiq RSS HCI over IPC/mailbox.

Working:

- ``beacon``
- ``broadcaster``
- ``observer``
- ``peripheral``
- ``peripheral_hr``
- ``peripheral_dis``
- ``peripheral_ht``
- ``peripheral_hids``
- ``central``
- ``central_hr``
- ``peripheral_amota``
- ``central_gatt_write`` — with UUID-based matching
- ``peripheral_gatt_write`` — with UUID-based matching
- ``encrypted_advertising/peripheral`` — paired against a phone
- ``extended_adv/advertiser``
- ``peripheral_past``
- ``periodic_*`` — with the buffer configuration below
- ``iso_broadcast``
- ``iso_receive``
- ``iso_peripheral``
- ``bap_broadcast_sink`` — with synthetic data
- ``bap_broadcast_source`` — with synthetic data
- ``bap_unicast_client`` — with synthetic data
- ``bap_unicast_server`` — with synthetic data

Sample Configuration Notes
**************************

Periodic advertising and sync
=============================

The ``periodic_*`` and encrypted advertising samples need larger event buffers
than the sample defaults. Add to ``prj.conf``::

   CONFIG_BT_BUF_EVT_DISCARDABLE_SIZE=255
   CONFIG_BT_BUF_EVT_RX_SIZE=255
   CONFIG_BT_BUF_EVT_RX_COUNT=16
   CONFIG_BT_BUF_EVT_DISCARDABLE_COUNT=6

GATT write samples
==================

``central_gatt_write`` and ``peripheral_gatt_write`` were validated with the
peer match changed from RSSI filtering to service and characteristic UUID
matching.

LE Audio
========

The BAP samples were validated with synthetic data. External audio board
support for ``mic`` and ``wireless_mic_dongle`` is not available yet.
