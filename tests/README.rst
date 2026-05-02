Ambiq Test Build Commands
#########################

This file lists simple test build commands for the Ambiq boards checked from
these board DTS files:

- ``boards/ambiq/apollo510dL_evb/apollo510dL_evb.dts``
- ``boards/ambiq/apollo330mP_evb/apollo330mP_evb.dts``

Only test paths that exist in this tree are listed here.
Where a test depends on the selected board exposing the required devicetree
nodes, aliases, or sysbuild wiring, the regular ``west build -b <board> ...
-p always`` form is used.

Related Ambiq Docs
******************

- `../doc/ambiq/How_to_Run_MCUBoot_Samples_and_Tests.rst <../doc/ambiq/How_to_Run_MCUBoot_Samples_and_Tests.rst>`_
- `../doc/ambiq/How_to_Run_Zephyr_MSPI_Samples_and_Tests.rst <../doc/ambiq/How_to_Run_Zephyr_MSPI_Samples_and_Tests.rst>`_
- `../doc/ambiq/RELEASE_NOTES.rst <../doc/ambiq/RELEASE_NOTES.rst>`_

Driver Tests
************

Crypto:

.. code-block:: console

   west build -b apollo510dL_evb tests/boards/ambiq/aes_hal_example -p always

GPIO:

.. code-block:: console

   west build -b <board> tests/drivers/gpio/gpio_basic_api -p always

MBOX:

.. code-block:: console

   west build -b <board> tests/drivers/mbox/mbox_data -p always

``tests/drivers/mbox/mbox_data`` uses sysbuild and needs a board with an
enabled mailbox consumer path. In the checked Ambiq DTS files, that applies to
``apollo510dL_evb`` and ``apollo330mP_evb``.

MSPI:

.. code-block:: console

   west build -b apollo510dL_evb tests/drivers/mspi/api -p always

Entropy:

.. code-block:: console

   west build -b apollo510dL_evb tests/drivers/entropy/api -p always
   west build -b apollo330mP_evb tests/drivers/entropy/api -p always

HWINFO:

.. code-block:: console

   west build -b <board> tests/drivers/hwinfo/api -p always

PWM:

.. code-block:: console

   west build -b <board> tests/drivers/pwm/pwm_api -p always

``tests/drivers/pwm/pwm_api`` requires a ``pwm-test`` alias on the selected
board.

SDIO:

.. code-block:: console

   west build -b apollo510dL_evb tests/subsys/sd/sdio -p always
   west build -b apollo330mP_evb tests/subsys/sd/sdio -p always

Benchmark Tests
***************

.. code-block:: console

   west build -b <board> tests/benchmarks/mbedtls -p always

MCUboot, DFU, and Retention
**************************

From the Ambiq MCUboot guide:

.. code-block:: console

   west build -b apollo510dL_evb -p always -d ../build/tests/boot/test_mcuboot ./tests/boot/test_mcuboot/ -T bootloader.mcuboot
   west build -b apollo510dL_evb -p always ./tests/boot/with_mcumgr/ -d ../build/tests/boot/with_mcumgr/test_upgrade -T boot.with_mcumgr.test_upgrade
   west build -b apollo510dL_evb -p always ./tests/boot/with_mcumgr -d ../build/tests/boot/with_mcumgr/test_upgrade -T boot.with_mcumgr.test_upgrade.swap_using_offset
   west build -b apollo510dL_evb -p always ./tests/boot/mcuboot_data_sharing -d ../build/tests/boot/mcuboot_data_sharing -T bootloader.mcuboot.data.sharing
   west build -b apollo510dL_evb -p always ./tests/subsys/settings/retention/ -d ../build/tests/settings-retention
   west build -b apollo510dL_evb -p always ./tests/drivers/retained_mem/api/ -d ../build/tests/drivers/retained_mem/api
   west build -b apollo510dL_evb -p always ./tests/drivers/flash/erase_blocks/ -d ../build/tests/drivers/flash/erase-blocks
   west build -b apollo510dL_evb -p always ./tests/subsys/dfu/img_util -d ../build/tests/dfu/img_util -T dfu.image_util

The MCUboot guide includes ``tests/boot/mcuboot_recovery_retention``, but that
section is still marked ``#TODO`` in the guide, so no command is duplicated
here yet.

Pinctrl Note
************

``tests/drivers/pinctrl/api`` was checked, but its upstream ``testcase.yaml``
currently limits it to ``native_sim`` rather than Ambiq boards, so no Ambiq
``west build -b <board> ...`` command is listed here.
