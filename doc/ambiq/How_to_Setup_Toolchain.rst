How to Set Up the Toolchain
###########################

Ambiq Zephyr releases are built from the ``ambiq-stable`` branch. The release
number is defined in the ``VERSION`` file at the repository root (for example
``4.3.0``).

Release branch and version
**************************

- **Branch:** ``ambiq-stable`` — manufacturing and customer release line.
- **Version file:** ``VERSION`` — kernel version and boot banner string.

The boot banner reports the ``VERSION`` file value, not ``git describe``.
Ambiq fork history does not track upstream release tags as git ancestors, so
git-based version strings would be misleading on ``ambiq-stable``.

Confirm the version::

   cat VERSION
   west build -b apollo510_evb -p always zephyr/samples/hello_world

The serial console boot banner should show::

   *** Booting Zephyr OS build 4.3.0 ***

Zephyr SDK matrix
*****************

+---------------+---------------------------+-----------------------------------+
| Ambiq release | Recommended SDK           | Notes                             |
+===============+===========================+===================================+
| 4.3.x         | Zephyr SDK 0.17.x (host)  | Default ``ZEPHYR_TOOLCHAIN_VARIANT |
|               |                           | = zephyr``                        |
+---------------+---------------------------+-----------------------------------+
| 4.3.x         | GCC 14 from SDK 1.0.x     | Cross-compile only; SDK 1.0.x     |
| (optional)    |                           | cannot be the host Zephyr SDK     |
|               |                           | package for 4.3 builds            |
+---------------+---------------------------+-----------------------------------+
| 4.4.x         | Zephyr SDK 1.0.x          | Set ``ZEPHYR_SDK_INSTALL_DIR`` to |
|               |                           | the 1.0.x install                 |
+---------------+---------------------------+-----------------------------------+

Environment variables
***********************

Set at minimum::

   export ZEPHYR_BASE=/path/to/zephyr
   export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
   export ZEPHYR_SDK_INSTALL_DIR=/path/to/zephyr-sdk-0.17.4

On Windows (PowerShell)::

   $env:ZEPHYR_BASE = "C:\path\to\zephyr"
   $env:ZEPHYR_TOOLCHAIN_VARIANT = "zephyr"
   $env:ZEPHYR_SDK_INSTALL_DIR = "C:\path\to\zephyr-sdk-0.17.4"

Build from the west workspace root (parent of ``zephyr/``)::

   west build -b apollo510_evb -p always zephyr/samples/hello_world

Zephyr 4.3 with GCC 14 (SDK 1.0.x cross-compile)
**************************************************

Zephyr 4.3 cannot load SDK 1.0.x as the host Zephyr SDK package. To use GCC 14
from an SDK 1.0.x install while keeping host tools on SDK 0.17.x, point the
cross-compile toolchain at the 1.0.x GNU prefix and use a pristine build after
changing the environment.

Example (adjust paths to your install locations)::

   export ZEPHYR_SDK_INSTALL_DIR=/path/to/zephyr-sdk-0.17.4
   export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
   export CROSS_COMPILE=/path/to/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-

   west build -b apollo510_evb -p always zephyr/samples/hello_world

CMake should report a cross-compile GNU 14.x toolchain. Always use ``-p
always`` (or delete the build directory) after switching SDK or compiler
profiles so cached toolchain paths are not reused.

Related documentation
*********************

- `README.rst <../../README.rst>`_ — Ambiq Zephyr hub
- `samples/README.rst <../../samples/README.rst>`_ — sample build commands
- `tests/README.rst <../../tests/README.rst>`_ — test build commands
