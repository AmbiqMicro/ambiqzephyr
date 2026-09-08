.. raw:: html

   <a href="https://www.zephyrproject.org">
     <p align="center">
       <picture>
         <img src="doc/_static/images/Zephyr-support-for-Ambiq.svg">
       </picture>
     </p>
   </a>

   <a href="https://bestpractices.coreinfrastructure.org/projects/74"><img src="https://bestpractices.coreinfrastructure.org/projects/74/badge"></a>
   <a href="https://scorecard.dev/viewer/?uri=github.com/zephyrproject-rtos/zephyr"><img src="https://api.securityscorecards.dev/projects/github.com/zephyrproject-rtos/zephyr/badge"></a>
   <a href="https://github.com/zephyrproject-rtos/zephyr/actions/workflows/twister.yaml?query=branch%3Amain"><img src="https://github.com/zephyrproject-rtos/zephyr/actions/workflows/twister.yaml/badge.svg?event=push"></a>

Release: 8 September 2026
#########################

**v4.3.0-ambiq.v1.0.1** is a maintenance release of the Ambiq Zephyr
distribution. It documents validated **Bluetooth** support across the Apollo3,
Apollo4 and Apollo5 EVBs, points the Ambiq HAL at its public repository, and
adds Apollo2 UART power management. The release is based on upstream Zephyr
v4.3.0.

See the `release notes`_ for the full detail, organised by chipset family:
Apollo510L/330P, Apollo510/510B, Apollo4x, Apollo3x and Apollo2.

Initialise a workspace from the ``ambiq-stable`` branch:

.. code-block:: console

   west init -m https://github.com/AmbiqMicro/ambiqzephyr --mr ambiq-stable
   west update

The previous line is frozen at tag ``ambiq-stable-pre-v1.0.0`` and remains
available at branch ``frozen/ambiq-stable-pre-v1.0.0-2026-07``. It receives no
further updates.

Known Limitations
#################

- Bluetooth Classic is not yet supported on Apollo510 Lite or
  Apollo330 Plus.

About Ambiq
###########

Ambiq is an Austin-based semiconductor company with a mission to enable intelligence (artificial intelligence (AI) and beyond) everywhere by delivering the lowest power semiconductor solutions. Built on its patented Subthreshold Power Optimized Technology (SPOT®) and the HELIA™ AI platform, Ambiq empowers manufacturers to bring more capable AI to the edge, where power, memory, and energy efficiency are most critical.

Built for intelligent, always-on edge devices across healthcare, wearables, industrial automation, smart environments, and other emerging AI applications, Ambiq continues to shape the future of always-on Edge AI globally. For more information, visit www.ambiq.com.

About Zephyr
############

The `Zephyr Project`_ is a scalable, real-time operating system supporting
multiple hardware architectures, optimized for resource constrained devices and
built with security in mind. It is an open source project hosted by the Linux
Foundation, developed in the open by a broad community of vendors and
individual contributors.

Zephyr provides a small-footprint kernel, a unified device driver model, a
devicetree-based hardware description, a configurable network and connectivity
stack, and a common build and test tooling (``west`` and ``twister``) shared
across every supported platform.

About This Repository
#####################

This repository is Ambiq's distribution of Zephyr. It tracks the upstream
Zephyr Project and adds the SoC support, drivers, boards, samples, and
documentation for Ambiq Apollo devices.

- **Release branch:** ``ambiq-stable`` is the manufacturing and customer
  release line. The release number lives in the ``VERSION`` file at the
  repository root and is what the boot banner reports.
- **Everything else** — supported peripherals, boards, build and flash
  commands, power management guidance — is documented under
  `doc/ambiq/ <doc/ambiq/>`_.

.. code-block:: console

   git checkout ambiq-stable

Getting Started
###############

New to Zephyr? Start with the `Introduction to Zephyr`_ for a high-level
overview, then follow the `Getting Started Guide`_ to install the dependencies
and set up a ``west`` workspace.

Once the workspace is in place, continue with the Ambiq documentation below for
toolchain versions, board support, and build commands.

Ambiq Documentation
###################

  | 🧭 `Ambiq Documentation Index <doc/ambiq/README.rst>`_ — start here
  | 🧰 `Toolchain Setup <doc/ambiq/How_to_Setup_Toolchain.rst>`_
  | 🧩 `Supported Features <doc/ambiq/Supported_Features.rst>`_ — drivers, power management, third-party libraries
  | 🔨 `Build and Flash <doc/ambiq/How_to_Build_and_Flash.rst>`_
  | ⚡ `Power Management <doc/ambiq/Power_Management.rst>`_
  | 📡 `Bluetooth <doc/ambiq/Bluetooth.rst>`_ — validated support per board
  | 💻 `Sample Build Commands <samples/README.rst>`_
  | 🧪 `Test Build Commands <tests/README.rst>`_
  | 📝 `Release Notes <https://github.com/AmbiqMicro/ambiqzephyr/releases>`_

For Apollo SoC datasheets, errata, and the AmbiqSuite SDK, see the
`Ambiq Content Portal`_ and `Ambiq Products`_.

.. start_include_here

Community Support
#################

Community support is provided via mailing lists and Discord; see the Resources
below for details.

.. _project-resources:

Resources
#########

Here's a quick summary of resources to help you find your way around:

Getting Started
***************

  | 📖 `Zephyr Documentation`_
  | 🚀 `Getting Started Guide`_
  | 🙋🏽 `Tips when asking for help`_
  | 💻 `Code samples`_

Code and Development
********************

  | 🌐 `Source Code Repository`_
  | 🌐 `Ambiq HAL Repository`_
  | 📦 `Releases`_
  | 🤝 `Contribution Guide`_

Community and Support
*********************

  | 💬 `Discord Server`_ for real-time community discussions
  | 📧 `User mailing list (users@lists.zephyrproject.org)`_
  | 📧 `Developer mailing list (devel@lists.zephyrproject.org)`_
  | 📬 `Other project mailing lists`_
  | 📚 `Project Wiki`_

Issue Tracking and Security
***************************

  | 🐛 `GitHub Issues`_
  | 🔒 `Security documentation`_
  | 🛡️ `Security Advisories Repository`_
  | ⚠️ Report security vulnerabilities at vulnerabilities@zephyrproject.org

Additional Resources
********************

  | 🌐 `Zephyr Project Website`_
  | 📺 `Zephyr Tech Talks`_

.. _Zephyr Project: https://www.zephyrproject.org
.. _Zephyr Project Website: https://www.zephyrproject.org
.. _Discord Server: https://chat.zephyrproject.org
.. _Zephyr Documentation: https://docs.zephyrproject.org
.. _Introduction to Zephyr: https://docs.zephyrproject.org/latest/introduction/index.html
.. _Getting Started Guide: https://docs.zephyrproject.org/latest/develop/getting_started/index.html
.. _Contribution Guide: https://docs.zephyrproject.org/latest/contribute/index.html
.. _Source Code Repository: https://github.com/AmbiqMicro/ambiqzephyr
.. _GitHub Issues: https://github.com/AmbiqMicro/ambiqzephyr/issues
.. _Releases: https://github.com/zephyrproject-rtos/zephyr/releases
.. _Project Wiki: https://github.com/zephyrproject-rtos/zephyr/wiki
.. _User mailing list (users@lists.zephyrproject.org): https://lists.zephyrproject.org/g/users
.. _Developer mailing list (devel@lists.zephyrproject.org): https://lists.zephyrproject.org/g/devel
.. _Other project mailing lists: https://lists.zephyrproject.org/g/main/subgroups
.. _Code samples: https://docs.zephyrproject.org/latest/samples/index.html
.. _Security documentation: https://docs.zephyrproject.org/latest/security/index.html
.. _Security Advisories Repository: https://github.com/zephyrproject-rtos/zephyr/security
.. _Tips when asking for help: https://docs.zephyrproject.org/latest/develop/getting_started/index.html#asking-for-help
.. _Zephyr Tech Talks: https://www.zephyrproject.org/tech-talks
.. _Ambiq Products: https://ambiq.com/products/
.. _Ambiq Content Portal: https://contentportal.ambiq.com/
.. _Ambiq HAL Repository: https://github.com/AmbiqMicro/hal_ambiq_internal

.. _release notes:
   https://github.com/AmbiqMicro/ambiqzephyr/releases
