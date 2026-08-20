Power Management
################

To achieve lowest power consumption, customer needs to follow the following process for inspection and configuration optimization:

1. According to the acutal usage of memory, adjust the memory configurations in soc_early_init_hook;

2. Make sure ``CONFIG_PM=y``, ``CONFIG_PM_DEVICE=y``, and ``CONFIG_PM_DEVICE_RUNTIME=y``, and explicitly enable ``CONFIG_PM_DEVICE_SYSTEM_MANAGED=y``
   so remaining devices are suspended automatically before deep sleep.

3. Audit the Devicetree to disable unused peripherals ``status = "disabled"`` or keep ``zephyr,pm-device-runtime-auto`` on blocks that should be power-managed automatically;
   remove the property from peripherals that must stay on.

4. In application code, call pm_device_runtime_put() / pm_device_runtime_get() around peripherals that should power-cycle on demand to ensure their usage count returns to zero after use.

Check `Zephyr Power Management <https://docs.zephyrproject.org/latest/services/pm/index.html>`_ for more detailed information.

Per-driver power management status is listed in
`Supported_Features.rst <Supported_Features.rst>`_.
