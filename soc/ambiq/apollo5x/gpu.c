/*
 * Copyright (c) 2025 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_nema_gpu

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>

#include "nema_regs.h"

#include "gpu.h"
#include <zephyr/sys/util.h>
#include <zephyr/cache.h>

LOG_MODULE_REGISTER(gpu_ambiq, CONFIG_GPU_AMBIQ_LOG_LEVEL);

/* The driver tracks submitted command list addresses/sizes and performs
 * explicit cache maintenance for GPU command lists: it flushes the command
 * list on submission (when programming NEMA_CMDSIZE) and invalidates cached
 * data when command list completion is observed.
 */

/** @brief Ambiq GPU driver private data structure. */
struct gpu_ambiq_data {
	/** @brief ID of the last completed command list. */
	volatile int last_cl_id;
	/** @brief Semaphore signaled by the ISR upon command list completion. */
	struct k_sem gpu_isr_sem;
	/** @brief In-flight submission limiter (0 = idle, 1 = busy). */
	atomic_t inflight;
	/** @brief Count of consecutive semaphore timeouts. */
	int consecutive_timeouts;
	/** @brief Lower/upper address of pending command list (NEMA_CMDADDR write). */
	uint64_t pending_cmdaddr;
	/** @brief Address of the last submitted command list. */
	uint64_t last_cmdaddr;
	/** @brief Size of the last submitted command list. */
	size_t last_cmdsize;
};

/** @brief Ambiq GPU driver configuration data. */
struct gpu_ambiq_config {
	/** @brief GPU register map base address. */
	uint32_t base;
	/** @brief Size of the GPU register map. */
	uint32_t size;
	/** @brief GPU interrupt number. */
	uint32_t interrupt;
	/** @brief Interrupt configuration function. */
	void (*irq_config_func)(void);
	/** @brief Flag to enable high performance mode. */
	bool high_performance;
};

#ifdef CONFIG_PM_DEVICE
/**
 * @brief Power management action handler for the GPU device.
 *
 * This function is called by the Zephyr PM subsystem to transition the GPU
 * between power states (e.g., resuming and suspending).
 *
 * @param dev Pointer to the GPU device structure.
 * @param action The requested power management action.
 * @return 0 on success, or a negative error code on failure.
 */
static int gpu_ambiq_pm_action(const struct device *dev, enum pm_device_action action)
{
	uint32_t ambiq_hal_status;
	int ret = 0;
	bool gpu_enabled;
	struct gpu_ambiq_data *data = dev->data;

	/* Check the current GPU power state. */
	ambiq_hal_status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_GFX, &gpu_enabled);
	if (ambiq_hal_status != AM_HAL_STATUS_SUCCESS) {
		LOG_ERR("Failed to get GPU power status: 0x%08X", ambiq_hal_status);
		return -EIO;
	}

	/* Decode the requested power state and take action. */
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		if (!gpu_enabled) {
			/* Power on the GPU peripheral. */
			ambiq_hal_status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);
			if (ambiq_hal_status != AM_HAL_STATUS_SUCCESS) {
				LOG_ERR("GPU power on failed: 0x%08X", ambiq_hal_status);
				ret = -EIO;
				break;
			}

			data->last_cl_id = -1;
		}
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		if (gpu_enabled) {
			/* Ensure GPU is idle before suspending. */
			if (gpu_ambiq_reg_read(dev, NEMA_STATUS) != 0) {
				LOG_WRN("GPU suspend request ignored: GPU is busy.");
				ret = -EBUSY;
			} else {
				/* Power down the GPU peripheral. */
				ambiq_hal_status =
					am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_GFX);
				if (ambiq_hal_status != AM_HAL_STATUS_SUCCESS) {
					LOG_ERR("GPU power down failed: 0x%08X", ambiq_hal_status);
					ret = -EIO;
				}
			}
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

#endif /* CONFIG_PM_DEVICE */

/**
 * @brief Interrupt Service Routine for the GPU.
 *
 * This ISR is triggered upon completion of a GPU command list. It clears the
 * interrupt, updates the last completed command list ID, and signals the
 * waiting thread.
 *
 * @param arg Pointer to the GPU device structure.
 */
static void gpu_ambiq_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct gpu_ambiq_data *data = dev->data;
	const struct gpu_ambiq_config *config = dev->config;

	int current_cl_id = (int)gpu_ambiq_reg_read(dev, NEMA_CLID);
	int previous_cl_id;

	do {
		/* Clear the GPU hardware interrupt flag. */
		gpu_ambiq_reg_write(dev, NEMA_INTERRUPT, 0);

		/* Clear the corresponding interrupt in the NVIC. */
		NVIC_ClearPendingIRQ(config->interrupt);

		previous_cl_id = current_cl_id;

		/*
		 * Read the CLID register again to handle a corner case where a new
		 * interrupt triggers after reading NEMA_CLID but before clearing
		 * NEMA_INTERRUPT.
		 */
		current_cl_id = (int)gpu_ambiq_reg_read(dev, NEMA_CLID);
	} while (current_cl_id != previous_cl_id);

	/* Atomically update the last command list ID and signal the semaphore. */
	data->last_cl_id = current_cl_id;
	k_sem_give(&data->gpu_isr_sem);
}

/**
 * @brief Reads a GPU register.
 *
 * @param dev Pointer to the GPU device structure.
 * @param reg Register offset from the GPU base address.
 * @return The value of the register.
 */
uint32_t gpu_ambiq_reg_read(const struct device *dev, uint32_t reg)
{
	const struct gpu_ambiq_config *config = dev->config;

	__ASSERT(reg < config->size, "Register offset out of bounds");

	volatile uint32_t *ptr = (volatile uint32_t *)(config->base + reg);
	return *ptr;
}

/**
 * @brief Writes a value to a GPU register.
 *
 * @param dev Pointer to the GPU device structure.
 * @param reg Register offset from the GPU base address.
 * @param value The value to write to the register.
 */
void gpu_ambiq_reg_write(const struct device *dev, uint32_t reg, uint32_t value)
{
	const struct gpu_ambiq_config *config = dev->config;
	struct gpu_ambiq_data *data = dev->data;

	__ASSERT(reg < config->size, "Register offset out of bounds");

	volatile uint32_t *ptr = (volatile uint32_t *)(config->base + reg);
	*ptr = value;

	/* Heuristic: if the nemagfx glue writes the command list address and size
	 * via the NEMA_CMDADDR / NEMA_CMDSIZE registers, flush the corresponding
	 * CPU cache range so the GPU/DMA sees up-to-date data.
	 */
	switch (reg) {
	case NEMA_CMDADDR:
		/* lower 32 bits of command list address */
		data->pending_cmdaddr =
			(data->pending_cmdaddr & 0xffffffff00000000ULL) | (uint32_t)value;
		break;
	case NEMA_CMDADDR_H:
		/* high 32 bits of command list address */
		data->pending_cmdaddr =
			(data->pending_cmdaddr & 0x00000000ffffffffULL) | ((uint64_t)value << 32);
		break;
	case NEMA_CMDSIZE:
		if (data->pending_cmdaddr != 0ULL && value != 0) {
			void *addr = (void *)(uintptr_t)data->pending_cmdaddr;
			size_t size = (size_t)value;
			/* Remember the submitted CL for post-completion invalidate. */
			data->last_cmdaddr = data->pending_cmdaddr;
			data->last_cmdsize = size;
			LOG_DBG("Flushing GPU command list at %p size %zu", addr, size);
			gpu_ambiq_cache_flush_range(addr, size);
		}
		/* clear pending address after use to avoid accidental reuse */
		data->pending_cmdaddr = 0ULL;
		break;
	default:
		break;
	}
}

/**
 * @brief Gets the ID of the last command list completed by the GPU.
 *
 * @param dev Pointer to the GPU device structure.
 * @return The last completed command list ID.
 */
int gpu_ambiq_get_last_cl_id(const struct device *dev)
{
	struct gpu_ambiq_data *data = dev->data;

	return data->last_cl_id;
}

/**
 * @brief Resets the ID of the last command list to its default value.
 *
 * @param dev Pointer to the GPU device structure.
 */
void gpu_ambiq_reset_last_cl_id(const struct device *dev)
{
	struct gpu_ambiq_data *data = dev->data;

	data->last_cl_id = -1;
}

/**
 * @brief Initializes the Ambiq GPU driver.
 *
 * This function sets up the GPU power mode, configures and enables interrupts,
 * and initializes the NemaGFX and NemaVG SDKs.
 *
 * @param dev Pointer to the GPU device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int gpu_ambiq_init(const struct device *dev)
{
	struct gpu_ambiq_data *data = dev->data;
	const struct gpu_ambiq_config *config = dev->config;

	/* Set performance mode based on devicetree property. */
	if (config->high_performance) {
		am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_GFX);

		/* Switch to High Performance (HP) mode. */
		am_hal_pwrctrl_gpu_mode_e current_mode;

		am_hal_pwrctrl_gpu_mode_select(AM_HAL_PWRCTRL_GPU_MODE_HIGH_PERFORMANCE);
		am_hal_pwrctrl_gpu_mode_status(&current_mode);
		if (AM_HAL_PWRCTRL_GPU_MODE_HIGH_PERFORMANCE != current_mode) {
			LOG_ERR("Failed to switch GPU to High Performance mode.");
			return -EIO;
		}
	}

	/* Ensure GPU peripheral is powered on. */
	am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);

	/* Initialize GPU hardware registers. */
	gpu_ambiq_reg_write(dev, NEMA_INTERRUPT, 0);

	/* Configure and enable the GPU interrupt. */
	NVIC_ClearPendingIRQ(config->interrupt);
	config->irq_config_func();

	data->last_cl_id = -1;

	return 0;
}

/**
 * @brief Waits for a GPU interrupt to occur.
 *
 * This function blocks until the GPU ISR signals the completion of a
 * command list.
 *
 * @param dev Pointer to the GPU device structure.
 * @param timeout_ms Timeout in milliseconds to wait for the interrupt.
 * @return 0 on success, -ETIMEDOUT if the wait timed out.
 */
int gpu_ambiq_wait_interrupt(const struct device *dev, uint32_t timeout_ms)
{
	struct gpu_ambiq_data *data = dev->data;
	int ret = 0;

	/* Acquire the single in-flight slot atomically to avoid races between
	 * concurrent callers. Honor the caller-provided timeout while waiting
	 * for the slot so callers with a finite timeout can't be blocked
	 * indefinitely by another thread that holds the slot.
	 */
	uint32_t start_ms = k_uptime_get_32();
	uint32_t elapsed_ms = 0;
	bool have_inflight = false;

	if (timeout_ms == 0) {
		/* No waiting allowed: attempt once and fail immediately on contention. */
		if (!atomic_cas(&data->inflight, 0, 1)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		have_inflight = true;
	} else {
		while (true) {
			if (atomic_cas(&data->inflight, 0, 1)) {
				have_inflight = true;
				break;
			}
			elapsed_ms = k_uptime_get_32() - start_ms;
			if (elapsed_ms >= timeout_ms) {
				ret = -ETIMEDOUT;
				goto out;
			}
			k_msleep(1);
		}
	}

	/* Compute remaining time for the semaphore wait after acquiring slot. */
	elapsed_ms = k_uptime_get_32() - start_ms;
	uint32_t rem_ms = (elapsed_ms >= timeout_ms) ? 0U : (timeout_ms - elapsed_ms);

	/* Pend on the semaphore, waiting for the ISR to signal completion. */
	if (k_sem_take(&data->gpu_isr_sem, K_MSEC(rem_ms)) != 0) {
		LOG_ERR("Timed out waiting for GPU interrupt.");
		/* Update timeout counter and clear tracked command-list state so a
		 * later successful call cannot invalidate a stale/reused region.
		 */
		data->consecutive_timeouts++;
		data->last_cmdaddr = 0ULL;
		data->last_cmdsize = 0;
		data->pending_cmdaddr = 0ULL;
		ret = -ETIMEDOUT;
		goto out;
	}

	/* Capture recent timeout count before clearing it so the backoff
	 * decision below reflects the history of this submission sequence.
	 */
	unsigned int recent_timeouts = data->consecutive_timeouts;

	data->consecutive_timeouts = 0;

	/* If we have a last-submitted command list region recorded, invalidate
	 * it so the CPU sees any GPU/DMA results written back to memory.
	 */
	if (data->last_cmdaddr != 0ULL && data->last_cmdsize != 0) {
		void *addr = (void *)(uintptr_t)data->last_cmdaddr;
		size_t size = data->last_cmdsize;

		LOG_DBG("Invalidating GPU command list at %p size %zu", addr, size);
		gpu_ambiq_cache_invalidate_range(addr, size);
		data->last_cmdaddr = 0ULL;
		data->last_cmdsize = 0;
	}

	/* Only add backoff delay when there have been recent timeouts; otherwise
	 * a plain yield is sufficient to let the scheduler run without adding
	 * fixed latency to every command-list submission.
	 */
	if (recent_timeouts > 0) {
		k_msleep(2);
	} else {
		k_yield();
	}

out:
	/* Ensure the in-flight slot is released only if this caller acquired it. */
	if (have_inflight) {
		atomic_set(&data->inflight, 0);
	}

	return ret;
}

/**
 * @brief Flush (clean) a data cache range so GPU (DMA) sees CPU-written data.
 *
 * This helper uses the Zephyr cache API and is intentionally lightweight; callers
 * (e.g., nemagfx glue) should flush command lists and buffer ranges right
 * before submitting them to the GPU.
 */
void gpu_ambiq_cache_flush_range(void *addr, size_t size)
{
	if (addr == NULL || size == 0) {
		return;
	}

	(void)sys_cache_data_flush_range(addr, size);
}

/**
 * @brief Invalidate a data cache range so CPU reads reflect GPU-written data.
 *
 * Call this after the GPU has completed a DMA write to a buffer.
 */
void gpu_ambiq_cache_invalidate_range(void *addr, size_t size)
{
	if (addr == NULL || size == 0) {
		return;
	}

	(void)sys_cache_data_invd_range(addr, size);
}

/**
 * @brief Macro for creating and initializing a GPU device instance.
 *
 * @param n The instance number of the device.
 */
#define GPU_AMBIQ_INIT(n)                                                                          \
	static struct gpu_ambiq_data gpu_ambiq_data##n = {                                         \
		.last_cl_id = -1,                                                                  \
		.gpu_isr_sem = Z_SEM_INITIALIZER(gpu_ambiq_data##n.gpu_isr_sem, 0, 1),             \
	};                                                                                         \
	static void gpu_irq_config_func_##n(void)                                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), gpu_ambiq_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};                                                                                         \
	static const struct gpu_ambiq_config gpu_ambiq_config##n = {                               \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.size = DT_INST_REG_SIZE(n),                                                       \
		.interrupt = DT_INST_IRQN(n),                                                      \
		.high_performance = DT_INST_PROP_OR(n, high_performance_mode, 0),                  \
		.irq_config_func = gpu_irq_config_func_##n,                                        \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(n, gpu_ambiq_pm_action);                                          \
	DEVICE_DT_INST_DEFINE(n, &gpu_ambiq_init, PM_DEVICE_DT_INST_GET(n), &gpu_ambiq_data##n,    \
			      &gpu_ambiq_config##n, POST_KERNEL, CONFIG_GPU_AMBIQ_INIT_PRIORITY,   \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(GPU_AMBIQ_INIT)
