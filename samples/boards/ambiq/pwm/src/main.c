/*
 * Copyright (c) 2025 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Ambiq CTimer PWM Output Example
 *
 * This sample demonstrates PWM output using the Ambiq CTimer PWM driver,
 * following the Ambiq SDK ctimer_pwm_output.c example.
 *
 * Three PWM outputs are configured:
 *   PWM A: Timer 0, Segment B, GPIO 13 (CTIM2)  - ~10% duty cycle
 *   PWM B: Timer 2, Segment A, GPIO 29 (CTIM9)  - ~50% duty cycle
 *   PWM C: Timer 7, Segment B, GPIO 11 (CTIM31) - ~70% duty cycle
 *
 * Clock source: 32.768 kHz (XT oscillator)
 * PWM Period: 327 cycles (~100 Hz)
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ambiq_pwm, CONFIG_PWM_LOG_LEVEL);

/*
 * Timer assignments matching ctimer_pwm_output.c:
 *   PWM_TIMER_A = Timer 0, Segment B, OUT1 on GPIO 13 (CTIM2)
 *   PWM_TIMER_B = Timer 2, Segment A, OUT2 on GPIO 29 (CTIM9)
 *   PWM_TIMER_C = Timer 7, Segment B, OUT2 on GPIO 11 (CTIM31)
 */

/* Get PWM devices directly from devicetree */
#define PWM_A_NODE DT_NODELABEL(pwm0)  /* Timer 0, GPIO 13 */
#define PWM_B_NODE DT_NODELABEL(pwm2)  /* Timer 2, GPIO 29 */
#define PWM_C_NODE DT_NODELABEL(pwm7)  /* Timer 7, GPIO 11 */

#if !DT_NODE_HAS_STATUS_OKAY(PWM_A_NODE)
#error "pwm0 node is not defined or enabled"
#endif

#if !DT_NODE_HAS_STATUS_OKAY(PWM_B_NODE)
#error "pwm2 node is not defined or enabled"
#endif

#if !DT_NODE_HAS_STATUS_OKAY(PWM_C_NODE)
#error "pwm7 node is not defined or enabled"
#endif

/* PWM device pointers */
static const struct device *pwm_a_dev = DEVICE_DT_GET(PWM_A_NODE);
static const struct device *pwm_b_dev = DEVICE_DT_GET(PWM_B_NODE);
static const struct device *pwm_c_dev = DEVICE_DT_GET(PWM_C_NODE);

/*
 * PWM configuration matching ctimer_pwm_output.c:
 *   Clock source: XT 32.768 kHz
 *   Period: 327 cycles (~100 Hz)
 *   PWM A duty: 32 cycles  (~10%)
 *   PWM B duty: 163 cycles (~50%)
 *   PWM C duty: 228 cycles (~70%)
 */
#define PWM_CLOCK_HZ      32768U
#define PWM_PERIOD_CYCLES 327U
#define PWM_A_DUTY_CYCLES 32U   /* ~10% */
#define PWM_B_DUTY_CYCLES 163U  /* ~50% */
#define PWM_C_DUTY_CYCLES 228U  /* ~70% */

/* PWM channel (always 0 for single-channel PWM) */
#define PWM_CHANNEL 0U

/**
 * @brief Set PWM output with specified period and duty cycles
 *
 * @param dev PWM device
 * @param name Human-readable name for logging
 * @param period_cycles Period in clock cycles
 * @param duty_cycles Duty (pulse width) in clock cycles
 * @return 0 on success, negative error code otherwise
 */
static int set_pwm_cycles(const struct device *dev, const char *name,
			  uint32_t period_cycles, uint32_t duty_cycles)
{
	int ret;

	ret = pwm_set_cycles(dev, PWM_CHANNEL, period_cycles, duty_cycles, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set %s: %d", name, ret);
		return ret;
	}

	uint32_t duty_percent = (duty_cycles * 100U) / period_cycles;
	printk("%s: period=%u cycles, duty=%u cycles (~%u%%)\n",
	       name, period_cycles, duty_cycles, duty_percent);

	return 0;
}

/**
 * @brief Initialize and verify PWM device
 *
 * @param dev PWM device
 * @param name Human-readable name for logging
 * @return 0 on success, negative error code otherwise
 */
static int init_pwm(const struct device *dev, const char *name)
{
	if (!device_is_ready(dev)) {
		LOG_ERR("PWM device %s (%s) is not ready", dev->name, name);
		return -ENODEV;
	}

	printk("  %s: %s ready\n", name, dev->name);
	return 0;
}

int main(void)
{
	int ret;

	printk("\n");
	printk("*****************************************************\n");
	printk("*         Ambiq CTimer PWM Output Example           *\n");
	printk("*****************************************************\n");
	printk("\n");
	printk("Configuration (matching ctimer_pwm_output.c):\n");
	printk("  PWM A: Timer 0, Segment B, GPIO 13 (CTIM2)\n");
	printk("  PWM B: Timer 2, Segment A, GPIO 29 (CTIM9)\n");
	printk("  PWM C: Timer 7, Segment B, GPIO 11 (CTIM31)\n");
	printk("  Clock: 32.768 kHz, Period: %u cycles (~100 Hz)\n", PWM_PERIOD_CYCLES);
	printk("\n");

	/* Initialize all 3 PWM devices */
	printk("Initializing PWM devices:\n");

	ret = init_pwm(pwm_a_dev, "PWM_A (Timer0/GPIO13)");
	if (ret < 0) {
		return ret;
	}

	ret = init_pwm(pwm_b_dev, "PWM_B (Timer2/GPIO29)");
	if (ret < 0) {
		return ret;
	}

	ret = init_pwm(pwm_c_dev, "PWM_C (Timer7/GPIO11)");
	if (ret < 0) {
		return ret;
	}

	printk("\n");

	/*
	 * Set PWM outputs matching ctimer_pwm_output.c:
	 *   PWM A: 327 period, 32 duty  (~10%)
	 *   PWM B: 327 period, 163 duty (~50%)
	 *   PWM C: 327 period, 228 duty (~70%)
	 */
	printk("Setting PWM outputs:\n");

	ret = set_pwm_cycles(pwm_a_dev, "PWM_A (Timer0/GPIO13)",
			     PWM_PERIOD_CYCLES, PWM_A_DUTY_CYCLES);
	if (ret < 0) {
		return ret;
	}

	ret = set_pwm_cycles(pwm_b_dev, "PWM_B (Timer2/GPIO29)",
			     PWM_PERIOD_CYCLES, PWM_B_DUTY_CYCLES);
	if (ret < 0) {
		return ret;
	}

	ret = set_pwm_cycles(pwm_c_dev, "PWM_C (Timer7/GPIO11)",
			     PWM_PERIOD_CYCLES, PWM_C_DUTY_CYCLES);
	if (ret < 0) {
		return ret;
	}

	printk("\n");
	printk("All 3 PWM outputs are now running.\n");
	printk("Measure PWM signals on GPIO 13, GPIO 29, and GPIO 11.\n");
	printk("\n");

	/*
	 * In the original ctimer_pwm_output.c, the system goes to deep sleep
	 * and the PWMs continue running. The timer ISR can be used to
	 * dynamically update duty cycles.
	 *
	 * Here we simply keep the system running with the configured PWMs.
	 */
	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
