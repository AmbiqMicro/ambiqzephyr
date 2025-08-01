/*
 * Copyright (c) 2016 Cadence Design Systems, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_XTENSA_XTENSA_IRQ_H_
#define ZEPHYR_INCLUDE_ARCH_XTENSA_XTENSA_IRQ_H_

#include <stdint.h>

#include <zephyr/toolchain.h>
#include <xtensa/config/core-isa.h>

#define CONFIG_GEN_IRQ_START_VECTOR 0

/**
 * @cond INTERNAL_HIDDEN
 */

/*
 * Call these functions to enable the specified interrupts.
 *
 * mask     - Bit mask of interrupts to be enabled.
 */
static inline void z_xt_ints_on(unsigned int mask)
{
	int val;

	__asm__ volatile("rsr.intenable %0" : "=r"(val));
	val |= mask;
	__asm__ volatile("wsr.intenable %0; rsync" : : "r"(val));
}
#if XCHAL_NUM_INTERRUPTS > 32
static inline void z_xt_ints1_on(unsigned int mask)
{
	int val;

	__asm__ volatile("rsr.intenable1 %0" : "=r"(val));
	val |= mask;
	__asm__ volatile("wsr.intenable1 %0; rsync" : : "r"(val));
}
#endif
#if XCHAL_NUM_INTERRUPTS > 64
static inline void z_xt_ints2_on(unsigned int mask)
{
	int val;

	__asm__ volatile("rsr.intenable2 %0" : "=r"(val));
	val |= mask;
	__asm__ volatile("wsr.intenable2 %0; rsync" : : "r"(val));
}
#endif
#if XCHAL_NUM_INTERRUPTS > 96
static inline void z_xt_ints3_on(unsigned int mask)
{
	int val;

	__asm__ volatile("rsr.intenable3 %0" : "=r"(val));
	val |= mask;
	__asm__ volatile("wsr.intenable3 %0; rsync" : : "r"(val));
}
#endif


/*
 * Call these functions to disable the specified interrupts.
 *
 * mask     - Bit mask of interrupts to be disabled.
 */
static inline void z_xt_ints_off(unsigned int mask)
{
	int val;

	__asm__ volatile("rsr.intenable %0" : "=r"(val));
	val &= ~mask;
	__asm__ volatile("wsr.intenable %0; rsync" : : "r"(val));
}
#if XCHAL_NUM_INTERRUPTS > 32
static inline void z_xt_ints1_off(unsigned int mask)
{
	int val;

	__asm__ volatile("rsr.intenable1 %0" : "=r"(val));
	val &= ~mask;
	__asm__ volatile("wsr.intenable1 %0; rsync" : : "r"(val));
}
#endif
#if XCHAL_NUM_INTERRUPTS > 64
static inline void z_xt_ints2_off(unsigned int mask)
{
	int val;

	__asm__ volatile("rsr.intenable2 %0" : "=r"(val));
	val &= ~mask;
	__asm__ volatile("wsr.intenable2 %0; rsync" : : "r"(val));
}
#endif
#if XCHAL_NUM_INTERRUPTS > 96
static inline void z_xt_ints3_off(unsigned int mask)
{
	int val;

	__asm__ volatile("rsr.intenable3 %0" : "=r"(val));
	val &= ~mask;
	__asm__ volatile("wsr.intenable3 %0; rsync" : : "r"(val));
}
#endif


/*
 * Call these functions to set the specified (s/w) interrupt.
 */
static inline void z_xt_set_intset(unsigned int arg)
{
#if XCHAL_HAVE_INTERRUPTS
	__asm__ volatile("wsr.intset %0; rsync" : : "r"(arg));
#else
	ARG_UNUSED(arg);
#endif
}
#if XCHAL_NUM_INTERRUPTS > 32
static inline void z_xt_set_intset1(unsigned int arg)
{
	__asm__ volatile("wsr.intset1 %0; rsync" : : "r"(arg));
}
#endif
#if XCHAL_NUM_INTERRUPTS > 64
static inline void z_xt_set_intset2(unsigned int arg)
{
	__asm__ volatile("wsr.intset2 %0; rsync" : : "r"(arg));
}
#endif
#if XCHAL_NUM_INTERRUPTS > 96
static inline void z_xt_set_intset3(unsigned int arg)
{
	__asm__ volatile("wsr.intset3 %0; rsync" : : "r"(arg));
}
#endif


/**
 * INTERNAL_HIDDEN @endcond
 */

#ifdef CONFIG_MULTI_LEVEL_INTERRUPTS

/* for _soc_irq_*() */
#include <soc.h>

#ifdef CONFIG_2ND_LEVEL_INTERRUPTS
#ifdef CONFIG_3RD_LEVEL_INTERRUPTS
#define CONFIG_NUM_IRQS (XCHAL_NUM_INTERRUPTS +\
			(CONFIG_NUM_2ND_LEVEL_AGGREGATORS +\
			CONFIG_NUM_3RD_LEVEL_AGGREGATORS) *\
			CONFIG_MAX_IRQ_PER_AGGREGATOR)
#else
#define CONFIG_NUM_IRQS (XCHAL_NUM_INTERRUPTS +\
			CONFIG_NUM_2ND_LEVEL_AGGREGATORS *\
			CONFIG_MAX_IRQ_PER_AGGREGATOR)
#endif /* CONFIG_3RD_LEVEL_INTERRUPTS */
#else
#define CONFIG_NUM_IRQS XCHAL_NUM_INTERRUPTS
#endif /* CONFIG_2ND_LEVEL_INTERRUPTS */

void z_soc_irq_init(void);
void z_soc_irq_enable(unsigned int irq);
void z_soc_irq_disable(unsigned int irq);
int z_soc_irq_is_enabled(unsigned int irq);

#define arch_irq_enable(irq)	z_soc_irq_enable(irq)
#define arch_irq_disable(irq)	z_soc_irq_disable(irq)

#define arch_irq_is_enabled(irq)	z_soc_irq_is_enabled(irq)

#ifdef CONFIG_DYNAMIC_INTERRUPTS
extern int z_soc_irq_connect_dynamic(unsigned int irq, unsigned int priority,
				     void (*routine)(const void *parameter),
				     const void *parameter, uint32_t flags);
#endif

#else

#define CONFIG_NUM_IRQS XCHAL_NUM_INTERRUPTS

#define arch_irq_enable(irq)	xtensa_irq_enable(irq)
#define arch_irq_disable(irq)	xtensa_irq_disable(irq)

#define arch_irq_is_enabled(irq)	xtensa_irq_is_enabled(irq)

#endif

/**
 * @brief Enable interrupt on Xtensa core.
 *
 * @param irq Interrupt to be enabled.
 */
static ALWAYS_INLINE void xtensa_irq_enable(uint32_t irq)
{
#if XCHAL_NUM_INTERRUPTS > 32
	switch (irq >> 5) {
	case 0:
		z_xt_ints_on(1 << irq);
		break;
	case 1:
		z_xt_ints1_on(1 << irq);
		break;
#if XCHAL_NUM_INTERRUPTS > 64
	case 2:
		z_xt_ints2_on(1 << irq);
		break;
#endif
#if XCHAL_NUM_INTERRUPTS > 96
	case 3:
		z_xt_ints3_on(1 << irq);
		break;
#endif
	default:
		break;
	}
#else
	z_xt_ints_on(1 << irq);
#endif
}

/**
 * @brief Disable interrupt on Xtensa core.
 *
 * @param irq Interrupt to be disabled.
 */
static ALWAYS_INLINE void xtensa_irq_disable(uint32_t irq)
{
#if XCHAL_NUM_INTERRUPTS > 32
	switch (irq >> 5) {
	case 0:
		z_xt_ints_off(1 << irq);
		break;
	case 1:
		z_xt_ints1_off(1 << irq);
		break;
#if XCHAL_NUM_INTERRUPTS > 64
	case 2:
		z_xt_ints2_off(1 << irq);
		break;
#endif
#if XCHAL_NUM_INTERRUPTS > 96
	case 3:
		z_xt_ints3_off(1 << irq);
		break;
#endif
	default:
		break;
	}
#else
	z_xt_ints_off(1 << irq);
#endif
}

/** Implementation of @ref arch_irq_lock. */
static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	unsigned int key;

	__asm__ volatile("rsil %0, %1"
			 : "=r"(key) : "i"(XCHAL_EXCM_LEVEL) : "memory");
	return key;
}

/** Implementation of @ref arch_irq_unlock. */
static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
	__asm__ volatile("wsr.ps %0; rsync"
			 :: "r"(key) : "memory");
}

/** Implementation of @ref arch_irq_unlocked. */
static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
	return (key & 0xf) == 0; /* INTLEVEL field */
}

/**
 * @brief Query if an interrupt is enabled on Xtensa core.
 *
 * @param irq Interrupt to be queried.
 *
 * @return True if interrupt is enabled, false otherwise.
 */
int xtensa_irq_is_enabled(unsigned int irq);

#include <zephyr/irq.h>

#endif /* ZEPHYR_INCLUDE_ARCH_XTENSA_XTENSA_IRQ_H_ */
