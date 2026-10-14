/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_BEKEN_BK7258_SOC_H_
#define _SOC_BEKEN_BK7258_SOC_H_

#ifndef _ASMLANGUAGE

#include <zephyr/arch/arm/cortex_m/nvic.h>

/*
 * CMSIS device configuration. The core is an Arm China STAR-MC1, whose CMSIS
 * header adds the L1 cache functions to the Cortex-M33 programmer's model.
 */
typedef enum {
	Reset_IRQn = -15,
	NonMaskableInt_IRQn = -14,
	HardFault_IRQn = -13,
	MemoryManagement_IRQn = -12,
	BusFault_IRQn = -11,
	UsageFault_IRQn = -10,
#if defined(CONFIG_ARM_SECURE_FIRMWARE)
	SecureFault_IRQn = -9,
#endif
	SVCall_IRQn = -5,
	DebugMonitor_IRQn = -4,
	PendSV_IRQn = -2,
	SysTick_IRQn = -1,
	Max_IRQn = CONFIG_NUM_IRQS,
} IRQn_Type;

#define __STAR_REV             0x0000U
#define __NVIC_PRIO_BITS       NUM_IRQ_PRIO_BITS
#define __Vendor_SysTickConfig 0U
#define __MPU_PRESENT          CONFIG_CPU_HAS_ARM_MPU
#define __FPU_PRESENT          CONFIG_CPU_HAS_FPU
#define __DSP_PRESENT          0U
#define __SAUREGION_PRESENT    CONFIG_CPU_HAS_ARM_SAU
#define __ICACHE_PRESENT       CONFIG_CPU_HAS_ICACHE
#define __DCACHE_PRESENT       CONFIG_CPU_HAS_DCACHE
#define __DTCM_PRESENT         0U

#include <core_starmc1.h>

/**
 * @brief Open CPU0's gate for a peripheral interrupt.
 *
 * A peripheral interrupt reaches the NVIC only while its bit in the SYS
 * block's per-CPU gate is set, in addition to being enabled in the NVIC.
 * Bits are numbered as the NVIC numbers the interrupts.
 *
 * @param irq NVIC interrupt number, 0 to 63.
 */
void bk7258_irq_gate_enable(unsigned int irq);

#endif /* !_ASMLANGUAGE */

#endif /* _SOC_BEKEN_BK7258_SOC_H_ */
