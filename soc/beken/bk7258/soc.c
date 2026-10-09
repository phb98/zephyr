/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <bk7258_aon.h>
#include <soc.h>

/*
 * Addresses are the secure aliases, which is what the BootROM leaves the core
 * running in. The non-secure alias of any peripheral is at +0x10000000.
 */
#define BK7258_WDT_BASE 0x44800000

/* CPU0's interrupt gate: sources 0 to 31, then 32 to 63 */
#define BK7258_SYS_CPU0_INT_EN 0x44010080

/* NMI watchdog control, laid out as the AON watchdog's */
#define BK7258_WDT_CTRL_OFFSET 0x10

/* NMI watchdog global control */
#define BK7258_WDT_GLOBAL_CTRL_OFFSET   0x08
#define BK7258_WDT_CLK_GATE_BYPASS      BIT(1)

/*
 * Stop both watchdogs. They are running when the BootROM hands over: left
 * alone, the NMI watchdog raises an NMI about 2.5 s into boot and the AON
 * watchdog resets the chip about 45 s in. Writing a period of zero with the
 * two-key sequence disables a watchdog.
 */
static void watchdog_disable(void)
{
	const uint32_t key_1st = FIELD_PREP(BK7258_AON_WDT_CTRL_KEY, BK7258_AON_WDT_KEY_1ST);
	const uint32_t key_2nd = FIELD_PREP(BK7258_AON_WDT_CTRL_KEY, BK7258_AON_WDT_KEY_2ND);

	sys_write32(key_1st, BK7258_AON_WDT_CTRL);
	sys_write32(key_2nd, BK7258_AON_WDT_CTRL);

	sys_set_bits(BK7258_WDT_BASE + BK7258_WDT_GLOBAL_CTRL_OFFSET,
		     BK7258_WDT_CLK_GATE_BYPASS);
	sys_write32(key_1st, BK7258_WDT_BASE + BK7258_WDT_CTRL_OFFSET);
	sys_write32(key_2nd, BK7258_WDT_BASE + BK7258_WDT_CTRL_OFFSET);
}

static struct k_spinlock irq_gate_lock;

void bk7258_irq_gate_enable(unsigned int irq)
{
	k_spinlock_key_t key;

	__ASSERT_NO_MSG(irq < CONFIG_NUM_IRQS);

	key = k_spin_lock(&irq_gate_lock);
	sys_set_bits(BK7258_SYS_CPU0_INT_EN + ((irq / 32U) * 4U), BIT(irq % 32U));
	k_spin_unlock(&irq_gate_lock, key);
}

void soc_early_init_hook(void)
{
	watchdog_disable();

	/* The BootROM hands over with the caches off; the MPU already keeps SRAM uncached */
	sys_cache_instr_enable();
	sys_cache_data_enable();
}
