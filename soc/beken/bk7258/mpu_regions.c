/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/mpu/arm_mpu_mem_cfg.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

/*
 * The flash window is cacheable, which serves code and read-only data from
 * the L1 caches. SRAM is not, so that bus masters sharing it see what the
 * CPU wrote without cache maintenance.
 */
static const struct arm_mpu_region mpu_regions[] = {
	MPU_REGION_ENTRY("FLASH", CONFIG_FLASH_BASE_ADDRESS,
			 REGION_FLASH_ATTR(CONFIG_FLASH_BASE_ADDRESS, KB(CONFIG_FLASH_SIZE))),
	MPU_REGION_ENTRY("SRAM", DT_CHOSEN_SRAM_ADDR,
			 REGION_RAM_NOCACHE_ATTR(DT_CHOSEN_SRAM_ADDR, DT_CHOSEN_SRAM_SIZE)),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
