/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT beken_bk7258_trng

#include <string.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/util.h>

/* Register offsets from the block base address */
#define BK_TRNG_GLOBAL_CTRL 0x08
#define BK_TRNG_CTRL        0x10
#define BK_TRNG_DATA        0x14

/*
 * BK_TRNG_GLOBAL_CTRL: the block is held in reset while SOFT_RESET_N is clear.
 * The generator runs with the block's clock gate bypassed, as in the vendor
 * SDK; with the gate active, the first words read after enabling are shifted
 * copies of each other.
 */
#define BK_TRNG_GLOBAL_CTRL_SOFT_RESET_N    BIT(0)
#define BK_TRNG_GLOBAL_CTRL_CLK_GATE_BYPASS BIT(1)

/* BK_TRNG_CTRL */
#define BK_TRNG_CTRL_EN BIT(0)

/*
 * Every read of BK_TRNG_DATA returns a new word, except that the first word
 * after enabling is the same each time. The vendor SDK drops 16.
 */
#define BK_TRNG_DISCARD_WORDS 16

struct entropy_bk7258_config {
	uintptr_t base;
};

static void entropy_bk7258_fill(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	const struct entropy_bk7258_config *config = dev->config;

	while (length > 0) {
		uint32_t word = sys_read32(config->base + BK_TRNG_DATA);
		uint16_t n = MIN(length, sizeof(word));

		memcpy(buffer, &word, n);
		buffer += n;
		length -= n;
	}
}

static int entropy_bk7258_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	entropy_bk7258_fill(dev, buffer, length);

	return 0;
}

static int entropy_bk7258_get_entropy_isr(const struct device *dev, uint8_t *buffer,
					  uint16_t length, uint32_t flags)
{
	ARG_UNUSED(flags);

	entropy_bk7258_fill(dev, buffer, length);

	return length;
}

static int entropy_bk7258_init(const struct device *dev)
{
	const struct entropy_bk7258_config *config = dev->config;
	mem_addr_t global_ctrl = config->base + BK_TRNG_GLOBAL_CTRL;
	mem_addr_t ctrl = config->base + BK_TRNG_CTRL;

	sys_write32(sys_read32(global_ctrl) | BK_TRNG_GLOBAL_CTRL_SOFT_RESET_N |
			    BK_TRNG_GLOBAL_CTRL_CLK_GATE_BYPASS,
		    global_ctrl);
	sys_write32(sys_read32(ctrl) | BK_TRNG_CTRL_EN, ctrl);

	for (int i = 0; i < BK_TRNG_DISCARD_WORDS; i++) {
		(void)sys_read32(config->base + BK_TRNG_DATA);
	}

	return 0;
}

static DEVICE_API(entropy, entropy_bk7258_api) = {
	.get_entropy = entropy_bk7258_get_entropy,
	.get_entropy_isr = entropy_bk7258_get_entropy_isr,
};

#define ENTROPY_BK7258_INIT(n)                                                                     \
	static const struct entropy_bk7258_config entropy_bk7258_config_##n = {                    \
		.base = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, entropy_bk7258_init, NULL, NULL, &entropy_bk7258_config_##n,      \
			      PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY, &entropy_bk7258_api);

DT_INST_FOREACH_STATUS_OKAY(ENTROPY_BK7258_INIT)
