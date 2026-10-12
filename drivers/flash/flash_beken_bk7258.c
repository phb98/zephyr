/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT beken_bk7258_flash_controller

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(flash_beken_bk7258, CONFIG_FLASH_LOG_LEVEL);

/*
 * The controller's data port: commands take a physical flash address and
 * move 32 bytes through an 8-word buffer. It issues write enable before a
 * program, erase or status register write and holds busy until the flash
 * has finished.
 */
#define FC_OP_CTRL          0x10
#define FC_OP_CTRL_OP_SW    BIT(29)
#define FC_OP_CTRL_WP_VALUE BIT(30)
#define FC_OP_CTRL_BUSY_SW  BIT(31)
#define FC_DATA_SW_FLASH    0x14
#define FC_DATA_FLASH_SW    0x18
#define FC_CMD_CFG          0x1c
#define FC_STATE            0x24
#define FC_STATE_STATUS_REG GENMASK(7, 0)
#define FC_STATE_M_VALUE    GENMASK(29, 22)
#define FC_CONFIG           0x28
#define FC_CONFIG_MODE_SEL  GENMASK(8, 4)
#define FC_CONFIG_WRSR_DATA GENMASK(25, 10)
#define FC_OP_CMD           0x54
#define FC_OP_CMD_ADDR      GENMASK(23, 0)
#define FC_OP_CMD_TYPE      GENMASK(28, 24)

#define FC_OP_RDSR  3U
#define FC_OP_WRSR  4U
#define FC_OP_READ  5U
#define FC_OP_RDSR2 6U
#define FC_OP_WRSR2 7U
#define FC_OP_PP    12U
#define FC_OP_SE    13U
#define FC_OP_CRMR  22U

/* Line modes of reads, execute-in-place included, as numbered in mode_sel */
#define FC_MODE_SINGLE 0U
#define FC_MODE_DUAL   1U
#define FC_MODE_QUAD   2U

/* SYS block bit, cleared and set again around a change of the line mode */
#define SYS_CPU0_LV_SLEEP_CFG 0x44010044
#define SYS2FLSH_2WIRE        BIT(7)

#define FC_BUF_WORDS 8U
#define FC_BUF_SIZE  (FC_BUF_WORDS * 4U)

/* Longer than any one operation takes, a sector erase being the longest */
#define FC_TIMEOUT_US USEC_PER_SEC

/* JESD216 quad enable requirements, numbered as in jedec,jesd216.yaml */
#define QER_NONE   0
#define QER_S2B1V1 1
#define QER_S1B6   2

#define FLASH_QER       DT_INST_ENUM_IDX_OR(0, quad_enable_requirements, QER_NONE)
#define FLASH_LOCK      DT_INST_PROP_OR(0, has_lock, 0)
#define FLASH_MODE_BITS DT_INST_PROP_OR(0, quad_mode_bits, 0)

#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)
#define FLASH_SIZE        DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_ERASE_SIZE  DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

BUILD_ASSERT(FLASH_SIZE <= MB(16), "the controller's flash addresses are 24 bits");
BUILD_ASSERT(FLASH_LOCK <= UINT8_MAX, "has-lock covers status register 1 only");
BUILD_ASSERT((FLASH_LOCK == 0) || DT_INST_NODE_HAS_PROP(0, quad_enable_requirements),
	     "has-lock needs quad-enable-requirements, which sets how to write the register");
BUILD_ASSERT(FLASH_MODE_BITS <= UINT8_MAX, "quad-mode-bits is one byte");

struct flash_bk7258_config {
	uintptr_t base;
};

struct flash_bk7258_data {
	/* Serializes commands: a program loads the buffer before issuing */
	struct k_sem lock;
};

static const struct flash_bk7258_config flash_bk7258_config = {
	.base = DT_INST_REG_ADDR(0),
};

static struct flash_bk7258_data flash_bk7258_data;

static const struct flash_parameters flash_bk7258_parameters = {
	.write_block_size = DT_PROP(SOC_NV_FLASH_NODE, write_block_size),
	.erase_value = 0xff,
};

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout flash_bk7258_pages_layout = {
	.pages_count = FLASH_SIZE / FLASH_ERASE_SIZE,
	.pages_size = FLASH_ERASE_SIZE,
};
#endif

static int flash_bk7258_cmd(const struct device *dev, uint32_t type, uint32_t addr)
{
	const struct flash_bk7258_config *cfg = dev->config;

	sys_write32(FIELD_PREP(FC_OP_CMD_TYPE, type) | FIELD_PREP(FC_OP_CMD_ADDR, addr),
		    cfg->base + FC_OP_CMD);
	sys_set_bits(cfg->base + FC_OP_CTRL, FC_OP_CTRL_OP_SW);

	if (!WAIT_FOR((sys_read32(cfg->base + FC_OP_CTRL) & FC_OP_CTRL_BUSY_SW) == 0U,
		      FC_TIMEOUT_US, NULL)) {
		LOG_ERR("command %u at 0x%06x timed out", type, addr);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Leaves continuous read, then sets the mode. Runs from RAM with interrupts
 * locked: until the controller and the flash agree on the mode again, no
 * read through the controller succeeds, execute-in-place included.
 */
__ramfunc static void flash_bk7258_mode_set(uintptr_t base, uint32_t mode)
{
	unsigned int key = irq_lock();
	uint32_t config = sys_read32(base + FC_CONFIG) & ~FC_CONFIG_MODE_SEL;

	while ((sys_read32(base + FC_OP_CTRL) & FC_OP_CTRL_BUSY_SW) != 0U) {
	}

	sys_write32(config, base + FC_CONFIG);
	sys_write32(FIELD_PREP(FC_OP_CMD_TYPE, FC_OP_CRMR), base + FC_OP_CMD);
	sys_set_bits(base + FC_OP_CTRL, FC_OP_CTRL_OP_SW);
	while ((sys_read32(base + FC_OP_CTRL) & FC_OP_CTRL_BUSY_SW) != 0U) {
	}

	sys_clear_bits(SYS_CPU0_LV_SLEEP_CFG, SYS2FLSH_2WIRE);
	if (mode == FC_MODE_QUAD) {
		sys_write32((sys_read32(base + FC_STATE) & ~FC_STATE_M_VALUE) |
				    FIELD_PREP(FC_STATE_M_VALUE, FLASH_MODE_BITS),
			    base + FC_STATE);
	}
	sys_write32(config | FIELD_PREP(FC_CONFIG_MODE_SEL, mode), base + FC_CONFIG);
	sys_set_bits(SYS_CPU0_LV_SLEEP_CFG, SYS2FLSH_2WIRE);

	barrier_dsync_fence_full();
	barrier_isync_fence_full();
	irq_unlock(key);
}

/*
 * In quad mode the flash is in continuous read, where a command other than a
 * read hangs the controller, so commands run in dual mode.
 */
static uint32_t flash_bk7258_cmd_mode_enter(const struct device *dev)
{
	const struct flash_bk7258_config *cfg = dev->config;
	uint32_t mode = FIELD_GET(FC_CONFIG_MODE_SEL, sys_read32(cfg->base + FC_CONFIG));

	if (mode == FC_MODE_QUAD) {
		flash_bk7258_mode_set(cfg->base, FC_MODE_DUAL);
	}

	return mode;
}

static void flash_bk7258_cmd_mode_exit(const struct device *dev, uint32_t mode)
{
	const struct flash_bk7258_config *cfg = dev->config;

	if (mode == FC_MODE_QUAD) {
		flash_bk7258_mode_set(cfg->base, mode);
	}
}

static bool flash_bk7258_range_is_valid(off_t offset, size_t len)
{
	return (offset >= 0) && (len <= FLASH_SIZE) && ((size_t)offset <= (FLASH_SIZE - len));
}

static int flash_bk7258_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct flash_bk7258_config *cfg = dev->config;
	struct flash_bk7258_data *dev_data = dev->data;
	uint32_t buf[FC_BUF_WORDS];
	uint8_t *dst = data;
	int ret = 0;

	if (!flash_bk7258_range_is_valid(offset, len)) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->lock, K_FOREVER);

	while (len > 0U) {
		size_t chunk = MIN(len, FC_BUF_SIZE);

		/* READ returns the 32 bytes from the exact byte address */
		ret = flash_bk7258_cmd(dev, FC_OP_READ, offset);
		if (ret != 0) {
			break;
		}

		for (size_t i = 0; i < FC_BUF_WORDS; i++) {
			buf[i] = sys_read32(cfg->base + FC_DATA_FLASH_SW);
		}

		memcpy(dst, buf, chunk);
		dst += chunk;
		offset += chunk;
		len -= chunk;
	}

	k_sem_give(&dev_data->lock);

	return ret;
}

static int flash_bk7258_write(const struct device *dev, off_t offset, const void *data,
			      size_t len)
{
	const struct flash_bk7258_config *cfg = dev->config;
	struct flash_bk7258_data *dev_data = dev->data;
	const uint8_t *src = data;
	uint32_t buf[FC_BUF_WORDS];
	uint32_t mode;
	int ret = 0;

	if (!flash_bk7258_range_is_valid(offset, len)) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->lock, K_FOREVER);
	mode = flash_bk7258_cmd_mode_enter(dev);

	while (len > 0U) {
		/*
		 * A program wraps within the flash's 256-byte page, so program
		 * whole aligned 32-byte units, which never cross one. Bytes
		 * outside the data are 0xff and leave the flash as it was.
		 */
		size_t start = offset % FC_BUF_SIZE;
		size_t chunk = MIN(len, FC_BUF_SIZE - start);

		memset(buf, 0xff, sizeof(buf));
		memcpy((uint8_t *)buf + start, src, chunk);

		for (size_t i = 0; i < FC_BUF_WORDS; i++) {
			sys_write32(buf[i], cfg->base + FC_DATA_SW_FLASH);
		}

		ret = flash_bk7258_cmd(dev, FC_OP_PP, offset - start);
		if (ret != 0) {
			break;
		}

		src += chunk;
		offset += chunk;
		len -= chunk;
	}

	flash_bk7258_cmd_mode_exit(dev, mode);
	k_sem_give(&dev_data->lock);

	return ret;
}

static int flash_bk7258_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_bk7258_data *dev_data = dev->data;
	uint32_t mode;
	int ret = 0;

	if (!flash_bk7258_range_is_valid(offset, size) || ((offset % FLASH_ERASE_SIZE) != 0) ||
	    ((size % FLASH_ERASE_SIZE) != 0U)) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->lock, K_FOREVER);
	mode = flash_bk7258_cmd_mode_enter(dev);

	for (; size > 0U; size -= FLASH_ERASE_SIZE, offset += FLASH_ERASE_SIZE) {
		ret = flash_bk7258_cmd(dev, FC_OP_SE, offset);
		if (ret != 0) {
			break;
		}
	}

	flash_bk7258_cmd_mode_exit(dev, mode);
	k_sem_give(&dev_data->lock);

	return ret;
}

static const struct flash_parameters *flash_bk7258_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_bk7258_parameters;
}

static int flash_bk7258_get_size(const struct device *dev, uint64_t *size)
{
	ARG_UNUSED(dev);

	*size = FLASH_SIZE;

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_bk7258_page_layout(const struct device *dev,
				     const struct flash_pages_layout **layout,
				     size_t *layout_size)
{
	ARG_UNUSED(dev);

	*layout = &flash_bk7258_pages_layout;
	*layout_size = 1;
}
#endif

static int flash_bk7258_read_sr(const struct device *dev, uint32_t type, uint8_t *val)
{
	const struct flash_bk7258_config *cfg = dev->config;
	int ret;

	ret = flash_bk7258_cmd(dev, type, 0);
	if (ret == 0) {
		*val = FIELD_GET(FC_STATE_STATUS_REG, sys_read32(cfg->base + FC_STATE));
	}

	return ret;
}

/* Clear the has-lock bits, block protection that refuses programs and erases silently */
static int flash_bk7258_unprotect(const struct device *dev)
{
	const struct flash_bk7258_config *cfg = dev->config;
	uint32_t type = FC_OP_WRSR;
	uint32_t value;
	uint8_t sr1, sr2;
	int ret;

	if (FLASH_LOCK == 0) {
		return 0;
	}

	ret = flash_bk7258_read_sr(dev, FC_OP_RDSR, &sr1);
	if ((ret != 0) || ((sr1 & FLASH_LOCK) == 0U)) {
		return ret;
	}

	value = sr1 & ~(uint32_t)FLASH_LOCK;

	/* With S2B1v1 a one-byte write clears status register 2, so write both */
	if (FLASH_QER == QER_S2B1V1) {
		ret = flash_bk7258_read_sr(dev, FC_OP_RDSR2, &sr2);
		if (ret != 0) {
			return ret;
		}
		value |= (uint32_t)sr2 << 8;
		type = FC_OP_WRSR2;
	}

	sys_write32(0, cfg->base + FC_CMD_CFG);
	sys_write32((sys_read32(cfg->base + FC_CONFIG) & ~FC_CONFIG_WRSR_DATA) |
			    FIELD_PREP(FC_CONFIG_WRSR_DATA, value),
		    cfg->base + FC_CONFIG);
	sys_set_bits(cfg->base + FC_OP_CTRL, FC_OP_CTRL_WP_VALUE);
	ret = flash_bk7258_cmd(dev, type, 0);
	sys_clear_bits(cfg->base + FC_OP_CTRL, FC_OP_CTRL_WP_VALUE);
	if (ret != 0) {
		return ret;
	}

	ret = flash_bk7258_read_sr(dev, FC_OP_RDSR, &sr1);
	if ((ret == 0) && ((sr1 & FLASH_LOCK) != 0U)) {
		LOG_ERR("block protection still set, SR1 0x%02x", sr1);
		return -EIO;
	}

	return ret;
}

/* Where the quad enable bit is follows quad-enable-requirements (JESD216) */
static int flash_bk7258_quad_enabled(const struct device *dev, bool *enabled)
{
	uint8_t sr;
	int ret;

	if (FLASH_QER == QER_NONE) {
		*enabled = true;
		return 0;
	}

	if (FLASH_QER == QER_S1B6) {
		ret = flash_bk7258_read_sr(dev, FC_OP_RDSR, &sr);
		if (ret == 0) {
			*enabled = (sr & BIT(6)) != 0U;
		}
	} else {
		ret = flash_bk7258_read_sr(dev, FC_OP_RDSR2, &sr);
		if (ret == 0) {
			*enabled = (sr & BIT(1)) != 0U;
		}
	}

	return ret;
}

/* Quad reads need the flash's quad enable bit, which is not changed here */
static int flash_bk7258_read_mode_init(const struct device *dev)
{
	const struct flash_bk7258_config *cfg = dev->config;
	uint32_t mode = IS_ENABLED(CONFIG_FLASH_BEKEN_BK7258_MODE_QUAD)   ? FC_MODE_QUAD
			: IS_ENABLED(CONFIG_FLASH_BEKEN_BK7258_MODE_DUAL) ? FC_MODE_DUAL
									  : FC_MODE_SINGLE;
	bool qe;
	int ret;

	if (mode == FC_MODE_QUAD) {
		ret = flash_bk7258_quad_enabled(dev, &qe);
		if (ret != 0) {
			return ret;
		}
		if (!qe) {
			LOG_WRN("quad enable clear, reads are dual");
			mode = FC_MODE_DUAL;
		}
	}

	if (mode != FC_MODE_SINGLE) {
		flash_bk7258_mode_set(cfg->base, mode);
	}

	return 0;
}

static int flash_bk7258_init(const struct device *dev)
{
	struct flash_bk7258_data *dev_data = dev->data;
	int ret;

	k_sem_init(&dev_data->lock, 1, 1);

	/* Reads are single line from boot, so commands need no mode change here */
	ret = flash_bk7258_unprotect(dev);
	if (ret != 0) {
		return ret;
	}

	return flash_bk7258_read_mode_init(dev);
}

static DEVICE_API(flash, flash_bk7258_api) = {
	.read = flash_bk7258_read,
	.write = flash_bk7258_write,
	.erase = flash_bk7258_erase,
	.get_parameters = flash_bk7258_get_parameters,
	.get_size = flash_bk7258_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_bk7258_page_layout,
#endif
};

DEVICE_DT_INST_DEFINE(0, flash_bk7258_init, NULL, &flash_bk7258_data, &flash_bk7258_config,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &flash_bk7258_api);
