/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT beken_bk7258_spi

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(spi_bk7258, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"
#include "spi_rtio.h"

/* The registers of one SPI block, at its base address */
struct spi_bk7258_regs {
	volatile uint32_t dev_id;
	volatile uint32_t dev_version;
	volatile uint32_t global_ctrl;
	volatile uint32_t dev_status;
	volatile uint32_t ctrl;
	volatile uint32_t cfg;
	volatile uint32_t status;
	volatile uint32_t data;
};

BUILD_ASSERT(offsetof(struct spi_bk7258_regs, ctrl) == 0x10);
BUILD_ASSERT(offsetof(struct spi_bk7258_regs, data) == 0x1c);

/* global_ctrl: the block is held in reset while this bit is clear */
#define BK_SPI_GLOBAL_CTRL_SOFT_RESET_N BIT(0)

/* ctrl */
#define BK_SPI_CTRL_CLK_DIV   GENMASK(15, 8)
#define BK_SPI_CTRL_NO_CSN    BIT(17)
#define BK_SPI_CTRL_16BIT     BIT(18)
#define BK_SPI_CTRL_LSB_FIRST BIT(19)
#define BK_SPI_CTRL_CPOL      BIT(20)
#define BK_SPI_CTRL_CPHA      BIT(21)
#define BK_SPI_CTRL_MASTER    BIT(22)
#define BK_SPI_CTRL_ENABLE    BIT(23)
/* bit[30]: unknown function */
#define BK_SPI_CTRL_KEEP      BIT(30)

/*
 * cfg. With both frame counts zero, the block clocks frames for as long as
 * the TX FIFO holds data and pauses when it is empty.
 */
#define BK_SPI_CFG_TX_EN BIT(0)
#define BK_SPI_CFG_RX_EN BIT(1)
/* bit[7:4]: reserved */
#define BK_SPI_CFG_KEEP  GENMASK(7, 4)

/* status */
#define BK_SPI_STATUS_TX_READY    BIT(1)
#define BK_SPI_STATUS_RX_READY    BIT(2)
#define BK_SPI_STATUS_INT_ALL     GENMASK(14, 8)
#define BK_SPI_STATUS_TX_FIFO_CLR BIT(16)
#define BK_SPI_STATUS_RX_FIFO_CLR BIT(17)

#define BK_SPI_FIFO_DEPTH 64U
#define BK_SPI_CLK_DIV_MAX 255U

struct spi_bk7258_config {
	struct spi_bk7258_regs *regs;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
};

struct spi_bk7258_data {
	struct spi_context ctx;
	/* Bytes per frame */
	uint8_t dfs;
	/* Frames of the current transfer still to be written, and to be read */
	size_t tx_left;
	size_t rx_left;
};

/*
 * status mixes write-1-to-clear interrupt flags and FIFO clear bits
 * with read-only and reserved ones. Writing back the value read would clear
 * every pending flag, and empty a FIFO whose clear bit reads back as 1, so
 * those bits are 1 only where the caller asks for it.
 */
static void spi_bk7258_status_write(struct spi_bk7258_regs *regs, uint32_t bits)
{
	uint32_t status = regs->status;

	status &= ~(BK_SPI_STATUS_INT_ALL | BK_SPI_STATUS_TX_FIFO_CLR | BK_SPI_STATUS_RX_FIFO_CLR);
	regs->status = status | bits;
}

static void spi_bk7258_cfg_write(struct spi_bk7258_regs *regs, uint32_t value)
{
	regs->cfg = (regs->cfg & BK_SPI_CFG_KEEP) | value;
}

static int spi_bk7258_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	const struct spi_bk7258_config *config = dev->config;
	struct spi_bk7258_data *data = dev->data;
	uint32_t word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
	uint32_t clk_rate;
	uint32_t div;
	uint32_t ctrl;
	int err;

	if (spi_context_configured(&data->ctx, spi_cfg)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_CONTROLLER) {
		LOG_ERR("Only controller mode is supported");
		return -ENOTSUP;
	}

	if ((spi_cfg->operation & SPI_MODE_LOOP) != 0U) {
		LOG_ERR("Loopback mode is not supported");
		return -ENOTSUP;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    (spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only single line mode is supported");
		return -ENOTSUP;
	}

	if (word_size != 8U && word_size != 16U) {
		LOG_ERR("Word size %u is not supported", word_size);
		return -ENOTSUP;
	}

	if (spi_cfg->frequency == 0U) {
		return -EINVAL;
	}

	err = clock_control_get_rate(config->clock_dev, config->clock_subsys, &clk_rate);
	if (err < 0) {
		return err;
	}

	/*
	 * SCK_clk = Source_clk/(1+div). Take the fastest rate that does
	 * not exceed the one requested.
	 */
	if (spi_cfg->frequency >= clk_rate) {
		div = 0U;
	} else {
		div = DIV_ROUND_UP(clk_rate, 2U * spi_cfg->frequency);
	}

	if (div > BK_SPI_CLK_DIV_MAX) {
		LOG_ERR("Frequency %u Hz is below the minimum of %u Hz", spi_cfg->frequency,
			clk_rate / (2U * BK_SPI_CLK_DIV_MAX));
		return -EINVAL;
	}

	/* Chip select is driven through cs-gpios, so the block leaves its CSN pad alone */
	ctrl = config->regs->ctrl & BK_SPI_CTRL_KEEP;
	ctrl |= BK_SPI_CTRL_ENABLE | BK_SPI_CTRL_MASTER | BK_SPI_CTRL_NO_CSN |
		FIELD_PREP(BK_SPI_CTRL_CLK_DIV, div);

	if (word_size == 16U) {
		ctrl |= BK_SPI_CTRL_16BIT;
	}

	if ((spi_cfg->operation & SPI_TRANSFER_LSB) != 0U) {
		ctrl |= BK_SPI_CTRL_LSB_FIRST;
	}

	if ((SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL) != 0U) {
		ctrl |= BK_SPI_CTRL_CPOL;
	}

	if ((SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA) != 0U) {
		ctrl |= BK_SPI_CTRL_CPHA;
	}

	config->regs->ctrl = ctrl;

	data->dfs = word_size / 8U;
	data->ctx.config = spi_cfg;

	return 0;
}

/* Empty both FIFOs and start clocking; frames go out as they are written */
static void spi_bk7258_start(const struct device *dev)
{
	const struct spi_bk7258_config *config = dev->config;
	struct spi_bk7258_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	size_t frames = MAX(spi_context_total_tx_len(ctx), spi_context_total_rx_len(ctx)) /
			data->dfs;

	data->tx_left = frames;
	data->rx_left = frames;

	spi_bk7258_status_write(config->regs,
				BK_SPI_STATUS_TX_FIFO_CLR | BK_SPI_STATUS_RX_FIFO_CLR);
	spi_bk7258_cfg_write(config->regs, BK_SPI_CFG_TX_EN | BK_SPI_CFG_RX_EN);
}

/*
 * Write one frame if the TX FIFO has room and read one if the RX FIFO has
 * data. Every frame written comes back as a frame read, so the transfer is
 * complete once rx_left reaches zero. At most a FIFO's worth of frames is
 * ever in flight, which keeps the RX FIFO from overflowing.
 */
static void spi_bk7258_exchange(const struct device *dev)
{
	const struct spi_bk7258_config *config = dev->config;
	struct spi_bk7258_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t status = config->regs->status;
	uint32_t frame;

	if ((data->tx_left > 0U) && ((status & BK_SPI_STATUS_TX_READY) != 0U) &&
	    ((data->rx_left - data->tx_left) < BK_SPI_FIFO_DEPTH)) {
		frame = 0U;

		if (spi_context_tx_buf_on(ctx)) {
			if (data->dfs == 1U) {
				frame = *ctx->tx_buf;
			} else {
				frame = UNALIGNED_GET((const uint16_t *)ctx->tx_buf);
			}
		}

		config->regs->data = frame;
		spi_context_update_tx(ctx, data->dfs, 1);
		data->tx_left--;
	}

	if ((status & BK_SPI_STATUS_RX_READY) != 0U) {
		frame = config->regs->data;

		if (spi_context_rx_buf_on(ctx)) {
			if (data->dfs == 1U) {
				*ctx->rx_buf = (uint8_t)frame;
			} else {
				UNALIGNED_PUT((uint16_t)frame, (uint16_t *)ctx->rx_buf);
			}
		}

		spi_context_update_rx(ctx, data->dfs, 1);
		data->rx_left--;
	}
}

/* The last frame is back, so the clock is idle: switch TX and RX off */
static void spi_bk7258_stop(const struct device *dev)
{
	const struct spi_bk7258_config *config = dev->config;

	spi_bk7258_cfg_write(config->regs, 0U);
}

static int spi_bk7258_transceive(const struct device *dev, const struct spi_config *spi_cfg,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	struct spi_bk7258_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int err;

	spi_context_lock(ctx, false, NULL, NULL, spi_cfg);

	err = spi_bk7258_configure(dev, spi_cfg);
	if (err != 0) {
		goto out;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, data->dfs);

	/*
	 * The block drives MOSI for as long as it owns the pad, so on a single
	 * data line it can send but never hear the other end.
	 */
	if (((spi_cfg->operation & SPI_HALF_DUPLEX) != 0U) &&
	    (spi_context_total_rx_len(ctx) > 0U)) {
		LOG_ERR("Half duplex transfers can only write");
		err = -ENOTSUP;
		goto out;
	}

	spi_context_cs_control(ctx, true);

	spi_bk7258_start(dev);
	while (data->rx_left > 0U) {
		spi_bk7258_exchange(dev);
	}
	spi_bk7258_stop(dev);

	spi_context_cs_control(ctx, false);

out:
	spi_context_release(ctx, err);

	return err;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_bk7258_transceive_async(const struct device *dev, const struct spi_config *spi_cfg,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				       void *userdata)
{
	return -ENOTSUP;
}
#endif

static int spi_bk7258_release(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct spi_bk7258_data *data = dev->data;

	ARG_UNUSED(spi_cfg);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_bk7258_init(const struct device *dev)
{
	const struct spi_bk7258_config *config = dev->config;
	struct spi_bk7258_data *data = dev->data;
	int err;

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err < 0) {
		return err;
	}

	config->regs->global_ctrl |= BK_SPI_GLOBAL_CTRL_SOFT_RESET_N;
	spi_bk7258_cfg_write(config->regs, 0U);

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, spi_bk7258_driver_api) = {
	.transceive = spi_bk7258_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_bk7258_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_bk7258_release,
};

#define SPI_BK7258_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static struct spi_bk7258_data spi_bk7258_data_##n = {                                      \
		SPI_CONTEXT_INIT_LOCK(spi_bk7258_data_##n, ctx),                                   \
		SPI_CONTEXT_INIT_SYNC(spi_bk7258_data_##n, ctx),                                   \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
                                                                                                   \
	static const struct spi_bk7258_config spi_bk7258_config_##n = {                            \
		.regs = (struct spi_bk7258_regs *)DT_INST_REG_ADDR(n),                             \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id),                \
	};                                                                                         \
                                                                                                   \
	SPI_DEVICE_DT_INST_DEFINE(n, spi_bk7258_init, NULL, &spi_bk7258_data_##n,                  \
				  &spi_bk7258_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,   \
				  &spi_bk7258_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_BK7258_INIT)
