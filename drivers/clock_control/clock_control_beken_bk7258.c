/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT beken_bk7258_cgu

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/beken_bk7258_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(clock_control_bk7258, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

/*
 * The SYS block is a flat file of 32-bit registers. Offsets below are the
 * word index of the vendor documentation multiplied by four.
 */
#define SYS_CPU0_INT_HALT_CLK_OP 0x10
#define SYS_CLK_DIV_MODE1        0x20
#define SYS_ANASPI_FREQ          0x2c
#define SYS_DEVICE_CLK_ENABLE    0x30
#define SYS_DEVICE_CLK_ENABLE1   0x34

/* SYS_CPU0_INT_HALT_CLK_OP */
#define CPU0_SPEED BIT(4)

/* SYS_ANASPI_FREQ: the analog serial clock is bus_clk / ((field + 1) * 2) */
#define ANASPI_FREQ GENMASK(5, 0)

/*
 * The analog block, in the "ana" region: the analog SPI status word, then
 * the analog registers. Every analog register is written over a serial link,
 * so a write is followed by polling the register's own bit in the status
 * word until the transfer has gone out.
 */
#define ANA_SPI_STATE 0x00
#define ANA_REG(n)    (0x18 + ((n) * 4))

/* ANA_REG(0): the DPLL's settings and the handshakes that calibrate it */
#define ANA_REG0_SPIDETEN BIT(4)
#define ANA_REG0_SPITRIG  BIT(19)
#define ANA_REG0_DSPTRIG  BIT(26)

/*
 * The whole of ANA_REG(0) as the vendor HAL writes it to configure the DPLL.
 * The fields are analog trim with no documented meaning, so the word is
 * carried over as a unit rather than split up. It decodes, per the SDK's
 * middleware/soc/bk7258/soc/sys_struct.h, to dpll_tsten 0, cp 3, spideten 1,
 * hvref 2, lvref 2, rzctrl26m 1, looprzctrl 6, rpc 1, openloop_en 0, cksel 0,
 * spitrig 0, band0 1, band1 1, band 4, bandmanual 0, dsptrig 0, lpen_dpll 0,
 * vctrl_dpllldo 1 -- so the two handshake bits this driver drives, SPITRIG
 * and DSPTRIG, are clear in it and SPIDETEN is set.
 */
#define ANA_REG0_DPLL_SETTINGS 0xf1305b56

/* ANA_REG(5) */
#define ANA_REG5_EN_DPLL BIT(5)

/* ANA_REG(9), which takes a write only while SPI_LATCH1V is set */
#define ANA_REG9_SPI_LATCH1V BIT(9)
#define ANA_REG9_VCOREHSEL   GENMASK(19, 16)

/* SYS_CLK_DIV_MODE1 */
#define CLKDIV_CORE  GENMASK(3, 0)
#define CKSEL_CORE   GENMASK(5, 4)
#define CLKDIV_UART0 GENMASK(9, 8)
#define CKSEL_UART0  BIT(10)
#define CLKDIV_UART1 GENMASK(12, 11)
#define CKSEL_UART1  BIT(13)
#define CLKDIV_UART2 GENMASK(15, 14)
#define CKSEL_UART2  BIT(16)

/* Highest clock that has a gate bit */
#define BK7258_CLK_GATED_MAX BK7258_CLK_CIS_AUXS

/* Gates from this one on live in the second enable register */
#define BK7258_CLK_ENABLE1_FIRST BK7258_CLK_H264

/*
 * Sources the core clock can be switched to, indexed by cksel_core: the
 * crystal, whose rate comes from devicetree, the digitally controlled
 * oscillator, and the two outputs of the DPLL.
 */
#define CKSEL_CORE_XTAL 0
#define CKSEL_CORE_DCO  1
#define CKSEL_CORE_320M 2

#define CORE_SRC_320M MHZ(320)
#define CORE_SRC_480M MHZ(480)

/* CPU0's ceiling, datasheet DS-BK7258-E10 §4.3: 480 MHz is CPU1 and CPU2 */
#define CPU0_MAX_FREQ MHZ(240)

/*
 * The crystal, read by node label as the STM32 clock drivers read clk_hse.
 * Every board carries it and the DPLL is derived from it.
 */
#define XTAL_FREQ DT_PROP(DT_NODELABEL(clk_xtal), clock_frequency)

/*
 * What devicetree says the core clock is taken from. The DPLL's clock
 * specifier cell is the cksel_core value that selects that output, so one
 * definition serves both.
 */
#define CORE_SRC_NODE DT_INST_CLOCKS_CTLR(0)

#if DT_NODE_HAS_COMPAT_STATUS(CORE_SRC_NODE, beken_bk7258_dpll, okay)
#define CORE_SRC_IS_DPLL 1
#define CORE_SRC_CKSEL   DT_INST_CLOCKS_CELL(0, output)
#define CORE_SRC_FREQ    ((CORE_SRC_CKSEL == BK7258_DPLL_320M) ? CORE_SRC_320M : CORE_SRC_480M)
#elif DT_NODE_HAS_COMPAT_STATUS(CORE_SRC_NODE, fixed_clock, okay)
#define CORE_SRC_IS_DPLL 0
#define CORE_SRC_CKSEL   CKSEL_CORE_XTAL
#define CORE_SRC_FREQ    XTAL_FREQ
#else
#error "beken,bk7258-cgu clocks must name the crystal or an enabled DPLL output"
#endif

#define CORE_PRESCALER DT_INST_PROP(0, core_prescaler)
#define CPU0_PRESCALER DT_INST_PROP(0, cpu0_prescaler)

/* The core divider divides by the field plus one; cpu0_speed set means by one */
#define CORE_CLKDIV      (CORE_PRESCALER - 1)
#define CPU0_SPEED_FIELD ((CPU0_PRESCALER == 1) ? CPU0_SPEED : 0U)

#define CORE_FREQ (CORE_SRC_FREQ / CORE_PRESCALER)
#define CPU0_FREQ (CORE_FREQ / CPU0_PRESCALER)

BUILD_ASSERT(CPU0_FREQ == DT_INST_PROP(0, clock_frequency),
	     "beken,bk7258-cgu clock-frequency must be the source divided by "
	     "core-prescaler and then cpu0-prescaler");
BUILD_ASSERT(CPU0_FREQ <= CPU0_MAX_FREQ, "CPU0 tops out at 240 MHz");
BUILD_ASSERT(DT_INST_PROP(0, clock_frequency) == CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
	     "SysTick counts CPU0's clock, so /cpus/cpu@0 clock-frequency must "
	     "match the clock controller's");

/*
 * VDDDIG settings, 0.6 V plus 25 mV a step, and the one the core clock needs
 * when it runs from the DPLL: the vendor HAL's DVFS table, in
 * sys_hal_switch_cpu_bus_freq_low_to_high(), votes 0.95 V at 480 MHz,
 * 0.925 V at 320 and 240 MHz, 0.9 V at 120 MHz and 0.875 V at 80 MHz and
 * below. A rate between two rows of the table takes the higher row's voltage.
 * On the crystal the BootROM's 0.85 V is left as it is.
 */
#define VCOREHSEL_0V875 0xb
#define VCOREHSEL_0V9   0xc
#define VCOREHSEL_0V925 0xd
#define VCOREHSEL_0V95  0xe

#define CORE_VCOREHSEL                                                                             \
	((CORE_FREQ > MHZ(320)) ? VCOREHSEL_0V95                                                   \
	 : (CORE_FREQ > MHZ(120)) ? VCOREHSEL_0V925                                                \
	 : (CORE_FREQ > MHZ(80)) ? VCOREHSEL_0V9                                                   \
	 : VCOREHSEL_0V875)

/*
 * Settling times, in microseconds. The vendor HAL waits these out in bare
 * loops of a fixed iteration count -- 120, 3400 and 3400 in
 * sys_hal_cali_dpll(), and 2600 after a VDDDIG change, which it comments as
 * 10 us -- so its durations depend on code generation and on the clock the
 * loop runs at.
 */
#define DELAY_VDDDIG_SETTLE_US 100
#define DELAY_DPLL_TRIGGER_US  10
#define DELAY_DPLL_DETECT_US   1000

/*
 * Analog serial clock divider the vendor HAL sets once the bus runs from the
 * DPLL: a 240 MHz bus / ((2 + 1) * 2) = 40 MHz
 */
#define ANASPI_FREQ_DPLL 2

struct bk7258_cgu_config {
	uintptr_t base;
	uintptr_t ana;
};

static struct k_spinlock bk7258_cgu_lock;

static int bk7258_get_core_clock(uint32_t mode1, uint32_t *rate)
{
	uint32_t source;

	switch (FIELD_GET(CKSEL_CORE, mode1)) {
	case CKSEL_CORE_XTAL:
		source = XTAL_FREQ;
		break;
	case CKSEL_CORE_DCO:
		/*
		 * 26 to 360 MHz; registers hold the calibration target,
		 * not the rate reached
		 */
		return -ENOTSUP;
	case CKSEL_CORE_320M:
		source = CORE_SRC_320M;
		break;
	default:
		source = CORE_SRC_480M;
		break;
	}

	*rate = source / (FIELD_GET(CLKDIV_CORE, mode1) + 1U);

	return 0;
}

/* CPU0 runs at the core clock, or at half of it when cpu0_speed is clear */
static int bk7258_get_cpu0_clock(const struct device *dev, uint32_t mode1, uint32_t *rate)
{
	const struct bk7258_cgu_config *config = dev->config;
	int ret;

	ret = bk7258_get_core_clock(mode1, rate);
	if (ret < 0) {
		return ret;
	}

	if ((sys_read32(config->base + SYS_CPU0_INT_HALT_CLK_OP) & CPU0_SPEED) == 0U) {
		*rate /= 2U;
	}

	return 0;
}

/* Source select and divider fields of a UART clock, or 0 for any other clock */
static uint32_t bk7258_get_uart_clock_fields(uint32_t id)
{
	switch (id) {
	case BK7258_CLK_UART0:
		return CKSEL_UART0 | CLKDIV_UART0;
	case BK7258_CLK_UART1:
		return CKSEL_UART1 | CLKDIV_UART1;
	case BK7258_CLK_UART2:
		return CKSEL_UART2 | CLKDIV_UART2;
	default:
		return 0U;
	}
}

static int bk7258_cgu_gate(const struct device *dev, uint32_t id, bool enable)
{
	const struct bk7258_cgu_config *config = dev->config;
	uintptr_t reg = config->base + SYS_DEVICE_CLK_ENABLE;
	uint32_t uart_fields = bk7258_get_uart_clock_fields(id);
	k_spinlock_key_t key;

	/* Only the UART clocks are switched */
	if (uart_fields == 0U) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&bk7258_cgu_lock);

	if (enable) {
		sys_clear_bits(config->base + SYS_CLK_DIV_MODE1, uart_fields);
		sys_set_bits(reg, BIT(id));
	} else {
		sys_clear_bits(reg, BIT(id));
	}

	k_spin_unlock(&bk7258_cgu_lock, key);

	return 0;
}

static int bk7258_cgu_on(const struct device *dev, clock_control_subsys_t sys)
{
	return bk7258_cgu_gate(dev, (uint32_t)(uintptr_t)sys, true);
}

static int bk7258_cgu_off(const struct device *dev, clock_control_subsys_t sys)
{
	return bk7258_cgu_gate(dev, (uint32_t)(uintptr_t)sys, false);
}

static enum clock_control_status bk7258_cgu_get_status(const struct device *dev,
						       clock_control_subsys_t sys)
{
	const struct bk7258_cgu_config *config = dev->config;
	uint32_t id = (uint32_t)(uintptr_t)sys;
	uintptr_t reg = config->base + SYS_DEVICE_CLK_ENABLE;

	if (id >= BK7258_CLK_XTAL && id <= BK7258_CLK_CPU0) {
		/* A clock with no gate is always running */
		return CLOCK_CONTROL_STATUS_ON;
	}

	if (id > BK7258_CLK_GATED_MAX) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	if (id >= BK7258_CLK_ENABLE1_FIRST) {
		reg = config->base + SYS_DEVICE_CLK_ENABLE1;
		id -= BK7258_CLK_ENABLE1_FIRST;
	}

	return ((sys_read32(reg) & BIT(id)) != 0U) ? CLOCK_CONTROL_STATUS_ON
						   : CLOCK_CONTROL_STATUS_OFF;
}

static int bk7258_get_uart_clock(uint32_t mode1, uint32_t id, uint32_t *rate)
{
	uint32_t divider;
	bool from_pll;

	switch (id) {
	case BK7258_CLK_UART0:
		from_pll = (mode1 & CKSEL_UART0) != 0U;
		divider = FIELD_GET(CLKDIV_UART0, mode1);
		break;
	case BK7258_CLK_UART1:
		from_pll = (mode1 & CKSEL_UART1) != 0U;
		divider = FIELD_GET(CLKDIV_UART1, mode1);
		break;
	default:
		from_pll = (mode1 & CKSEL_UART2) != 0U;
		divider = FIELD_GET(CLKDIV_UART2, mode1);
		break;
	}

	/*
	 * The audio PLL is the other source a UART can take. It starts at
	 * 98.304 MHz but audio sample rates change it, so its rate is not
	 * modelled here.
	 */
	if (from_pll) {
		return -ENOTSUP;
	}

	/* Unlike the core divider, this one divides by a power of two */
	*rate = XTAL_FREQ >> divider;

	return 0;
}

static int bk7258_cgu_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	const struct bk7258_cgu_config *config = dev->config;
	uint32_t id = (uint32_t)(uintptr_t)sys;
	uint32_t mode1 = sys_read32(config->base + SYS_CLK_DIV_MODE1);

	switch (id) {
	case BK7258_CLK_XTAL:
		*rate = XTAL_FREQ;
		return 0;
	case BK7258_CLK_CORE:
		return bk7258_get_core_clock(mode1, rate);
	case BK7258_CLK_CPU0:
		return bk7258_get_cpu0_clock(dev, mode1, rate);
	case BK7258_CLK_UART0:
	case BK7258_CLK_UART1:
	case BK7258_CLK_UART2:
		return bk7258_get_uart_clock(mode1, id, rate);
	default:
		return -ENOTSUP;
	}
}

/*
 * Delays here run before any kernel timer, and the core is still on whatever
 * the BootROM left, which devicetree gives as the crystal. A loop iteration
 * cannot be shorter than one cycle, so this waits at least the time asked
 * for; overshooting only lengthens boot.
 */
static void bk7258_delay_us(uint32_t us)
{
	for (uint32_t i = 0U; i < (us * (XTAL_FREQ / 1000000U)); i++) {
		arch_nop();
	}
}

static uint32_t bk7258_ana_get(const struct bk7258_cgu_config *config, uint8_t n)
{
	return sys_read32(config->ana + ANA_REG(n));
}

/* An analog register is written over a serial link; wait for it to go out */
static void bk7258_ana_write(const struct bk7258_cgu_config *config, uint8_t n, uint32_t value)
{
	sys_write32(value, config->ana + ANA_REG(n));

	while ((sys_read32(config->ana + ANA_SPI_STATE) & BIT(n)) != 0U) {
	}
}

/* Raise VDDDIG. ANA_REG(8) to ANA_REG(13) take a write only while latched. */
static void bk7258_vdddig_set(const struct bk7258_cgu_config *config, uint32_t vcorehsel)
{
	uint32_t reg = bk7258_ana_get(config, 9);

	if (FIELD_GET(ANA_REG9_VCOREHSEL, reg) == vcorehsel) {
		return;
	}

	bk7258_ana_write(config, 9, reg | ANA_REG9_SPI_LATCH1V);

	reg = (reg & ~ANA_REG9_VCOREHSEL) | FIELD_PREP(ANA_REG9_VCOREHSEL, vcorehsel);
	bk7258_ana_write(config, 9, reg | ANA_REG9_SPI_LATCH1V);
	bk7258_ana_write(config, 9, reg & ~ANA_REG9_SPI_LATCH1V);

	bk7258_delay_us(DELAY_VDDDIG_SETTLE_US);
}

/*
 * Power the DPLL up and calibrate it. The BootROM leaves it powered down, so
 * this is a cold start every time. There is no lock bit, calibration-done
 * flag or rate readback anywhere in the register map, so the settling times
 * are waited out blind.
 */
static void bk7258_dpll_start(const struct bk7258_cgu_config *config)
{
	uint32_t reg;

	bk7258_ana_write(config, 5, bk7258_ana_get(config, 5) | ANA_REG5_EN_DPLL);

	reg = ANA_REG0_DPLL_SETTINGS;
	bk7258_ana_write(config, 0, reg);
	bk7258_ana_write(config, 0, reg | ANA_REG0_DSPTRIG);
	bk7258_ana_write(config, 0, reg);

	/* SPITRIG is low in those settings, so raising it is the trigger */
	bk7258_delay_us(DELAY_DPLL_TRIGGER_US);
	reg |= ANA_REG0_SPITRIG;
	bk7258_ana_write(config, 0, reg);

	/* Then cycle the unlock detector, letting the loop settle on each edge */
	bk7258_ana_write(config, 0, reg & ~ANA_REG0_SPIDETEN);
	bk7258_delay_us(DELAY_DPLL_DETECT_US);
	bk7258_ana_write(config, 0, reg);
	bk7258_delay_us(DELAY_DPLL_DETECT_US);
}

/*
 * Put the core and CPU0 on the source, and at the rate, that devicetree asks
 * for. Each register takes one read-modify-write store, and the dividers are
 * written before the source so the core never runs from a new source through
 * a divider meant for the old one.
 */
static void bk7258_core_configure(const struct bk7258_cgu_config *config)
{
	uint32_t anaspi;
	uint32_t mode1;
	uint32_t halt;

	if (CORE_SRC_IS_DPLL) {
		bk7258_vdddig_set(config, CORE_VCOREHSEL);
		bk7258_dpll_start(config);

		/*
		 * The analog serial clock is bus_clk / ((field + 1) * 2), so
		 * set the divider the bus is about to need before it rises.
		 */
		anaspi = sys_read32(config->base + SYS_ANASPI_FREQ);
		anaspi = (anaspi & ~ANASPI_FREQ) | FIELD_PREP(ANASPI_FREQ, ANASPI_FREQ_DPLL);
		sys_write32(anaspi, config->base + SYS_ANASPI_FREQ);
	}

	mode1 = sys_read32(config->base + SYS_CLK_DIV_MODE1);
	mode1 = (mode1 & ~CLKDIV_CORE) | FIELD_PREP(CLKDIV_CORE, CORE_CLKDIV);
	sys_write32(mode1, config->base + SYS_CLK_DIV_MODE1);

	halt = sys_read32(config->base + SYS_CPU0_INT_HALT_CLK_OP);
	halt = (halt & ~CPU0_SPEED) | CPU0_SPEED_FIELD;
	sys_write32(halt, config->base + SYS_CPU0_INT_HALT_CLK_OP);

	mode1 = (mode1 & ~CKSEL_CORE) | FIELD_PREP(CKSEL_CORE, CORE_SRC_CKSEL);
	sys_write32(mode1, config->base + SYS_CLK_DIV_MODE1);
}

static int bk7258_cgu_init(const struct device *dev)
{
	const struct bk7258_cgu_config *config = dev->config;
	uint32_t mode1;
	uint32_t core;
	uint32_t cpu0;
	int ret;

	bk7258_core_configure(config);

	/* Report the rates read back from the registers just programmed */
	mode1 = sys_read32(config->base + SYS_CLK_DIV_MODE1);

	ret = bk7258_get_core_clock(mode1, &core);
	if (ret < 0) {
		return ret;
	}

	ret = bk7258_get_cpu0_clock(dev, mode1, &cpu0);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("core %u Hz, cpu0 %u Hz", core, cpu0);

	return 0;
}

static DEVICE_API(clock_control, bk7258_cgu_driver_api) = {
	.on = bk7258_cgu_on,
	.off = bk7258_cgu_off,
	.get_rate = bk7258_cgu_get_rate,
	.get_status = bk7258_cgu_get_status,
};

#define BK7258_CGU_INIT(n)                                                        \
	static const struct bk7258_cgu_config bk7258_cgu_config_##n = {            \
		.base = DT_INST_REG_ADDR_BY_NAME(n, cgu),                          \
		.ana = DT_INST_REG_ADDR_BY_NAME(n, ana),                           \
	};                                                                         \
                                                                                   \
	DEVICE_DT_INST_DEFINE(n, bk7258_cgu_init, NULL, NULL,                      \
			      &bk7258_cgu_config_##n, PRE_KERNEL_1,                \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,                  \
			      &bk7258_cgu_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BK7258_CGU_INIT)
