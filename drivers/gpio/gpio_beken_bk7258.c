/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT beken_bk7258_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <bk7258_aon.h>
#include <soc.h>

/* One configuration word per pad, shared with the pin controller */
#define PAD_INPUT          BIT(0)
#define PAD_OUTPUT         BIT(1)
#define PAD_INPUT_EN       BIT(2)
#define PAD_OUTPUT_DIS     BIT(3)
#define PAD_PULL_UP        BIT(4)
#define PAD_PULL_EN        BIT(5)
#define PAD_SECOND_FUNC    BIT(6)
#define PAD_INT_TYPE_MASK  GENMASK(11, 10)
#define PAD_INT_EN         BIT(12)
#define PAD_INT_CLR        BIT(13)

#define PAD_INT_LOW_LEVEL  0U
#define PAD_INT_HIGH_LEVEL 1U
#define PAD_INT_RISING     2U
#define PAD_INT_FALLING    3U

struct gpio_bk7258_config {
	struct gpio_driver_config common;
	uintptr_t pad;
	uintptr_t int_status;
};

struct gpio_bk7258_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	/* Pins with their interrupt enabled, and which of those are level-triggered */
	uint32_t int_enabled;
	uint32_t int_level;
};

/* Pad words are updated by read-modify-write, from threads and from the ISR */
static struct k_spinlock gpio_bk7258_lock;

/* Set once every pad interrupt is cleared and the shared interrupt is enabled */
static bool gpio_bk7258_ints_cleared;

/* The ports raise one interrupt, served for all of them */
static const struct device *const gpio_bk7258_ports[] = {
	DT_INST_FOREACH_STATUS_OKAY(DEVICE_DT_INST_GET_COMMA)
};

/* Every port node, enabled or not: base of its pad words and their count */
#define GPIO_BK7258_PORT_PADS(node_id)                                                            \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, beken_bk7258_gpio),                               \
		    ({DT_REG_ADDR_BY_NAME(node_id, pad), DT_PROP(node_id, ngpios)},), ())

static const struct {
	uintptr_t pad;
	uint8_t count;
} gpio_bk7258_all_ports[] = {
	DT_FOREACH_CHILD(DT_INST_PARENT(0), GPIO_BK7258_PORT_PADS)
};

static inline uintptr_t pad_addr(const struct device *dev, gpio_pin_t pin)
{
	const struct gpio_bk7258_config *cfg = dev->config;

	return cfg->pad + (pin * 4U);
}

static int gpio_bk7258_pin_configure(const struct device *dev, gpio_pin_t pin,
				     gpio_flags_t flags)
{
	uint32_t mask = PAD_OUTPUT | PAD_INPUT_EN | PAD_OUTPUT_DIS | PAD_PULL_UP | PAD_PULL_EN |
			PAD_SECOND_FUNC;
	uintptr_t addr = pad_addr(dev, pin);
	k_spinlock_key_t key;
	uint32_t val;
	uint32_t set = 0U;

	if ((flags & GPIO_SINGLE_ENDED) != 0U) {
		return -ENOTSUP;
	}

	if (((flags & GPIO_PULL_UP) != 0U) && ((flags & GPIO_PULL_DOWN) != 0U)) {
		return -EINVAL;
	}

	if ((flags & GPIO_INPUT) != 0U) {
		set |= PAD_INPUT_EN;
	}

	if ((flags & GPIO_OUTPUT) == 0U) {
		set |= PAD_OUTPUT_DIS;
	}

	if ((flags & GPIO_PULL_UP) != 0U) {
		set |= PAD_PULL_EN | PAD_PULL_UP;
	} else if ((flags & GPIO_PULL_DOWN) != 0U) {
		set |= PAD_PULL_EN;
	}

	key = k_spin_lock(&gpio_bk7258_lock);

	val = sys_read32(addr);

	/* Interrupt detection reads the pad through its input path, so an armed pin keeps it */
	if ((val & PAD_INT_EN) != 0U) {
		set |= PAD_INPUT_EN;
	}

	/* The level and the output enable written in one write to prevent glitches */
	if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
		set |= PAD_OUTPUT;
	} else if ((flags & GPIO_OUTPUT_INIT_LOW) == 0U) {
		set |= val & PAD_OUTPUT;
	}

	sys_write32((val & ~mask) | set, addr);

	k_spin_unlock(&gpio_bk7258_lock, key);

	return 0;
}

static int gpio_bk7258_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_bk7258_config *cfg = dev->config;
	uint32_t pins = cfg->common.port_pin_mask;
	gpio_port_value_t val = 0U;

	while (pins != 0U) {
		gpio_pin_t pin = find_lsb_set(pins) - 1;

		if ((sys_read32(pad_addr(dev, pin)) & PAD_INPUT) != 0U) {
			val |= BIT(pin);
		}
		pins &= ~BIT(pin);
	}

	*value = val;

	return 0;
}

static void port_update_output(const struct device *dev, uint32_t pins, uint32_t set,
			       uint32_t toggle)
{
	const struct gpio_bk7258_config *cfg = dev->config;
	k_spinlock_key_t key;

	pins &= cfg->common.port_pin_mask;

	key = k_spin_lock(&gpio_bk7258_lock);

	while (pins != 0U) {
		gpio_pin_t pin = find_lsb_set(pins) - 1;
		uintptr_t addr = pad_addr(dev, pin);
		uint32_t val = sys_read32(addr);

		if ((toggle & BIT(pin)) != 0U) {
			val ^= PAD_OUTPUT;
		} else if ((set & BIT(pin)) != 0U) {
			val |= PAD_OUTPUT;
		} else {
			val &= ~PAD_OUTPUT;
		}
		sys_write32(val, addr);
		pins &= ~BIT(pin);
	}

	k_spin_unlock(&gpio_bk7258_lock, key);
}

static int gpio_bk7258_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					   gpio_port_value_t value)
{
	port_update_output(dev, mask, value, 0U);

	return 0;
}

static int gpio_bk7258_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	port_update_output(dev, pins, pins, 0U);

	return 0;
}

static int gpio_bk7258_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	port_update_output(dev, pins, 0U, 0U);

	return 0;
}

static int gpio_bk7258_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	port_update_output(dev, pins, 0U, pins);

	return 0;
}

static int gpio_bk7258_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	struct gpio_bk7258_data *data = dev->data;
	uintptr_t addr = pad_addr(dev, pin);
	k_spinlock_key_t key;
	uint32_t type;
	uint32_t val;

	if (mode == GPIO_INT_MODE_DISABLED) {
		/* A latched status holds the interrupt line after INT_EN is cleared */
		key = k_spin_lock(&gpio_bk7258_lock);
		sys_write32((sys_read32(addr) & ~PAD_INT_EN) | PAD_INT_CLR, addr);
		data->int_enabled &= ~BIT(pin);
		k_spin_unlock(&gpio_bk7258_lock, key);

		return 0;
	}

	switch (trig & GPIO_INT_TRIG_BOTH) {
	case GPIO_INT_TRIG_LOW:
		type = (mode == GPIO_INT_MODE_EDGE) ? PAD_INT_FALLING : PAD_INT_LOW_LEVEL;
		break;
	case GPIO_INT_TRIG_HIGH:
		type = (mode == GPIO_INT_MODE_EDGE) ? PAD_INT_RISING : PAD_INT_HIGH_LEVEL;
		break;
	default:
		return -ENOTSUP;
	}

	if ((mode != GPIO_INT_MODE_EDGE) && (mode != GPIO_INT_MODE_LEVEL)) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&gpio_bk7258_lock);

	/* Detection reads the pad through its input path */
	val = sys_read32(addr) & ~(PAD_INT_EN | PAD_INT_TYPE_MASK);
	val |= PAD_INPUT_EN | FIELD_PREP(PAD_INT_TYPE_MASK, type);
	sys_write32(val, addr);
	sys_write32(val | PAD_INT_CLR, addr);
	sys_write32(val | PAD_INT_EN, addr);

	data->int_enabled |= BIT(pin);
	if (mode == GPIO_INT_MODE_LEVEL) {
		data->int_level |= BIT(pin);
	} else {
		data->int_level &= ~BIT(pin);
	}

	k_spin_unlock(&gpio_bk7258_lock, key);

	return 0;
}

static int gpio_bk7258_manage_callback(const struct device *dev, struct gpio_callback *callback,
				       bool set)
{
	struct gpio_bk7258_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static uint32_t gpio_bk7258_get_pending_int(const struct device *dev)
{
	const struct gpio_bk7258_config *cfg = dev->config;
	struct gpio_bk7258_data *data = dev->data;

	return sys_read32(cfg->int_status) & data->int_enabled;
}

#ifdef CONFIG_GPIO_GET_DIRECTION
static int gpio_bk7258_port_get_direction(const struct device *dev, gpio_port_pins_t map,
					  gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
	const struct gpio_bk7258_config *cfg = dev->config;
	gpio_port_pins_t in = 0U;
	gpio_port_pins_t out = 0U;

	map &= cfg->common.port_pin_mask;

	while (map != 0U) {
		gpio_pin_t pin = find_lsb_set(map) - 1;
		uint32_t val = sys_read32(pad_addr(dev, pin));

		if ((val & PAD_INPUT_EN) != 0U) {
			in |= BIT(pin);
		}
		if ((val & PAD_OUTPUT_DIS) == 0U) {
			out |= BIT(pin);
		}
		map &= ~BIT(pin);
	}

	if (inputs != NULL) {
		*inputs = in;
	}
	if (outputs != NULL) {
		*outputs = out;
	}

	return 0;
}
#endif /* CONFIG_GPIO_GET_DIRECTION */

static void port_int_clear(const struct device *dev, uint32_t pins)
{
	k_spinlock_key_t key = k_spin_lock(&gpio_bk7258_lock);

	while (pins != 0U) {
		gpio_pin_t pin = find_lsb_set(pins) - 1;
		uintptr_t addr = pad_addr(dev, pin);

		/* The clear bit reads back as zero, so this writes nothing else */
		sys_write32(sys_read32(addr) | PAD_INT_CLR, addr);
		pins &= ~BIT(pin);
	}

	k_spin_unlock(&gpio_bk7258_lock, key);
}

static void port_isr(const struct device *dev)
{
	struct gpio_bk7258_data *data = dev->data;
	uint32_t pending = gpio_bk7258_get_pending_int(dev);

	if (pending == 0U) {
		return;
	}

	/*
	 * Edges are cleared first, so one arriving during the callbacks is
	 * kept. A level is cleared after them: cleared earlier, it would
	 * latch again at once while the level holds.
	 */
	port_int_clear(dev, pending & ~data->int_level);
	gpio_fire_callbacks(&data->callbacks, dev, pending);
	port_int_clear(dev, pending & data->int_level);
}

static void gpio_bk7258_isr(const void *arg)
{
	ARG_UNUSED(arg);

	for (size_t i = 0; i < ARRAY_SIZE(gpio_bk7258_ports); i++) {
		port_isr(gpio_bk7258_ports[i]);
	}
}

static void gpio_bk7258_irq_init(void)
{
	uint32_t r41;
  /* Disable interrupt on all pins by default */
	for (size_t i = 0; i < ARRAY_SIZE(gpio_bk7258_all_ports); i++) {
		for (uint8_t pin = 0; pin < gpio_bk7258_all_ports[i].count; pin++) {
			uintptr_t addr = gpio_bk7258_all_ports[i].pad + (pin * 4U);

			sys_write32((sys_read32(addr) & ~PAD_INT_EN) | PAD_INT_CLR, addr);
		}
	}

	/*
	 * io_drv = 1 runs interrupt detection from the 26 MHz crystal. At the
	 * reset value, 0, it runs from the 32 kHz clock, which misses pulses
	 * shorter than about 30 us and reports edges 30 to 60 us late.
	 */
	r41 = sys_read32(BK7258_AON_PMU_R41);
	r41 = (r41 & ~BK7258_AON_PMU_R41_IO_DRV) | FIELD_PREP(BK7258_AON_PMU_R41_IO_DRV, 1U);
	sys_write32(r41, BK7258_AON_PMU_R41);

	/* One interrupt serves every port */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), gpio_bk7258_isr, NULL, 0);
	bk7258_irq_gate_enable(DT_INST_IRQN(0));
	irq_enable(DT_INST_IRQN(0));
}

static int gpio_bk7258_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	if (!gpio_bk7258_ints_cleared) {
		gpio_bk7258_irq_init();
		gpio_bk7258_ints_cleared = true;
	}

	return 0;
}

static DEVICE_API(gpio, gpio_bk7258_driver_api) = {
	.pin_configure = gpio_bk7258_pin_configure,
	.port_get_raw = gpio_bk7258_port_get_raw,
	.port_set_masked_raw = gpio_bk7258_port_set_masked_raw,
	.port_set_bits_raw = gpio_bk7258_port_set_bits_raw,
	.port_clear_bits_raw = gpio_bk7258_port_clear_bits_raw,
	.port_toggle_bits = gpio_bk7258_port_toggle_bits,
	.pin_interrupt_configure = gpio_bk7258_pin_interrupt_configure,
	.manage_callback = gpio_bk7258_manage_callback,
	.get_pending_int = gpio_bk7258_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_bk7258_port_get_direction,
#endif
};

#define GPIO_BK7258_SAME_IRQ(n)                                                                   \
	BUILD_ASSERT(DT_INST_IRQN(n) == DT_INST_IRQN(0), "BK7258 GPIO ports share one interrupt");

DT_INST_FOREACH_STATUS_OKAY(GPIO_BK7258_SAME_IRQ)

#define GPIO_BK7258_INIT(n)                                                                       \
	static struct gpio_bk7258_data gpio_bk7258_data_##n;                                      \
                                                                                                  \
	static const struct gpio_bk7258_config gpio_bk7258_config_##n = {                         \
		.common = GPIO_COMMON_CONFIG_FROM_DT_INST(n),                                     \
		.pad = DT_INST_REG_ADDR_BY_NAME(n, pad),                                          \
		.int_status = DT_INST_REG_ADDR_BY_NAME(n, int_status),                            \
	};                                                                                        \
                                                                                                  \
	DEVICE_DT_INST_DEFINE(n, gpio_bk7258_init, NULL, &gpio_bk7258_data_##n,                   \
			      &gpio_bk7258_config_##n, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,   \
			      &gpio_bk7258_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_BK7258_INIT)
