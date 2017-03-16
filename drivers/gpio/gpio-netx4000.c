/*
* GPIO UDC driver for Hilscher netX4000 based platforms
*
* drivers/gpio/gpio-netx4000.c
*
* (C) Copyright 2015 Hilscher Gesellschaft fuer Systemautomation mbH
* http://www.hilscher.com
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation; version 2 of
* the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.test
*
*/

#define DRIVER_DESC  "GPIO driver for Hilscher netx4000 based platforms"
#define DRIVER_NAME  "gpio-netx4000"

#include <common.h>
#include <errno.h>
#include <io.h>
#include <of.h>
#include <gpio.h>
#include <init.h>

#define OFS_GPIO_IN		0x00
#define OFS_GPIO_IN_MASK	0x04
#define OFS_GPIO_OUT		0x08
#define OFS_GPIO_OUT_MASK	0x0C
#define OFS_GPIO_TOGGLE		0x10
#define OFS_GPIO_OUT_MASK1	0x14
#define OFS_GPIO_OE		0x18
#define OFS_GPIO_IRQ		0x1C
#define OFS_GPIO_IRQ_PEDGE	0x20
#define OFS_GPIO_IRQ_NEDGE	0x24

#define GET_NX4KCHIP(x) (container_of(x, struct nx4k_gpio_chip, chip))

struct nx4k_gpio_chip {
	void  __iomem *base;
	struct gpio_chip chip;
	struct nx4k_gpio_regs __iomem *regs;
};

struct nx4k_gpio_regs {
	u32 in;          /* value of the external input pin */
	u32 in_mask;     /* masks out values of 'in' */
	u32 out;         /* value of the output pins */
	u32 out_mask;    /* maks for 'out' */
	u32 toggle;      /* toggle (XOR) output data ('out') */
	u32 out_mask1;   /* alternative value for 'out' */
	u32 oe;          /* sets corresponding oe-pin active high */
	u32 irq_source;  /* irq source */
	u32 irq_pedge;   /* 1-> sensitive on rising edge */
	u32 irq_nedge;   /* 1-> sensitive on falling edge */
};

static void netx4000_gpio_set_value(struct gpio_chip *chip,
					unsigned gpio,
					int value)
{
	struct nx4k_gpio_chip *nx4kchip = GET_NX4KCHIP(chip);
	void __iomem *base = nx4kchip->base;
	u32 val;

	if (!base)
		return;

	val = readl(&nx4kchip->regs->out);

	if (value)
		val |= 1 << gpio;
	else
		val &= ~(1 << gpio);

	writel(val, &nx4kchip->regs->out);
}

static int netx4000_gpio_direction_input(struct gpio_chip *chip,
						unsigned gpio)
{
	struct nx4k_gpio_chip *nx4kchip = GET_NX4KCHIP(chip);
	void __iomem *base = nx4kchip->base;
	u32 val;

	if (!base)
		return -EINVAL;

	val = readl(&nx4kchip->regs->oe);
	val &= ~(1 << gpio);
	writel(val, &nx4kchip->regs->oe);

	return 0;
}

static int netx4000_gpio_direction_output(struct gpio_chip *chip,
						unsigned gpio,
						int value)
{
	struct nx4k_gpio_chip *nx4kchip = GET_NX4KCHIP(chip);
	u32 val;

	val = readl(&nx4kchip->regs->oe);
	val |= (1 << gpio);
	writel(val, &nx4kchip->regs->oe);

	return 0;
}

static int netx4000_gpio_get_value(struct gpio_chip *chip,
					unsigned gpio)
{
	struct nx4k_gpio_chip *nx4kchip = GET_NX4KCHIP(chip);
	u32 val;

	val = readl(&nx4kchip->regs->in);

	return val & (1 << gpio) ? 1 : 0;
}

static struct gpio_ops netx4000_gpio_ops = {
	.direction_input = netx4000_gpio_direction_input,
	.direction_output = netx4000_gpio_direction_output,
	.get = netx4000_gpio_get_value,
	.set = netx4000_gpio_set_value,
};

static int netx4000_gpio_probe(struct device_d *dev)
{
	struct nx4k_gpio_chip *nx4kchip;
	int ret;

	nx4kchip = xzalloc(sizeof(*nx4kchip));
	if (!nx4kchip)
		return -ENOMEM;

	nx4kchip->base = dev_request_mem_region(dev, 0);
	nx4kchip->regs = nx4kchip->base;
	nx4kchip->chip.ops = &netx4000_gpio_ops;
	nx4kchip->chip.dev = dev;
	nx4kchip->chip.base = dev->id * 32;
	nx4kchip->chip.ngpio = 32;

	ret = gpiochip_add(&nx4kchip->chip);

	dev_dbg(dev, "Probing of  gpiochip%d with base %d returned %d\n",
		dev->id,
		nx4kchip->chip.base,
		ret);

	dev_info(dev, "successfully initialized!\n");

	return ret;
}

static __maybe_unused struct of_device_id netx4000_gpio_dt_ids[] = {
	{
		.compatible = "hilscher,netx4000-gpio",
	}, {
		/* sentinel */
	},
};

static struct driver_d netx4000_gpio_driver = {
	.name = DRIVER_NAME,
	.probe = netx4000_gpio_probe,
	.of_compatible = DRV_OF_COMPAT(netx4000_gpio_dt_ids),
};

static int __init netx4000_gpio_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return platform_driver_register(&netx4000_gpio_driver);
}
device_initcall(netx4000_gpio_init);

/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
