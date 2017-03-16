/*
* I2C bus driver for Hilscher netx4000 based platforms
*
* drivers/clocksource/clksrc-netx4000.c
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
* GNU General Public License for more details.
*
*/

#define DRIVER_DESC  "Clocksource driver for Hilscher netx4000 based platforms"
#define DRIVER_NAME "clksrc-netx4000"

#include <common.h>
#include <io.h>
#include <init.h>
#include <clock.h>
#include <linux/clk.h>
#include <mach/netx4000_a9_private.h>

uint64_t global_timer_read(void)
{
	return readl(GLBL_TIMER_COUNTER_LOWER);
}

static struct clocksource cs = {
	.read = global_timer_read,
	.mask = CLOCKSOURCE_MASK(32),
	.shift  = 10,
};

static int clocksource_probe(struct device_d *dev)
{
	struct clk *tmpclk;
	uint32_t rate;

	tmpclk = clk_get(dev, NULL);
	if(IS_ERR(tmpclk))
		return PTR_ERR(tmpclk);

	rate = clk_get_rate(tmpclk);

	writel(0, GLBL_TIMER_CONTROL);

	writel(0, GLBL_TIMER_COUNTER_LOWER);
	writel(0, GLBL_TIMER_COUNTER_UPPER);

	writel(GLBL_TIMER_CONTROL_ENABLE_MASK, GLBL_TIMER_CONTROL);

	cs.mult = clocksource_hz2mult(rate, cs.shift);

	init_clock(&cs);

	return 0;
}

static struct of_device_id netx4000_timer_dt_ids[] = {
	{ .compatible = "arm,cortex-a9-global-timer", },
	{ }
};

static struct driver_d netx4000_clocksource_driver = {
	.name = DRIVER_NAME,
	.probe = clocksource_probe,
	.of_compatible = DRV_OF_COMPAT(netx4000_timer_dt_ids),
};

static int __init netx4000_clocksource_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return platform_driver_register(&netx4000_clocksource_driver);
}
device_initcall(netx4000_clocksource_init);

/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
