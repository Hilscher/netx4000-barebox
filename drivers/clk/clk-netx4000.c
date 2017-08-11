/*
* Clock driver for Hilscher netx4000 based platforms
*
* drivers/clk/clk-netx4000.c
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

#include <common.h>
#include <init.h>
#include <driver.h>
#include <io.h>
#include <malloc.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <mach/hardware.h>

#if defined(CONFIG_OFTREE) && defined(CONFIG_COMMON_CLK_OF_PROVIDER)
/* *************************/
/* Gated peripheral clocks */
/* *************************/

#define NOCPWRCTRL_OFS          (0x40)
#define NOCPWRMASK_OFS          (0x44)
#define NOCPWRSTAT_OFS          (0x48)
#define CLKCFG_OFS              (0x4C)

struct netx4000_hw_clk {
	struct clk clk;
	u32 rate;
	u32 portctrl_mask;
};
#define to_netx4000_hw_clk(p) container_of(p, struct netx4000_hw_clk, clk)

static unsigned long netx4000_clk_recalc_rate(struct clk *hw,
		unsigned long parent_rate)
{
	struct netx4000_hw_clk *netx4000_hw_clk = to_netx4000_hw_clk(hw);
	return netx4000_hw_clk->rate;
}

static int netx4000_clk_enable(struct clk *hw)
{
	struct netx4000_hw_clk *netx4000_hw_clk = to_netx4000_hw_clk(hw);
	u32 mask = netx4000_hw_clk->portctrl_mask;
	void __iomem *base = (void __iomem*)(NETX4000_SYSTEMCTRL_VIRT_BASE);
	uint32_t val;

	/* Check if this clock is allowed to be disabled/enabled by hardware */
	if((ioread32(base + NOCPWRMASK_OFS) & mask) != mask)
		return -EPERM;

	/* Enable clock and power */
	while( (ioread32(base + NOCPWRSTAT_OFS) & mask) != mask) {
		val = ioread32(base + CLKCFG_OFS);
		iowrite32(val | mask, base + CLKCFG_OFS);
		val = ioread32(base + NOCPWRCTRL_OFS);
		iowrite32(val | mask, base + NOCPWRCTRL_OFS);
	}

	return 0;
}

static void netx4000_clk_disable(struct clk *hw)
{
	struct netx4000_hw_clk *netx4000_hw_clk = to_netx4000_hw_clk(hw);
	u32 mask = netx4000_hw_clk->portctrl_mask;
	void __iomem *base = (void __iomem*)(NETX4000_SYSTEMCTRL_VIRT_BASE);
	uint32_t val;

	/* Check if this clock is allowed to be disabled/enabled by hardware */
	if((ioread32(base + NOCPWRMASK_OFS) & mask) != mask)
		return;

	/* Disable clock and power */
	val = ioread32(base + NOCPWRCTRL_OFS);
	iowrite32(val & ~mask, base + NOCPWRCTRL_OFS);
        while( (ioread32(base + NOCPWRSTAT_OFS) & mask) == mask) ;
	val = ioread32(base + CLKCFG_OFS);
	iowrite32(val & ~mask, base + CLKCFG_OFS);
}

static int netx4000_clk_is_enabled(struct clk *hw)
{
	struct netx4000_hw_clk *netx4000_hw_clk = to_netx4000_hw_clk(hw);
	u32 mask = netx4000_hw_clk->portctrl_mask;
	void __iomem *base = (void __iomem*)(NETX4000_SYSTEMCTRL_VIRT_BASE);

	return (ioread32(base + NOCPWRSTAT_OFS) & mask) ? 1 : 0;
}

static const struct clk_ops netx4000_hw_clk_ops = {
	.recalc_rate = netx4000_clk_recalc_rate,
        .enable = netx4000_clk_enable,
        .disable = netx4000_clk_disable,
        .is_enabled = netx4000_clk_is_enabled,
};

void of_netx4000_periph_clk_setup(struct device_node *node)
{
	const char *clk_name = node->name;
	struct netx4000_hw_clk *netx4000_hw_clk;

	netx4000_hw_clk = xzalloc(sizeof(*netx4000_hw_clk));

	of_property_read_string(node, "clock-output-names", &clk_name);

	netx4000_hw_clk->clk.name = clk_name;
	netx4000_hw_clk->clk.ops = &netx4000_hw_clk_ops;

	of_property_read_u32(node, "clock-frequency", &netx4000_hw_clk->rate);
	of_property_read_u32(node, "clock-mask", &netx4000_hw_clk->portctrl_mask);

	clk_register(&netx4000_hw_clk->clk);

	of_clk_add_provider(node, of_clk_src_simple_get, &netx4000_hw_clk->clk);
}
EXPORT_SYMBOL_GPL(of_netx4000_periph_clk_setup);
CLK_OF_DECLARE(netx4000_periph_clk, "hilscher,netx4000-periph-clock", of_netx4000_periph_clk_setup);


/* *************************/
/* CPU main clock */
/* *************************/
static int of_netx4000_cpu_clk_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
        u32 rate = get_netx4000_cpu_rate();

	of_property_read_string(node, "clock-output-names", &clk_name);

	clk = clk_fixed(clk_name, rate);
	if (IS_ERR(clk))
		return IS_ERR(clk);
	return of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(netx4000_cpu_clk, "hilscher,netx4000-cpu-clock", of_netx4000_cpu_clk_setup);
#endif
