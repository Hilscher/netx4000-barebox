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
