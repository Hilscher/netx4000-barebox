/*
 * @file
 * @brief netx4000 Generic platform initialzation code
 *
 * (C) Copyright 2014 Hilscher Gesellschaft fuer Systemautomation mbH
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

void __noreturn reset_cpu(unsigned long addr)
{
	// TODO: perform reset by writing ASIC_CTRL / RESET_CTRL
	/* Not reached */
	while (1);
}

uint32_t get_netx4000_cpu_rate(void)
{
	volatile u32* ulRAPSysCtrlBootMode = (volatile u32*)0xf8000000;

	/* get current PLL speed */
	if ((*ulRAPSysCtrlBootMode & (1<<8)) == (1<<8))
		return (600*1000*1000);/* 600MHZ */
	else
		return (400*1000*1000);/* 400MHZ */
}

static int netx4000_init(void)
{
	struct device_node *root;

	root = of_get_root_node();
	if (root) {

		return 0;
	}

	/*NOTE: We should never reach this point, since the current */
	/*      implementation only supports DT. Otherwise we have  */
	/*      to setup all devices manually.                      */
	return -EINVAL;

}
postcore_initcall(netx4000_init);
