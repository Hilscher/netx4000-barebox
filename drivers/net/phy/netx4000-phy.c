/*
* PHY driver for Hilscher netx4000 based platforms
*
* drivers/net/phy/netx4000-phy.c
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
*/

#define DRIVER_DESC "PHY driver for Hilscher netX4000 based platforms"
#define DRIVER_NAME "phy-netx4000"

#include <common.h>
#include <init.h>
#include <linux/phy.h>


static struct phy_driver netx4000_phy_driver = {
	.phy_id = 0x00332002, /* vendor 00:0c:c8, model 0 rev 2 */
	.phy_id_mask = 0xfffffff0,
	.drv.name = DRIVER_NAME,
	.features = PHY_BASIC_FEATURES,
};

static int netx4000_phy_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return phy_driver_register(&netx4000_phy_driver);
}
fs_initcall(netx4000_phy_init);


/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
