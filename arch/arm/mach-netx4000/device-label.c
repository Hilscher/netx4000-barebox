/*
* Device-Label driver for Hilscher netX4000 based platforms
*
* arch/arm/mach-netx4000/device-label.c
*
* (C) Copyright 2017 Hilscher Gesellschaft fuer Systemautomation mbH
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

#include <init.h>
#include <common.h>

#include <libfile.h>
#include <environment.h>
#include <io.h>
#include <net.h>
#include <digest.h>

#define DL_FILE "/boot/devicelabel"

#define MODULE "device-label"

typedef u8 macAddr[6];
struct flash_device_label {
	macAddr macaddr[2];
};

static int __init netx4000_init_devicelabel_env(void)
{
	void *buf;
	int len, ret;
	char *p1,*val, var[64];

	pr_info("%s: Searching for %s", MODULE, DL_FILE);
	buf = read_file(DL_FILE, &len);
	if (!buf) {
		pr_err(" ... not found\n");
		return -ENOENT;
	}
	pr_info(" ... found\n");

	p1 = buf;
	while (p1 < (char*)(buf+len)) {
		/* Skip comment and empty lines */
		if ((*p1 == '#') || (*p1 == '\n')) {
			p1 = strstr(p1, "\n") + 1;
			continue;
		}
		p1 = strtok(p1, "=");
		if (!p1)
			break;
		val = strtok(NULL, "\n");
		snprintf(var, sizeof(var), "devicelabel_%s", p1);
		ret = setenv(var, val);
		if (ret)
		  	pr_err("Setting up environment variable %s=%s failed\n", var, val);
		else
			pr_debug("Setting up environment variable %s=%s\n", var, val);
		p1 = val + strlen(val) + 1;
	}

	return 0;
}

static int netx4000_parse_fdl(struct flash_device_label *fdl)
{
	pr_err("%s: Parsing of Flash-Device-Label currently not supported!\n", MODULE);
	return -1;
}

static int netx4000_parse_devicelabel_env(struct flash_device_label *fdl)
{
	const u8 *envMacAddr = NULL;
	u8 macAddr[18];

	envMacAddr = getenv("devicelabel_eth0_ethaddr");
	if (envMacAddr) {
		if ((envMacAddr[0] == '"') || (envMacAddr[0] == '\''))
			envMacAddr++;
 		snprintf(macAddr, sizeof(macAddr), "%s", envMacAddr);
		string_to_ethaddr(macAddr, (u8 *)&fdl->macaddr[0]);
	}

	envMacAddr = getenv("devicelabel_eth1_ethaddr");
	if (envMacAddr) {
		if ((envMacAddr[0] == '"') || (envMacAddr[0] == '\''))
			envMacAddr++;
 		snprintf(macAddr, sizeof(macAddr), "%s", envMacAddr);
		string_to_ethaddr(envMacAddr, (u8 *)&fdl->macaddr[1]);
	}

	return 0;
}

static int netx4000_create_local_mac_address(macAddr *mac)
{
	struct digest *digest;
	u8 *hash;
	u32 chipid[4], i;
	static u8 addrCounter = 0;

	chipid[0] = readl(0xf80000b0);
	chipid[1] = readl(0xf80000b4);
	chipid[2] = readl(0xf80000b8);
	chipid[3] = readl(0xf80000bc);
	pr_debug("%s: chipid: %08x-%08x-%08x-%08x\n", MODULE, chipid[0], chipid[1], chipid[2], chipid[3]);

	digest = digest_alloc("sha256");
	if (!digest) {
		pr_err("%s: Error: digest_alloc() failed", MODULE);
		return -ENODEV;
	}
	hash = xzalloc(digest_length(digest));

	if (!hash) {
		pr_err("%s: Error: xzalloc() failed", MODULE);
		return -ENOMEM;
	}

	digest_digest(digest, &chipid[0], sizeof(chipid[0]) * 4, hash);
	pr_debug("%s: hash: ", MODULE);
	for (i = 0; i < digest_length(digest); i++)
		pr_debug("%02x", hash[i]);
	pr_debug("\n");
		for (i=5; i; i--) {
		if (i == 5)
			hash[i] += addrCounter++;
		else
			hash[i]++;
		if (hash[i])
			break;
	}
	hash[0] &= ~0x1; /* mark it as individual mac address */
	hash[0] |= 0x2;  /* mark it as local mac address */

	memcpy(mac, hash, sizeof(*mac));

	pr_debug("%s: mac address (%02x:%02x:%02x:%02x:%02x:%02x)\n", MODULE,
		hash[0], hash[1], hash[2], hash[3], hash[4], hash[5]
	);

	free(hash);
	free(digest);

	return 0;
}

static int netx4000_devices_init_fixup(struct device_node *root, void *data)
{
	struct flash_device_label *fdl = data;
	struct device_node *node;

	node = of_find_node_by_path_from(root, "/amba/gmac@f8010000");
	if (node) {
		 of_property_write_u8_array(node, "mac-address", (u8 *)&fdl->macaddr[0], sizeof(macAddr));
	}
	else
		pr_debug("%s: '/amba/gmac@f8010000' not found in DT!\n", MODULE);

	node = of_find_node_by_path_from(root, "/amba/gmac@f8014000");
	if (node) {
		 of_property_write_u8_array(node, "mac-address", (u8 *)&fdl->macaddr[1], sizeof(macAddr));
	}
	else
		pr_debug("%s: '/amba/gmac@f8014000' not found in DT!\n", MODULE);

	node = of_find_node_by_path_from(root, "/amba/p3qsx@f8040000/port0");
	if (node) {
		 of_property_write_u8_array(node, "mac-address", (u8 *)&fdl->macaddr[0], sizeof(macAddr));
	}
	else
		pr_debug("%s: '/amba/p3qsx@f8040000/port0' not found in DT!\n", MODULE);

	node = of_find_node_by_path_from(root, "/amba/p3qsx@f8040000/port1");
	if (node) {
		 of_property_write_u8_array(node, "mac-address", (u8 *)&fdl->macaddr[1], sizeof(macAddr));
	}
	else
		pr_debug("%s: '/amba/p3qsx@f8040000/port1' not found in DT!\n", MODULE);

	return 0;
}

static int netx4000_devices_init(void)
{
	struct flash_device_label *fdl = NULL;
	int err;

	fdl = xzalloc(sizeof(*fdl));
	if (!fdl)
		return -ENOMEM;

	/* Initialize the environment by device-label file. */
	err = netx4000_init_devicelabel_env();
	if (err) {
		pr_debug("%s: =====================================\n", MODULE);
		pr_debug("%s: Invalid or missing Device-Label file!\n", MODULE);
		pr_debug("%s: =====================================\n", MODULE);
	}

	/* Parse Flash-Device-Label stored in SDRAM */
	err = netx4000_parse_fdl(fdl);
	if (err) {
		pr_err("%s: ======================================\n", MODULE);
		pr_err("%s: Invalid or missing Flash-Device-Label!\n", MODULE);
		pr_err("%s: ======================================\n", MODULE);

		/* Parse device-label environment */
		err = netx4000_parse_devicelabel_env(fdl);
	}

	/* Check for valid MAC addresses */
	if (!is_valid_ether_addr(fdl->macaddr[0])) {
		pr_err("%s: Invalid macaddr[0] => Create a local address only for development!\n", MODULE);
		netx4000_create_local_mac_address(&fdl->macaddr[0]);
	}
	if (!is_valid_ether_addr(fdl->macaddr[1])) {
		pr_err("%s: Invalid macaddr[1] => Create a local address only for development!\n", MODULE);
		netx4000_create_local_mac_address(&fdl->macaddr[1]);
	}

	/* Patch barebox device tree */
	netx4000_devices_init_fixup(NULL, fdl);

	/* Register function to fixup kernel device tree */
	of_register_fixup(netx4000_devices_init_fixup, fdl);

	return 0;
}
environment_initcall(netx4000_devices_init);
