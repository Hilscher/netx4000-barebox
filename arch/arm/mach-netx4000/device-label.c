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
#include <crc.h>
#include <linux/ctype.h>

#define MODULE "device-label"

#define DL_FILE "/boot/devicelabel"
#define FDL_RAM_ADDR (void*)0x0505f000

struct fdl_header {
	u8 abStartToken[12];  // Fixed String to detect the begin of the device production data 'ProductData>'
	u16 usLabelSize;      // Size of the complete Label incl. this header and the footer
	u16 usContentSize;
};

struct fdl_basic_device_data {
	u8 dummy[32];
};

struct fdl_mac_addresses_communication_side {
	u8 dummy[64];
};

struct fdl_mac_addresses_application_side {
	u8 macaddr0[6];
	u8 macaddr0_res[2];
	u8 macaddr1[6];
	u8 macaddr1_res[2];
	u8 macaddr2[6];
	u8 macaddr2_res[2];
	u8 macaddr3[6];
	u8 macaddr3_res[2];
};

struct fdl_product_identification_information {
	u8 dummy[112];
};

struct fdl_oem_identification {
	u8 dummy[236];
};

struct fdl_flash_layout {
	u8 dummy[488];
};

struct fdl_footer {
	u32 ulCRC;          // CRC-32 (IEEE 802.3) of Content
	u8 abEndToken[12];  //Fixed String to detect the end of the device production data: '<ProductData'
};

struct fdl_content{
	struct fdl_basic_device_data bdd;
	struct fdl_mac_addresses_communication_side macAddrCom;
	struct fdl_mac_addresses_application_side  macAddrApp;
	struct fdl_product_identification_information pii;
	struct fdl_oem_identification oi;
	struct fdl_flash_layout fl;
};

typedef u8 macAddr[6];
struct device_label {
	macAddr macaddr[4];
};

/* Parse device-label file and initialize barebox environment. */
static int __init netx4000_init_env_by_dlf(void)
{
	void *buf;
	int len, ret;
	char *p1,*val, var[64];

	pr_info("%s: Searching for %s ... ", MODULE, DL_FILE);
	buf = read_file(DL_FILE, &len);
	if (!buf) {
		pr_err("not found\n");
		return -ENOENT;
	}
	pr_info("found\n");

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
		snprintf(var, sizeof(var), "dlf_%s", p1);
		ret = setenv(var, val);
		if (ret)
		  	pr_err("Setting up environment variable %s=%s failed\n", var, val);
		else
			pr_debug("Setting up environment variable %s=%s\n", var, val);
		p1 = val + strlen(val) + 1;
	}

	return 0;
}

/* Parse flash-device-label stored in SDRAM and initialize device label. */
static int netx4000_init_dl_by_fdl(struct device_label *dl)
{
	struct fdl_header const *header = FDL_RAM_ADDR;
	struct fdl_content const *content = (void*)header + sizeof(*header);
	struct fdl_footer const *footer = (void*)content + header->usContentSize;

	pr_info("%s: Searching for FDL at 0x%p ...", MODULE, header);

	while (1) {
		u32 crc;

		/* Verify mandatory fdl values --> */
		if (strncmp(header->abStartToken, "ProductData>", sizeof(header->abStartToken)) != 0) {
			u8 buf[sizeof(header->abStartToken)], i;

			for(i = 0; i < sizeof(header->abStartToken); i++) {
				buf[i] = (isprint(header->abStartToken[i])) ? header->abStartToken[i] : '.';
			}
			pr_err("\n%s: Start token mismatch ('%.*s' instead of '%s')\n", MODULE, sizeof(header->abStartToken), buf, "ProductData>");
			break;
		}
		if (header->usLabelSize > 4096 ) {
			pr_err("\n%s: Length exceeds 4KiB range (%d)\n", MODULE, header->usLabelSize);
			break;
		}
		if (header->usContentSize > (4096 - sizeof(*header) - sizeof(*footer))) {
			pr_err("\n%s: Length exceeds 4KiB-32B range (%d)\n", MODULE, header->usContentSize);
			break;
		}
		if (strncmp(footer->abEndToken, "<ProductData", sizeof(footer->abEndToken)) != 0) {
			u8 buf[sizeof(header->abStartToken)], i;

			for(i = 0; i < sizeof(header->abStartToken); i++) {
				buf[i] = (isprint(header->abStartToken[i])) ? header->abStartToken[i] : '.';
			}
			pr_err("\n%s: End token mismatch ('%.*s' instead of '%s')\n", MODULE, sizeof(footer->abEndToken), buf, "<ProductData");
			break;
		}
		crc = crc32(0, (void*)content, header->usContentSize);
		if (footer->ulCRC != crc) {
			pr_err("\n%s: CRC mismatch (0x%08x instead of 0x%08x)\n", MODULE, footer->ulCRC, crc);
			break;
		}
		/* <-- Verify mandatory fdl values */

		pr_debug("%s: mac addr 0: %02x:%02x:%02x:%02x:%02x:%02x\n", MODULE,
			content->macAddrApp.macaddr0[0], content->macAddrApp.macaddr0[1], content->macAddrApp.macaddr0[2],
			content->macAddrApp.macaddr0[3], content->macAddrApp.macaddr0[4], content->macAddrApp.macaddr0[5]
		);

		pr_debug("%s: mac addr 1: %02x:%02x:%02x:%02x:%02x:%02x\n", MODULE,
			content->macAddrApp.macaddr1[0], content->macAddrApp.macaddr1[1], content->macAddrApp.macaddr1[2],
			content->macAddrApp.macaddr1[3], content->macAddrApp.macaddr1[4], content->macAddrApp.macaddr1[5]
		);

		pr_debug("%s: mac addr 2: %02x:%02x:%02x:%02x:%02x:%02x\n", MODULE,
			content->macAddrApp.macaddr2[0], content->macAddrApp.macaddr2[1], content->macAddrApp.macaddr2[2],
			content->macAddrApp.macaddr2[3], content->macAddrApp.macaddr2[4], content->macAddrApp.macaddr2[5]
		);

		pr_debug("%s: mac addr 3: %02x:%02x:%02x:%02x:%02x:%02x\n", MODULE,
			content->macAddrApp.macaddr3[0], content->macAddrApp.macaddr3[1], content->macAddrApp.macaddr3[2],
			content->macAddrApp.macaddr3[3], content->macAddrApp.macaddr3[4], content->macAddrApp.macaddr3[5]
		);

		memcpy(dl->macaddr[0], content->macAddrApp.macaddr0, sizeof(dl->macaddr[0]));
		memcpy(dl->macaddr[1], content->macAddrApp.macaddr1, sizeof(dl->macaddr[1]));
		memcpy(dl->macaddr[2], content->macAddrApp.macaddr2, sizeof(dl->macaddr[2]));
		memcpy(dl->macaddr[3], content->macAddrApp.macaddr3, sizeof(dl->macaddr[3]));

		pr_info(" ... found\n");
		return 0;
	}

	return -1;
}

/* Parse barebox environment and initialize device label. */
static int netx4000_init_dl_by_env(struct device_label *dl)
{
	const u8 *envMacAddr = NULL;
	u8 macAddr[18];

	envMacAddr = getenv("dlf_gmac0_ethaddr");
	if (envMacAddr) {
		if ((envMacAddr[0] == '"') || (envMacAddr[0] == '\''))
			envMacAddr++;
		snprintf(macAddr, sizeof(macAddr), "%s", envMacAddr);
		string_to_ethaddr(macAddr, (u8 *)&dl->macaddr[0]);
	}

	envMacAddr = getenv("dlf_gmac1_ethaddr");
	if (envMacAddr) {
		if ((envMacAddr[0] == '"') || (envMacAddr[0] == '\''))
			envMacAddr++;
		snprintf(macAddr, sizeof(macAddr), "%s", envMacAddr);
		string_to_ethaddr(macAddr, (u8 *)&dl->macaddr[1]);
	}

	envMacAddr = getenv("dlf_xceth0_ethaddr");
	if (envMacAddr) {
		if ((envMacAddr[0] == '"') || (envMacAddr[0] == '\''))
			envMacAddr++;
		snprintf(macAddr, sizeof(macAddr), "%s", envMacAddr);
		string_to_ethaddr(macAddr, (u8 *)&dl->macaddr[2]);
	}

	envMacAddr = getenv("dlf_xceth1_ethaddr");
	if (envMacAddr) {
		if ((envMacAddr[0] == '"') || (envMacAddr[0] == '\''))
			envMacAddr++;
		snprintf(macAddr, sizeof(macAddr), "%s", envMacAddr);
		string_to_ethaddr(macAddr, (u8 *)&dl->macaddr[3]);
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

#define show_mac_addr(if_name, mac_addr) { \
	pr_info("%s: %s %02x:%02x:%02x:%02x:%02x:%02x %s\n", MODULE, if_name, \
		mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], \
		((mac_addr[0] & 0x3) == 0x2) ? "not for production use" : ""); \
}

static int netx4000_devices_init_fixup(struct device_node *root, void *data)
{
	struct device_label *dl = data;
	struct device_node *node;
	u32 chipid[4];

	chipid[0] = readl(0xf80000b0);
	chipid[1] = readl(0xf80000b4);
	chipid[2] = readl(0xf80000b8);
	chipid[3] = readl(0xf80000bc);

	node = of_find_node_by_path_from(root, "/cpus");
	if(node) {
		char* serialnr = basprintf("%08x-%08x-%08x-%08x", chipid[0], chipid[1], chipid[2], chipid[3]);
		of_property_write_string(node, "serial-nr", serialnr);
		free(serialnr);
	} else
		pr_debug("%s: '/cpus' not found in DT!\n", MODULE);

	/* GMAC */
	node = of_find_node_by_path_from(root, "/amba/gmac@f8010000");
	if (node) {
		of_property_write_u8_array(node, "mac-address", (u8 *)&dl->macaddr[0], sizeof(macAddr));
		if(!root)
			show_mac_addr("gmac0", dl->macaddr[0]);
	}

	node = of_find_node_by_path_from(root, "/amba/gmac@f8014000");
	if (node) {
		of_property_write_u8_array(node, "mac-address", (u8 *)&dl->macaddr[1], sizeof(macAddr));
		if(!root)
			show_mac_addr("gmac1", dl->macaddr[1]);
	}

	/* 3port switch */
	node = of_find_node_by_path_from(root, "/amba/p3qsx@f8040000/port0");
	if (node) {
		of_property_write_u8_array(node, "mac-address", (u8 *)&dl->macaddr[0], sizeof(macAddr));
		if(!root)
			show_mac_addr("p3qsx0", dl->macaddr[0]);
	}

	node = of_find_node_by_path_from(root, "/amba/p3qsx@f8040000/port1");
	if (node) {
		of_property_write_u8_array(node, "mac-address", (u8 *)&dl->macaddr[1], sizeof(macAddr));
		if(!root)
			show_mac_addr("p3qsx1", dl->macaddr[1]);
	}

	/* xceth */
	node = of_find_node_by_path_from(root, "/xceth0");
	if (node) {
		of_property_write_u8_array(node, "mac-address", (u8 *)&dl->macaddr[2], sizeof(macAddr));
		if(!root)
			show_mac_addr("xceth0", dl->macaddr[2]);
	}

	node = of_find_node_by_path_from(root, "/xceth1");
	if (node) {
		of_property_write_u8_array(node, "mac-address", (u8 *)&dl->macaddr[3], sizeof(macAddr));
		if(!root)
			show_mac_addr("xceth1", dl->macaddr[3]);
	}

	return 0;
}

static int netx4000_devices_init(void)
{
	struct device_label *dl = NULL;
	int err;

	dl = xzalloc(sizeof(*dl));
	if (!dl)
		return -ENOMEM;

	/* Parse device-label file and initialize barebox environment. */
	err = netx4000_init_env_by_dlf();
	if (err) {
		pr_debug("%s: =====================================\n", MODULE);
		pr_debug("%s: Invalid or missing Device-Label file!\n", MODULE);
		pr_debug("%s: =====================================\n", MODULE);
	}

	/* Parse flash-device-label stored in SDRAM and initialize device label. */
	err = netx4000_init_dl_by_fdl(dl);
	if (err) {
		pr_err("%s: ======================================\n", MODULE);
		pr_err("%s: Invalid or missing Flash-Device-Label!\n", MODULE);
		pr_err("%s: ======================================\n", MODULE);

		/* Parse barebox environment and initialize device label. */
		err = netx4000_init_dl_by_env(dl);
	}

	/* Check for valid MAC addresses */
	if (!is_valid_ether_addr(dl->macaddr[0])) {
		netx4000_create_local_mac_address(&dl->macaddr[0]);
	}
	if (!is_valid_ether_addr(dl->macaddr[1])) {
		netx4000_create_local_mac_address(&dl->macaddr[1]);
	}
	if (!is_valid_ether_addr(dl->macaddr[2])) {
		netx4000_create_local_mac_address(&dl->macaddr[2]);
	}
	if (!is_valid_ether_addr(dl->macaddr[3])) {
		netx4000_create_local_mac_address(&dl->macaddr[3]);
	}

	/* Patch barebox device tree */
	netx4000_devices_init_fixup(NULL, dl);

	/* Register function to fixup kernel device tree */
	of_register_fixup(netx4000_devices_init_fixup, dl);

	return 0;
}
environment_initcall(netx4000_devices_init);
