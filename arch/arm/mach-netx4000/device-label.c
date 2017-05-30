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

//#include <digest.h>

#define DRIVER_DESC "Device-Label driver for Hilscher netX4000 based platforms"
#define DRIVER_NAME "device-label-netx4000"

#define DL_FILE "/boot/devicelabel"

static int __init netx4000_device_label_init(void)
{
	void *buf;
	int len, ret;
	char *p1,*val, var[64];

	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);

	pr_info("%s: Searching for %s", DRIVER_NAME, DL_FILE);
	buf = read_file(DL_FILE, &len);
	if (!buf) {
		pr_err(" ... not found\n");
		return 0;  /*-ENOENT*/
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
environment_initcall(netx4000_device_label_init);

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
