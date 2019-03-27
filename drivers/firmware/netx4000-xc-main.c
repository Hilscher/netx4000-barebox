/*
* XC firmware handler for Hilscher netx4000 based platforms
*
* drivers/firmware/netx4000-xc-main.c
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

#include <common.h>
#include <init.h>
#include <linux/list.h>

#include <firmware.h>
#include <firmware/netx4000-xc.h>

#include <mach/regdef_netx4000.h>
#include "netx4000-xc-hal.h"

#define DRIVER_DESC  "xc firmware handler for Hilscher netx4000 based platforms"
#define DRIVER_NAME "netx4000-xc"

#define MAX_FW_BUF  (64*1024)

/* Regdef */

#define regdef(mask,shift,name) \
static inline int g##name(int val) { \
	return (val >> shift) & mask; \
} \
static inline int s##name(int val) { \
	return (val & mask) << shift; \
}

#define ASIC_CTRL_ACCESS_KEY	(volatile void*)0xf408017c
#define CLOCK_ENABLE			(volatile void*)0xf4080138
regdef (0x1, 16, XC_MISC)
regdef (0x1, 11, XMAC11)
regdef (0x1, 10, XMAC10)
regdef (0x1,  9, XMAC01)
regdef (0x1,  8, XMAC00)
regdef (0x1,  7, TPEC11)
regdef (0x1,  6, TPEC10)
regdef (0x1,  5, TPEC01)
regdef (0x1,  4, TPEC00)
regdef (0x1,  3, RPEC11)
regdef (0x1,  2, RPEC10)
regdef (0x1,  1, RPEC01)
regdef (0x1,  0, RPEC00)

#define XC_GLOBAL_CLOCK_ENABLE (sXC_MISC(1))
#define XC_PORT00_CLOCK_EN (sRPEC00(1)|sTPEC00(1)|sXMAC00(1))
#define XC_PORT01_CLOCK_EN (sRPEC01(1)|sTPEC01(1)|sXMAC01(1))
#define XC_PORT10_CLOCK_EN (sRPEC10(1)|sTPEC10(1)|sXMAC10(1))
#define XC_PORT11_CLOCK_EN (sRPEC11(1)|sTPEC11(1)|sXMAC11(1))

struct priv_data {
	struct firmware_handler fh;
	struct device_d *dev;
	char *fwbuf, *pfwbuf;
	uint32_t used_xc_ports;
};

/*
-------------------------------------------------------------------------------------------------------------------------
|                            xc0                            |                            xc1                            | eXcNo
-------------------------------------------------------------------------------------------------------------------------
|            xcPort0          |            xcPort1          |            xcPort0          |            xcPort1          | eXcPortNo
-------------------------------------------------------------------------------------------------------------------------
| rpec0 | tpec0 | rpu0 | tpu0 | rpec1 | tpec1 | rpu1 | tpu1 | rpec2 | tpec2 | rpu2 | tpu2 | rpec3 | tpec3 | rpu3 | tpu3 | eXcPortType
-------------------------------------------------------------------------------------------------------------------------
*/

typedef enum {
	XC0 = 0,
	XC1,
	XC_MAX
} eXcNo_t;

typedef enum {
	XC_PORT0 = 0,
	XC_PORT1,
	XC_PORT_MAX
} eXcPortNo_t;

typedef enum {
	XC_PORT_TYPE_RPEC = 0,
	XC_PORT_TYPE_TPEC,
	XC_PORT_TYPE_RPU,
	XC_PORT_TYPE_TPU,
	XC_PORT_TYPE_MAX
} eXcPortType_t;

struct xcunit {
	char name[8];
	eXcNo_t uXcNo;
	eXcPortNo_t uXcPortNo;
	eXcPortType_t eXcPortType;
};

#define for_each_xc(xc) \
for (int xc=0; xc<XC_MAX; xc++)

#define for_each_xc_port(xc,port) \
for_each_xc(xc) \
	for (int port=0; port<XC_PORT_MAX; port++)

#define for_each_xc_port_type(xc,port,type) \
for_each_xc_port(xc,port) \
	for (int type=0; type<XC_PORT_TYPE_MAX; type++)

#define xcunit_index(xc,port,type) ((xc*XC_PORT_MAX*XC_PORT_TYPE_MAX) + (port*XC_PORT_TYPE_MAX) + type)

struct xcunit xcunit[] = {
	{ "rpec0", XC0, XC_PORT0, XC_PORT_TYPE_RPEC},
	{ "tpec0", XC0, XC_PORT0, XC_PORT_TYPE_TPEC},
	{ "rpu0",  XC0, XC_PORT0, XC_PORT_TYPE_RPU},
	{ "tpu0",  XC0, XC_PORT0, XC_PORT_TYPE_TPU},
	{ "rpec1", XC0, XC_PORT1, XC_PORT_TYPE_RPEC},
	{ "tpec1", XC0, XC_PORT1, XC_PORT_TYPE_TPEC},
	{ "rpu1",  XC0, XC_PORT1, XC_PORT_TYPE_RPU},
	{ "tpu1",  XC0, XC_PORT1, XC_PORT_TYPE_TPU},
	{ "rpec2", XC1, XC_PORT0, XC_PORT_TYPE_RPEC},
	{ "tpec2", XC1, XC_PORT0, XC_PORT_TYPE_TPEC},
	{ "rpu2",  XC1, XC_PORT0, XC_PORT_TYPE_RPU},
	{ "tpu2",  XC1, XC_PORT0, XC_PORT_TYPE_TPU},
	{ "rpec3", XC1, XC_PORT1, XC_PORT_TYPE_RPEC},
	{ "tpec3", XC1, XC_PORT1, XC_PORT_TYPE_TPEC},
	{ "rpu3",  XC1, XC_PORT1, XC_PORT_TYPE_RPU},
	{ "tpu3",  XC1, XC_PORT1, XC_PORT_TYPE_TPU}
};

typedef enum {
	fw_loaded = 0,
	fw_requested,
	fw_resetted,
	fw_uploaded,
	fw_started,
	fw_stopped
} eFwStatus_t;

struct firmware_data {
	struct list_head list;
	struct device_d *owner;
	eFwStatus_t status;
	char firmware[16];
	char version[16];
	uint32_t *xc_microcode[XC_MAX][XC_PORT_MAX][XC_PORT_TYPE_MAX];
	struct xc_res *xc_port_res[XC_MAX][XC_PORT_MAX];
};

static LIST_HEAD(firmware_data_list);

/* FIXME: Enable clock */
static void clock_enable(uint32_t mask)
{
	uint32_t keyval, val;

	val = ioread32(CLOCK_ENABLE);
	keyval = ioread32(ASIC_CTRL_ACCESS_KEY);
	iowrite32(keyval, ASIC_CTRL_ACCESS_KEY);
	iowrite32(val|mask, CLOCK_ENABLE);
}

/* ioctl command functions */

static int netx4000_xc_ioctl_fw_request(struct firmware_handler *fh, void *ptr)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;
	struct ioctl_request *req = ptr;
	struct firmware_data *fwdata;
	uint32_t reqData = 0;
	int rc = -ENODEV;

	dev_dbg(dev, "%s: +++ (*fh=%p, *ptr=%p)\n", __func__, fh, ptr);

	list_for_each_entry(fwdata, &firmware_data_list, list) {
		if (strcmp(fwdata->firmware, req->id))
			continue;

		/* Check ownership */
		if (fwdata->owner && (fwdata->owner != req->dev)) {
			dev_err(dev, "Firmware %s is owned by %s!\n", fwdata->firmware, fwdata->owner->name);
			return -EACCES;
		}

		/* Check for already requested ports */
		for_each_xc_port(xc, port) {
			uint32_t uPortNo = (xc*XC_MAX)+port;
			if (fwdata->xc_port_res[xc][port] && (priv->used_xc_ports & (1<<uPortNo)))
				return -EBUSY;
		}

		fwdata->owner = req->dev;

		for_each_xc_port_type(xc, port, type) {
			uint32_t uPortNo = (xc*XC_MAX)+port;

			if (fwdata->xc_port_res[xc][port])
				break;

			if (!fwdata->xc_microcode[xc][port][type])
				continue;

			fwdata->xc_port_res[xc][port] = xc_alloc_xc_res(dev, uPortNo);
			if (!fwdata->xc_port_res[xc][port]) {
				dev_err(dev, "%s: xc_alloc_xc_res() failed!\n", __func__);
				return -ENOMEM;
			}

			/* Mark XC port as used */
			priv->used_xc_ports |= (1<<uPortNo);

			/* Enable all required XC clocks */
			clock_enable(XC_PORT00_CLOCK_EN<<uPortNo);

			/* TODO: */
			reqData |= (1<<uPortNo);
		}
		fwdata->status = fw_requested;
		*(uint32_t*)req->data = reqData;
		return 0;
	}
	return rc;
}

// static int netx4000_xc_ioctl_fw_release(struct firmware_handler *fh, void *ptr)
// {
// 	struct priv_data *priv = container_of(fh, struct priv_data, fh);
// 	struct device_d *dev = priv->dev;
// 	struct ioctl_request *req = ptr;
// 	struct firmware_data *fwdata;
//
// 	dev_dbg(dev, "%s: +++ (*fh=%p, *ptr=%p)\n", __func__, fh, ptr);
//
// 	return -EINVAL;
// }

static int netx4000_xc_ioctl_fw_upload(struct firmware_handler *fh, void *ptr)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;
	struct ioctl_request *req = ptr;
	struct firmware_data *fwdata;
	int rc = -ENODEV;

	dev_dbg(dev, "%s: +++ (*fh=%p, *ptr=%p)\n", __func__, fh, ptr);

	list_for_each_entry(fwdata, &firmware_data_list, list) {
		if (strcmp(fwdata->firmware, req->id))
			continue;

		/* Check ownership */
		if (fwdata->owner != req->dev) {
			dev_err(dev, "Firmware %s is owned by %s!\n", fwdata->firmware, (fwdata->owner) ? fwdata->owner->name : "nobody");
			return -EACCES;
		}

		/* Check firmware status */
		switch (fwdata->status) {
			case fw_resetted:
				break;
			default:
				dev_err(dev, "Uploading firmware %s failed!\n", fwdata->firmware);
				return -EACCES;
		}

		for_each_xc_port_type(xc, port, type) {
			if (!fwdata->xc_microcode[xc][port][type])
				continue;

			rc = xc_load(fwdata->xc_port_res[xc][port], type, fwdata->xc_microcode[xc][port][type]);
			if (rc)
				return rc;
		}
		fwdata->status = fw_uploaded;
		return 0;
	}
	return rc;
}

static int netx4000_xc_ioctl_fw_reset(struct firmware_handler *fh, void *ptr)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;
	struct ioctl_request *req = ptr;
	struct firmware_data *fwdata;
	int rc = -ENODEV;

	dev_dbg(dev, "%s: +++ (*fh=%p, *ptr=%p)\n", __func__, fh, ptr);

	list_for_each_entry(fwdata, &firmware_data_list, list) {
		if (strcmp(fwdata->firmware, req->id))
			continue;

		/* Check ownership */
		if (fwdata->owner != req->dev) {
			dev_err(dev, "Firmware %s is owned by %s!\n", fwdata->firmware, (fwdata->owner) ? fwdata->owner->name : "nobody");
			return -EACCES;
		}

		/* Check firmware status */
		switch (fwdata->status) {
			case fw_requested:
			case fw_stopped:
				break;
			default:
				dev_err(dev, "Resetting firmware %s failed!\n", fwdata->firmware);
				return -EACCES;
		}

		for_each_xc_port(xc, port) {
			if (!fwdata->xc_port_res[xc][port])
				continue;

			rc = xc_reset(fwdata->xc_port_res[xc][port]);
			if (rc)
				return rc;
		}
		fwdata->status = fw_resetted;
		return 0;
	}
	return rc;
}

static int netx4000_xc_ioctl_fw_start(struct firmware_handler *fh, void *ptr)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;
	struct ioctl_request *req = ptr;
	struct firmware_data *fwdata;
	int rc = -ENODEV;

	dev_dbg(dev, "%s: +++ (*fh=%p, *ptr=%p)\n", __func__, fh, ptr);

	list_for_each_entry(fwdata, &firmware_data_list, list) {
		if (strcmp(fwdata->firmware, req->id))
			continue;

		/* Check ownership */
		if (fwdata->owner != req->dev) {
			dev_err(dev, "Firmware %s is owned by %s!\n", fwdata->firmware, (fwdata->owner) ? fwdata->owner->name : "nobody");
			return -EACCES;
		}

		/* Check firmware status */
		switch (fwdata->status) {
			case fw_uploaded:
			case fw_stopped:
				break;
			default:
				dev_err(dev, "Starting firmware %s failed!\n", fwdata->firmware);
				return -EACCES;
		}

		for_each_xc_port(xc, port) {
			if (!fwdata->xc_port_res[xc][port])
				continue;

			rc = xc_start(fwdata->xc_port_res[xc][port]);
			if (rc)
				return rc;
		}
		fwdata->status = fw_started;
		return 0;
	}
	return rc;
}

static int netx4000_xc_ioctl_fw_stop(struct firmware_handler *fh, void *ptr)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;
	struct ioctl_request *req = ptr;
	struct firmware_data *fwdata;
	int rc = -ENODEV;

	dev_dbg(dev, "%s: +++ (*fh=%p, *ptr=%p)\n", __func__, fh, ptr);

	list_for_each_entry(fwdata, &firmware_data_list, list) {
		if (strcmp(fwdata->firmware, req->id))
			continue;

		/* Check ownership */
		if (fwdata->owner != req->dev) {
			dev_err(dev, "Firmware %s is owned by %s!\n", fwdata->firmware, (fwdata->owner) ? fwdata->owner->name : "nobody");
			return -EACCES;
		}

		/* Check firmware status */
		switch (fwdata->status) {
			case fw_started:
				break;
			default:
				dev_err(dev, "Stopping firmware %s failed!\n", fwdata->firmware);
				return -EACCES;
		}

		for_each_xc_port(xc, port) {
			if (!fwdata->xc_port_res[xc][port])
				continue;

			rc = xc_stop(fwdata->xc_port_res[xc][port]);
			if (rc)
				return rc;
		}
		fwdata->status = fw_stopped;
		return 0;
	}
	return rc;
}

/* Code to support firmware files provided in DT format */

static int netx4000_xc_parse_fw_dtb(struct firmware_handler *fh, char *buf)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;
	struct device_node *root, *node;
	struct property *prop;
	struct firmware_data *fwdata = NULL;
	int rc = 0;

	dev_dbg(dev, "%s: +++ (*fh=%p, *buf=%p)\n", __func__, fh, buf);

	root = of_unflatten_dtb_const(buf);
	if (IS_ERR(root))
		return PTR_ERR(root);

	dev_dbg(dev, "Firmware in DT format found:\n");

	node = of_find_node_by_path_from(root, "/");
	if (node) {
		void *pv;

		rc = of_property_read_string(node, "firmware", (const char**)&pv);
		if (rc) {
			dev_err(dev, "Reading firmware name failed!\n");
			rc = -ENODATA;
			goto err_out;
		}

		/* Check if firmware is already loaded */
		list_for_each_entry(fwdata, &firmware_data_list, list) {
			if (!strncmp(fwdata->firmware, pv, sizeof(fwdata->firmware))) {
				dev_err(dev, "Firmware %s is already loaded!\n", (char*)pv);
				rc = -EACCES;
				goto err_out;
			}
		}

		/* Allocate memory to store firmware related informations. */
		fwdata = xzalloc(sizeof(*fwdata));
		if (!fwdata) {
			dev_err(dev, "%s: xzalloc() failed!\n", __func__);
			rc = -ENOMEM;
			goto err_out;
		}

		dev_info(dev, "firmware = %s\n", (char*)pv);
		strncpy(fwdata->firmware, pv, sizeof(fwdata->firmware));

		rc = of_property_read_string(node, "version", (const char**)&pv);
		if (rc) {
			dev_err(dev, "Reading firmware version failed!\n");
			rc = -ENODATA;
			goto err_out;
		}
		dev_info(dev, "version = %s\n", (char*)pv);
		strncpy(fwdata->version, pv, sizeof(fwdata->version));

		for_each_xc_port_type(xc, port, type) {
			struct device_node *mc_node = node;
			int i = xcunit_index(xc, port, type);

			/* Check for microcode in root node */
			prop = of_find_property(mc_node, xcunit[i].name, NULL);
			if (!prop) {
				/* Check for microcode in child nodes */
				mc_node = NULL;
				while ((mc_node = of_get_next_available_child(node, mc_node))) {
					prop = of_find_property(mc_node, xcunit[i].name, NULL);
					if (prop)
						break;
				}
				if (!prop)
					continue;
			}

			fwdata->xc_microcode[xc][port][type] = xzalloc(max(8 /* program size, trailing loads size */, prop->length));
			if (!fwdata->xc_microcode[xc][port][type]) {
				dev_err(dev, "%s: xzalloc() failed!\n", __func__);
				rc = -ENOMEM;
				goto err_out;
			}

			rc = of_property_read_u32_array(mc_node, xcunit[i].name, fwdata->xc_microcode[xc][port][type], prop->length/sizeof(uint32_t));
			if (IS_ERR((const void*)rc)) {
				dev_err(dev, "Reading microcode failed!\n");
				rc = PTR_ERR((const void*)rc);
				goto err_out;
			}
		}
		list_add_tail(&fwdata->list, &firmware_data_list);
	}

	of_delete_node(root);
	return rc;

err_out:
	for_each_xc_port_type(xc, port, type) {
		if (fwdata->xc_microcode[xc][port][type])
			free(fwdata->xc_microcode[xc][port][type]);
	}

	if (fwdata)
		free(fwdata);

	of_delete_node(root);
	return rc;
}

/* Code to support firmware files provided in binary format */

static int netx4000_xc_parse_fw_binary(struct firmware_handler *fh, char *buf)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;

	dev_dbg(dev, "%s: +++ (*fh=%p, *buf=%p)\n", __func__, fh, buf);

	dev_err(dev, "Firmware in binary format currently not supported!\n");

	return -ENOTSUPP;
}

/* Firmware handler ops */

static int netx4000_xc_open(struct firmware_handler *fh)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;

	dev_dbg(dev, "%s: +++ (*fh=%p)\n", __func__, fh);

	return 0;
}

static int netx4000_xc_write(struct firmware_handler *fh, const void *buf, size_t sz)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;

	dev_dbg(dev, "%s: +++ (*fh=%p, *buf=%p, sz=%u)\n", __func__, fh, buf, sz);

	if (!priv->fwbuf) {
		priv->fwbuf = xzalloc(MAX_FW_BUF);
		priv->pfwbuf = priv->fwbuf;
	}

	memcpy(priv->pfwbuf, buf, sz);
	priv->pfwbuf += sz;

	return 0;
}

static int netx4000_xc_close(struct firmware_handler *fh)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;
	int rc = 0;

	dev_dbg(dev, "%s: +++ (*fh=%p)\n", __func__, fh);

	if (priv->fwbuf) {
		do {
			rc = netx4000_xc_parse_fw_dtb(fh, priv->fwbuf);
			if (!rc)
				break;

			rc = netx4000_xc_parse_fw_binary(fh, priv->fwbuf);
			if (!rc)
				break;

			dev_err(dev, "Unsupported firmware format!\n");
			rc = -EINVAL;
		} while (0);

		free(priv->fwbuf);
		priv->fwbuf = NULL;
		priv->pfwbuf = NULL;
	}

	return rc;
}

static int netx4000_xc_ioctl(struct firmware_handler *fh, int cmd, void *ptr)
{
	struct priv_data *priv = container_of(fh, struct priv_data, fh);
	struct device_d *dev = priv->dev;
	int rc = 0;

	dev_dbg(dev, "%s: +++ (*fh=%p, cmd=%d, *ptr=%p\n", __func__, fh, cmd, ptr);

	switch (cmd) {
		case FW_REQUEST:
			rc = netx4000_xc_ioctl_fw_request(fh, ptr);
			break;
// 		case FW_RELEASE:
// 			rc = netx4000_xc_ioctl_fw_release(fh, ptr);
// 			break;
		case FW_UPLOAD:
			rc = netx4000_xc_ioctl_fw_upload(fh, ptr);
			break;
		case FW_RESET:
			rc = netx4000_xc_ioctl_fw_reset(fh, ptr);
			break;
		case FW_START:
			rc = netx4000_xc_ioctl_fw_start(fh, ptr);
			break;
		case FW_STOP:
			rc = netx4000_xc_ioctl_fw_stop(fh, ptr);
			break;
		default:
			dev_err(dev, "Unsupported firmware ioctl %d!\n", cmd);
			rc = -ENOTSUPP;
			break;
	}

	return rc;
}

/* Driver initializtion */

static int netx4000_xc_probe(struct device_d *dev)
{
	struct priv_data *priv;
	struct firmware_handler *fh;
	const char *alias = of_alias_get(dev->device_node);
	const char *model = NULL;
	int rc = 0;

	priv = xzalloc(sizeof(*priv));
	if (!priv) {
		dev_err(dev, "%s: xzalloc() failed!\n", __func__);
		return -ENOMEM;
	}

	priv->dev = dev;

	fh = &priv->fh;

	if (alias)
		fh->id = xstrdup(alias);
	else
		fh->id = xstrdup(DRIVER_NAME);

	fh->dev = dev;
	fh->open = netx4000_xc_open;
	fh->write = netx4000_xc_write;
	fh->close = netx4000_xc_close;
	fh->ioctl = netx4000_xc_ioctl;

	of_property_read_string(dev->device_node, "compatible", &model);
	if (model)
		fh->model = xstrdup(model);

	/* Enable global XC clock */
	clock_enable(XC_GLOBAL_CLOCK_ENABLE);

	rc = firmwaremgr_register(fh);
	if (rc) {
		free(priv);
		return rc;
	}
	dev_info(dev, "successfully initialized!\n");

	return 0;
}

static struct of_device_id netx4000_xc_id_table[] = {
	{ .compatible = "hilscher,netx4000-xc", .data = NULL },
	{ }
};

static struct driver_d netx4000_xc_driver = {
	.name = DRIVER_NAME,
	.probe = netx4000_xc_probe,
	.of_compatible = DRV_OF_COMPAT(netx4000_xc_id_table),
};

static int __init netx4000_xc_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return platform_driver_register(&netx4000_xc_driver);
}
device_initcall(netx4000_xc_init);

/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
