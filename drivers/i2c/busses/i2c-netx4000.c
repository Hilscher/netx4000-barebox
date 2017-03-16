/*
* I2C bus driver for Hilscher netx4000 based platforms
*
* drivers/i2c/busses/i2c-netx4000.c
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

#define DRIVER_DESC  "I2C bus driver for Hilscher netx4000 based platforms"
#define DRIVER_NAME "i2c-netx4000"

/* --- Includes --- */

#include <init.h>
#include <i2c/i2c.h>
#include <asm/mmu.h>
#include <io.h>
#include <clock.h>

#include <of_address.h>

/* --- Chip definitions --- */

/* Register offsets */
#define I2C_MCR      0x00 /* I2C master control register */
#define I2C_SCR      0x04 /* I2C slave control register */
#define I2C_CMD      0x08 /* I2C master command register */
#define I2C_MDR      0x0c /* I2C master data register (master FIFO) */
#define I2C_SDR      0x10 /* I2C slave data register (slave FIFO) */
#define I2C_MFIFO_CR 0x14 /* I2C master FIFO control register */
#define I2C_SFIFO_CR 0x18 /* I2C slave FIFO control register */
#define I2C_SR       0x1c /* I2C status register */
#define I2C_IRQMSK   0x20 /* I2C interrupt mask set or clear register */
#define I2C_IRQSR    0x24 /* I2C interrupt state register (raw interrupt before masking) */
#define I2C_IRQMSKED 0x28 /* I2C masked interrupt state register */
#define I2C_DMACR    0x2c /* I2C DMA control register */
#define I2C_PIO      0x30 /* Direct I2C IO controlling */

/* I2C master control register (i2c_mcr) */
#define MCR_EN_TIMEOUT (1 << 18)
#define MCR_RST_I2C    (1 << 17)
#define MCR_PIO_MODE   (1 << 16)
#define MCR_SADR_SHIFT 4
#define MCR_SADR_MASK  (0x7f << MCR_SADR_SHIFT)
#define MCR_MODE_SHIFT 1
#define MCR_MODE_MASK  (0x7 << MCR_MODE_SHIFT)
#define MCR_MODE_50K   (0b000 << MCR_MODE_SHIFT) /* Fast/Standard, 50kbit/s */
#define MCR_MODE_100K  (0b001 << MCR_MODE_SHIFT) /* Fast/Standard, 100kbit/s */
#define MCR_MODE_200K  (0b010 << MCR_MODE_SHIFT) /* Fast/Standard, 200kbit/s */
#define MCR_MODE_400K  (0b011 << MCR_MODE_SHIFT) /* Fast/Standard, 400kbit/s */
#define MCR_MODE_800K  (0b100 << MCR_MODE_SHIFT) /* High-speed, 800kbit/s */
#define MCR_MODE_1200K (0b101 << MCR_MODE_SHIFT) /* High-speed, 1.2Mbit/s */
#define MCR_MODE_1700K (0b110 << MCR_MODE_SHIFT) /* High-speed, 1.7Mbit/s */
#define MCR_MODE_3400K (0b111 << MCR_MODE_SHIFT) /* High-speed, 3.4Mbit/s) */
#define MCR_EN_I2C     (1 << 0)

/* I2C slave control register (i2c_scr) */
#define SCR_AUTORESET_AC_START (1 << 20)
#define SCR_AC_GCALL           (1 << 18)
#define SCR_AC_START           (1 << 17)
#define SCR_AC_SRX             (1 << 16)
#define SCR_SID10              (1 << 10)
#define SCR_SID_SHIFT          0
#define SCR_SID_MASK           (0x3ff << SCR_SID_SHIFT)

/* I2C master command register (i2c_cmd) */
#define CMD_ACPOLLMAX_SHIFT 20
#define CMD_ACPOLLMAX_MASK (0xff << CMD_ACPOLLMAX_SHIFT)
#define CMD_TSIZE_SHIFT    8
#define CMD_TSIZE_MASK     (0x3ff << CMD_TSIZE_SHIFT)
#define CMD_CMD_SHIFT      1
#define CMD_CMD_MASK       (0x7  << CMD_CMD_SHIFT)
#define CMD_CMD_START      (0b000 << CMD_CMD_SHIFT) /* (r)START-condition */
#define CMD_CMD_S_AC       (0b001 << CMD_CMD_SHIFT) /* + Acknowledge-polling */
#define CMD_CMD_S_AC_T     (0b010 << CMD_CMD_SHIFT) /* + data */
#define CMD_CMD_S_AC_TC    (0b011 << CMD_CMD_SHIFT) /* + data ... */
#define CMD_CMD_CT         (0b100 << CMD_CMD_SHIFT) /* ... data */
#define CMD_CMD_CTC        (0b101 << CMD_CMD_SHIFT) /* ... data ... */
#define CMD_CMD_STOP       (0b110 << CMD_CMD_SHIFT) /* STOP-condition */
#define CMD_CMD_IDLE       (0b111 << CMD_CMD_SHIFT) /* Nothing to do, last cmd finished, break current cmd */
#define CMD_NWR            (1 << 0)

/* I2C master data register (master FIFO) (i2c_mdr) */
#define MDR_MDATA_SHIFT 0
#define MDR_MDATA_MASK  (0xff << MDR_MDATA_SHIFT)

/* I2C slave data register (slave FIFO) (i2c_sdr) */
#define SDR_SDATA_SHIFT 0
#define SDR_SDATA_MASK  (0xff << SDR_SDATA_SHIFT)

/* I2C master FIFO control register (i2c_mfifo_cr) */
#define MFIFO_CR_MFIFO_CLR      (1 << 8)
#define MFIFO_CR_MFIFO_WM_SHIFT 0
#define MFIFO_CR_MFIFO_WM_MASK  (0xf << MFIFO_CR_MFIFO_WM_SHIFT)

/* I2C slave FIFO control register (i2c_sfifo_cr) */
#define SFIFO_CR_SFIFO_CLR      (1 << 8)
#define SFIFO_CR_SFIFO_WM_SHIFT 0
#define SFIFO_CR_SFIFO_WM_MASK  (0xf << SFIFO_CR_SFIFO_WM_SHIFT)

/* I2C status register (i2c_sr) */
#define SR_SDA_STATE         (1 << 31)
#define SR_SCL_STATE         (1 << 30)
#define SR_TIMEOUT           (1 << 28)
#define SR_SID10_ACED        (1 << 27)
#define SR_GCALL_ACED        (1 << 26)
#define SR_NWR_ACED          (1 << 25)
#define SR_LAST_AC           (1 << 24)
#define SR_SLAVE_ACCESS      (1 << 23)
#define SR_STARTED           (1 << 22)
#define SR_NWR               (1 << 21)
#define SR_BUS_MASTER        (1 << 20)
#define SR_SFIFO_ERR_UNDR    (1 << 19)
#define SR_SFIFO_ERR_OVFL    (1 << 18)
#define SR_SFIFO_FULL        (1 << 17)
#define SR_SFIFO_EMPTY       (1 << 16)
#define SR_SFIFO_LEVEL_SHIFT 10
#define SR_SFIFO_LEVEL_MASK  (0x1f << SR_SFIFO_LEVEL_SHIFT)
#define SR_MFIFO_ERR_UNDR    (1 << 9)
#define SR_MFIFO_ERR_OVFL    (1 << 8)
#define SR_MFIFO_FULL        (1 << 7)
#define SR_MFIFO_EMPTY       (1 << 6)
#define SR_MFIFO_LEVEL_SHIFT 0
#define SR_MFIFO_LEVEL_MASK  (0x1f << SR_MFIFO_LEVEL_SHIFT)

/* I2C Common IRQ defines */
#define SREQ     (1 << 6)
#define SFIFO    (1 << 5)
#define MFIFO    (1 << 4)
#define BUS_BUSY (1 << 3)
#define FIFO_ERR (1 << 2)
#define CMD_ERR  (1 << 1)
#define CMD_OK   (1 << 0)

/* I2C interrupt mask set or clear register (i2c_irqmsk) */
#define IRQMSK_SREQ      SREQ
#define IRQMSK_SFIFO_REQ SFIFO
#define IRQMSK_MFIFO_REQ MFIFO
#define IRQMSK_BUS_BUSY  BUS_BUSY
#define IRQMSK_FIFO_ERR  FIFO_ERR
#define IRQMSK_CMD_ERR   CMD_ERR
#define IRQMSK_CMD_OK    CMD_OK
#define IRQMSK_SHIFT     0
#define IRQMSK_MASK      (0x3f << IRQMSK_SHIFT)

/* I2C interrupt state register (raw interrupt before masking) (i2c_irqsr) */
#define IRQSR_SREQ      SREQ
#define IRQSR_SFIFO_REQ SFIFO
#define IRQSR_MFIFO_REQ MFIFO
#define IRQSR_BUS_BUSY  BUS_BUSY
#define IRQSR_FIFO_ERR  FIFO_ERR
#define IRQSR_CMD_ERR   CMD_ERR
#define IRQSR_CMD_OK    CMD_OK
#define IRQSR_SHIFT     0
#define IRQSR_MASK      (0x3f << IRQMSK_SHIFT)

/* I2C masked interrupt state register (i2c_irqmsked) */
#define IRQMSKED_SREQ      SREQ
#define IRQMSKED_SFIFO_REQ SFIFO
#define IRQMSKED_MFIFO_REQ MFIFO
#define IRQMSKED_BUS_BUSY  BUS_BUSY
#define IRQMSKED_FIFO_ERR  FIFO_ERR
#define IRQMSKED_CMD_ERR   CMD_ERR
#define IRQMSKED_CMD_OK    CMD_OK
#define IRQMSKED_SHIFT     0
#define IRQMSKED_MASK      (0x3f << IRQMSK_SHIFT)

/* I2C DMA control register (i2c_dmacr) */
#define DMACR_SDMAB_EN (1 << 3)
#define DMACR_SDMAS_EN (1 << 2)
#define DMACR_MDMAB_EN (1 << 1)
#define DMACR_MDMAS_EN (1 << 0)

/* Direct I2C IO controlling (i2c_pio) */
#define PIO_SDA_IN_RO (1 << 6)
#define PIO_SDA_OE    (1 << 5)
#define PIO_SDA_OUT   (1 << 4)
#define PIO_SCL_IN_RO (1 << 2)
#define PIO_SCL_OE    (1 << 1)
#define PIO_SCL_OUT   (1 << 0)

/* --- Global definitions --- */

#define I2C_TIMEOUT (1000 * MSECOND)

struct netx4000_i2c_pdata {
	void  __iomem *base;
	uint32_t bus_clock; /* Hz */
	uint32_t speed_mode; /* chip mode */
	uint32_t acpollmax;
	struct device_d *dev;
	struct i2c_adapter adapter;
};

/* --- Macros --- */

#define IOSET32(mask, addr) { uint32_t val; val = readl(addr); writel(val | mask, addr); }
#define IOCLEAR32(mask, addr) { uint32_t val; val = readl(addr); writel(val & ~mask, addr); }

/* --- Source code --- */

static int32_t netx4000_i2c_wait_for_idle(struct netx4000_i2c_pdata *pdata)
{
	void __iomem *base = pdata->base;
	uint32_t irqsr;
	int32_t rc = 0;

	dev_dbg(pdata->dev, "%s() called\n", __func__);

	/* Check for IDLE command */
	while ((readl(base + I2C_CMD) & CMD_CMD_MASK) != CMD_CMD_IDLE) {
		if (readl(base + I2C_SR) & SR_TIMEOUT) {
			/* Clear the timeout flag */
			IOSET32(SR_TIMEOUT, base + I2C_SR);
			break;
		}
	}

	/* Handle raw interrupts */
	irqsr = readl(base + I2C_IRQSR);
	writel(irqsr, base + I2C_IRQSR);

	if (irqsr & BUS_BUSY) {
		rc = -EBUSY;
	} else if (irqsr & CMD_ERR) {
		rc = -EIO;
	}

	if (rc)
		return rc; /* Error */

	return 0; /* Success */
}

static inline int32_t netx4000_i2c_send(struct netx4000_i2c_pdata *pdata, struct i2c_msg *msg, uint32_t nwr, uint32_t stop)
{
	void __iomem *base = pdata->base;
	uint32_t nbytes = msg->len, cmd, copy;
	uint64_t ts;
	int32_t rc = 0;

	dev_dbg(pdata->dev, "%s() called - addr: 0x%04x, len: %d, nwr: %d, stop: %d, flags: 0x%x\n"
			, __func__, msg->addr, msg->len, nwr, stop, msg->flags);

	/* Copy the message buffer to fifo until it is full */
	while (nbytes) {
		if (readl(base + I2C_SR) & SR_MFIFO_FULL)
			break;
		writel((uint32_t)(*msg->buf++), base + I2C_MDR);
		nbytes--;
	}

	/* Prepare and send the command register */
	cmd = ((msg->len - 1) << CMD_TSIZE_SHIFT) | ((stop) ? CMD_CMD_CT : CMD_CMD_CTC) | nwr;
	writel(cmd, base + I2C_CMD);

	/* If there are more data, copy these also to the fifo */
	while (nbytes) {
		/* We like to process a data size of 75% of fifo size */
		copy = min(nbytes, (uint32_t)(16 * 3/4));

		ts = get_time_ns();
		while (((readl(base + I2C_SR) & SR_MFIFO_LEVEL_MASK) >> SR_MFIFO_LEVEL_SHIFT) > (16 - copy)) {
			if (is_timeout(ts, I2C_TIMEOUT)) {
				dev_err(pdata->dev, "%s() - FIFO timed out\n", __func__);
				rc = -ETIMEDOUT;
				break;
			}
		}

		if (rc)
			break;

		while (copy--) {
			writel((uint32_t)*msg->buf++, base + I2C_MDR);
			nbytes--;
		}
	}

	if (rc)
		return rc; /* Error */

	return netx4000_i2c_wait_for_idle(pdata);
}

static inline int32_t netx4000_i2c_recv(struct netx4000_i2c_pdata *pdata, struct i2c_msg *msg, uint32_t nwr, uint32_t stop)
{
	void __iomem *base = pdata->base;
	uint32_t nbytes = msg->len, cmd, copy;
	uint64_t ts;
	int32_t rc = 0;

	dev_dbg(pdata->dev, "%s() called - addr: 0x%04x, len: %d, nwr: %d, stop: %d, flags: 0x%x\n"
			, __func__, msg->addr, msg->len, nwr, stop, msg->flags);

	/* Prepare and send the command register */
	cmd = ((msg->len - 1) << CMD_TSIZE_SHIFT) | ((stop) ? CMD_CMD_CT : CMD_CMD_CTC) | nwr;
	writel(cmd, base + I2C_CMD);

	/* Copy the data from fifo to message buffer */
	while (nbytes) {
		/* We like to process a data size of 75% of fifo size */
		copy = min(nbytes, (uint32_t)(16 * 3/4));

		ts = get_time_ns();
		while (((readl(base + I2C_SR) & SR_MFIFO_LEVEL_MASK) >> SR_MFIFO_LEVEL_SHIFT) < copy) {
			if (is_timeout(ts, I2C_TIMEOUT)) {
				dev_err(pdata->dev, "%s() - FIFO timed out\n", __func__);
				rc = -ETIMEDOUT;
				break;
			}
		}

		if (rc)
			break;

		while (copy--) {
			*msg->buf++ = (uint8_t)(readl(base + I2C_MDR) & MDR_MDATA_MASK);
			nbytes--;
		}
	}

	if (rc)
		return rc; /* Error */

	return netx4000_i2c_wait_for_idle(pdata);
}

static int32_t netx4000_i2c_master_xfer(struct i2c_adapter *adapter, struct i2c_msg msgs[], int32_t num)
{
	struct netx4000_i2c_pdata *pdata = container_of(adapter, struct netx4000_i2c_pdata, adapter);
	void __iomem *base = pdata->base;
	uint32_t i, nwr, start, stop;
	int32_t rc = 0;

	dev_dbg(pdata->dev, "%s() called\n", __func__);

	/* Clear master FIFO */
	writel(MFIFO_CR_MFIFO_CLR, base + I2C_MFIFO_CR);

	/* Clear any pending interrupts */
	writel(MFIFO_CR_MFIFO_CLR, base + I2C_IRQSR);

	/* Enable command timeout detection, set the slave address, the mode (speed) and enable I2C master */
	writel(MCR_EN_TIMEOUT | ((msgs[0].addr << MCR_SADR_SHIFT) & MCR_SADR_MASK) | pdata->speed_mode | MCR_EN_I2C, base + I2C_MCR);

	/* Check for bus arbitration */
	rc = netx4000_i2c_wait_for_idle(pdata);

	/* Handle all given i2c messages */
	for (i = 0; (i < num) && (rc == 0); i++) {
		/* Check error criteria */
		if (msgs[i].len > 1024)
			rc = -EINVAL;
		if (msgs[i].flags & ~(I2C_M_RD | I2C_M_DATA_ONLY | I2C_M_STOP)) /* These are supported flags */
			rc = -EOPNOTSUPP;
		if (rc)
			break;

		/* Handle the flags */
		nwr = (msgs[i].flags & I2C_M_RD) ? 1 : 0;
		start = (msgs[i].flags & I2C_M_DATA_ONLY) ? 0 : 1;
		stop = ((msgs[i].flags & I2C_M_STOP) || (i == (num - 1))) ? 1 : 0;

		/* Handle the start condition */
		if (start) {
			writel((pdata->acpollmax << CMD_ACPOLLMAX_SHIFT) | CMD_CMD_S_AC | nwr, base + I2C_CMD);
			rc = netx4000_i2c_wait_for_idle(pdata);
			if (rc)
				break;
		}

		/* Handle the data transfer (read/write) */
		if (msgs[i].len > 0) {
			if (nwr)
				rc = netx4000_i2c_recv(pdata, &msgs[i], nwr, stop);
			else
				rc = netx4000_i2c_send(pdata, &msgs[i], nwr, stop);
			if (rc)
				break;
		}

		/* Handle the stop condition */
		if (stop) {
			writel(CMD_CMD_STOP, base + I2C_CMD);
			rc = netx4000_i2c_wait_for_idle(pdata);
			if (rc)
				break;
		}
	}

	/* Disable i2c master */
	writel(0, base + I2C_MCR);

	if (rc)
		return rc; /* Error */

	return num; /* Success */
}

static int32_t netx4000_i2c_chip_init(struct netx4000_i2c_pdata *pdata)
{
	void __iomem *base = pdata->base;

	dev_dbg(pdata->dev, "%s() called\n", __func__);

	/* Chip reset */
	writel(MCR_RST_I2C, base + I2C_MCR);

	return 0;
}

static int32_t netx4000_i2c_probe(struct device_d *dev)
{
	struct netx4000_i2c_pdata *pdata;
	uint8_t bus_clock_default = 0, bus_clock_fallback = 0;
	int32_t rc = 0;

	dev_dbg(dev, "%s() called\n", __func__);

	/* Allocate memory for private data */
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(dev, "xzalloc() failed\n");
		rc = -ENOMEM;
		goto err_out;
	}

	pdata->dev = dev;

	/* Read the register base address from DT and map it */
	pdata->base = of_iomap(dev->device_node, 0);
	if (pdata->base == NULL) {
		dev_err(pdata->dev, "of_iomap() failed\n");
		rc = -EIO;
		goto err_out;
	}

	/* Read, parse and handle the 'clock-frequency' from DT (optional) */
	{
		uint32_t i;
		const uint32_t clock[][2] = {
			{3400000, MCR_MODE_3400K},
			{1700000, MCR_MODE_1700K},
			{1200000, MCR_MODE_1200K},
			{800000, MCR_MODE_800K},
			{400000, MCR_MODE_400K},
			{200000, MCR_MODE_200K},
			{100000, MCR_MODE_100K},
			{50000, MCR_MODE_50K},
			{0, 0}
		};

		/* Read 'clock-frequency' from DT */
		rc = of_property_read_u32(dev->device_node, "clock-frequency", &pdata->bus_clock);
		if (rc)
			pdata->bus_clock = 0;

		/* Parse it */
		for (i = 0; pdata->bus_clock < clock[i][0]; i++);

		/* Check for fallback */
		if (pdata->bus_clock > clock[i][0])
			bus_clock_fallback = 1;

		/* Check for default bus clock */
		if (clock[i][0] == 0) {
			bus_clock_default = 1;
			i -= 2; /* 100kHz */
		}

		/* Set up the private data */
		pdata->bus_clock = clock[i][0];
		pdata->speed_mode = clock[i][1];
	}

	pdata->acpollmax = 255;

	/* Initialize the i2c chip */
	rc = netx4000_i2c_chip_init(pdata);
	if (rc) {
		dev_err(pdata->dev, "netx4000_i2c_chip_init() failed\n");
		goto err_out;
	}

	/* Set up the i2c adapter */
	pdata->adapter.master_xfer = netx4000_i2c_master_xfer,
	pdata->adapter.nr = dev->id;
	pdata->adapter.dev.parent = dev;
	pdata->adapter.dev.device_node = dev->device_node;

	/* Register the i2c adapter (chip) */
	rc = i2c_add_numbered_adapter(&pdata->adapter);
	if (rc) {
		dev_err(pdata->dev, "i2c_add_numbered_adapter() failed\n");
		goto err_out;
	}

	if (bus_clock_fallback | bus_clock_default)
		dev_warn(pdata->dev, "i2c%d invalid or missing clock-frequency in device tree => %s\n"
			, pdata->adapter.nr, (bus_clock_default) ? "default frequency" : "fallback");

	dev_info(pdata->dev, "i2c%d (%dkHz) successfully initialized!\n", pdata->adapter.nr, pdata->bus_clock / 1000);

	return 0;

err_out:
	if (pdata != NULL)
		free(pdata);

	return rc;
}

static __maybe_unused struct of_device_id netx4000_i2c_dt_ids[] = {
	{ .compatible = "hilscher,netx4000-i2c", },
	{ /* sentinel */ },
};

static struct driver_d netx4000_i2c_driver = {
	.name = DRIVER_NAME,
	.probe = netx4000_i2c_probe,
	.of_compatible = DRV_OF_COMPAT(netx4000_i2c_dt_ids),
};

static int __init netx4000_i2c_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return platform_driver_register(&netx4000_i2c_driver);
}
device_initcall(netx4000_i2c_init);

/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
