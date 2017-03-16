/*
* SPI (PL022) driver
*
* drivers/spi/spi-pl022.c
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

#define DRIVER_DESC  "SPI (PL022) driver"
#define DRIVER_NAME "spi-pl022"

#include <init.h>
#include <common.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <asm-generic/gpio.h>
#include <spi/spi.h>
#include <malloc.h>
#include <of_gpio.h>
#include <linux/amba/bus.h>

#define SSP_WRITE_BITS(reg, val, mask, sb) \
	((reg) = (((reg) & ~(mask)) | (((val)<<(sb)) & (mask))))

#define DRIVE_TX		0
#define DO_NOT_DRIVE_TX		1

#define SSP_CR0(r)	(r + 0x000)
#define SSP_CR1(r)	(r + 0x004)
#define SSP_DR(r)	(r + 0x008)
#define SSP_SR(r)	(r + 0x00C)
#define SSP_CPSR(r)	(r + 0x010)
#define SSP_IMSC(r)	(r + 0x014)
#define SSP_RIS(r)	(r + 0x018)
#define SSP_MIS(r)	(r + 0x01C)
#define SSP_ICR(r)	(r + 0x020)
#define SSP_DMACR(r)	(r + 0x024)
#define SSP_ITCR(r)	(r + 0x080)
#define SSP_ITIP(r)	(r + 0x084)
#define SSP_ITOP(r)	(r + 0x088)
#define SSP_TDR(r)	(r + 0x08C)

#define SSP_PID0(r)	(r + 0xFE0)
#define SSP_PID1(r)	(r + 0xFE4)
#define SSP_PID2(r)	(r + 0xFE8)
#define SSP_PID3(r)	(r + 0xFEC)

#define SSP_CID0(r)	(r + 0xFF0)
#define SSP_CID1(r)	(r + 0xFF4)
#define SSP_CID2(r)	(r + 0xFF8)
#define SSP_CID3(r)	(r + 0xFFC)

/*
 * SSP Control Register 0  - SSP_CR0
 */
#define SSP_CR0_MASK_DSS	(0x0FUL << 0)
#define SSP_CR0_MASK_FRF	(0x3UL << 4)
#define SSP_CR0_MASK_SPO	(0x1UL << 6)
#define SSP_CR0_MASK_SPH	(0x1UL << 7)
#define SSP_CR0_MASK_SCR	(0xFFUL << 8)

/*
 * SSP Control Register 0  - SSP_CR1
 */
#define SSP_CR1_MASK_LBM	(0x1UL << 0)
#define SSP_CR1_MASK_SSE	(0x1UL << 1)
#define SSP_CR1_MASK_MS		(0x1UL << 2)
#define SSP_CR1_MASK_SOD	(0x1UL << 3)

/*
 * SSP Status Register - SSP_SR
 */
#define SSP_SR_MASK_TFE		(0x1UL << 0) /* Transmit FIFO empty */
#define SSP_SR_MASK_TNF		(0x1UL << 1) /* Transmit FIFO not full */
#define SSP_SR_MASK_RNE		(0x1UL << 2) /* Receive FIFO not empty */
#define SSP_SR_MASK_RFF		(0x1UL << 3) /* Receive FIFO full */
#define SSP_SR_MASK_BSY		(0x1UL << 4) /* Busy Flag */

/*
 * SSP Clock Prescale Register  - SSP_CPSR
 */
#define SSP_CPSR_MASK_CPSDVSR	(0xFFUL << 0)

/*
 * SSP DMA Control Register - SSP_DMACR
 */
/* Receive DMA Enable bit */
#define SSP_DMACR_MASK_RXDMAE		(0x1UL << 0)
/* Transmit DMA Enable bit */
#define SSP_DMACR_MASK_TXDMAE		(0x1UL << 1)

/*
 * SSP State - Whether Enabled or Disabled
 */
#define SSP_DISABLED			(0)
#define SSP_ENABLED			(1)

/*
 * SSP DMA State - Whether DMA Enabled or Disabled
 */
#define SSP_DMA_DISABLED		(0)
#define SSP_DMA_ENABLED			(1)

/*
 * SSP Clock Parameter ranges
 */
#define CPSDVR_MIN 0x02
#define CPSDVR_MAX 0xFE
#define SCR_MIN 0x00
#define SCR_MAX 0xFF

/*
 * SSP Interrupt related Macros
 */
#define DEFAULT_SSP_REG_IMSC  0x0UL
#define DISABLE_ALL_INTERRUPTS DEFAULT_SSP_REG_IMSC
#define ENABLE_ALL_INTERRUPTS (~DEFAULT_SSP_REG_IMSC)

#define CLEAR_ALL_INTERRUPTS  0x3

#define SPI_POLLING_TIMEOUT 1000


/*
 * The type of reading going on on this chip
 */
enum ssp_reading {
	READING_NULL,
	READING_U8,
	READING_U16,
	READING_U32
};

/**
 * The type of writing going on on this chip
 */
enum ssp_writing {
	WRITING_NULL,
	WRITING_U8,
	WRITING_U16,
	WRITING_U32
};

/**
 * enum ssp_clock_params - clock parameters, to set SSP clock at a
 * desired freq
 */
struct ssp_clock_params {
	u8 cpsdvsr; /* value from 2 to 254 (even only!) */
	u8 scr;	    /* value from 0 to 255 */
};

/**
 * enum ssp_hierarchy - whether SSP is configured as Master or Slave
 */
enum ssp_hierarchy {
	SSP_MASTER,
	SSP_SLAVE
};

/**
 * whether SSP is in loopback mode or not
 */
enum ssp_loopback {
	LOOPBACK_DISABLED,
	LOOPBACK_ENABLED
};

/**
 * enum SPI Clock Phase - clock phase (Motorola SPI interface only)
 * @SSP_CLK_FIRST_EDGE: Receive data on first edge transition
 * (actual direction depends on polarity)
 * @SSP_CLK_SECOND_EDGE: Receive data on second edge transition
 * (actual direction depends on polarity)
 */
enum ssp_spi_clk_phase {
	SSP_CLK_FIRST_EDGE,
	SSP_CLK_SECOND_EDGE
};

/**
 * enum SPI Clock Polarity - clock polarity (Motorola SPI interface only)
 * @SSP_CLK_POL_IDLE_LOW: Low inactive level
 * @SSP_CLK_POL_IDLE_HIGH: High inactive level
 */
enum ssp_spi_clk_pol {
	SSP_CLK_POL_IDLE_LOW,
	SSP_CLK_POL_IDLE_HIGH
};

/**
 * enum ssp_interface - interfaces allowed for this SSP Controller
 * @SSP_INTERFACE_MOTOROLA_SPI: Motorola Interface
 * @SSP_INTERFACE_TI_SYNC_SERIAL: Texas Instrument Synchronous Serial
 * interface
 * @SSP_INTERFACE_NATIONAL_MICROWIRE: National Semiconductor Microwire
 * interface
 * @SSP_INTERFACE_UNIDIRECTIONAL: Unidirectional interface (STn8810
 * &STn8815 only)
 */
enum ssp_interface {
	SSP_INTERFACE_MOTOROLA_SPI,
	SSP_INTERFACE_TI_SYNC_SERIAL,
	SSP_INTERFACE_NATIONAL_MICROWIRE,
	SSP_INTERFACE_UNIDIRECTIONAL
};


struct pl022 {
	struct device_d			*dev;
	void __iomem			*virtbase;
	struct clk			*clk;
	/* Two optional pin states - default & sleep */
	struct pinctrl			*pinctrl;
	struct pinctrl_state		*pins_default;
	struct pinctrl_state		*pins_idle;
	struct pinctrl_state		*pins_sleep;
	struct spi_master		*master;

	struct spi_transfer		*cur_transfer;
	struct chip_data		*cur_chip;
	void				*tx;
	void				*tx_end;
	void				*rx;
	void				*rx_end;
	enum ssp_reading		read;
	enum ssp_writing		write;
	u32				exp_fifo_level;
	int cur_cs;
	int *cs_array;
	int fifodepth;
};

struct chip_data {
	u32 cr0;
	u16 cr1;
	u16 cpsr;
	int dmacr;
	u8 n_bytes;
	enum ssp_reading read;
	enum ssp_writing write;
	void (*chipselect) (struct spi_device *spi, int is_active);
};

static inline u32 spi_rate(u32 rate, u16 cpsdvsr, u16 scr)
{
	return rate / (cpsdvsr * (1 + scr));
}

static int calculate_effective_freq(struct pl022 *pl022, int freq, struct
				    ssp_clock_params * clk_freq)
{
	/* Lets calculate the frequency parameters */
	u16 cpsdvsr = CPSDVR_MIN, scr = SCR_MIN;
	u32 rate, max_tclk, min_tclk, best_freq = 0, best_cpsdvsr = 0,
		best_scr = 0, tmp, found = 0;

	rate = clk_get_rate(pl022->clk);
	/* cpsdvscr = 2 & scr 0 */
	max_tclk = spi_rate(rate, CPSDVR_MIN, SCR_MIN);
	/* cpsdvsr = 254 & scr = 255 */
	min_tclk = spi_rate(rate, CPSDVR_MAX, SCR_MAX);

	if (freq > max_tclk)
		dev_warn(pl022->dev,
			"Max speed that can be programmed is %d Hz, you requested %d\n",
			max_tclk, freq);

	if (freq < min_tclk) {
		dev_err(pl022->dev,
			"Requested frequency: %d Hz is less than minimum possible %d Hz\n",
			freq, min_tclk);
		return -EINVAL;
	}

	/*
	 * best_freq will give closest possible available rate (<= requested
	 * freq) for all values of scr & cpsdvsr.
	 */
	while ((cpsdvsr <= CPSDVR_MAX) && !found) {
		while (scr <= SCR_MAX) {
			tmp = spi_rate(rate, cpsdvsr, scr);

			if (tmp > freq) {
				/* we need lower freq */
				scr++;
				continue;
			}

			/*
			 * If found exact value, mark found and break.
			 * If found more closer value, update and break.
			 */
			if (tmp > best_freq) {
				best_freq = tmp;
				best_cpsdvsr = cpsdvsr;
				best_scr = scr;

				if (tmp == freq)
					found = 1;
			}
			/*
			 * increased scr will give lower rates, which are not
			 * required
			 */
			break;
		}
		cpsdvsr += 2;
		scr = SCR_MIN;
	}
	clk_freq->cpsdvsr = (u8) (best_cpsdvsr & 0xFF);
	clk_freq->scr = (u8) (best_scr & 0xFF);
	return 0;
}

/**
 * cs_control - switch chip select for given spi slave
 * @spi:       pointer to a spi slave
 * @is_active: enable/disable chipselect of spi slave
 */
static void cs_control(struct spi_device *spi, int is_active)
{
	unsigned int cs = 0;
	struct pl022 *pl022 = spi->master->dev->priv;
	int gpio = pl022->cs_array[spi->chip_select];

	if (spi->mode & SPI_CS_HIGH)
		cs = 1;

	if (!is_active) {
		if (gpio >= 0)
			gpio_set_value(gpio, !cs);
		return;
	}
	if (gpio >= 0)
		gpio_set_value(gpio, cs);
}

/**
 * flush - flush the FIFO to reach a clean state
 * @pl022: SSP driver private data structure
 */
static int flush(struct pl022 *pl022)
{
	unsigned long limit = (1<<12) << 1;
	do {
		while (readw(SSP_SR(pl022->virtbase)) & SSP_SR_MASK_RNE)
			readw(SSP_DR(pl022->virtbase));
	} while ((readw(SSP_SR(pl022->virtbase)) & SSP_SR_MASK_BSY) && limit--);
	pl022->exp_fifo_level = 0;

	return limit;
}

/**
 * restore_state - Load configuration of current chip
 * @pl022: SSP driver private data structure
 */
static void restore_state(struct pl022 *pl022)
{
	struct chip_data *chip = pl022->cur_chip;

	writew(chip->cr0, SSP_CR0(pl022->virtbase));
	writew(chip->cr1, SSP_CR1(pl022->virtbase));
	writew(chip->dmacr, SSP_DMACR(pl022->virtbase));
	writew(chip->cpsr, SSP_CPSR(pl022->virtbase));
	writew(DISABLE_ALL_INTERRUPTS, SSP_IMSC(pl022->virtbase));
	writew(CLEAR_ALL_INTERRUPTS, SSP_ICR(pl022->virtbase));
}

/**
 * This will write to TX and read from RX according to the parameters
 * set in pl022.
 */
static void readwriter(struct pl022 *pl022)
{
	/* Read as much as you can */
	while ((readw(SSP_SR(pl022->virtbase)) & SSP_SR_MASK_RNE)
	       && (pl022->rx < pl022->rx_end)) {
		switch (pl022->read) {
		case READING_NULL:
			readw(SSP_DR(pl022->virtbase));
			break;
		case READING_U8:
			*(u8 *) (pl022->rx) =
				readw(SSP_DR(pl022->virtbase)) & 0xFFU;
			break;
		case READING_U16:
			*(u16 *) (pl022->rx) =
				(u16) readw(SSP_DR(pl022->virtbase));
			break;
		case READING_U32:
			*(u32 *) (pl022->rx) =
				readl(SSP_DR(pl022->virtbase));
			break;
		}
		pl022->rx += (pl022->cur_chip->n_bytes);
		pl022->exp_fifo_level--;
	}
	/*
	 * Write as much as possible up to the RX FIFO size
	 */
	while ((pl022->exp_fifo_level < pl022->fifodepth)
	       && (pl022->tx < pl022->tx_end)) {
		switch (pl022->write) {
		case WRITING_NULL:
			writew(0x0, SSP_DR(pl022->virtbase));
			break;
		case WRITING_U8:
			writew(*(u8 *) (pl022->tx), SSP_DR(pl022->virtbase));
			break;
		case WRITING_U16:
			writew((*(u16 *) (pl022->tx)), SSP_DR(pl022->virtbase));
			break;
		case WRITING_U32:
			writel(*(u32 *) (pl022->tx), SSP_DR(pl022->virtbase));
			break;
		}
		pl022->tx += (pl022->cur_chip->n_bytes);
		pl022->exp_fifo_level++;
		/*
		 * This inner reader takes care of things appearing in the RX
		 * FIFO as we're transmitting. This will happen a lot since the
		 * clock starts running when you put things into the TX FIFO,
		 * and then things are continuously clocked into the RX FIFO.
		 */
		while ((readw(SSP_SR(pl022->virtbase)) & SSP_SR_MASK_RNE)
		       && (pl022->rx < pl022->rx_end)) {
			switch (pl022->read) {
			case READING_NULL:
				readw(SSP_DR(pl022->virtbase));
				break;
			case READING_U8:
				*(u8 *) (pl022->rx) =
					readw(SSP_DR(pl022->virtbase)) & 0xFFU;
				break;
			case READING_U16:
				*(u16 *) (pl022->rx) =
					(u16) readw(SSP_DR(pl022->virtbase));
				break;
			case READING_U32:
				*(u32 *) (pl022->rx) =
					readl(SSP_DR(pl022->virtbase));
				break;
			}
			pl022->rx += (pl022->cur_chip->n_bytes);
			pl022->exp_fifo_level--;
		}
	}
}

/**
 * This sets up the pointers to memory for the next message to
 * send out on the SPI bus.
 */
static int set_up_next_transfer(struct pl022 *pl022,
				struct spi_transfer *transfer)
{
	int residue;

	/* Sanity check the message for this bus width */
	residue = pl022->cur_transfer->len % pl022->cur_chip->n_bytes;
	if (unlikely(residue != 0)) {
		dev_err(pl022->dev,
			"message of %u bytes to transmit but the current "
			"chip bus has a data width of %u bytes!\n",
			pl022->cur_transfer->len,
			pl022->cur_chip->n_bytes);
		dev_err(pl022->dev, "skipping this message\n");
		return -EIO;
	}
	pl022->tx = (void *)transfer->tx_buf;
	pl022->tx_end = pl022->tx + pl022->cur_transfer->len;
	pl022->rx = (void *)transfer->rx_buf;
	pl022->rx_end = pl022->rx + pl022->cur_transfer->len;
	pl022->write =
	    pl022->tx ? pl022->cur_chip->write : WRITING_NULL;
	pl022->read = pl022->rx ? pl022->cur_chip->read : READING_NULL;
	return 0;
}

/**
 * pl022_transfer - generic transfer method
 * @spi: pointer to a spi slave
 * @mesg:
 */
static int pl022_transfer(struct spi_device *spi, struct spi_message *mesg)
{
	struct pl022 *pl022 = spi->master->dev->priv;
	struct spi_transfer *t = NULL;

	flush(pl022);
	pl022->cur_chip = spi->controller_state;
	pl022->cur_chip->chipselect(spi, 1);
	restore_state(pl022);

	list_for_each_entry(t, &mesg->transfers, transfer_list) {
		set_up_next_transfer(pl022, t);
		readwriter(pl022);
	}
	pl022->cur_chip->chipselect(spi, 0);

	return 0;
}

/**
 * pl022_setup - generic setup method for spi slave
 * @spi: pointer to a spi slave
 */
static int pl022_setup(struct spi_device *spi)
{
	struct pl022 *pl022 = spi->master->dev->priv;
	struct device_node *np = spi->dev.device_node;
	struct ssp_clock_params clk_freq = { .cpsdvsr = 0, .scr = 0};
	struct chip_data *chip;
	enum ssp_interface iface;
	int status = 0;
	u32 bits;
	u32 tmp;

	if (!spi->max_speed_hz)
		return -EINVAL;

	spi->controller_state = xzalloc(sizeof(*chip));
	if (!spi->controller_state)
		return -ENOMEM;

	chip = spi->controller_state;
	chip->chipselect = cs_control;

	of_property_read_u32(np, "pl022,interface", &iface);

	if (status < 0)
		goto err;

	/* default */
	bits = spi->bits_per_word;
	/*of_property_read_u32(spi->dev.parent->device_node, "pl022,width",
				&bits);
	spi->bits_per_word = bits;*/

	/* Now set controller state based on controller data */
	/* Check bits per word with vendor specific range */
	if (bits <= 3) {
		status = -ENOTSUPP;
		dev_err(&spi->dev, "illegal data size for this controller!\n");
		goto err;
	} else if (bits <= 8) {
		dev_dbg(&spi->dev, "4 <= n <=8 bits per word\n");
		chip->n_bytes = 1;
		chip->read = READING_U8;
		chip->write = WRITING_U8;
	} else if (bits <= 16) {
		dev_dbg(&spi->dev, "9 <= n <= 16 bits per word\n");
		chip->n_bytes = 2;
		chip->read = READING_U16;
		chip->write = WRITING_U16;
	} else {
		dev_dbg(&spi->dev, "17 <= n <= 32 bits per word\n");
		chip->n_bytes = 4;
		chip->read = READING_U32;
		chip->write = WRITING_U32;
	}

	/* Now Initialize all register settings required for this chip */
	chip->cr0 = 0;
	chip->cr1 = 0;
	chip->cpsr = 0;
	chip->dmacr = 0;

	/* cr0 */
	/* frame format */
	SSP_WRITE_BITS(chip->cr0, bits - 1,
			SSP_CR0_MASK_DSS, 0);
	SSP_WRITE_BITS(chip->cr0, iface,
			SSP_CR0_MASK_FRF, 4);

	/* clock polarity */
	if (spi->mode & SPI_CPOL)
		tmp = SSP_CLK_POL_IDLE_HIGH;
	else
		tmp = SSP_CLK_POL_IDLE_LOW;
	SSP_WRITE_BITS(chip->cr0, tmp, SSP_CR0_MASK_SPO, 6);

	if (spi->mode & SPI_CPHA)
		tmp = SSP_CLK_SECOND_EDGE;
	else
		tmp = SSP_CLK_FIRST_EDGE;

	SSP_WRITE_BITS(chip->cr0, tmp, SSP_CR0_MASK_SPH, 7);

	calculate_effective_freq(pl022, spi->max_speed_hz, &clk_freq);
	SSP_WRITE_BITS(chip->cr0, clk_freq.scr, SSP_CR0_MASK_SCR, 8);

	/* cr1 */
	if (spi->mode & SPI_LOOP)
		tmp = LOOPBACK_ENABLED;
	else
		tmp = LOOPBACK_DISABLED;

	SSP_WRITE_BITS(chip->cr1, tmp, SSP_CR1_MASK_LBM, 0);
	SSP_WRITE_BITS(chip->cr1, SSP_DISABLED, SSP_CR1_MASK_SSE, 1);
	SSP_WRITE_BITS(chip->cr1, SSP_MASTER, SSP_CR1_MASK_MS, 2);
	SSP_WRITE_BITS(chip->cr1, DO_NOT_DRIVE_TX, SSP_CR1_MASK_SOD, 3);

	/* cpsr */
	chip->cpsr = clk_freq.cpsdvsr;

	/* dmacr */
	SSP_WRITE_BITS(chip->dmacr, SSP_DMA_DISABLED,
			SSP_DMACR_MASK_RXDMAE, 0);
	SSP_WRITE_BITS(chip->dmacr, SSP_DMA_DISABLED,
			SSP_DMACR_MASK_TXDMAE, 1);
	writew(chip->dmacr, SSP_DMACR(pl022->virtbase));

	return 0;

err:
	free(chip);
	return status;
}

static int pl022_spi_dt_probe(struct pl022 *pl022)
{
	struct device_node *node = pl022->master->dev->device_node;
	int ret, i;
	u32 num_cs;

	if (!node)
		return -ENODEV;

	ret = of_property_read_u32(node, "num-cs", &num_cs);
	if (ret)
		return ret;

	pl022->master->num_chipselect = num_cs;
	pl022->cs_array = xzalloc(sizeof(u32) * num_cs);

	for (i = 0; i < num_cs; i++) {
		int cs_gpio = of_get_named_gpio(node, "cs-gpios", i);
		pl022->cs_array[i] = cs_gpio;
		gpio_direction_output(cs_gpio, 1);
	}
	return 0;
}

static int pl022_probe(struct amba_device *adev, const struct amba_id *id)
{
	struct spi_master *master;
	struct pl022 *pl022 = NULL;
	struct device_d *dev = &adev->dev;
	int status = 0;

	pl022 = xzalloc(sizeof(*pl022));
	if (!pl022)
		return -ENOMEM;

	master = xzalloc(sizeof(*master));
	if (!master) {
		free(pl022);
		return -ENOMEM;
	}
	pl022->master = master;

	/* Allocate master with space for data */
	master->dev = dev;
	master->bus_num = dev->id;
	master->setup = pl022_setup;
	master->transfer = pl022_transfer;
	master->dev->priv = pl022;

	pl022_spi_dt_probe(pl022);

	pl022->virtbase = amba_get_mem_region(adev);
	if (!pl022->virtbase) {
		status = -ENOMEM;
		goto err_free;
	}

	pl022->clk = clk_get(dev, NULL);
	if (IS_ERR(pl022->clk))
		goto err_free;

	//TODO: clock handling (enable...)

	/* Disable SSP */
	writew((readw(SSP_CR1(pl022->virtbase)) & (~SSP_CR1_MASK_SSE)),
	       SSP_CR1(pl022->virtbase));

	return spi_register_master(master);

err_free:
	free(master);
	free(pl022);
	return status;
}

static struct amba_id pl022_ids[] = {
	{
		.id	= 0x00241022,
		.mask	= 0x00ffffff,
		.data	= NULL,
	},
	{
		.id	= 0x00041022,
		.mask	= 0x000fffff,
		.data	= NULL,
	},
};

/* device registered by platforms */
static struct of_device_id pl022_id_table[] = {
	{ .compatible = "arm,ssp-pl022" },
	{ }
};

struct amba_driver pl022_driver = {
	.drv = {
		.name          = DRIVER_NAME,
		.of_compatible = DRV_OF_COMPAT(pl022_id_table),
	},
	.probe		= pl022_probe,
	.id_table	= pl022_ids,
};

static int __init pl022_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return amba_driver_register(&pl022_driver);
}
device_initcall(pl022_init);

/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
