/*
* MCI (SD/MMC) driver for Hilscher netx4000 based platforms
*
* drivers/mci/mci-netx4000.c
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

// #define DEBUG 1

#define DRIVER_DESC  "SD/MMC driver for Hilscher netx4000 based platforms"
#define DRIVER_NAME "mci-netx4000"

#include <common.h>
#include <malloc.h>
#include <init.h>
#include <clock.h>
#include <asm/io.h>
#include <gpio.h>
#include <of_gpio.h>
#include <linux/clk.h>
#include "mci-netx4000.h"
#include <mci.h>
#include "sdhci.h"

/* ---- time out count ---- */
#define SD_TIMEOUT		(  100 * 1000 * 1000) /* 100ms - commnad timeout */
#define SD_TIMEOUT_CMD		(  100 * 1000 * 1000) /* 100ms - commnad timeout */
#define SD_TIMEOUT_MULTIPLE	( 1000 * 1000 * 1000) /* 1000ms - block transfer timeout */
#define SD_TIMEOUT_RESP	( 1000 * 1000 * 1000) /* command sequence timeout */
#define SD_TIMEOUT_ERASE_CMD	( 1000 * 1000 * 1000) /* erase timeout */
#define SD_TIMEOUT_PROG_CMD	( 1000 * 1000 * 1000) /* programing timeout */

/* All errors */
#define SD_INFO2_MASK_ALL_ERR		( MSK_NX4000_SDIO_SD_INFO2_ERR0 \
					| MSK_NX4000_SDIO_SD_INFO2_ERR1 \
					| MSK_NX4000_SDIO_SD_INFO2_ERR2 \
					| MSK_NX4000_SDIO_SD_INFO2_ERR3 \
					| MSK_NX4000_SDIO_SD_INFO2_ERR4 \
					| MSK_NX4000_SDIO_SD_INFO2_ERR5 \
					| MSK_NX4000_SDIO_SD_INFO2_ERR6 \
					| MSK_NX4000_SDIO_SD_INFO2_ILA )

static int s_fNextAppCmd = 0;
static int s_fAutomaticBlockCount = 1;

struct sdmmc_host {
	struct device_d				*dev;
	struct mci_host				mci;
	struct netx4000_sdio_reg __iomem	*regs;
	struct clk				*clk;
	unsigned short 				int_info1;
	unsigned short 				int_info2;
};
#define to_sdmmc_host(mci) container_of(mci, struct sdmmc_host, mci)

static int netx4000_wait_for_sclkdiven(struct sdmmc_host* host)
{
	while (!(readl(&host->regs->sd_info2) & MSK_NX4000_SDIO_SD_INFO2_SCLKDIVEN)) {
		/* FIXME: timeout handling */
	}
	return 0;
}

static u32 wait_for_irq_or_error(struct sdmmc_host *host, u32 info1_mask, u32 info2_mask, u32 error_mask, u32 time, u32 *error)
{
	u32 ret = 0;
	//u32 timeout = get_time_ns() + time;
	//TODO: response timeout -> longer than 640 cycles SDCLK
	u32 timeout = (time > 0 ) ? time : 1;
	u32 value = 0;
	do{
		/* wait for irq */
		if (info1_mask) {
			value = readl(&host->regs->sd_info1);
			if ( (value = (value & info1_mask)) ) {
				writel(~value,&host->regs->sd_info1);
				ret = value;
			}
		} else {
			value = readl(&host->regs->sd_info2);
			if ( (value = (value & info2_mask)) ) {
				writel(~value,&host->regs->sd_info2);
				ret = value;
			}
		}
		/* check error */
		value = readl(&host->regs->sd_info2);
		if ( (value = (value & error_mask)) ) {
			//TODO:debug
			writel(~value,&host->regs->sd_info2);
			ret = 0;
			if (error)
				*error = value;
			break;
		}
	//} while ((ret<0) && (get_time_ns()<timeout));
	} while ((ret==0) && (--timeout>0));

	return ret;
}

static int wait_for_response(struct sdmmc_host *host) {
	int ret = 0;
	u32 error = 0;
	if (0 == wait_for_irq_or_error( host, MSK_NX4000_SDIO_SD_INFO1_INFO0, 0, SD_INFO2_MASK_ALL_ERR, SD_TIMEOUT_RESP, &error)) {
		ret = -EIO;
		if (error & MSK_NX4000_SDIO_SD_INFO2_ERR6)
			ret = -ETIMEDOUT;
	}
	return ret;
}

static int wait_for_buffer_ready(struct sdmmc_host *host) {
	int ret = 0;
	u32 error = 0;
	if (0 == wait_for_irq_or_error( host, 0, MSK_NX4000_SDIO_SD_INFO2_BRE | MSK_NX4000_SDIO_SD_INFO2_BWE, SD_INFO2_MASK_ALL_ERR, SD_TIMEOUT, &error)) {
		ret = -EIO;
		if (error & MSK_NX4000_SDIO_SD_INFO2_ERR6)
			ret = -ETIMEDOUT;
	}
	return ret;
}

static int wait_for_access_end(struct sdmmc_host *host)
{
	int ret = 0;
	u32 error = 0;
	if (0 == wait_for_irq_or_error( host, MSK_NX4000_SDIO_SD_INFO1_INFO2, 0, SD_INFO2_MASK_ALL_ERR, SD_TIMEOUT, &error)) {
		ret = -EIO;
		if (error & MSK_NX4000_SDIO_SD_INFO2_ERR6)
			ret = -ETIMEDOUT;
	}
	return ret;
}

static int read_data( struct sdmmc_host *host, uint8_t *buff, long num)
{
	uint32_t data32 = 0;
	uint32_t count = num/sizeof(uint32_t);
	while(count>0) {
		data32 = readl(&host->regs->sd_buf0);
		*(buff++) = (uint8_t) (data32 & 0x000000FF);
		*(buff++) = (uint8_t)((data32 & 0x0000FF00)>> 8);
		*(buff++) = (uint8_t)((data32 & 0x00FF0000)>>16);
		*(buff++) = (uint8_t)((data32 & 0xFF000000)>>24);
		num -= sizeof(uint32_t);
		count--;
	}
	if(num>0) {
		int i;
		data32 = readl(&host->regs->sd_buf0);
		for(i=0;i<num;i++) {
			*(buff++) = data32 & 0xFF;
			data32    = (data32 >> (8*i));
		}
	}
	return 0;
}

static int write_data( struct sdmmc_host *host, const uint8_t *buff, long num)
{
	uint32_t data32 = 0;
	uint32_t count = num/sizeof(uint32_t);
	while(count>0) {
		data32  = (*buff++);
		data32 |= (*buff++ <<  8);
		data32 |= (*buff++ << 16);
		data32 |= (*buff++ << 24);
		writel(data32, &host->regs->sd_buf0);
		num -= sizeof(uint32_t);
		count--;
	}
	if(num>0) {
		int i;
		data32 = 0;
		for(i=0;i<num;i--) {
			data32 |= (*buff++ << (8*i));
		}
		writel( data32, &host->regs->sd_buf0);
	}
	return 0;

}

static int transfer_data( struct sdmmc_host *host, const uint8_t *buff, long cnt, long size, int dir)
{
	long i;
	int err = 0;

	for(i=cnt; i>0 ;i--){
		/* wait BWE/BRE or error */
		if (0>(err = wait_for_buffer_ready(host))) {
			break;
		}
		if (dir == MMC_DATA_READ) {
			/* read to SD_BUF */
			if(read_data(host, (uint8_t*)buff, size) != 0){
				err = -EIO;
				break;
			}
		} else {
			/* write to SD_BUF */
			if(write_data(host, buff, size) != 0){
				err = -EIO;
				break;
			}
		}
		/* update buffer */
		buff+=size;
	}
	if (err == 0)
		err = wait_for_access_end( host);

	return err;
}

static int get_resp(struct sdmmc_host *host, struct mci_cmd *cmd)
{
	struct netx4000_sdio_reg __iomem *regs = host->regs;
	/* ----- Select RESP Register Depend on the Response Type ----- */
	switch(cmd->resp_type) {
		/* No Response */
		case MMC_RSP_NONE:
			break;
		case MMC_RSP_R2:
		{/* CID (->CMD2,CMD10) / CSD (->CMD9) */
			uint8_t CRC = 0xAF;

			/* read all register and shift 8bit < */
			cmd->response[3] = readl(&regs->sd_rsp10);
			cmd->response[2] = readl(&regs->sd_rsp32);
			cmd->response[1] = readl(&regs->sd_rsp54);
			cmd->response[0] = readl(&regs->sd_rsp76);

			/* shifting and crc calc */
			cmd->response[0] = (cmd->response[0] << 8);
			cmd->response[0] |= 0xFF & (cmd->response[1] >> 24);
			cmd->response[1]  = (cmd->response[1] << 8);
			cmd->response[1] |= 0xFF & (cmd->response[2] >> 24);
			cmd->response[2]  = (cmd->response[2] << 8);
			cmd->response[2] |= 0xFF & (cmd->response[3] >> 24);
			cmd->response[3]  = (cmd->response[3] << 8);
			cmd->response[3] |= 0xFF & CRC;
		}
			break;
		/* Nomal Response (32bits Length) */
		case MMC_RSP_R1:
		case MMC_RSP_R1b: /* Nomal Response with an Optional Busy Signal */
		case MMC_RSP_R3: /* OCR Register (32bits Length) */
			/* MMC_RSP_R4 MMC_RSP_R5 MMC_RSP_R6 MMC_RSP_R7 same format */
			cmd->response[0] = readl(&regs->sd_rsp10);
			break;
		default:
			/* unknown type */
			dev_err(host->dev, "Unknown response type (%i)\n", cmd->resp_type);
			break;
	}
	return 0;
}

static int handle_stop_cmd( struct sdmmc_host *host)
{
	struct netx4000_sdio_reg __iomem *regs = host->regs;

	/* signal transfer stop */
	writel( 1, &regs->sd_stop);
	wait_for_response(host);
	netx4000_wait_for_sclkdiven(host);
	wait_for_access_end(host);
	return 0;
}

static void setup_cmd(struct sdmmc_host *host, struct mci_cmd *cmd, struct mci_data *data)
{
	struct netx4000_sdio_reg __iomem *regs = host->regs;
	u32 cmd_tmp = 0;

	cmd_tmp = (cmd->cmdidx & MSK_NX4000_SDIO_SD_CMD_CF);
	if (cmd->cmdidx == MMC_CMD_APP_CMD)
		s_fNextAppCmd = 1;

	/* handle application commands (followed of CMD55 -> s_fNextAppCmd = 1) */
	switch (cmd->cmdidx)
	{
		case 6:
		case 13:
		case 22:
		case 23:
		case 41:
		case 42:
		case 51:
			if (s_fNextAppCmd) {
				s_fNextAppCmd = 0;
				cmd_tmp |= (1<<6);
				break;
			}
		default:
		break;
	}

	/* We have to configure a extended mode as some commands cannot be used in normal mode.
	 * To make it easier we do it for all commands.
	 * A test passed successfuly for a SDHC-Card (v2.0) and a MultiMediaCard (v5.0). */
	switch(cmd->resp_type) {
		case MMC_RSP_NONE:
			cmd_tmp |= (3 << SRT_NX4000_SDIO_SD_CMD_MD_RSP); /* Extended mode/No response */
			break;
		case MMC_RSP_R1:
// 		case MMC_RSP_R5:
// 		case MMC_RSP_R6:
// 		case MMC_RSP_R7:
			cmd_tmp |= (4 << SRT_NX4000_SDIO_SD_CMD_MD_RSP); /* Extended mode/SD card R1, R5, R6, R7 response */
			break;
		case MMC_RSP_R1b:
			cmd_tmp |= (5 << SRT_NX4000_SDIO_SD_CMD_MD_RSP); /* Extended mode/SD card R1b response */
			break;
		case MMC_RSP_R2:
			cmd_tmp |= (6 << SRT_NX4000_SDIO_SD_CMD_MD_RSP); /* Extended mode/SD card R2 response */
			break;
		case MMC_RSP_R3:
// 		case MMC_RSP_R4:
			cmd_tmp |= (7 << SRT_NX4000_SDIO_SD_CMD_MD_RSP); /* Extended mode/SD card R3, R4 response */
			break;
	}

	/* build command */
	if (data) {/* additional data transmitted */
		cmd_tmp |= 1 << SRT_NX4000_SDIO_SD_CMD_MD3;

		if (MMC_DATA_READ == data->flags)
			cmd_tmp |= 1 << SRT_NX4000_SDIO_SD_CMD_MD4;

		/* disable block counting (sends stop command after all blocks are transferred) */
		writel(data->blocksize, &regs->sd_size);
		/* not required in since automatic mode is disabled */
		writel( data->blocks, &regs->sd_seccnt); /* deliver number of blocks */
		if (s_fAutomaticBlockCount)
			writel( (1<<8), &regs->sd_stop);/* enable automatic block count */
		else
			writel( 0, &regs->sd_stop);/* disable */

		if (data->blocks>1) {
			cmd_tmp |= MSK_NX4000_SDIO_SD_CMD_MD5; /* enable multiple block mode */
			if (!s_fAutomaticBlockCount)
				cmd_tmp |= MSK_NX4000_SDIO_SD_CMD_MD_MLT_BLK;/* stop command required */
		}
	}

	writel( cmd->cmdarg, &regs->sd_arg0);
	/* issue command */
	writel(cmd_tmp, &regs->sd_cmd);
}

static int sdmmc_send_cmd(struct mci_host *mci, struct mci_cmd *cmd,
				struct mci_data *data)
{
	struct sdmmc_host *host = to_sdmmc_host(mci);
	struct netx4000_sdio_reg __iomem *regs = host->regs;
	int ret = 0;

	/* handle stop command separate */
	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION) {
		if (s_fAutomaticBlockCount)
			return 0;/* stop command is handle by hardware -> so return */
		else
			return handle_stop_cmd( host);
	}

	netx4000_wait_for_sclkdiven(host);

	/* Acknowledge any pending "response end" bit. */
	writel(~MSK_NX4000_SDIO_SD_INFO1_INFO0,&regs->sd_info1);
	/* clear all flags */
	writel(0, &regs->sd_info2);

	/* issue command */
	setup_cmd(host, cmd, data);
	/* wait resp or error */
	if (0>(ret = wait_for_response( host))) {
		s_fNextAppCmd = 0;
		return ret;
	} else {
		/* in case of no error transfer data present */
		get_resp( host, cmd);
		if (data) {
			if (data->flags & MMC_DATA_WRITE)
				ret = transfer_data( host, data->src, data->blocks, data->blocksize, data->flags);
			else
				ret = transfer_data( host, data->dest, data->blocks, data->blocksize, data->flags);
		}
	}

	return ret;
}

static void sdmmc_set_clock(struct sdmmc_host *host, u32 clock)
{
	u32 clock_setting = 0;
	u32 div;
	unsigned long max = clk_get_rate(host->clk);

	/* do not write to clk-ctrl while SCLKDIVEN is not set */
	netx4000_wait_for_sclkdiven(host);

	/* disable clock */
	writel(clock_setting, &host->regs->sd_clk_ctrl);

	/* calculate new clock divider ... */
	if (clock>=max) {
		writel( 0xFF, &host->regs->sd_clk_ctrl);
	} else {
		div = 2;
		while (div < 1024) {
			if (clock>(max/div))
				break;

			div <<= 1;
		}
		/* shift for register */
		/* clear clock settings */
		clock_setting &= ~0xFF;
		clock_setting |= MSK_NX4000_SDIO_SD_CLK_CTRL_DIV & (div >> 2);
	}
	/* ... and write it to chip */
	writel(clock_setting | MSK_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_OFFEN, &host->regs->sd_clk_ctrl);

	/* enable clock */
	clock_setting |= MSK_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_EN;
	writel( clock_setting | MSK_NX4000_SDIO_SD_CLK_CTRL_SD_CLK_OFFEN, &host->regs->sd_clk_ctrl);
}

static void sdmmc_set_ios(struct mci_host *mci, struct mci_ios *ios)
{
	struct sdmmc_host *host = to_sdmmc_host(mci);
	u32 val;

	dev_dbg(host->dev,"%s: clock=%d, bus_width=%d, timing=%d\n", __func__,
		ios->clock, ios->bus_width, ios->timing
	);

	/* set clock */
	if (ios->clock)
		sdmmc_set_clock(host, ios->clock);

	/* set bus width */
	val = readl(&host->regs->sd_option);
	if (ios->bus_width == MMC_BUS_WIDTH_1)
		val |= (1 << SRT_NX4000_SDIO_SD_OPTION_WIDTH);
	else if (ios->bus_width == MMC_BUS_WIDTH_4)
		val &= ~(1 << SRT_NX4000_SDIO_SD_OPTION_WIDTH);

	writel( val, &host->regs->sd_option);
}

static int sdmmc_init(struct mci_host *mci, struct device_d *dev)
{
	struct sdmmc_host *host = to_sdmmc_host(mci);
	struct netx4000_sdio_reg __iomem *regs = host->regs;
	u32 tmpreg;

	/* reset */
	writel( 0, &regs->soft_rst);
	ndelay(50);
	writel( 1, &regs->soft_rst);

	/* clear all interrupts */
	writel( 0x0000031D, &regs->sd_info1_mask); /* disable all interrupts */
	writel( 0x00008B7F, &regs->sd_info2_mask); /* disable all interrupts */
	writel( 0x0000C007, &regs->sdio_info1_mask); /* disable all interrupts */

	/* initialize all register */
	writel( 0x00000000, &regs->cc_ext_mode);
	writel( 0x00000000, &regs->sdif_mode);
	writel( 0x00000000, &regs->host_mode);/* 32-bit access */
	writel( 0x00000000, &regs->sdio_mode);
	writel( 0x00000000, &regs->ext_swap);
	writel( 0x00000000, &regs->sd_portsel);

	/* reset pending infos */
	tmpreg = readl( &regs->sd_info1);
	tmpreg |= ~(1|4);
	writel( tmpreg, &regs->sd_info1);
	writel( 0x00000000, &regs->sd_info2);
	writel( 0x00000000, &regs->sdio_info1);
	writel( 0x0000C0EE, &regs->sd_option);/* max. timeout */

	sdmmc_set_clock(host, 400000);
	writel( ~0x10,&regs->sd_info1);

	return 0;
}

static int sdmmc_card_present(struct mci_host *mci)
{
	struct sdmmc_host *host = to_sdmmc_host(mci);
	struct netx4000_sdio_reg __iomem *regs = host->regs;
	u32 reg;

	reg = readl(&regs->sd_info1);
	if (reg & MSK_NX4000_SDIO_SD_INFO1_INFO5)  /* check CD level */
		return 1; /* inserted */

	return 0;
}

static int sdmmc_probe(struct device_d *dev)
{
	struct sdmmc_host *host;
	struct mci_host *mci;

	host = xzalloc(sizeof(*host));

	host->dev = dev;

	host->regs = dev_request_mem_region(dev, 0);
	if (!host->regs) {
		dev_err(dev, "could not get iomem region\n");
		return -ENODEV;
	}

	host->clk = clk_get(dev, NULL);
	if (IS_ERR(host->clk))
		return PTR_ERR(host->clk);
	clk_enable(host->clk);

	mci = &host->mci;
	mci->hw_dev = dev;
	mci->f_max = clk_get_rate(host->clk);
	mci->f_min = clk_get_rate(host->clk)/512;

	/* setup function pointer */
	mci->init = sdmmc_init;
	mci->send_cmd = sdmmc_send_cmd;
	mci->card_present = sdmmc_card_present;
	mci->set_ios = sdmmc_set_ios;
	//TODO: voltage
	mci->voltages = 0x00FF8000;//MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
	mci->host_caps |= MMC_CAP_4_BIT_DATA;

	dev->priv = host;
	//TODO: device detect func
	//dev->detect =

	return mci_register( &host->mci);
}

static __maybe_unused struct of_device_id sdmmc_compatible[] = {
	{
		.compatible = "hilscher,netx4000-sdio",
	}, {
		/* sentinel */
	}
};

static struct driver_d netx4000_sdmmc_driver = {
	.name  = DRIVER_NAME,
	.probe = sdmmc_probe,
	.of_compatible = DRV_OF_COMPAT(sdmmc_compatible),
};

static int __init netx4000_sdmmc_init(void)
{
	pr_info("%s: %s\n", DRIVER_NAME, DRIVER_DESC);
	return platform_driver_register(&netx4000_sdmmc_driver);
}
device_initcall(netx4000_sdmmc_init);

/* --- Module information --- */

MODULE_AUTHOR("Hilscher Gesellschaft fuer Systemautomation mbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
