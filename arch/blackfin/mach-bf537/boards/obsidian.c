/* obsidian.c --- board resources for the Lumenosys Robotics Obsidian board
 * Copyright (c) 2013 Devin Butterfield. All rights reserved. 
 * Filename: obsidian.c
 * Description: 
 * Author: Devin Butterfield
 * Maintainer: 
 * Created: Mon Jul 29 17:06:37 2013 (-0700)
 * Version: 
 * Last-Updated: Sun Jan 25 13:33:49 2015 (-0800)
 *           By: Devin Butterfield
 *     Update #: 84
 * URL: 
 * Keywords: 
 * Compatibility: 
 * 
 */

/* Commentary: 
 * 
 * 
 * 
 */

/* Change Log:
 * 
 * 
 */

/* *This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 3, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth
 * Floor, Boston, MA 02110-1301, USA.
 */

/* Code: */
#include <linux/device.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/plat-ram.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/adp5588.h>
#include <linux/etherdevice.h>
#include <linux/ata_platform.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/usb/sl811.h>
#include <linux/spi/mmc_spi.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <asm/dma.h>
#include <asm/bfin5xx_spi.h>
#include <asm/reboot.h>
#include <asm/portmux.h>
#include <asm/dpmc.h>
#include <asm/bfin_sport.h>
#ifdef CONFIG_REGULATOR_FIXED_VOLTAGE
#include <linux/regulator/fixed.h>
#endif
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/userspace-consumer.h>

#include <linux/platform_data/ad7091R2.h>

/*
 * Name the Board for the /proc/cpuinfo
 */
const char bfin_board_name[] = "Lumenosys Robotics Obsidian board";

#if defined(CONFIG_RTC_DRV_BFIN) || defined(CONFIG_RTC_DRV_BFIN_MODULE)
static struct platform_device rtc_device = {
	.name = "rtc-bfin",
	.id   = -1,
};
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#include <linux/gpio_keys.h>

static struct gpio_keys_button bfin_gpio_keys_table[] = {
	/* {BTN_0, GPIO_PH6, 1, "gpio-keys: BTN0"}, */
	{
		.code = BTN_0,
		.gpio = GPIO_PH6, /* gpio38 */
		.active_low = 1,
		.desc = "gpio-keys: BTN0",
		.type = EV_KEY,
		.wakeup = 1,
		.debounce_interval = 20,
		.can_disable = 1,
	},
	/* {BTN_1, GPIO_PH1, 1, "gpio-keys: BTN1"}, */
	{
		.code = BTN_1,
		.gpio = GPIO_PH1, /* gpio33 */
		.active_low = 1,
		.desc = "gpio-keys: BTN1",
		.type = EV_KEY,
		.wakeup = 1,
		.debounce_interval = 20,
		.can_disable = 1,
	},
};

static struct gpio_keys_platform_data bfin_gpio_keys_data = {
	.buttons        = bfin_gpio_keys_table,
	.nbuttons       = ARRAY_SIZE(bfin_gpio_keys_table),
};

static struct platform_device bfin_device_gpiokeys = {
	.name      = "gpio-keys",
	.dev = {
		.platform_data = &bfin_gpio_keys_data,
	},
};
#endif

#if defined(CONFIG_REGULATOR_FIXED_VOLTAGE) || defined(CONFIG_REGULATOR_FIXED_VOLTAGE_MODULE)
static struct regulator_consumer_supply ad7091R2_consumer_supplies[] = {
	REGULATOR_SUPPLY("vcc", "spi0.2"),
};
 
static struct regulator_init_data obsidian_avdd_reg_init_data = {
	.constraints= {
		.name= "3V3",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = ad7091R2_consumer_supplies,
	.num_consumer_supplies = ARRAY_SIZE(ad7091R2_consumer_supplies),
};
 
static struct fixed_voltage_config obsidian_vdd_pdata = {
	.supply_name= "int-3V3",
	.microvolts= 3300000,
	.gpio= -EINVAL,
	.enabled_at_boot = 1,
	.init_data= &obsidian_avdd_reg_init_data,
};
static struct platform_device brd_voltage_regulator = {
	.name= "reg-fixed-voltage",
	.id= -1,
	.num_resources= 0,
	.dev= {
		.platform_data= &obsidian_vdd_pdata,
	},
};
#endif

#if defined(CONFIG_CAN_BFIN) || defined(CONFIG_CAN_BFIN_MODULE)
static unsigned short bfin_can_peripherals[] = {
	P_CAN0_RX, P_CAN0_TX, 0
};

static struct resource bfin_can_resources[] = {
	{
		.start = 0xFFC02A00,
		.end = 0xFFC02FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = IRQ_CAN_RX,
		.end = IRQ_CAN_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_CAN_TX,
		.end = IRQ_CAN_TX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_CAN_ERROR,
		.end = IRQ_CAN_ERROR,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device bfin_can_device = {
	.name = "bfin_can",
	.num_resources = ARRAY_SIZE(bfin_can_resources),
	.resource = bfin_can_resources,
	.dev = {
		.platform_data = &bfin_can_peripherals, /* Passed to driver */
	},
};
#endif

#if defined(CONFIG_BFIN_MAC) || defined(CONFIG_BFIN_MAC_MODULE)
#include <linux/bfin_mac.h>
static const unsigned short bfin_mac_peripherals[] = P_MII0;

static struct bfin_phydev_platform_data bfin_phydev_data[] = {
	{
		.addr = 1,
		.irq = PHY_POLL, /* IRQ_MAC_PHYINT */
	},
};

static struct bfin_mii_bus_platform_data bfin_mii_bus_data = {
	.phydev_number = 1,
	.phydev_data = bfin_phydev_data,
	.phy_mode = PHY_INTERFACE_MODE_MII,
	.mac_peripherals = bfin_mac_peripherals,
};

static struct platform_device bfin_mii_bus = {
	.name = "bfin_mii_bus",
	.dev = {
		.platform_data = &bfin_mii_bus_data,
	}
};

static struct platform_device bfin_mac_device = {
	.name = "bfin_mac",
	.dev = {
		.platform_data = &bfin_mii_bus,
	}
};
#endif

#if defined(CONFIG_MTD_M25P80) \
	|| defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition bfin_spi_flash_partitions[] = {
	{
		.name = "bootloader(spi)",
		.size = 0x000C0000,     /* moved up to 0xC0000 to accomodate
                                   larger flash sector size of
                                   M25P128 */
		.offset = 0,
		.mask_flags = MTD_CAP_ROM
	}, {
		.name = "linux kernel(spi)",
		.size = 0x200000, /* was 0x180000, moved up to fit
				   * kernel with ipv6 and filesystem support */
		.offset = MTDPART_OFS_APPEND,
	}, {
		.name = "file system(spi)",
		.size = MTDPART_SIZ_FULL,
		.offset = MTDPART_OFS_APPEND,
	}
};

static struct flash_platform_data bfin_spi_flash_data = {
	.name = "m25p80",
	.parts = bfin_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(bfin_spi_flash_partitions),
	/* .type = "m25p64", */
};

/* SPI flash chip (m25p64) */
static struct bfin5xx_spi_chip spi_flash_chip_info = {
	.enable_dma = 0,         /* use dma transfer with this chip*/
};
#endif

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
#define MMC_SPI_CARD_DETECT_INT IRQ_PH7

static int bfin_mmc_spi_init(struct device *dev,
	irqreturn_t (*detect_int)(int, void *), void *data)
{
	return request_irq(MMC_SPI_CARD_DETECT_INT, detect_int,
		IRQF_TRIGGER_FALLING, "mmc-spi-detect", data);
}

static void bfin_mmc_spi_exit(struct device *dev, void *data)
{
	free_irq(MMC_SPI_CARD_DETECT_INT, data);
}

static struct mmc_spi_platform_data bfin_mmc_spi_pdata = {
	.init = bfin_mmc_spi_init,
	.exit = bfin_mmc_spi_exit,
	.detect_delay = 100, /* msecs */
};

static struct bfin5xx_spi_chip  mmc_spi_chip_info = {
	.enable_dma = 0,
	.pio_interrupt = 0,
};
#endif

/* add support for userspace ad7091r2 driver */
static struct bfin5xx_spi_chip  ad7091R2_spi_chip_info = {
	.enable_dma = 0,
};

static struct ad7091R2_platform_data ad7091R2_pdata = {
	.mode = 0,
	.gpio_convst = GPIO_PF15,
};

static struct spi_board_info bfin_spi_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80)			\
	|| defined(CONFIG_MTD_M25P80_MODULE)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 30000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 1, /* Framework chip select. On Obsidian it is SPISSEL1*/
		.platform_data = &bfin_spi_flash_data,
		.controller_data = &spi_flash_chip_info,
		.mode = SPI_MODE_3,
	},
#endif
	
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias = "mmc_spi",
		.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 3,
		.platform_data = &bfin_mmc_spi_pdata,
		.controller_data = &mmc_spi_chip_info,
		.mode = SPI_MODE_3,
	},
#endif
	
/* #if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE) */
/* 	{ */
/* 		.modalias = "spidev", */
/* 		.max_speed_hz = 20000000,     /\* max spi clock (SCK) speed in HZ *\/ */
/* 		.bus_num = 0, */
/* 		.chip_select = 2, */
/* 		.mode = SPI_MODE_3, */
/* 	}, */
/* #endif */
	
#if defined(CONFIG_AD7091R2) || defined(CONFIG_AD7091R2_MODULE)
	{
		.modalias = "ad7091R2", /* Name of spi_driver for this device */
		.max_speed_hz = 30000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* Framework bus number */
		.chip_select = 2, /* Framework chip select */
		.controller_data = &ad7091R2_spi_chip_info,
		.platform_data = &ad7091R2_pdata,
		.mode = SPI_MODE_3,
	},
#endif
	
};

#if defined(CONFIG_SPI_BFIN5XX) || defined(CONFIG_SPI_BFIN5XX_MODULE)
/* SPI controller data */
static struct bfin5xx_spi_master bfin_spi0_info = {
	.num_chipselect = MAX_CTRL_CS + MAX_BLACKFIN_GPIOS,
	.enable_dma = 1,  /* master has the ability to do dma transfer */
	.pin_req = {P_SPI0_SCK, P_SPI0_MISO, P_SPI0_MOSI, 0},
};

/* SPI (0) */
static struct resource bfin_spi0_resource[] = {
	[0] = {
		.start = SPI0_REGBASE,
		.end   = SPI0_REGBASE + 0xFF,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
		.start = CH_SPI,
		.end   = CH_SPI,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.start = IRQ_SPI,
		.end   = IRQ_SPI,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device bfin_spi0_device = {
	.name = "bfin-spi",
	.id = 0, /* Bus number */
	.num_resources = ARRAY_SIZE(bfin_spi0_resource),
	.resource = bfin_spi0_resource,
	.dev = {
		.platform_data = &bfin_spi0_info, /* Passed to driver */
	},
};

#endif  /* spi master and devices */

#if defined(CONFIG_SPI_BFIN_SPORT) || defined(CONFIG_SPI_BFIN_SPORT_MODULE)

/* SPORT SPI controller data */
static struct bfin5xx_spi_master bfin_sport_spi0_info = {
	.num_chipselect = MAX_BLACKFIN_GPIOS,
	.enable_dma = 0,  /* master don't support DMA */
	.pin_req = {P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_DRPRI,
		P_SPORT0_RSCLK, P_SPORT0_TFS, P_SPORT0_RFS, 0},
};

static struct resource bfin_sport_spi0_resource[] = {
	[0] = {
		.start = SPORT0_TCR1,
		.end   = SPORT0_TCR1 + 0xFF,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
		.start = IRQ_SPORT0_ERROR,
		.end   = IRQ_SPORT0_ERROR,
		.flags = IORESOURCE_IRQ,
		},
};

static struct platform_device bfin_sport_spi0_device = {
	.name = "bfin-sport-spi",
	.id = 1, /* Bus number */
	.num_resources = ARRAY_SIZE(bfin_sport_spi0_resource),
	.resource = bfin_sport_spi0_resource,
	.dev = {
		.platform_data = &bfin_sport_spi0_info, /* Passed to driver */
	},
};

static struct bfin5xx_spi_master bfin_sport_spi1_info = {
	.num_chipselect = MAX_BLACKFIN_GPIOS,
	.enable_dma = 0,  /* master don't support DMA */
	.pin_req = {P_SPORT1_DTPRI, P_SPORT1_TSCLK, P_SPORT1_DRPRI,
		P_SPORT1_RSCLK, P_SPORT1_TFS, P_SPORT1_RFS, 0},
};

static struct resource bfin_sport_spi1_resource[] = {
	[0] = {
		.start = SPORT1_TCR1,
		.end   = SPORT1_TCR1 + 0xFF,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
		.start = IRQ_SPORT1_ERROR,
		.end   = IRQ_SPORT1_ERROR,
		.flags = IORESOURCE_IRQ,
		},
};

static struct platform_device bfin_sport_spi1_device = {
	.name = "bfin-sport-spi",
	.id = 2, /* Bus number */
	.num_resources = ARRAY_SIZE(bfin_sport_spi1_resource),
	.resource = bfin_sport_spi1_resource,
	.dev = {
		.platform_data = &bfin_sport_spi1_info, /* Passed to driver */
	},
};

#endif  /* sport spi master and devices */

#if defined(CONFIG_VIDEO_BLACKFIN_CAPTURE) \
	|| defined(CONFIG_VIDEO_BLACKFIN_CAPTURE_MODULE)
#include <linux/videodev2.h>
#include <media/blackfin/bfin_capture.h>
#include <media/blackfin/ppi.h>

static const unsigned short ppi_req[] = {
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3,
	P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7,
	P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2,
	0,
};

static const struct ppi_info ppi_info = {
	.type = PPI_TYPE_PPI,
	.dma_ch = CH_PPI,
	.irq_err = IRQ_PPI_ERROR,
	.base = (void __iomem *)PPI_CONTROL,
	.pin_req = ppi_req,
};

#if defined(CONFIG_VIDEO_VS6624) \
	|| defined(CONFIG_VIDEO_VS6624_MODULE)
static struct v4l2_input vs6624_inputs[] = {
	{
		.index = 0,
		.name = "Camera",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = V4L2_STD_UNKNOWN,
	},
};

static struct bcap_route vs6624_routes[] = {
	{
		.input = 0,
		.output = 0,
	},
};

static const unsigned vs6624_ce_pin = GPIO_PF10;

static struct bfin_capture_config bfin_capture_data = {
	.card_name = "BF537",
	.inputs = vs6624_inputs,
	.num_inputs = ARRAY_SIZE(vs6624_inputs),
	.routes = vs6624_routes,
	.i2c_adapter_id = 0,
	.board_info = {
		.type = "vs6624",
		.addr = 0x10,
		.platform_data = (void *)&vs6624_ce_pin,
	},
	.ppi_info = &ppi_info,
	.ppi_control = (PACK_EN | DLEN_8 | XFR_TYPE | 0x0020),
};
#endif

static struct platform_device bfin_capture_device = {
	.name = "bfin_capture",
	.dev = {
		.platform_data = &bfin_capture_data,
	},
};
#endif


#if defined(CONFIG_SERIAL_BFIN) || defined(CONFIG_SERIAL_BFIN_MODULE)
#ifdef CONFIG_SERIAL_BFIN_UART0
static struct resource bfin_uart0_resources[] = {
	{
		.start = UART0_THR,
		.end = UART0_GCTL+2,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = IRQ_UART0_TX,
		.end = IRQ_UART0_TX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_UART0_RX,
		.end = IRQ_UART0_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_UART0_ERROR,
		.end = IRQ_UART0_ERROR,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CH_UART0_TX,
		.end = CH_UART0_TX,
		.flags = IORESOURCE_DMA,
	},
	{
		.start = CH_UART0_RX,
		.end = CH_UART0_RX,
		.flags = IORESOURCE_DMA,
	},
#ifdef CONFIG_BFIN_UART0_CTSRTS
	{	/* CTS pin */
		.start = GPIO_PG7,
		.end = GPIO_PG7,
		.flags = IORESOURCE_IO,
	},
	{	/* RTS pin */
		.start = GPIO_PG6,
		.end = GPIO_PG6,
		.flags = IORESOURCE_IO,
	},
#endif
};

static unsigned short bfin_uart0_peripherals[] = {
	P_UART0_TX, P_UART0_RX, 0
};

static struct platform_device bfin_uart0_device = {
	.name = "bfin-uart",
	.id = 0,
	.num_resources = ARRAY_SIZE(bfin_uart0_resources),
	.resource = bfin_uart0_resources,
	.dev = {
		.platform_data = &bfin_uart0_peripherals, /* Passed to driver */
	},
};
#endif
#ifdef CONFIG_SERIAL_BFIN_UART1
static struct resource bfin_uart1_resources[] = {
	{
		.start = UART1_THR,
		.end = UART1_GCTL+2,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = IRQ_UART1_TX,
		.end = IRQ_UART1_TX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_UART1_RX,
		.end = IRQ_UART1_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_UART1_ERROR,
		.end = IRQ_UART1_ERROR,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CH_UART1_TX,
		.end = CH_UART1_TX,
		.flags = IORESOURCE_DMA,
	},
	{
		.start = CH_UART1_RX,
		.end = CH_UART1_RX,
		.flags = IORESOURCE_DMA,
	},
#ifdef CONFIG_BFIN_UART0_CTSRTS
	{	/* CTS pin */
		.start = GPIO_PG7,
		.end = GPIO_PG7,
		.flags = IORESOURCE_IO,
	},
	{	/* RTS pin */
		.start = GPIO_PG6,
		.end = GPIO_PG6,
		.flags = IORESOURCE_IO,
	},
#endif
};

static unsigned short bfin_uart1_peripherals[] = {
	P_UART1_TX, P_UART1_RX, 0
};

static struct platform_device bfin_uart1_device = {
	.name = "bfin-uart",
	.id = 1,
	.num_resources = ARRAY_SIZE(bfin_uart1_resources),
	.resource = bfin_uart1_resources,
	.dev = {
		.platform_data = &bfin_uart1_peripherals, /* Passed to driver */
	},
};
#endif
#endif


/* #if defined(CONFIG_I2C_ADI_TWI) || defined(CONFIG_I2C_ADI_TWI_MODULE) */
#if defined(CONFIG_I2C_BLACKFIN_TWI) || defined(CONFIG_I2C_BLACKFIN_TWI_MODULE)
static const u16 bfin_twi0_pins[] = {P_TWI0_SCL, P_TWI0_SDA, 0};

static struct resource bfin_twi0_resource[] = {
	[0] = {
		.start = TWI0_REGBASE,
		.end   = TWI0_REGBASE,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TWI,
		.end   = IRQ_TWI,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device i2c_bfin_twi_device = {
	.name = "i2c-bfin-twi",
	.id = 0,
	.num_resources = ARRAY_SIZE(bfin_twi0_resource),
	.resource = bfin_twi0_resource,
	.dev = {
		.platform_data = &bfin_twi0_pins,
	},
};
#endif

static struct i2c_board_info __initdata bfin_i2c_board_info[] = {
};

#if defined(CONFIG_BFIN_RTDM_I2C) || defined(CONFIG_BFIN_RTDM_I2C_MODULE)
static const u16 bfin_twi0_pins[] = {P_TWI0_SCL, P_TWI0_SDA, 0};

static struct resource bfin_twi0_resource[] = {
	[0] = {
		.start = TWI0_REGBASE,
		.end   = TWI0_REGBASE,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TWI,
		.end   = IRQ_TWI,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device i2c_bfin_twi_device = {
	.name = "bfin_rtdm_i2c",
	.id = 0,
	.num_resources = ARRAY_SIZE(bfin_twi0_resource),
	.resource = bfin_twi0_resource,
	.dev = {
		.platform_data = &bfin_twi0_pins,
	},
};
#endif

/* static struct i2c_board_info __initdata bfin_i2c_board_info[] = { */
/* }; */

#if defined(CONFIG_SERIAL_BFIN_SPORT) || defined(CONFIG_SERIAL_BFIN_SPORT_MODULE)
#ifdef CONFIG_SERIAL_BFIN_SPORT0_UART
static struct resource bfin_sport0_uart_resources[] = {
	{
		.start = SPORT0_TCR1,
		.end = SPORT0_MRCS3+4,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = IRQ_SPORT0_RX,
		.end = IRQ_SPORT0_RX+1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_SPORT0_ERROR,
		.end = IRQ_SPORT0_ERROR,
		.flags = IORESOURCE_IRQ,
	},
};

static unsigned short bfin_sport0_peripherals[] = {
	P_SPORT0_TFS, P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_RFS,
	P_SPORT0_DRPRI, P_SPORT0_RSCLK, 0
};

static struct platform_device bfin_sport0_uart_device = {
	.name = "bfin-sport-uart",
	.id = 0,
	.num_resources = ARRAY_SIZE(bfin_sport0_uart_resources),
	.resource = bfin_sport0_uart_resources,
	.dev = {
		.platform_data = &bfin_sport0_peripherals, /* Passed to driver */
	},
};
#endif
#ifdef CONFIG_SERIAL_BFIN_SPORT1_UART
static struct resource bfin_sport1_uart_resources[] = {
	{
		.start = SPORT1_TCR1,
		.end = SPORT1_MRCS3+4,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = IRQ_SPORT1_RX,
		.end = IRQ_SPORT1_RX+1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_SPORT1_ERROR,
		.end = IRQ_SPORT1_ERROR,
		.flags = IORESOURCE_IRQ,
	},
};

static unsigned short bfin_sport1_peripherals[] = {
	P_SPORT1_TFS, P_SPORT1_DTPRI, P_SPORT1_TSCLK, P_SPORT1_RFS,
	P_SPORT1_DRPRI, P_SPORT1_RSCLK, 0
};

static struct platform_device bfin_sport1_uart_device = {
	.name = "bfin-sport-uart",
	.id = 1,
	.num_resources = ARRAY_SIZE(bfin_sport1_uart_resources),
	.resource = bfin_sport1_uart_resources,
	.dev = {
		.platform_data = &bfin_sport1_peripherals, /* Passed to driver */
	},
};
#endif
#endif

static const unsigned int cclk_vlev_datasheet[] =
{
	VRPAIR(VLEV_085, 250000000),
	VRPAIR(VLEV_090, 376000000),
	VRPAIR(VLEV_095, 426000000),
	VRPAIR(VLEV_100, 426000000),
	VRPAIR(VLEV_105, 476000000),
	VRPAIR(VLEV_110, 476000000),
	VRPAIR(VLEV_115, 476000000),
	VRPAIR(VLEV_120, 500000000),
	VRPAIR(VLEV_125, 533000000),
	VRPAIR(VLEV_130, 600000000),
};

static struct bfin_dpmc_platform_data bfin_dmpc_vreg_data = {
	.tuple_tab = cclk_vlev_datasheet,
	.tabsize = ARRAY_SIZE(cclk_vlev_datasheet),
	.vr_settling_time = 25 /* us */,
};

static struct platform_device bfin_dpmc = {
	.name = "bfin dpmc",
	.dev = {
		.platform_data = &bfin_dmpc_vreg_data,
	},
};

#if defined(CONFIG_SND_BF5XX_I2S) || defined(CONFIG_SND_BF5XX_I2S_MODULE) || \
	defined(CONFIG_SND_BF5XX_TDM) || defined(CONFIG_SND_BF5XX_TDM_MODULE) || \
	defined(CONFIG_SND_BF5XX_AC97) || defined(CONFIG_SND_BF5XX_AC97_MODULE)

#define SPORT_REQ(x) \
	[x] = {P_SPORT##x##_TFS, P_SPORT##x##_DTPRI, P_SPORT##x##_TSCLK, \
		P_SPORT##x##_RFS, P_SPORT##x##_DRPRI, P_SPORT##x##_RSCLK, 0}

static const u16 bfin_snd_pin[][7] = {
	SPORT_REQ(0),
	SPORT_REQ(1),
};
static struct bfin_snd_platform_data bfin_snd_data[] = {
	{
		.pin_req = &bfin_snd_pin[0][0],
	},
	{
		.pin_req = &bfin_snd_pin[1][0],
	},
};

#define BFIN_SND_RES(x) \
	[x] = { \
		{ \
			.start = SPORT##x##_TCR1, \
			.end = SPORT##x##_TCR1, \
			.flags = IORESOURCE_MEM \
		}, \
		{ \
			.start = CH_SPORT##x##_RX, \
			.end = CH_SPORT##x##_RX, \
			.flags = IORESOURCE_DMA, \
		}, \
		{ \
			.start = CH_SPORT##x##_TX, \
			.end = CH_SPORT##x##_TX, \
			.flags = IORESOURCE_DMA, \
		}, \
		{ \
			.start = IRQ_SPORT##x##_ERROR, \
			.end = IRQ_SPORT##x##_ERROR, \
			.flags = IORESOURCE_IRQ, \
		} \
	}

static struct resource bfin_snd_resources[][4] = {
	BFIN_SND_RES(0),
	BFIN_SND_RES(1),
};
#endif

#if defined(CONFIG_IIO_BFIN_TMR_TRIGGER) || defined(CONFIG_IIO_BFIN_TMR_TRIGGER_MODULE)
static struct resource iio_bfin_trigger_resources[] = {
	{
		.start = IRQ_TIMER5,
		.end = IRQ_TIMER5,
		.flags = IORESOURCE_IRQ,
	},
};


/**
 * struct iio_bfin_timer_trigger_pdata - timer trigger platform data
 * @output_enable: Enable external trigger pulse generation.
 * @active_low: Whether the trigger pulse is active low.
 * @duty_ns: Length of the trigger pulse in nanoseconds.
 *
 * This struct is used to configure the output pulse generation of the blackfin
 * timer trigger. If output_enable is set to true an external trigger signal
 * will generated on the pin corresponding to the timer. This is useful for
 * converters which needs an external signal to start conversion. active_low and
 * duty_ns are used to configure the type of the trigger pulse. If output_enable
 * is set to false no external trigger pulse will be generated and active_low
 * and duty_ns are ignored.
 **/
struct iio_bfin_timer_trigger_pdata {
	bool output_enable;
	bool active_low;
	unsigned int duty_ns;
};

static struct iio_bfin_timer_trigger_pdata iio_bfin_tmr_trig_pdata = {
	.output_enable = 1,
	.active_low = 1,
	.duty_ns = 100,
};

static struct platform_device iio_bfin_trigger = {
	.name = "iio_bfin_tmr_trigger",
	.id = 0,
	.num_resources = ARRAY_SIZE(iio_bfin_trigger_resources),
	.resource = iio_bfin_trigger_resources,
	.dev = {
		.platform_data = &iio_bfin_tmr_trig_pdata,
	},
};
#endif


static struct platform_device *obsidian_devices[] __initdata = {

	&bfin_dpmc,

#if defined(CONFIG_RTC_DRV_BFIN) || defined(CONFIG_RTC_DRV_BFIN_MODULE)
	&rtc_device,
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	&bfin_device_gpiokeys,
#endif

#if defined(CONFIG_CAN_BFIN) || defined(CONFIG_CAN_BFIN_MODULE)
	&bfin_can_device,
#endif

#if defined(CONFIG_BFIN_MAC) || defined(CONFIG_BFIN_MAC_MODULE)
	&bfin_mii_bus,
	&bfin_mac_device,
#endif

#if defined(CONFIG_SPI_BFIN5XX) || defined(CONFIG_SPI_BFIN5XX_MODULE)
	&bfin_spi0_device,
#endif

#if defined(CONFIG_SPI_BFIN_SPORT) || defined(CONFIG_SPI_BFIN_SPORT_MODULE)
	&bfin_sport_spi0_device,
	&bfin_sport_spi1_device,
#endif

#if defined(CONFIG_SERIAL_BFIN) || defined(CONFIG_SERIAL_BFIN_MODULE)
#ifdef CONFIG_SERIAL_BFIN_UART0
	&bfin_uart0_device,
#endif
#ifdef CONFIG_SERIAL_BFIN_UART1
	&bfin_uart1_device,
#endif
#endif
#if defined(CONFIG_VIDEO_BLACKFIN_CAPTURE) \
	|| defined(CONFIG_VIDEO_BLACKFIN_CAPTURE_MODULE)
	&bfin_capture_device,
#endif

#if defined(CONFIG_I2C_BLACKFIN_TWI) || defined(CONFIG_I2C_BLACKFIN_TWI_MODULE)
	&i2c_bfin_twi_device,
#endif

/* #if defined(CONFIG_I2C_ADI_TWI) || defined(CONFIG_I2C_ADI_TWI_MODULE) */
/* 	&i2c_bfin_twi_device, */
/* #endif */

#if defined(CONFIG_BFIN_RTDM_I2C) || defined(CONFIG_BFIN_RTDM_I2C_MODULE)
	&i2c_bfin_twi_device,
#endif
    
#if defined(CONFIG_SERIAL_BFIN_SPORT) || defined(CONFIG_SERIAL_BFIN_SPORT_MODULE)
#ifdef CONFIG_SERIAL_BFIN_SPORT0_UART
	&bfin_sport0_uart_device,
#endif
#ifdef CONFIG_SERIAL_BFIN_SPORT1_UART
	&bfin_sport1_uart_device,
#endif
#endif

#if defined(CONFIG_REGULATOR_FIXED_VOLTAGE) || defined(CONFIG_REGULATOR_FIXED_VOLTAGE_MODULE)
	&brd_voltage_regulator,
#endif

#if defined(CONFIG_IIO_BFIN_TMR_TRIGGER) || defined(CONFIG_IIO_BFIN_TMR_TRIGGER_MODULE)
	&iio_bfin_trigger,
#endif

};

static int __init obsidian_init(void)
{
	printk(KERN_INFO "%s(): registering device resources\n", __func__);
	/* bfin_plat_nand_init(); */
	/* adf702x_mac_init(); */
	platform_add_devices(obsidian_devices, ARRAY_SIZE(obsidian_devices));
	i2c_register_board_info(0, bfin_i2c_board_info,
				ARRAY_SIZE(bfin_i2c_board_info));
	spi_register_board_info(bfin_spi_board_info, ARRAY_SIZE(bfin_spi_board_info));

	/* if (net2272_init()) */
	/* 	pr_warning("unable to configure net2272; it probably won't work\n"); */

	return 0;
}

arch_initcall(obsidian_init);


static struct platform_device *obsidian_early_devices[] __initdata = {
#if defined(CONFIG_SERIAL_BFIN_CONSOLE) || defined(CONFIG_EARLY_PRINTK)
#ifdef CONFIG_SERIAL_BFIN_UART0
	&bfin_uart0_device,
#endif
#ifdef CONFIG_SERIAL_BFIN_UART1
	&bfin_uart1_device,
#endif
#endif

#if defined(CONFIG_SERIAL_BFIN_SPORT_CONSOLE)
#ifdef CONFIG_SERIAL_BFIN_SPORT0_UART
	&bfin_sport0_uart_device,
#endif
#ifdef CONFIG_SERIAL_BFIN_SPORT1_UART
	&bfin_sport1_uart_device,
#endif
#endif
};

void __init native_machine_early_platform_add_devices(void)
{
	printk(KERN_INFO "register early platform devices\n");
	early_platform_add_devices(obsidian_early_devices,
		ARRAY_SIZE(obsidian_early_devices));
}

void native_machine_restart(char *cmd)
{
	/* workaround reboot hang when booting from SPI */
	if ((bfin_read_SYSCR() & 0x7) == 0x3)
		bfin_reset_boot_spi_cs(P_DEFAULT_BOOT_SPI_CS);
}

/*
 * Currently the MAC address is saved in Flash by U-Boot
 */
#define FLASH_MAC	0x203f0000
void bfin_get_ether_addr(char *addr)
{
	*(u32 *)(&(addr[0])) = bfin_read32(FLASH_MAC);
	*(u16 *)(&(addr[4])) = bfin_read16(FLASH_MAC + 4);
}
EXPORT_SYMBOL(bfin_get_ether_addr);


/* bf537_obsidian.c ends here */
