/*
 * Analog Devices SPI3 controller driver
 *
 * Copyright (c) 2013 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <asm/bfin6xx_spi.h>
#include <asm/cacheflush.h>
#include <asm/dma.h>
#include <asm/portmux.h>

#define START_STATE	((void *)0)
#define RUNNING_STATE	((void *)1)
#define DONE_STATE	((void *)2)
#define ERROR_STATE	((void *)-1)

struct bfin_spi_master;

struct bfin_spi_transfer_ops {
	void (*write) (struct bfin_spi_master *);
	void (*read) (struct bfin_spi_master *);
	void (*duplex) (struct bfin_spi_master *);
};

/* runtime info for spi master */
struct bfin_spi_master {
	/* SPI framework hookup */
	struct spi_master *master;

	/* Regs base of SPI controller */
	struct bfin_spi_regs __iomem *regs;

	/* Pin request list */
	u16 *pin_req;

	/* Message Transfer pump */
	struct tasklet_struct pump_transfers;

	/* Current message transfer state info */
	struct spi_message *cur_msg;
	struct spi_transfer *cur_transfer;
	struct bfin_spi_device *cur_chip;
	unsigned transfer_len;
	unsigned cs_change;

	/* transfer buffer */
	void *tx;
	void *tx_end;
	void *rx;
	void *rx_end;

	/* dma info */
	unsigned int tx_dma;
	unsigned int rx_dma;
	dma_addr_t tx_dma_addr;
	dma_addr_t rx_dma_addr;
	unsigned long dummy_buffer; /* used in unidirectional transfer */
	unsigned long tx_dma_size;
	unsigned long rx_dma_size;
	int tx_num;
	int rx_num;

	/* store register value for suspend/resume */
	u32 control;
	u32 ssel;

	const struct bfin_spi_transfer_ops *ops;
};

struct bfin_spi_device {
	u32 control;
	u32 clock;
	u32 ssel;

	u8 cs;
	u16 cs_chg_udelay; /* Some devices require > 255usec delay */
	u32 cs_gpio;
	u32 tx_dummy_val; /* tx value for rx only transfer */
	bool enable_dma;
	const struct bfin_spi_transfer_ops *ops;
};

static void bfin_spi_enable(struct bfin_spi_master *drv_data)
{
	bfin_write_or(&drv_data->regs->control, SPI_CTL_EN);
}

static void bfin_spi_disable(struct bfin_spi_master *drv_data)
{
	bfin_write_and(&drv_data->regs->control, ~SPI_CTL_EN);
}

/* Caculate the SPI_CLOCK register value based on input HZ */
static u32 hz_to_spi_clock(u32 speed_hz)
{
	u_long sclk = get_sclk1();
	u32 spi_clock = sclk / speed_hz;

	if (spi_clock)
		spi_clock--;
	return spi_clock;
}

static int bfin_spi_flush(struct bfin_spi_master *drv_data)
{
	unsigned long limit = loops_per_jiffy << 1;

	/* wait for stop and clear stat */
	while (!(bfin_read(&drv_data->regs->status) & SPI_STAT_SPIF) && --limit)
		cpu_relax();

	bfin_write(&drv_data->regs->status, 0xFFFFFFFF);

	return limit;
}

/* Chip select operation functions for cs_change flag */
static void bfin_spi_cs_active(struct bfin_spi_master *drv_data, struct bfin_spi_device *chip)
{
	if (likely(chip->cs < MAX_CTRL_CS))
		bfin_write_and(&drv_data->regs->ssel, ~chip->ssel);
	else
		gpio_set_value(chip->cs_gpio, 0);
}

static void bfin_spi_cs_deactive(struct bfin_spi_master *drv_data,
				struct bfin_spi_device *chip)
{
	if (likely(chip->cs < MAX_CTRL_CS))
		bfin_write_or(&drv_data->regs->ssel, chip->ssel);
	else
		gpio_set_value(chip->cs_gpio, 1);

	/* Move delay here for consistency */
	if (chip->cs_chg_udelay)
		udelay(chip->cs_chg_udelay);
}

/* enable or disable the pin muxed by GPIO and SPI CS to work as SPI CS */
static inline void bfin_spi_cs_enable(struct bfin_spi_master *drv_data,
					struct bfin_spi_device *chip)
{
	if (chip->cs < MAX_CTRL_CS)
		bfin_write_or(&drv_data->regs->ssel, chip->ssel >> 8);
}

static inline void bfin_spi_cs_disable(struct bfin_spi_master *drv_data,
					struct bfin_spi_device *chip)
{
	if (chip->cs < MAX_CTRL_CS)
		bfin_write_and(&drv_data->regs->ssel, ~(chip->ssel >> 8));
}

/* stop controller and re-config current chip*/
static void bfin_spi_restore_state(struct bfin_spi_master *drv_data)
{
	struct bfin_spi_device *chip = drv_data->cur_chip;

	/* Clear status and disable clock */
	bfin_write(&drv_data->regs->status, 0xFFFFFFFF);
	bfin_write(&drv_data->regs->rx_control, 0x0);
	bfin_write(&drv_data->regs->tx_control, 0x0);
	bfin_spi_disable(drv_data);

	SSYNC();

	/* Load the registers */
	bfin_write(&drv_data->regs->control, chip->control);
	bfin_write(&drv_data->regs->clock, chip->clock);

	bfin_spi_enable(drv_data);
	drv_data->tx_num = drv_data->rx_num = 0;
	/* we always choose tx transfer initiate */
	bfin_write(&drv_data->regs->rx_control, SPI_RXCTL_REN);
	bfin_write(&drv_data->regs->tx_control,
			SPI_TXCTL_TEN | SPI_TXCTL_TTI);
	bfin_spi_cs_active(drv_data, chip);
}

/* discard invalid rx data and empty rfifo */
static inline void dummy_read(struct bfin_spi_master *drv_data)
{
	while (!(bfin_read(&drv_data->regs->status) & SPI_STAT_RFE))
		bfin_read(&drv_data->regs->rfifo);
}

static void bfin_spi_u8_write(struct bfin_spi_master *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->tx < drv_data->tx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u8 *)(drv_data->tx++)));
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		bfin_read(&drv_data->regs->rfifo);
	}
}

static void bfin_spi_u8_read(struct bfin_spi_master *drv_data)
{
	u32 tx_val = drv_data->cur_chip->tx_dummy_val;

	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, tx_val);
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(drv_data->rx++) = bfin_read(&drv_data->regs->rfifo);
	}
}

static void bfin_spi_u8_duplex(struct bfin_spi_master *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u8 *)(drv_data->tx++)));
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(drv_data->rx++) = bfin_read(&drv_data->regs->rfifo);
	}
}

static const struct bfin_spi_transfer_ops bfin_bfin_spi_transfer_ops_u8 = {
	.write  = bfin_spi_u8_write,
	.read   = bfin_spi_u8_read,
	.duplex = bfin_spi_u8_duplex,
};

static void bfin_spi_u16_write(struct bfin_spi_master *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->tx < drv_data->tx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u16 *)drv_data->tx));
		drv_data->tx += 2;
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		bfin_read(&drv_data->regs->rfifo);
	}
}

static void bfin_spi_u16_read(struct bfin_spi_master *drv_data)
{
	u32 tx_val = drv_data->cur_chip->tx_dummy_val;

	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, tx_val);
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)drv_data->rx = bfin_read(&drv_data->regs->rfifo);
		drv_data->rx += 2;
	}
}

static void bfin_spi_u16_duplex(struct bfin_spi_master *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u16 *)drv_data->tx));
		drv_data->tx += 2;
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)drv_data->rx = bfin_read(&drv_data->regs->rfifo);
		drv_data->rx += 2;
	}
}

static const struct bfin_spi_transfer_ops bfin_bfin_spi_transfer_ops_u16 = {
	.write  = bfin_spi_u16_write,
	.read   = bfin_spi_u16_read,
	.duplex = bfin_spi_u16_duplex,
};

static void bfin_spi_u32_write(struct bfin_spi_master *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->tx < drv_data->tx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u32 *)drv_data->tx));
		drv_data->tx += 4;
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		bfin_read(&drv_data->regs->rfifo);
	}
}

static void bfin_spi_u32_read(struct bfin_spi_master *drv_data)
{
	u32 tx_val = drv_data->cur_chip->tx_dummy_val;

	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, tx_val);
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)drv_data->rx = bfin_read(&drv_data->regs->rfifo);
		drv_data->rx += 4;
	}
}

static void bfin_spi_u32_duplex(struct bfin_spi_master *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u32 *)drv_data->tx));
		drv_data->tx += 4;
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)drv_data->rx = bfin_read(&drv_data->regs->rfifo);
		drv_data->rx += 4;
	}
}

static const struct bfin_spi_transfer_ops bfin_bfin_spi_transfer_ops_u32 = {
	.write  = bfin_spi_u32_write,
	.read   = bfin_spi_u32_read,
	.duplex = bfin_spi_u32_duplex,
};


/* test if there is more transfer to be done */
static void *bfin_spi_next_transfer(struct bfin_spi_master *drv_data)
{
	struct spi_message *msg = drv_data->cur_msg;
	struct spi_transfer *trans = drv_data->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
		drv_data->cur_transfer =
		    list_entry(trans->transfer_list.next,
			       struct spi_transfer, transfer_list);
		return RUNNING_STATE;
	} else
		return DONE_STATE;
}

static void bfin_spi_giveback(struct bfin_spi_master *drv_data)
{
	struct bfin_spi_device *chip = drv_data->cur_chip;

	if (!drv_data->cs_change)
		bfin_spi_cs_deactive(drv_data, chip);

	spi_finalize_current_message(drv_data->master);
}

static void bfin_spi_pump_transfers(unsigned long data)
{
	struct bfin_spi_master *drv_data = (struct bfin_spi_master *)data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct bfin_spi_device *chip = NULL;
	unsigned int bits_per_word;
	u32 cr, cr_width;
	bool tranf_success = true;
	bool full_duplex = false;

	/* Get current state information */
	message = drv_data->cur_msg;
	transfer = drv_data->cur_transfer;
	chip = drv_data->cur_chip;

	/* Handle for abort */
	if (message->state == ERROR_STATE) {
		message->status = -EIO;
		bfin_spi_giveback(drv_data);
		return;
	}

	/* Handle end of message */
	if (message->state == DONE_STATE) {
		message->status = 0;
		bfin_spi_flush(drv_data);
		bfin_spi_giveback(drv_data);
		return;
	}

	/* Delay if requested at end of transfer */
	if (message->state == RUNNING_STATE) {
		previous = list_entry(transfer->transfer_list.prev,
				      struct spi_transfer, transfer_list);
		if (previous->delay_usecs)
			udelay(previous->delay_usecs);
	}

	/* Flush any existing transfers that may be sitting in the hardware */
	if (bfin_spi_flush(drv_data) == 0) {
		message->status = -EIO;
		bfin_spi_giveback(drv_data);
		return;
	}

	if ((transfer->len == 0) || (transfer->tx_buf == NULL
				&& transfer->rx_buf == NULL)) {
		/* Move to next transfer of this msg */
		message->state = bfin_spi_next_transfer(drv_data);
		/* Schedule next transfer tasklet */
		tasklet_schedule(&drv_data->pump_transfers);
		return;
	}

	if (transfer->tx_buf != NULL) {
		drv_data->tx = (void *)transfer->tx_buf;
		drv_data->tx_end = drv_data->tx + transfer->len;
	} else {
		drv_data->tx = NULL;
	}

	if (transfer->rx_buf != NULL) {
		full_duplex = (transfer->tx_buf != NULL) ? true : false;
		drv_data->rx = transfer->rx_buf;
		drv_data->rx_end = drv_data->rx + transfer->len;
	} else {
		drv_data->rx = NULL;
	}

	drv_data->transfer_len = transfer->len;
	drv_data->cs_change = transfer->cs_change;

	/* Bits per word setup */
	bits_per_word = transfer->bits_per_word ? :
		message->spi->bits_per_word ? : 8;
	switch (bits_per_word) {
	case 8:
		cr_width = SPI_CTL_SIZE08;
		drv_data->ops = &bfin_bfin_spi_transfer_ops_u8;
		break;
	case 16:
		cr_width = SPI_CTL_SIZE16;
		drv_data->ops = &bfin_bfin_spi_transfer_ops_u16;
		break;
	case 32:
		cr_width = SPI_CTL_SIZE32;
		drv_data->ops = &bfin_bfin_spi_transfer_ops_u32;
		break;
	default:
		message->status = -EINVAL;
		bfin_spi_giveback(drv_data);
		return;
	}
	cr = bfin_read(&drv_data->regs->control) & ~SPI_CTL_SIZE;
	cr |= cr_width;
	bfin_write(&drv_data->regs->control, cr);

	message->state = RUNNING_STATE;

	/* Speed setup (surely valid because already checked) */
	if (transfer->speed_hz)
		bfin_write(&drv_data->regs->clock, hz_to_spi_clock(transfer->speed_hz));
	else
		bfin_write(&drv_data->regs->clock, chip->clock);

	bfin_write(&drv_data->regs->status, 0xFFFFFFFF);
	bfin_spi_cs_active(drv_data, chip);

	if (chip->enable_dma) {
		u32 dma_config;
		unsigned long word_count, word_size;
		void *tx_buf, *rx_buf;

		switch (bits_per_word) {
		case 8:
			dma_config = WDSIZE_8 | PSIZE_8;
			word_count = drv_data->transfer_len;
			word_size = 1;
			break;
		case 16:
			dma_config = WDSIZE_16 | PSIZE_16;
			word_count = drv_data->transfer_len / 2;
			word_size = 2;
			break;
		default:
			dma_config = WDSIZE_32 | PSIZE_32;
			word_count = drv_data->transfer_len / 4;
			word_size = 4;
			break;
		}

		if (full_duplex) {
			WARN_ON((drv_data->tx_end - drv_data->tx)
					!= (drv_data->rx_end - drv_data->rx));
			tx_buf = drv_data->tx;
			rx_buf = drv_data->rx;
			drv_data->tx_dma_size = drv_data->rx_dma_size
						= drv_data->transfer_len;
			set_dma_x_modify(drv_data->tx_dma, word_size);
			set_dma_x_modify(drv_data->rx_dma, word_size);
		} else if (drv_data->tx) {
			tx_buf = drv_data->tx;
			rx_buf = &drv_data->dummy_buffer;
			drv_data->tx_dma_size = drv_data->transfer_len;
			drv_data->rx_dma_size = sizeof(drv_data->dummy_buffer);
			set_dma_x_modify(drv_data->tx_dma, word_size);
			set_dma_x_modify(drv_data->rx_dma, 0);
		} else {
			drv_data->dummy_buffer = chip->tx_dummy_val;
			tx_buf = &drv_data->dummy_buffer;
			rx_buf = drv_data->rx;
			drv_data->tx_dma_size = sizeof(drv_data->dummy_buffer);
			drv_data->rx_dma_size = drv_data->transfer_len;
			set_dma_x_modify(drv_data->tx_dma, 0);
			set_dma_x_modify(drv_data->rx_dma, word_size);
		}

		drv_data->tx_dma_addr = dma_map_single(&message->spi->dev,
					(void *)tx_buf,
					drv_data->tx_dma_size,
					DMA_TO_DEVICE);
		if (dma_mapping_error(&message->spi->dev,
					drv_data->tx_dma_addr)) {
			message->state = ERROR_STATE;
			return;
		}

		drv_data->rx_dma_addr = dma_map_single(&message->spi->dev,
					(void *)rx_buf,
					drv_data->rx_dma_size,
					DMA_FROM_DEVICE);
		if (dma_mapping_error(&message->spi->dev,
					drv_data->rx_dma_addr)) {
			message->state = ERROR_STATE;
			dma_unmap_single(&message->spi->dev,
					drv_data->tx_dma_addr,
					drv_data->tx_dma_size,
					DMA_TO_DEVICE);
			return;
		}

		dummy_read(drv_data);
		set_dma_x_count(drv_data->tx_dma, word_count);
		set_dma_x_count(drv_data->rx_dma, word_count);
		set_dma_start_addr(drv_data->tx_dma, drv_data->tx_dma_addr);
		set_dma_start_addr(drv_data->rx_dma, drv_data->rx_dma_addr);
		dma_config |= DMAFLOW_STOP | RESTART | DI_EN;
		set_dma_config(drv_data->tx_dma, dma_config);
		set_dma_config(drv_data->rx_dma, dma_config | WNR);
		enable_dma(drv_data->tx_dma);
		enable_dma(drv_data->rx_dma);
		SSYNC();

		bfin_write(&drv_data->regs->rx_control, SPI_RXCTL_REN | SPI_RXCTL_RDR_NE);
		SSYNC();
		bfin_write(&drv_data->regs->tx_control,
				SPI_TXCTL_TEN | SPI_TXCTL_TTI | SPI_TXCTL_TDR_NF);

		return;
	}

	if (full_duplex) {
		/* full duplex mode */
		WARN_ON((drv_data->tx_end - drv_data->tx)
				!= (drv_data->rx_end - drv_data->rx));

		drv_data->ops->duplex(drv_data);

		if (drv_data->tx != drv_data->tx_end)
			tranf_success = false;
	} else if (drv_data->tx != NULL) {
		/* write only half duplex */
		drv_data->ops->write(drv_data);

		if (drv_data->tx != drv_data->tx_end)
			tranf_success = false;
	} else {
		/* read only half duplex */
		drv_data->ops->read(drv_data);
		if (drv_data->rx != drv_data->rx_end)
			tranf_success = false;
	}

	if (!tranf_success) {
		message->state = ERROR_STATE;
	} else {
		/* Update total byte transferred */
		message->actual_length += drv_data->transfer_len;
		/* Move to next transfer of this msg */
		message->state = bfin_spi_next_transfer(drv_data);
		if (drv_data->cs_change && message->state != DONE_STATE) {
			bfin_spi_flush(drv_data);
			bfin_spi_cs_deactive(drv_data, chip);
		}
	}

	/* Schedule next transfer tasklet */
	tasklet_schedule(&drv_data->pump_transfers);
}

static int bfin_spi_transfer_one_message(struct spi_master *master,
					struct spi_message *m)
{
	struct bfin_spi_master *drv_data = spi_master_get_devdata(master);

	drv_data->cur_msg = m;
	drv_data->cur_chip = spi_get_ctldata(drv_data->cur_msg->spi);
	bfin_spi_restore_state(drv_data);

	drv_data->cur_msg->state = START_STATE;
	drv_data->cur_transfer = list_entry(drv_data->cur_msg->transfers.next,
					    struct spi_transfer, transfer_list);

	tasklet_schedule(&drv_data->pump_transfers);
	return 0;
}

#define MAX_SPI_SSEL	7

static const u16 ssel[][MAX_SPI_SSEL] = {
	{P_SPI0_SSEL1, P_SPI0_SSEL2, P_SPI0_SSEL3,
	P_SPI0_SSEL4, P_SPI0_SSEL5,
	P_SPI0_SSEL6, P_SPI0_SSEL7},

	{P_SPI1_SSEL1, P_SPI1_SSEL2, P_SPI1_SSEL3,
	P_SPI1_SSEL4, P_SPI1_SSEL5,
	P_SPI1_SSEL6, P_SPI1_SSEL7},

	{P_SPI2_SSEL1, P_SPI2_SSEL2, P_SPI2_SSEL3,
	P_SPI2_SSEL4, P_SPI2_SSEL5,
	P_SPI2_SSEL6, P_SPI2_SSEL7},
};

static int bfin_spi_setup(struct spi_device *spi)
{
	struct bfin_spi_master *drv_data = spi_master_get_devdata(spi->master);
	struct bfin_spi_device *chip = spi_get_ctldata(spi);
	u32 bfin_ctl_reg = SPI_CTL_ODM | SPI_CTL_PSSE;
	int ret = -EINVAL;

	if (spi->bits_per_word != 8
		&& spi->bits_per_word != 16
		&& spi->bits_per_word != 32) {
		dev_err(&spi->dev, "%d bits_per_word is not supported\n",
				spi->bits_per_word);
		return -EINVAL;
	}

	if (!chip) {
		struct bfin6xx_spi_chip *chip_info = spi->controller_data;

		chip = kzalloc(sizeof(*chip), GFP_KERNEL);
		if (!chip) {
			dev_err(&spi->dev, "can not allocate chip data\n");
			return -ENOMEM;
		}
		if (chip_info) {
			if (chip_info->control & ~bfin_ctl_reg) {
				dev_err(&spi->dev, "do not set bits "
					"that the SPI framework manages\n");
				goto error;
			}
			chip->control = chip_info->control;
			chip->cs_chg_udelay = chip_info->cs_chg_udelay;
			chip->tx_dummy_val = chip_info->tx_dummy_val;
			chip->enable_dma = chip_info->enable_dma;
		}
		chip->cs = spi->chip_select;
		if (chip->cs < MAX_CTRL_CS) {
			chip->ssel = (1 << chip->cs) << 8;
			ret = peripheral_request(ssel[spi->master->bus_num]
					[chip->cs-1], dev_name(&spi->dev));
			if (ret) {
				dev_err(&spi->dev, "peripheral_request() error\n");
				goto error;
			}
		} else {
			chip->cs_gpio = chip->cs - MAX_CTRL_CS;
			ret = gpio_request_one(chip->cs_gpio, GPIOF_OUT_INIT_HIGH,
						dev_name(&spi->dev));
			if (ret) {
				dev_err(&spi->dev, "gpio_request_one() error\n");
				goto error;
			}
		}
		spi_set_ctldata(spi, chip);
	}

	/* force a default base state */
	chip->control &= bfin_ctl_reg;

	/* translate common spi framework into our register */
	if (spi->mode & ~(SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST)) {
		dev_err(&spi->dev, "unsupported spi modes detected\n");
		goto pin_error;
	}
	if (spi->mode & SPI_CPOL)
		chip->control |= SPI_CTL_CPOL;
	if (spi->mode & SPI_CPHA)
		chip->control |= SPI_CTL_CPHA;
	if (spi->mode & SPI_LSB_FIRST)
		chip->control |= SPI_CTL_LSBF;
	chip->control |= SPI_CTL_MSTR;
	/* we choose software to controll cs */
	chip->control &= ~SPI_CTL_ASSEL;

	chip->clock = hz_to_spi_clock(spi->max_speed_hz);

	bfin_spi_cs_enable(drv_data, chip);
	bfin_spi_cs_deactive(drv_data, chip);

	return 0;
pin_error:
	if (chip->cs < MAX_CTRL_CS)
		peripheral_free(ssel[spi->master->bus_num][chip->cs - 1]);
	else
		gpio_free(chip->cs_gpio);
error:
	if (chip) {
		kfree(chip);
		spi_set_ctldata(spi, NULL);
	}

	return ret;
}

static void bfin_spi_cleanup(struct spi_device *spi)
{
	struct bfin_spi_device *chip = spi_get_ctldata(spi);
	struct bfin_spi_master *drv_data = spi_master_get_devdata(spi->master);

	if (!chip)
		return;

	if (chip->cs < MAX_CTRL_CS) {
		peripheral_free(ssel[spi->master->bus_num]
					[chip->cs-1]);
		bfin_spi_cs_disable(drv_data, chip);
	} else
		gpio_free(chip->cs_gpio);

	kfree(chip);
	spi_set_ctldata(spi, NULL);
}

static irqreturn_t bfin_spi_tx_dma_isr(int irq, void *dev_id)
{
	struct bfin_spi_master *drv_data = dev_id;
	u32 dma_stat = get_dma_curr_irqstat(drv_data->tx_dma);

	clear_dma_irqstat(drv_data->tx_dma);
	if (dma_stat & DMA_DONE)
		drv_data->tx_num++;
	if (dma_stat & DMA_ERR)
		dev_err(&drv_data->master->dev,
				"spi tx dma error: %d\n", dma_stat);
	bfin_write_and(&drv_data->regs->tx_control, ~SPI_TXCTL_TDR_NF);
	return IRQ_HANDLED;
}

static irqreturn_t bfin_spi_rx_dma_isr(int irq, void *dev_id)
{
	struct bfin_spi_master *drv_data = dev_id;
	struct bfin_spi_device *chip = drv_data->cur_chip;
	struct spi_message *msg = drv_data->cur_msg;
	u32 dma_stat = get_dma_curr_irqstat(drv_data->rx_dma);

	clear_dma_irqstat(drv_data->rx_dma);
	if (dma_stat & DMA_DONE)
		drv_data->rx_num++;
	if (dma_stat & DMA_ERR) {
		msg->state = ERROR_STATE;
		dev_err(&drv_data->master->dev,
				"spi rx dma error: %d\n", dma_stat);
	} else {
		msg->actual_length += drv_data->transfer_len;
		if (drv_data->cs_change)
			bfin_spi_cs_deactive(drv_data, chip);
		msg->state = bfin_spi_next_transfer(drv_data);
	}
	bfin_write(&drv_data->regs->tx_control, 0);
	bfin_write(&drv_data->regs->rx_control, 0);
	if (drv_data->rx_num != drv_data->tx_num)
		dev_dbg(&drv_data->master->dev,
				"dma interrupt missing: tx=%d,rx=%d\n",
				drv_data->tx_num, drv_data->rx_num);
	tasklet_schedule(&drv_data->pump_transfers);
	return IRQ_HANDLED;
}

static int bfin_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bfin6xx_spi_master *info = dev->platform_data;
	struct spi_master *master;
	struct bfin_spi_master *drv_data;
	struct resource *mem, *res;
	unsigned int tx_dma, rx_dma;
	int ret;

	if (!info) {
		dev_err(dev, "platform data missing!\n");
		return -ENODEV;
	}

	/* get register base and tx/rx dma */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "can not get register base\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!res) {
		dev_err(dev, "can not get tx dma resource\n");
		return -ENXIO;
	}
	tx_dma = res->start;

	res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (!res) {
		dev_err(dev, "can not get rx dma resource\n");
		return -ENXIO;
	}
	rx_dma = res->start;

	/* allocate master with space for drv_data */
	master = spi_alloc_master(dev, sizeof(*drv_data));
	if (!master) {
		dev_err(dev, "can not alloc spi_master\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, master);

	/* the mode bits supported by this driver */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;

	master->bus_num = pdev->id;
	master->num_chipselect = info->num_chipselect;
	master->cleanup = bfin_spi_cleanup;
	master->setup = bfin_spi_setup;
	master->transfer_one_message = bfin_spi_transfer_one_message;

	drv_data = spi_master_get_devdata(master);
	drv_data->master = master;
	drv_data->tx_dma = tx_dma;
	drv_data->rx_dma = rx_dma;
	drv_data->pin_req = info->pin_req;

	drv_data->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(drv_data->regs)) {
		dev_err(dev, "can not map register memory\n");
		ret = PTR_ERR(drv_data->regs);
		goto err_put_master;
	}

	/* request tx and rx dma */
	ret = request_dma(tx_dma, "SPI_TX_DMA");
	if (ret) {
		dev_err(dev, "can not request SPI TX DMA channel\n");
		goto err_put_master;
	}
	set_dma_callback(tx_dma, bfin_spi_tx_dma_isr, drv_data);

	ret = request_dma(rx_dma, "SPI_RX_DMA");
	if (ret) {
		dev_err(dev, "can not request SPI RX DMA channel\n");
		goto err_free_tx_dma;
	}
	set_dma_callback(drv_data->rx_dma, bfin_spi_rx_dma_isr, drv_data);

	/* request CLK, MOSI and MISO */
	ret = peripheral_request_list(drv_data->pin_req, "bfin-spi");
	if (ret < 0) {
		dev_err(dev, "can not request spi pins\n");
		goto err_free_rx_dma;
	}

	bfin_write(&drv_data->regs->control, SPI_CTL_MSTR | SPI_CTL_CPHA);
	bfin_write(&drv_data->regs->ssel, 0x0000FE00);
	bfin_write(&drv_data->regs->delay, 0x0);

	tasklet_init(&drv_data->pump_transfers,
			bfin_spi_pump_transfers, (unsigned long)drv_data);
	/* register with the SPI framework */
	ret = spi_register_master(master);
	if (ret) {
		dev_err(dev, "can not  register spi master\n");
		goto err_free_peripheral;
	}

	dev_info(dev, "bfin-spi probe success\n");
	return ret;

err_free_peripheral:
	peripheral_free_list(drv_data->pin_req);
err_free_rx_dma:
	free_dma(rx_dma);
err_free_tx_dma:
	free_dma(tx_dma);
err_put_master:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return ret;
}

static int bfin_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct bfin_spi_master *drv_data = spi_master_get_devdata(master);

	bfin_spi_disable(drv_data);

	peripheral_free_list(drv_data->pin_req);
	free_dma(drv_data->rx_dma);
	free_dma(drv_data->tx_dma);

	platform_set_drvdata(pdev, NULL);
	spi_unregister_master(drv_data->master);
	return 0;
}

#ifdef CONFIG_PM
static int bfin_spi_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct bfin_spi_master *drv_data = spi_master_get_devdata(master);

	spi_master_suspend(master);

	drv_data->control = bfin_read(&drv_data->regs->control);
	drv_data->ssel = bfin_read(&drv_data->regs->ssel);

	bfin_write(&drv_data->regs->control, SPI_CTL_MSTR | SPI_CTL_CPHA);
	bfin_write(&drv_data->regs->ssel, 0x0000FE00);
	free_dma(drv_data->rx_dma);
	free_dma(drv_data->tx_dma);

	return 0;
}

static int bfin_spi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct bfin_spi_master *drv_data = spi_master_get_devdata(master);
	int ret = 0;

	ret = request_dma(drv_data->tx_dma, "SPI_TX_DMA");
	if (ret) {
		dev_err(dev, "cannot request SPI TX DMA channel\n");
		return ret;
	}
	set_dma_callback(drv_data->tx_dma, bfin_spi_tx_dma_isr, drv_data);

	ret = request_dma(drv_data->rx_dma, "SPI_RX_DMA");
	if (ret) {
		dev_err(dev, "cannot request SPI RX DMA channel\n");
		free_dma(drv_data->tx_dma);
		return ret;
	}
	/* rx dma is enabled when resume in spi boot mode */
	disable_dma(drv_data->rx_dma);
	set_dma_callback(drv_data->rx_dma, bfin_spi_rx_dma_isr, drv_data);

	bfin_write(&drv_data->regs->control, drv_data->control);
	bfin_write(&drv_data->regs->ssel, drv_data->ssel);

	ret = spi_master_resume(master);
	if (ret) {
		free_dma(drv_data->rx_dma);
		free_dma(drv_data->tx_dma);
	}

	return ret;
}
static const struct dev_pm_ops bfin_spi_pm_ops = {
	.suspend = bfin_spi_suspend,
	.resume  = bfin_spi_resume,
};
#endif /* CONFIG_PM */

MODULE_ALIAS("platform:bfin-spi");
static struct platform_driver bfin_spi_driver = {
	.driver	= {
		.name	= "bfin-spi",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm     = &bfin_spi_pm_ops,
#endif
	},
	.remove		= bfin_spi_remove,
};

static int __init bfin_spi_init(void)
{
	return platform_driver_probe(&bfin_spi_driver, bfin_spi_probe);
}
subsys_initcall(bfin_spi_init);

static void __exit bfin_spi_exit(void)
{
	platform_driver_unregister(&bfin_spi_driver);
}
module_exit(bfin_spi_exit);

MODULE_DESCRIPTION("Analog Devices SPI3 controller driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
