/* bfin_rtdm_i2c.c --- 
 * 
 * Filename: bfin_rtdm_i2c.c
 * Description: 
 * Author: Devin Butterfield
 * Maintainer: 
 * Created: Fri Feb  7 22:02:41 2014 (-0800)
 * Version: 
 * Last-Updated: Fri Mar 21 14:56:32 2014 (-0700)
 *           By: Devin Butterfield
 *     Update #: 225
 * URL: 
 * Keywords: 
 * Compatibility: 
 * 
 */

/* Commentary: 
 * 
 * RTDM driver based on original i2c-bfin-twi driver, but allows
 * accessing the TWI/I2C device on the ADI Blackfin in hard real-time
 * context.
 *
 * Much of this code derived from the i2c-bfin-twi Linux driver
 * copyright, Analog Devices.
 *
 * Note: this driver provides an RTDM compatible interface to the I2C
 * device; it is not a "proper" linux I2C driver.
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

#include <linux/module.h>
#include <linux/platform_device.h>

#include <rtdm/rtdm_driver.h>

#include <asm/portmux.h>
#include <asm/bfin_twi.h>
#include <asm/bfin_rtdm_i2c.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Devin Butterfield <db@lumenosys.com>");

#define DEVICE_NAME			"bfin_rtdm_i2c"
#define SOME_SUB_CLASS			4712

/* SMBus mode*/
#define TWI_I2C_MODE_STANDARD	    1
#define TWI_I2C_MODE_STANDARDSUB    2
#define TWI_I2C_MODE_COMBINED	    3
#define TWI_I2C_MODE_REPEAT	    4

struct bfin_rtdm_i2c_dev {
	rtdm_event_t wc_evt;		    /* wait for completion event */
	struct bfin_twi_iface *iface;
	struct rtdm_device rtdm_dev;	/* RTDM device structure */
	int timeout;
	int retries;
};

struct bfin_rtdm_i2c_context {
	rtdm_irq_t irq_handle;		/* device IRQ handle */
	rtdm_lock_t lock;             /* lock to protect context struct */
	unsigned short addr;          /* chip address */
	unsigned int flags;           /* ??? */
	struct bfin_rtdm_i2c_dev *dev; /* pointer to our device struct */
};

/*
 * Wait for master transfer to complete.
 * It puts current process to sleep until we get interrupt or timeout expires.
 * Returns the number of transferred bytes or error (<0)
 */
static int bfin_rtdm_i2c_wait_for_completion(struct bfin_rtdm_i2c_dev *dev)
{
	/* make sure we are in RT context -- otherwise system will
	 * HANG if we use the RTDM call! */
	if (rtdm_in_rt_context()) {
		return rtdm_event_timedwait (&dev->wc_evt, 
					     dev->timeout * 1000000, 0);
	} else {
		if (!wait_for_completion_timeout(&dev->iface->complete, 
						 dev->timeout*HZ)) {
			return -1;
		} else {
			return 0;
		}
	}
}


static void bfin_twi_handle_interrupt(struct bfin_rtdm_i2c_dev *dev,
                                      unsigned short twi_int_status)
{
	unsigned short mast_stat;
	struct bfin_twi_iface *iface = dev->iface;
	
	mast_stat = read_MASTER_STAT(iface);
	
	if (twi_int_status & XMTSERV) {
		/* Transmit next data */
		if (iface->writeNum > 0) {
			SSYNC();
			write_XMT_DATA8(iface, *(iface->transPtr++));
			iface->writeNum--;
		}
		/* start receive immediately after complete sending in
		 * combine mode.
		 */
		else if (iface->cur_mode == TWI_I2C_MODE_COMBINED)
			write_MASTER_CTL(iface,
					 read_MASTER_CTL(iface) | MDIR);
		else if (iface->manual_stop)
			write_MASTER_CTL(iface,
					 read_MASTER_CTL(iface) | STOP);
		else if (iface->cur_mode == TWI_I2C_MODE_REPEAT &&
			 iface->cur_msg + 1 < iface->msg_num) {
			if (iface->pmsg[iface->cur_msg + 1].flags & I2C_M_RD)
				write_MASTER_CTL(iface,
						 read_MASTER_CTL(iface) | MDIR);
			else
				write_MASTER_CTL(iface,
						 read_MASTER_CTL(iface) & ~MDIR);
		}
	}
	if (twi_int_status & RCVSERV) {
		if (iface->readNum > 0) {
			/* Receive next data */
			*(iface->transPtr) = read_RCV_DATA8(iface);
			if (iface->cur_mode == TWI_I2C_MODE_COMBINED) {
				/* Change combine mode into sub mode after
				 * read first data.
				 */
				iface->cur_mode = TWI_I2C_MODE_STANDARDSUB;
				/* Get read number from first byte in block
				 * combine mode.
				 */
				if (iface->readNum == 1 && iface->manual_stop)
					iface->readNum = *iface->transPtr + 1;
			}
			iface->transPtr++;
			iface->readNum--;
		}
		
		if (iface->readNum == 0) {
			if (iface->manual_stop) {
				/* Temporary workaround to avoid possible bus stall -
				 * Flush FIFO before issuing the STOP condition
				 */
				read_RCV_DATA16(iface);
				write_MASTER_CTL(iface,
						 read_MASTER_CTL(iface) | STOP);
			} else if (iface->cur_mode == TWI_I2C_MODE_REPEAT &&
				   iface->cur_msg + 1 < iface->msg_num) {
				if (iface->pmsg[iface->cur_msg + 1].flags & I2C_M_RD)
					write_MASTER_CTL(iface,
							 read_MASTER_CTL(iface) | MDIR);
				else
					write_MASTER_CTL(iface,
							 read_MASTER_CTL(iface) & ~MDIR);
			}
		}
	}
	if (twi_int_status & MERR) {
		write_INT_MASK(iface, 0);
		write_MASTER_STAT(iface, 0x3e);
		write_MASTER_CTL(iface, 0);
		iface->result = -EIO;
		
		if (mast_stat & LOSTARB)
			rtdm_printk(DEVICE_NAME ": Lost Arbitration\n");
		if (mast_stat & ANAK)
			rtdm_printk(DEVICE_NAME ": Address Not Acknowledged\n");
		if (mast_stat & DNAK)
			rtdm_printk(DEVICE_NAME ": Data Not Acknowledged\n");
		if (mast_stat & BUFRDERR)
			rtdm_printk(DEVICE_NAME ": Buffer Read Error\n");
		if (mast_stat & BUFWRERR)
			rtdm_printk(DEVICE_NAME ": Buffer Write Error\n");
		
		/* Faulty slave devices, may drive SDA low after a transfer
		 * finishes. To release the bus this code generates up to 9
		 * extra clocks until SDA is released.
		 */
		
		if (read_MASTER_STAT(iface) & SDASEN) {
			int cnt = 9;
			do {
				write_MASTER_CTL(iface, SCLOVR);
				rtdm_task_busy_sleep(6000);
				/* udelay(6); */
				write_MASTER_CTL(iface, 0);
				rtdm_task_busy_sleep(6000);
				/* udelay(6); */
			} while ((read_MASTER_STAT(iface) & SDASEN) && cnt--);

			write_MASTER_CTL(iface, SDAOVR | SCLOVR);
			rtdm_task_busy_sleep(6000);
			/* udelay(6); */
			write_MASTER_CTL(iface, SDAOVR);
			rtdm_task_busy_sleep(6000);
			/* udelay(6); */
			write_MASTER_CTL(iface, 0);
		}

		/* If it is a quick transfer, only address without data,
		 * not an err, return 1.
		 */
		if (iface->cur_mode == TWI_I2C_MODE_STANDARD &&
		    iface->transPtr == NULL &&
		    (twi_int_status & MCOMP) && (mast_stat & DNAK))
			iface->result = 1;

		complete(&iface->complete);
 		/* if (!rtdm_in_rt_context()) { */
		/* 	printk(DEVICE_NAME ": Oops: left RT context!!!\n"); */
		/* } */
		rtdm_event_signal(&dev->wc_evt);
		return;
	}
	if (twi_int_status & MCOMP) {
		if (twi_int_status & (XMTSERV|RCVSERV) &&
		    (read_MASTER_CTL(iface) & MEN) == 0 &&
		    (iface->cur_mode == TWI_I2C_MODE_REPEAT ||
		     iface->cur_mode == TWI_I2C_MODE_COMBINED)) {
			iface->result = -1;
			write_INT_MASK(iface, 0);
			write_MASTER_CTL(iface, 0);
		} else if (iface->cur_mode == TWI_I2C_MODE_COMBINED) {
			if (iface->readNum == 0) {
				/* set the read number to 1 and ask for manual
				 * stop in block combine mode
				 */
				iface->readNum = 1;
				iface->manual_stop = 1;
				write_MASTER_CTL(iface,
						 read_MASTER_CTL(iface) | (0xff << 6));
			} else {
				/* set the readd number in other
				 * combine mode.
				 */
				write_MASTER_CTL(iface,
						 (read_MASTER_CTL(iface) &
						  (~(0xff << 6))) |
						 (iface->readNum << 6));
			}
			/* remove restart bit and enable master receive */
			write_MASTER_CTL(iface,
					 read_MASTER_CTL(iface) & ~RSTART);
		} else if (iface->cur_mode == TWI_I2C_MODE_REPEAT &&
			   iface->cur_msg+1 < iface->msg_num) {
			iface->cur_msg++;
			iface->transPtr = iface->pmsg[iface->cur_msg].buf;
			iface->writeNum = iface->readNum =
				iface->pmsg[iface->cur_msg].len;
			/* Set Transmit device address */
			write_MASTER_ADDR(iface,
					  iface->pmsg[iface->cur_msg].addr);
			if (iface->pmsg[iface->cur_msg].flags & I2C_M_RD)
				iface->read_write = I2C_SMBUS_READ;
			else {
				iface->read_write = I2C_SMBUS_WRITE;
				/* Transmit first data */
				if (iface->writeNum > 0) {
					write_XMT_DATA8(iface,
							*(iface->transPtr++));
					iface->writeNum--;
				}
			}

			if (iface->pmsg[iface->cur_msg].len <= 255) {
				write_MASTER_CTL(iface,
						 (read_MASTER_CTL(iface) &
						  (~(0xff << 6))) |
						 (iface->pmsg[iface->cur_msg].len << 6));
				iface->manual_stop = 0;
			} else {
				write_MASTER_CTL(iface,
						 (read_MASTER_CTL(iface) |
						  (0xff << 6)));
				iface->manual_stop = 1;
			}
			/* remove restart bit before last message */
			if (iface->cur_msg+1 == iface->msg_num)
				write_MASTER_CTL(iface,
						 read_MASTER_CTL(iface) & ~RSTART);
		} else {
			iface->result = 1;
			write_INT_MASK(iface, 0);
			write_MASTER_CTL(iface, 0);
		}
		complete(&iface->complete);
		/* if (!rtdm_in_rt_context()) { */
		/* 	printk(DEVICE_NAME ": Oops: left RT context!!!\n"); */
		/* } */
    
		rtdm_event_signal(&dev->wc_evt);
	}
}

/*
 * I2C interrupt handler
 */
static int bfin_rtdm_i2c_interrupt(rtdm_irq_t *irq_context)
{
	struct bfin_rtdm_i2c_context *ctx;
	/* unsigned long flags; */
	unsigned short twi_int_status;

	/* rtdm_printk(DEVICE_NAME ": got interrupt!\n"); */
  
	ctx = rtdm_irq_get_arg(irq_context, struct bfin_rtdm_i2c_context);
	rtdm_lock_get(&ctx->lock);

	while (1) {
		twi_int_status = read_INT_STAT(ctx->dev->iface);
		if (!twi_int_status)
			break;
		/* Clear interrupt status */
		write_INT_STAT(ctx->dev->iface, twi_int_status);
		bfin_twi_handle_interrupt(ctx->dev, twi_int_status);
		SSYNC();
	}
  
	rtdm_lock_put(&ctx->lock);
  
	return RTDM_IRQ_HANDLED;
}

/*
 * One i2c master transfer
 */
static int bfin_twi_do_master_xfer(struct bfin_rtdm_i2c_dev *dev,
				   struct i2c_msg *msgs, int num)
{
	struct bfin_twi_iface *iface = dev->iface;
	struct i2c_msg *pmsg;
	int rc = 0;

	if (!(read_CONTROL(iface) & TWI_ENA))
		return -ENXIO;

	if (read_MASTER_STAT(iface) & BUSBUSY)
		return -EAGAIN;

	iface->pmsg = msgs;
	iface->msg_num = num;
	iface->cur_msg = 0;

	pmsg = &msgs[0];
	if (pmsg->flags & I2C_M_TEN) {
		rtdm_printk(DEVICE_NAME "10 bits addr not supported!\n");
		return -EINVAL;
	}

	if (iface->msg_num > 1)
		iface->cur_mode = TWI_I2C_MODE_REPEAT;
	iface->manual_stop = 0;
	iface->transPtr = pmsg->buf;
	iface->writeNum = iface->readNum = pmsg->len;
	iface->result = 0;
    
	init_completion(&(iface->complete));

	/* if (!rtdm_in_rt_context()) { */
	/* 	printk(DEVICE_NAME ": Oops: left RT context!!!\n"); */
	/* } */

	rtdm_event_init(&dev->wc_evt, 0);
    
	/* Set Transmit device address */
	write_MASTER_ADDR(iface, pmsg->addr);

	/* FIFO Initiation. Data in FIFO should be
	 *  discarded before start a new operation.
	 */
	write_FIFO_CTL(iface, 0x3);
	SSYNC();
	write_FIFO_CTL(iface, 0);
	SSYNC();

	if (pmsg->flags & I2C_M_RD)
		iface->read_write = I2C_SMBUS_READ;
	else {
		iface->read_write = I2C_SMBUS_WRITE;
		/* Transmit first data */
		if (iface->writeNum > 0) {
			write_XMT_DATA8(iface, *(iface->transPtr++));
			iface->writeNum--;
			SSYNC();
		}
	}

	/* clear int stat */
	write_INT_STAT(iface, MERR | MCOMP | XMTSERV | RCVSERV);

	/* Interrupt mask . Enable XMT, RCV interrupt */
	write_INT_MASK(iface, MCOMP | MERR | RCVSERV | XMTSERV);
	SSYNC();

	if (pmsg->len <= 255)
		write_MASTER_CTL(iface, pmsg->len << 6);
	else {
		write_MASTER_CTL(iface, 0xff << 6);
		iface->manual_stop = 1;
	}

	/* Master enable */
	write_MASTER_CTL(iface, read_MASTER_CTL(iface) | MEN |
			 (iface->msg_num > 1 ? RSTART : 0) |
			 ((iface->read_write == I2C_SMBUS_READ) ? MDIR : 0) |
			 ((CONFIG_BFIN_RTDM_I2C_CLK_KHZ > 100) ? FAST : 0));
	SSYNC();

	while (!iface->result) {
		if (bfin_rtdm_i2c_wait_for_completion(dev)) {
			iface->result = -1;
			rtdm_printk(DEVICE_NAME ": master transfer timeout\n");
		}
	}

	if (iface->result == 1)
		rc = iface->cur_msg + 1;
	else
		rc = iface->result;

	return rc;
}

/*
 * Generic i2c master transfer entrypoint
 */
static int bfin_twi_master_xfer(struct bfin_rtdm_i2c_dev *dev,
				struct i2c_msg *msgs, int num)
{
	return bfin_twi_do_master_xfer(dev, msgs, num);
}

/*
 * One I2C SMBus transfer
 */
int bfin_twi_do_smbus_xfer(struct bfin_rtdm_i2c_dev *dev, u16 addr,
			   unsigned short flags, char read_write,
			   u8 command, int size, union i2c_smbus_data *data)
{
	struct bfin_twi_iface *iface = dev->iface;
	int rc = 0;

	/* rtdm_printk(DEVICE_NAME ": check TWI_ENA..."); */
	if (!(read_CONTROL(iface) & TWI_ENA))
		return -ENXIO;

	/* rtdm_printk("ok\n"); */
	/* rtdm_printk(DEVICE_NAME ": check BUSYBUSY..."); */
	if (read_MASTER_STAT(iface) & BUSBUSY)
		return -EAGAIN;
	/* rtdm_printk("ok\n"); */
    
	iface->writeNum = 0;
	iface->readNum = 0;

	/* Prepare datas & select mode */
	switch (size) {
	case I2C_SMBUS_QUICK:
		//      rtdm_printk(DEVICE_NAME ": QUICK\n");
		iface->transPtr = NULL;
		iface->cur_mode = TWI_I2C_MODE_STANDARD;
		break;
	case I2C_SMBUS_BYTE:
		//      rtdm_printk(DEVICE_NAME ": BYTE\n");
		if (data == NULL)
			iface->transPtr = NULL;
		else {
			if (read_write == I2C_SMBUS_READ)
				iface->readNum = 1;
			else
				iface->writeNum = 1;
			iface->transPtr = &data->byte;
		}
		iface->cur_mode = TWI_I2C_MODE_STANDARD;
		break;
	case I2C_SMBUS_BYTE_DATA:
		//      rtdm_printk(DEVICE_NAME ": BYTE_DATA\n");
		if (read_write == I2C_SMBUS_READ) {
			iface->readNum = 1;
			iface->cur_mode = TWI_I2C_MODE_COMBINED;
		} else {
			iface->writeNum = 1;
			iface->cur_mode = TWI_I2C_MODE_STANDARDSUB;
		}
		iface->transPtr = &data->byte;
		break;
	case I2C_SMBUS_WORD_DATA:
		//            rtdm_printk(DEVICE_NAME ": WORD_DATA\n");
		if (read_write == I2C_SMBUS_READ) {
			iface->readNum = 2;
			iface->cur_mode = TWI_I2C_MODE_COMBINED;
		} else {
			iface->writeNum = 2;
			iface->cur_mode = TWI_I2C_MODE_STANDARDSUB;
		}
		iface->transPtr = (u8 *)&data->word;
		break;
	case I2C_SMBUS_PROC_CALL:
		//            rtdm_printk(DEVICE_NAME ": PROC_CALL\n");
		iface->writeNum = 2;
		iface->readNum = 2;
		iface->cur_mode = TWI_I2C_MODE_COMBINED;
		iface->transPtr = (u8 *)&data->word;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		//            rtdm_printk(DEVICE_NAME ": BLOCK_DATA\n");
		if (read_write == I2C_SMBUS_READ) {
			iface->readNum = 0;
			iface->cur_mode = TWI_I2C_MODE_COMBINED;
		} else {
			iface->writeNum = data->block[0] + 1;
			iface->cur_mode = TWI_I2C_MODE_STANDARDSUB;
		}
		iface->transPtr = data->block;
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		//      rtdm_printk(DEVICE_NAME ": I2C_BLOCK_DATA\n");
		if (read_write == I2C_SMBUS_READ) {
			iface->readNum = data->block[0];
			iface->cur_mode = TWI_I2C_MODE_COMBINED;
			rtdm_printk(DEVICE_NAME ": set read size %d\n", iface->readNum);
		} else {
			iface->writeNum = data->block[0];
			iface->cur_mode = TWI_I2C_MODE_STANDARDSUB;
		}
		iface->transPtr = (u8 *)&data->block[1];
		break;
	default:
		return -1;
	}

	iface->result = 0;
	iface->manual_stop = 0;
	iface->read_write = read_write;
	iface->command = command;
	init_completion(&(iface->complete));

	/* if (!rtdm_in_rt_context()) { */
	/* 	printk(DEVICE_NAME ": Oops: left RT context!!!\n"); */
	/* } */

    
	/* rtdm_printk(DEVICE_NAME ":init event..."); */
	rtdm_event_init(&dev->wc_evt, 0);
	/* rtdm_printk("ok.\n"); */
    
	/* FIFO Initiation. Data in FIFO should be discarded before
	 * start a new operation.
	 */
	/* rtdm_printk(DEVICE_NAME ": discarding data..."); */
	write_FIFO_CTL(iface, 0x3);
	SSYNC();
	write_FIFO_CTL(iface, 0);
	//    rtdm_printk("ok.\n");
    
	/* clear int stat */
	write_INT_STAT(iface, MERR | MCOMP | XMTSERV | RCVSERV);

	/* Set Transmit device address */
	write_MASTER_ADDR(iface, addr);
	SSYNC();
	//    rtdm_printk("ok2.\n");
    
	switch (iface->cur_mode) {
	case TWI_I2C_MODE_STANDARDSUB:
		write_XMT_DATA8(iface, iface->command);
		write_INT_MASK(iface, MCOMP | MERR |
			       ((iface->read_write == I2C_SMBUS_READ) ?
				RCVSERV : XMTSERV));
		SSYNC();

		if (iface->writeNum + 1 <= 255)
			write_MASTER_CTL(iface, (iface->writeNum + 1) << 6);
		else {
			write_MASTER_CTL(iface, 0xff << 6);
			iface->manual_stop = 1;
		}
		/* Master enable */
		write_MASTER_CTL(iface, read_MASTER_CTL(iface) | MEN |
				 ((CONFIG_BFIN_RTDM_I2C_CLK_KHZ>100) ? FAST : 0));
		break;
	case TWI_I2C_MODE_COMBINED:
		write_XMT_DATA8(iface, iface->command);
		write_INT_MASK(iface, MCOMP | MERR | RCVSERV | XMTSERV);
		SSYNC();

		if (iface->writeNum > 0)
			write_MASTER_CTL(iface, (iface->writeNum + 1) << 6);
		else
			write_MASTER_CTL(iface, 0x1 << 6);
		/* Master enable */
		write_MASTER_CTL(iface, read_MASTER_CTL(iface) | MEN | RSTART |
				 ((CONFIG_BFIN_RTDM_I2C_CLK_KHZ>100) ? FAST : 0));
		break;
	default:
		write_MASTER_CTL(iface, 0);
		if (size != I2C_SMBUS_QUICK) {
			/* Don't access xmit data register when this is a
			 * read operation.
			 */
			if (iface->read_write != I2C_SMBUS_READ) {
				if (iface->writeNum > 0) {
					write_XMT_DATA8(iface,
							*(iface->transPtr++));
					if (iface->writeNum <= 255)
						write_MASTER_CTL(iface,
								 iface->writeNum << 6);
					else {
						write_MASTER_CTL(iface,
								 0xff << 6);
						iface->manual_stop = 1;
					}
					iface->writeNum--;
				} else {
					write_XMT_DATA8(iface, iface->command);
					write_MASTER_CTL(iface, 1 << 6);
				}
			} else {
				if (iface->readNum > 0 && iface->readNum <= 255)
					write_MASTER_CTL(iface,
							 iface->readNum << 6);
				else if (iface->readNum > 255) {
					write_MASTER_CTL(iface, 0xff << 6);
					iface->manual_stop = 1;
				} else
					break;
			}
		}
		write_INT_MASK(iface, MCOMP | MERR |
			       ((iface->read_write == I2C_SMBUS_READ) ?
				RCVSERV : XMTSERV));
		SSYNC();

		/* Master enable */
		write_MASTER_CTL(iface, read_MASTER_CTL(iface) | MEN |
				 ((iface->read_write == I2C_SMBUS_READ) ? MDIR : 0) |
				 ((CONFIG_BFIN_RTDM_I2C_CLK_KHZ > 100) ? FAST : 0));
		break;
	}
	SSYNC();

	//    rtdm_printk("ok going to wait for completion...\n");
	/* if (!rtdm_in_rt_context()) { */
	/* 	rtdm_printk(DEVICE_NAME ": WARNING: nrt context!\n"); */
	/* } */
	while (!iface->result) {
      
		if (bfin_rtdm_i2c_wait_for_completion(dev)) {
			iface->result = -1;
			rtdm_printk(DEVICE_NAME "smbus transfer timeout\n");
		}
		//rtdm_printk("***\n");
		//rtdm_printk(".\n");
	}
    
	rc = (iface->result >= 0) ? 0 : -1;

	return rc;
}

/*
 * Generic I2C SMBus transfer entrypoint
 */
int bfin_twi_smbus_xfer(struct bfin_rtdm_i2c_dev *dev, u16 addr,
			unsigned short flags, char read_write,
			u8 command, int size, union i2c_smbus_data *data)
{
	return bfin_twi_do_smbus_xfer(dev, addr, flags,
				      read_write, command, size, data);
}

#if 0
static int bfin_rtdm_i2c_read(u16 addr, int flags, u8 *buf, int len)
{
        int err;
        int tries = 0;
        struct i2c_msg msgs[] = {
                {
                        .addr = addr,
                        .flags = flags;
                        .len = 1,
                        .buf = buf,
                },
                {
                        .addr = addr,
                        .flags = flags | I2C_M_RD,
                        .len = len,
                        .buf = buf,
                },
        };

        do {
                err = bfin_twi_master_xfer(dev, msgs, 2);
                if (err != 2)
                        msleep_interruptible(I2C_RETRY_DELAY);
        } while ((err != 2) && (++tries < I2C_RETRIES));

        if (err != 2) {
                dev_err(&gyro->client->dev, "read transfer error\n");
                err = -EIO;
        } else {
                err = 0;
        }

        return err;
}

static int bfin_rtdm_i2c_write(u16 addr, int flags, u8 *buf, int len)
{
	int err;
        int tries = 0;
        struct i2c_msg msgs[] = {
                {
                        .addr = gyro->client->addr,
                        .flags = gyro->client->flags & I2C_M_TEN,
                        .len = len + 1,
                        .buf = buf,
                },
        };

        do {
                err = i2c_transfer(gyro->client->adapter, msgs, 1);
                if (err != 1)
                        msleep_interruptible(I2C_RETRY_DELAY);
        } while ((err != 1) && (++tries < I2C_RETRIES));

        if (err != 1) {
                dev_err(&gyro->client->dev, "write transfer error\n");
                err = -EIO;
        } else {
                err = 0;
        }

        return err;
}
#endif

/*
 * i2c driver ioctl
 * 
 */
int bfin_rtdm_i2c_ioctl(struct rtdm_dev_context *context,
			rtdm_user_info_t * user_info,
			unsigned int request, void *arg)
{
	int err;
	int datasize;
	struct bfin_rtdm_i2c_context *ctx;
	struct bfin_rtdm_i2c_dev *dev;
	struct bfin_rtdm_i2c_ioctl_data data_arg;
	union i2c_smbus_data tmp_data;
  
	ctx = (struct bfin_rtdm_i2c_context *)context->dev_private;
	dev = ctx->dev;

	switch (request) {
	case I2C_SLAVE:
		ctx->addr = (long)arg;
		//    printk(DEVICE_NAME ": got I2C slave address 0x%x\n", ctx->addr);
		break;
	case I2C_SMBUS:
		/* copy in the transfer data from userspace */
		if (rtdm_copy_from_user(user_info, &data_arg,
					(struct bfin_rtdm_i2c_ioctl_data __user *)arg,
					sizeof(struct bfin_rtdm_i2c_ioctl_data))) {
			return -EFAULT;
		}
    
		/* check transfer type */
		if ((data_arg.smbus_data.read_write != I2C_SMBUS_READ) &&
		    (data_arg.smbus_data.read_write != I2C_SMBUS_WRITE)) {
			return -EINVAL;
		}
    
		/* check data pointer */
		if (data_arg.smbus_data.data == NULL) {
			rtdm_printk("rtdm_i2c data pointer is NULL\n");
			return -EINVAL;
		}

		/* validate transfer type and set actual size */
		switch(data_arg.smbus_data.size) {
		case I2C_SMBUS_QUICK:
			/* standard mode */
			rtdm_printk(DEVICE_NAME "I2C_SMBUS_QUICK not implemented\n");
			return -EINVAL;
      
		case I2C_SMBUS_BYTE:
			/* only read or write byte, no command/reg */
		case I2C_SMBUS_BYTE_DATA:
			datasize = sizeof(data_arg.smbus_data.data->byte);
			break;      
		case I2C_SMBUS_WORD_DATA:
			/* combined mode */
			datasize = sizeof(data_arg.smbus_data.data->word);
			break;
		case I2C_SMBUS_PROC_CALL:
			/* combined mode */
			rtdm_printk(DEVICE_NAME "I2C_SMBUS_PROC_CALL not implemented\n");
			return -EINVAL;
		case I2C_SMBUS_BLOCK_DATA:
			/* combined mode */
		case I2C_SMBUS_I2C_BLOCK_DATA:
			/* combined mode */
			datasize = sizeof(data_arg.smbus_data.data->block);
			break;
		default:
			rtdm_printk(DEVICE_NAME "invalid transfer type %d\n", data_arg.smbus_data.size);
			return -EINVAL;
		}

		if ((data_arg.smbus_data.size == I2C_SMBUS_BLOCK_DATA) ||
		    (data_arg.smbus_data.size == I2C_SMBUS_I2C_BLOCK_DATA)) {
			/* This is a block access, so we need the
			 * first byte of the block data buffer from
			 * the user, as this is where the block size
			 * is specified.
			 */
			if (rtdm_copy_from_user(user_info, &tmp_data, data_arg.smbus_data.data, 1))
				return -EFAULT;

			rtdm_printk(DEVICE_NAME ": got block size %d\n", tmp_data.block[0]);
			
		} else if (data_arg.smbus_data.read_write != I2C_SMBUS_READ) {
			/*
			 * this is a write
			 */
			/* copy in the write data */
			if (rtdm_copy_from_user(user_info, &tmp_data, data_arg.smbus_data.data, datasize))
				return -EFAULT;
		}

		/* rtdm_printk(DEVICE_NAME ": addr=0x%x\n", ctx->addr); */
		/* rtdm_printk(DEVICE_NAME ": flags=0x%x\n", ctx->flags); */
		/* rtdm_printk(DEVICE_NAME ": read_write=%d\n", data_arg.read_write); */
		/* rtdm_printk(DEVICE_NAME ": command=0x%x\n", data_arg.command); */
		/* rtdm_printk(DEVICE_NAME ": size=%d\n", data_arg.size); */
		/* rtdm_printk(DEVICE_NAME ": not doing transfer\n"); */
    
		/* start the transfer */
		err = bfin_twi_smbus_xfer(dev, data_arg.addr, data_arg.flags,
					  data_arg.smbus_data.read_write,
					  data_arg.smbus_data.command,
					  data_arg.smbus_data.size,
					  &tmp_data);
		if (!err && (data_arg.smbus_data.read_write == I2C_SMBUS_READ)) {
			if (rtdm_copy_to_user(user_info, data_arg.smbus_data.data, &tmp_data,
					      datasize))
				return -EFAULT;
		}
    
		return err;
    
	default:
		return -EINVAL;
	}
  
	return 0;
}

/**
 * Open the device
 *
 * This function is called when the device shall be opened.
 *
 */
static int bfin_rtdm_i2c_open(struct rtdm_dev_context *context,
                              rtdm_user_info_t * user_info, int oflags)
{
	struct bfin_rtdm_i2c_context *ctx;
	//struct bfin_rtdm_i2c_dev *dev;
	//  int dev_id = context->device->device_id;
	int err;

	/* printk("bfin_rtdm_i2c_open(): called\n"); */

	ctx = (struct bfin_rtdm_i2c_context *)context->dev_private;
	ctx->dev = (struct bfin_rtdm_i2c_dev *)context->device->device_data;
	//dev = ctx->dev;

	ctx->dev->timeout = 5;
  
	rtdm_lock_init(&ctx->lock);

  
	/* init the event */
	rtdm_event_init(&ctx->dev->wc_evt, 0);
  
	/* request the IRQ */
	err = rtdm_irq_request(&ctx->irq_handle, ctx->dev->iface->irq,
			       bfin_rtdm_i2c_interrupt, RTDM_IRQTYPE_SHARED | RTDM_IRQTYPE_EDGE,
			       context->device->proc_name, ctx);
	if (err) {
		return err;
	}
  
	return 0;
}

/**
 * Close the device
 *
 * This function is called when the device shall be closed.
 *
 */
static int bfin_rtdm_i2c_close(struct rtdm_dev_context *context,
                               rtdm_user_info_t * user_info)
{
	struct bfin_rtdm_i2c_context *ctx;
	//struct bfin_rtdm_i2c_iface *iface;
	//rtdm_lockctx_t lock_ctx;
	//int dev_id = context->device->device_id;
	//int err;

	ctx = (struct bfin_rtdm_i2c_context *)context->dev_private;
  
	/* printk("bfin_rtdm_i2c_close(): called\n"); */

	//  rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	rtdm_irq_free(&ctx->irq_handle);

	/* rtdm_event_destroy(&ctx->in_event); */
  
	return 0;
}

/**
 * Read from the device
 *
 * This function is called when the device is read in realtime context.
 *
 */
static ssize_t bfin_rtdm_i2c_read_rt(struct rtdm_dev_context *context,
                                     rtdm_user_info_t * user_info, void *buf,
                                     size_t nbyte)
{
	pr_debug("bfin_rtdm_i2c_read_rt(): not implemented\n");
	return 0;
}

/**
 * Write in the device
 *
 * This function is called when the device is written in realtime context.
 *
 */
static ssize_t bfin_rtdm_i2c_write_rt(struct rtdm_dev_context *context,
                                      rtdm_user_info_t * user_info,
                                      const void *buf, size_t nbyte)
{
	printk("bfin_rtdm_i2c_write_rt(): not implemented\n");
	return 0;
}

/**
 * The RTDM device template
 *
 */
static struct rtdm_device __initdata device_tmpl = {
	.struct_version = RTDM_DEVICE_STRUCT_VER,
  
	.device_flags = RTDM_NAMED_DEVICE,
	.context_size = sizeof(struct bfin_rtdm_i2c_context),
	.device_name = "bfin_rtdm_i2c",
  
	.open_nrt = bfin_rtdm_i2c_open,
  
	.ops = {
		.close_nrt = bfin_rtdm_i2c_close,
		.ioctl_rt  = bfin_rtdm_i2c_ioctl,
		.ioctl_nrt = bfin_rtdm_i2c_ioctl,
		.read_rt   = bfin_rtdm_i2c_read_rt,
		.write_rt  = bfin_rtdm_i2c_write_rt,
	},
  
	.device_class = RTDM_CLASS_EXPERIMENTAL,
	.device_sub_class = SOME_SUB_CLASS,
	.profile_version = 1,
	.driver_name = "bfin_rtdm_i2c",
	.driver_version = RTDM_DRIVER_VER(0, 1, 0),
	.peripheral_name = "Blackfin Real-time I2C Driver",
	.provider_name = "Devin Butterfield",
};


/**
 * This function is called when the module is loaded
 *
 * Initialize and register device instances
 *
 */
static int bfin_rtdm_i2c_probe(struct platform_device *pdev)
{
	int ret;
	struct bfin_twi_iface *iface;
	struct bfin_rtdm_i2c_dev *dev;
	unsigned int clkhilow;
	struct resource *res;

	iface = kzalloc(sizeof(struct bfin_twi_iface), GFP_KERNEL);
	if (!iface) {
		printk(DEVICE_NAME "Cannot allocate memory\n");
		ret = -ENOMEM;
		goto out_error_nomem;
	}

	dev = kzalloc(sizeof(struct bfin_rtdm_i2c_dev), GFP_KERNEL);
	if (!dev) {
		printk(DEVICE_NAME "Cannot allocate memory\n");
		ret = -ENOMEM;
		goto out_error_nomem;
	}
  
	memcpy(&dev->rtdm_dev, &device_tmpl, sizeof(struct rtdm_device));
	dev->rtdm_dev.device_id = 0;
	dev->rtdm_dev.proc_name = dev->rtdm_dev.device_name;
	dev->rtdm_dev.device_data = dev;
	dev->iface = iface;
  
	// init our lock?

	/* register the RTDM device */
	ret = rtdm_dev_register(&dev->rtdm_dev);
	if (ret)
		goto out_error_get_res;

	/* Find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out_error_get_res;
	}
  
	iface->regs_base = ioremap(res->start, resource_size(res));
	if (iface->regs_base == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		ret = -ENXIO;
		goto out_error_ioremap;
	}

	iface->irq = platform_get_irq(pdev, 0);
	if (iface->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		ret = -ENOENT;
		goto out_error_no_irq;
	}

	dev->timeout = 5;
	dev->retries = 3;
  
	ret = peripheral_request_list((unsigned short *)pdev->dev.platform_data,
				      "bfin_rtdm_i2c");
	if (ret) {
		dev_err(&pdev->dev, "Can't setup pin mux!\n");
		goto out_error_pin_mux;
	}

	/* ret = request_irq(iface->irq, bfin_twi_interrupt_entry, */
	/*                   0, pdev->name, iface); */
	/* if (rc) { */
	/*   dev_err(&pdev->dev, "Can't get IRQ %d !\n", iface->irq); */
	/*   rc = -ENODEV; */
	/*   goto out_error_req_irq; */
	/* } */
  
	/* Set TWI internal clock as 10MHz */
	write_CONTROL(iface, ((get_sclk() / 1000 / 1000 + 5) / 10) & 0x7F);
  
	/*
	 * We will not end up with a CLKDIV=0 because no one will specify
	 * 20kHz SCL or less in Kconfig now. (5 * 1000 / 20 = 250)
	 */
	clkhilow = ((10 * 1000 / CONFIG_BFIN_RTDM_I2C_CLK_KHZ) + 1) / 2;
  
	/* Set Twi interface clock as specified */
	write_CLKDIV(iface, (clkhilow << 8) | clkhilow);
  
	/* Enable TWI */
	write_CONTROL(iface, read_CONTROL(iface) | TWI_ENA);
	SSYNC();
  
	/* rc = i2c_add_numbered_adapter(p_adap); */
	/* if (rc < 0) { */
	/*   dev_err(&pdev->dev, "Can't add i2c adapter!\n"); */
	/*   goto out_error_add_adapter; */
	/* } */

	platform_set_drvdata(pdev, dev);

	dev_info(&pdev->dev, "Real-Time I2C driver for the ADI Blackfin. "
		 "regs_base@%p\n", iface->regs_base);
  
	return 0;
  
out_error_no_irq:
	peripheral_free_list((unsigned short *)pdev->dev.platform_data);
out_error_pin_mux:
	iounmap(iface->regs_base);
out_error_ioremap:
out_error_get_res:
	kfree(iface);
	kfree(dev);
out_error_nomem:

  
	return ret;
}

/**
 * This function is called when the module is unloaded
 *
 * It unregister the RTDM device, polling at 1000 ms for pending users.
 *
 */
static int bfin_rtdm_i2c_remove(struct platform_device *pdev)
{
	struct bfin_rtdm_i2c_dev *dev = platform_get_drvdata(pdev);
  
	platform_set_drvdata(pdev, NULL);

	/* free RTDM IRQ here? */

	/* unregister the RTDM device */
	rtdm_dev_unregister(&dev->rtdm_dev, 1000);

	peripheral_free_list((unsigned short *)pdev->dev.platform_data);
	iounmap(dev->iface->regs_base);
	kfree(dev->iface);
	kfree(dev);

	return 0;
}

static struct platform_driver bfin_rtdm_i2c_driver = {
	.probe		= bfin_rtdm_i2c_probe,
	.remove		= bfin_rtdm_i2c_remove,
	/* .suspend	    = bfin_rtdm_i2c_suspend, */
	/* .resume		= bfin_rtdm_i2c_resume, */
	.driver		= {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init bfin_rtdm_i2c_init(void)
{
	return platform_driver_register(&bfin_rtdm_i2c_driver);
}

static void __exit bfin_rtdm_i2c_exit(void)
{
	platform_driver_unregister(&bfin_rtdm_i2c_driver);
}


module_init(bfin_rtdm_i2c_init);
module_exit(bfin_rtdm_i2c_exit);


/* bfin_rtdm_i2c.c ends here */
