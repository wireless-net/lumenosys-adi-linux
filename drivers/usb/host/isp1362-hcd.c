/*
 * ISP1362 HCD (Host Controller Driver) for USB.
 *
 * Copyright (C) 2005 Lothar Wassmann <LW@KARO-electronics.de>
 *
 * Derived from the SL811 HCD, rewritten for ISP116x.
 * Copyright (C) 2005 Olav Kongas <ok@artecdesign.ee>
 *
 * Portions:
 * Copyright (C) 2004 Psion Teklogix (for NetBook PRO)
 * Copyright (C) 2004 David Brownell
 *
 */

/*
  The ISP1362 chip requires a large delay (300ns and 462ns) between
  accesses to the address and data register.
  The following timing options exist:

  1. Configure your memory controller if it can accomodate such delays (the best)
  2. Implement platform-specific delay function possibly
     combined with configuring the memory controller; see
     include/linux/usb_isp1362.h for more info.
  3. Use ndelay (easiest, poorest).

  Use the corresponding macros USE_PLATFORM_DELAY and USE_NDELAY in the
  platform specific section of isp1362.h to select the appropriate variant.

  Also note that according to the Philips "ISP1362 Errata" document
  Rev 1.00 from 27 May data corruption may occur when the #WR signal
  is reasserted (even with #CS deasserted) within 132ns after a
  write cycle to any controller register. If the hardware doesn't
  implement the recommended fix (gating the #WR with #CS) software
  must ensure that no further write cycle (not necessarily to the chip!)
  is issued by the CPU within this interval.

  For PXA25x this can be ensured by using VLIO with the maximum
  recovery time (MSCx = 0x7f8c) with a memory clock of 99.53 MHz.
*/

#ifdef CONFIG_USB_DEBUG
 #define DEBUG
#else
 #undef DEBUG
#endif

/*
 * The PXA255 UDC apparently doesn't handle GET_STATUS, GET_CONFIG and
 * GET_INTERFACE requests correctly when the SETUP and DATA stages of the
 * requests are carried out in separate frames. This will delay any SETUP
 * packets until the start of the next frame so that this situation is
 * unlikely to occur (and makes usbtest happy running with a PXA255 target device).
 */
//#define BUGGY_PXA2XX_UDC_USBTEST

//#define PTD_TRACE
//#define URB_TRACE
//#define VERBOSE
//#define REGISTERS

/* This enables a memory test on the ISP1362 chip memory to make sure the
 * chip access timing is correct.
 */
//#define CHIP_BUFFER_TEST

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/usb_isp1362.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/byteorder.h>
#include <asm/bitops.h>
#include <asm/unaligned.h>

static int dbg_level = 0;
#ifdef DEBUG
module_param(dbg_level, int, 0644);
#else
module_param(dbg_level, int, 0);
#define	STUB_DEBUG_FILE
#endif

#include "../core/hcd.h"
#include "../core/usb.h"
#include "isp1362.h"


#define DRIVER_VERSION	"2005-04-04"
#define DRIVER_DESC	"ISP1362 USB Host Controller Driver"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static const char hcd_name[] = "isp1362-hcd";

static void isp1362_hc_stop(struct usb_hcd *hcd);
static int isp1362_hc_start(struct usb_hcd *hcd);

/*-------------------------------------------------------------------------*/

/*
 * When called from the interrupthandler only isp1362_hcd->irqenb is modified,
 * since the interrupt handler will write isp1362_hcd->irqenb to HCuPINT upon
 * completion.
 * We don't need a 'disable' counterpart, since interrupts will be disabled
 * only by the interrupt handler.
 */
static inline void isp1362_enable_int(struct isp1362_hcd *isp1362_hcd, u16 mask)
{
	if ((isp1362_hcd->irqenb | mask) == isp1362_hcd->irqenb) {
		return;
	}
	if (mask & ~isp1362_hcd->irqenb) {
		isp1362_write_reg16(isp1362_hcd, HCuPINT, mask & ~isp1362_hcd->irqenb);
	}
	isp1362_hcd->irqenb |= mask;
	if (isp1362_hcd->irq_active) {
		return;
	}
	isp1362_write_reg16(isp1362_hcd, HCuPINTENB, isp1362_hcd->irqenb);
}

/*-------------------------------------------------------------------------*/

static inline struct isp1362_ep_queue *get_ptd_queue(struct isp1362_hcd *isp1362_hcd,
							     u16 offset)
{
	struct isp1362_ep_queue * epq = NULL;

	if (offset < isp1362_hcd->istl_queue[1].buf_start) {
		epq = &isp1362_hcd->istl_queue[0];
	} else if (offset < isp1362_hcd->intl_queue.buf_start) {
		epq = &isp1362_hcd->istl_queue[1];
	} else if (offset < isp1362_hcd->atl_queue.buf_start) {
		epq = &isp1362_hcd->intl_queue;
	} else if (offset < isp1362_hcd->atl_queue.buf_start +
		   isp1362_hcd->atl_queue.buf_size) {
		epq = &isp1362_hcd->atl_queue;
	}
	if (epq) {
		DBG(1, "%s: PTD $%04x is on %s queue\n", __FUNCTION__, offset, epq->name);
	} else {
		WARN("%s: invalid PTD $%04x\n", __FUNCTION__, offset);
	}

	return epq;
}

static inline int get_ptd_offset(struct isp1362_ep_queue *epq, u8 index)
{
	int offset;

	if (index * epq->blk_size > epq->buf_size) {
		WARN("%s: Bad %s index %d(%d)\n", __FUNCTION__, epq->name, index,
		     epq->buf_size / epq->blk_size);
		return -EINVAL;
	}
	offset = epq->buf_start + index * epq->blk_size;
	DBG(3, "%s: %s PTD[%02x] # %04x\n", __FUNCTION__, epq->name, index, offset);

	return offset;
}

/*-------------------------------------------------------------------------*/

static inline u16 max_transfer_size(struct isp1362_ep_queue *epq, size_t size, int mps)
{
	u16 xfer_size = min_t(size_t, MAX_XFER_SIZE, size);

	xfer_size = min_t(size_t, xfer_size, epq->buf_avail * epq->blk_size - PTD_HEADER_SIZE);
	if (xfer_size < size && xfer_size % mps) {
		xfer_size -= xfer_size % mps;
	}

	return xfer_size;
}

static int claim_ptd_buffers(struct isp1362_ep_queue *epq, struct isp1362_ep *ep, u16 len)
{
	int ptd_offset = -EINVAL;
	int index;
	int num_ptds = ((len + PTD_HEADER_SIZE - 1) / epq->blk_size) + 1;
	int found = -1;
	int last = -1;

	BUG_ON(len > epq->buf_size);

	if (!epq->buf_avail) {
		return -ENOMEM;
	}

	if (ep->num_ptds) {
		ERR("%s: %s len %d/%d num_ptds %d buf_map %08lx skip_map %08lx\n", __FUNCTION__,
		    epq->name, len, epq->blk_size, num_ptds, epq->buf_map, epq->skip_map);
	}
	BUG_ON(ep->num_ptds != 0);

	for (index = 0; index <= epq->buf_count - num_ptds; index++) {
		if (__test_bit(index, &epq->buf_map)) {
			continue;
		}
		found = index;
		for (last = index + 1; last < index + num_ptds; last++) {
			if (__test_bit(last, &epq->buf_map)) {
				found = -1;
				break;
			}
		}
		if (found >= 0) {
			break;
		}
	}
	if (found < 0) {
		return -EOVERFLOW;
	}

	DBG(1, "%s: Found %d PTDs[%d] for %d/%d byte\n", __FUNCTION__,
	    num_ptds, found, len, epq->blk_size - PTD_HEADER_SIZE);
	ptd_offset = get_ptd_offset(epq, found);
	WARN_ON(ptd_offset < 0);
	ep->ptd_offset = ptd_offset;
	ep->num_ptds += num_ptds;
	epq->buf_avail -= num_ptds;
	BUG_ON(epq->buf_avail > epq->buf_count);
	ep->ptd_index = found;
	for (index = found; index < last; index++) {
		__set_bit(index, &epq->buf_map);
	}
	DBG(1, "%s: Done %s PTD[%d] $%04x, avail %d count %d claimed %d %08lx:%08lx\n",
	    __FUNCTION__, epq->name, ep->ptd_index, ep->ptd_offset,
	    epq->buf_avail, epq->buf_count, num_ptds, epq->buf_map, epq->skip_map);

	return found;
}

static inline void release_ptd_buffers(struct isp1362_ep_queue *epq, struct isp1362_ep *ep)
{
	int index = ep->ptd_index;
	int last = ep->ptd_index + ep->num_ptds;

	if (last > epq->buf_count) {
		ERR("%s: ep %p req %d len %d %s PTD[%d] $%04x num_ptds %d buf_count %d buf_avail %d buf_map %08lx skip_map %08lx\n",
		    __FUNCTION__, ep, ep->num_req, ep->length, epq->name, ep->ptd_index,
		    ep->ptd_offset, ep->num_ptds, epq->buf_count, epq->buf_avail,
		    epq->buf_map, epq->skip_map);
	}
	BUG_ON(last > epq->buf_count);

	for (; index < last; index++) {
		__clear_bit(index, &epq->buf_map);
		__set_bit(index, &epq->skip_map);
	}
	epq->buf_avail += ep->num_ptds;
	epq->ptd_count--;

	BUG_ON(epq->buf_avail > epq->buf_count);
	BUG_ON(epq->ptd_count > epq->buf_count);

	DBG(1, "%s: Done %s PTDs $%04x released %d avail %d count %d\n",
	    __FUNCTION__, epq->name,
	    ep->ptd_offset, ep->num_ptds, epq->buf_avail, epq->buf_count);
	DBG(1, "%s: buf_map %08lx skip_map %08lx\n", __FUNCTION__,
	    epq->buf_map, epq->skip_map);

	ep->num_ptds = 0;
	ep->ptd_offset = -EINVAL;
	ep->ptd_index = -EINVAL;
}

/*-------------------------------------------------------------------------*/

/*
  Set up PTD's.
*/
static void prepare_ptd(struct isp1362_hcd *isp1362_hcd, struct urb *urb, struct isp1362_ep *ep,
			struct isp1362_ep_queue *epq, u16 fno)
{
	struct ptd *ptd;
	int toggle;
	int dir;
	u16 len;
	size_t buf_len = urb->transfer_buffer_length - urb->actual_length;

	DBG(3, "%s: %s ep %p\n", __func__, epq->name, ep);

	ptd = &ep->ptd;

	spin_lock(&urb->lock);
	ep->data = (unsigned char *)urb->transfer_buffer + urb->actual_length;

	switch (ep->nextpid) {
	case USB_PID_IN:
		toggle = usb_gettoggle(urb->dev, ep->epnum, 0);
		dir = PTD_DIR_IN;
		if (usb_pipecontrol(urb->pipe)) {
			len = min_t(size_t, ep->maxpacket, buf_len);
		} else if (usb_pipeisoc(urb->pipe)) {
			len = min_t(size_t, urb->iso_frame_desc[fno].length, MAX_XFER_SIZE);
			ep->data = urb->transfer_buffer + urb->iso_frame_desc[fno].offset;
		} else {
			len = max_transfer_size(epq, buf_len, ep->maxpacket);
		}
		DBG(1, "%s: IN    len %d/%d/%d from URB\n", __FUNCTION__, len, ep->maxpacket,
		    buf_len);
		break;
	case USB_PID_OUT:
		toggle = usb_gettoggle(urb->dev, ep->epnum, 1);
		dir = PTD_DIR_OUT;
		if (usb_pipecontrol(urb->pipe)) {
			len = min_t(size_t, ep->maxpacket, buf_len);
		} else if (usb_pipeisoc(urb->pipe)) {
			len = min_t(size_t, urb->iso_frame_desc[0].length, MAX_XFER_SIZE);
		} else {
			len = max_transfer_size(epq, buf_len, ep->maxpacket);
		}
		if (len == 0) {
			INFO("%s: Sending ZERO packet: %d\n", __FUNCTION__,
			     urb->transfer_flags & URB_ZERO_PACKET);
		}
		DBG(1, "%s: OUT   len %d/%d/%d from URB\n", __FUNCTION__, len, ep->maxpacket,
		    buf_len);
		break;
	case USB_PID_SETUP:
		toggle = 0;
		dir = PTD_DIR_SETUP;
		len = sizeof(struct usb_ctrlrequest);
		DBG(1, "%s: SETUP len %d\n", __FUNCTION__, len);
		ep->data = urb->setup_packet;
		break;
	case USB_PID_ACK:
		toggle = 1;
		len = 0;
		dir = (urb->transfer_buffer_length && usb_pipein(urb->pipe)) ?
			PTD_DIR_OUT : PTD_DIR_IN;
		DBG(1, "%s: ACK   len %d\n", __FUNCTION__, len);
		break;
	default:
		// To please gcc
		toggle = dir = len = 0;
		ERR("%s@%d: ep->nextpid %02x\n", __func__, __LINE__, ep->nextpid);
		BUG_ON(1);
	}

	ep->length = len;
	if (!len) {
		ep->data = NULL;
	}

	ptd->count = PTD_CC_MSK | PTD_ACTIVE_MSK | PTD_TOGGLE(toggle);
	ptd->mps = PTD_MPS(ep->maxpacket) | PTD_SPD(urb->dev->speed == USB_SPEED_LOW) |
		PTD_EP(ep->epnum);
	ptd->len = PTD_LEN(len) | PTD_DIR(dir);
	ptd->faddr = PTD_FA(usb_pipedevice(urb->pipe));

	if (usb_pipeint(urb->pipe)) {
		ptd->faddr |= PTD_SF_INT(ep->branch);
		ptd->faddr |= PTD_PR(ep->interval ? __ffs(ep->interval) : 0);
	}
	if (usb_pipeisoc(urb->pipe)) {
		ptd->faddr |= PTD_SF_ISO(fno);
	}

	spin_unlock(&urb->lock);
	DBG(1, "%s: Finished\n", __FUNCTION__);
}

static void isp1362_write_ptd(struct isp1362_hcd *isp1362_hcd, struct isp1362_ep *ep,
			      struct isp1362_ep_queue *epq)
{
	struct ptd *ptd = &ep->ptd;
	int len = PTD_GET_DIR(ptd) == PTD_DIR_IN ? 0 : ep->length;

	_BUG_ON(ep->ptd_offset < 0);

	prefetch(ptd);
	isp1362_write_buffer(isp1362_hcd, ptd, ep->ptd_offset, PTD_HEADER_SIZE);
	if (len) {
		isp1362_write_buffer(isp1362_hcd, ep->data,
				     ep->ptd_offset + PTD_HEADER_SIZE, len);
	}

	dump_ptd(ptd);
	dump_ptd_out_data(ptd, ep->data);
}

static void isp1362_read_ptd(struct isp1362_hcd *isp1362_hcd, struct isp1362_ep *ep,
			     struct isp1362_ep_queue *epq)
{
	struct ptd *ptd = &ep->ptd;
	int act_len;

	WARN_ON(list_empty(&ep->active));
	BUG_ON(ep->ptd_offset < 0);

	list_del_init(&ep->active);
	DBG(1, "%s: ep %p removed from active list %p\n", __FUNCTION__, ep, &epq->active);

	prefetchw(ptd);
	isp1362_read_buffer(isp1362_hcd, ptd, ep->ptd_offset, PTD_HEADER_SIZE);
	dump_ptd(ptd);
	act_len = PTD_GET_COUNT(ptd);
	if (PTD_GET_DIR(ptd) != PTD_DIR_IN || act_len == 0) {
		return;
	}
	if (act_len > ep->length) {
		ERR("%s: ep %p PTD $%04x act_len %d ep->length %d\n", __FUNCTION__, ep,
			 ep->ptd_offset, act_len, ep->length);
	}
	BUG_ON(act_len > ep->length);
	/* Only transfer the amount of data that has actually been overwritten
	 * in the chip buffer. We don't want any data that doesn't belong to the
	 * transfer to leak out of the chip to the callers transfer buffer!
	 */
	prefetchw(ep->data);
	isp1362_read_buffer(isp1362_hcd, ep->data,
			    ep->ptd_offset + PTD_HEADER_SIZE, act_len);
	dump_ptd_in_data(ptd, ep->data);
}

/*
 * INT PTDs will stay in the chip until data is available.
 * This function will remove a PTD from the chip when the URB is dequeued.
 * Must be called with the spinlock held and IRQs disabled
 */
static void remove_ptd(struct isp1362_hcd *isp1362_hcd, struct isp1362_ep *ep)

{
	int index;
	struct isp1362_ep_queue *epq;

	DBG(1, "%s: ep %p PTD[%d] $%04x\n", __FUNCTION__, ep, ep->ptd_index, ep->ptd_offset);
	BUG_ON(ep->ptd_offset < 0);

	epq = get_ptd_queue(isp1362_hcd, ep->ptd_offset);
	BUG_ON(!epq);

	// put ep in remove_list for cleanup
	WARN_ON(!list_empty(&ep->remove_list));
	list_add_tail(&ep->remove_list, &isp1362_hcd->remove_list);
	// let SOF interrupt handle the cleanup
	isp1362_enable_int(isp1362_hcd, HCuPINT_SOF);

	index = ep->ptd_index;
	if (index < 0) {
		// ISO queues don't have SKIP registers
		return;
	}

	DBG(1, "%s: Disabling PTD[%02x] $%04x %08lx|%08x\n", __FUNCTION__,
	    index, ep->ptd_offset, epq->skip_map, 1 << index);

	// prevent further processing of PTD (will be effective after next SOF)
	epq->skip_map |= 1 << index;
	if (epq == &isp1362_hcd->atl_queue) {
		DBG(2, "%s: ATLSKIP = %08x -> %08lx\n", __FUNCTION__,
		    isp1362_read_reg32(isp1362_hcd, HCATLSKIP), epq->skip_map);
		isp1362_write_reg32(isp1362_hcd, HCATLSKIP, epq->skip_map);
		if (~epq->skip_map == 0) {
			isp1362_clr_mask16(isp1362_hcd, HCBUFSTAT, HCBUFSTAT_ATL_ACTIVE);
		}
	} else if (epq == &isp1362_hcd->intl_queue) {
		DBG(2, "%s: INTLSKIP = %08x -> %08lx\n", __FUNCTION__,
		    isp1362_read_reg32(isp1362_hcd, HCINTLSKIP), epq->skip_map);
		isp1362_write_reg32(isp1362_hcd, HCINTLSKIP, epq->skip_map);
		if (~epq->skip_map == 0) {
			isp1362_clr_mask16(isp1362_hcd, HCBUFSTAT, HCBUFSTAT_INTL_ACTIVE);
		}
	}
}

/*
  Take done or failed requests out of schedule. Give back
  processed urbs.
*/
static void finish_request(struct isp1362_hcd *isp1362_hcd, struct isp1362_ep *ep,
			   struct urb *urb, struct pt_regs *regs, int status)
     __releases(isp1362_hcd->lock)
     __acquires(isp1362_hcd->lock)
{
	urb->hcpriv = NULL;
	ep->error_count = 0;

	if (usb_pipecontrol(urb->pipe)) {
		ep->nextpid = USB_PID_SETUP;
	}

	spin_lock(&urb->lock);
	if (urb->status == -EINPROGRESS) {
		urb->status = status;
	}
	spin_unlock(&urb->lock);
	URB_DBG("%s: req %d FA %d ep%d%s %s: len %d/%d %s stat %d\n", __func__,
		ep->num_req, usb_pipedevice(urb->pipe),
		usb_pipeendpoint(urb->pipe),
		!usb_pipein(urb->pipe) ? "out" : "in",
		({
			char *s;
			if (usb_pipecontrol(urb->pipe)) {
				s = "ctrl";
			} else if(usb_pipeint(urb->pipe)) {
				s = "int";
			} else if(usb_pipebulk(urb->pipe)) {
				s = "bulk";
			} else {
				s = "iso";
			}
			s;}),
		urb->actual_length, urb->transfer_buffer_length,
		!(urb->transfer_flags & URB_SHORT_NOT_OK) ?
		"short_ok" : "", urb->status);

	spin_unlock(&isp1362_hcd->lock);
	usb_hcd_giveback_urb(isp1362_hcd_to_hcd(isp1362_hcd), urb, regs);
	spin_lock(&isp1362_hcd->lock);

	// take idle endpoints out of the schedule right away
	if (!list_empty(&ep->hep->urb_list)) {
		return;
	}

	if (ep->interval) {
		// periodic deschedule
		DBG(1, "deschedule qh%d/%p branch %d load %d bandwidth %d -> %d\n", ep->interval,
		    ep, ep->branch, ep->load,
		    isp1362_hcd->load[ep->branch],
		    isp1362_hcd->load[ep->branch] - ep->load);
		isp1362_hcd->load[ep->branch] -= ep->load;
		ep->branch = PERIODIC_SIZE;
		if (urb->bandwidth) {
			DBG(0, "%s: Releasing bandwidth for urb %p ep %p req %d\n",
			    __FUNCTION__, urb, ep, ep->num_req);
			usb_release_bandwidth(urb->dev, urb, usb_pipeisoc(urb->pipe));
		}
	}
	// async deschedule
	if (!list_empty(&ep->schedule)) {
		list_del_init(&ep->schedule);
	}
}

/*
 * Analyze transfer results, handle partial transfers and errors
*/
static void postproc_ep(struct isp1362_hcd *isp1362_hcd, struct isp1362_ep *ep,
			struct pt_regs *regs)
{
	struct urb *urb = get_urb(ep);
	struct usb_device *udev;
	struct ptd *ptd;
	int short_ok;
	u16 len;
	int urbstat = -EINPROGRESS;
	u8 cc;

	DBG(2, "%s: ep %p req %d\n", __FUNCTION__, ep, ep->num_req);

	udev = urb->dev;
	ptd = &ep->ptd;
	cc = PTD_GET_CC(ptd);
	if (cc == PTD_NOTACCESSED) {
		ERR("%s: req %d PTD %p Untouched by ISP1362\n", __FUNCTION__,
		    ep->num_req, ptd);
		cc = PTD_DEVNOTRESP;
	}

	short_ok = !(urb->transfer_flags & URB_SHORT_NOT_OK);
	len = urb->transfer_buffer_length - urb->actual_length;

	/* Data underrun is special. For allowed underrun
	   we clear the error and continue as normal. For
	   forbidden underrun we finish the DATA stage
	   immediately while for control transfer,
	   we do a STATUS stage.
	*/
	if (cc == PTD_DATAUNDERRUN) {
		if (short_ok) {
			DBG(1, "%s: req %d Allowed data underrun short_%sok %d/%d/%d byte\n",
			    __func__, ep->num_req, short_ok ? "" : "not_",
			    PTD_GET_COUNT(ptd), ep->maxpacket, len);
			cc = PTD_CC_NOERROR;
			urbstat = 0;
		} else {
			DBG(1, "%s: req %d Data Underrun %s nextpid %02x short_%sok %d/%d/%d byte\n",
			    __FUNCTION__, ep->num_req,
			    usb_pipein(urb->pipe) ? "IN" : "OUT", ep->nextpid,
			    short_ok ? "" : "not_",
			    PTD_GET_COUNT(ptd), ep->maxpacket, len);
			if (usb_pipecontrol(urb->pipe)) {
				ep->nextpid = USB_PID_ACK;
				// save the data underrun error code for later and
				// procede with the status stage
				urb->actual_length += PTD_GET_COUNT(ptd);
				BUG_ON(urb->actual_length > urb->transfer_buffer_length);
				spin_lock(&urb->lock);
				if (urb->status == -EINPROGRESS) {
					urb->status = cc_to_error[PTD_DATAUNDERRUN];
				}
				spin_unlock(&urb->lock);
			} else {
				usb_settoggle(udev, ep->epnum, ep->nextpid == USB_PID_OUT,
					      PTD_GET_TOGGLE(ptd));
				urbstat = cc_to_error[PTD_DATAUNDERRUN];
			}
			goto out;
		}
	}

	if (cc != PTD_CC_NOERROR) {
		if (++ep->error_count >= 3 || cc == PTD_CC_STALL || cc == PTD_DATAOVERRUN) {
			urbstat = cc_to_error[cc];
			DBG(1, "%s: req %d nextpid %02x, status %d, error %d, error_count %d\n",
			    __func__, ep->num_req, ep->nextpid, urbstat, cc,
			    ep->error_count);
		}
		goto out;
	}

	switch (ep->nextpid) {
	case USB_PID_OUT:
		if (PTD_GET_COUNT(ptd) != ep->length) {
		       ERR("%s: count=%d len=%d\n", __FUNCTION__,
			   PTD_GET_COUNT(ptd), ep->length);
		}
		BUG_ON(PTD_GET_COUNT(ptd) != ep->length);
		urb->actual_length += ep->length;
		BUG_ON(urb->actual_length > urb->transfer_buffer_length);
		usb_settoggle(udev, ep->epnum, 1, PTD_GET_TOGGLE(ptd));
		if (urb->actual_length == urb->transfer_buffer_length) {
			DBG(3, "%s: req %d xfer complete %d/%d status %d -> 0\n", __func__,
			    ep->num_req, len, ep->maxpacket, urbstat);
			if (usb_pipecontrol(urb->pipe)) {
				DBG(3, "%s: req %d %s Wait for ACK\n", __FUNCTION__,
				    ep->num_req,
				    usb_pipein(urb->pipe) ? "IN" : "OUT");
				ep->nextpid = USB_PID_ACK;
			} else {
				if (len % ep->maxpacket ||
				    !(urb->transfer_flags & URB_ZERO_PACKET)) {
					urbstat = 0;
					DBG(3, "%s: req %d URB %s status %d count %d/%d/%d\n",
					    __func__, ep->num_req, usb_pipein(urb->pipe) ? "IN" : "OUT",
					    urbstat, len, ep->maxpacket, urb->actual_length);
				}
			}
		}
		break;
	case USB_PID_IN:
		len = PTD_GET_COUNT(ptd);
		BUG_ON(len > ep->length);
		urb->actual_length += len;
		BUG_ON(urb->actual_length > urb->transfer_buffer_length);
		usb_settoggle(udev, ep->epnum, 0, PTD_GET_TOGGLE(ptd));
		// if transfer completed or (allowed) data underrun
		if ((urb->transfer_buffer_length == urb->actual_length) ||
		    len % ep->maxpacket) {
			DBG(3, "%s: req %d xfer complete %d/%d status %d -> 0\n", __func__,
			    ep->num_req, len, ep->maxpacket, urbstat);
			if (usb_pipecontrol(urb->pipe)) {
				DBG(3, "%s: req %d %s Wait for ACK\n", __FUNCTION__,
				    ep->num_req,
				    usb_pipein(urb->pipe) ? "IN" : "OUT");
				ep->nextpid = USB_PID_ACK;
			} else {
				urbstat = 0;
				DBG(3, "%s: req %d URB %s status %d count %d/%d/%d\n",
				    __func__, ep->num_req, usb_pipein(urb->pipe) ? "IN" : "OUT",
				    urbstat, len, ep->maxpacket, urb->actual_length);
			}
		}
		break;
	case USB_PID_SETUP:
		if (urb->transfer_buffer_length == urb->actual_length) {
			ep->nextpid = USB_PID_ACK;
		} else if (usb_pipeout(urb->pipe)) {
			usb_settoggle(udev, 0, 1, 1);
			ep->nextpid = USB_PID_OUT;
		} else {
			usb_settoggle(udev, 0, 0, 1);
			ep->nextpid = USB_PID_IN;
		}
		break;
	case USB_PID_ACK:
		DBG(3, "%s: req %d got ACK %d -> 0\n", __FUNCTION__, ep->num_req,
		    urbstat);
		WARN_ON(urbstat != -EINPROGRESS);
		urbstat = 0;
		ep->nextpid = 0;
		break;
	default:
		BUG_ON(1);
	}

 out:
	if (urbstat != -EINPROGRESS) {
		DBG(2, "%s: Finishing ep %p req %d urb %p status %d\n", __FUNCTION__,
		    ep, ep->num_req, urb, urbstat);
		finish_request(isp1362_hcd, ep, urb, regs, urbstat);
	}
}

static void finish_unlinks(struct isp1362_hcd *isp1362_hcd, struct pt_regs *regs)
{
	struct isp1362_ep *ep;
	struct isp1362_ep *tmp;

	list_for_each_entry_safe(ep, tmp, &isp1362_hcd->remove_list, remove_list) {
		struct isp1362_ep_queue *epq =
			get_ptd_queue(isp1362_hcd, ep->ptd_offset);
		int index = ep->ptd_index;

		BUG_ON(epq == NULL);
		if (index >= 0) {
			DBG(1, "%s: remove PTD[%d] $%04x\n", __FUNCTION__, index, ep->ptd_offset);
			BUG_ON(ep->num_ptds == 0);
			release_ptd_buffers(epq, ep);
		}
		if (!list_empty(&ep->hep->urb_list)) {
			struct urb *urb = get_urb(ep);

			DBG(1, "%s: Finishing req %d ep %p from remove_list\n", __FUNCTION__,
			    ep->num_req, ep);
			finish_request(isp1362_hcd, ep, urb, regs, -ESHUTDOWN);
		}
		WARN_ON(list_empty(&ep->active));
		if (!list_empty(&ep->active)) {
			list_del_init(&ep->active);
			DBG(1, "%s: ep %p removed from active list\n", __FUNCTION__, ep);
		}
		list_del_init(&ep->remove_list);
		DBG(1, "%s: ep %p removed from remove_list\n", __FUNCTION__, ep);
	}
	DBG(1, "%s: Done\n", __FUNCTION__);
}

static inline void enable_atl_transfers(struct isp1362_hcd *isp1362_hcd, int count)
{
	if (count > 0) {
		if (count < isp1362_hcd->atl_queue.ptd_count) {
			isp1362_write_reg16(isp1362_hcd, HCATLDTC, count);
		}
		isp1362_enable_int(isp1362_hcd, HCuPINT_ATL);
		isp1362_write_reg32(isp1362_hcd, HCATLSKIP, isp1362_hcd->atl_queue.skip_map);
		isp1362_set_mask16(isp1362_hcd, HCBUFSTAT, HCBUFSTAT_ATL_ACTIVE);
	} else {
		isp1362_enable_int(isp1362_hcd, HCuPINT_SOF);
	}
}

static inline void enable_intl_transfers(struct isp1362_hcd *isp1362_hcd)
{
	isp1362_enable_int(isp1362_hcd, HCuPINT_INTL);
	isp1362_set_mask16(isp1362_hcd, HCBUFSTAT, HCBUFSTAT_INTL_ACTIVE);
	isp1362_write_reg32(isp1362_hcd, HCINTLSKIP, isp1362_hcd->intl_queue.skip_map);
}

static inline void enable_istl_transfers(struct isp1362_hcd *isp1362_hcd, int flip)
{
	isp1362_enable_int(isp1362_hcd, flip ? HCuPINT_ISTL1 : HCuPINT_ISTL0);
	isp1362_set_mask16(isp1362_hcd, HCBUFSTAT, flip ?
			   HCBUFSTAT_ISTL1_FULL : HCBUFSTAT_ISTL0_FULL);
}

static int submit_req(struct isp1362_hcd *isp1362_hcd, struct urb *urb,
		      struct isp1362_ep *ep, struct isp1362_ep_queue *epq)
{
	int index = epq->free_ptd;

	prepare_ptd(isp1362_hcd, urb, ep, epq, 0);
	index = claim_ptd_buffers(epq, ep, ep->length);
	if (index == -ENOMEM) {
		DBG(1, "%s: req %d No free %s PTD available: %d, %08lx:%08lx\n", __FUNCTION__,
		    ep->num_req, epq->name, ep->num_ptds, epq->buf_map, epq->skip_map);
		return index;
	} else if (index == -EOVERFLOW) {
		DBG(1, "%s: req %d Not enough space for %d byte %s PTD %d %08lx:%08lx\n",
		    __FUNCTION__, ep->num_req, ep->length, epq->name, ep->num_ptds,
		    epq->buf_map, epq->skip_map);
		return index;
	} else {
		BUG_ON(index < 0);
	}
	list_add_tail(&ep->active, &epq->active);
	DBG(1, "%s: ep %p req %d len %d added to active list %p\n", __FUNCTION__,
	    ep, ep->num_req, ep->length, &epq->active);
	DBG(1, "%s: Submitting %s PTD $%04x for ep %p req %d\n", __FUNCTION__, epq->name,
	    ep->ptd_offset, ep, ep->num_req);
	isp1362_write_ptd(isp1362_hcd, ep, epq);
	__clear_bit(ep->ptd_index, &epq->skip_map);

	return 0;
}

static void start_atl_transfers(struct isp1362_hcd *isp1362_hcd)
{
	int ptd_count = 0;
	struct isp1362_ep_queue *epq = &isp1362_hcd->atl_queue;
	struct isp1362_ep *ep;
	int defer = 0;

	if (atomic_read(&epq->finishing)) {
		DBG(1, "%s: finish_transfers is active for %s\n", __FUNCTION__, epq->name);
		return;
	}

	list_for_each_entry(ep, &isp1362_hcd->async, schedule) {
		struct urb *urb = get_urb(ep);
		int ret;

		if (!list_empty(&ep->active)) {
			DBG(2, "%s: Skipping active %s ep %p\n", __FUNCTION__, epq->name, ep);
			continue;
		}

		DBG(1, "%s: Processing %s ep %p req %d\n", __FUNCTION__, epq->name,
		    ep, ep->num_req);

		ret = submit_req(isp1362_hcd, urb, ep, epq);
		if (ret == -ENOMEM) {
			defer = 1;
			break;
		} else if (ret == -EOVERFLOW) {
			defer = 1;
			continue;
		}
#ifdef BUGGY_PXA2XX_UDC_USBTEST
		defer = ep->nextpid == USB_PID_SETUP;
#endif
		ptd_count++;
	}

	/* Avoid starving of endpoints */
	if (isp1362_hcd->async.next != isp1362_hcd->async.prev) {
		DBG(2, "%s: Cycling ASYNC schedule %d\n", __FUNCTION__, ptd_count);
		list_move(&isp1362_hcd->async, isp1362_hcd->async.next);
	}
	if (ptd_count || defer) {
		enable_atl_transfers(isp1362_hcd, defer ? 0 : ptd_count);
	}

	epq->ptd_count += ptd_count;
	if (epq->ptd_count > epq->stat_maxptds) {
		epq->stat_maxptds = epq->ptd_count;
		DBG(0, "%s: max_ptds: %d\n", __FUNCTION__, epq->stat_maxptds);
	}
}

static void start_intl_transfers(struct isp1362_hcd *isp1362_hcd)
{
	int ptd_count = 0;
	struct isp1362_ep_queue *epq = &isp1362_hcd->intl_queue;
	struct isp1362_ep *ep;

	if (atomic_read(&epq->finishing)) {
		DBG(1, "%s: finish_transfers is active for %s\n", __FUNCTION__, epq->name);
		return;
	}

	list_for_each_entry(ep, &isp1362_hcd->periodic, schedule) {
		struct urb *urb = get_urb(ep);
		int ret;

		if (!list_empty(&ep->active)) {
			DBG(1, "%s: Skipping active %s ep %p\n", __FUNCTION__,
			    epq->name, ep);
			continue;
		}

		DBG(1, "%s: Processing %s ep %p req %d\n", __FUNCTION__,
		    epq->name, ep, ep->num_req);
		ret = submit_req(isp1362_hcd, urb, ep, epq);
		if (ret == -ENOMEM) {
			break;
		} else if (ret == -EOVERFLOW) {
			continue;
		}
		ptd_count++;
	}

	if (ptd_count) {
		static int last_count = 0;

		if (ptd_count != last_count) {
			DBG(0, "%s: ptd_count: %d\n", __FUNCTION__, ptd_count);
			last_count = ptd_count;
		}
		enable_intl_transfers(isp1362_hcd);
	}

	epq->ptd_count += ptd_count;
	if (epq->ptd_count > epq->stat_maxptds) {
		epq->stat_maxptds = epq->ptd_count;
	}
}

static inline int next_ptd(struct isp1362_ep_queue *epq, struct isp1362_ep *ep)
{
	u16 ptd_offset = ep->ptd_offset;
	int num_ptds = (ep->length + PTD_HEADER_SIZE + (epq->blk_size - 1)) / epq->blk_size;

	DBG(2, "%s: PTD offset $%04x + %04x => %d * %04x -> $%04x\n", __FUNCTION__, ptd_offset,
	    ep->length, num_ptds, epq->blk_size, ptd_offset + num_ptds * epq->blk_size);

	ptd_offset += num_ptds * epq->blk_size;
	if (ptd_offset < epq->buf_start + epq->buf_size) {
		return ptd_offset;
	} else {
		return -ENOMEM;
	}
}

static void start_iso_transfers(struct isp1362_hcd *isp1362_hcd)
{
	int ptd_count = 0;
	int flip = isp1362_hcd->istl_flip;
	struct isp1362_ep_queue *epq;
	int ptd_offset;
	struct isp1362_ep *ep;
	struct isp1362_ep *tmp;
	u16 fno = isp1362_read_reg32(isp1362_hcd, HCFMNUM);

 fill2:
	epq = &isp1362_hcd->istl_queue[flip];
	if (atomic_read(&epq->finishing)) {
		DBG(1, "%s: finish_transfers is active for %s\n", __FUNCTION__, epq->name);
		return;
	}

	if (!list_empty(&epq->active)) {
		return;
	}

	ptd_offset = epq->buf_start;
	list_for_each_entry_safe(ep, tmp, &isp1362_hcd->isoc, schedule) {
		struct urb *urb = get_urb(ep);
		s16 diff = fno - (u16)urb->start_frame;

		DBG(1, "%s: Processing %s ep %p\n", __FUNCTION__, epq->name, ep);

		if (diff > urb->number_of_packets) {
			// time frame for this URB has elapsed
			finish_request(isp1362_hcd, ep, urb, NULL, -EOVERFLOW);
			continue;
		} else if (diff < -1) {
			// URB is not due in this frame or the next one.
			// Comparing with '-1' instead of '0' accounts for double
			// buffering in the ISP1362 which enables us to queue the PTD
			// one frame ahead of time
		} else if (diff == -1) {
			// submit PTD's that are due in the next frame
			prepare_ptd(isp1362_hcd, urb, ep, epq, fno);
			if (ptd_offset + PTD_HEADER_SIZE + ep->length >
			    epq->buf_start + epq->buf_size) {
				ERR("%s: Not enough ISO buffer space for %d byte PTD\n",
				    __func__, ep->length);
				continue;
			}
			ep->ptd_offset = ptd_offset;
			list_add_tail(&ep->active, &epq->active);

			ptd_offset = next_ptd(epq, ep);
			if (ptd_offset < 0) {
				WARN("%s: req %d No more %s PTD buffers available\n", __func__,
				     ep->num_req, epq->name);
				break;
			}
		}
	}
	list_for_each_entry(ep, &epq->active, active) {
		if (epq->active.next == &ep->active) {
			ep->ptd.mps |= PTD_LAST_MSK;
		}
		isp1362_write_ptd(isp1362_hcd, ep, epq);
		ptd_count++;
	}

	if (ptd_count) {
		enable_istl_transfers(isp1362_hcd, flip);
	}

	epq->ptd_count += ptd_count;
	if (epq->ptd_count > epq->stat_maxptds) {
		epq->stat_maxptds = epq->ptd_count;
	}

	// check, whether the second ISTL buffer may also be filled
	if (!(isp1362_read_reg16(isp1362_hcd, HCBUFSTAT) &
	      (flip ? HCBUFSTAT_ISTL0_FULL : HCBUFSTAT_ISTL1_FULL))) {
		fno++;
		ptd_count = 0;
		flip = 1 - flip;
		goto fill2;
	}
}

static void finish_transfers(struct isp1362_hcd *isp1362_hcd, unsigned long done_map,
			     struct isp1362_ep_queue *epq, struct pt_regs *regs)
{
	struct isp1362_ep *ep;
	struct isp1362_ep *tmp;

	if (list_empty(&epq->active)) {
		DBG(1, "%s: Nothing to do for %s queue\n", __FUNCTION__, epq->name);
		return;
	}

	DBG(1, "%s: Finishing %s transfers %08lx\n", __FUNCTION__, epq->name, done_map);

	atomic_inc(&epq->finishing);
	list_for_each_entry_safe(ep, tmp, &epq->active, active) {
		int index = ep->ptd_index;

		DBG(1, "%s: Checking %s PTD[%02x] $%04x\n", __FUNCTION__, epq->name,
		    index, ep->ptd_offset);

		BUG_ON(index < 0);
		if (__test_and_clear_bit(index, &done_map)) {
			isp1362_read_ptd(isp1362_hcd, ep, epq);
			epq->free_ptd = index;
			BUG_ON(ep->num_ptds == 0);
			release_ptd_buffers(epq, ep);

			DBG(1, "%s: ep %p req %d removed from active list\n", __FUNCTION__,
			    ep, ep->num_req);
			if (!list_empty(&ep->remove_list)) {
				list_del_init(&ep->remove_list);
				DBG(1, "%s: ep %p removed from remove list\n", __FUNCTION__, ep);
			}
			DBG(1, "%s: Postprocessing %s ep %p req %d\n", __FUNCTION__, epq->name,
			    ep, ep->num_req);
			postproc_ep(isp1362_hcd, ep, regs);
		}
		if (!done_map) {
			break;
		}
	}
	if (done_map) {
		WARN("%s: done_map not clear: %08lx:%08lx\n", __FUNCTION__, done_map,
		     epq->skip_map);
	}
	atomic_dec(&epq->finishing);
}

static void finish_iso_transfers(struct isp1362_hcd *isp1362_hcd, struct isp1362_ep_queue *epq,
				 struct pt_regs *regs)
{
	struct isp1362_ep *ep;
	struct isp1362_ep *tmp;

	if (list_empty(&epq->active)) {
		DBG(1, "%s: Nothing to do for %s queue\n", __FUNCTION__, epq->name);
		return;
	}

	DBG(1, "%s: Finishing %s transfers\n", __FUNCTION__, epq->name);

	atomic_inc(&epq->finishing);
	list_for_each_entry_safe(ep, tmp, &epq->active, active) {
		DBG(1, "%s: Checking PTD $%04x\n", __FUNCTION__, ep->ptd_offset);

		isp1362_read_ptd(isp1362_hcd, ep, epq);
		DBG(1, "%s: Postprocessing %s ep %p\n", __FUNCTION__, epq->name, ep);
		postproc_ep(isp1362_hcd, ep, regs);
	}
	WARN_ON(epq->blk_size != 0);
	atomic_dec(&epq->finishing);
}

static irqreturn_t isp1362_irq(struct usb_hcd *hcd, struct pt_regs *regs)
{
	int handled = 0;
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	u16 irqstat;
	u16 svc_mask;

	spin_lock(&isp1362_hcd->lock);

	BUG_ON(isp1362_hcd->irq_active++);

	isp1362_write_reg16(isp1362_hcd, HCuPINTENB, 0);

	irqstat = isp1362_read_reg16(isp1362_hcd, HCuPINT);
	DBG(3, "%s: got IRQ %04x:%04x\n", __FUNCTION__, irqstat, isp1362_hcd->irqenb);

	// only handle interrupts that are currently enabled
	irqstat &= isp1362_hcd->irqenb;
	isp1362_write_reg16(isp1362_hcd, HCuPINT, irqstat);
	svc_mask = irqstat;

	if (irqstat & HCuPINT_SOF) {
		isp1362_hcd->irqenb &= ~HCuPINT_SOF;
		isp1362_hcd->irq_stat[ISP1362_INT_SOF]++;
		handled = 1;
		svc_mask &= ~HCuPINT_SOF;
		DBG(3, "%s: SOF\n", __FUNCTION__);
		isp1362_hcd->fmindex = isp1362_read_reg32(isp1362_hcd, HCFMNUM);
		if (!list_empty(&isp1362_hcd->remove_list)) {
			finish_unlinks(isp1362_hcd, regs);
		}
		if (!list_empty(&isp1362_hcd->async) && !(irqstat & HCuPINT_ATL)) {
			if (list_empty(&isp1362_hcd->atl_queue.active)) {
				start_atl_transfers(isp1362_hcd);
			} else {
				isp1362_enable_int(isp1362_hcd, HCuPINT_ATL);
				isp1362_write_reg32(isp1362_hcd, HCATLSKIP,
						    isp1362_hcd->atl_queue.skip_map);
				isp1362_set_mask16(isp1362_hcd, HCBUFSTAT, HCBUFSTAT_ATL_ACTIVE);
			}
		}
	}

	if (irqstat & HCuPINT_ISTL0) {
		isp1362_hcd->irq_stat[ISP1362_INT_ISTL0]++;
		handled = 1;
		svc_mask &= ~HCuPINT_ISTL0;
		isp1362_clr_mask16(isp1362_hcd, HCBUFSTAT, HCBUFSTAT_ISTL0_FULL);
		DBG(1, "%s: ISTL0\n", __FUNCTION__);
		WARN_ON(isp1362_hcd->istl_flip);
		WARN_ON(isp1362_read_reg16(isp1362_hcd, HCBUFSTAT) & HCBUFSTAT_ISTL0_ACTIVE);
		WARN_ON(!isp1362_read_reg16(isp1362_hcd, HCBUFSTAT) & HCBUFSTAT_ISTL0_DONE);
		isp1362_hcd->irqenb &= ~HCuPINT_ISTL0;
	}

	if (irqstat & HCuPINT_ISTL1) {
		isp1362_hcd->irq_stat[ISP1362_INT_ISTL1]++;
		handled = 1;
		svc_mask &= ~HCuPINT_ISTL1;
		isp1362_clr_mask16(isp1362_hcd, HCBUFSTAT, HCBUFSTAT_ISTL1_FULL);
		DBG(1, "%s: ISTL1\n", __FUNCTION__);
		WARN_ON(!isp1362_hcd->istl_flip);
		WARN_ON(isp1362_read_reg16(isp1362_hcd, HCBUFSTAT) & HCBUFSTAT_ISTL1_ACTIVE);
		WARN_ON(!isp1362_read_reg16(isp1362_hcd, HCBUFSTAT) & HCBUFSTAT_ISTL1_DONE);
		isp1362_hcd->irqenb &= ~HCuPINT_ISTL1;
	}

	if (irqstat & (HCuPINT_ISTL0 | HCuPINT_ISTL1)) {
		WARN_ON((irqstat & (HCuPINT_ISTL0 | HCuPINT_ISTL1)) ==
			(HCuPINT_ISTL0 | HCuPINT_ISTL1));
		finish_iso_transfers(isp1362_hcd,
				     &isp1362_hcd->istl_queue[isp1362_hcd->istl_flip], regs);
		start_iso_transfers(isp1362_hcd);
		isp1362_hcd->istl_flip = 1 - isp1362_hcd->istl_flip;
	}

	if (irqstat & HCuPINT_INTL) {
		u32 done_map = isp1362_read_reg32(isp1362_hcd, HCINTLDONE);
		u32 skip_map = isp1362_read_reg32(isp1362_hcd, HCINTLSKIP);
		isp1362_hcd->irq_stat[ISP1362_INT_INTL]++;

		DBG(2, "%s: INTL\n", __FUNCTION__);

		svc_mask &= ~HCuPINT_INTL;

		isp1362_write_reg32(isp1362_hcd, HCINTLSKIP, skip_map | done_map);
		if (~(done_map | skip_map) == 0) {
			// All PTDs are finished, disable INTL processing entirely
			isp1362_clr_mask16(isp1362_hcd, HCBUFSTAT, HCBUFSTAT_INTL_ACTIVE);
		}

		handled = 1;
		WARN_ON(!done_map);
		if (done_map) {
			DBG(3, "%s: INTL done_map %08x\n", __func__, done_map);
			finish_transfers(isp1362_hcd, done_map, &isp1362_hcd->intl_queue, regs);
			start_intl_transfers(isp1362_hcd);
		}
	}

	if (irqstat & HCuPINT_ATL) {
		u32 done_map = isp1362_read_reg32(isp1362_hcd, HCATLDONE);
		u32 skip_map = isp1362_read_reg32(isp1362_hcd, HCATLSKIP);
		isp1362_hcd->irq_stat[ISP1362_INT_ATL]++;

		DBG(2, "%s: ATL\n", __FUNCTION__);

		svc_mask &= ~HCuPINT_ATL;

		isp1362_write_reg32(isp1362_hcd, HCATLSKIP, skip_map | done_map);
		if (~(done_map | skip_map) == 0) {
			isp1362_clr_mask16(isp1362_hcd, HCBUFSTAT, HCBUFSTAT_ATL_ACTIVE);
		}
		if (done_map) {
			DBG(3, "%s: ATL done_map %08x\n", __func__, done_map);
			finish_transfers(isp1362_hcd, done_map, &isp1362_hcd->atl_queue, regs);
			start_atl_transfers(isp1362_hcd);
		}
		handled = 1;
	}

	if (irqstat & HCuPINT_OPR) {
		u32 intstat = isp1362_read_reg32(isp1362_hcd, HCINTSTAT);
		isp1362_hcd->irq_stat[ISP1362_INT_OPR]++;

		svc_mask &= ~HCuPINT_OPR;
		DBG(2, "%s: OPR %08x:%08x\n", __FUNCTION__, intstat, isp1362_hcd->intenb);
		intstat &= isp1362_hcd->intenb;
		if (intstat & OHCI_INTR_UE) {
			ERR("Unrecoverable error\n");
			// FIXME: do here reset or cleanup or whatever
		}
		if (intstat & OHCI_INTR_RHSC) {
			isp1362_hcd->rhstatus = isp1362_read_reg32(isp1362_hcd, HCRHSTATUS);
			isp1362_hcd->rhport[0] = isp1362_read_reg32(isp1362_hcd, HCRHPORT1);
			isp1362_hcd->rhport[1] = isp1362_read_reg32(isp1362_hcd, HCRHPORT2);
		}
		if (intstat & OHCI_INTR_RD) {
			INFO("%s: RESUME DETECTED\n", __FUNCTION__);
			isp1362_show_reg(isp1362_hcd, HCCONTROL);
			usb_hcd_resume_root_hub(hcd);
		}
		isp1362_write_reg32(isp1362_hcd, HCINTSTAT, intstat);
		irqstat &= ~HCuPINT_OPR;
		handled = 1;
	}

	if (irqstat & HCuPINT_SUSP) {
		isp1362_hcd->irq_stat[ISP1362_INT_SUSP]++;
		handled = 1;
		svc_mask &= ~HCuPINT_SUSP;

		INFO("%s: SUSPEND IRQ\n", __FUNCTION__);
	}

	if (irqstat & HCuPINT_CLKRDY) {
		isp1362_hcd->irq_stat[ISP1362_INT_CLKRDY]++;
		handled = 1;
		isp1362_hcd->irqenb &= ~HCuPINT_CLKRDY;
		svc_mask &= ~HCuPINT_CLKRDY;
		INFO("%s: CLKRDY IRQ\n", __FUNCTION__);
	}

	if (svc_mask) {
		ERR("%s: Unserviced interrupt(s) %04x\n", __func__, svc_mask);
	}
	isp1362_write_reg16(isp1362_hcd, HCuPINTENB, isp1362_hcd->irqenb);
	isp1362_hcd->irq_active--;
	spin_unlock(&isp1362_hcd->lock);

	return IRQ_RETVAL(handled);
}

/*-------------------------------------------------------------------------*/

#define	MAX_PERIODIC_LOAD	900	// out of 1000 usec
static int balance(struct isp1362_hcd *isp1362_hcd, u16 interval, u16 load)
{
	int i, branch = -ENOSPC;

	// search for the least loaded schedule branch of that interval
	// which has enough bandwidth left unreserved.
	for (i = 0; i < interval; i++) {
		if (branch < 0 || isp1362_hcd->load[branch] > isp1362_hcd->load[i]) {
			int j;

			for (j = i; j < PERIODIC_SIZE; j += interval) {
				if ((isp1362_hcd->load[j] + load) > MAX_PERIODIC_LOAD) {
					ERR("%s: new load %d load[%02x] %d max %d\n", __FUNCTION__,
					    load, j, isp1362_hcd->load[j], MAX_PERIODIC_LOAD);
					break;
				}
			}
			if (j < PERIODIC_SIZE) {
				continue;
			}
			branch = i;
		}
	}
	return branch;
}

/* NB! ALL the code above this point runs with isp1362_hcd->lock
   held, irqs off
*/

/*-------------------------------------------------------------------------*/

static int isp1362_urb_enqueue(struct usb_hcd *hcd, struct usb_host_endpoint *hep,
			       struct urb *urb, gfp_t mem_flags)
{
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	struct usb_device *udev = urb->dev;
	unsigned int pipe = urb->pipe;
	int is_out = !usb_pipein(pipe);
	int type = usb_pipetype(pipe);
	int epnum = usb_pipeendpoint(pipe);
	struct isp1362_ep *ep = NULL;
	unsigned long flags;
	int retval = 0;

	DBG(3, "%s: urb %p\n", __func__, urb);

	if (type == PIPE_ISOCHRONOUS) {
		ERR("Isochronous transfers not supported\n");
		return -ENOSPC;
	}

	URB_DBG("%s: FA %d ep%d%s %s: len %d %s%s\n", __func__,
		usb_pipedevice(pipe), epnum,
		is_out ? "out" : "in",
		({
			char *s;
			if (usb_pipecontrol(pipe)) {
				s = "ctrl";
			} else if(usb_pipeint(pipe)) {
				s = "int";
			} else if(usb_pipebulk(pipe)) {
				s = "bulk";
			} else {
				s = "iso";
			}
			s;}),
		urb->transfer_buffer_length,
		(urb->transfer_flags & URB_ZERO_PACKET) ? "ZERO_PACKET " : "",
		!(urb->transfer_flags & URB_SHORT_NOT_OK) ?
		"short_ok" : "");

	// avoid all allocations within spinlocks: request or endpoint
	if (!hep->hcpriv) {
		ep = kcalloc(1, sizeof *ep, mem_flags);
	}
	spin_lock_irqsave(&isp1362_hcd->lock, flags);

	/* don't submit to a dead or disabled port */
	if (!((isp1362_hcd->rhport[0] | isp1362_hcd->rhport[1]) &
	      (1 << USB_PORT_FEAT_ENABLE)) ||
	    !HC_IS_RUNNING(hcd->state)) {
		retval = -ENODEV;
		goto fail;
	}

	if (hep->hcpriv) {
		kfree(ep);
		ep = hep->hcpriv;
	} else if (!ep) {
		retval = -ENOMEM;
		goto fail;
	} else {
		INIT_LIST_HEAD(&ep->schedule);
		INIT_LIST_HEAD(&ep->active);
		INIT_LIST_HEAD(&ep->remove_list);
		ep->udev = usb_get_dev(udev);
		ep->hep = hep;
		ep->epnum = epnum;
		ep->maxpacket = usb_maxpacket(udev, urb->pipe, is_out);
		ep->ptd_offset = -EINVAL;
		ep->ptd_index = -EINVAL;
		usb_settoggle(udev, epnum, is_out, 0);

		if (type == PIPE_CONTROL) {
			ep->nextpid = USB_PID_SETUP;
		} else if (is_out) {
			ep->nextpid = USB_PID_OUT;
		} else {
			ep->nextpid = USB_PID_IN;
		}

		switch (type) {
		case PIPE_ISOCHRONOUS:
		case PIPE_INTERRUPT:
			if (urb->interval > PERIODIC_SIZE) {
				urb->interval = PERIODIC_SIZE;
			}
			ep->interval = urb->interval;
			ep->branch = PERIODIC_SIZE;
			ep->load = usb_calc_bus_time(udev->speed, !is_out,
						     (type == PIPE_ISOCHRONOUS),
						     usb_maxpacket(udev, pipe, is_out)) / 1000;
			break;
		}
		hep->hcpriv = ep;
	}
	ep->num_req = isp1362_hcd->req_serial++;

	/* maybe put endpoint into schedule */
	switch (type) {
	case PIPE_CONTROL:
	case PIPE_BULK:
		if (list_empty(&ep->schedule)) {
			DBG(1, "%s: Adding ep %p req %d to async schedule\n",
				__FUNCTION__, ep, ep->num_req);
			list_add_tail(&ep->schedule, &isp1362_hcd->async);
		}
		break;
	case PIPE_ISOCHRONOUS:
	case PIPE_INTERRUPT:
		urb->interval = ep->interval;

		// urb submitted for already existing EP
		if (ep->branch < PERIODIC_SIZE) {
			break;
		}

		retval = balance(isp1362_hcd, ep->interval, ep->load);
		if (retval < 0) {
			ERR("%s: balance returned %d\n", __FUNCTION__, retval);
			goto fail;
		}
		ep->branch = retval;
		retval = 0;
		isp1362_hcd->fmindex = isp1362_read_reg32(isp1362_hcd, HCFMNUM);
		DBG(1, "%s: Current frame %04x branch %02x start_frame %04x(%04x)\n",
		    __FUNCTION__, isp1362_hcd->fmindex, ep->branch,
		    ((isp1362_hcd->fmindex + PERIODIC_SIZE - 1) &
		     ~(PERIODIC_SIZE - 1)) + ep->branch,
		    (isp1362_hcd->fmindex & (PERIODIC_SIZE - 1)) + ep->branch);

		if (list_empty(&ep->schedule)) {
			if (type == PIPE_ISOCHRONOUS) {
				u16 frame = isp1362_hcd->fmindex;

				frame += max_t(u16, 8, ep->interval);
				frame &= ~(ep->interval - 1);
				frame |= ep->branch;
				if (frame_before(frame, isp1362_hcd->fmindex)) {
					frame += ep->interval;
				}
				urb->start_frame = frame;

				DBG(1, "%s: Adding ep %p to isoc schedule\n", __func__, ep);
				list_add_tail(&ep->schedule, &isp1362_hcd->isoc);
			} else {
				DBG(1, "%s: Adding ep %p to periodic schedule\n", __func__, ep);
				list_add_tail(&ep->schedule, &isp1362_hcd->periodic);
			}
		} else {
			DBG(1, "%s: ep %p already scheduled\n", __func__, ep);
		}
		usb_claim_bandwidth(udev, urb, ep->load / ep->interval, type == PIPE_ISOCHRONOUS);
		DBG(2, "%s: load %d bandwidth %d -> %d\n", __FUNCTION__,
		    ep->load / ep->interval, isp1362_hcd->load[ep->branch],
		    isp1362_hcd->load[ep->branch] + ep->load);
		isp1362_hcd->load[ep->branch] += ep->load;
	}

	/* in case of unlink-during-submit */
	spin_lock(&urb->lock);
	if (urb->status != -EINPROGRESS) {
		spin_unlock(&urb->lock);
		WARN("%s: Finishing ep %p req %d before submission with status %08x\n",
		     __func__, ep, ep->num_req, urb->status);
		finish_request(isp1362_hcd, ep, urb, NULL, 0);
		retval = 0;
		goto fail;
	}
	urb->hcpriv = hep;
	ALIGNSTAT(isp1362_hcd, urb->transfer_buffer);
	spin_unlock(&urb->lock);

	switch (type) {
	case PIPE_CONTROL:
	case PIPE_BULK:
		start_atl_transfers(isp1362_hcd);
		break;
	case PIPE_INTERRUPT:
		start_intl_transfers(isp1362_hcd);
		break;
	case PIPE_ISOCHRONOUS:
		start_iso_transfers(isp1362_hcd);
		break;
	default:
		BUG();
	}

 fail:
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
	if (retval) {
		DBG(0, "%s: urb %p failed with %d\n", __FUNCTION__, urb, retval);
	}
	return retval;
}

static int isp1362_urb_dequeue(struct usb_hcd *hcd, struct urb *urb)
{
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	struct usb_host_endpoint *hep;
	unsigned long flags;
	struct isp1362_ep *ep;
	int retval = 0;

	DBG(3, "%s: urb %p\n", __func__, urb);

	spin_lock_irqsave(&isp1362_hcd->lock, flags);
	hep = urb->hcpriv;
	if (!hep) {
		spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
		return -EIDRM;
	}

	ep = hep->hcpriv;
	if (ep) {
		// In front of queue?
		if (ep->hep->urb_list.next == &urb->urb_list) {
			if (!list_empty(&ep->active)) {
				DBG(1, "%s: urb %p ep %p req %d active PTD[%d] $%04x\n", __func__,
				    urb, ep, ep->num_req, ep->ptd_index, ep->ptd_offset);
				// disable processing and queue PTD for removal
				remove_ptd(isp1362_hcd, ep);
				urb = NULL;
			}
		}
		if (urb) {
			DBG(1, "%s: Finishing ep %p req %d\n", __func__, ep,
			    ep->num_req);
			finish_request(isp1362_hcd, ep, urb, NULL, -ESHUTDOWN);
		} else {
			DBG(1, "%s: urb %p active; wait4irq\n", __func__, urb);
		}
	} else {
		WARN("%s: No EP in URB %p\n", __FUNCTION__, urb);
		retval = -EINVAL;
	}
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);

	DBG(3, "%s: exit\n",__func__);

	return retval;
}

static void isp1362_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
	struct isp1362_ep *ep = hep->hcpriv;
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	unsigned long flags;

	DBG(1, "%s: ep %p\n", __func__, ep);
	if (!ep) {
		return;
	}
	spin_lock_irqsave(&isp1362_hcd->lock, flags);
	if (!list_empty(&hep->urb_list)) {
		if (!list_empty(&ep->active) && list_empty(&ep->remove_list)) {
			DBG(1, "%s: Removing ep %p req %d PTD[%d] $%04x\n", __FUNCTION__,
			    ep, ep->num_req, ep->ptd_index, ep->ptd_offset);
			remove_ptd(isp1362_hcd, ep);
			INFO("%s: Waiting for Interrupt to clean up\n", __FUNCTION__);
		}
	}
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
	// Wait for interrupt to clear out active list
	while (!list_empty(&ep->active)) {
		msleep(1);
	}

	DBG(1, "%s: Freeing EP %p\n", __FUNCTION__, ep);

	usb_put_dev(ep->udev);
	kfree(ep);
	hep->hcpriv = NULL;
}

static int isp1362_get_frame(struct usb_hcd *hcd)
{
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	u32 fmnum;
	unsigned long flags;

	spin_lock_irqsave(&isp1362_hcd->lock, flags);
	fmnum = isp1362_read_reg32(isp1362_hcd, HCFMNUM);
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);

	return (int)fmnum;
}

/*-------------------------------------------------------------------------*/

static int isp1362_bus_suspend(struct usb_hcd *hcd);
// Adapted from ohci-hub.c
static int isp1362_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	int ports, i, changed = 0;
	int can_suspend = hcd->can_wakeup;

	if (!HC_IS_RUNNING(hcd->state)) {
		return -ESHUTDOWN;
	}

	ports = isp1362_hcd->rhdesca & RH_A_NDP;
	BUG_ON(ports > 2);

	/* init status */
	if (isp1362_hcd->rhstatus & (RH_HS_LPSC | RH_HS_OCIC)) {
		buf[0] = changed = 1;
	} else {
		buf[0] = 0;
	}

	for (i = 0; i < ports; i++) {
		u32 status = isp1362_hcd->rhport[i];

		if (status & (RH_PS_CSC | RH_PS_PESC | RH_PS_PSSC |
			      RH_PS_OCIC | RH_PS_PRSC)) {
			changed = 1;
			buf[0] |= 1 << (i + 1);
			continue;
		}

		if (!(status & RH_PS_CCS)) {
			continue;
		}
		if ((status & RH_PS_PSS) && hcd->remote_wakeup) {
			continue;
		}
		can_suspend = 0;
	}
#if defined(CONFIG_USB_SUSPEND) || defined(CONFIG_PM)
	if (can_suspend && !changed &&
	    list_empty(&isp1362_hcd->async) &&
	    list_empty(&isp1362_hcd->periodic) &&
	    list_empty(&isp1362_hcd->isoc) &&
	    (isp1362_read_reg32(isp1362_hcd, HCCONTROL) & OHCI_CTRL_HCFS) == OHCI_USB_OPER &&
	    time_after(jiffies, isp1362_hcd->next_statechange) &&
	    usb_trylock_device(hcd->self.root_hub)) {
		DBG(0, "%s: Autosuspending root hub\n", __FUNCTION__);
		isp1362_set_mask16(isp1362_hcd, HCHWCFG, HCHWCFG_CLKNOTSTOP);
		(void) isp1362_bus_suspend(hcd);
		hcd->state = HC_STATE_RUNNING;
		usb_unlock_device(hcd->self.root_hub);
	}
#endif
	return changed;
}

static void isp1362_hub_descriptor(struct isp1362_hcd *isp1362_hcd,
				   struct usb_hub_descriptor *desc)
{
	u32 reg = isp1362_hcd->rhdesca;

	DBG(3, "%s: enter\n", __func__);

	desc->bDescriptorType = 0x29;
	desc->bDescLength = 9;
	desc->bHubContrCurrent = 0;
	desc->bNbrPorts = reg & 0x3;
	// Power switching, device type, overcurrent.
	desc->wHubCharacteristics = cpu_to_le16((reg >> 8) & 0x1f);
	DBG(0, "%s: hubcharacteristics = %02x\n", __FUNCTION__, cpu_to_le16((reg >> 8) & 0x1f));
	desc->bPwrOn2PwrGood = (reg >> 24) & 0xff;
	// two bitmaps:  ports removable, and legacy PortPwrCtrlMask
	desc->bitmap[0] = desc->bNbrPorts == 1 ? 1 << 1 : 3 << 1;
	desc->bitmap[1] = ~0;

	DBG(3, "%s: exit\n", __func__);
}

// Adapted from ohci-hub.c
static int isp1362_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue,
			       u16 wIndex, char *buf, u16 wLength)
{
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	int retval = 0;
	unsigned long flags;
	unsigned long t1;
	int ports = isp1362_hcd->rhdesca & RH_A_NDP;
	u32 tmp = 0;

	switch (typeReq) {
	case ClearHubFeature:
		DBG(0, "ClearHubFeature: ");
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
			_DBG(0, "C_HUB_OVER_CURRENT\n");
			spin_lock_irqsave(&isp1362_hcd->lock, flags);
			isp1362_write_reg32(isp1362_hcd, HCRHSTATUS, RH_HS_OCIC);
			spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
		case C_HUB_LOCAL_POWER:
			_DBG(0, "C_HUB_LOCAL_POWER\n");
			break;
		default:
			goto error;
		}
		break;
	case SetHubFeature:
		DBG(0, "SetHubFeature: ");
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			_DBG(0, "C_HUB_OVER_CURRENT or C_HUB_LOCAL_POWER\n");
			break;
		default:
			goto error;
		}
		break;
	case GetHubDescriptor:
		DBG(0, "GetHubDescriptor\n");
		isp1362_hub_descriptor(isp1362_hcd, (struct usb_hub_descriptor *)buf);
		break;
	case GetHubStatus:
		DBG(0, "GetHubStatus\n");
		put_unaligned (cpu_to_le32(0), (__le32 *) buf);
		break;
	case GetPortStatus:
#ifndef VERBOSE
		DBG(0, "GetPortStatus\n");
#endif
		if (!wIndex || wIndex > ports) {
			goto error;
		}
		tmp = isp1362_hcd->rhport[--wIndex];
		put_unaligned (cpu_to_le32(tmp), (__le32 *) buf);
#ifndef	VERBOSE
		if (*(u16 *) (buf + 2))	/* only if wPortChange is interesting */
#endif
			DBG(0, "GetPortStatus: port[%d]  %08x\n", wIndex + 1, tmp);
		break;
	case ClearPortFeature:
		DBG(0, "ClearPortFeature: ");
		if (!wIndex || wIndex > ports) {
			goto error;
		}
		wIndex--;

		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			_DBG(0, "USB_PORT_FEAT_ENABLE\n");
			tmp = RH_PS_CCS;
			break;
		case USB_PORT_FEAT_C_ENABLE:
			_DBG(0, "USB_PORT_FEAT_C_ENABLE\n");
			tmp = RH_PS_PESC;
			break;
		case USB_PORT_FEAT_SUSPEND:
			_DBG(0, "USB_PORT_FEAT_SUSPEND\n");
			tmp = RH_PS_POCI;
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			_DBG(0, "USB_PORT_FEAT_C_SUSPEND\n");
			tmp = RH_PS_PSSC;
			break;
		case USB_PORT_FEAT_POWER:
			_DBG(0, "USB_PORT_FEAT_POWER\n");
			tmp = RH_PS_LSDA;

			break;
		case USB_PORT_FEAT_C_CONNECTION:
			_DBG(0, "USB_PORT_FEAT_C_CONNECTION\n");
			tmp = RH_PS_CSC;
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			_DBG(0, "USB_PORT_FEAT_C_OVER_CURRENT\n");
			tmp = RH_PS_OCIC;
			break;
		case USB_PORT_FEAT_C_RESET:
			_DBG(0, "USB_PORT_FEAT_C_RESET\n");
			tmp = RH_PS_PRSC;
			break;
		default:
			goto error;
		}

		spin_lock_irqsave(&isp1362_hcd->lock, flags);
		isp1362_write_reg32(isp1362_hcd, HCRHPORT1 + wIndex, tmp);
		isp1362_hcd->rhport[wIndex] =
			isp1362_read_reg32(isp1362_hcd, HCRHPORT1 + wIndex);
		spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
		break;
	case SetPortFeature:
		DBG(0, "SetPortFeature: ");
		if (!wIndex || wIndex > ports) {
			goto error;
		}
		wIndex--;
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			_DBG(0, "USB_PORT_FEAT_SUSPEND\n");
#ifdef	CONFIG_USB_OTG
			if (ohci->hcd.self.otg_port == (wIndex + 1) &&
			    ohci->hcd.self.b_hnp_enable) {
				start_hnp(ohci);
				break;
			}
#endif
			spin_lock_irqsave(&isp1362_hcd->lock, flags);
			isp1362_write_reg32(isp1362_hcd, HCRHPORT1 + wIndex, RH_PS_PSS);
			isp1362_hcd->rhport[wIndex] =
				isp1362_read_reg32(isp1362_hcd, HCRHPORT1 + wIndex);
			spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
			break;
		case USB_PORT_FEAT_POWER:
			_DBG(0, "USB_PORT_FEAT_POWER\n");
			spin_lock_irqsave(&isp1362_hcd->lock, flags);
			isp1362_write_reg32(isp1362_hcd, HCRHPORT1 + wIndex, RH_PS_PPS);
			isp1362_hcd->rhport[wIndex] =
				isp1362_read_reg32(isp1362_hcd, HCRHPORT1 + wIndex);
			spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
			break;
		case USB_PORT_FEAT_RESET:
			_DBG(0, "USB_PORT_FEAT_RESET\n");
			spin_lock_irqsave(&isp1362_hcd->lock, flags);

			t1 = jiffies + msecs_to_jiffies(USB_RESET_WIDTH);
			while (time_before(jiffies, t1)) {
				// spin until any current reset finishes
				for (;;) {
					tmp = isp1362_read_reg32(isp1362_hcd, HCRHPORT1 + wIndex);
					if (!(tmp & RH_PS_PRS)) {
						break;
					}
					udelay(500);
				}
				if (!(tmp & RH_PS_CCS)) {
					break;
				}
				// Reset lasts 10ms (claims datasheet)
				isp1362_write_reg32(isp1362_hcd, HCRHPORT1 + wIndex, (RH_PS_PRS));

				spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
				msleep(10);
				spin_lock_irqsave(&isp1362_hcd->lock, flags);
			}

			isp1362_hcd->rhport[wIndex] = isp1362_read_reg32(isp1362_hcd,
									 HCRHPORT1 + wIndex);
			spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
			break;
		default:
			goto error;
		}
		break;

	default:
	error:
		/* "protocol stall" on error */
		_DBG(0, "PROTOCOL STALL\n");
		retval = -EPIPE;
	}

	return retval;
}

#ifdef	CONFIG_PM
static int isp1362_bus_suspend(struct usb_hcd *hcd)
{
	int status = 0;
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	unsigned long flags;

	if (time_before(jiffies, isp1362_hcd->next_statechange)) {
		msleep(5);
	}

	spin_lock_irqsave(&isp1362_hcd->lock, flags);

	isp1362_hcd->hc_control = isp1362_read_reg32(isp1362_hcd, HCCONTROL);
	switch (isp1362_hcd->hc_control & OHCI_CTRL_HCFS) {
	case OHCI_USB_RESUME:
		DBG(0, "%s: resume/suspend?\n", __FUNCTION__);
		isp1362_hcd->hc_control &= ~OHCI_CTRL_HCFS;
		isp1362_hcd->hc_control |= OHCI_USB_RESET;
		isp1362_write_reg32(isp1362_hcd, HCCONTROL, isp1362_hcd->hc_control);
		/* FALL THROUGH */
	case OHCI_USB_RESET:
		status = -EBUSY;
		WARN("%s: needs reinit!\n", __FUNCTION__);
		goto done;
	case OHCI_USB_SUSPEND:
		WARN("%s: already suspended?\n", __FUNCTION__);
		goto done;
	}
	DBG(0, "%s: suspend root hub\n", __FUNCTION__);

	/* First stop any processing */
	hcd->state = HC_STATE_QUIESCING;
	if (!list_empty(&isp1362_hcd->atl_queue.active) ||
	    !list_empty(&isp1362_hcd->intl_queue.active) ||
	    !list_empty(&isp1362_hcd->istl_queue[0] .active) ||
	    !list_empty(&isp1362_hcd->istl_queue[1] .active)) {
		int limit;

		isp1362_write_reg32(isp1362_hcd, HCATLSKIP, ~0);
		isp1362_write_reg32(isp1362_hcd, HCINTLSKIP, ~0);
		isp1362_write_reg16(isp1362_hcd, HCBUFSTAT, 0);
		isp1362_write_reg16(isp1362_hcd, HCuPINTENB, 0);
		isp1362_write_reg32(isp1362_hcd, HCINTSTAT, OHCI_INTR_SF);

		DBG(0, "%s: stopping schedules ...\n", __FUNCTION__);
		limit = 2000;
		while (limit > 0) {
			udelay(250);
			limit =- 250;
			if (isp1362_read_reg32(isp1362_hcd, HCINTSTAT) & OHCI_INTR_SF) {
				break;
			}
		}
		mdelay(7);
		if (isp1362_read_reg16(isp1362_hcd, HCuPINT) & HCuPINT_ATL) {
			u32 done_map = isp1362_read_reg32(isp1362_hcd, HCATLDONE);
			finish_transfers(isp1362_hcd, done_map, &isp1362_hcd->atl_queue, NULL);
		}
		if (isp1362_read_reg16(isp1362_hcd, HCuPINT) & HCuPINT_INTL) {
			u32 done_map = isp1362_read_reg32(isp1362_hcd, HCINTLDONE);
			finish_transfers(isp1362_hcd, done_map, &isp1362_hcd->intl_queue, NULL);
		}
		if (isp1362_read_reg16(isp1362_hcd, HCuPINT) & HCuPINT_ISTL0) {
			finish_iso_transfers(isp1362_hcd, &isp1362_hcd->istl_queue[0] , NULL);
		}
		if (isp1362_read_reg16(isp1362_hcd, HCuPINT) & HCuPINT_ISTL1) {
			finish_iso_transfers(isp1362_hcd, &isp1362_hcd->istl_queue[1], NULL);
		}
	}
	DBG(0, "%s: HCINTSTAT: %08x\n", __FUNCTION__,
		    isp1362_read_reg32(isp1362_hcd, HCINTSTAT));
	isp1362_write_reg32(isp1362_hcd, HCINTSTAT,
			    isp1362_read_reg32(isp1362_hcd, HCINTSTAT));

	/* Suspend hub */
	isp1362_hcd->hc_control = OHCI_USB_SUSPEND;
	/* maybe resume can wake root hub */
	if (hcd->remote_wakeup) {
		DBG(0, "%s: Enabling RWE\n", __FUNCTION__);
		isp1362_hcd->hc_control |= OHCI_CTRL_RWE;
	}
	isp1362_show_reg(isp1362_hcd, HCCONTROL);
	isp1362_write_reg32(isp1362_hcd, HCCONTROL, isp1362_hcd->hc_control);
	isp1362_show_reg(isp1362_hcd, HCCONTROL);

#if 1
	isp1362_hcd->hc_control = isp1362_read_reg32(isp1362_hcd, HCCONTROL);
	if ((isp1362_hcd->hc_control & OHCI_CTRL_HCFS) != OHCI_USB_SUSPEND) {
		ERR("%s: controller won't suspend %08x\n", __FUNCTION__,
		    isp1362_hcd->hc_control);
		status = -EBUSY;
	} else {
#else
	if (1) {
#endif
		/* no resumes until devices finish suspending */
		isp1362_hcd->next_statechange = jiffies + msecs_to_jiffies(5);
	}
done:
	if (status == 0) {
		hcd->state = HC_STATE_SUSPENDED;
		DBG(0, "%s: HCD suspended: %08x\n", __FUNCTION__,
		    isp1362_read_reg32(isp1362_hcd, HCCONTROL));
	}
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
	return status;
}

static int isp1362_bus_resume(struct usb_hcd *hcd)
{
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	u32 port;
	unsigned long flags;
	int status = -EINPROGRESS;

	if (time_before(jiffies, isp1362_hcd->next_statechange)) {
		msleep(5);
	}

	spin_lock_irqsave(&isp1362_hcd->lock, flags);
	isp1362_hcd->hc_control = isp1362_read_reg32(isp1362_hcd, HCCONTROL);
	INFO("%s: HCCONTROL: %08x\n", __FUNCTION__, isp1362_hcd->hc_control);
	if (hcd->state == HC_STATE_RESUMING) {
		WARN("%s: duplicate resume\n", __FUNCTION__);
		status = 0;
	} else switch (isp1362_hcd->hc_control & OHCI_CTRL_HCFS) {
	case OHCI_USB_SUSPEND:
		DBG(0, "%s: resume root hub\n", __FUNCTION__);
		isp1362_hcd->hc_control &= ~OHCI_CTRL_HCFS;
		isp1362_hcd->hc_control |= OHCI_USB_RESUME;
		isp1362_write_reg32(isp1362_hcd, HCCONTROL, isp1362_hcd->hc_control);
		break;
	case OHCI_USB_RESUME:
		/* HCFS changes sometime after INTR_RD */
		DBG(0, "%s: remote wakeup\n", __FUNCTION__);
		break;
	case OHCI_USB_OPER:
		DBG(0, "%s: odd resume\n", __FUNCTION__);
		status = 0;
		hcd->self.root_hub->dev.power.power_state = PMSG_ON;
		break;
	default:		/* RESET, we lost power */
		DBG(0, "%s: root hub hardware reset\n", __FUNCTION__);
		status = -EBUSY;
	}
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
	if (status == -EBUSY) {
		DBG(0, "%s: Restarting HC\n", __FUNCTION__);
		isp1362_hc_stop(hcd);
		return isp1362_hc_start(hcd);
	}
	if (status != -EINPROGRESS) {
		return status;
	}
	spin_lock_irqsave(&isp1362_hcd->lock, flags);
	port = isp1362_read_reg32(isp1362_hcd, HCRHDESCA) & RH_A_NDP;
	while (port--) {
		u32 stat = isp1362_read_reg32(isp1362_hcd, HCRHPORT1 + port);

		/* force global, not selective, resume */
		if (!(stat & RH_PS_PSS)) {
			DBG(0, "%s: Not Resuming RH port %d\n", __FUNCTION__, port);
			continue;
		}
		DBG(0, "%s: Resuming RH port %d\n", __FUNCTION__, port);
		isp1362_write_reg32(isp1362_hcd, HCRHPORT1 + port, RH_PS_POCI);
	}
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);

	/* Some controllers (lucent) need extra-long delays */
	hcd->state = HC_STATE_RESUMING;
	mdelay(20 /* usb 11.5.1.10 */ + 15);

	isp1362_hcd->hc_control = OHCI_USB_OPER;
	if (hcd->can_wakeup) {
		isp1362_hcd->hc_control |= OHCI_CTRL_RWC;
	}
	spin_lock_irqsave(&isp1362_hcd->lock, flags);
	isp1362_show_reg(isp1362_hcd, HCCONTROL);
	isp1362_write_reg32(isp1362_hcd, HCCONTROL, isp1362_hcd->hc_control);
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
	/* TRSMRCY */
	msleep(10);

	/* keep it alive for ~5x suspend + resume costs */
	isp1362_hcd->next_statechange = jiffies + msecs_to_jiffies(250);

	hcd->self.root_hub->dev.power.power_state = PMSG_ON;
	hcd->state = HC_STATE_RUNNING;
	return 0;
}
#else
#define	isp1362_bus_suspend	NULL
#define	isp1362_bus_resume	NULL
#endif

/*-------------------------------------------------------------------------*/

#ifdef STUB_DEBUG_FILE

static inline void create_debug_file(struct isp1362_hcd *isp1362_hcd)
{
}
static inline void remove_debug_file(struct isp1362_hcd *isp1362_hcd)
{
}

#else

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static void dump_irq(struct seq_file *s, char *label, u16 mask)
{
	seq_printf(s, "%-15s %04x%s%s%s%s%s%s\n", label, mask,
		   mask & HCuPINT_CLKRDY ? " clkrdy" : "",
		   mask & HCuPINT_SUSP ? " susp" : "",
		   mask & HCuPINT_OPR ? " opr" : "",
		   mask & HCuPINT_EOT ? " eot" : "",
		   mask & HCuPINT_ATL ? " atl" : "",
		   mask & HCuPINT_SOF ? " sof" : "");
}

static void dump_int(struct seq_file *s, char *label, u32 mask)
{
	seq_printf(s, "%-15s %08x%s%s%s%s%s%s%s\n", label, mask,
		   mask & OHCI_INTR_MIE ? " MIE" : "",
		   mask & OHCI_INTR_RHSC ? " rhsc" : "",
		   mask & OHCI_INTR_FNO ? " fno" : "",
		   mask & OHCI_INTR_UE ? " ue" : "",
		   mask & OHCI_INTR_RD ? " rd" : "",
		   mask & OHCI_INTR_SF ? " sof" : "",
		   mask & OHCI_INTR_SO ? " so" : "");
}

static void dump_ctrl(struct seq_file *s, char *label, u32 mask)
{
	seq_printf(s, "%-15s %08x%s%s%s\n", label, mask,
		   mask & OHCI_CTRL_RWC ? " rwc" : "",
		   mask & OHCI_CTRL_RWE ? " rwe" : "",
		   ({
			   char *hcfs;
			   switch (mask & OHCI_CTRL_HCFS) {
			   case OHCI_USB_OPER:
				   hcfs = " oper";
				   break;
			   case OHCI_USB_RESET:
				   hcfs = " reset";
				   break;
			   case OHCI_USB_RESUME:
				   hcfs = " resume";
				   break;
			   case OHCI_USB_SUSPEND:
				   hcfs = " suspend";
				   break;
			   default:
				   hcfs = " ?";
			   }
			   hcfs;
		   }));
}

static void dump_regs(struct seq_file *s, struct isp1362_hcd *isp1362_hcd)
{
	seq_printf(s, "HCREVISION [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCREVISION),
		   isp1362_read_reg32(isp1362_hcd, HCREVISION));
	seq_printf(s, "HCCONTROL  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCCONTROL),
		   isp1362_read_reg32(isp1362_hcd, HCCONTROL));
	seq_printf(s, "HCCMDSTAT  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCCMDSTAT),
		   isp1362_read_reg32(isp1362_hcd, HCCMDSTAT));
	seq_printf(s, "HCINTSTAT  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCINTSTAT),
		   isp1362_read_reg32(isp1362_hcd, HCINTSTAT));
	seq_printf(s, "HCINTENB   [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCINTENB),
		   isp1362_read_reg32(isp1362_hcd, HCINTENB));
	seq_printf(s, "HCFMINTVL  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCFMINTVL),
		   isp1362_read_reg32(isp1362_hcd, HCFMINTVL));
	seq_printf(s, "HCFMREM    [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCFMREM),
		   isp1362_read_reg32(isp1362_hcd, HCFMREM));
	seq_printf(s, "HCFMNUM    [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCFMNUM),
		   isp1362_read_reg32(isp1362_hcd, HCFMNUM));
	seq_printf(s, "HCLSTHRESH [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCLSTHRESH),
		   isp1362_read_reg32(isp1362_hcd, HCLSTHRESH));
	seq_printf(s, "HCRHDESCA  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCRHDESCA),
		   isp1362_read_reg32(isp1362_hcd, HCRHDESCA));
	seq_printf(s, "HCRHDESCB  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCRHDESCB),
		   isp1362_read_reg32(isp1362_hcd, HCRHDESCB));
	seq_printf(s, "HCRHSTATUS [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCRHSTATUS),
		   isp1362_read_reg32(isp1362_hcd, HCRHSTATUS));
	seq_printf(s, "HCRHPORT1  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCRHPORT1),
		   isp1362_read_reg32(isp1362_hcd, HCRHPORT1));
	seq_printf(s, "HCRHPORT2  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCRHPORT2),
		   isp1362_read_reg32(isp1362_hcd, HCRHPORT2));
	seq_printf(s, "\n");
	seq_printf(s, "HCHWCFG    [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCHWCFG),
		   isp1362_read_reg16(isp1362_hcd, HCHWCFG));
	seq_printf(s, "HCDMACFG   [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCDMACFG),
		   isp1362_read_reg16(isp1362_hcd, HCDMACFG));
	seq_printf(s, "HCXFERCTR  [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCXFERCTR),
		   isp1362_read_reg16(isp1362_hcd, HCXFERCTR));
	seq_printf(s, "HCuPINT    [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCuPINT),
		   isp1362_read_reg16(isp1362_hcd, HCuPINT));
	seq_printf(s, "HCuPINTENB [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCuPINTENB),
		   isp1362_read_reg16(isp1362_hcd, HCuPINTENB));
	seq_printf(s, "HCCHIPID   [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCCHIPID),
		   isp1362_read_reg16(isp1362_hcd, HCCHIPID));
	seq_printf(s, "HCSCRATCH  [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCSCRATCH),
		   isp1362_read_reg16(isp1362_hcd, HCSCRATCH));
	seq_printf(s, "HCBUFSTAT  [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCBUFSTAT),
		   isp1362_read_reg16(isp1362_hcd, HCBUFSTAT));
	seq_printf(s, "HCDIRADDR  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCDIRADDR),
		   isp1362_read_reg32(isp1362_hcd, HCDIRADDR));
#if 0
	seq_printf(s, "HCDIRDATA  [%02x]     %04x\n", ISP1362_REG_NO(HCDIRDATA),
		   isp1362_read_reg16(isp1362_hcd, HCDIRDATA));
#endif
	seq_printf(s, "HCISTLBUFSZ[%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCISTLBUFSZ),
		   isp1362_read_reg16(isp1362_hcd, HCISTLBUFSZ));
	seq_printf(s, "HCISTLRATE [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCISTLRATE),
		   isp1362_read_reg16(isp1362_hcd, HCISTLRATE));
	seq_printf(s, "\n");
	seq_printf(s, "HCINTLBUFSZ[%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCINTLBUFSZ),
		   isp1362_read_reg16(isp1362_hcd, HCINTLBUFSZ));
	seq_printf(s, "HCINTLBLKSZ[%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCINTLBLKSZ),
		   isp1362_read_reg16(isp1362_hcd, HCINTLBLKSZ));
	seq_printf(s, "HCINTLDONE [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCINTLDONE),
		   isp1362_read_reg32(isp1362_hcd, HCINTLDONE));
	seq_printf(s, "HCINTLSKIP [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCINTLSKIP),
		   isp1362_read_reg32(isp1362_hcd, HCINTLSKIP));
	seq_printf(s, "HCINTLLAST [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCINTLLAST),
		   isp1362_read_reg32(isp1362_hcd, HCINTLLAST));
	seq_printf(s, "HCINTLCURR [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCINTLCURR),
		   isp1362_read_reg16(isp1362_hcd, HCINTLCURR));
	seq_printf(s, "\n");
	seq_printf(s, "HCATLBUFSZ [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCATLBUFSZ),
		   isp1362_read_reg16(isp1362_hcd, HCATLBUFSZ));
	seq_printf(s, "HCATLBLKSZ [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCATLBLKSZ),
		   isp1362_read_reg16(isp1362_hcd, HCATLBLKSZ));
#if 0
	seq_printf(s, "HCATLDONE  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCATLDONE),
		   isp1362_read_reg32(isp1362_hcd, HCATLDONE));
#endif
	seq_printf(s, "HCATLSKIP  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCATLSKIP),
		   isp1362_read_reg32(isp1362_hcd, HCATLSKIP));
	seq_printf(s, "HCATLLAST  [%02x] %08x\n", ISP1362_REG_NO(ISP1362_REG_HCATLLAST),
		   isp1362_read_reg32(isp1362_hcd, HCATLLAST));
	seq_printf(s, "HCATLCURR  [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCATLCURR),
		   isp1362_read_reg16(isp1362_hcd, HCATLCURR));
	seq_printf(s, "\n");
	seq_printf(s, "HCATLDTC   [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCATLDTC),
		   isp1362_read_reg16(isp1362_hcd, HCATLDTC));
	seq_printf(s, "HCATLDTCTO [%02x]     %04x\n", ISP1362_REG_NO(ISP1362_REG_HCATLDTCTO),
		   isp1362_read_reg16(isp1362_hcd, HCATLDTCTO));
}

static int proc_isp1362_show(struct seq_file *s, void *unused)
{
	struct isp1362_hcd *isp1362_hcd = s->private;
	struct isp1362_ep *ep;
	int i;

	seq_printf(s, "%s\n%s version %s\n",
		   isp1362_hcd_to_hcd(isp1362_hcd)->product_desc, hcd_name, DRIVER_VERSION);

	/* collect statistics to help estimate potential win for
	 * DMA engines that care about alignment (PXA)
	 */
	seq_printf(s, "alignment:  16b/%ld 8b/%ld 4b/%ld 2b/%ld 1b/%ld\n",
		   isp1362_hcd->stat16, isp1362_hcd->stat8, isp1362_hcd->stat4,
		   isp1362_hcd->stat2, isp1362_hcd->stat1);
	seq_printf(s, "max # ptds in ATL  fifo: %d\n", isp1362_hcd->atl_queue.stat_maxptds);
	seq_printf(s, "max # ptds in INTL fifo: %d\n", isp1362_hcd->intl_queue.stat_maxptds);
	seq_printf(s, "max # ptds in ISTL fifo: %d\n",
		   max(isp1362_hcd->istl_queue[0] .stat_maxptds,
		       isp1362_hcd->istl_queue[1] .stat_maxptds));

	// FIXME: don't show the following in suspended state
	spin_lock_irq(&isp1362_hcd->lock);

	dump_irq(s, "hc_irq_enable", isp1362_read_reg16(isp1362_hcd, HCuPINTENB));
	dump_irq(s, "hc_irq_status", isp1362_read_reg16(isp1362_hcd, HCuPINT));
	dump_int(s, "ohci_int_enable", isp1362_read_reg32(isp1362_hcd, HCINTENB));
	dump_int(s, "ohci_int_status", isp1362_read_reg32(isp1362_hcd, HCINTSTAT));
	dump_ctrl(s, "ohci_control", isp1362_read_reg32(isp1362_hcd, HCCONTROL));

	for (i = 0; i < NUM_ISP1362_IRQS; i++) {
		if (isp1362_hcd->irq_stat[i]) {
			seq_printf(s, "%-15s: %d\n",
				   ISP1362_INT_NAME(i), isp1362_hcd->irq_stat[i]);
		}
	}

	dump_regs(s, isp1362_hcd);
	list_for_each_entry(ep, &isp1362_hcd->async, schedule) {
		struct urb *urb;

		seq_printf(s, "%p, ep%d%s, maxpacket %d:\n", ep, ep->epnum,
			   ({
				   char *s;
				   switch (ep->nextpid) {
				   case USB_PID_IN:
					   s = "in";
					   break;
				   case USB_PID_OUT:
					   s = "out";
					   break;
				   case USB_PID_SETUP:
					   s = "setup";
					   break;
				   case USB_PID_ACK:
					   s = "status";
					   break;
				   default:
					   s = "?";
					   break;
				   };
				   s;}), ep->maxpacket) ;
		list_for_each_entry(urb, &ep->hep->urb_list, urb_list) {
			seq_printf(s, "  urb%p, %d/%d\n", urb,
				   urb->actual_length,
				   urb->transfer_buffer_length);
		}
	}
	if (!list_empty(&isp1362_hcd->async)) {
		seq_printf(s, "\n");
	}
	dump_ptd_queue(&isp1362_hcd->atl_queue);

	seq_printf(s, "periodic size= %d\n", PERIODIC_SIZE);

	list_for_each_entry(ep, &isp1362_hcd->periodic, schedule) {
		seq_printf(s, "branch:%2d load:%3d PTD[%d] $%04x:\n", ep->branch,
			   isp1362_hcd->load[ep->branch], ep->ptd_index, ep->ptd_offset);

		seq_printf(s, "   %d/%p (%sdev%d ep%d%s max %d)\n",
			   ep->interval, ep,
			   (ep->udev->speed == USB_SPEED_FULL) ? "" : "ls ",
			   ep->udev->devnum, ep->epnum,
			   (ep->epnum == 0) ? "" :
			   ((ep->nextpid == USB_PID_IN) ?
			    "in" : "out"), ep->maxpacket);
	}
	dump_ptd_queue(&isp1362_hcd->intl_queue);

	seq_printf(s, "ISO:\n");

	list_for_each_entry(ep, &isp1362_hcd->isoc, schedule) {
		seq_printf(s, "   %d/%p (%sdev%d ep%d%s max %d)\n",
			   ep->interval, ep,
			   (ep->udev->speed == USB_SPEED_FULL) ? "" : "ls ",
			   ep->udev->devnum, ep->epnum,
			   (ep->epnum == 0) ? "" :
			   ((ep->nextpid == USB_PID_IN) ?
			    "in" : "out"), ep->maxpacket);
	}

	spin_unlock_irq(&isp1362_hcd->lock);
	seq_printf(s, "\n");

	return 0;
}

static int proc_isp1362_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_isp1362_show, PDE(inode)->data);
}

static struct file_operations proc_ops = {
	.open = proc_isp1362_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* expect just one isp1362_hcd per system */
static const char proc_filename[] = "driver/isp1362";

static void create_debug_file(struct isp1362_hcd *isp1362_hcd)
{
	struct proc_dir_entry *pde;

	pde = create_proc_entry(proc_filename, 0, NULL);
	if (pde == NULL) {
		WARN("%s: Failed to create debug file '%s'\n", __FUNCTION__, proc_filename);
		return;
	}

	pde->proc_fops = &proc_ops;
	pde->data = isp1362_hcd;
	isp1362_hcd->pde = pde;
}

static void remove_debug_file(struct isp1362_hcd *isp1362_hcd)
{
	if (isp1362_hcd->pde) {
		remove_proc_entry(proc_filename, 0);
	}
}

#endif

/*-------------------------------------------------------------------------*/

static void isp1362_sw_reset(struct isp1362_hcd *isp1362_hcd)
{
	int tmp = 20;
	unsigned long flags;

	spin_lock_irqsave(&isp1362_hcd->lock, flags);

	isp1362_write_reg16(isp1362_hcd, HCSWRES, HCSWRES_MAGIC);
	isp1362_write_reg32(isp1362_hcd, HCCMDSTAT, OHCI_HCR);
	while (--tmp) {
		mdelay(1);
		if (!(isp1362_read_reg32(isp1362_hcd, HCCMDSTAT) & OHCI_HCR)) {
			break;
		}
	}
	if (!tmp) {
		ERR("Software reset timeout\n");
	}
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
}

static int isp1362_mem_config(struct usb_hcd *hcd)
{
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	unsigned long flags;
	u32 total;
	u16 istl_size = ISP1362_ISTL_BUFSIZE;
	u16 intl_blksize = ISP1362_INTL_BLKSIZE + PTD_HEADER_SIZE;
	u16 intl_size = ISP1362_INTL_BUFFERS * intl_blksize;
	u16 atl_blksize = ISP1362_ATL_BLKSIZE + PTD_HEADER_SIZE;
	u16 atl_buffers = (ISP1362_BUF_SIZE - (istl_size + intl_size)) / atl_blksize;
	u16 atl_size;
	int i;

	WARN_ON(istl_size & 3);
	WARN_ON(atl_blksize & 3);
	WARN_ON(intl_blksize & 3);
	WARN_ON(atl_blksize < PTD_HEADER_SIZE);
	WARN_ON(intl_blksize < PTD_HEADER_SIZE);

	BUG_ON((unsigned)ISP1362_INTL_BUFFERS > 32);
	if (atl_buffers > 32) {
		atl_buffers = 32;
	}
	atl_size = atl_buffers * atl_blksize;
	total = atl_size + intl_size + istl_size;
	dev_info(hcd->self.controller, "ISP1362 Memory usage:\n");
	dev_info(hcd->self.controller, "  ISTL:    2 * %4d:     %4d @ $%04x:$%04x\n",
		 istl_size / 2, istl_size, 0, istl_size / 2);
	dev_info(hcd->self.controller, "  INTL: %4d * (%3d+8):  %4d @ $%04x\n",
		 ISP1362_INTL_BUFFERS, intl_blksize - PTD_HEADER_SIZE,
		 intl_size, istl_size);
	dev_info(hcd->self.controller, "  ATL : %4d * (%3d+8):  %4d @ $%04x\n",
		 atl_buffers, atl_blksize - PTD_HEADER_SIZE,
		 atl_size, istl_size + intl_size);
	dev_info(hcd->self.controller, "  USED/FREE:   %4d      %4d\n", total,
		 ISP1362_BUF_SIZE - total);

	if (total > ISP1362_BUF_SIZE) {
		dev_err(hcd->self.controller, "%s: Memory requested: %d, available %d\n",
			__FUNCTION__, total, ISP1362_BUF_SIZE);
		return -ENOMEM;
	}

	total = istl_size + intl_size + atl_size;
	spin_lock_irqsave(&isp1362_hcd->lock, flags);

	for (i = 0; i < 2; i++) {
		isp1362_hcd->istl_queue[i].buf_start = i * istl_size / 2,
		isp1362_hcd->istl_queue[i].buf_size = istl_size / 2;
		isp1362_hcd->istl_queue[i].blk_size = 4;
		INIT_LIST_HEAD(&isp1362_hcd->istl_queue[i].active);
		snprintf(isp1362_hcd->istl_queue[i].name,
			 sizeof(isp1362_hcd->istl_queue[i].name), "ISTL%d", i);
		DBG(3, "%s: %5s buf $%04x %d\n", __FUNCTION__,
		     isp1362_hcd->istl_queue[i].name,
		     isp1362_hcd->istl_queue[i].buf_start,
		     isp1362_hcd->istl_queue[i].buf_size);
	}
	isp1362_write_reg16(isp1362_hcd, HCISTLBUFSZ, istl_size / 2);

	isp1362_hcd->intl_queue.buf_start = istl_size;
	isp1362_hcd->intl_queue.buf_size = intl_size;
	isp1362_hcd->intl_queue.buf_count = ISP1362_INTL_BUFFERS;
	isp1362_hcd->intl_queue.blk_size = intl_blksize;
	isp1362_hcd->intl_queue.buf_avail = isp1362_hcd->intl_queue.buf_count;
	isp1362_hcd->intl_queue.skip_map = ~0;
	INIT_LIST_HEAD(&isp1362_hcd->intl_queue.active);

	isp1362_write_reg16(isp1362_hcd, HCINTLBUFSZ,
			    isp1362_hcd->intl_queue.buf_size);
	isp1362_write_reg16(isp1362_hcd, HCINTLBLKSZ,
			    isp1362_hcd->intl_queue.blk_size - PTD_HEADER_SIZE);
	isp1362_write_reg32(isp1362_hcd, HCINTLSKIP, ~0);
	isp1362_write_reg32(isp1362_hcd, HCINTLLAST,
			    1 << (ISP1362_INTL_BUFFERS - 1));

	isp1362_hcd->atl_queue.buf_start = istl_size + intl_size;
	isp1362_hcd->atl_queue.buf_size = atl_size;
	isp1362_hcd->atl_queue.buf_count = atl_buffers;
	isp1362_hcd->atl_queue.blk_size = atl_blksize;
	isp1362_hcd->atl_queue.buf_avail = isp1362_hcd->atl_queue.buf_count;
	isp1362_hcd->atl_queue.skip_map = ~0;
	INIT_LIST_HEAD(&isp1362_hcd->atl_queue.active);

	isp1362_write_reg16(isp1362_hcd, HCATLBUFSZ,
			    isp1362_hcd->atl_queue.buf_size);
	isp1362_write_reg16(isp1362_hcd, HCATLBLKSZ,
			    isp1362_hcd->atl_queue.blk_size - PTD_HEADER_SIZE);
	isp1362_write_reg32(isp1362_hcd, HCATLSKIP, ~0);
	isp1362_write_reg32(isp1362_hcd, HCATLLAST,
			    1 << (atl_buffers - 1));

	snprintf(isp1362_hcd->atl_queue.name,
		 sizeof(isp1362_hcd->atl_queue.name), "ATL");
	snprintf(isp1362_hcd->intl_queue.name,
		 sizeof(isp1362_hcd->intl_queue.name), "INTL");
	DBG(3, "%s: %5s buf $%04x %2d * %4d = %4d\n", __FUNCTION__,
	     isp1362_hcd->intl_queue.name,
	     isp1362_hcd->intl_queue.buf_start,
	     ISP1362_INTL_BUFFERS, isp1362_hcd->intl_queue.blk_size,
	     isp1362_hcd->intl_queue.buf_size);
	DBG(3, "%s: %5s buf $%04x %2d * %4d = %4d\n", __FUNCTION__,
	     isp1362_hcd->atl_queue.name,
	     isp1362_hcd->atl_queue.buf_start,
	     atl_buffers, isp1362_hcd->atl_queue.blk_size,
	     isp1362_hcd->atl_queue.buf_size);

	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);

	return 0;
}

static int isp1362_hc_reset(struct usb_hcd *hcd)
{
	int ret = 0;
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	unsigned long t;
	unsigned long timeout = 20;
	unsigned long flags;
	int clkrdy = 0;

	INFO("%s:\n", __FUNCTION__);

	if (isp1362_hcd->board && isp1362_hcd->board->reset) {
		isp1362_hcd->board->reset(hcd->self.controller, 1);
		msleep(20);
		if (isp1362_hcd->board->clock) {
			isp1362_hcd->board->clock(hcd->self.controller, 1);
		}
		isp1362_hcd->board->reset(hcd->self.controller, 0);
	} else {
		isp1362_sw_reset(isp1362_hcd);
	}

	// chip has been reset. First we need to see a clock
	t = jiffies + msecs_to_jiffies(timeout);
	while (!clkrdy && time_before_eq(jiffies, t)) {
		spin_lock_irqsave(&isp1362_hcd->lock, flags);
		clkrdy = isp1362_read_reg16(isp1362_hcd, HCuPINT) & HCuPINT_CLKRDY;
		spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
		if (!clkrdy) {
			msleep(4);
		}
	}

	spin_lock_irqsave(&isp1362_hcd->lock, flags);
	isp1362_write_reg16(isp1362_hcd, HCuPINT, HCuPINT_CLKRDY);
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
	if (!clkrdy) {
		ERR("Clock not ready after %lums\n", timeout);
		ret = -ENODEV;
	}
	return ret;
}

static void isp1362_hc_stop(struct usb_hcd *hcd)
{
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	unsigned long flags;
	u32 tmp;

	INFO("%s:\n", __FUNCTION__);

	del_timer_sync(&hcd->rh_timer);

	spin_lock_irqsave(&isp1362_hcd->lock, flags);

	isp1362_write_reg16(isp1362_hcd, HCuPINTENB, 0);

	// Switch off power for all ports
	tmp = isp1362_read_reg32(isp1362_hcd, HCRHDESCA);
	tmp &= ~(RH_A_NPS | RH_A_PSM);
	isp1362_write_reg32(isp1362_hcd, HCRHDESCA, tmp);
	isp1362_write_reg32(isp1362_hcd, HCRHSTATUS, RH_HS_LPS);

	// Reset the chip
	if (isp1362_hcd->board && isp1362_hcd->board->reset) {
		isp1362_hcd->board->reset(hcd->self.controller, 1);
	} else {
		isp1362_sw_reset(isp1362_hcd);
	}
	if (isp1362_hcd->board && isp1362_hcd->board->clock) {
		isp1362_hcd->board->clock(hcd->self.controller, 0);
	}
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
}

#ifdef CHIP_BUFFER_TEST
static int isp1362_chip_test(struct isp1362_hcd *isp1362_hcd)
{
	int ret = 0;
	u16 *ref;
	unsigned long flags;

	ref = kmalloc(2 * ISP1362_BUF_SIZE, GFP_KERNEL);
	if (ref) {
		int offset;
		u16 *tst = &ref[ISP1362_BUF_SIZE / 2];

		for (offset = 0; offset < ISP1362_BUF_SIZE / 2; offset++) {
			ref[offset] = ~offset;
			tst[offset] = offset;
		}

		for (offset = 0; offset < 4; offset++) {
			int j;

			for (j = 0; j < 8; j++) {
				spin_lock_irqsave(&isp1362_hcd->lock, flags);
				isp1362_write_buffer(isp1362_hcd, (u8*)ref + offset, 0, j);
				isp1362_read_buffer(isp1362_hcd, (u8*)tst + offset, 0, j);
				spin_unlock_irqrestore(&isp1362_hcd->lock, flags);

				if (memcmp(ref, tst, j)) {
					ret = -ENODEV;
					ERR("%s: memory check with %d byte offset %d failed\n",
					    __FUNCTION__, j, offset);
					dump_data((u8*)ref + offset, j);
					dump_data((u8*)tst + offset, j);
				}
			}
		}

		spin_lock_irqsave(&isp1362_hcd->lock, flags);
		isp1362_write_buffer(isp1362_hcd, ref, 0, ISP1362_BUF_SIZE);
		isp1362_read_buffer(isp1362_hcd, tst, 0, ISP1362_BUF_SIZE);
		spin_unlock_irqrestore(&isp1362_hcd->lock, flags);

		if (memcmp(ref, tst, ISP1362_BUF_SIZE)) {
			ret = -ENODEV;
			ERR("%s: memory check failed\n", __FUNCTION__);
			dump_data((u8*)tst, ISP1362_BUF_SIZE / 2);
		}

		for (offset = 0; offset < 256; offset++) {
			int test_size = 0;

			yield();

			memset(tst, 0, ISP1362_BUF_SIZE);
			spin_lock_irqsave(&isp1362_hcd->lock, flags);
			isp1362_write_buffer(isp1362_hcd, tst, 0, ISP1362_BUF_SIZE);
			isp1362_read_buffer(isp1362_hcd, tst, 0, ISP1362_BUF_SIZE);
			spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
			if (memcmp(tst, tst + (ISP1362_BUF_SIZE / (2 * sizeof(*tst))),
				   ISP1362_BUF_SIZE / 2)) {
				ERR("%s: Failed to clear buffer\n", __FUNCTION__);
				dump_data((u8*)tst, ISP1362_BUF_SIZE);
				break;
			}
			spin_lock_irqsave(&isp1362_hcd->lock, flags);
			isp1362_write_buffer(isp1362_hcd, ref, offset * 2, PTD_HEADER_SIZE);
			isp1362_write_buffer(isp1362_hcd, ref + PTD_HEADER_SIZE / sizeof(*ref),
					     offset * 2 + PTD_HEADER_SIZE, test_size);
			isp1362_read_buffer(isp1362_hcd, tst, offset * 2,
					    PTD_HEADER_SIZE + test_size);
			spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
			if (memcmp(ref, tst, PTD_HEADER_SIZE + test_size)) {
				dump_data(((u8*)ref) + offset, PTD_HEADER_SIZE + test_size);
				dump_data((u8*)tst, PTD_HEADER_SIZE + test_size);
				spin_lock_irqsave(&isp1362_hcd->lock, flags);
				isp1362_read_buffer(isp1362_hcd, tst, offset * 2,
						    PTD_HEADER_SIZE + test_size);
				spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
				if (memcmp(ref, tst, PTD_HEADER_SIZE + test_size)) {
					ret = -ENODEV;
					ERR("%s: memory check with offset %02x failed\n",
					    __FUNCTION__, offset);
					break;
				}
				WARN("%s: memory check with offset %02x ok after second read\n",
				     __FUNCTION__, offset);
			}
		}
		kfree(ref);
	}
	return ret;
}
#endif

static int isp1362_hc_start(struct usb_hcd *hcd)
{
	int ret;
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	struct isp1362_platform_data *board = isp1362_hcd->board;
	struct usb_device *udev;
	u16 hwcfg;
	u16 chipid;
	unsigned long flags;

	INFO("%s:\n", __FUNCTION__);

	spin_lock_irqsave(&isp1362_hcd->lock, flags);
	chipid = isp1362_read_reg16(isp1362_hcd, HCCHIPID);
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);

	if ((chipid & HCCHIPID_MASK) != HCCHIPID_MAGIC) {
		ERR("%s: Invalid chip ID %04x\n", __func__, chipid);
		return -ENODEV;
	}

#ifdef CHIP_BUFFER_TEST
	ret = isp1362_chip_test(isp1362_hcd);
	if (ret) {
		return -ENODEV;
	}
#endif
	spin_lock_irqsave(&isp1362_hcd->lock, flags);
	// clear interrupt status and disable all interrupt sources
	isp1362_write_reg16(isp1362_hcd, HCuPINT, 0xff);
	isp1362_write_reg16(isp1362_hcd, HCuPINTENB, 0);

	// HW conf
	hwcfg = HCHWCFG_INT_ENABLE | HCHWCFG_DBWIDTH(1);
	if (board->sel15Kres) {
		hwcfg |= HCHWCFG_PULLDOWN_DS2 |
			(MAX_ROOT_PORTS > 1) ? HCHWCFG_PULLDOWN_DS1 : 0;
	}
	if (board->clknotstop) {
		hwcfg |= HCHWCFG_CLKNOTSTOP;
	}
	if (board->oc_enable) {
		hwcfg |= HCHWCFG_ANALOG_OC;
	}
	if (board->int_act_high) {
		hwcfg |= HCHWCFG_INT_POL;
	}
	if (board->int_edge_triggered) {
		hwcfg |= HCHWCFG_INT_TRIGGER;
	}
	if (board->dreq_act_high) {
		hwcfg |= HCHWCFG_DREQ_POL;
	}
	if (board->dack_act_high) {
		hwcfg |= HCHWCFG_DACK_POL;
	}
	isp1362_write_reg16(isp1362_hcd, HCHWCFG, hwcfg);
	isp1362_show_reg(isp1362_hcd, HCHWCFG);
	isp1362_write_reg16(isp1362_hcd, HCDMACFG, 0);
	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);

	ret = isp1362_mem_config(hcd);
	if (ret) {
		return ret;
	}

	spin_lock_irqsave(&isp1362_hcd->lock, flags);

	// Root hub conf
	isp1362_hcd->rhdesca = 0;
	if (board->no_power_switching) {
		isp1362_hcd->rhdesca |= RH_A_NPS;
	}
	if (board->power_switching_mode) {
		isp1362_hcd->rhdesca |= RH_A_PSM;
	}
	if (board->potpg) {
		isp1362_hcd->rhdesca |= (board->potpg << 24) & RH_A_POTPGT;
	} else {
		isp1362_hcd->rhdesca |= (25 << 24) & RH_A_POTPGT;
	}

	isp1362_write_reg32(isp1362_hcd, HCRHDESCA, isp1362_hcd->rhdesca & ~RH_A_OCPM);
	isp1362_write_reg32(isp1362_hcd, HCRHDESCA, isp1362_hcd->rhdesca | RH_A_OCPM);
	isp1362_hcd->rhdesca = isp1362_read_reg32(isp1362_hcd, HCRHDESCA);

	isp1362_hcd->rhdescb = RH_B_PPCM;
	isp1362_write_reg32(isp1362_hcd, HCRHDESCB, isp1362_hcd->rhdescb);
	isp1362_hcd->rhdescb = isp1362_read_reg32(isp1362_hcd, HCRHDESCB);

	isp1362_read_reg32(isp1362_hcd, HCFMINTVL);
	isp1362_write_reg32(isp1362_hcd, HCFMINTVL, (FSMP(FI) << 16) | FI);
	isp1362_write_reg32(isp1362_hcd, HCLSTHRESH, LSTHRESH);

	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);

	udev = usb_alloc_dev(NULL, &hcd->self, 0);
	if (!udev) {
		isp1362_hc_stop(hcd);
		return -ENOMEM;
	}

	isp1362_hcd->hc_control = OHCI_USB_OPER;
	if (board->remote_wakeup_connected) {
		hcd->can_wakeup = 1;
		isp1362_hcd->hc_control |= OHCI_CTRL_RWC;
	}

	udev->speed = USB_SPEED_FULL;
	hcd->state = HC_STATE_RUNNING;

	spin_lock_irqsave(&isp1362_hcd->lock, flags);
	// Set up interrupts
	isp1362_hcd->intenb = OHCI_INTR_MIE | OHCI_INTR_RHSC | OHCI_INTR_UE;
	isp1362_hcd->intenb |= OHCI_INTR_RD;
	isp1362_hcd->irqenb = HCuPINT_OPR | HCuPINT_SUSP;
	isp1362_write_reg32(isp1362_hcd, HCINTENB, isp1362_hcd->intenb);
	isp1362_write_reg16(isp1362_hcd, HCuPINTENB, isp1362_hcd->irqenb);

	// Go operational
	isp1362_write_reg32(isp1362_hcd, HCCONTROL, isp1362_hcd->hc_control);
	// enable global power
	isp1362_write_reg32(isp1362_hcd, HCRHSTATUS, RH_HS_LPSC | RH_HS_DRWE);

	spin_unlock_irqrestore(&isp1362_hcd->lock, flags);

	return 0;
}

/*-------------------------------------------------------------------------*/

static struct hc_driver isp1362_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"ISP1362 Host Controller",
	.hcd_priv_size =	sizeof(struct isp1362_hcd),

	.irq =			isp1362_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	.reset =		isp1362_hc_reset,
	.start =		isp1362_hc_start,
	.stop =			isp1362_hc_stop,

	.urb_enqueue =		isp1362_urb_enqueue,
	.urb_dequeue =		isp1362_urb_dequeue,
	.endpoint_disable =	isp1362_endpoint_disable,

	.get_frame_number =	isp1362_get_frame,

	.hub_status_data =	isp1362_hub_status_data,
	.hub_control =		isp1362_hub_control,
	.bus_suspend =		isp1362_bus_suspend,
	.bus_resume =		isp1362_bus_resume,
};

/*-------------------------------------------------------------------------*/

#define resource_len(r) (((r)->end - (r)->start) + 1)

static int __devexit isp1362_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	struct resource *res;

	remove_debug_file(isp1362_hcd);
	DBG(0, "%s: Removing HCD\n", __FUNCTION__);
	usb_remove_hcd(hcd);

	DBG(0, "%s: Unmapping data_reg @ %08x\n", __FUNCTION__,
	    (u32)isp1362_hcd->data_reg);
	iounmap(isp1362_hcd->data_reg);

	DBG(0, "%s: Unmapping addr_reg @ %08x\n", __FUNCTION__,
	    (u32)isp1362_hcd->addr_reg);
	iounmap(isp1362_hcd->addr_reg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	DBG(0, "%s: release mem_region: %08lx\n", __FUNCTION__, res->start);
	if (res) release_mem_region(res->start, resource_len(res));

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	DBG(0, "%s: release mem_region: %08lx\n", __FUNCTION__, res->start);
	if (res) release_mem_region(res->start, resource_len(res));

	DBG(0, "%s: put_hcd\n", __FUNCTION__);
	usb_put_hcd(hcd);
	DBG(0, "%s: Done\n", __FUNCTION__);

	return 0;
}

static int __init isp1362_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct isp1362_hcd *isp1362_hcd;
	struct resource *addr, *data;
	void __iomem *addr_reg;
	void __iomem *data_reg;
	int irq;
	int retval = 0;

	/* basic sanity checks first.  board-specific init logic should
	 * have initialized this the three resources and probably board
	 * specific platform_data.  we don't probe for IRQs, and do only
	 * minimal sanity checking.
	 */
	if (pdev->num_resources < 3) {
		retval = -ENODEV;
		goto err1;
	}

	data = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	irq = platform_get_irq(pdev, 0);
	if (!addr || !data || irq < 0) {
		retval = -ENODEV;
		goto err1;
	}

#ifdef CONFIG_USB_HCD_DMA
	if (pdev->dev.dma_mask) {
		struct resource *dma_res = platform_get_resource(pdev, IORESOURCE_MEM, 2);

		if (!dma_res) {
			retval = -ENODEV;
			goto err1;
		}
		isp1362_hcd->data_dma = dma_res->start;
		isp1362_hcd->max_dma_size = resource_len(dma_res);
	}
#else
	if (pdev->dev.dma_mask) {
		DBG(1, "won't do DMA");
		retval = -ENODEV;
		goto err1;
	}
#endif

	if (!request_mem_region(addr->start, resource_len(addr), hcd_name)) {
		retval = -EBUSY;
		goto err1;
	}
	addr_reg = ioremap(addr->start, resource_len(addr));
	if (addr_reg == NULL) {
		retval = -ENOMEM;
		goto err2;
	}

	if (!request_mem_region(data->start, resource_len(data), hcd_name)) {
		retval = -EBUSY;
		goto err3;
	}
	data_reg = ioremap(data->start, resource_len(data));
	if (data_reg == NULL) {
		retval = -ENOMEM;
		goto err4;
	}

	/* allocate and initialize hcd */
	hcd = usb_create_hcd(&isp1362_hc_driver, &pdev->dev, pdev->dev.bus_id);
	if (!hcd) {
		retval = -ENOMEM;
		goto err5;
	}
	hcd->rsrc_start = data->start;
	isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	isp1362_hcd->data_reg = data_reg;
	isp1362_hcd->addr_reg = addr_reg;

	isp1362_hcd->next_statechange = jiffies;
	spin_lock_init(&isp1362_hcd->lock);
	INIT_LIST_HEAD(&isp1362_hcd->async);
	INIT_LIST_HEAD(&isp1362_hcd->periodic);
	INIT_LIST_HEAD(&isp1362_hcd->isoc);
	INIT_LIST_HEAD(&isp1362_hcd->remove_list);
	isp1362_hcd->board = pdev->dev.platform_data;
#if USE_PLATFORM_DELAY
	if (!isp1362_hcd->board->delay) {
		dev_err(hcd->self.controller, "No platform delay function given\n");
		retval = -ENODEV;
		goto err6;
	}
#endif

#ifdef	CONFIG_ARM
	if (isp1362_hcd->board) {
		set_irq_type(irq, isp1362_hcd->board->int_act_high ? IRQT_RISING : IRQT_FALLING);
	}
#endif

	retval = usb_add_hcd(hcd, irq, IRQF_TRIGGER_LOW | SA_INTERRUPT | SA_SHIRQ);
	if (retval != 0) {
		goto err6;
	}
	INFO("%s, irq %d\n", hcd->product_desc, irq);

	create_debug_file(isp1362_hcd);

	return 0;

 err6:
	DBG(0, "%s: Freeing dev %08x\n", __FUNCTION__, (u32)isp1362_hcd);
	usb_put_hcd(hcd);
 err5:
	DBG(0, "%s: Unmapping data_reg @ %08x\n", __FUNCTION__, (u32)data_reg);
	iounmap(data_reg);
 err4:
	DBG(0, "%s: Releasing mem region %08lx\n", __FUNCTION__, data->start);
	release_mem_region(data->start, resource_len(data));
 err3:
	DBG(0, "%s: Unmapping addr_reg @ %08x\n", __FUNCTION__, (u32)addr_reg);
	iounmap(addr_reg);
 err2:
	DBG(0, "%s: Releasing mem region %08lx\n", __FUNCTION__, addr->start);
	release_mem_region(addr->start, resource_len(addr));
 err1:
	printk("init error, %d\n", retval);

	return retval;
}

#ifdef	CONFIG_PM
static int isp1362_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	unsigned long flags;
	int retval = 0;

	DBG(0, "%s: Suspending device\n", __FUNCTION__);

	if (state.event == PM_EVENT_FREEZE) {
		DBG(0, "%s: Suspending root hub\n", __FUNCTION__);
		retval = isp1362_bus_suspend(hcd);
	} else {
		DBG(0, "%s: Suspending RH ports\n", __FUNCTION__);
		spin_lock_irqsave(&isp1362_hcd->lock, flags);
		isp1362_write_reg32(isp1362_hcd, HCRHSTATUS, RH_HS_LPS);
		spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
	}
	if (retval == 0) {
		pdev->dev.power.power_state = state;
	}
	return retval;
}

static int isp1362_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct isp1362_hcd *isp1362_hcd = hcd_to_isp1362_hcd(hcd);
	unsigned long flags;

	DBG(0, "%s: Resuming\n", __FUNCTION__);

	if (pdev->dev.power.power_state.event == PM_EVENT_SUSPEND
			|| !hcd->can_wakeup) {
		DBG(0, "%s: Resume RH ports\n", __FUNCTION__);
		spin_lock_irqsave(&isp1362_hcd->lock, flags);
		isp1362_write_reg32(isp1362_hcd, HCRHSTATUS, RH_HS_LPSC);
		spin_unlock_irqrestore(&isp1362_hcd->lock, flags);
		return 0;
	}

	pdev->dev.power.power_state = PMSG_ON;

	return isp1362_bus_resume(isp1362_hcd_to_hcd(isp1362_hcd));
}
#else
#define	isp1362_suspend	NULL
#define	isp1362_resume	NULL
#endif

static struct platform_driver isp1362_driver = {
	.probe = isp1362_probe,
	.remove = __devexit_p(isp1362_remove),

	.suspend = isp1362_suspend,
	.resume = isp1362_resume,
	.driver = {
		.name = (char *)hcd_name,
		.owner = THIS_MODULE,
	},
};

/*-------------------------------------------------------------------------*/

static int __init isp1362_init(void)
{
	if (usb_disabled()) {
		return -ENODEV;
	}
	INFO("driver %s, %s\n", hcd_name, DRIVER_VERSION);
	return platform_driver_register(&isp1362_driver);
}

module_init(isp1362_init);

static void __exit isp1362_cleanup(void)
{
	platform_driver_unregister(&isp1362_driver);
}

module_exit(isp1362_cleanup);
