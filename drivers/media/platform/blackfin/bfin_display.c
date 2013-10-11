/*
 * Analog Devices video display driver for (E)PPI interface
 *
 * Copyright (c) 2011 - 2013 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/types.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include <asm/dma.h>

#include <media/blackfin/bfin_display.h>
#include <media/blackfin/ppi.h>

#define DISPLAY_DRV_NAME        "bfin_display"
#define DISP_MIN_NUM_BUF        2

struct bfin_disp_format {
	char *desc;
	u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;
	int bpp; /* bits per pixel */
	int dlen; /* data length for ppi in bits */
};

struct bfin_disp_buffer {
	struct vb2_buffer vb;
	struct list_head list;
};

struct bfin_disp_device {
	/* display device instance */
	struct v4l2_device v4l2_dev;
	/* v4l2 control handler */
	struct v4l2_ctrl_handler ctrl_handler;
	/* device node data */
	struct video_device video_dev;
	/* sub device instance */
	struct v4l2_subdev *sd;
	/* display config */
	struct bfin_display_config *cfg;
	/* ppi interface */
	struct ppi_if *ppi;
	/* current output */
	unsigned int cur_output;
	/* current selected standard */
	v4l2_std_id std;
	/* current selected dv_timings */
	struct v4l2_dv_timings dv_timings;
	/* used to store pixel format */
	struct v4l2_pix_format fmt;
	/* bits per pixel*/
	int bpp;
	/* data length for ppi in bits */
	int dlen;
	/* used to store encoder supported format */
	struct bfin_disp_format *enc_formats;
	/* number of encoder formats array */
	int num_enc_formats;
	/* pointing to current video buffer */
	struct bfin_disp_buffer *cur_frm;
	/* buffer queue used in videobuf2 */
	struct vb2_queue buffer_queue;
	/* allocator-specific contexts for each plane */
	struct vb2_alloc_ctx *alloc_ctx;
	/* queue of filled frames */
	struct list_head dma_queue;
	/* used in videobuf2 callback */
	spinlock_t lock;
	/* used to serialize all ioctls */
	struct mutex mutex;
	/* used to serialize all queuing ioctls */
	struct mutex qlock;
};

static const struct bfin_disp_format bfin_disp_formats[] = {
	{
		.desc        = "UYVY 4:2:2 8bits",
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.mbus_code   = V4L2_MBUS_FMT_UYVY8_2X8,
		.bpp         = 16,
		.dlen        = 8,
	},
	{
		.desc        = "YUYV 4:2:2 8bits",
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.mbus_code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.bpp         = 16,
		.dlen        = 8,
	},
	{
		.desc        = "UYVY 4:2:2 16bits",
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.mbus_code   = V4L2_MBUS_FMT_UYVY8_1X16,
		.bpp         = 16,
		.dlen        = 16,
	},
	{
		.desc        = "RGB 565",
		.pixelformat = V4L2_PIX_FMT_RGB565,
		.mbus_code   = V4L2_MBUS_FMT_RGB565_2X8_LE,
		.bpp         = 16,
		.dlen        = 8,
	},
	{
		.desc        = "RGB 444",
		.pixelformat = V4L2_PIX_FMT_RGB444,
		.mbus_code   = V4L2_MBUS_FMT_RGB444_2X8_PADHI_LE,
		.bpp         = 16,
		.dlen        = 8,
	},

};
#define DISP_MAX_FMTS ARRAY_SIZE(bfin_disp_formats)

static struct bfin_disp_buffer *to_bfin_disp_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct bfin_disp_buffer, vb);
}

static int bfin_disp_init_encoder_formats(struct bfin_disp_device *disp)
{
	enum v4l2_mbus_pixelcode code;
	struct bfin_disp_format *df;
	unsigned int num_formats = 0;
	int i, j;

	while (!v4l2_subdev_call(disp->sd, video,
				enum_mbus_fmt, num_formats, &code))
		num_formats++;
	if (!num_formats)
		return -ENXIO;

	df = kzalloc(num_formats * sizeof(*df), GFP_KERNEL);
	if (!df)
		return -ENOMEM;

	for (i = 0; i < num_formats; i++) {
		v4l2_subdev_call(disp->sd, video,
				enum_mbus_fmt, i, &code);
		for (j = 0; j < DISP_MAX_FMTS; j++)
			if (code == bfin_disp_formats[j].mbus_code)
				break;
		if (j == DISP_MAX_FMTS) {
			/* we don't allow this encoder working with our bridge */
			kfree(df);
			return -EINVAL;
		}
		df[i] = bfin_disp_formats[j];
	}
	disp->enc_formats = df;
	disp->num_enc_formats = num_formats;
	return 0;
}

static void bfin_disp_free_encoder_formats(struct bfin_disp_device *disp)
{
	disp->num_enc_formats = 0;
	kfree(disp->enc_formats);
	disp->enc_formats = NULL;
}

static int bfin_disp_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct bfin_disp_device *disp = vb2_get_drv_priv(vq);

	if (*nbuffers < DISP_MIN_NUM_BUF)
		*nbuffers = DISP_MIN_NUM_BUF;

	*nplanes = 1;
	sizes[0] = disp->fmt.sizeimage;
	alloc_ctxs[0] = disp->alloc_ctx;

	return 0;
}

static int bfin_disp_buffer_init(struct vb2_buffer *vb)
{
	struct bfin_disp_buffer *buf = to_bfin_disp_vb(vb);

	INIT_LIST_HEAD(&buf->list);
	return 0;
}

static int bfin_disp_buffer_prepare(struct vb2_buffer *vb)
{
	struct bfin_disp_device *disp = vb2_get_drv_priv(vb->vb2_queue);
	struct bfin_disp_buffer *buf = to_bfin_disp_vb(vb);
	unsigned long size;

	size = disp->fmt.sizeimage;
	if (vb2_plane_size(vb, 0) < size) {
		v4l2_err(&disp->v4l2_dev, "buffer too small (%lu < %lu)\n",
				vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	vb2_set_plane_payload(&buf->vb, 0, size);

	return 0;
}

static void bfin_disp_buffer_queue(struct vb2_buffer *vb)
{
	struct bfin_disp_device *disp = vb2_get_drv_priv(vb->vb2_queue);
	struct bfin_disp_buffer *buf = to_bfin_disp_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&disp->lock, flags);
	list_add_tail(&buf->list, &disp->dma_queue);
	spin_unlock_irqrestore(&disp->lock, flags);
}

static void bfin_disp_buffer_cleanup(struct vb2_buffer *vb)
{
	struct bfin_disp_device *disp = vb2_get_drv_priv(vb->vb2_queue);
	struct bfin_disp_buffer *buf = to_bfin_disp_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&disp->lock, flags);
	list_del_init(&buf->list);
	spin_unlock_irqrestore(&disp->lock, flags);
}

static irqreturn_t bfin_disp_isr(int irq, void *dev_id)
{
	struct ppi_if *ppi = dev_id;
	struct bfin_disp_device *disp = ppi->priv;
	struct vb2_buffer *vb = &disp->cur_frm->vb;
	dma_addr_t addr;

	spin_lock(&disp->lock);

	if (!list_empty(&disp->dma_queue)) {
		v4l2_get_timestamp(&vb->v4l2_buf.timestamp);
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		disp->cur_frm = list_entry(disp->dma_queue.next,
				struct bfin_disp_buffer, list);
		list_del_init(&disp->cur_frm->list);
	}

	clear_dma_irqstat(ppi->info->dma_ch);

	addr = vb2_dma_contig_plane_dma_addr(&disp->cur_frm->vb, 0);
	ppi->ops->update_addr(ppi, (unsigned long)addr);
	ppi->ops->start(ppi);

	spin_unlock(&disp->lock);

	return IRQ_HANDLED;
}

static int bfin_disp_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct bfin_disp_device *disp = vb2_get_drv_priv(vq);
	struct ppi_if *ppi = disp->ppi;
	struct ppi_params params;
	dma_addr_t addr;
	unsigned long flags;
	int ret;

	/* enable streamon on the sub device */
	ret = v4l2_subdev_call(disp->sd, video, s_stream, 1);
	if (ret && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&disp->v4l2_dev, "stream on failed in subdev\n");
		return ret;
	}

	/* set ppi params */
	params.width = disp->fmt.width;
	params.height = disp->fmt.height;
	params.bpp = disp->bpp;
	params.dlen = disp->dlen;
	params.ppi_control = disp->cfg->ppi_control;
	params.int_mask = disp->cfg->int_mask;
	if (disp->cfg->outputs[disp->cur_output].capabilities
			& V4L2_OUT_CAP_DV_TIMINGS) {
		struct v4l2_bt_timings *bt = &disp->dv_timings.bt;

		params.hdelay = bt->hsync + bt->hbackporch;
		params.vdelay = bt->vsync + bt->vbackporch;
		params.line = bt->hfrontporch + bt->hsync
				+ bt->hbackporch + bt->width;
		params.frame = bt->vfrontporch + bt->vsync
				+ bt->vbackporch + bt->height;
		if (bt->interlaced)
			params.frame += bt->il_vfrontporch + bt->il_vsync
					+ bt->il_vbackporch;
		params.hsync = bt->hsync;
		params.vsync = bt->vsync;
	} else if (disp->cfg->outputs[disp->cur_output].capabilities
			& V4L2_OUT_CAP_STD) {
		params.hdelay = 0;
		params.vdelay = 0;
		if (disp->std & V4L2_STD_525_60) {
			params.line = 858;
			params.frame = 525;
			if ((ppi->info->type == PPI_TYPE_EPPI3)
				&& (params.ppi_control & BLANKGEN)) {
				params.active_lines = 0x00F300F4;
				params.blank_lines = 0x03110210;
			}
		} else {
			params.line = 864;
			params.frame = 625;
			if ((ppi->info->type == PPI_TYPE_EPPI3)
				&& (params.ppi_control & BLANKGEN)) {
				params.active_lines = 0x01200120;
				params.blank_lines = 0x02170216;
			}
		}
	} else {
		params.hdelay = 0;
		params.vdelay = 0;
		params.line = params.width + disp->cfg->blank_pixels;
		params.frame = params.height;
	}
	ret = ppi->ops->set_params(ppi, &params);
	if (ret < 0) {
		v4l2_err(&disp->v4l2_dev,
				"set ppi params failed\n");
		return ret;
	}

	/* attach ppi DMA irq handler */
	ret = ppi->ops->attach_irq(ppi, bfin_disp_isr);
	if (ret < 0) {
		v4l2_err(&disp->v4l2_dev,
				"attach interrupt handler failed\n");
		return ret;
	}

	spin_lock_irqsave(&disp->lock, flags);
	/* if dma queue is empty, return error */
	if (list_empty(&disp->dma_queue)) {
		spin_unlock_irqrestore(&disp->lock, flags);
		v4l2_err(&disp->v4l2_dev, "dma queue is empty\n");
		return -EINVAL;
	}

	/* get the next frame from the dma queue */
	disp->cur_frm = list_entry(disp->dma_queue.next,
					struct bfin_disp_buffer, list);
	/* remove buffer from the dma queue */
	list_del_init(&disp->cur_frm->list);
	spin_unlock_irqrestore(&disp->lock, flags);

	addr = vb2_dma_contig_plane_dma_addr(&disp->cur_frm->vb, 0);
	/* update DMA address */
	ppi->ops->update_addr(ppi, (unsigned long)addr);
	/* enable ppi */
	ppi->ops->start(ppi);

	return 0;
}

static int bfin_disp_stop_streaming(struct vb2_queue *vq)
{
	struct bfin_disp_device *disp = vb2_get_drv_priv(vq);
	struct ppi_if *ppi = disp->ppi;
	unsigned long flags;
	int ret;

	if (!vb2_is_streaming(vq))
		return 0;

	ppi->ops->stop(ppi);
	ppi->ops->detach_irq(ppi);
	ret = v4l2_subdev_call(disp->sd, video, s_stream, 0);
	if (ret && (ret != -ENOIOCTLCMD))
		v4l2_err(&disp->v4l2_dev,
				"stream off failed in subdev\n");

	spin_lock_irqsave(&disp->lock, flags);
	/* release all active buffers */
	while (!list_empty(&disp->dma_queue)) {
		disp->cur_frm = list_entry(disp->dma_queue.next,
						struct bfin_disp_buffer, list);
		list_del_init(&disp->cur_frm->list);
		vb2_buffer_done(&disp->cur_frm->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&disp->lock, flags);
	return 0;
}

static struct vb2_ops bfin_disp_video_qops = {
	.queue_setup            = bfin_disp_queue_setup,
	.buf_init               = bfin_disp_buffer_init,
	.buf_prepare            = bfin_disp_buffer_prepare,
	.buf_cleanup            = bfin_disp_buffer_cleanup,
	.buf_queue              = bfin_disp_buffer_queue,
	.wait_prepare           = vb2_ops_wait_prepare,
	.wait_finish            = vb2_ops_wait_finish,
	.start_streaming        = bfin_disp_start_streaming,
	.stop_streaming         = bfin_disp_stop_streaming,
};

static int bfin_disp_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct bfin_disp_device *disp = video_drvdata(file);
	struct bfin_display_config *config = disp->cfg;

	if (!(config->outputs[disp->cur_output].capabilities
			& V4L2_OUT_CAP_STD))
		return -ENODATA;

	*std = disp->std;
	return 0;
}

static int bfin_disp_s_std(struct file *file, void *priv, v4l2_std_id std)
{
	struct bfin_disp_device *disp = video_drvdata(file);
	struct bfin_display_config *config = disp->cfg;
	int ret;

	if (vb2_is_busy(&disp->buffer_queue))
		return -EBUSY;

	if (!(config->outputs[disp->cur_output].capabilities
			& V4L2_OUT_CAP_STD))
		return -ENODATA;

	ret = v4l2_subdev_call(disp->sd, video, s_std_output, std);
	if (ret < 0)
		return ret;

	disp->std = std;
	return 0;
}

static int bfin_disp_g_dv_timings(struct file *file, void *priv,
				struct v4l2_dv_timings *timings)
{
	struct bfin_disp_device *disp = video_drvdata(file);
	struct bfin_display_config *config = disp->cfg;
	int ret;

	if (!(config->outputs[disp->cur_output].capabilities
			& V4L2_OUT_CAP_DV_TIMINGS))
		return -ENODATA;

	ret = v4l2_subdev_call(disp->sd, video,
				g_dv_timings, timings);
	if (ret < 0)
		return ret;

	disp->dv_timings = *timings;
	return 0;
}

static int bfin_disp_s_dv_timings(struct file *file, void *priv,
				struct v4l2_dv_timings *timings)
{
	struct bfin_disp_device *disp = video_drvdata(file);
	struct bfin_display_config *config = disp->cfg;
	int ret;

	if (vb2_is_busy(&disp->buffer_queue))
		return -EBUSY;

	if (!(config->outputs[disp->cur_output].capabilities
			& V4L2_OUT_CAP_DV_TIMINGS))
		return -ENODATA;

	ret = v4l2_subdev_call(disp->sd, video, s_dv_timings, timings);
	if (ret < 0)
		return ret;

	disp->dv_timings = *timings;
	return 0;
}

static int bfin_disp_enum_output(struct file *file, void *priv,
				struct v4l2_output *output)
{
	struct bfin_disp_device *disp = video_drvdata(file);
	struct bfin_display_config *config = disp->cfg;

	if (output->index >= config->num_outputs)
		return -EINVAL;

	*output = config->outputs[output->index];
	return 0;
}

static int bfin_disp_g_output(struct file *file, void *priv, unsigned int *index)
{
	struct bfin_disp_device *disp = video_drvdata(file);

	*index = disp->cur_output;
	return 0;
}

static int bfin_disp_s_output(struct file *file, void *priv, unsigned int index)
{
	struct bfin_disp_device *disp = video_drvdata(file);
	struct bfin_display_config *config = disp->cfg;
	struct disp_route *route;
	int ret;

	if (vb2_is_busy(&disp->buffer_queue))
		return -EBUSY;

	if (index >= config->num_outputs)
		return -EINVAL;

	route = &config->routes[index];
	ret = v4l2_subdev_call(disp->sd, video, s_routing,
				0, route->output, route->config);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&disp->v4l2_dev, "Failed to set output\n");
		return ret;
	}
	disp->cur_output = index;
	/* update tvnorms from the subdevice */
	disp->video_dev.tvnorms = config->outputs[index].std;
	/* if this route has specific config, update ppi control */
	if (route->ppi_control)
		config->ppi_control = route->ppi_control;
	return 0;
}

static int bfin_disp_try_format(struct bfin_disp_device *disp,
				struct v4l2_pix_format *pixfmt,
				struct bfin_disp_format *bfin_disp_fmt)
{
	struct bfin_disp_format *df = disp->enc_formats;
	struct bfin_disp_format *fmt = NULL;
	struct v4l2_mbus_framefmt mbus_fmt;
	int ret, i;

	for (i = 0; i < disp->num_enc_formats; i++) {
		fmt = &df[i];
		if (pixfmt->pixelformat == fmt->pixelformat)
			break;
	}
	if (i == disp->num_enc_formats)
		fmt = &df[0];

	v4l2_fill_mbus_format(&mbus_fmt, pixfmt, fmt->mbus_code);
	ret = v4l2_subdev_call(disp->sd, video,
				try_mbus_fmt, &mbus_fmt);
	if (ret < 0)
		return ret;
	v4l2_fill_pix_format(pixfmt, &mbus_fmt);
	if (bfin_disp_fmt) {
		for (i = 0; i < disp->num_enc_formats; i++) {
			fmt = &df[i];
			if (mbus_fmt.code == fmt->mbus_code)
				break;
		}
		*bfin_disp_fmt = *fmt;
	}
	pixfmt->bytesperline = pixfmt->width * fmt->bpp / 8;
	pixfmt->sizeimage = pixfmt->bytesperline * pixfmt->height;
	pixfmt->priv = 0;
	return 0;
}

static int bfin_disp_enum_fmt_vid_out(struct file *file, void  *priv,
					struct v4l2_fmtdesc *fmt)
{
	struct bfin_disp_device *disp = video_drvdata(file);
	struct bfin_disp_format *df = disp->enc_formats;

	if (fmt->index >= disp->num_enc_formats)
		return -EINVAL;

	strlcpy(fmt->description,
		df[fmt->index].desc,
		sizeof(fmt->description));
	fmt->pixelformat = df[fmt->index].pixelformat;
	return 0;
}

static int bfin_disp_try_fmt_vid_out(struct file *file, void *priv,
					struct v4l2_format *fmt)
{
	struct bfin_disp_device *disp = video_drvdata(file);
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;

	return bfin_disp_try_format(disp, pixfmt, NULL);
}

static int bfin_disp_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct bfin_disp_device *disp = video_drvdata(file);

	fmt->fmt.pix = disp->fmt;
	return 0;
}

static int bfin_disp_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct bfin_disp_device *disp = video_drvdata(file);
	struct v4l2_mbus_framefmt mbus_fmt;
	struct bfin_disp_format bfin_disp_fmt;
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
	int ret;

	if (vb2_is_busy(&disp->buffer_queue))
		return -EBUSY;

	/* see if format works */
	ret = bfin_disp_try_format(disp, pixfmt, &bfin_disp_fmt);
	if (ret < 0)
		return ret;

	v4l2_fill_mbus_format(&mbus_fmt, pixfmt, bfin_disp_fmt.mbus_code);
	ret = v4l2_subdev_call(disp->sd, video, s_mbus_fmt, &mbus_fmt);
	if (ret < 0)
		return ret;
	disp->fmt = *pixfmt;
	disp->bpp = bfin_disp_fmt.bpp;
	disp->dlen = bfin_disp_fmt.dlen;
	return 0;
}

static int bfin_disp_querycap(struct file *file, void  *priv,
				struct v4l2_capability *cap)
{
	struct bfin_disp_device *disp = video_drvdata(file);

	cap->device_caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	strlcpy(cap->driver, DISPLAY_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "platform:bfin_display", sizeof(cap->bus_info));
	strlcpy(cap->card, disp->cfg->card_name, sizeof(cap->card));
	return 0;
}

static int bfin_disp_g_parm(struct file *file, void *fh,
				struct v4l2_streamparm *a)
{
	struct bfin_disp_device *disp = video_drvdata(file);

	return v4l2_subdev_call(disp->sd, video, g_parm, a);
}

static int bfin_disp_s_parm(struct file *file, void *fh,
				struct v4l2_streamparm *a)
{
	struct bfin_disp_device *disp = video_drvdata(file);

	return v4l2_subdev_call(disp->sd, video, s_parm, a);
}

static int bfin_disp_log_status(struct file *file, void *priv)
{
	struct bfin_disp_device *disp = video_drvdata(file);
	/* status for sub devices */
	v4l2_device_call_all(&disp->v4l2_dev, 0, core, log_status);
	return 0;
}

static const struct v4l2_ioctl_ops bfin_disp_ioctl_ops = {
	.vidioc_querycap         = bfin_disp_querycap,
	.vidioc_g_fmt_vid_out    = bfin_disp_g_fmt_vid_out,
	.vidioc_enum_fmt_vid_out = bfin_disp_enum_fmt_vid_out,
	.vidioc_s_fmt_vid_out    = bfin_disp_s_fmt_vid_out,
	.vidioc_try_fmt_vid_out  = bfin_disp_try_fmt_vid_out,
	.vidioc_enum_output      = bfin_disp_enum_output,
	.vidioc_g_output         = bfin_disp_g_output,
	.vidioc_s_output         = bfin_disp_s_output,
	.vidioc_s_std            = bfin_disp_s_std,
	.vidioc_g_std            = bfin_disp_g_std,
	.vidioc_s_dv_timings     = bfin_disp_s_dv_timings,
	.vidioc_g_dv_timings     = bfin_disp_g_dv_timings,
	.vidioc_reqbufs          = vb2_ioctl_reqbufs,
	.vidioc_querybuf         = vb2_ioctl_querybuf,
	.vidioc_qbuf             = vb2_ioctl_qbuf,
	.vidioc_dqbuf            = vb2_ioctl_dqbuf,
	.vidioc_streamon         = vb2_ioctl_streamon,
	.vidioc_streamoff        = vb2_ioctl_streamoff,
	.vidioc_g_parm           = bfin_disp_g_parm,
	.vidioc_s_parm           = bfin_disp_s_parm,
	.vidioc_log_status       = bfin_disp_log_status,
};

static struct v4l2_file_operations bfin_disp_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
#ifndef CONFIG_MMU
	.get_unmapped_area = vb2_fop_get_unmapped_area,
#endif
	.poll = vb2_fop_poll,
};

static int bfin_disp_probe(struct platform_device *pdev)
{
	struct bfin_disp_device *disp;
	struct video_device *vfd;
	struct i2c_adapter *i2c_adap;
	struct bfin_display_config *config;
	struct vb2_queue *q;
	struct disp_route *route;
	int ret;

	config = pdev->dev.platform_data;
	if (!config || !config->num_outputs) {
		v4l2_err(pdev->dev.driver, "Invalid board config\n");
		return -ENODEV;
	}

	disp = devm_kzalloc(&pdev->dev, sizeof(*disp), GFP_KERNEL);
	if (!disp)
		return -ENOMEM;

	disp->cfg = config;

	disp->ppi = ppi_create_instance(pdev, config->ppi_info);
	if (!disp->ppi)
		return -ENODEV;
	disp->ppi->priv = disp;

	disp->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(disp->alloc_ctx)) {
		ret = PTR_ERR(disp->alloc_ctx);
		goto err_free_ppi;
	}

	ret = v4l2_device_register(&pdev->dev, &disp->v4l2_dev);
	if (ret) {
		v4l2_err(pdev->dev.driver,
				"Unable to register v4l2 device\n");
		goto err_cleanup_ctx;
	}

	disp->v4l2_dev.ctrl_handler = &disp->ctrl_handler;
	ret = v4l2_ctrl_handler_init(&disp->ctrl_handler, 0);
	if (ret) {
		v4l2_err(&disp->v4l2_dev,
				"Unable to init control handler\n");
		goto err_unreg_v4l2;
	}

	spin_lock_init(&disp->lock);
	mutex_init(&disp->mutex);
	mutex_init(&disp->qlock);
	/* init video dma queues */
	INIT_LIST_HEAD(&disp->dma_queue);

	/* initialize queue */
	q = &disp->buffer_queue;
	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	q->io_modes = VB2_MMAP;
	q->drv_priv = disp;
	q->buf_struct_size = sizeof(struct bfin_disp_buffer);
	q->ops = &bfin_disp_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	/* provide a mutex to vb2 queue */
	q->lock = &disp->qlock;
	q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	ret = vb2_queue_init(q);
	if (ret) {
		v4l2_err(&disp->v4l2_dev,
				"Unable to init videobuf2 queue\n");
		goto err_free_handler;
	}

	/* initialize field of video device */
	vfd = &disp->video_dev;
	vfd->release    = video_device_release_empty;
	vfd->fops       = &bfin_disp_fops;
	vfd->ioctl_ops  = &bfin_disp_ioctl_ops;
	vfd->tvnorms    = V4L2_STD_UNKNOWN;
	vfd->v4l2_dev   = &disp->v4l2_dev;
	vfd->vfl_dir    = VFL_DIR_TX;
	vfd->queue      = q;
	set_bit(V4L2_FL_USE_FH_PRIO, &vfd->flags);
	strncpy(vfd->name, DISPLAY_DRV_NAME, sizeof(vfd->name));
	/* provide a mutex to v4l2 core */
	vfd->lock = &disp->mutex;

	/* load up the subdevice */
	i2c_adap = i2c_get_adapter(config->i2c_adapter_id);
	if (!i2c_adap) {
		v4l2_err(&disp->v4l2_dev,
				"Unable to find i2c adapter\n");
		goto err_free_handler;

	}
	disp->sd = v4l2_i2c_new_subdev_board(&disp->v4l2_dev,
			i2c_adap, &config->board_info, NULL);
	if (!disp->sd) {
		v4l2_err(&disp->v4l2_dev,
				"Unable to register sub device\n");
		goto err_put_adap;
	}

	/*
	 * explicitly set output, otherwise some boards
	 * may not work at the state as we expected
	 */
	route = &config->routes[0];
	ret = v4l2_subdev_call(disp->sd, video, s_routing,
				route->output, route->output, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&disp->v4l2_dev, "Unable to set output\n");
		goto err_unreg_sd;
	}
	disp->cur_output = 0;
	/* update tvnorms from the subdevice */
	vfd->tvnorms = config->outputs[0].std;
	/* if this route has specific config, update ppi control */
	if (route->ppi_control)
		config->ppi_control = route->ppi_control;

	/* now we can probe the default state */
	if (config->outputs[0].capabilities & V4L2_OUT_CAP_STD) {
		v4l2_std_id std;
		ret = v4l2_subdev_call(disp->sd, video, g_std_output, &std);
		if (ret) {
			v4l2_err(&disp->v4l2_dev,
					"Unable to get std\n");
			goto err_unreg_sd;
		}
		disp->std = std;
	}
	if (config->outputs[0].capabilities & V4L2_OUT_CAP_DV_TIMINGS) {
		struct v4l2_dv_timings dv_timings;
		ret = v4l2_subdev_call(disp->sd, video,
				g_dv_timings, &dv_timings);
		if (ret) {
			v4l2_err(&disp->v4l2_dev,
					"Unable to get dv timings\n");
			goto err_unreg_sd;
		}
		disp->dv_timings = dv_timings;
	}
	ret = bfin_disp_init_encoder_formats(disp);
	if (ret) {
		v4l2_err(&disp->v4l2_dev,
				"Unable to create encoder formats table\n");
		goto err_unreg_sd;
	}

	/* register video device */
	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&disp->v4l2_dev,
				"Unable to register video device\n");
		goto err_free_ef;
	}
	video_set_drvdata(vfd, disp);
	v4l2_info(&disp->v4l2_dev, "video device registered as: %s\n",
			video_device_node_name(vfd));

	return 0;
err_free_ef:
	bfin_disp_free_encoder_formats(disp);
err_unreg_sd:
	v4l2_device_unregister_subdev(disp->sd);
	i2c_unregister_device(v4l2_get_subdevdata(disp->sd));
err_put_adap:
	i2c_put_adapter(i2c_adap);
err_free_handler:
	v4l2_ctrl_handler_free(&disp->ctrl_handler);
err_unreg_v4l2:
	v4l2_device_unregister(&disp->v4l2_dev);
err_cleanup_ctx:
	vb2_dma_contig_cleanup_ctx(disp->alloc_ctx);
err_free_ppi:
	ppi_delete_instance(disp->ppi);
	return ret;
}

static int bfin_disp_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct bfin_disp_device *disp = container_of(v4l2_dev,
					struct bfin_disp_device, v4l2_dev);
	struct i2c_client *client = v4l2_get_subdevdata(disp->sd);

	video_unregister_device(&disp->video_dev);
	bfin_disp_free_encoder_formats(disp);
	v4l2_device_unregister_subdev(disp->sd);
	i2c_unregister_device(client);
	i2c_put_adapter(client->adapter);
	v4l2_ctrl_handler_free(&disp->ctrl_handler);
	v4l2_device_unregister(v4l2_dev);
	vb2_dma_contig_cleanup_ctx(disp->alloc_ctx);
	ppi_delete_instance(disp->ppi);
	return 0;
}

static struct platform_driver bfin_disp_driver = {
	.driver = {
		.name  = DISPLAY_DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = bfin_disp_probe,
	.remove = bfin_disp_remove,
};
module_platform_driver(bfin_disp_driver);

MODULE_DESCRIPTION("Analog Devices blackfin video display driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
