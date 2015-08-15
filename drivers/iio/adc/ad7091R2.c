/*
 * AD7091R-2 SPI ADC driver
 *   
 * Copyright 2014 Devin Butterfield.
 *
 * Licensed under the GPL-2 or later.
 */
/* Based on:
 * 
 * AD7466/7/8 AD7476/5/7/8 (A) SPI ADC driver
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/bitmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/spinlock.h>

#include <linux/platform_data/ad7091R2.h>

#define REG_ADDR(ADDR)     (ADDR << 11)
#define REG_WRITE          (1 << 10)

#define RES_MASK(bits)	((1 << (bits)) - 1)

#define AD7091R2_CH_AIN1		0x2 /* convert on channel 1 */
#define AD7091R2_CH_AIN0		0x1 /* convert on channel 0 */

struct ad7091R2_state;

struct ad7091R2_chip_info {
	unsigned int			int_vref_uv;
	struct iio_chan_spec		channel[3]; /* ch0,ch1,timestamp */
	void (*reset)(struct ad7091R2_state *);
};

struct ad7091R2_state {
	spinlock_t lock;
	unsigned int                    num_enabled;
	unsigned int                    scan_idx;
	struct spi_device		*spi;
	const struct ad7091R2_chip_info	*chip_info;
	struct ad7091R2_platform_data   *pdata;
	struct regulator		*reg;
	uint16_t                        mode;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.  Make
	 * the buffer large enough for two 16 bit samples and one 64
	 * bit aligned 64 bit timestamp. (although this driver doesn't
	 * yet support DMA)
	 */
	unsigned char data[ALIGN(4, sizeof(s64)) + sizeof(s64)]
			____cacheline_aligned;
};

enum ad7091R2_supported_device_ids {
	ID_AD7091R2,
};

static int ad7091R2_ring_preenable(struct iio_dev *indio_dev)
{
	struct ad7091R2_state *st = iio_priv(indio_dev);
	int ret;
	u16 cmd;

	ret = iio_sw_buffer_preenable(indio_dev);
	if (ret < 0)
		return ret;

	/* power up the device and enable internal reference */
	cmd = cpu_to_be16(REG_ADDR(0x02) | REG_WRITE | /*st->pdata->mode |*/ 0x1);
	ret = spi_write(st->spi, &cmd, 2);
	if (ret) {
		return ret;
	}

	/* enable the specified channels */
	cmd = cpu_to_be16(REG_ADDR(0x01) | REG_WRITE | *indio_dev->active_scan_mask);
	ret = spi_write(st->spi, &cmd, 2);
	if (ret) {
		return ret;
	}

	st->num_enabled = bitmap_weight(indio_dev->active_scan_mask, 
					indio_dev->masklength);

	/* in order to sync up with device channel sequencer, we set
	 * our starting index to the last count in the scan. The first
	 * trigger event will be for the last in the old sequence (1
	 * conversion latency after sequence update per
	 * datasheet).  */
	//st->scan_idx = st->num_enabled - 1;
	
	return 0;
}

static int ad7091R2_ring_postdisable(struct iio_dev *indio_dev)
{
	struct ad7091R2_state *st = iio_priv(indio_dev);
	int ret;
	u16 cmd;

	/* turn off internal reference and power down */
	cmd = cpu_to_be16(REG_ADDR(0x02) | REG_WRITE | /*st->pdata->mode |*/ 0x0);
	ret = spi_write(st->spi, &cmd, 2);

	return ret;
}

static irqreturn_t ad7091R2_trigger_handler(int irq, void  *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad7091R2_state *st = iio_priv(indio_dev);
	s64 time_ns;
	int ret;
	u16 *buffer = (u16 *)&st->data[0];
	u16 val;
	u16 chan;

	/* If only a single channel is enabled, the scan index stays
	 * at zero, so the channel data always shows up in the first
	 * position in the buffer. If two channels are enabled, then
	 * they will show up in ascending order in the buffer -- as
	 * received from hardware. */

	ret = spi_read(st->spi, &val, 2);
	if (ret < 0) {
		dev_err(&st->spi->dev, "SPI read error\n");
		goto done;
	}

	if (st->num_enabled > 1) {
		chan = (val & 0xff) >> 5; /* grab channel bits */
		buffer[chan] = val;
	} else {
		buffer[0] = val;
	}

	time_ns = iio_get_time_ns();
		
	if (indio_dev->scan_timestamp)
		((s64 *)st->data)[1] = time_ns;
		
	iio_push_to_buffers(indio_dev, st->data);
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops ad7091R2_ring_setup_ops = {
	.preenable = &ad7091R2_ring_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &ad7091R2_ring_postdisable,
};


static void ad7091R2_reset(struct ad7091R2_state *st)
{
	uint16_t config;
	spi_read(st->spi, &config, 2);
	config |= 0x0001;	/* set the soft reset bit (BE ordering) */
	spi_write(st->spi, &config, 2);
}

static int ad7091R2_scan_direct(struct ad7091R2_state *st, unsigned int ch)
{
	/* not supported */
	return -1;
}

static int ad7091R2_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret;
	struct ad7091R2_state *st = iio_priv(indio_dev);
	int scale_uv;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		if (iio_buffer_enabled(indio_dev))
			ret = -EBUSY;
		else
			ret = ad7091R2_scan_direct(st, chan->address);
		mutex_unlock(&indio_dev->mlock);

		if (ret < 0)
			return ret;
		*val = ret >> chan->scan_type.shift;
		*val &= RES_MASK(chan->scan_type.realbits);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		if (!st->chip_info->int_vref_uv) {
			scale_uv = regulator_get_voltage(st->reg);
			if (scale_uv < 0)
				return scale_uv;
		} else {
			scale_uv = st->chip_info->int_vref_uv;
		}
		scale_uv >>= chan->scan_type.realbits;
		*val =  scale_uv / 1000;
		*val2 = (scale_uv % 1000) * 1000;
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

static const struct ad7091R2_chip_info ad7091R2_chip_info_tbl[] = {
	[ID_AD7091R2] = {
		.channel[0] = {
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.channel = 0,
			.info_mask = IIO_CHAN_INFO_RAW_SEPARATE_BIT |
			IIO_CHAN_INFO_SCALE_SHARED_BIT,
			.address = 0,
			.scan_index = 0,
			.scan_type = IIO_ST('u', 12, 16, 0),
		},
		.channel[1] = {
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.channel = 1,
			.info_mask = IIO_CHAN_INFO_RAW_SEPARATE_BIT |
			IIO_CHAN_INFO_SCALE_SHARED_BIT,
			.address = 1,
			.scan_index = 1,
			.scan_type = IIO_ST('u', 12, 16, 0),
		},
		.channel[2] = IIO_CHAN_SOFT_TIMESTAMP(2),
		.int_vref_uv = 2500000,
		.reset = ad7091R2_reset,
	},
};

static const struct iio_info ad7091R2_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &ad7091R2_read_raw,
};

static int ad7091R2_probe(struct spi_device *spi)
{
	struct ad7091R2_platform_data *pdata = spi->dev.platform_data;
	struct ad7091R2_state *st;
	struct iio_dev *indio_dev;
	int ret;
	u16 cmd;

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	st = iio_priv(indio_dev);

        spin_lock_init(&st->lock);

	st->chip_info =
		&ad7091R2_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	st->pdata = pdata;

	st->num_enabled = 0;
	st->scan_idx = 0;

	st->reg = regulator_get(&spi->dev, "vcc");
	if (IS_ERR(st->reg)) {
		ret = PTR_ERR(st->reg);
		dev_err(&spi->dev, "ad7091R2: regulator_get failed\n");
		goto error_free_dev;
	}

	ret = regulator_enable(st->reg);
	if (ret) {
		dev_err(&spi->dev, "ad7091R2: reuglator_enable failed\n");
		goto error_put_reg;
	}

	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	/* Establish that the iio_dev is a child of the spi device */
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->num_channels = 3;
	indio_dev->info = &ad7091R2_info;

	/* device init */

	/* enable internal reference */
	cmd = cpu_to_be16(REG_ADDR(0x02) | REG_WRITE | /*st->pdata->mode |*/ 0x1);
	ret = spi_write(st->spi, &cmd, 2);
	if (ret) {
		goto error_disable_reg;
	}
	
	/* by default, enable both channels */
	cmd = cpu_to_be16(REG_ADDR(0x01) | REG_WRITE | AD7091R2_CH_AIN0 | AD7091R2_CH_AIN1);
	ret = spi_write(st->spi, &cmd, 2);
	if (ret) {
		goto error_disable_reg;
	}

	/* buffer init */	
	ret = iio_triggered_buffer_setup(indio_dev, NULL,
			&ad7091R2_trigger_handler, &ad7091R2_ring_setup_ops);
	if (ret) {
		dev_err(&spi->dev, "ad7091R2: buffer_setup failed\n");
		goto error_disable_reg;
	}

	if (st->chip_info->reset)
		st->chip_info->reset(st);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "ad7091R2: register failed\n");
		goto error_ring_unregister;
	}

	printk(KERN_INFO "ad7091R2: IIO driver for Analog Devices AD7091R-2 ADC\n");
	return 0;

error_ring_unregister:
	iio_triggered_buffer_cleanup(indio_dev);
error_disable_reg:
	regulator_disable(st->reg);
error_put_reg:
	regulator_put(st->reg);
error_free_dev:
	iio_device_free(indio_dev);
error_ret:
	return ret;
}

static int ad7091R2_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad7091R2_state *st = iio_priv(indio_dev);

	/* gpio_free(st->pdata->gpio_convst); */
	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	regulator_disable(st->reg);
	regulator_put(st->reg);
	iio_device_free(indio_dev);

	return 0;
}

static const struct spi_device_id ad7091R2_id[] = {
	{"ad7091R2", ID_AD7091R2},
	{}
};
MODULE_DEVICE_TABLE(spi, ad7091R2_id);

static struct spi_driver ad7091R2_driver = {
	.driver = {
		.name	= "ad7091R2",
		.owner	= THIS_MODULE,
	},
	.probe		= ad7091R2_probe,
	.remove		= ad7091R2_remove,
	.id_table	= ad7091R2_id,
};
module_spi_driver(ad7091R2_driver);

MODULE_AUTHOR("Devin Butterfield <db@lumenosys.com>");
MODULE_DESCRIPTION("Analog Devices AD7091R2 2-channel ADCs");
MODULE_LICENSE("GPL v2");
