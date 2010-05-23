/*
 * Driver for ADI Direct Digital Synthesis ad9850
 *
 * Copyright (c) 2010-2010 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include "../iio.h"
#include "../sysfs.h"

#define DRV_NAME "ad9850"

#define value_mask (u16)0xf000
#define addr_shift 12

/* Register format: 4 bits addr + 12 bits value */
struct ad9850_config {
	u8 control[5];
};

struct ad9850_state {
	struct mutex lock;
	struct iio_dev *idev;
	struct spi_device *sdev;
};

static ssize_t ad9850_set_parameter(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t len)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	int ret;
	struct ad9850_config *config = (struct ad9850_config *)buf;
	struct iio_dev *idev = dev_get_drvdata(dev);
	struct ad9850_state *st = idev->dev_data;

	xfer.len = len;
	xfer.tx_buf = config;
	mutex_lock(&st->lock);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(st->sdev, &msg);
	if (ret)
		goto error_ret;
error_ret:
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(dds, S_IWUSR, NULL, ad9850_set_parameter, NULL);

static struct attribute *ad9850_attributes[] = {
	&iio_dev_attr_dds.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9850_attribute_group = {
	.name = DRV_NAME,
	.attrs = ad9850_attributes,
};

static int __devinit ad9850_probe(struct spi_device *spi)
{
	struct ad9850_state *st;
	int ret = 0;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	spi_set_drvdata(spi, st);

	mutex_init(&st->lock);
	st->sdev = spi;

	st->idev = iio_allocate_device();
	if (st->idev == NULL) {
		ret = -ENOMEM;
		goto error_free_st;
	}
	st->idev->dev.parent = &spi->dev;
	st->idev->num_interrupt_lines = 0;
	st->idev->event_attrs = NULL;

	st->idev->attrs = &ad9850_attribute_group;
	st->idev->dev_data = (void *)(st);
	st->idev->driver_module = THIS_MODULE;
	st->idev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(st->idev);
	if (ret)
		goto error_free_dev;
	spi->max_speed_hz = 2000000;
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 16;
	spi_setup(spi);

	return 0;

error_free_dev:
	iio_free_device(st->idev);
error_free_st:
	kfree(st);
error_ret:
	return ret;
}

static int __devexit ad9850_remove(struct spi_device *spi)
{
	struct ad9850_state *st = spi_get_drvdata(spi);

	iio_device_unregister(st->idev);
	kfree(st);

	return 0;
}

static struct spi_driver ad9850_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = ad9850_probe,
	.remove = __devexit_p(ad9850_remove),
};

static __init int ad9850_spi_init(void)
{
	return spi_register_driver(&ad9850_driver);
}
module_init(ad9850_spi_init);

static __exit void ad9850_spi_exit(void)
{
	spi_unregister_driver(&ad9850_driver);
}
module_exit(ad9850_spi_exit);

MODULE_AUTHOR("Cliff Cai");
MODULE_DESCRIPTION("Analog Devices ad9850 driver");
