/*
 * rt5677-spi.c  --  RT5677 ALSA SoC audio codec driver
 *
 * Copyright 2013 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include "rt5677-spi.h"

static struct spi_device *g_spi;

/**
 * rt5677_spi_write - Write data to SPI.
 * @txbuf: Data Buffer for writing.
 * @len: Data length.
 *
 *
 * Returns true for success.
 */
int rt5677_spi_write(u8 *txbuf, size_t len)
{
	static DEFINE_MUTEX(lock);
	int status;

	mutex_lock(&lock);

	status = spi_write(g_spi, txbuf, len);

	mutex_unlock(&lock);

	if (status)
		dev_err(&g_spi->dev, "rt5677_spi_write error %d\n", status);

	return status;
}

/**
 * rt5677_spi_burst_read - Read data from SPI by rt5677 dsp memory address.
 * @addr: Start address.
 * @rxbuf: Data Buffer for reading.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5677_spi_burst_read(unsigned int addr, u8 *rxbuf, size_t len)
{
	u8 spi_cmd = 0x04;
	int status;
	u8 write_buf[8];
	unsigned int i, end, offset = 0;

	struct spi_message message;
	struct spi_transfer x[3];
	static DEFINE_MUTEX(lock);


	while (offset < len) {
		if (offset + SPI_BUF_LEN <= len)
			end = SPI_BUF_LEN;
		else
			end = len % SPI_BUF_LEN;

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		mutex_lock(&lock);

		spi_message_init(&message);
		memset(x, 0, sizeof(x));

		x[0].len = 5;
		x[0].tx_buf = write_buf;
		spi_message_add_tail(&x[0], &message);

		x[1].len = 4;
		x[1].rx_buf = write_buf;
		spi_message_add_tail(&x[1], &message);

		x[2].len = len;
		x[2].rx_buf = rxbuf + offset;
		spi_message_add_tail(&x[2], &message);

		status = spi_sync(g_spi, &message);

		mutex_unlock(&lock);

		if (status)
			return false;

		offset += SPI_BUF_LEN;
	}

	for (i = 0; i < len; i += 8) {
		write_buf[0] = rxbuf[i + 0];
		write_buf[1] = rxbuf[i + 1];
		write_buf[2] = rxbuf[i + 2];
		write_buf[3] = rxbuf[i + 3];
		write_buf[4] = rxbuf[i + 4];
		write_buf[5] = rxbuf[i + 5];
		write_buf[6] = rxbuf[i + 6];
		write_buf[7] = rxbuf[i + 7];

		rxbuf[i + 0] = write_buf[7];
		rxbuf[i + 1] = write_buf[6];
		rxbuf[i + 2] = write_buf[5];
		rxbuf[i + 3] = write_buf[4];
		rxbuf[i + 4] = write_buf[3];
		rxbuf[i + 5] = write_buf[2];
		rxbuf[i + 6] = write_buf[1];
		rxbuf[i + 7] = write_buf[0];
	}

	return true;
}

/**
 * rt5677_spi_burst_write - Write data to SPI by rt5677 dsp memory address.
 * @addr: Start address.
 * @txbuf: Data Buffer for writng.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5677_spi_burst_write(u32 addr, u8 *txbuf, size_t len)
{
	u8 spi_cmd = 0x05;
	u8 *write_buf;
	unsigned int i, end, offset = 0;

	write_buf = kmalloc(8 * 30 + 6, GFP_KERNEL);

	if (write_buf == NULL)
		return -ENOMEM;

	while (offset < len) {
		if (offset + SPI_BUF_LEN <= len)
			end = SPI_BUF_LEN;
		else
			end = len % SPI_BUF_LEN;

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		for (i = 0; i < end; i += 8) {
			write_buf[i + 12] = txbuf[offset + i + 0];
			write_buf[i + 11] = txbuf[offset + i + 1];
			write_buf[i + 10] = txbuf[offset + i + 2];
			write_buf[i +  9] = txbuf[offset + i + 3];
			write_buf[i +  8] = txbuf[offset + i + 4];
			write_buf[i +  7] = txbuf[offset + i + 5];
			write_buf[i +  6] = txbuf[offset + i + 6];
			write_buf[i +  5] = txbuf[offset + i + 7];
		}

		write_buf[end + 5] = spi_cmd;

		rt5677_spi_write(write_buf, end + 6);

		offset += SPI_BUF_LEN;
	}

	kfree(write_buf);

	return 0;
}

static int rt5677_spi_probe(struct spi_device *spi)
{
	g_spi = spi;
	return 0;
}

static int rt5677_spi_remove(struct spi_device *spi)
{
	return 0;
}

#ifdef CONFIG_PM
static int rt5677_suspend(struct device *dev)
{
	return 0;
}

static int rt5677_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops rt5677_pm_ops = {
	.suspend = rt5677_suspend,
	.resume = rt5677_resume,
};
#endif /*CONFIG_PM */

static struct spi_driver rt5677_spi_driver = {
	.driver = {
			.name = "rt5677_spidev",
			.bus = &spi_bus_type,
			.owner = THIS_MODULE,
#if defined(CONFIG_PM)
			.pm = &rt5677_pm_ops,
#endif
	},
	.probe = rt5677_spi_probe,
	.remove = rt5677_spi_remove,
};

static int __init rt5677_spi_init(void)
{
	return spi_register_driver(&rt5677_spi_driver);
}
module_init(rt5677_spi_init);

static void __exit rt5677_spi_exit(void)
{
	spi_unregister_driver(&rt5677_spi_driver);
}
module_exit(rt5677_spi_exit);

MODULE_DESCRIPTION("ASoC RT5677 SPI driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");

