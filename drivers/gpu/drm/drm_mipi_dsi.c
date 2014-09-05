/*
 * MIPI DSI Bus
 *
 * Copyright (C) 2012-2013, Samsung Electronics, Co., Ltd.
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <drm/drm_mipi_dsi.h>

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <video/mipi_display.h>

static int mipi_dsi_device_match(struct device *dev, struct device_driver *drv)
{
	return of_driver_match_device(dev, drv);
}

static const struct dev_pm_ops mipi_dsi_device_pm_ops = {
	.runtime_suspend = pm_generic_runtime_suspend,
	.runtime_resume = pm_generic_runtime_resume,
	.suspend = pm_generic_suspend,
	.resume = pm_generic_resume,
	.freeze = pm_generic_freeze,
	.thaw = pm_generic_thaw,
	.poweroff = pm_generic_poweroff,
	.restore = pm_generic_restore,
};

static struct bus_type mipi_dsi_bus_type = {
	.name = "mipi-dsi",
	.match = mipi_dsi_device_match,
	.pm = &mipi_dsi_device_pm_ops,
};

static void mipi_dsi_dev_release(struct device *dev)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);

	of_node_put(dev->of_node);
	kfree(dsi);
}

static const struct device_type mipi_dsi_device_type = {
	.release = mipi_dsi_dev_release,
};

static struct mipi_dsi_device *mipi_dsi_device_alloc(struct mipi_dsi_host *host)
{
	struct mipi_dsi_device *dsi;

	dsi = kzalloc(sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return ERR_PTR(-ENOMEM);

	dsi->host = host;
	dsi->dev.bus = &mipi_dsi_bus_type;
	dsi->dev.parent = host->dev;
	dsi->dev.type = &mipi_dsi_device_type;

	device_initialize(&dsi->dev);

	return dsi;
}

static int mipi_dsi_device_add(struct mipi_dsi_device *dsi)
{
	struct mipi_dsi_host *host = dsi->host;

	dev_set_name(&dsi->dev, "%s.%d", dev_name(host->dev),  dsi->channel);

	return device_add(&dsi->dev);
}

static struct mipi_dsi_device *
of_mipi_dsi_device_add(struct mipi_dsi_host *host, struct device_node *node)
{
	struct mipi_dsi_device *dsi;
	struct device *dev = host->dev;
	int ret;
	u32 reg;

	ret = of_property_read_u32(node, "reg", &reg);
	if (ret) {
		dev_err(dev, "device node %s has no valid reg property: %d\n",
			node->full_name, ret);
		return ERR_PTR(-EINVAL);
	}

	if (reg > 3) {
		dev_err(dev, "device node %s has invalid reg property: %u\n",
			node->full_name, reg);
		return ERR_PTR(-EINVAL);
	}

	dsi = mipi_dsi_device_alloc(host);
	if (IS_ERR(dsi)) {
		dev_err(dev, "failed to allocate DSI device %s: %ld\n",
			node->full_name, PTR_ERR(dsi));
		return dsi;
	}

	dsi->dev.of_node = of_node_get(node);
	dsi->channel = reg;

	ret = mipi_dsi_device_add(dsi);
	if (ret) {
		dev_err(dev, "failed to add DSI device %s: %d\n",
			node->full_name, ret);
		kfree(dsi);
		return ERR_PTR(ret);
	}

	return dsi;
}

int mipi_dsi_host_register(struct mipi_dsi_host *host)
{
	struct device_node *node;

	for_each_available_child_of_node(host->dev->of_node, node)
		of_mipi_dsi_device_add(host, node);

	return 0;
}
EXPORT_SYMBOL(mipi_dsi_host_register);

static int mipi_dsi_remove_device_fn(struct device *dev, void *priv)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);

	device_unregister(&dsi->dev);

	return 0;
}

void mipi_dsi_host_unregister(struct mipi_dsi_host *host)
{
	device_for_each_child(host->dev, NULL, mipi_dsi_remove_device_fn);
}
EXPORT_SYMBOL(mipi_dsi_host_unregister);

/**
 * mipi_dsi_attach - attach a DSI device to its DSI host
 * @dsi: DSI peripheral
 */
int mipi_dsi_attach(struct mipi_dsi_device *dsi)
{
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;

	if (!ops || !ops->attach)
		return -ENOSYS;

	return ops->attach(dsi->host, dsi);
}
EXPORT_SYMBOL(mipi_dsi_attach);

/**
 * mipi_dsi_detach - detach a DSI device from its DSI host
 * @dsi: DSI peripheral
 */
int mipi_dsi_detach(struct mipi_dsi_device *dsi)
{
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;

	if (!ops || !ops->detach)
		return -ENOSYS;

	return ops->detach(dsi->host, dsi);
}
EXPORT_SYMBOL(mipi_dsi_detach);

/*
 * mipi_dsi_set_maximum_return_packet_size() - specify the maximum size of the
 *    the payload in a long packet transmitted from the peripheral back to the
 *    host processor
 * @dsi: DSI peripheral device
 * @value: the maximum size of the payload
 *
 * Return: 0 on success or a negative error code on failure.
 */
int mipi_dsi_set_maximum_return_packet_size(struct mipi_dsi_device *dsi,
					    u16 value)
{
	u8 tx[2] = { value & 0xff, value >> 8 };
	struct mipi_dsi_msg msg;
	ssize_t err;

	memset(&msg, 0, sizeof(msg));
	msg.channel = dsi->channel;
	msg.type = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE;
	msg.tx_len = sizeof(tx);
	msg.tx_buf = tx;

	err = dsi->host->ops->transfer(dsi->host, &msg);
	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL(mipi_dsi_set_maximum_return_packet_size);

/**
 * mipi_dsi_generic_write() - transmit data using a generic write packet
 * @dsi: DSI peripheral device
 * @payload: buffer containing the payload
 * @size: size of payload buffer
 *
 * This function will automatically choose the right data type depending on
 * the payload length.
 *
 * Return: The number of bytes transmitted on success or a negative error code
 * on failure.
 */
ssize_t mipi_dsi_generic_write(struct mipi_dsi_device *dsi, const void *payload,
			       size_t size)
{
	struct mipi_dsi_msg msg;
	ssize_t err;
	u8 *tx;

	memset(&msg, 0, sizeof(msg));
	msg.channel = dsi->channel;
	msg.flags = MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_REQ_ACK;

	if (size > 2) {
		tx = kmalloc(2 + size, GFP_KERNEL);
		if (!tx)
			return -ENOMEM;

		tx[0] = (size >> 0) & 0xff;
		tx[1] = (size >> 8) & 0xff;

		memcpy(tx + 2, payload, size);

		msg.tx_len = 2 + size;
		msg.tx_buf = tx;
	} else {
		msg.tx_buf = payload;
		msg.tx_len = size;
	}

	switch (size) {
	case 0:
		msg.type = MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM;
		break;

	case 1:
		msg.type = MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM;
		break;

	case 2:
		msg.type = MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM;
		break;

	default:
		msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
		break;
	}

	err = dsi->host->ops->transfer(dsi->host, &msg);

	if (size > 2)
		kfree(tx);

	return err;
}
EXPORT_SYMBOL(mipi_dsi_generic_write);

/**
 * mipi_dsi_generic_read() - receive data using a generic read packet
 * @dsi: DSI peripheral device
 * @params: buffer containing the request parameters
 * @num_params: number of request parameters
 * @data: buffer in which to return the received data
 * @size: size of receive buffer
 *
 * This function will automatically choose the right data type depending on
 * the number of parameters passed in.
 *
 * Return: The number of bytes successfully read or a negative error code on
 * failure.
 */
ssize_t mipi_dsi_generic_read(struct mipi_dsi_device *dsi, const void *params,
			      size_t num_params, void *data, size_t size)
{
	struct mipi_dsi_msg msg;

	memset(&msg, 0, sizeof(msg));
	msg.channel = dsi->channel;
	msg.flags = MIPI_DSI_MSG_USE_LPM;
	msg.tx_len = num_params;
	msg.tx_buf = params;
	msg.rx_len = size;
	msg.rx_buf = data;

	switch (num_params) {
	case 0:
		msg.type = MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM;
		break;

	case 1:
		msg.type = MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM;
		break;

	case 2:
		msg.type = MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM;
		break;

	default:
		return -EINVAL;
	}

	return dsi->host->ops->transfer(dsi->host, &msg);
}
EXPORT_SYMBOL(mipi_dsi_generic_read);

/**
 * mipi_dsi_dcs_write_buffer() - transmit a DCS command with payload
 * @dsi: DSI peripheral device
 * @data: buffer containing data to be transmitted
 * @len: size of transmission buffer
 *
 * This function will automatically choose the right data type depending on
 * the command payload length.
 *
 * Return: The number of bytes successfully transmitted or a negative error
 * code on failure.
 */
ssize_t mipi_dsi_dcs_write_buffer(struct mipi_dsi_device *dsi,
				  const void *data, size_t len)
{
	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.tx_buf = data,
		.tx_len = len
	};

	if (!dsi->host->ops || !dsi->host->ops->transfer)
		return -ENOSYS;

	switch (len) {
	case 0:
		return -EINVAL;

	case 1:
		msg.type = MIPI_DSI_DCS_SHORT_WRITE;
		break;

	case 2:
		msg.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		break;

	default:
		msg.type = MIPI_DSI_DCS_LONG_WRITE;
		break;
	}

	return dsi->host->ops->transfer(dsi->host, &msg);
}
EXPORT_SYMBOL(mipi_dsi_dcs_write_buffer);

/**
 * mipi_dsi_dcs_write() - send DCS write command
 * @dsi: DSI peripheral device
 * @cmd: DCS command
 * @data: buffer containing the command payload
 * @len: command payload length
 *
 * This function will automatically choose the right data type depending on
 * the command payload length.
 *
 * Return: The number of bytes successfully transmitted or a negative error
 * code on failure.
 */
ssize_t mipi_dsi_dcs_write(struct mipi_dsi_device *dsi, u8 cmd,
			   const void *data, size_t len)
{
	struct mipi_dsi_msg msg;
	ssize_t err;
	size_t size;
	u8 *tx;

	if (!dsi->host->ops || !dsi->host->ops->transfer)
		return -ENOSYS;

	if (len > 0) {
		unsigned int offset = 0;

		/*
		 * DCS long write packets contain the word count in the header
		 * bytes 1 and 2 and have a payload containing the DCS command
		 * byte folowed by word count minus one bytes.
		 *
		 * DCS short write packets encode the DCS command and up to
		 * one parameter in header bytes 1 and 2.
		 */
		if (len > 1)
			size = 3 + len;
		else
			size = 1 + len;

		tx = kmalloc(size, GFP_KERNEL);
		if (!tx)
			return -ENOMEM;

		/* write word count to header for DCS long write packets */
		if (len > 1) {
			tx[offset++] = ((1 + len) >> 0) & 0xff;
			tx[offset++] = ((1 + len) >> 8) & 0xff;
		}

		/* write the DCS command byte followed by the payload */
		tx[offset++] = cmd;
		memcpy(tx + offset, data, len);
	} else {
		tx = &cmd;
		size = 1;
	}

	memset(&msg, 0, sizeof(msg));
	msg.channel = dsi->channel;
	msg.tx_len = size;
	msg.tx_buf = tx;

	switch (len) {
	case 0:
		msg.type = MIPI_DSI_DCS_SHORT_WRITE;
		break;
	case 1:
		msg.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		break;
	default:
		msg.type = MIPI_DSI_DCS_LONG_WRITE;
		break;
	}

	err = dsi->host->ops->transfer(dsi->host, &msg);

	if (len > 0)
		kfree(tx);

	return err;
}
EXPORT_SYMBOL(mipi_dsi_dcs_write);

/**
 * mipi_dsi_dcs_read() - send DCS read request command
 * @dsi: DSI peripheral device
 * @cmd: DCS command
 * @data: buffer in which to receive data
 * @len: size of receive buffer
 *
 * Return: The number of bytes read or a negative error code on failure.
 */
ssize_t mipi_dsi_dcs_read(struct mipi_dsi_device *dsi, u8 cmd, void *data,
			  size_t len)
{
	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.type = MIPI_DSI_DCS_READ,
		.tx_buf = &cmd,
		.tx_len = 1,
		.rx_buf = data,
		.rx_len = len
	};

	if (!dsi->host->ops || !dsi->host->ops->transfer)
		return -ENOSYS;

	return dsi->host->ops->transfer(dsi->host, &msg);
}
EXPORT_SYMBOL(mipi_dsi_dcs_read);

static int mipi_dsi_drv_probe(struct device *dev)
{
	struct mipi_dsi_driver *drv = to_mipi_dsi_driver(dev->driver);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);

	return drv->probe(dsi);
}

static int mipi_dsi_drv_remove(struct device *dev)
{
	struct mipi_dsi_driver *drv = to_mipi_dsi_driver(dev->driver);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);

	return drv->remove(dsi);
}

static void mipi_dsi_drv_shutdown(struct device *dev)
{
	struct mipi_dsi_driver *drv = to_mipi_dsi_driver(dev->driver);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);

	drv->shutdown(dsi);
}

/**
 * mipi_dsi_driver_register - register a driver for DSI devices
 * @drv: DSI driver structure
 */
int mipi_dsi_driver_register(struct mipi_dsi_driver *drv)
{
	drv->driver.bus = &mipi_dsi_bus_type;
	if (drv->probe)
		drv->driver.probe = mipi_dsi_drv_probe;
	if (drv->remove)
		drv->driver.remove = mipi_dsi_drv_remove;
	if (drv->shutdown)
		drv->driver.shutdown = mipi_dsi_drv_shutdown;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL(mipi_dsi_driver_register);

/**
 * mipi_dsi_driver_unregister - unregister a driver for DSI devices
 * @drv: DSI driver structure
 */
void mipi_dsi_driver_unregister(struct mipi_dsi_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL(mipi_dsi_driver_unregister);

static int __init mipi_dsi_bus_init(void)
{
	return bus_register(&mipi_dsi_bus_type);
}
postcore_initcall(mipi_dsi_bus_init);

MODULE_AUTHOR("Andrzej Hajda <a.hajda@samsung.com>");
MODULE_DESCRIPTION("MIPI DSI Bus");
MODULE_LICENSE("GPL and additional rights");
