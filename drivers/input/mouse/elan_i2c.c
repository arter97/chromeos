﻿/*
 * Elan I2C/SMBus Touchpad driver
 *
 * Copyright (c) 2013 ELAN Microelectronics Corp.
 *
 * Author: 林政維 (Duson Lin) <dusonlin@emc.com.tw>
 * Version: 1.5.5
 *
 * Based on cyapa driver:
 * copyright (c) 2011-2012 Cypress Semiconductor, Inc.
 * copyright (c) 2011-2012 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#include <linux/acpi.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#define DRIVER_NAME		"elan_i2c"
#define ELAN_DRIVER_VERSION	"1.5.5"
#define ETP_PRESSURE_OFFSET	25
#define ETP_MAX_PRESSURE	255
#define ETP_FWIDTH_REDUCE	90
#define ETP_FINGER_WIDTH	15
#define ETP_RETRY_COUNT		3

#define ELAN_ADAPTER_FUNC_NONE   0
#define ELAN_ADAPTER_FUNC_I2C    1
#define ELAN_ADAPTER_FUNC_SMBUS  2
#define ELAN_ADAPTER_FUNC_BOTH   3

/* Length of Elan touchpad information */
#define ETP_INF_LENGTH		2
#define ETP_MAX_FINGERS		5
#define ETP_FINGER_DATA_LEN	5
#define ETP_REPORT_ID		0x5D
#define ETP_MAX_REPORT_LEN	34
#define ETP_ENABLE_ABS		0x0001
#define ETP_ENABLE_CALIBRATE	0x0002
#define ETP_DISABLE_CALIBRATE	0x0000
#define ETP_DISABLE_POWER	0x0001

/* Elan smbus command */
#define ETP_SMBUS_IAP_CMD		0x00
#define ETP_SMBUS_ENABLE_TP		0x20
#define ETP_SMBUS_SLEEP_CMD		0x21
#define ETP_SMBUS_IAP_PASSWORD_WRITE	0x29
#define ETP_SMBUS_IAP_PASSWORD_READ	0x80
#define ETP_SMBUS_WRITE_FW_BLOCK	0x2A
#define ETP_SMBUS_IAP_RESET_CMD		0x2B
#define ETP_SMBUS_RANGE_CMD		0xA0
#define ETP_SMBUS_FW_VERSION_CMD	0xA1
#define ETP_SMBUS_XY_TRACENUM_CMD	0xA2
#define ETP_SMBUS_SM_VERSION_CMD	0xA3
#define ETP_SMBUS_UNIQUEID_CMD		0xA3
#define ETP_SMBUS_RESOLUTION_CMD	0xA4
#define ETP_SMBUS_HELLOPACKET_CMD	0xA7
#define ETP_SMBUS_PACKET_QUERY		0xA8
#define ETP_SMBUS_IAP_VERSION_CMD	0xAC
#define ETP_SMBUS_IAP_CTRL_CMD		0xAD
#define ETP_SMBUS_IAP_CHECKSUM_CMD	0xAE
#define ETP_SMBUS_FW_CHECKSUM_CMD	0xAF
#define ETP_SMBUS_MAX_BASELINE_CMD	0xC3
#define ETP_SMBUS_MIN_BASELINE_CMD	0xC4
#define ETP_SMBUS_CALIBRATE_QUERY	0xC5
#define ETP_SMBUS_REPORT_LEN		32
#define ETP_SMBUS_FINGER_DATA_OFFSET	2
#define ETP_SMBUS_HELLOPACKET_LEN	5
#define ETP_SMBUS_IAP_PASSWORD		0x1234
#define ETP_SMBUS_IAP_MODE_ON		(1<<6)

/* Elan i2c command */
#define ETP_I2C_RESET			0x0100
#define ETP_I2C_WAKE_UP			0x0800
#define ETP_I2C_SLEEP			0x0801
#define ETP_I2C_DESC_CMD		0x0001
#define ETP_I2C_REPORT_DESC_CMD		0x0002
#define ETP_I2C_STAND_CMD		0x0005
#define ETP_I2C_UNIQUEID_CMD		0x0101
#define ETP_I2C_FW_VERSION_CMD		0x0102
#define ETP_I2C_SM_VERSION_CMD		0x0103
#define ETP_I2C_XY_TRACENUM_CMD		0x0105
#define ETP_I2C_MAX_X_AXIS_CMD		0x0106
#define ETP_I2C_MAX_Y_AXIS_CMD		0x0107
#define ETP_I2C_RESOLUTION_CMD		0x0108
#define ETP_I2C_IAP_VERSION_CMD		0x0110
#define ETP_I2C_SET_CMD			0x0300
#define ETP_I2C_POWER_CMD		0x0307
#define ETP_I2C_FW_CHECKSUM_CMD		0x030F
#define ETP_I2C_IAP_CTRL_CMD		0x0310
#define ETP_I2C_IAP_CMD			0x0311
#define ETP_I2C_IAP_RESET_CMD		0x0314
#define ETP_I2C_IAP_CHECKSUM_CMD	0x0315
#define ETP_I2C_CALIBRATE_CMD		0x0316
#define ETP_I2C_MAX_BASELINE_CMD	0x0317
#define ETP_I2C_MIN_BASELINE_CMD	0x0318
#define ETP_I2C_REPORT_LEN		34
#define ETP_I2C_FINGER_DATA_OFFSET	4
#define ETP_I2C_REPORT_ID_OFFSET	2
#define ETP_I2C_DESC_LENGTH		30
#define ETP_I2C_REPORT_DESC_LENGTH	158
#define ETP_I2C_IAP_PASSWORD		0x1EA5
#define ETP_I2C_IAP_RESET		0xF0F0
#define ETP_I2C_MAIN_MODE_ON		(1<<9)
#define ETP_I2C_IAP_REG_L		0x01
#define ETP_I2C_IAP_REG_H		0x06

/* IAP F/W updater */
#define ETP_FW_NAME		"elan_i2c.bin"
#define ETP_IAP_VERSION_ADDR	0x0082
#define ETP_IAP_START_ADDR	0x0083
#define ETP_FW_IAP_PAGE_ERR	(1<<5)
#define ETP_FW_IAP_INTERFACE_ERR (1<<4)
#define ETP_FW_PAGE_SIZE	64
#define ETP_FW_PAGE_COUNT	768
#define ETP_FW_SIZE		(ETP_FW_PAGE_SIZE * ETP_FW_PAGE_COUNT)

enum tp_mode {
	UNKNOWN_MODE,
	IAP_MODE,
	MAIN_MODE
};
enum lid_state {
	LID_UNKNOWN,
	LID_OPEN,
	LID_CLOSED
};

/* The main device structure */
struct elan_tp_data {
	struct i2c_client	*client;
	struct input_dev	*input;
	struct regulator	*vcc;

	/* for fw update */
	struct completion	fw_completion;

	struct mutex		sysfs_mutex;

	unsigned int		max_x;
	unsigned int		max_y;
	unsigned int		width_x;
	unsigned int		width_y;
	unsigned int		irq;
	u16			product_id;
	u16			fw_version;
	u16			sm_version;
	u16			iap_version;
	u16			iap_start_addr;
	bool			smbus;
	bool			wait_signal_from_updatefw;
	bool			irq_wake;

	u16			min_baseline;
	u16			max_baseline;
	bool			baseline_ready;

	async_cookie_t		async_init_cookie;
	bool			initialized;

	bool			lid_handler_registered;
	struct input_handler	lid_handler;
	/* lid state */
	enum lid_state		lid_status;
	/* touchpad is active or off based on irq */
	bool			active;
	bool			suspended;
};

static int elan_i2c_read_cmd(struct i2c_client *client, u16 reg, u8 *val);
static int elan_i2c_write_cmd(struct i2c_client *client, u16 reg, u16 cmd);
static int elan_initialize(struct elan_tp_data *data);
static int elan_i2c_reset(struct i2c_client *client);
static int elan_resume(struct device *dev);

/*
 **********************************************************
 * IAP firmware updater related routines
 **********************************************************
 */

static enum tp_mode elan_iap_getmode(struct elan_tp_data *data)
{
	u16 constant;
	int retval;
	u8 val[3];
	struct i2c_client *client = data->client;

	if (data->smbus) {
		retval = i2c_smbus_read_block_data(client,
						   ETP_SMBUS_IAP_CTRL_CMD,
						   val);
		if (retval < 0) {
			dev_err(&client->dev, "read iap ctrol fail.\n");
			return UNKNOWN_MODE;
		}
		constant = be16_to_cpup((__be16 *)val);
		dev_dbg(&client->dev, "iap control reg: 0x%04x.\n", constant);
		if ((constant & ETP_SMBUS_IAP_MODE_ON) == 0x00)
			return MAIN_MODE;
	} else {
		retval = elan_i2c_read_cmd(client, ETP_I2C_IAP_CTRL_CMD, val);
		if (retval < 0) {
			dev_err(&client->dev, "read iap ctrol fail.\n");
			return UNKNOWN_MODE;
		}
		constant = le16_to_cpup((__le16 *)val);
		dev_dbg(&client->dev, "iap control reg: 0x%04x.\n", constant);
		if (constant & ETP_I2C_MAIN_MODE_ON)
			return MAIN_MODE;
	}

	return IAP_MODE;
}

static int elan_iap_checksum(struct elan_tp_data *data)
{
	int retval = 0;
	u16 checksum = -1;
	u8 val[3];
	struct i2c_client *client = data->client;

	if (data->smbus) {
		retval = i2c_smbus_read_block_data(client,
						   ETP_SMBUS_IAP_CHECKSUM_CMD,
						   val);
		if (retval < 0) {
			dev_err(&client->dev, "Read checksum fail, %d\n",
				retval);
			return -1;
		}
		checksum = be16_to_cpup((__be16 *)val);
	} else {
		retval = elan_i2c_read_cmd(client,
					ETP_I2C_IAP_CHECKSUM_CMD, val);
		if (retval < 0) {
			dev_err(&client->dev, "Read checksum fail, %d\n",
				retval);
			return -EIO;
		}
		checksum = le16_to_cpup((__le16 *)val);
	}
	return checksum;
}

static bool elan_iap_reset(struct elan_tp_data *data)
{
	int retval = 0;
	struct i2c_client *client = data->client;

	if (data->smbus)
		retval = i2c_smbus_write_byte(client,
					      ETP_SMBUS_IAP_RESET_CMD);
	else
		retval = elan_i2c_write_cmd(client, ETP_I2C_IAP_RESET_CMD,
					    ETP_I2C_IAP_RESET);
	if (retval < 0) {
		dev_err(&client->dev, "cannot reset IC, %d\n", retval);
		return false;
	}
	return true;
}

static bool elan_iap_setflashkey(struct elan_tp_data *data)
{
	int retval = 0;
	struct i2c_client *client = data->client;
	u8 smbus_cmd[4] = {0x00, 0x0B, 0x00, 0x5A};

	if (data->smbus)
		retval = i2c_smbus_write_block_data(client,
						    ETP_SMBUS_IAP_CMD,
						    4,
						    smbus_cmd);
	else
		retval = elan_i2c_write_cmd(client, ETP_I2C_IAP_CMD,
					    ETP_I2C_IAP_PASSWORD);
	if (retval < 0) {
		dev_err(&client->dev, "cannot set flash key, %d\n", retval);
		return false;
	}

	return true;
}

static int elan_check_fw(struct elan_tp_data *data,
			const struct firmware *fw)
{
	struct device *dev = &data->client->dev;
	u8 val[3];

	/* Firmware must match exact PAGE_NUM * PAGE_SIZE bytes */
	if (fw->size != ETP_FW_SIZE) {
		dev_err(dev, "invalid firmware size = %zu, expected %d.\n",
			fw->size, ETP_FW_SIZE);
		return -EBADF;
	}

	/* Get IAP Start Address*/
	memcpy(val, &fw->data[ETP_IAP_START_ADDR * 2], 2);
	data->iap_start_addr = le16_to_cpup((__le16 *)val);
	return 0;
}


static int elan_smbus_prepare_fw_update(struct elan_tp_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &data->client->dev;
	u16 password;
	u8 val[3];
	u8 cmd[4] = {0x0F, 0x78, 0x00, 0x06};

	/* Get FW in which mode	(IAP_MODE/MAIN_MODE)  */
	enum tp_mode mode = elan_iap_getmode(data);
	if (mode == UNKNOWN_MODE)
		return -EIO;

	if (mode == MAIN_MODE) {

		/* set flash key*/
		if (elan_iap_setflashkey(data) == false) {
			dev_err(dev, "cannot set flash key\n");
			return -EIO;
		}

		/* write iap password */
		if (i2c_smbus_write_byte(client,
					 ETP_SMBUS_IAP_PASSWORD_WRITE) < 0) {
			dev_err(dev, "cannot write iap password\n");
			return -EIO;
		}

		if (i2c_smbus_write_block_data(client,
					       ETP_SMBUS_IAP_CMD, 4, cmd) < 0) {
			dev_err(dev, "cannot write cmd\n");
			return -EIO;
		}

		/* read password to check we enabled successfully. */
		if (i2c_smbus_read_block_data(client,
					      ETP_SMBUS_IAP_PASSWORD_READ,
					      val) < 0) {
			dev_err(dev, "cannot get iap password\n");
			return -EIO;
		}
		password = be16_to_cpup((__be16 *)val);

		if (password != ETP_SMBUS_IAP_PASSWORD) {
			dev_err(dev, "wrong iap password = 0x%X\n", password);
			return -EIO;
		}
		/* wait 30ms, from MAIN_MODE change to IAP_MODE*/
		msleep(30);
	}

	/* set flash key*/
	if (elan_iap_setflashkey(data) == false) {
		dev_err(dev, "cannot set flash key\n");
		return -EIO;
	}

	/* Reset IC */
	if (elan_iap_reset(data) == false) {
		dev_err(dev, "iap reset fail.\n");
		return -EIO;
	}

	return 0;
}

static int elan_i2c_prepare_fw_update(struct elan_tp_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &data->client->dev;
	u8 val[3];
	u16 password;

	/* Get FW in which mode	(IAP_MODE/MAIN_MODE)  */
	enum tp_mode mode = elan_iap_getmode(data);
	if (mode == UNKNOWN_MODE)
		return -EIO;

	if (mode == IAP_MODE) {
		/* Reset IC */
		if (elan_iap_reset(data) == false)
			return -EIO;
		msleep(30);
	}

	/* set flash key*/
	if (elan_iap_setflashkey(data) == false) {
		dev_err(dev, "cannot set flash key\n");
		return -EIO;
	}

	/* Wait for F/W IAP initialization */
	if (mode == MAIN_MODE)
		msleep(100);
	else
		msleep(30);

	/* check is in iap mode or not*/
	if (elan_iap_getmode(data) == MAIN_MODE) {
		dev_err(dev, "status wrong.\n");
		return -EIO;
	}

	/* set flash key again */
	if (elan_iap_setflashkey(data) == false) {
		dev_err(dev, "cannot set flash key\n");
		return -EIO;
	}

	/* Wait for F/W IAP initialization */
	if (mode == MAIN_MODE)
		msleep(100);
	else
		msleep(30);

	/* read back to check we actually enabled successfully. */
	if (elan_i2c_read_cmd(client, ETP_I2C_IAP_CMD, val) < 0) {
		dev_err(dev, "cannot get iap register\n");
		return -EIO;
	}
	password = le16_to_cpup((__le16 *)val);

	if (password != ETP_I2C_IAP_PASSWORD) {
		dev_err(dev, "wrong iap password = 0x%X\n", password);
		return -EIO;
	}
	return 0;
}

static bool elan_iap_page_write_ok(struct elan_tp_data *data)
{
	u16 constant;
	int retval = 0;
	u8 val[3];
	struct i2c_client *client = data->client;


	if (data->smbus) {
		retval = i2c_smbus_read_block_data(client,
						   ETP_SMBUS_IAP_CTRL_CMD, val);
		if (retval < 0)
			return false;
		constant = be16_to_cpup((__be16 *)val);
	} else {
		retval = elan_i2c_read_cmd(client,
					   ETP_I2C_IAP_CTRL_CMD, val);
		if (retval < 0)
			return false;
		constant = le16_to_cpup((__le16 *)val);
	}

	if (constant & ETP_FW_IAP_PAGE_ERR)
		return false;

	if (constant & ETP_FW_IAP_INTERFACE_ERR)
		return false;
	return true;
}

static int elan_smbus_write_fw_block(struct elan_tp_data *data,
				     const u8 *page, u16 checksum, int idx)
{
	struct device *dev = &data->client->dev;
	int half_page_size = ETP_FW_PAGE_SIZE / 2;
	int repeat = ETP_RETRY_COUNT;

	do {
		/* due to smbus can write 32 bytes one time,
		so, we must write data 2 times.
		*/
		i2c_smbus_write_block_data(data->client,
					   ETP_SMBUS_WRITE_FW_BLOCK,
					   half_page_size,
					   page);
		i2c_smbus_write_block_data(data->client,
					   ETP_SMBUS_WRITE_FW_BLOCK,
					   half_page_size,
					   (page + half_page_size));
		/* Wait for F/W to update one page ROM data. */
		usleep_range(8000, 10000);
		if (elan_iap_page_write_ok(data))
			break;
		dev_dbg(dev, "IAP retry this page! [%d]\n", idx);
		repeat--;
	} while (repeat > 0);

	if (repeat > 0)
		return 0;
	return -EIO;

}

static int elan_i2c_write_fw_block(struct elan_tp_data *data,
				   const u8 *page, u16 checksum, int idx)
{
	struct device *dev = &data->client->dev;
	int ret;
	int repeat = ETP_RETRY_COUNT;
	u8 page_store[ETP_FW_PAGE_SIZE + 4];

	page_store[0] = ETP_I2C_IAP_REG_L;
	page_store[1] = ETP_I2C_IAP_REG_H;
	memcpy(&page_store[2], page, ETP_FW_PAGE_SIZE);

	/* recode checksum at last two bytes */
	page_store[ETP_FW_PAGE_SIZE+2] = (u8)(checksum & 0xFF);
	page_store[ETP_FW_PAGE_SIZE+3] = (u8)((checksum >> 8)&0xFF);

	do {
		ret = i2c_master_send(data->client, page_store,
				      ETP_FW_PAGE_SIZE + 4);

		/* Wait for F/W to update one page ROM data. */
		msleep(20);

		if (ret == (ETP_FW_PAGE_SIZE + 4)) {
			if (elan_iap_page_write_ok(data))
				break;
		}
		dev_dbg(dev, "IAP retry this page! [%d]\n", idx);
		repeat--;
	} while (repeat > 0);

	if (repeat > 0)
		return 0;
	return -1;
}

static int elan_write_fw_block(struct elan_tp_data *data,
			       const u8 *page, u16 checksum, int idx)
{
	int ret;
	if (data->smbus)
		ret = elan_smbus_write_fw_block(data, page, checksum, idx);
	else
		ret = elan_i2c_write_fw_block(data, page, checksum, idx);
	return ret;
}

static int elan_prepare_fw_update(struct elan_tp_data *data)
{
	int ret = 0;
	if (data->smbus)
		ret = elan_smbus_prepare_fw_update(data);
	else
		ret = elan_i2c_prepare_fw_update(data);
	return ret;
}

static int elan_wait_for_chg(struct elan_tp_data *data, unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	struct completion *comp = &data->fw_completion;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		dev_err(dev, "Wait for completion interrupted.\n");
		/*
		 * TODO: handle -EINTR better by terminating fw update process
		 * before get complete INT signal from firmware
		 */
		return -EINTR;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int elan_firmware(struct elan_tp_data *data, const char *fw_name)
{
	struct device *dev = &data->client->dev;
	const struct firmware *fw;
	int i, j, ret;
	u16 boot_page_count;
	u16 sw_checksum, fw_checksum;
	u8 buffer[ETP_INF_LENGTH];

	dev_dbg(dev, "Start firmware update....\n");

	ret = request_firmware(&fw, fw_name, dev);
	if (ret) {
		dev_err(dev, "cannot load firmware from %s, %d\n",
			fw_name, ret);
		goto done;
	}
	/* check fw data match current iap version */
	ret = elan_check_fw(data, fw);
	if (ret) {
		dev_err(dev, "Invalid Elan firmware from %s, %d\n",
			fw_name, ret);
		goto done;
	}
	/* setup IAP status */
	ret = elan_prepare_fw_update(data);
	if (ret)
		goto done;
	sw_checksum = 0;
	fw_checksum = 0;
	boot_page_count = (data->iap_start_addr * 2) / ETP_FW_PAGE_SIZE;
	for (i = boot_page_count; i < ETP_FW_PAGE_COUNT; i++) {
		u16 checksum = 0;
		const u8 *page = &fw->data[i * ETP_FW_PAGE_SIZE];

		for (j = 0; j < ETP_FW_PAGE_SIZE; j += 2)
			checksum += ((page[j + 1] << 8) | page[j]);

		ret = elan_write_fw_block(data, page, checksum, i);
		if (ret) {
			dev_err(dev, "write page %d fail\n", i);
			goto done;
		}
		sw_checksum += checksum;
	}

	/* Wait WDT reset and power on reset */
	if (data->smbus) {
		msleep(600);
	} else {
		/* only i2c interface need waiting for fw reset signal */
		data->wait_signal_from_updatefw = true;
		reinit_completion(&data->fw_completion);

		msleep(600);

		ret = elan_i2c_reset(data->client);
		if (ret < 0) {
			dev_err(dev, "device reset failed %d.\n", ret);
			goto done;
		}

		ret = elan_wait_for_chg(data, 300);
		if (ret) {
			dev_err(dev, "Failed waiting for reset %d.\n", ret);
			goto done;
		}

		ret = i2c_master_recv(data->client, buffer, ETP_INF_LENGTH);
		if (ret != ETP_INF_LENGTH) {
			dev_err(dev, "Failed INT signal data ret=%d\n", ret);
			goto done;
		}
		data->wait_signal_from_updatefw = false;
	}

	/* check checksum */
	fw_checksum = elan_iap_checksum(data);
	if (sw_checksum != fw_checksum) {
		dev_err(dev, "checksum diff sw=[%04X], fw=[%04X]\n",
			sw_checksum, fw_checksum);
		ret = -EIO;
		goto done;
	}
	ret = 0;
done:
	if (ret != 0)
		elan_iap_reset(data);
	else {
		/* Reinitialize TP after fw is updated */
		disable_irq(data->irq);
		data->active = false;
		elan_initialize(data);
		enable_irq(data->irq);
		data->active = true;
	}
	release_firmware(fw);
	return ret;
}

/*
 *******************************************************************
 * Elan smbus interface
 *******************************************************************
 */
static int elan_smbus_initialize(struct i2c_client *client)
{
	u8 check[ETP_SMBUS_HELLOPACKET_LEN] = {0x55, 0x55, 0x55, 0x55, 0x55};
	u8 values[ETP_SMBUS_HELLOPACKET_LEN] = {0, 0, 0, 0, 0};
	int ret;

	/* Get hello packet */
	ret = i2c_smbus_read_block_data(client,
					ETP_SMBUS_HELLOPACKET_CMD, values);
	if (ret != ETP_SMBUS_HELLOPACKET_LEN) {
		dev_err(&client->dev, "hello packet length fail\n");
		return -1;
	}

	/* compare hello packet */
	if (memcmp(values, check, ETP_SMBUS_HELLOPACKET_LEN)) {
		dev_err(&client->dev, "hello packet fail [%x %x %x %x %x]\n",
			values[0], values[1], values[2], values[3], values[4]);
		return -1;
	}

	/* enable tp */
	ret = i2c_smbus_write_byte(client, ETP_SMBUS_ENABLE_TP);
	return ret;
}

static int elan_smbus_enable_absolute_mode(struct i2c_client *client)
{
	u8 cmd[4] = {0x00, 0x07, 0x00, ETP_ENABLE_ABS};

	return i2c_smbus_write_block_data(client, ETP_SMBUS_IAP_CMD, 4, cmd);
}

static int elan_smbus_enable_calibrate(struct i2c_client *client)
{
	u8 cmd[4] = {0x00, 0x07, 0x00, ETP_ENABLE_ABS|ETP_ENABLE_CALIBRATE};

	return i2c_smbus_write_block_data(client,
					  ETP_SMBUS_IAP_CMD, 4, cmd);
}

static int elan_smbus_disable_calibrate(struct i2c_client *client)
{
	u8 cmd[4] = {0x00, 0x07, 0x00, ETP_ENABLE_ABS|ETP_DISABLE_CALIBRATE};

	return i2c_smbus_write_block_data(client,
					  ETP_SMBUS_IAP_CMD, 4, cmd);
}

/*
 ******************************************************************
 * Elan i2c interface
 ******************************************************************
 */
static int elan_i2c_read_block(struct i2c_client *client,
			       u16 reg, u8 *val, u16 len)
{
	struct i2c_msg msgs[2];
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags & I2C_M_TEN;
	msgs[1].flags |= I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = val;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret != 2 ? -EIO : 0;
}

static int elan_i2c_read_cmd(struct i2c_client *client, u16 reg, u8 *val)
{
	int retval;

	retval = elan_i2c_read_block(client, reg, val, ETP_INF_LENGTH);
	if (retval < 0) {
		dev_err(&client->dev, "reading cmd (0x%04x) fail.\n", reg);
		return retval;
	}
	return 0;
}

static int elan_i2c_write_cmd(struct i2c_client *client, u16 reg, u16 cmd)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = cmd & 0xff;
	buf[3] = (cmd >> 8) & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 4;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret != 1 ? -EIO : 0;
}

static int elan_i2c_reset(struct i2c_client *client)
{
	return elan_i2c_write_cmd(client, ETP_I2C_STAND_CMD,
				  ETP_I2C_RESET);
}

static int elan_i2c_wake_up(struct i2c_client *client)
{
	return elan_i2c_write_cmd(client, ETP_I2C_STAND_CMD,
				  ETP_I2C_WAKE_UP);
}

static int elan_i2c_sleep(struct i2c_client *client)
{
	return elan_i2c_write_cmd(client, ETP_I2C_STAND_CMD,
				  ETP_I2C_SLEEP);
}

static int elan_i2c_enable_absolute_mode(struct i2c_client *client)
{
	return elan_i2c_write_cmd(client, ETP_I2C_SET_CMD,
				  ETP_ENABLE_ABS);
}

static int elan_i2c_get_desc(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_block(client, ETP_I2C_DESC_CMD, val,
				   ETP_I2C_DESC_LENGTH);
}

static int elan_i2c_get_report_desc(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_block(client, ETP_I2C_REPORT_DESC_CMD,
				   val, ETP_I2C_REPORT_DESC_LENGTH);
}

static int elan_i2c_enable_calibrate(struct i2c_client *client)
{
	return elan_i2c_write_cmd(client, ETP_I2C_SET_CMD,
				  ETP_ENABLE_ABS|ETP_ENABLE_CALIBRATE);
}

static int elan_i2c_disable_calibrate(struct i2c_client *client)
{
	return elan_i2c_write_cmd(client, ETP_I2C_SET_CMD,
				  ETP_ENABLE_ABS|ETP_DISABLE_CALIBRATE);
}

static int elan_i2c_enable_power(struct i2c_client *client)
{
	u8 val[2];
	int reg;
	elan_i2c_read_cmd(client, ETP_I2C_POWER_CMD, val);
	reg = le16_to_cpup((__le16 *)val);
	reg &= (~ETP_DISABLE_POWER);
	return elan_i2c_write_cmd(client, ETP_I2C_POWER_CMD, reg);
}

static int elan_i2c_disable_power(struct i2c_client *client)
{
	u8 val[2];
	int reg;
	elan_i2c_read_cmd(client, ETP_I2C_POWER_CMD, val);
	reg = le16_to_cpup((__le16 *)val);
	reg |= ETP_DISABLE_POWER;
	return elan_i2c_write_cmd(client, ETP_I2C_POWER_CMD, reg);
}

static int elan_i2c_initialize(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	int rc;
	u8 val[256];

	rc = elan_i2c_reset(client);
	if (rc < 0) {
		dev_err(dev, "device reset failed.\n");
		return -1;
	}

	/* wait for get reset return flag */
	msleep(100);
	/* get reset return flag 0000 */
	rc = i2c_master_recv(client, val, ETP_INF_LENGTH);
	if (rc < 0) {
		dev_err(dev, "get device reset return value failed.\n");
		return -1;
	}

	rc = elan_i2c_get_desc(client, val);
	if (rc < 0) {
		dev_err(dev, "cannot get device descriptor.\n");
		return -1;
	}

	rc = elan_i2c_get_report_desc(client, val);
	if (rc < 0) {
		dev_err(dev, "fetching report descriptor failed.\n");
		return -1;
	}
	return 0;
}

/*
 ******************************************************************
 * General functions
 ******************************************************************
 */
/*
 * (value from firmware) * 10 + 790 = dpi
 * we also have to convert dpi to dots/mm (*10/254 to avoid floating point)
 */
static unsigned int elan_convert_res(char val)
{
	int res;
	if (val & 0x80) {
		val = ~val + 1;
		res = (790 - val * 10) * 10 / 254;
	} else
		res = (val * 10 + 790) * 10 / 254;
	return res;
}

static int elan_get_iap_version(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus) {
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_IAP_VERSION_CMD, val);
		ret = val[2];
	} else {
		elan_i2c_read_cmd(data->client,
				  ETP_I2C_IAP_VERSION_CMD, val);
		ret = val[0];
	}
	return ret;
}

static int elan_get_x_max(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus) {
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_RANGE_CMD, val);
		ret = (0x0f & val[0]) << 8 | val[1];
	} else {
		elan_i2c_read_cmd(data->client,
				  ETP_I2C_MAX_X_AXIS_CMD, val);
		ret = (0x0f & val[1]) << 8 | val[0];
	}
	return ret;
}

static int elan_get_y_max(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus) {
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_RANGE_CMD, val);
		ret = (0xf0 & val[0]) << 4 | val[2];
	} else {
		elan_i2c_read_cmd(data->client,
				  ETP_I2C_MAX_Y_AXIS_CMD, val);
		ret = (0x0f & val[1]) << 8 | val[0];
	}
	return ret;
}

static int elan_get_x_tracenum(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus) {
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_XY_TRACENUM_CMD, val);
		ret = (val[1] - 1);
	} else {
		elan_i2c_read_cmd(data->client,
				  ETP_I2C_XY_TRACENUM_CMD, val);
		ret = (val[0] - 1);
	}
	return ret;
}

static int elan_get_y_tracenum(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus) {
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_XY_TRACENUM_CMD, val);
		ret = (val[2] - 1);
	} else {
		ret = elan_i2c_read_cmd(data->client,
					ETP_I2C_XY_TRACENUM_CMD, val);
		ret = (val[1] - 1);
	}
	return ret;
}

static int elan_get_fw_version(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus) {
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_FW_VERSION_CMD, val);
		ret = val[2];
	} else {
		elan_i2c_read_cmd(data->client,
				  ETP_I2C_FW_VERSION_CMD, val);
		ret = val[0];
	}
	return ret;
}

static int elan_get_sm_version(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus)
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_SM_VERSION_CMD, val);
	else
		elan_i2c_read_cmd(data->client,
				  ETP_I2C_SM_VERSION_CMD, val);
	ret = val[0];
	return ret;
}

static int elan_get_product_id(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus) {
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_UNIQUEID_CMD, val);
		ret = val[1];
	} else {
		elan_i2c_read_cmd(data->client,
				  ETP_I2C_UNIQUEID_CMD, val);
		ret = val[0];
	}
	return ret;
}

static int elan_get_x_resolution(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus) {
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_RESOLUTION_CMD, val);
		ret = elan_convert_res(val[1] & 0x0F);
	} else {
		elan_i2c_read_cmd(data->client,
				  ETP_I2C_RESOLUTION_CMD, val);
		ret = elan_convert_res(val[0]);
	}
	return ret;
}

static int elan_get_y_resolution(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus) {
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_RESOLUTION_CMD, val);
		ret = elan_convert_res((val[1] & 0xF0) >> 4);
	} else {
		elan_i2c_read_cmd(data->client,
				  ETP_I2C_RESOLUTION_CMD, val);
		ret = elan_convert_res(val[1]);
	}
	return ret;
}

static int elan_get_fw_checksum(struct elan_tp_data *data)
{
	int ret;
	u8 val[3];
	if (data->smbus) {
		i2c_smbus_read_block_data(data->client,
					  ETP_SMBUS_FW_CHECKSUM_CMD, val);
		ret = be16_to_cpup((__be16 *)val);
	} else {
		elan_i2c_read_cmd(data->client,
				  ETP_I2C_FW_CHECKSUM_CMD, val);
		ret = le16_to_cpup((__le16 *)val);
	}
	return ret;
}

static int elan_get_max_baseline(struct elan_tp_data *data, u16 *value)
{
	int error;
	u8 buf[3];

	if (data->smbus) {
		error = i2c_smbus_read_block_data(data->client,
						  ETP_SMBUS_MAX_BASELINE_CMD,
						  buf);
		if (error)
			return error;

		*value = be16_to_cpup((__be16 *)buf);
	} else {
		error = elan_i2c_read_cmd(data->client,
					  ETP_I2C_MAX_BASELINE_CMD, buf);
		if (error)
			return error;

		*value = le16_to_cpup((__le16 *)buf);
	}

	return 0;
}

static int elan_get_min_baseline(struct elan_tp_data *data, u16 *value)
{
	int error;
	u8 buf[3];

	if (data->smbus) {
		error = i2c_smbus_read_block_data(data->client,
						  ETP_SMBUS_MIN_BASELINE_CMD,
						  buf);
		if (error)
			return error;

		*value = be16_to_cpup((__be16 *)buf);
	} else {
		error = elan_i2c_read_cmd(data->client,
					  ETP_I2C_MIN_BASELINE_CMD, buf);
		if (error)
			return error;

		*value = le16_to_cpup((__le16 *)buf);
	}

	return 0;
}

static int elan_enable_calibrate(struct elan_tp_data *data)
{
	int ret;

	if (data->smbus)
		ret = elan_smbus_enable_calibrate(data->client);
	else
		ret = elan_i2c_enable_calibrate(data->client);

	return ret;
}

static int elan_disable_calibrate(struct elan_tp_data *data)
{
	int ret;

	if (data->smbus)
		ret = elan_smbus_disable_calibrate(data->client);
	else
		ret = elan_i2c_disable_calibrate(data->client);

	return ret;
}

static int elan_enable_power(struct elan_tp_data *data)
{
	int repeat = ETP_RETRY_COUNT;
	int error;

	error = regulator_enable(data->vcc);
	if (error) {
		dev_err(&data->client->dev,
			"Failed to enable regulator: %d\n", error);
		return error;
	}

	do {
		if (data->smbus)
			/* no effect */
			error = 0;
		else
			error = elan_i2c_enable_power(data->client);
		if (error >= 0)
			return 0;

		repeat--;
		msleep(30);
	} while (repeat > 0);

	return error;
}

static int elan_disable_power(struct elan_tp_data *data)
{
	int repeat = ETP_RETRY_COUNT;
	int error;

	do {
		if (data->smbus)
			/* no effect */
			error = 0;
		else
			error = elan_i2c_disable_power(data->client);
		if (error >= 0) {
			error = regulator_disable(data->vcc);
			if (error) {
				dev_err(&data->client->dev,
					"Failed to disable regulator: %d\n",
					error);
				if (!data->smbus)
					elan_i2c_enable_power(data->client);
				break;
			}

			return 0;
		}

		repeat--;
		msleep(30);
	} while (repeat > 0);

	return error;
}

static int elan_sleep(struct elan_tp_data *data)
{
	int ret;
	int repeat = ETP_RETRY_COUNT;

	do {
		if (data->smbus)
			ret = i2c_smbus_write_byte(data->client,
						   ETP_SMBUS_SLEEP_CMD);
		else
			ret = elan_i2c_sleep(data->client);
		if (ret >= 0)
			break;
		repeat--;
		msleep(30);
	} while (repeat > 0);

	return ret;
}

static int elan_initialize(struct elan_tp_data *data)
{
	int ret;
	int repeat = ETP_RETRY_COUNT;

	do {
		if (data->smbus) {
			ret = elan_smbus_initialize(data->client);
			if (ret < 0) {
				dev_err(&data->client->dev,
					"device initialize failed.\n");
				goto err_initialize;
			}

			ret = elan_smbus_enable_absolute_mode(data->client);
			if (ret < 0)
				dev_err(&data->client->dev,
					"cannot switch to absolute mode.\n");
		} else {
			ret = elan_i2c_initialize(data->client);
			if (ret < 0) {
				dev_err(&data->client->dev,
					"device initialize failed.\n");
				goto err_initialize;
			}

			ret = elan_i2c_enable_absolute_mode(data->client);
			if (ret < 0) {
				dev_err(&data->client->dev,
					"cannot switch to absolute mode.\n");
				goto err_initialize;
			}

			ret = elan_i2c_wake_up(data->client);
			if (ret < 0)
				dev_err(&data->client->dev,
					"device wake up failed.\n");
		}
		if (ret >= 0)
			break;
err_initialize:
		repeat--;
		msleep(30);
	} while (repeat > 0);
	return ret;
}

/*
 *******************************************************************
 * below routines export interfaces to sysfs file system.
 * so user can get firmware/driver/hardware information using cat command.
 * e.g.: use below command to get firmware version
 *      cat /sys/bus/i2c/drivers/elan_i2c/x-0015/firmware_version
 *******************************************************************
 */
static ssize_t elan_sysfs_read_fw_checksum(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	unsigned int checksum = 0;
	struct elan_tp_data *data = dev_get_drvdata(dev);
	checksum = elan_get_fw_checksum(data);
	return sprintf(buf, "0x%04x\n", checksum);
}

static ssize_t elan_sysfs_read_product_id(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct elan_tp_data *data = dev_get_drvdata(dev);
	data->product_id = elan_get_product_id(data);
	return sprintf(buf, "%d.0\n", data->product_id);
}

static ssize_t elan_sysfs_read_fw_ver(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct elan_tp_data *data = dev_get_drvdata(dev);
	data->fw_version = elan_get_fw_version(data);
	return sprintf(buf, "%d.0\n", data->fw_version);
}

static ssize_t elan_sysfs_read_sm_ver(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct elan_tp_data *data = dev_get_drvdata(dev);
	data->sm_version = elan_get_sm_version(data);
	return sprintf(buf, "%d.0\n", data->sm_version);
}

static ssize_t elan_sysfs_read_iap_ver(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct elan_tp_data *data = dev_get_drvdata(dev);
	data->iap_version = elan_get_iap_version(data);
	return sprintf(buf, "%d.0\n", data->iap_version);
}


static ssize_t elan_sysfs_update_fw(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct elan_tp_data *data = dev_get_drvdata(dev);
	int ret, i;
	const char *fw_name;
	char tmp[count];

	/* Do not allow paths that step out of /lib/firmware  */
	if (strstr(buf, "../") != NULL)
		return -EINVAL;

	if (!strncmp(buf, "1", count) || !strncmp(buf, "1\n", count)) {
		fw_name = ETP_FW_NAME;
	} else {
		/* check input file name buffer include '\n' or not */
		for (i = 0; i < count; i++) {
			if (buf[i] != '\n')
				tmp[i] = buf[i];
			else
				tmp[i] = '\0';
		}
		fw_name = tmp;
	}

	ret = elan_firmware(data, fw_name);
	if (ret)
		dev_err(dev, "firmware update failed.\n");
	else
		dev_dbg(dev, "firmware update succeeded.\n");
	return ret ? ret : count;
}

static ssize_t elan_sysfs_calibrate(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct elan_tp_data *data = dev_get_drvdata(dev);
	/* start calibarate cmd */
	u8 smbus_cmd[4] = {0x00, 0x08, 0x00, 0x01};
	u8 val[3];
	int tries = 20;
	int ret = 0;
	val[0] = 0;

	disable_irq(data->irq);
	data->active = false;
	elan_enable_calibrate(data);
	if (data->smbus)
		i2c_smbus_write_block_data(data->client,
					   ETP_SMBUS_IAP_CMD, 4, smbus_cmd);
	else
		elan_i2c_write_cmd(data->client,
				   ETP_I2C_CALIBRATE_CMD, 1);

	do {
		/* wait 250ms and check finish or not */
		msleep(250);

		if (data->smbus)
			i2c_smbus_read_block_data(data->client,
						  ETP_SMBUS_CALIBRATE_QUERY,
						  val);
		else
			elan_i2c_read_block(data->client,
					    ETP_I2C_CALIBRATE_CMD, val, 1);

		/* calibrate finish */
		if (val[0] == 0)
			break;
	} while (--tries);

	elan_disable_calibrate(data);
	enable_irq(data->irq);
	data->active = true;

	if (tries == 0) {
		dev_err(dev, "Failed to calibrate. Timeout.\n");
		ret = -ETIMEDOUT;
	}
	return sprintf(buf, "calibration finish\n");
}

static ssize_t elan_sysfs_read_mode(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct elan_tp_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (int)elan_iap_getmode(data));
}

static DEVICE_ATTR(product_id, S_IRUGO, elan_sysfs_read_product_id, NULL);
static DEVICE_ATTR(firmware_version, S_IRUGO, elan_sysfs_read_fw_ver, NULL);
static DEVICE_ATTR(sample_version, S_IRUGO, elan_sysfs_read_sm_ver, NULL);
static DEVICE_ATTR(iap_version, S_IRUGO, elan_sysfs_read_iap_ver, NULL);
static DEVICE_ATTR(fw_checksum, S_IRUGO, elan_sysfs_read_fw_checksum, NULL);
static DEVICE_ATTR(calibrate, S_IRUGO, elan_sysfs_calibrate, NULL);
static DEVICE_ATTR(mode, S_IRUGO, elan_sysfs_read_mode, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, elan_sysfs_update_fw);

static struct attribute *elan_sysfs_entries[] = {
	&dev_attr_product_id.attr,
	&dev_attr_firmware_version.attr,
	&dev_attr_sample_version.attr,
	&dev_attr_iap_version.attr,
	&dev_attr_fw_checksum.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_mode.attr,
	&dev_attr_update_fw.attr,
	NULL,
};

static const struct attribute_group elan_sysfs_group = {
	.attrs = elan_sysfs_entries,
};

static ssize_t acquire_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct elan_tp_data *data = dev_get_drvdata(dev);
	int retval;

	retval = mutex_lock_interruptible(&data->sysfs_mutex);
	if (retval)
		return retval;

	disable_irq(data->irq);
	data->active = false;

	data->baseline_ready = false;

	retval = elan_enable_calibrate(data);
	if (retval) {
		dev_dbg(dev, "Failed to enable calibration mode to get baseline: %d\n",
			retval);
		goto out;
	}

	msleep(250);

	retval = elan_get_max_baseline(data, &data->max_baseline);
	if (retval) {
		dev_dbg(dev, "Failed to read max baseline form device: %d\n",
			retval);
		goto out_disable_calibrate;
	}

	retval = elan_get_min_baseline(data, &data->min_baseline);
	if (retval) {
		dev_dbg(dev, "Failed to read min baseline form device: %d\n",
			retval);
		goto out_disable_calibrate;
	}

	data->baseline_ready = true;

out_disable_calibrate:
	retval = elan_disable_calibrate(data);
	if (retval)
		dev_dbg(dev, "Failed to disable calibration mode after acquiring baseline: %d\n",
			retval);
out:
	data->active = true;
	enable_irq(data->irq);
	mutex_unlock(&data->sysfs_mutex);
	return retval ?: count;
}

static ssize_t min_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct elan_tp_data *data = dev_get_drvdata(dev);
	int retval;

	retval = mutex_lock_interruptible(&data->sysfs_mutex);
	if (retval)
		return retval;

	if (!data->baseline_ready) {
		retval = -ENODATA;
		goto out;
	}

	retval = snprintf(buf, PAGE_SIZE, "%d", data->min_baseline);

out:
	mutex_unlock(&data->sysfs_mutex);
	return retval;
}

static ssize_t max_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct elan_tp_data *data = dev_get_drvdata(dev);
	int retval;

	retval = mutex_lock_interruptible(&data->sysfs_mutex);
	if (retval)
		return retval;

	if (!data->baseline_ready) {
		retval = -ENODATA;
		goto out;
	}

	retval = snprintf(buf, PAGE_SIZE, "%d", data->max_baseline);

out:
	mutex_unlock(&data->sysfs_mutex);
	return retval;
}


static DEVICE_ATTR_WO(acquire);
static DEVICE_ATTR_RO(min);
static DEVICE_ATTR_RO(max);

static struct attribute *elan_baseline_sysfs_entries[] = {
	&dev_attr_acquire.attr,
	&dev_attr_min.attr,
	&dev_attr_max.attr,
	NULL,
};

static const struct attribute_group elan_baseline_sysfs_group = {
	.name = "baseline",
	.attrs = elan_baseline_sysfs_entries,
};

static const struct attribute_group *elan_sysfs_groups[] = {
	&elan_sysfs_group,
	&elan_baseline_sysfs_group,
	NULL
};

/*
 ******************************************************************
 * lid event functions
 ******************************************************************
 */
/*
 * We rely on EV_SW and SW_LID bits to identify a LID device, and hook
 * up our filter to listen for SW_LID events to enable/disable touchpad when
 * LID is open/closed.
 */
static const struct input_device_id lid_device_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_SWBIT,
		.evbit = { BIT_MASK(EV_SW) },
		.swbit = { BIT_MASK(SW_LID) },
	},
	{ },
};

static int lid_device_connect(struct input_handler *handler,
			      struct input_dev *input_dev,
			      const struct input_device_id *id)
{
	struct input_handle *lid_handle;
	struct elan_tp_data *data = handler->private;
	struct device *dev = &data->client->dev;
	int error;

	dev_dbg(dev, "elan: LID device: %s connected", input_dev->name);
	lid_handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!lid_handle)
		return -ENOMEM;

	lid_handle->dev = input_dev;
	lid_handle->handler = handler;
	lid_handle->name = "lid_event_handler";
	lid_handle->private = handler->private;

	error = input_register_handle(lid_handle);
	if (error) {
		dev_err(dev, "Failed to register lid_event_handler, error %d\n",
		       error);
		goto err_free;
	}

	error = input_open_device(lid_handle);
	if (error) {
		dev_err(dev, "Failed to open input device, error %d\n", error);
		goto err_unregister;
	}

	return 0;
err_unregister:
	input_unregister_handle(lid_handle);
err_free:
	kfree(lid_handle);
	return error;
}

static void lid_device_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static bool lid_event_filter(struct input_handle *handle,
			     unsigned int type, unsigned int code, int value)
{
	struct elan_tp_data *data = handle->private;
	struct device *dev = &data->client->dev;
	struct input_dev *input = data->input;

	if (type == EV_SW && code == SW_LID && data->smbus == false) {
		dev_dbg(dev, "elan %s: %s touch device\n",
			dev_name(&data->client->dev),
			(value ? "disable" : "enable"));

		mutex_lock(&input->mutex);
		data->lid_status = (value ? LID_CLOSED : LID_OPEN);
		if (data->suspended) {
			mutex_unlock(&input->mutex);
			return false;
		}

		if (value == 0)	{
			/* activate touchpad if it was resumed before. */
			if (!data->active)
				elan_resume(dev);
			else
				elan_enable_power(data);
		} else {
			elan_disable_power(data);
		}
		mutex_unlock(&input->mutex);
	}

	return false;
}

static void lid_event_register_handler(struct elan_tp_data *data)
{
	int error;
	struct input_handler *lid_handler = &data->lid_handler;
	struct device *dev = &data->client->dev;

	if (data->lid_handler_registered) {
		dev_err(dev, "lid handler is registered already\n");
		return;
	}

	lid_handler->filter	= lid_event_filter;
	lid_handler->connect	= lid_device_connect;
	lid_handler->disconnect	= lid_device_disconnect;
	lid_handler->name	= "elan_lid_event_handler";
	lid_handler->id_table	= lid_device_ids;
	lid_handler->private	= data;

	error = input_register_handler(lid_handler);
	if (error) {
		dev_err(dev, "Failed to register lid handler(%d)\n", error);
		return;
	}
	data->lid_handler_registered = true;
}

static void lid_event_unregister_handler(struct elan_tp_data *data)
{
	if (data->lid_handler_registered) {
		input_unregister_handler(&data->lid_handler);
		data->lid_handler_registered = false;
	}
}

/*
 ******************************************************************
 * Elan isr functions
 ******************************************************************
 */
static int elan_check_packet(struct elan_tp_data *data, u8 *packet)
{
	u8 rid;

	if (data->smbus)
		rid = packet[0];
	else
		rid = packet[ETP_I2C_REPORT_ID_OFFSET];

	/* check report id */
	if (rid != ETP_REPORT_ID) {
		dev_err(&data->client->dev, "report id [%x] fail.\n", rid);
		return -1;
	}
	return 0;
}

static void elan_report_absolute(struct elan_tp_data *data, u8 *packet)
{
	struct input_dev *input;
	u8 *finger_data;
	bool finger_on;
	int pos_x, pos_y;
	int pressure, mk_x, mk_y;
	int i, area_x, area_y, major, minor, new_pressure;
	int btn_click;
	u8  tp_info;
	struct device *dev = &data->client->dev;

	if (!data) {
		dev_err(dev, "tp data structure is null");
		return;
	}
	input = data->input;
	if (!input) {
		dev_err(dev, "input structure is null");
		return;
	}

	if (data->smbus) {
		finger_data = &packet[ETP_SMBUS_FINGER_DATA_OFFSET];
		tp_info = packet[1];
	} else {
		finger_data = &packet[ETP_I2C_FINGER_DATA_OFFSET];
		tp_info = packet[3];
	}

	btn_click = (tp_info & 0x01);
	for (i = 0; i < ETP_MAX_FINGERS; i++) {
		finger_on = (tp_info >> (3 + i)) & 0x01;

		/* analyze touched finger raw data*/
		if (finger_on) {
			pos_x = ((finger_data[0] & 0xf0) << 4) |
							finger_data[1];
			pos_y = ((finger_data[0] & 0x0f) << 8) |
							finger_data[2];
			pos_y =  data->max_y - pos_y;
			mk_x = (finger_data[3] & 0x0f);
			mk_y = (finger_data[3] >> 4);
			pressure = finger_data[4];

			if (pos_x > data->max_x) {
				dev_err(dev, "[%d] x=%d y=%d over max_x (%d)",
					i, pos_x, pos_y, data->max_x);
				finger_data += ETP_FINGER_DATA_LEN;
				continue;
			}

			if (pos_y > data->max_y) {
				dev_err(dev, "[%d] x=%d y=%d over max_y (%d)",
					i, pos_x, pos_y, data->max_y);
				finger_data += ETP_FINGER_DATA_LEN;
				continue;
			}

			/*
			 * to avoid fat finger be as palm, so reduce the
			 * width x and y per trace
			 */
			area_x = mk_x * (data->width_x - ETP_FWIDTH_REDUCE);
			area_y = mk_y * (data->width_y - ETP_FWIDTH_REDUCE);

			major = max(area_x, area_y);
			minor = min(area_x, area_y);

			new_pressure = pressure + ETP_PRESSURE_OFFSET;
			if (new_pressure > ETP_MAX_PRESSURE)
				new_pressure = ETP_MAX_PRESSURE;

			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER,
						   true);
			input_report_abs(input, ABS_MT_POSITION_X, pos_x);
			input_report_abs(input, ABS_MT_POSITION_Y, pos_y);
			input_report_abs(input, ABS_MT_PRESSURE, new_pressure);
			input_report_abs(input, ABS_TOOL_WIDTH, mk_x);
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, major);
			input_report_abs(input, ABS_MT_TOUCH_MINOR, minor);
			finger_data += ETP_FINGER_DATA_LEN;
		} else {
			input_mt_slot(input, i);
			input_mt_report_slot_state(input,
						   MT_TOOL_FINGER, false);
		}
	}

	input_report_key(input, BTN_LEFT, (btn_click == 1));
	input_mt_report_pointer_emulation(input, true);
	input_sync(input);
}

static irqreturn_t elan_isr(int irq, void *dev_id)
{
	struct elan_tp_data *data = dev_id;
	u8 raw[ETP_MAX_REPORT_LEN];
	int retval;
	int report_len;
	struct device *dev = &data->client->dev;

	/*
	 * When device is connected to i2c bus, when all IAP page writes
	 * complete, the driver will receive interrupt and must read
	 * 0000 to confirm that IAP is finished.
	*/
	if (data->wait_signal_from_updatefw) {
		complete(&data->fw_completion);
		goto elan_isr_end;
	}

	if (data->smbus) {
		report_len = ETP_SMBUS_REPORT_LEN;
		retval = i2c_smbus_read_block_data(data->client,
						   ETP_SMBUS_PACKET_QUERY,
						   raw);
	} else {
		report_len = ETP_I2C_REPORT_LEN;
		retval = i2c_master_recv(data->client, raw, report_len);
	}

	if (retval != report_len) {
		dev_err(dev, "wrong packet len(%d)", retval);
		goto elan_isr_end;
	}

	if (elan_check_packet(data, raw) < 0) {
		dev_err(dev, "wrong packet format.");
		goto elan_isr_end;
	}

	elan_report_absolute(data, raw);

elan_isr_end:
	return IRQ_HANDLED;
}

/*
 ******************************************************************
 * Elan initialization functions
 ******************************************************************
 */
static int elan_setup_input_device(struct elan_tp_data *data)
{
	struct i2c_client *client = data->client;
	struct input_dev *input = data->input;
	unsigned int x_res, y_res;
	int max_width, min_width;

	data->product_id = elan_get_product_id(data);
	data->fw_version = elan_get_fw_version(data);
	data->sm_version = elan_get_sm_version(data);
	data->iap_version = elan_get_iap_version(data);
	data->max_x = elan_get_x_max(data);
	data->max_y = elan_get_y_max(data);
	data->width_x = data->max_x / elan_get_x_tracenum(data);
	data->width_y = data->max_y / elan_get_y_tracenum(data);
	x_res = elan_get_x_resolution(data);
	y_res = elan_get_y_resolution(data);
	max_width = max(data->width_x, data->width_y);
	min_width = min(data->width_x, data->width_y);

	dev_dbg(&client->dev,
		"Elan Touchpad Information:\n"
		"    Module product ID:  0x%04x\n"
		"    Firmware Version:  0x%04x\n"
		"    Sample Version:  0x%04x\n"
		"    IAP Version:  0x%04x\n"
		"    Max ABS X,Y:   %d,%d\n"
		"    Width X,Y:   %d,%d\n"
		"    Resolution X,Y:   %d,%d (dots/mm)\n",
		data->product_id,
		data->fw_version,
		data->sm_version,
		data->iap_version,
		data->max_x, data->max_y,
		data->width_x, data->width_y,
		x_res, y_res);

	__set_bit(EV_ABS, input->evbit);
	__set_bit(INPUT_PROP_BUTTONPAD, input->propbit);
	__set_bit(BTN_LEFT, input->keybit);

	/* Set up ST parameters */
	input_set_abs_params(input, ABS_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, data->max_y, 0, 0);
	input_abs_set_res(input, ABS_X, x_res);
	input_abs_set_res(input, ABS_Y, y_res);
	input_set_abs_params(input, ABS_PRESSURE, 0, ETP_MAX_PRESSURE, 0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, ETP_FINGER_WIDTH, 0, 0);

	/* And MT parameters */
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, data->max_y, 0, 0);
	input_abs_set_res(input, ABS_MT_POSITION_X, x_res);
	input_abs_set_res(input, ABS_MT_POSITION_Y, y_res);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			     ETP_MAX_PRESSURE, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0,
			     ETP_FINGER_WIDTH * max_width, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0,
			     ETP_FINGER_WIDTH * min_width, 0, 0);

	return 0;
}

static u8 elan_check_adapter_functionality(struct i2c_client *client)
{
	u8 ret = ELAN_ADAPTER_FUNC_NONE;

	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		ret |= ELAN_ADAPTER_FUNC_I2C;
	if (i2c_check_functionality(client->adapter,
				    I2C_FUNC_SMBUS_BYTE_DATA |
				    I2C_FUNC_SMBUS_BLOCK_DATA |
				    I2C_FUNC_SMBUS_I2C_BLOCK))
		ret |= ELAN_ADAPTER_FUNC_SMBUS;
	return ret;
}

static void elan_async_init(void *arg, async_cookie_t cookie)
{
	struct elan_tp_data *data = arg;
	struct i2c_client *client = data->client;
	unsigned long irqflags;
	int error;

	/* Initialize the touchpad. */
	error = elan_initialize(data);
	if (error < 0)
		goto err_out;

	/* Read touchpad parameters and set up input device properties. */
	error = elan_setup_input_device(data);
	if (error)
		goto err_out;

	error = input_register_device(data->input);
	if (error) {
		dev_err(&client->dev, "failed to register input device: %d\n",
			error);
		goto err_out;
	}

	/*
	 * Systems using device tree should set up interrupt via DTS,
	 * the rest will use the default falling edge interrupts.
	 */
	irqflags = client->dev.of_node ? 0 : IRQF_TRIGGER_FALLING;

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, elan_isr,
					  irqflags | IRQF_ONESHOT,
					  client->name, data);
	if (error) {
		dev_err(&client->dev, "cannot register irq=%d\n", client->irq);
		goto err_unregister_input;
	}

	data->active = true;

	error = sysfs_create_groups(&client->dev.kobj, elan_sysfs_groups);
	if (error) {
		dev_err(&client->dev, "failed to create sysfs attributes: %d\n",
			error);
		goto err_free_irq;
	}

	/* register lid event handler */
	lid_event_register_handler(data);

	data->initialized = true;

	return;

err_free_irq:
	devm_free_irq(&client->dev, client->irq, data);
err_unregister_input:
	/*
	 * Even though input device is managed we want to explicitly
	 * unregister it here so we do not end up with input device
	 * but without corresponding sysfs attributes.
	 */
	input_unregister_device(data->input);
err_out:
	dev_err(&client->dev, "Elan Trackpad probe failed: %d\n", error);
}

static void elan_disable_regulator(void *_data)
{
	struct elan_tp_data *data = _data;

	regulator_disable(data->vcc);
}

static int elan_probe(struct i2c_client *client,
		      const struct i2c_device_id *dev_id)
{
	struct device *dev = &client->dev;
	struct elan_tp_data *data;
	struct input_dev *input;
	int error;
	u8 adapter_func;

	adapter_func = elan_check_adapter_functionality(client);
	if (adapter_func == ELAN_ADAPTER_FUNC_NONE) {
		dev_err(dev, "not a supported I2C/SMBus adapter\n");
		return -EIO;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct elan_tp_data),
			    GFP_KERNEL);
	if (!data) {
		dev_err(dev, "allocate TP data structure fail.\n");
		return -ENOMEM;
	}

	/* check protocol type */
	data->smbus = adapter_func == ELAN_ADAPTER_FUNC_SMBUS;
	data->client = client;
	data->wait_signal_from_updatefw = false;
	data->lid_status = LID_UNKNOWN;
	init_completion(&data->fw_completion);
	mutex_init(&data->sysfs_mutex);

	data->input = input = devm_input_allocate_device(&client->dev);
	if (!input)
		return -ENOMEM;

	input->name = "Elan Touchpad";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &data->client->dev;

	error = input_mt_init_slots(input, ETP_MAX_FINGERS,
				    INPUT_MT_POINTER | INPUT_MT_DROP_UNUSED);
	if (error) {
		dev_err(&client->dev, "allocate MT slots failed, %d\n", error);
		return error;
	}

	/*
	 * The rest of input device's parameters will be set later,
	 * in asynchronous portion of probe.
	 */

	data->vcc = devm_regulator_get(&client->dev, "vcc");
	if (IS_ERR(data->vcc)) {
		error = PTR_ERR(data->vcc);
		if (error != -EPROBE_DEFER)
			dev_err(&client->dev,
				"Failed to get 'vcc' regulator: %d\n",
				error);
		return error;
	}

	error = regulator_enable(data->vcc);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable regulator: %d\n", error);
		return error;
	}

	error = devm_add_action(&client->dev,
				elan_disable_regulator, data);
	if (error) {
		regulator_disable(data->vcc);
		dev_err(&client->dev,
			"Failed to add disable regulator action: %d\n",
			error);
		return error;
	}

	/*
	 * Systems using device tree should set up wakeup via DTS,
	 * the rest will configure device as wakeup source by default.
	 */
	if (!client->dev.of_node)
		device_init_wakeup(&client->dev, true);

	i2c_set_clientdata(client, data);

	/* Do slower init steps asynchronously. */
	data->async_init_cookie = async_schedule(elan_async_init, data);

	return 0;
}

static int elan_remove(struct i2c_client *client)
{
	struct elan_tp_data *data = i2c_get_clientdata(client);

	/*
	 * First let's make sure our asynchronous probe has completed.
	 * Note that async_synchronize_cookie ensures that calls
	 * happened _prior_ to the one for which a cookie was given
	 * out have completed, but we need to make sure that _our_
	 * call is done.
	 */
	async_synchronize_cookie(data->async_init_cookie + 1);

	if (data->initialized) {
		lid_event_unregister_handler(data);
		sysfs_remove_groups(&client->dev.kobj, elan_sysfs_groups);
	}

	/* The rest of resources are managed ones. */
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int elan_suspend(struct device *dev)
{
	int ret = 0;
	struct elan_tp_data *data = dev_get_drvdata(dev);

	/* Set suspended flag true as system suspends. */
	data->suspended = true;

	/* Skip the step if it was turned off before. */
	if (!data->active)
		return 0;

	disable_irq(data->irq);
	data->active = false;

	if (device_may_wakeup(dev)) {
		if (data->lid_status == LID_CLOSED) {
			ret = elan_disable_power(data);
		} else {
			ret = elan_sleep(data);
			/* Enable wake from IRQ */
			data->irq_wake = (enable_irq_wake(data->irq) == 0);
		}
	} else
		ret = elan_disable_power(data);

	if (ret < 0)
		dev_err(dev, "suspend mode failed, %d\n", ret);
	return ret;
}

static int elan_resume(struct device *dev)
{
	int ret = 0;
	struct elan_tp_data *data = dev_get_drvdata(dev);

	/* set suspended flag false as system resumes. */
	data->suspended = false;
	/*
	 * Do not activate touchpad if lid is closed, defer the activation
	 * until lid is open
	 */
	if (data->lid_status == LID_CLOSED)
		return 0;

	if (device_may_wakeup(dev) && data->irq_wake)
		disable_irq_wake(data->irq);

	ret = elan_enable_power(data);
	if (ret < 0)
		dev_err(dev, "resume active power failed, %d\n", ret);
	ret = elan_initialize(data);
	if (ret < 0)
		dev_err(dev, "resume initialize tp failed, %d\n", ret);

	enable_irq(data->irq);
	data->active = true;
	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(elan_pm_ops, elan_suspend, elan_resume);

static const struct i2c_device_id elan_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, elan_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id elan_acpi_id[] = {
	{ "ELAN0000", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, elan_acpi_id);
#endif

#ifdef CONFIG_OF
static const struct of_device_id elan_of_match[] = {
	{ .compatible = "elan,i2c_touchpad" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, elan_of_match);
#endif

static struct i2c_driver elan_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm	= &elan_pm_ops,
		.acpi_match_table = ACPI_PTR(elan_acpi_id),
		.of_match_table = of_match_ptr(elan_of_match),
	},
	.probe		= elan_probe,
	.remove		= elan_remove,
	.id_table	= elan_id,
};

module_i2c_driver(elan_driver);

MODULE_AUTHOR("Duson Lin <dusonlin@emc.com.tw>");
MODULE_DESCRIPTION("Elan I2C/SMBus Touchpad driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(ELAN_DRIVER_VERSION);
