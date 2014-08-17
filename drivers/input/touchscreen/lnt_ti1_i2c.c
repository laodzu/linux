/*
 * Driver for the "Touchinterface 1" by LNT Automation GmbH,
 * implementation of the I2C transport.
 * http://www.LNT-Automation.de/ti1/
 *
 * Copyright (C) 2013-2014 DENX Software Engineering
 *
 * Author: Gerhard Sittig <gsi@denx.de>
 * modelled after the AD7879 driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>	/* BUS_I2C */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm.h>
#include <linux/types.h>

#include "lnt_ti1_common.h"

static int do_10bit_address;
module_param_named(tenbit, do_10bit_address, int, 0);
MODULE_PARM_DESC(tenbit, "use 10bit i2c slave address");

static int tx_delay = 10;
module_param(tx_delay, int, 0);
MODULE_PARM_DESC(tx_delay, "delay between command req and resp");

static int lnt_ti1_i2c_command(struct device *dev,
			       enum lnt_ti1_cmd_t cmd,
			       uint8_t *rxdata, size_t rxdlen)
{
	struct i2c_client *client;
	uint8_t cmd_buf;
	int rc;

	client = to_i2c_client(dev);

	/*
	 * Send the request in a separate transmission.
	 */
	cmd_buf = cmd;
	rc = i2c_master_send(client, &cmd_buf, sizeof(cmd_buf));
	if (rc < 0)
		return rc;
	if (tx_delay)
		msleep_interruptible(tx_delay);

	/*
	 * Fetch the response in a separate transmission.
	 */
	if (rxdlen) {
		rc = i2c_master_recv(client, rxdata, rxdlen);
		if (rc < 0)
			return rc;
	}

	return 0;
}

static const struct lnt_ti1_bus_ops lnt_ti1_i2c_bus_ops = {
	.bustype	= BUS_I2C,
	.commandfunc	= lnt_ti1_i2c_command,
};

static int lnt_ti1_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct lnt_ti1 *ts;
	uint32_t features;

	features = I2C_FUNC_I2C;
	if (do_10bit_address)
		features |= I2C_FUNC_10BIT_ADDR;
	else
		features &= ~I2C_FUNC_10BIT_ADDR;
	if (!i2c_check_functionality(client->adapter, features)) {
		dev_err(&client->dev, "needed I2C functionality is missing\n");
		return -EIO;
	}

	if (do_10bit_address)
		client->flags |= I2C_CLIENT_TEN;
	else
		client->flags &= ~I2C_CLIENT_TEN;

	ts = lnt_ti1_probe(&client->dev,
			   &lnt_ti1_i2c_bus_ops);
	if (IS_ERR(ts))
		return PTR_ERR(ts);

	i2c_set_clientdata(client, ts);

	return 0;
}

static int lnt_ti1_i2c_remove(struct i2c_client *client)
{
	struct lnt_ti1 *ts = i2c_get_clientdata(client);

	lnt_ti1_remove(ts);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id lnt_ti1_id[] = {
	{ "lnt,ti1-i2c", 0, },
	{ "ti1-i2c", 0, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, lnt_ti1_id);

static struct i2c_driver lnt_ti1_i2c_driver = {
	.driver = {
		.name	= "lnt-ti1-i2c",
		.owner	= THIS_MODULE,
		.pm	= &lnt_ti1_pm_ops,
	},
	.probe		= lnt_ti1_i2c_probe,
	.remove		= lnt_ti1_i2c_remove,
	.id_table	= lnt_ti1_id,
};

module_i2c_driver(lnt_ti1_i2c_driver);

MODULE_AUTHOR("Gerhard Sittig <gsi@denx.de>");
MODULE_DESCRIPTION("LNT Touchinterface 1 controller I2C bus driver");
MODULE_LICENSE("GPL");
