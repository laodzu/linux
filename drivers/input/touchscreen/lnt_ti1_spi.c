/*
 * Driver for the "Touchinterface 1" by LNT Automation GmbH,
 * implementation of the SPI transport.
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

#include <linux/input.h>	/* BUS_SPI */
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "lnt_ti1_common.h"

/*
 * The maximum data rate for SPI communication to the LNT TI1 is 18MHz.
 * A delay might be required between the transmission of a request and
 * the transmission of a response, which occur within the same transfer.
 */
#define LNT_TI1_SPI_MAX_SPEED_HZ	18000000
#define LNT_TI1_SPI_CMD_DATA_MSECS	10

/*
 * This routine sends a request and receives a response.  The two parts
 * are split across two partial SPI transfers, where a delay might apply
 * between them.  Input data is sampled already during transmission of
 * the request data, to catch the reflected command byte.
 *
 * XXX TODO  Current touch controller firmware requires support for
 * "deferred responses", where a transfer that communicates the _next_
 * request will receive the response data of the _previous_ request.
 * The current core logic in the driver that is common across all TI1
 * variants implements this support, though it's rather ugly.  Recent
 * spec upgrades suggest that the situation might improve, in that the
 * availability of a NOP command might move this funny detail into the
 * transport driver here.  An even better solution would be to
 * completely eliminate this "deferred response" ugliness, and instead
 * run regular and straight forward communication like the other
 * transports do.  Both of these improvements would go here, and would
 * remove the abstraction violation from the core driver.
 *
 * For the "deferred response, NOP available" case, the actual sequence
 * to communicate a specific command would be:
 * - first transfer, send the command byte, receive nothing
 * - optional delay between "request" and "response"
 * - second transfer, send a NOP command, receive the response for the
 *   previously sent request
 *
 * For the "immediate response, delay required" case, the actual
 * sequence to communicate a specific command would be either:
 * - single transfer, send the command byte, optionally delay for a
 *   specified period of time, receive the response
 * or (to not keep the bus busy for no reason)
 * - first transfer, send the command byte
 * - optionally delay
 * - second transfer, receive the response data
 */
static int lnt_ti1_spi_command(struct device *dev,
			       enum lnt_ti1_cmd_t cmd,
			       uint8_t *rxdata, size_t rxdlen)
{
	struct spi_device *spi;
	uint8_t txdata;
	struct spi_transfer xfercmd;
	struct spi_transfer xferdat;
	struct spi_message msg;
	int rc;

	/*
	 * Get a handle to the SPI device.
	 */
	spi = to_spi_device(dev);

	/*
	 * Fill in the transfer descriptions for the request and the
	 * response parts.
	 */
	memset(&xfercmd, 0, sizeof(xfercmd));
	txdata = cmd;
	xfercmd.tx_buf = &txdata;
	xfercmd.len = sizeof(txdata);
	xfercmd.bits_per_word = 8;

	memset(&xferdat, 0, sizeof(xferdat));
	xferdat.rx_buf = rxdata;
	xferdat.len = rxdlen;
	xferdat.bits_per_word = 8;

	/*
	 * Receive the first byte already at the time when the command
	 * is sent out.  The (optional) second partial transfer only
	 * communicates the remainder starting at the second RX byte.
	 */
	xfercmd.rx_buf = xferdat.rx_buf;
	if (rxdlen) {
		xferdat.rx_buf += xfercmd.len;
		xferdat.len -= xfercmd.len;
	}

	/*
	 * Construct an SPI message description.
	 */
	spi_message_init(&msg);
	spi_message_add_tail(&xfercmd, &msg);
	if (rxdlen) {
		xfercmd.cs_change = 0;
		xfercmd.delay_usecs = LNT_TI1_SPI_CMD_DATA_MSECS * 1000;
		spi_message_add_tail(&xferdat, &msg);
	}

	/*
	 * Have the SPI message transferred.
	 */
	rc = spi_sync(spi, &msg);
	if (rc) {
		dev_err(&spi->dev, "%s() failed, rc %d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static const struct lnt_ti1_bus_ops lnt_ti1_spi_bus_ops = {
	.bustype	= BUS_SPI,
	.commandfunc	= lnt_ti1_spi_command,
};

static int lnt_ti1_spi_probe(struct spi_device *spi)
{
	struct lnt_ti1 *ts;
	int err;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	if (!spi->max_speed_hz)
		spi->max_speed_hz = LNT_TI1_SPI_MAX_SPEED_HZ;
	if (spi->max_speed_hz > LNT_TI1_SPI_MAX_SPEED_HZ)
		spi->max_speed_hz = LNT_TI1_SPI_MAX_SPEED_HZ;
	err = spi_setup(spi);
	if (err) {
		dev_err(&spi->dev, "SPI setup failed\n");
		return err;
	}

	ts = lnt_ti1_probe(&spi->dev, &lnt_ti1_spi_bus_ops);
	if (IS_ERR(ts))
		return PTR_ERR(ts);

	spi_set_drvdata(spi, ts);

	return 0;
}

static int lnt_ti1_spi_remove(struct spi_device *spi)
{
	struct lnt_ti1 *ts = spi_get_drvdata(spi);

	lnt_ti1_remove(ts);
	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id lnt_ti1_id[] = {
	{ "lnt,ti1-spi", 0, },
	{ "ti1-spi", 0, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, lnt_ti1_id);
MODULE_ALIAS("spi:lnt,ti1-spi");
MODULE_ALIAS("spi:ti1-spi");

static struct spi_driver lnt_ti1_spi_driver = {
	.driver = {
		.name	= "lnt-ti1-spi",
		.owner	= THIS_MODULE,
		.pm	= &lnt_ti1_pm_ops,
	},
	.probe		= lnt_ti1_spi_probe,
	.remove		= lnt_ti1_spi_remove,
};

module_spi_driver(lnt_ti1_spi_driver);

MODULE_AUTHOR("Gerhard Sittig <gsi@denx.de>");
MODULE_DESCRIPTION("LNT Touchinterface 1 controller SPI bus driver");
MODULE_LICENSE("GPL");
