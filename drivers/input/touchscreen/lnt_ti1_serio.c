/*
 * Driver for the "Touchinterface 1" by LNT Automation GmbH,
 * implementation of the UART transport (serio attachment).
 * http://www.LNT-Automation.de/ti1/
 *
 * Copyright (C) 2014 DENX Software Engineering
 *
 * Author: Gerhard Sittig <gsi@denx.de>
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
#include <linux/errno.h>
#include <linux/input.h>	/* BUS_RS232 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serio.h>
#include <linux/slab.h>

#include "lnt_ti1_common.h"

static int tx_delay = 10;
module_param(tx_delay, int, 0);
MODULE_PARM_DESC(tx_delay, "delay between command req and resp");

struct lnt_ti1_serio_t {
	struct lnt_ti1 *ts;
	uint8_t *rxdata;
	size_t rxdwant, rxdgot;
	struct completion rxdone;
};

static irqreturn_t lnt_ti1_serio_interrupt(struct serio *serio,
					   unsigned char data,
					   unsigned int flags)
{
	struct lnt_ti1_serio_t *ti1_serio = serio_get_drvdata(serio);

	/*
	 * Store the received data if an RX buffer was provided by the
	 * caller.  Raise the completion at the buffer's end.  Ignore
	 * excess receive data or data outside of receive windows.
	 */
	if (ti1_serio->rxdata) {
		ti1_serio->rxdata[ti1_serio->rxdgot++] = data;
		if (ti1_serio->rxdgot == ti1_serio->rxdwant) {
			ti1_serio->rxdata = NULL;
			complete(&ti1_serio->rxdone);
		}
	}

	return IRQ_HANDLED;
}

static int lnt_ti1_serio_xmit_bytes(struct serio *serio, uint8_t *p, size_t l)
{
	int rc;
	uint8_t b;

	if (!p || !l)
		return 0;

	while (l-- > 0) {
		b = *p++;
		rc = serio_write(serio, b);
		if (rc)
			return rc;
	}
	return 0;
}

static int lnt_ti1_serio_command(struct device *dev,
				 enum lnt_ti1_cmd_t cmd,
				 uint8_t *rxdata, size_t rxdlen)
{
	struct serio *serio;
	struct lnt_ti1_serio_t *ti1_serio;
	uint8_t txdata;
	int rc;

	serio = to_serio_port(dev);

	ti1_serio = serio_get_drvdata(serio);

	/*
	 * Prepare for reception.
	 */
	init_completion(&ti1_serio->rxdone);
	ti1_serio->rxdwant = rxdlen;
	ti1_serio->rxdgot = 0;
	ti1_serio->rxdata = rxdata;

	/*
	 * Send out the command, optionally delay.  The response will be
	 * initiated by the peer, and will get received in an ISR.
	 */
	txdata = cmd;
	rc = lnt_ti1_serio_xmit_bytes(serio, &txdata, sizeof(txdata));
	if (rc < 0)
		return rc;
	if (tx_delay)
		msleep_interruptible(tx_delay);

	/*
	 * If RX data is expected, wait for reception to complete.
	 * Completion is sufficient, the ISR will have filled in the
	 * caller's buffer already.
	 */
	if (rxdlen) {
		if (wait_for_completion_timeout(&ti1_serio->rxdone, HZ) == 0)
			return -ETIMEDOUT;
		if (ti1_serio->rxdgot < ti1_serio->rxdwant)
			return -EAGAIN;
	}

	return 0;
}

static const struct lnt_ti1_bus_ops lnt_ti1_serio_bus_ops = {
	.bustype	= BUS_RS232,
	.commandfunc	= lnt_ti1_serio_command,
};

static void lnt_ti1_serio_lookup_ofnode(struct device *dev)
{

	/*
	 * Make sure our device has an OF node spec.  The common
	 * LNT TI1 support wants to lookup GPIO specs from it.
	 */
	if (dev->of_node)
		return;
	dev->of_node = of_find_compatible_node(NULL, NULL, "lnt,ti1-serio");
}

static int lnt_ti1_serio_connect(struct serio *serio, struct serio_driver *drv)
{
	struct lnt_ti1_serio_t *ti1_serio;
	struct lnt_ti1 *ts;
	int err;

	ti1_serio = kzalloc(sizeof(*ti1_serio), GFP_KERNEL);
	if (!ti1_serio)
		return -ENOMEM;
	serio_set_drvdata(serio, ti1_serio);
	init_completion(&ti1_serio->rxdone);

	err = serio_open(serio, drv);
	if (err)
		goto fail_drvdata;

	lnt_ti1_serio_lookup_ofnode(&serio->dev);
	ts = lnt_ti1_probe(&serio->dev, &lnt_ti1_serio_bus_ops);
	if (IS_ERR(ts)) {
		err = PTR_ERR(ts);
		goto fail_close;
	}
	ti1_serio->ts = ts;

	return 0;

fail_close:
	serio_close(serio);
fail_drvdata:
	serio_set_drvdata(serio, NULL);
	kfree(ti1_serio);
	return err;
}

static void lnt_ti1_serio_disconnect(struct serio *serio)
{
	struct lnt_ti1_serio_t *ti1_serio;

	ti1_serio = serio_get_drvdata(serio);

	lnt_ti1_remove(ti1_serio->ts);

	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	kfree(ti1_serio);
}

static struct serio_device_id lnt_ti1_serio_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_LNT_TI1,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(serio, lnt_ti1_serio_serio_ids);

static struct serio_driver lnt_ti1_serio_drv = {
	.driver		= {
		.name	= "lnt-ti1-serio",
		.owner	= THIS_MODULE,
		.pm	= &lnt_ti1_pm_ops,
	},
	.description	= "LNT TI1 UART driver",
	.id_table	= lnt_ti1_serio_serio_ids,
	.interrupt	= lnt_ti1_serio_interrupt,
	.connect	= lnt_ti1_serio_connect,
	.disconnect	= lnt_ti1_serio_disconnect,
};

module_serio_driver(lnt_ti1_serio_drv);

MODULE_AUTHOR("Gerhard Sittig <gsi@denx.de>");
MODULE_DESCRIPTION("LNT Touchinterface 1 controller UART driver");
MODULE_LICENSE("GPL");
