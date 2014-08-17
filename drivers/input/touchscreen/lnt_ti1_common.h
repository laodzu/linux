/*
 * Driver for the "Touchinterface 1" by LNT Automation GmbH
 * http://www.LNT-Automation.de/ti1/
 *
 * Copyright (C) 2013-2014 DENX Software Engineering
 *
 * Author: Gerhard Sittig <gsi@denx.de>
 * alternative I2C/SPI support was modelled after the AD7879 driver
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
#ifndef LNT_TI1_COMMON_H
#define LNT_TI1_COMMON_H

#include <linux/types.h>

struct lnt_ti1;
struct device;

enum lnt_ti1_cmd_t {
	LNT_TI1_CMD_GET_SAMPLE_0	= 0x01,
	LNT_TI1_CMD_GET_SAMPLE_9	= 0x02,
	LNT_TI1_CMD_GET_VERSION		= 0x11,
	LNT_TI1_CMD_SET_SETTINGS	= 0x12,
	LNT_TI1_CMD_GET_SETTINGS	= 0x13,
	LNT_TI1_CMD_MODE_TOUCH		= 0xf1,
	LNT_TI1_CMD_MODE_CONFIG		= 0xf4,
};

struct lnt_ti1_bus_ops {
	uint16_t bustype;
	uint16_t vendor;
	int (*commandfunc)(struct device *dev,
			   enum lnt_ti1_cmd_t cmd,
			   uint8_t *rxdata, size_t rxdlen);
};

extern const struct dev_pm_ops lnt_ti1_pm_ops;

struct lnt_ti1 *lnt_ti1_probe(struct device *dev,
			      const struct lnt_ti1_bus_ops *busops);
void lnt_ti1_remove(struct lnt_ti1 *ts);

#endif /* LNT_TI1_COMMON_H */
