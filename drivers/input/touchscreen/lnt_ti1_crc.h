/*
 * CRC8 implementation for the "Touchinterface 1" by LNT Automation GmbH
 * http://www.LNT-Automation.de/ti1/
 *
 * Copyright (C) 2013 GIGATRONIK Stuttgart GmbH, KE, Jan Kurz
 *
 * Copyright (C) 2014 DENX Software Engineering, Gerhard Sittig
 * coding style cleanup, convenience wrapper
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

#ifndef LNT_TI1_CRC_H
#define LNT_TI1_CRC_H

/*
 * These CRC8 routines are specific to the LNT Touchinterface 1 driver,
 * and match the implementation of the touchscreen controller's firmware.
 */

#include <linux/types.h>

/*
 * these are intrinsic routines to
 * - seed the CRC calculation
 * - update the CRC from more input data
 * - finalize the CRC calculation
 */
uint8_t lnt_ti1_crc_init(void);
uint8_t lnt_ti1_crc_update(uint8_t crc, uint8_t *p, size_t l);
uint8_t lnt_ti1_crc_finalize(uint8_t crc);

/*
 * this is a convenience wrapper to determine
 * the CRC for a given buffer in one call
 */
uint8_t lnt_ti1_crc_calc(uint8_t *p, size_t l);

#endif /* LNT_TI1_CRC_H */
