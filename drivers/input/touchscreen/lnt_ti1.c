/*
 * driver for the "Touchinterface 1" by LNT Automation GmbH
 * http://www.LNT-Automation.de/ti1/
 *
 * Copyright (C) 2013 DENX Software Engineering
 *
 * Author: Gerhard Sittig <gsi@denx.de>
 * derived from the TSC2005 driver
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

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/pm.h>
#include <linux/spi/spi.h>

/*
 * the LNT Automation Touchinterface1 is connected to the computer
 * by means of SPI as well as an (interrupt capable) GPIO
 *
 * the TI1 controller signals via GPIO when touch samples have
 * become available, these samples then can get fetched without
 * delay until the stream of samples is exhausted, while the GPIO
 * pin level reflects the availability of more samples
 *
 * button release events get derived from both touch samples
 * without "press" information, as well as the absence of sample
 * data over a longer period of time, i.e. loss of communication
 * to the touch controller after ESD events or other issues
 */

/*
 * potential for improvement:
 * - test multi touch input events, optionally enable them by default
 * - eliminate deferred SPI responses, either by implementing it in the
 *   firmware, or by adding a NOP command valid for all modes
 * - verify CRC calculation over responses once we get real content
 * - make use of the version information and settings / capabilities
 *   once we get the respective data
 * - test and maybe re-consider the penup timer logic and its
 *   interaction with multi touch operation; consider an explicit
 *   periodic check of the communication channel when NOP is available
 * - get the fudge values from device tree? ignore those and set zero
 *   values? assume a certain percentage of the resolution?
 * - test and verify open/close and suspend/resume operation, wakeup
 * - do evaluate the heartbeat information
 * - remove the fake sample data, or move it out of the get_samples()
 *   routine and have an explicit test routine directly call the data
 *   processing routine
 * - keep reading touch sensor data as long as the pen down GPIO is low
 */

/* devel hacks, to get removed again {{{ */

/*
 * dirty devel hacks and workarounds for the interesting protocol
 *
 * SPI communication appears to suffer from an issue that does not apply
 * to other channels: the master needs to drive the slave's sending its
 * response, while the slave requires some time to execute the master's
 * request
 *
 * the touch controller firmware implements an approach of only
 * answering a previous request while receiving the next request, i.e.
 * one instruction is "in flight" while emitting this next instruction
 * communicates the response to the previous command
 *
 * this driver implements the appropriate counterpart for touch sample
 * data (the regular mode of operation, repeatedly emitting the "first
 * sample" and "next samples" requests), and does a dirty repetition for
 * config mode commands (version info, settings) [ note that not all
 * combinations of the below switches result in successful operation,
 * it's a development hack and waits for its removal ]
 *
 * it would be nice to receive the response for a request within the
 * same SPI transaction, and later firmware may support such
 * communication, but we're not there yet (this fix would then make
 * these dirty hacks obsolete); unfortunately there is no NOP command
 * either with which one might emulate the immediate response even in
 * the presence of deferred responses
 *
 * builtin touch sample response data (which clobbers or shadows what
 * was read from the touch controller) is a hack for development of the
 * data processing and deriving input events, in the absence of real
 * content in the current firmware's responses
 */
#define LNT_TI1_SPI_2x_GET_VERSION	1
#define LNT_TI1_SPI_2x_GET_SETTINGS	1
#define LNT_TI1_SPI_DEFERRED_RESPONSE	1
#define LNT_TI1_SPI_RX_DURING_CMD_TX	1
#define LNT_TI1_SPI_CMD_REFLECTION	1
#define LNT_TI1_SPI_FAKE_SAMPLE_DATA	0

/* XXX TODO (non-consuming) verification of the response packet
 * (CRC check upon lnt_ti1_command() invocations) */

static int lnt_ti1_diag = 2;
module_param_named(diag, lnt_ti1_diag, int, 0);
MODULE_PARM_DESC(diag, "verbosity level of diagnostics output");

/* }}} devel hacks */

static int lnt_ti1_mtouch = 0;
module_param_named(multi, lnt_ti1_mtouch, int, 0);
MODULE_PARM_DESC(multi, "enable/disable multi touch input events");

/*
 * maximum SPI communication data rate (18 Mbps), and the delay to
 * insert between the command byte and clocking out the response data
 */
#define LNT_TI1_SPI_MAX_SPEED_HZ	18000000
#define LNT_TI1_SPI_CMD_DATA_MSECS	0

/*
 * period of time without new touch sensor data after which to revert
 * the input event (release buttons in the absence of communication,
 * like ESD issues), firmware authors suggest to use higher values
 */
#define LNT_TI1_PENUP_TIME_MS	80

/* maximum touch screen resolution and number of fingers */
#ifndef MAX_12BIT
#define MAX_12BIT		((1 << 12) - 1)
#endif
#define MAX_COORDINATE_VALUE	MAX_12BIT
#define MAX_SUPPORTED_POINTS	10

/* CRC-8, shamelessly ripped from Simon Glass' U-Boot contribution {{{ */

/* #include "linux/crc8.h" (this API looks weird) */
static u8 crc8(const u8 *data, size_t len, u8 prev_crc);

/*
 * Copyright (c) 2013 Google, Inc
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

static u8 crc8(const u8 *data, size_t len, u8 prev_crc)
{
	unsigned int crc;
	int i;

	crc = prev_crc << 8;
	while (len-- > 0) {
		crc ^= *data++ << 8;
		for (i = 8; i; i--) {
			if (crc & 0x8000)
				crc ^= (0x1070 << 3);
			crc <<= 1;
		}
	}
	crc >>= 8;
	crc &= 0xff;
	return crc;
}

/* }}} CRC-8 */

/*
 * list of commands which the touch controller supports, and
 * structure of parameters associated with these commands
 */
enum lnt_ti1_cmd_t {
	LNT_TI1_CMD_GET_SAMPLE_0	= 0x01,
	LNT_TI1_CMD_GET_SAMPLE_9	= 0x02,
	LNT_TI1_CMD_GET_VERSION		= 0x11,
	LNT_TI1_CMD_GET_SETTINGS	= 0x12,
	LNT_TI1_CMD_MODE_TOUCH		= 0xf1,
	LNT_TI1_CMD_MODE_CONFIG		= 0xf4,
};

struct lnt_ti1 {
	struct spi_device	*spi;
	struct input_dev	*idev;
	char			phys[32];
	struct mutex		mutex;
	spinlock_t		lock;
	int			gpio;
	int			irq;
	unsigned int		max_x, max_y, max_p;
	unsigned int		fudge_x, fudge_y, fudge_p;
	int			max_id;
	bool			is_opened;
	bool			is_suspended;
	bool			is_pendown;
	struct timer_list	penup_timer;
};

/* send a command, retrieve response data */
static int lnt_ti1_command(struct lnt_ti1 *ts, enum lnt_ti1_cmd_t cmd,
			   uint8_t *rxdata, size_t rxdlen, bool chkcrc)
{
	uint8_t txdata = cmd;
	struct spi_transfer xfercmd = {
		.tx_buf = &txdata,
		.len = 1,
		.bits_per_word = 8,
	};
	struct spi_transfer xferdat = {
		.rx_buf = rxdata,
		.len = rxdlen,
		.bits_per_word = 8,
	};
	struct spi_message msg;
	int rc;

	spi_message_init(&msg);
	spi_message_add_tail(&xfercmd, &msg);
	if (rxdlen) {
		xfercmd.cs_change = 0;
		xfercmd.delay_usecs = LNT_TI1_SPI_CMD_DATA_MSECS * 1000;
		spi_message_add_tail(&xferdat, &msg);
	}
#if LNT_TI1_SPI_RX_DURING_CMD_TX
	/* receive the first byte already at the time when the command
	 * is sent out, the (optional) second partial transfer only
	 * communicates the remainder starting at the second RX byte */
	xfercmd.rx_buf = xferdat.rx_buf;
	if (rxdlen) {
		xferdat.rx_buf++;
		xferdat.len--;
	}
#endif

	rc = spi_sync(ts->spi, &msg);
	if (rc) {
		dev_err(&ts->spi->dev, "%s() failed, rc %d\n", __func__, rc);
		return rc;
	}

	if (lnt_ti1_diag > 1) {
		/* diagnostics, dump all data */
		char dattext[rxdlen * 5 + 4], *wrpos;
		size_t idx;

		wrpos = dattext; *wrpos = '\0';
		for (idx = 0; idx < rxdlen; idx++) {
			wrpos += sprintf(wrpos, " 0x%02x", rxdata[idx]);
		}
		dev_info(&ts->spi->dev, "DATA cmd[0x%x] data[%s ]\n", cmd, dattext);
	}

	if (chkcrc) {
		uint8_t got, want;

		got = rxdata[rxdlen - 1];
		want = crc8(rxdata, rxdlen - 1, 0);
		if (lnt_ti1_diag > 0)
			dev_info(&ts->spi->dev,
				 "%s() CRC want[0x%02x] got[0x%02x]\n",
				 __func__, want, got);
		return -EINVAL;
	}

	return 0;
}

static void lnt_ti1_check_heartbeat(struct lnt_ti1 *ts, int stamp)
{

	/* XXX TODO
	 *
	 * track the progress of the 'stamp' information, detect
	 * changes between zero and non-zero (idle and active),
	 * detect when non-zero stamps don't move forward any
	 * longer
	 *
	 * this may be considered some kind of "ESD postprocessing"
	 *
	 * ATM the touch controller firmware appears to not provide
	 * these stamps, so we don't process anything here yet
	 */
	(void)ts;
	(void)stamp;
}

static void lnt_ti1_update_pen_state(struct lnt_ti1 *ts,
				     int x, int y, int press,
				     int id, int down)
{
	if (lnt_ti1_mtouch) {
		input_mt_slot(ts->idev, id);
		input_mt_report_slot_state(ts->idev, MT_TOOL_FINGER, !!down);
		if (down) {
			input_report_abs(ts->idev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->idev, ABS_MT_POSITION_Y, y);
			input_report_abs(ts->idev, ABS_MT_PRESSURE, press);
		}
		input_mt_report_pointer_emulation(ts->idev, true);
	} else {
		if (down) {
			input_report_abs(ts->idev, ABS_X, x);
			input_report_abs(ts->idev, ABS_Y, y);
			input_report_abs(ts->idev, ABS_PRESSURE, press);
			if (!ts->is_pendown) {
				input_report_key(ts->idev, BTN_TOUCH, !!down);
				ts->is_pendown = true;
			}
		} else {
			input_report_abs(ts->idev, ABS_PRESSURE, 0);
			if (ts->is_pendown) {
				input_report_key(ts->idev, BTN_TOUCH, 0);
				ts->is_pendown = false;
			}
		}
	}
	input_sync(ts->idev);
	if (lnt_ti1_diag > 0)
		dev_info(&ts->spi->dev, "at(%d, %d) id(%d) down(%d)\n",
			 x, y, id, down);
}

/* these accessors _consume_ the data they read (advance in the data stream) */

static inline uint8_t get_u8(uint8_t **p, size_t *l)
{
	uint8_t b;

	if (!*l)
		return 0;
	b = *(*p)++; l--;
	return b;
}

static inline uint16_t get_u16_le(uint8_t **p, size_t *l)
{
	uint16_t v;

	v = get_u8(p, l);
	v |= get_u8(p, l) << 8;
	return v;
}

static bool lnt_ti1_process_header(struct lnt_ti1 *ts,
				   uint8_t **data, size_t *dlen,
				   uint8_t want_cmd)
{
	uint8_t b;
#define HEADER_STATUS_GET_ACTIVE(b)	((b) & BIT(0))
#define HEADER_STATUS_GET_HBEAT(b)	(((b) & 0xf0) >> 4)

	/*
	 * header layout:
	 * - 8bit, reflection of the command byte which the response
	 *   corresponds to
	 * - 8bit, status (active aka initialized) and heartbeat
	 *   (free running counter, non-zero when active)
	 */
#if LNT_TI1_SPI_CMD_REFLECTION
	if (*dlen < 2)
		return false;
	b = get_u8(data, dlen);
	if (b != want_cmd)
		return false;
#else
	(void)want_cmd;
#endif

	if (*dlen < 1)
		return false;
	b = get_u8(data, dlen);
	if (!HEADER_STATUS_GET_ACTIVE(b))
		return false;
	lnt_ti1_check_heartbeat(ts, HEADER_STATUS_GET_HBEAT(b));
	return true;
}

static bool lnt_ti1_process_sample(struct lnt_ti1 *ts,
				   uint8_t **data, size_t *dlen)
{
	uint8_t raw_status;
#define SAMPLE_STATUS_IS_VALID		BIT(2)
#define SAMPLE_STATUS_BIT_IN_RANGE	BIT(1)
#define SAMPLE_STATUS_IS_PRESSED	BIT(0)
	uint8_t raw_touches;
#define SAMPLE_TOUCH_GET_COUNT(b)	(((b) & 0xf0) >> 4)
#define SAMPLE_TOUCH_GET_NUMBER(b)	(((b) & 0x0f) >> 0)
	uint16_t raw_x, raw_y;
	int pressure, finger;
	bool down;
	unsigned long flags;

	/*
	 * touch sample layout:
	 *   - 8bit, "status"
	 *   - 8bit, "touches"
	 *   - 16bit LE, X coord
	 *   - 16bit LE, Y coord
	 *
	 * important: first consume all input data, then do the
	 * checking and the interpretation and input events
	 */
	if (*dlen < 6)
		return false;
	raw_status = get_u8(data, dlen);
	raw_touches = get_u8(data, dlen);
	raw_x = get_u16_le(data, dlen);
	raw_y = get_u16_le(data, dlen);
	if (!(raw_status & SAMPLE_STATUS_IS_VALID))
		return false;
	pressure = ts->max_p;
	finger = raw_touches;
	down = !!(raw_status & SAMPLE_STATUS_IS_PRESSED);
	spin_lock_irqsave(&ts->lock, flags);
	lnt_ti1_update_pen_state(ts, raw_x, raw_y, pressure, finger, down);
	spin_unlock_irqrestore(&ts->lock, flags);
	return true;
}

static void lnt_ti1_discard_sample(struct lnt_ti1 *ts,
				   uint8_t **data, size_t *dlen)
{
	(void)get_u8(data, dlen);
	(void)get_u8(data, dlen);
	(void)get_u16_le(data, dlen);
	(void)get_u16_le(data, dlen);
}

/*
 * interpret the information in a buffer which holds adjacent copies
 * of the "first sample" and "next samples" responses
 */
static bool lnt_ti1_parse_samples(struct lnt_ti1 *ts,
				  uint8_t *data, size_t dlen)
{
	size_t sample_count;
#define LNT_TI1_CMD_GET_SAMPLE_9_MAX_SAMPLES	9

	/*
	 * "first sample" response layout:
	 * - response header
	 * - 8bit, number of available samples
	 * - one sample
	 */
	if (!lnt_ti1_process_header(ts, &data, &dlen, LNT_TI1_CMD_GET_SAMPLE_0))
		return false;
	if (dlen < 1)
		return false;
	sample_count = get_u8(&data, &dlen);
	if (sample_count > 0) {
		if (!lnt_ti1_process_sample(ts, &data, &dlen))
			return false;
		sample_count--;
	} else {
		lnt_ti1_discard_sample(ts, &data, &dlen);
	}
	(void)get_u8(&data, &dlen);	/* skip the CRC */

	/*
	 * don't take the shortcut of returning here when sample count
	 * has become zero, instead always do process the "next samples"
	 * response, for the following reasons
	 * - the controller may have more samples buffered than this
	 *   resonse can carry, i.e. we limit the processing here to what
	 *   is contained in the response
	 * - the "next samples" response is more recent than the "first
	 *   sample" response, i.e. the number might have increased during
	 *   transmission of those two responses
	 */

	/*
	 * "next samples" response layout:
	 * - response header
	 * - 8bit, number of available samples
	 * - up to nine samples, each consisting of
	 */
	if (!lnt_ti1_process_header(ts, &data, &dlen, LNT_TI1_CMD_GET_SAMPLE_9))
		return false;
	if (dlen < 1)
		return false;
	sample_count = get_u8(&data, &dlen);
	if (sample_count > LNT_TI1_CMD_GET_SAMPLE_9_MAX_SAMPLES)
		sample_count = LNT_TI1_CMD_GET_SAMPLE_9_MAX_SAMPLES;
	while (sample_count--) {
		if (!lnt_ti1_process_sample(ts, &data, &dlen))
			return false;
	}
	(void)get_u8(&data, &dlen);	/* skip the CRC */

	return true;
}

static int lnt_ti1_get_samples(struct lnt_ti1 *ts)
{
	uint8_t buff[96];
	size_t buffoff, chunklen;
	int rc;

	/* acquire both "first sample" and "next samples" responses */
	buffoff = 0;

#if LNT_TI1_SPI_DEFERRED_RESPONSE

	/* emit "first sample" and "next samples" requests in immediate
	 * succession, passing the _next_ opcode while fetching the
	 * _previous_ response within the SPI transaction (yes, it's a
	 * funny protocol which the controller is speaking ...) */

	chunklen = 10;
	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_SAMPLE_9, buff + buffoff, chunklen, 0);
	if (rc)
		return rc;
	buffoff += chunklen;

	chunklen = 58;
	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_SAMPLE_0, buff + buffoff, chunklen, 0);
	if (rc)
		return rc;
	buffoff += chunklen;

#else	/* LNT_TI1_SPI_DEFERRED_RESPONSE */

	/* expect the peer to respond to this very request within the
	 * same SPI transaction (optionally sending the request twice to
	 * make sure the response belongs to the request, potentially
	 * risking loss of data by transferring yet not processing it),
	 * it's the approach of assuming regular communication yet
	 * dealing with intermediate development hacks */

	chunklen = 10;
	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_SAMPLE_0, buff, chunklen, 0);
	if (rc)
		return rc;
	buffoff += chunklen;

	chunklen = 58;
	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_SAMPLE_9, buff, chunklen, 0);
	if (rc)
		return rc;
	buffoff += chunklen;

#endif	/* LNT_TI1_SPI_DEFERRED_RESPONSE */

#if LNT_TI1_SPI_FAKE_SAMPLE_DATA
#define MAKE16LE(x)			(x) % 256, (x) / 256
#define MAKESTS(val, down)		((val) ? 4 : 0) | ((down) ? 1 : 0)
#define MAKESMPL(val, down, num, x, y)	MAKESTS(val, down), num, MAKE16LE(x), MAKE16LE(y)
#define EMPTYSMPL			0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	do {
		static size_t test_case = 0;
		static const uint8_t test_data_invalid[] = {
			2,
			0x01, 0x00,
		};
		static const uint8_t test_data_incomplete[] = {
			10,
			/* incomplete (0x01 but not 0x02) */
			0x01, 0x21,
			0,
			EMPTYSMPL,
			0x00,
		};
		static const uint8_t test_data_no_samples[] = {
			10 + 58,
			/* full buffer but no samples */
			0x01, 0x21,
			0,
			EMPTYSMPL,
			0x00,
			0x02, 0x31,
			0,
			EMPTYSMPL, EMPTYSMPL, EMPTYSMPL, EMPTYSMPL,
			EMPTYSMPL, EMPTYSMPL, EMPTYSMPL, EMPTYSMPL,
			EMPTYSMPL,
			0x00,
		};
		static const uint8_t test_data_one_sample_penup[] = {
			10 + 58,
			/* test case: one sample, pen up */
			0x01, 0x21,
			1,
			MAKESMPL(1, 0, 0, 0, 0),
			0x00,
			0x02, 0x31,
			0,
			EMPTYSMPL, EMPTYSMPL, EMPTYSMPL, EMPTYSMPL,
			EMPTYSMPL, EMPTYSMPL, EMPTYSMPL, EMPTYSMPL,
			EMPTYSMPL,
			0x00,
		};
		static const uint8_t test_data_one_sample_pendown[] = {
			10 + 58,
			/* test case: one sample, pen down at 200/100 */
			0x01, 0x21,
			1,
			MAKESMPL(1, 1, 2, 200, 100),
			0x00,
			0x02, 0x31,
			0,
			EMPTYSMPL, EMPTYSMPL, EMPTYSMPL, EMPTYSMPL,
			EMPTYSMPL, EMPTYSMPL, EMPTYSMPL, EMPTYSMPL,
			EMPTYSMPL,
			0x00,
		};
		static const uint8_t test_data_three_samples_pendown[] = {
			10 + 58,
			/* test case: three samples, pen down at 220/110,
			 * pen down at 240/120, pen down at 260/130 */
			0x01, 0x21,
			1,
			MAKESMPL(1, 1, 3, 220, 110),
			0x00,
			0x02, 0x31,
			2,
			MAKESMPL(1, 1, 3, 240, 120),
			MAKESMPL(1, 1, 3, 260, 130),
			EMPTYSMPL, EMPTYSMPL, EMPTYSMPL, EMPTYSMPL,
			EMPTYSMPL, EMPTYSMPL, EMPTYSMPL,
			0x00,
		};
		static struct {
			const uint8_t *data;
			const char *caption;
		} test_data[] = {
			{ test_data_invalid, "invalid", },
			{ test_data_incomplete, "incomplete", },
			{ test_data_no_samples, "no samples", },
			{ test_data_one_sample_penup, "one sample, pen up", },
			{ test_data_one_sample_pendown, "one sample, pen down", },
			{ test_data_three_samples_pendown, "three samples, pen down", },
		};

		const uint8_t *p;
		size_t l;

		if (lnt_ti1_diag > 0)
			dev_info(&ts->spi->dev, "FAKE data, case %d [%s]\n",
				 test_case, test_data[test_case].caption);
		p = test_data[test_case].data;
		test_case++;
		test_case %= ARRAY_SIZE(test_data);
		l = *p++;
		if (l > buffoff)
			l = buffoff;
		memmove(buff, p, l);
	} while (0);
#endif

	/* interpret the (chunked) response */
	lnt_ti1_parse_samples(ts, buff, buffoff);

	return 0;
}

static int lnt_ti1_read_id_data(struct lnt_ti1 *ts)
{
	u8 iddata[14];
	int rc;

	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_VERSION, iddata, sizeof(iddata), 0);
	if (rc)
		dev_err(&ts->spi->dev, "ID data read error %d\n", rc);

#if LNT_TI1_SPI_2x_GET_VERSION
	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_VERSION, iddata, sizeof(iddata), 0);
	if (rc)
		dev_err(&ts->spi->dev, "ID data read error %d\n", rc);
#endif

	return rc;
}

/* XXX TODO fill in property results */
static int lnt_ti1_read_settings(struct lnt_ti1 *ts)
{
	u8 settings[14];
	int rc;

	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_SETTINGS, settings, sizeof(settings), 0);
	if (rc)
		dev_err(&ts->spi->dev, "settings read error %d\n", rc);
#if LNT_TI1_SPI_2x_GET_SETTINGS
	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_SETTINGS, settings, sizeof(settings), 0);
	if (rc)
		dev_err(&ts->spi->dev, "settings read error %d\n", rc);
#endif

	return rc;
}

static irqreturn_t lnt_ti1_irq_thread(int irq, void *args)
{
	struct lnt_ti1 *ts;
	int rc;
	unsigned long flags;

	ts = args;
	if (lnt_ti1_diag > 0)
		dev_info(&ts->spi->dev, "IRQ thread ...\n");

#if 1
	rc = lnt_ti1_get_samples(ts);
#else
	/* XXX TODO turn this into such a loop? */
	while (!gpio_get_value(ts->gpio)) {
		rc = lnt_ti1_get_samples(ts);
	}
#endif

	/*
	 * re-arm the penup timer if the pen is down, to have the
	 * timeout lift the pen in case the touch controller won't
	 * provide an "up" sample, or stops communicating to us
	 */
	if (ts->is_pendown || lnt_ti1_mtouch) {
		spin_lock_irqsave(&ts->lock, flags);
		mod_timer(&ts->penup_timer,
			  jiffies + msecs_to_jiffies(LNT_TI1_PENUP_TIME_MS));
		spin_unlock_irqrestore(&ts->lock, flags);
	}

	return IRQ_HANDLED;
}

/* a period of time without touch samples has passed while the pen is down */
static void lnt_ti1_penup_timer(unsigned long data)
{
	struct lnt_ti1 *ts = (void *)data;
	unsigned long flags;
	int finger;

	if (lnt_ti1_diag > 0)
		dev_info(&ts->spi->dev, "penup timer\n");

	spin_lock_irqsave(&ts->lock, flags);
	if (lnt_ti1_mtouch) {
		finger = MAX_SUPPORTED_POINTS;
		while (finger--)
			lnt_ti1_update_pen_state(ts, 0, 0, 0, finger, 0);
	} else {
		lnt_ti1_update_pen_state(ts, 0, 0, 0, 0, 0);
	}
	spin_unlock_irqrestore(&ts->lock, flags);
}

static void lnt_ti1_start_scan(struct lnt_ti1 *ts)
{
	(void)lnt_ti1_command(ts, LNT_TI1_CMD_MODE_TOUCH, NULL, 0, 0);
#if LNT_TI1_SPI_DEFERRED_RESPONSE
	(void)lnt_ti1_command(ts, LNT_TI1_CMD_GET_SAMPLE_0, NULL, 0, 0);
#endif
}

static void lnt_ti1_stop_scan(struct lnt_ti1 *ts)
{
	(void)lnt_ti1_command(ts, LNT_TI1_CMD_MODE_CONFIG, NULL, 0, 0);
}

/* must be called with ts->mutex held */
static void __lnt_ti1_disable(struct lnt_ti1 *ts)
{
	lnt_ti1_stop_scan(ts);

	disable_irq(ts->irq);
	del_timer_sync(&ts->penup_timer);
	enable_irq(ts->irq);
}

/* must be called with ts->mutex held */
static void __lnt_ti1_enable(struct lnt_ti1 *ts)
{
	lnt_ti1_start_scan(ts);
}

static int lnt_ti1_open(struct input_dev *input)
{
	struct lnt_ti1 *ts = input_get_drvdata(input);

	mutex_lock(&ts->mutex);
	if (!ts->is_suspended)
		__lnt_ti1_enable(ts);
	ts->is_opened = true;
	mutex_unlock(&ts->mutex);
	return 0;
}

static void lnt_ti1_close(struct input_dev *input)
{
	struct lnt_ti1 *ts = input_get_drvdata(input);

	mutex_lock(&ts->mutex);
	if (!ts->is_suspended)
		__lnt_ti1_disable(ts);
	ts->is_opened = false;
	mutex_unlock(&ts->mutex);
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int lnt_ti1_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct lnt_ti1 *ts = spi_get_drvdata(spi);

	mutex_lock(&ts->mutex);
	if (!ts->is_suspended && ts->is_opened)
		__lnt_ti1_disable(ts);
	ts->is_suspended = true;
	mutex_unlock(&ts->mutex);
	return 0;
}

static int lnt_ti1_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct lnt_ti1 *ts = spi_get_drvdata(spi);

	mutex_lock(&ts->mutex);
	if (ts->is_suspended && ts->is_opened)
		__lnt_ti1_enable(ts);
	ts->is_suspended = false;
	mutex_unlock(&ts->mutex);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(lnt_ti1_pm_ops, lnt_ti1_suspend, lnt_ti1_resume);

static int lnt_ti1_probe(struct spi_device *spi)
{
	struct lnt_ti1 *ts;
	struct input_dev *input_dev;
	struct device_node *np;
	unsigned int gpio, irq;
	int rc;

	/* allocate the touchscreen device information, and attach the
	 * SPI communication channel and the input device */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		rc = -ENOMEM;
		goto err_free_mem;
	}
	ts->spi = spi;
	spi_set_drvdata(spi, ts);
	input_dev = input_allocate_device();
	if (!input_dev) {
		rc = -ENOMEM;
		goto err_free_mem;
	}
	ts->idev = input_dev;

	/* setup SPI communication first */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	if (!spi->max_speed_hz)
		spi->max_speed_hz = LNT_TI1_SPI_MAX_SPEED_HZ;
	rc = spi_setup(spi);
	if (rc)
		goto err_free_mem;

	/* talk to the peer, test communication */
	if (1)
		lnt_ti1_read_id_data(ts);

	/* fetch touch controller properties */
	rc = lnt_ti1_read_settings(ts);
	if (rc)
		goto err_free_mem;
	/* XXX TODO do interpret returned settings (resolution),
	 * the below values are hardcoded assumptions */
	ts->fudge_x	= 4;
	ts->fudge_y	= 8;
	ts->fudge_p	= 2;
	ts->max_x	= MAX_COORDINATE_VALUE;
	ts->max_y	= MAX_COORDINATE_VALUE;
	ts->max_p	= MAX_COORDINATE_VALUE;

	/* arrange for background work and synchronization */
	mutex_init(&ts->mutex);
	spin_lock_init(&ts->lock);
	setup_timer(&ts->penup_timer, lnt_ti1_penup_timer, (unsigned long)ts);

	/* configure the input device */
	snprintf(ts->phys, sizeof(ts->phys), "%s/input-ts",
		 dev_name(&spi->dev));
	input_dev->name = "lnt-ti1 touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = &spi->dev;
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, 0, ts->max_x, ts->fudge_x, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ts->max_y, ts->fudge_y, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, ts->max_p, ts->fudge_p, 0);
	if (lnt_ti1_mtouch) {
		input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
				     ts->max_x, ts->fudge_x, 0);
		input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
				     ts->max_y, ts->fudge_y, 0);
		input_mt_init_slots(input_dev, MAX_SUPPORTED_POINTS, 0);
	}
	input_dev->open = lnt_ti1_open;
	input_dev->close = lnt_ti1_close;
	input_set_drvdata(input_dev, ts);

	/* ensure the touchscreen is off, register for notifications
	 * from the touch controller */
	lnt_ti1_stop_scan(ts);

	np = spi->dev.of_node;
	gpio = of_get_gpio(np, 0);
	rc = gpio_request(gpio, "touch-pen");
	if (rc) {
		dev_err(&ts->spi->dev, "GPIO request failed, rc %d\n", rc);
		goto err_free_gpio_irq;
	}
	ts->gpio = gpio;
	rc = gpio_direction_input(ts->gpio);
	if (rc) {
		dev_err(&ts->spi->dev, "GPIO request failed, rc %d\n", rc);
		goto err_free_gpio_irq;
	}
	irq = gpio_to_irq(ts->gpio);
	rc = request_threaded_irq(irq, NULL, lnt_ti1_irq_thread,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				  "touch-pen", ts);
	if (rc) {
		dev_err(&ts->spi->dev, "IRQ registration failed, rc %d\n", rc);
		goto err_free_gpio_irq;
	}
	ts->irq = irq;
	dev_dbg(&ts->spi->dev, "GPIO %d, IRQ %d\n", gpio, irq);

	/* register the input device, everything was setup */
	rc = input_register_device(ts->idev);
	if (rc) {
		dev_err(&spi->dev, "input device registration failed, rc %d\n", rc);
		goto err_free_gpio_irq;
	}

	/* have the pen IRQ wake the machine from sleep */
	irq_set_irq_wake(ts->irq, 1);
	return 0;

err_free_gpio_irq:
	if (ts && ts->irq)
		free_irq(ts->irq, ts);
	if (ts && ts->gpio)
		gpio_free(ts->gpio);
err_free_mem:
	if (ts && ts->idev)
		input_free_device(ts->idev);
	if (ts)
		kfree(ts);
	return rc;
}

static int lnt_ti1_remove(struct spi_device *spi)
{
	struct lnt_ti1 *ts = spi_get_drvdata(spi);

	free_irq(ts->irq, ts);
	gpio_free(ts->gpio);
	input_unregister_device(ts->idev);
	kfree(ts);

	spi_set_drvdata(spi, NULL);
	return 0;
}

static struct spi_driver lnt_ti1_driver = {
	.driver	= {
		.name	= "lnt-ti1",
		.owner	= THIS_MODULE,
		.pm	= &lnt_ti1_pm_ops,
	},
	.probe	= lnt_ti1_probe,
	.remove	= lnt_ti1_remove,
};

module_spi_driver(lnt_ti1_driver);

MODULE_AUTHOR("Gerhard Sittig <gsi@denx.de>");
MODULE_DESCRIPTION("LNT Automation Touchinterface1 driver");
MODULE_LICENSE("GPL");	/* Licensed under the GPL-2 or later */
MODULE_ALIAS("spi:lnt-ti1");

/*
 * vim:foldmethod=marker textwidth=72
 */
