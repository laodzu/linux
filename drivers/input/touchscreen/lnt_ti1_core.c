/*
 * Driver for the "Touchinterface 1" by LNT Automation GmbH
 * http://www.LNT-Automation.de/ti1/
 *
 * Copyright (C) 2013-2014 DENX Software Engineering
 *
 * Author: Gerhard Sittig <gsi@denx.de>
 * the initial implementation was modelled after the TSC2005 driver
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
#include <linux/slab.h>

#include "lnt_ti1_common.h"
#include "lnt_ti1_crc.h"

/*
 * The LNT Automation Touchinterface 1 is connected to the computer by
 * means of e.g. SPI or I2C as well as an (interrupt capable) GPIO line.
 *
 * The TI1 controller signals via GPIO when touch samples have become
 * available.  These samples then can get fetched without delay until
 * the stream of samples is exhausted, while the GPIO pin level reflects
 * the availability of more samples.  Optionally the firmware might
 * re-trigger the GPIO signal (de-assert and re-assert it) as more
 * samples become available while previous samples still are pending.
 *
 * Button press events always get communicated in exchanged packets.
 * Button release events get derived from both of the following
 * conditions:  touch samples without "press" information, as well as
 * the absence of sample data over a longer period of time.  The latter
 * approach copes with loss of communication to the touch controller
 * after e.g. ESD events or other issues, too.
 */

static int lnt_ti1_diag;
module_param_named(diag, lnt_ti1_diag, int, 0);
MODULE_PARM_DESC(diag, "verbosity level of diagnostics output");

static int lnt_ti1_mtouch = 0;
module_param_named(multi, lnt_ti1_mtouch, int, 0);
MODULE_PARM_DESC(multi, "enable multi touch input events");

static int tx_delay = 10;
module_param(tx_delay, int, 0);
MODULE_PARM_DESC(tx_delay, "delay between commands");

/*
 * The period of time without new touch sensor data, after which to
 * revert the input event (release buttons in the absence of
 * communication, like ESD issues).  Firmware authors suggest to use
 * higher values (which ones exactly?).
 */
#define LNT_TI1_PENUP_TIME_MS	80

/* The maximum touch screen resolution and number of fingers. */
#ifndef MAX_12BIT
#define MAX_12BIT		((1 << 12) - 1)
#endif
#define MAX_COORDINATE_VALUE	MAX_12BIT
#define MAX_SUPPORTED_POINTS	10

struct lnt_ti1 {
	const struct lnt_ti1_bus_ops *busops;
	struct device *dev;
	struct input_dev *idev;
	char phys[32];
	spinlock_t lock;
	int gpio;
	int irq;
	unsigned int max_x, max_y, max_p;
	unsigned int fudge_x, fudge_y, fudge_p;
	unsigned int max_fingers;
	bool is_active;
	bool is_suspended;
	bool is_pendown;
	struct timer_list penup_timer;
	int event_count;
};

/* This routine sends a command, and retrieves response data. */
static int lnt_ti1_command(struct lnt_ti1 *ts, enum lnt_ti1_cmd_t cmd,
			   uint8_t *rxdata, size_t rxdlen)
{
	int rc;
	uint8_t got, want;

	rc = ts->busops->commandfunc(ts->dev, cmd, rxdata, rxdlen);
	if (tx_delay)
		msleep_interruptible(tx_delay);
	if (rc < 0) {
		dev_err(ts->dev, "communication of command failed\n");
		return rc;
	}

	if (lnt_ti1_diag > 1) {
		/* diagnostics, dump all data */
		if (rxdlen) {
			dev_info(ts->dev, "%s() sent cmd[0x%x], got data:\n",
				 __func__, cmd);
			print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 1,
				       rxdata, rxdlen, 0);
		} else {
			dev_info(ts->dev, "%s() sent cmd[0x%x], no data\n",
				 __func__, cmd);
		}
	}

	if (rxdlen) {
		got = rxdata[rxdlen - 1];
		want = lnt_ti1_crc_calc(rxdata, rxdlen - 1);
		if (want != got) {
			dev_warn(ts->dev,
				 "CRC mismatch in the protocol response, want 0x%02x, got 0x%02x\n",
				 want, got);
			return -EINVAL;
		}
	}

	return 0;
}

static void lnt_ti1_check_heartbeat(struct lnt_ti1 *ts, int stamp)
{

	/*
	 * XXX TODO
	 *
	 * This routine might track the progress of the 'stamp'
	 * information.  To detect changes between zero and non-zero,
	 * which reflect changes between the idle and the active state.
	 * And to detect when non-zero stamps don't move forward any
	 * longer, which means that packets get exchanged yet the
	 * firmware's internal operation appears to be stalled.
	 *
	 * This may be considered some additional kind of "ESD
	 * postprocessing".
	 *
	 * But at the moment the touch controller firmware appears to
	 * not provide these stamps, so we don't process anything here
	 * yet.
	 */
	(void)ts;
	(void)stamp;
}

static void lnt_ti1_update_pen_state(struct lnt_ti1 *ts,
				     int x, int y, int press,
				     int id, int down)
{
	if (lnt_ti1_mtouch && id < ts->max_fingers) {
		/*
		 * Report all events in multi touch mode.
		 */
		input_mt_slot(ts->idev, id);
		input_mt_report_slot_state(ts->idev, MT_TOOL_FINGER, !!down);
		if (down) {
			input_report_abs(ts->idev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->idev, ABS_MT_POSITION_Y, y);
			input_report_abs(ts->idev, ABS_MT_PRESSURE, press);
		}
		input_mt_report_pointer_emulation(ts->idev, true);
		if (lnt_ti1_diag > 0)
			dev_info(ts->dev,
				 "at(%d, %d) id(%d) down(%d) MT reported\n",
				 x, y, id, down);
	} else if (!lnt_ti1_mtouch && !id) {
		/*
		 * Report events for the 0th finger exclusively in the
		 * single touch configuration, to avoid mouse cursor
		 * warps when multiple fingers are touching the sensor.
		 */
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
		if (lnt_ti1_diag > 0)
			dev_info(ts->dev,
				 "at(%d, %d) id(%d) down(%d) reported\n",
				 x, y, id, down);
	} else {
		if (lnt_ti1_diag > 0)
			dev_info(ts->dev,
				 "at(%d, %d) id(%d) down(%d) IGNORED\n",
				 x, y, id, down);
	}
	input_sync(ts->idev);
}

/*
 * The following accessors _consume_ the data they read.  The read
 * accessors advance in the data stream, while picking up the data.
 */

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

static inline uint32_t get_u32_le(uint8_t **p, size_t *l)
{
	uint32_t v;

	v = get_u16_le(p, l);
	v |= get_u16_le(p, l) << 16;
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
	 * Layout of the packet header:
	 * - 8bit, reflection of the command byte which the response
	 *   corresponds to
	 * - 8bit, status (active aka initialized) and heartbeat
	 *   (free running counter, non-zero when active)
	 */
	if (*dlen < 2)
		return false;
	b = get_u8(data, dlen);
	if (b != want_cmd)
		return false;

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
	 * Layout of a touch sample:
	 *   - 8bit, "status"
	 *   - 8bit, "touches"
	 *   - 16bit LE, X coord
	 *   - 16bit LE, Y coord
	 *
	 * It's important that first all input data gets consumed,
	 * before subsequent checks and interpretation might bail out
	 * and return to the caller.
	 */
	if (*dlen < 6)
		return false;
	raw_status = get_u8(data, dlen);
	raw_touches = get_u8(data, dlen);
	raw_x = get_u16_le(data, dlen);
	raw_y = get_u16_le(data, dlen);
	if (!(raw_status & SAMPLE_STATUS_IS_VALID))
		return false;
	if (!SAMPLE_TOUCH_GET_COUNT(raw_touches))
		return true;
	pressure = ts->max_p;
	finger = SAMPLE_TOUCH_GET_NUMBER(raw_touches);
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
 * This routine interprets the information in a buffer which holds
 * adjacent copies of the "first sample" and "next samples" responses.
 */
static bool lnt_ti1_parse_samples(struct lnt_ti1 *ts,
				  uint8_t *data, size_t dlen)
{
	size_t sample_count;
#define LNT_TI1_CMD_GET_SAMPLE_9_MAX_SAMPLES	9

	/*
	 * Layout of the "first sample" response:
	 * - response header
	 * - 8bit, number of available samples
	 * - one sample
	 * - 8bit, CRC over all preceeding bytes
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
		ts->event_count++;
	} else {
		lnt_ti1_discard_sample(ts, &data, &dlen);
	}
	(void)get_u8(&data, &dlen);	/* skip the CRC */

	/*
	 * It's important to _not_ exit here if the "first sample"
	 * packet suggests that no other data is available.  Instead the
	 * "next samples" packet should always get processed for the
	 * following reasons:
	 *
	 * - The controller might have buffered more samples than this
	 *   first packet can carry.  Skipping samples that were
	 *   available when the packet was created yet could not fit
	 *   into this packet would be a bad idea.  (Note that it's
	 *   uncertain whether the first packet's count is the number of
	 *   samples available in the controller at the time of creation
	 *   of the packet, or the number of samples which the packet
	 *   will carry.)
	 * - The "next samples" response is more recent than the "first
	 *   sample" response.  This means that the number of available
	 *   samples might have increased during transmission of these
	 *   two responses.
	 */

	/*
	 * Layout of the "next samples" response:
	 * - response header
	 * - 8bit, number of available samples
	 * - up to nine samples
	 * - 8bit, CRC over all preceeding bytes
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
		ts->event_count++;
	}
	(void)get_u8(&data, &dlen);	/* skip the CRC */

	return true;
}

static int lnt_ti1_get_samples(struct lnt_ti1 *ts)
{
	uint8_t buff[96];
	size_t buffoff, chunklen;
	int rc;

	/*
	 * Acquire both the "first sample" and "next samples" responses.
	 */
	buffoff = 0;

	chunklen = 10;
	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_SAMPLE_0,
			     buff + buffoff, chunklen);
	if (rc)
		return rc;
	buffoff += chunklen;

	chunklen = 58;
	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_SAMPLE_9,
			     buff + buffoff, chunklen);
	if (rc)
		return rc;
	buffoff += chunklen;

	/*
	 * Interpret the response which consists of the two chunks.
	 */
	lnt_ti1_parse_samples(ts, buff, buffoff);

	return 0;
}

static int lnt_ti1_check_cmd(struct lnt_ti1 *ts,
			     uint8_t *data, size_t size,
			     enum lnt_ti1_cmd_t want_cmd,
			     bool *all_ones, bool *all_zero)
{
	bool same;
	uint8_t *p;

	(void)ts;

	if (!data || !size)
		return -1;

	if (want_cmd && data[0] != want_cmd)
		return -2;

	if (all_ones) {
		same = true;
		for (p = &data[1]; p < data + size; p++) {
			if (*p != 0xff)
				same = false;
		}
		*all_ones = same;
	}
	if (all_zero) {
		same = true;
		for (p = &data[1]; p < data + size; p++) {
			if (*p != 0x00)
				same = false;
		}
		*all_zero = same;
	}

	return 0;
}

static int lnt_ti1_read_id_data(struct lnt_ti1 *ts)
{
	u8 iddata[17];
	int rc;
	uint8_t *p;
	size_t l;
	bool all_ones, all_zero;
	uint8_t bval;
	uint16_t hval;
	uint32_t wval;
	uint16_t vendor, product, serial;
	uint32_t swdata, swmaj, swmin, swrev;
	uint32_t hwdata, hwsap, hwrev;

	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_VERSION, iddata, sizeof(iddata));
	if (rc) {
		dev_err(ts->dev, "ID data read error %d\n", rc);
		return rc;
	}

	/*
	 * (Silently) ignore response data that is unexpected or
	 * suspicious.  The response needs to correspond to the request.
	 * Data bytes of all zeroes or all ones are improbable.
	 */
	if (lnt_ti1_check_cmd(ts, iddata, sizeof(iddata),
			      LNT_TI1_CMD_GET_VERSION, &all_ones, &all_zero))
		return 0;
	if (all_ones || all_zero)
		return 0;

	/*
	 * Layout of the ID/version information:
	 *
	 * - 8bit, reflection of the command byte
	 * - 8bit, status and heartbeat
	 * - 16bit LE(?), vendor ID
	 * - 16bit LE(?), product ID
	 * - 16bit LE(?), serial
	 * - 32bit 1+1+2(?), SW version (version, release, SVN)
	 * - 32bit 3+1(?), HW version (SAP numberr, revision)
	 * - 8bit, CRC over all preceeding bytes
	 *
	 * Note:  The above terse description is all the information
	 * available from the (still preliminary) specification.  None
	 * of the available adapter or firmware versions have yet
	 * supported this command or provided any information.  So no
	 * useful evaluation can be done, instead printing the
	 * information is all we can do ...
	 */
	p = &iddata[0];
	l = sizeof(iddata);
	bval = get_u8(&p, &l);		/* command byte, tested above */
	bval = get_u8(&p, &l);		/* status/hearbeat, ignored */
	hval = get_u16_le(&p, &l); vendor = hval;
	hval = get_u16_le(&p, &l); product = hval;
	hval = get_u16_le(&p, &l); serial = hval;
	wval = get_u32_le(&p, &l); swdata = wval;
	wval = get_u32_le(&p, &l); hwdata = wval;
	bval = get_u8(&p, &l);		/* CRC, tested upon packet reception */

	dev_info(ts->dev,
		 "identification: vendor 0x%04x, product 0x%04x, serial 0x%04x\n",
		 vendor, product, serial);
	swmaj = (swdata >>  0) & 0xff;
	swmin = (swdata >>  8) & 0xff;
	swrev = (swdata >> 16) & 0xffff;
	hwsap = (hwdata >>  0) & 0xffffff;
	hwrev = (hwdata >> 24) & 0xff;
	dev_info(ts->dev,
		 "version information: software %d.%d.%d, hardware %d.%d\n",
		 swmaj, swmin, swrev, hwsap, hwrev);

	return 0;
}

static int lnt_ti1_read_settings(struct lnt_ti1 *ts)
{
	u8 settings[16];
	int rc;
	uint8_t *p;
	size_t l;
	bool all_ones, all_zero;
	uint8_t bval;
	uint16_t hval;

	rc = lnt_ti1_command(ts, LNT_TI1_CMD_GET_SETTINGS, settings, sizeof(settings));
	if (rc) {
		dev_err(ts->dev, "settings read error %d\n", rc);
		return rc;
	}

	/*
	 * (Silently) ignore response data that is unexpected or
	 * suspicious.  The response needs to correspond to the request.
	 * Data bytes of all zeroes or all ones are improbable.
	 */
	if (lnt_ti1_check_cmd(ts, settings, sizeof(settings),
			      LNT_TI1_CMD_GET_SETTINGS, &all_ones, &all_zero))
		return 0;
	if (all_ones || all_zero)
		return 0;

	/*
	 * Layout of the capabilities/settings query:
	 * - 8bit, reflection of the command byte
	 * - 8bit, status and heartbeat
	 * - 16bit LE, resolution (assumption: X)
	 * - 16bit LE, resolution (assumption: Y)
	 * - 16bit LE, offset (assumption: X)
	 * - 16bit LE, offset (assumption: Y)
	 * - 8bit, max finger count
	 * - 8bit, sensitivity
	 * - 8bit, debounce
	 * - 8bit, palm suppression
	 * - 8bit, swipe into touch from outside (?)
	 * - 8bit, CRC over all preceeding bytes
	 */
	p = &settings[0];
	l = sizeof(settings);
	bval = get_u8(&p, &l);		/* command byte, tested above */
	bval = get_u8(&p, &l);		/* status/hearbeat, ignored */
	hval = get_u16_le(&p, &l); ts->max_x = hval;
	hval = get_u16_le(&p, &l); ts->max_y = hval;
	hval = get_u16_le(&p, &l);	/* offset, ignored */
	hval = get_u16_le(&p, &l);	/* offset, ignored */
	bval = get_u8(&p, &l); ts->max_fingers = bval;
	bval = get_u8(&p, &l);		/* sensitivity, ignored */
	bval = get_u8(&p, &l);		/* debounce, ignored */
	bval = get_u8(&p, &l);		/* palm suppression, ignored */
	bval = get_u8(&p, &l);		/* swipe into, ignored */
	bval = get_u8(&p, &l);		/* CRC, tested upon packet reception */
	ts->max_fingers = min_t(int, MAX_SUPPORTED_POINTS, ts->max_fingers);

	if (lnt_ti1_diag > 0) {
		dev_info(ts->dev,
			 "capa/settings: max coord %d/%d, max fingers %d\n",
			 ts->max_x, ts->max_y, ts->max_fingers);
	}

	return 0;
}

static irqreturn_t lnt_ti1_irq_thread(int irq, void *args)
{
	struct lnt_ti1 *ts;
	int rc;
	unsigned long flags;
	int last_event_count;

	ts = args;
	if (lnt_ti1_diag > 0)
		dev_info(ts->dev, "IRQ thread ...\n");

	/*
	 * Prevent the IRQ handler from fetching sample data in those
	 * periods of time where initialization has not yet completed
	 * or shutdown is pending.  Unfortunately it's not possible to
	 * register an ISR and still keep it disabled.  Even immediate
	 * disabling after registration leaves a window for unwanted
	 * activity, when at the time of registration IRQ events are
	 * already pending.
	 */
	if (!ts->is_active) {
		dev_info(ts->dev, "not (yet) active, IRQ ignored\n");
		return IRQ_HANDLED;
	}

	/*
	 * Keep fetching and processing touch data in greedy ways.  This
	 * reduces the number of ISR invocations to the necessary amount.
	 *
	 * Errors are not fatal here, neither subsequent errors after
	 * previous successful interpretation, nor errors upon initial
	 * communication.  Let's cope with ISR invocations / execution
	 * that last longer than the availability of data, as well as
	 * erroneous ISR invocation in the absence of data.  Errors just
	 * terminate the loop that tries to keep processing data as fast
	 * and as greedy as it can.
	 *
	 * Note that observation from test setups strongly suggests to
	 * *NOT* implement a more naive approach here.  Especially
	 * testing the GPIO pin's level has proven to be a BadIdea(TM).
	 * Because of the firmware's re-generating an edge during low
	 * levels in the presence of even more available data, the level
	 * of the CHG signal is not stable, and even need not reflect
	 * the ISR trigger condition upon entry into this routine.  The
	 * following two simpler implementations had to get dropped:
	 * - while (!chg) { ... } -- The falling edge has triggered the
	 *   ISR, the GPIO level upon entry still was HIGH, so nothing
	 *   was read.  Not a single touch event was fetched from the
	 *   touch controller.
	 * - do { ... } while (!chg); -- The ISR got triggered, some
	 *   samples were fetched, yet multiple touches in quick
	 *   succession were missed.
	 *
	 * As a result of the above observation, this more complex
	 * approach was implemented.  It reads at least one set of
	 * samples, and only stops asking for more touch data upon
	 * communication errors, or in the absence of a single parsable
	 * touch event after successful communication.
	 */
	if (lnt_ti1_diag > 0)
		dev_info(ts->dev, "fetching samples ...\n");
	do {
		last_event_count = ts->event_count;
		rc = lnt_ti1_get_samples(ts);
		if (rc)
			break;
		if (ts->event_count == last_event_count)
			break;
	} while (1);
	if (lnt_ti1_diag > 0)
		dev_info(ts->dev, "done fetching samples\n");

	/*
	 * Re-arm the penup timer if the pen is down.  To have the
	 * timeout lift the pen in case the touch controller won't
	 * provide an "up" sample, or stops communicating to us.
	 *
	 * For the multi touch case, determination of "is any of the
	 * fingers down?" is rather complex and is not done here.
	 * Instead it's just assumed that any active finger will keep
	 * sending touch events, and that the absence of touch events
	 * within a certain period of time is an implicit release event
	 * for any finger that might be down.  Let the MT backend do
	 * the book keeping that we don't care enough to do here.
	 */
	if (ts->is_pendown || lnt_ti1_mtouch) {
		spin_lock_irqsave(&ts->lock, flags);
		mod_timer(&ts->penup_timer,
			  jiffies + msecs_to_jiffies(LNT_TI1_PENUP_TIME_MS));
		spin_unlock_irqrestore(&ts->lock, flags);
	}

	return IRQ_HANDLED;
}

/*
 * A period of time without touch samples has passed while the pen is down.
 */
static void lnt_ti1_penup_timer(unsigned long data)
{
	struct lnt_ti1 *ts = (void *)data;
	unsigned long flags;
	int finger;

	if (lnt_ti1_diag > 0)
		dev_info(ts->dev, "penup timer\n");

	spin_lock_irqsave(&ts->lock, flags);
	finger = lnt_ti1_mtouch ? ts->max_fingers : 1;
	while (finger--)
		lnt_ti1_update_pen_state(ts, 0, 0, 0, finger, 0);
	spin_unlock_irqrestore(&ts->lock, flags);
}

static void lnt_ti1_start_scan(struct lnt_ti1 *ts)
{
	(void)lnt_ti1_command(ts, LNT_TI1_CMD_MODE_TOUCH, NULL, 0);
}

static void lnt_ti1_stop_scan(struct lnt_ti1 *ts)
{
	(void)lnt_ti1_command(ts, LNT_TI1_CMD_MODE_CONFIG, NULL, 0);
}

static void __lnt_ti1_disable(struct lnt_ti1 *ts)
{
	lnt_ti1_stop_scan(ts);

	disable_irq(ts->irq);
	if (del_timer_sync(&ts->penup_timer))
		lnt_ti1_penup_timer((unsigned long)ts);
}

static void __lnt_ti1_enable(struct lnt_ti1 *ts)
{
	lnt_ti1_start_scan(ts);
	enable_irq(ts->irq);
}

static int lnt_ti1_open(struct input_dev *input)
{
	struct lnt_ti1 *ts = input_get_drvdata(input);

	/* protected by idev->mutex */
	if (!ts->is_suspended)
		__lnt_ti1_enable(ts);
	return 0;
}

static void lnt_ti1_close(struct input_dev *input)
{
	struct lnt_ti1 *ts = input_get_drvdata(input);

	/* protected by idev->mutex */
	if (!ts->is_suspended)
		__lnt_ti1_disable(ts);
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int lnt_ti1_suspend(struct device *dev)
{
	struct lnt_ti1 *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->idev->mutex);
	if (!ts->is_suspended && ts->idev->users)
		__lnt_ti1_disable(ts);
	ts->is_suspended = true;
	mutex_unlock(&ts->idev->mutex);
	return 0;
}

static int lnt_ti1_resume(struct device *dev)
{
	struct lnt_ti1 *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->idev->mutex);
	if (ts->is_suspended && ts->idev->users)
		__lnt_ti1_enable(ts);
	ts->is_suspended = false;
	mutex_unlock(&ts->idev->mutex);
	return 0;
}
#endif

SIMPLE_DEV_PM_OPS(lnt_ti1_pm_ops, lnt_ti1_suspend, lnt_ti1_resume);
EXPORT_SYMBOL(lnt_ti1_pm_ops);

struct lnt_ti1 *lnt_ti1_probe(struct device *dev,
			      const struct lnt_ti1_bus_ops *busops)
{
	struct lnt_ti1 *ts;
	struct input_dev *input_dev;
	struct device_node *np;
	unsigned int gpio, irq;
	int rc;

	/*
	 * Allocate the touchscreen device information.
	 */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		rc = -ENOMEM;
		goto err_free_mem;
	}
	ts->busops = busops;
	ts->dev = dev;
	input_dev = input_allocate_device();
	if (!input_dev) {
		rc = -ENOMEM;
		goto err_free_mem;
	}
	ts->idev = input_dev;

	/*
	 * Enforce config mode here as the first action, because current
	 * implementations of the controller start in touch mode, and we
	 * might find the controller in any arbitrary mode when this
	 * driver gets loaded.  Communication errors are not fatal here.
	 */
	lnt_ti1_stop_scan(ts);

	/*
	 * Talk to the peer, test the communication in bypassing.
	 * Future firmware implementations might require adjustment in
	 * subsequent communication, current implementations don't allow
	 * identification of the controller or the firmware running on
	 * it.  Ignore communication errors here for robustness.
	 *
	 * XXX TODO  Ignoring communication errors here does not really
	 * improve robustness, especially after recent spec and firmware
	 * upgrades trim down the protocol to "touch mode only".  It is
	 * very appropriate to abort probe() here if identification
	 * fails.
	 */
	lnt_ti1_read_id_data(ts);

	/*
	 * Preset some properties from empiric data that was gathered
	 * during development.  Fetch the controller's capabilities and
	 * have these override the pre-set values if better information
	 * is available.  Ignore communication errors here for
	 * compatibility with firmware versions that don't provide
	 * capabilities information.
	 */
	ts->fudge_x	= 4;
	ts->fudge_y	= 8;
	ts->fudge_p	= 2;
	ts->max_x	= MAX_COORDINATE_VALUE;
	ts->max_y	= MAX_COORDINATE_VALUE;
	ts->max_p	= MAX_COORDINATE_VALUE;
	ts->max_fingers	= MAX_SUPPORTED_POINTS;
	lnt_ti1_read_settings(ts);

	/*
	 * Arrange for background work and synchronization.
	 */
	spin_lock_init(&ts->lock);
	setup_timer(&ts->penup_timer, lnt_ti1_penup_timer, (unsigned long)ts);

	/*
	 * Configure the input device.
	 */
	snprintf(ts->phys, sizeof(ts->phys), "%s/input-ts", dev_name(dev));
	input_dev->name = "lnt-ti1 touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = busops->bustype;
	input_dev->id.vendor = busops->vendor;
	input_dev->dev.parent = dev;
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_X, 0, ts->max_x, ts->fudge_x, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ts->max_y, ts->fudge_y, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, ts->max_p, ts->fudge_p, 0);
	if (lnt_ti1_mtouch) {
		input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
				     ts->max_x, ts->fudge_x, 0);
		input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
				     ts->max_y, ts->fudge_y, 0);
		input_mt_init_slots(input_dev, ts->max_fingers, 0);
	}
	input_dev->open = lnt_ti1_open;
	input_dev->close = lnt_ti1_close;
	input_set_drvdata(input_dev, ts);

	np = dev->of_node;
	gpio = of_get_gpio(np, 0);
	if (!gpio_is_valid(gpio)) {
		dev_err(ts->dev, "invalid GPIO %d, node %p\n", gpio, np);
		/*
		 * FALLTHROUGH, have gpio_request() provide an
		 * appropriate error code, but still provide the
		 * preceeding cause of an invalid GPIO spec
		 */
	}
	rc = gpio_request(gpio, "touch-pen");
	if (rc) {
		dev_err(ts->dev, "GPIO request failed, rc %d\n", rc);
		goto err_free_gpio_irq;
	}
	ts->gpio = gpio;
	rc = gpio_direction_input(ts->gpio);
	if (rc) {
		dev_err(ts->dev, "GPIO request failed, rc %d\n", rc);
		goto err_free_gpio_irq;
	}
	/*
	 * The trigger to fetch sample data can get specified either by
	 * an 'interrupts' spec, or get derived from the GPIO pin.  We
	 * prefer the 'interrupts' spec here, and fall back to the GPIO
	 * as an IRQ controller then.  This allows to use GPIO pins
	 * where the GPIO chip is not an IRQ controller.
	 *
	 * Using an interrupt thread (deferred interrupt handler)
	 * appears appropriate for such a low speed device.  Given the
	 * documented constraints for commands, potentially slow
	 * physical communication channels, and especially the long and
	 * arbitrary delays in the communication, deferred ISR execution
	 * actually becomes a necessity.
	 *
	 * Triggering on levels is not a good idea.  The controller
	 * might be in an arbitrary state, and the level may be low at
	 * the time of initialization.  Either because touch data
	 * already is available, or because the firmware has not yet
	 * asserted the high level.  Both conditions have been observed
	 * in the field.
	 */
	irq = irq_of_parse_and_map(np, 0);
	if (irq < 0 || irq == IRQ_NONE)
		irq = gpio_to_irq(ts->gpio);
	rc = request_threaded_irq(irq, NULL, lnt_ti1_irq_thread,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				  "touch-pen", ts);
	if (rc) {
		dev_err(ts->dev, "IRQ registration failed, rc %d\n", rc);
		goto err_free_gpio_irq;
	}
	ts->irq = irq;
	dev_dbg(ts->dev, "GPIO %d, IRQ %d\n", gpio, irq);
	__lnt_ti1_disable(ts);

	/*
	 * Register the input device, now that everything was setup.
	 * Allow the ISR to fetch sample data, after the required
	 * communication of ID data, mode settings, etc has completed.
	 */
	rc = input_register_device(ts->idev);
	if (rc) {
		dev_err(dev, "input device registration failed, rc %d\n", rc);
		goto err_free_gpio_irq;
	}
	ts->is_active = true;

	/*
	 * Have the pen IRQ wake the machine from sleep.
	 */
	irq_set_irq_wake(ts->irq, 1);
	return ts;

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
	return ERR_PTR(rc);
}
EXPORT_SYMBOL(lnt_ti1_probe);

void lnt_ti1_remove(struct lnt_ti1 *ts)
{
	ts->is_active = false;
	/*
	 * No synchronization or "draining" of IRQ events is needed
	 * here.  The driver only can get removed after the last user
	 * has closed the device.  Which means that interrupts are
	 * disabled here already.
	 */
	free_irq(ts->irq, ts);
	gpio_free(ts->gpio);
	input_unregister_device(ts->idev);
	kfree(ts);
}
EXPORT_SYMBOL(lnt_ti1_remove);

MODULE_AUTHOR("Gerhard Sittig <gsi@denx.de>");
MODULE_DESCRIPTION("LNT Automation Touchinterface 1 driver");
MODULE_LICENSE("GPL");	/* Licensed under the GPL-2 or later */
