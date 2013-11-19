/*
 * Driver for 93xx46 EEPROMs
 *
 * (C) 2011 DENX Software Engineering, Anatolij Gustschin <agust@denx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/eeprom_93xx46.h>

#define OP_START	0x4
#define OP_WRITE	(OP_START | 0x1)
#define OP_READ		(OP_START | 0x2)
#define ADDR_EWDS	0x00
#define ADDR_ERAL	0x20
#define ADDR_EWEN	0x30

struct eeprom_93xx46_dev {
	struct spi_device *spi;
	struct eeprom_93xx46_platform_data *pdata;
	struct bin_attribute bin;
	struct mutex lock;
	int addrlen;
};

static ssize_t
eeprom_93xx46_bin_read(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{
	struct eeprom_93xx46_dev *edev;
	struct device *dev;
	struct spi_message m;
	struct spi_transfer t[2];
	int bits, ret;
	u16 cmd_addr;
	u8 cmd_bytes[2];

	dev = container_of(kobj, struct device, kobj);
	edev = dev_get_drvdata(dev);

	if (unlikely(off >= edev->bin.size))
		return 0;
	if ((off + count) > edev->bin.size)
		count = edev->bin.size - off;
	if (unlikely(!count))
		return count;

	cmd_addr = OP_READ << edev->addrlen;

	if (edev->addrlen == 7) {
		cmd_addr |= off & 0x7f;
		bits = 10;
	} else {
		cmd_addr |= off & 0x3f;
		bits = 9;
	}

	/*
	 * XXX TODO reconsider this bits = 9/10 approach
	 *
	 * SPI transfers with bits per word that are not
	 * multiples of eight?  I don't think so
	 *
	 * not only is this not supported by all SPI controllers
	 * (may only work with bitbanged interfaces), in addition
	 * the EEPROM does not really need this -- all the slave
	 * wants is an explicit start bit (a leading bit of high
	 * level1) after CS has become active
	 *
	 * the existing code already did right-adjust the command
	 * and the address, the command includes the start bit,
	 * so everything is already in place ...
	 */
	bits = 8;	/* hard override */

	dev_dbg(&edev->spi->dev, "read cmd 0x%x, %d Hz\n",
		cmd_addr, edev->spi->max_speed_hz);

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	cmd_bytes[0] = (cmd_addr >> 8) & 0xff;
	cmd_bytes[1] = (cmd_addr >> 0) & 0xff;
	t[0].tx_buf = &cmd_bytes[0];
	t[0].len = 2;
	// t[0].bits_per_word = bits;
	t[0].cs_change = 0;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = count;
	// t[1].bits_per_word = 8;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&edev->lock);

	if (edev->pdata->prepare)
		edev->pdata->prepare(edev);

  dev_err(&edev->spi->dev, "%s() reading %d bytes at 0x%x, cmd 0x%x\n", __func__, count, (int)off, cmd_addr);
	ret = spi_sync(edev->spi, &m);
  dev_err(&edev->spi->dev, "%s() read rc %d\n", __func__, ret);
	/* have to wait at least Tcsl ns */
	ndelay(250);
	if (ret) {
		dev_err(&edev->spi->dev, "read %zu bytes at %d: err. %d\n",
			count, (int)off, ret);
	}

	if (edev->pdata->finish)
		edev->pdata->finish(edev);

	mutex_unlock(&edev->lock);
	return ret ? : count;
}

static int eeprom_93xx46_ew(struct eeprom_93xx46_dev *edev, int is_on)
{
	struct spi_message m;
	struct spi_transfer t;
	int bits, ret;
	u16 cmd_addr;

	cmd_addr = OP_START << edev->addrlen;
	if (edev->addrlen == 7) {
		cmd_addr |= (is_on ? ADDR_EWEN : ADDR_EWDS) << 1;
		bits = 10;
	} else {
		cmd_addr |= (is_on ? ADDR_EWEN : ADDR_EWDS);
		bits = 9;
	}

	dev_dbg(&edev->spi->dev, "ew cmd 0x%04x\n", cmd_addr);

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));

	t.tx_buf = &cmd_addr;
	t.len = 2;
	t.bits_per_word = bits;
	spi_message_add_tail(&t, &m);

	mutex_lock(&edev->lock);

	if (edev->pdata->prepare)
		edev->pdata->prepare(edev);

	ret = spi_sync(edev->spi, &m);
	/* have to wait at least Tcsl ns */
	ndelay(250);
	if (ret)
		dev_err(&edev->spi->dev, "erase/write %sable error %d\n",
			is_on ? "en" : "dis", ret);

	if (edev->pdata->finish)
		edev->pdata->finish(edev);

	mutex_unlock(&edev->lock);
	return ret;
}

static ssize_t
eeprom_93xx46_write_word(struct eeprom_93xx46_dev *edev,
			 const char *buf, unsigned off)
{
	struct spi_message m;
	struct spi_transfer t[2];
	int bits, data_len, ret;
	u16 cmd_addr;

	cmd_addr = OP_WRITE << edev->addrlen;

	if (edev->addrlen == 7) {
		cmd_addr |= off & 0x7f;
		bits = 10;
		data_len = 1;
	} else {
		cmd_addr |= off & 0x3f;
		bits = 9;
		data_len = 2;
	}

	dev_dbg(&edev->spi->dev, "write cmd 0x%x\n", cmd_addr);

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	t[0].tx_buf = (char *)&cmd_addr;
	t[0].len = 2;
	t[0].bits_per_word = bits;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	t[1].len = data_len;
	t[1].bits_per_word = 8;
	spi_message_add_tail(&t[1], &m);

	ret = spi_sync(edev->spi, &m);
	/* have to wait program cycle time Twc ms */
	mdelay(6);
	return ret;
}

static ssize_t
eeprom_93xx46_bin_write(struct file *filp, struct kobject *kobj,
			struct bin_attribute *bin_attr,
			char *buf, loff_t off, size_t count)
{
	struct eeprom_93xx46_dev *edev;
	struct device *dev;
	int i, ret, step = 1;

	dev = container_of(kobj, struct device, kobj);
	edev = dev_get_drvdata(dev);

	if (unlikely(off >= edev->bin.size))
		return 0;
	if ((off + count) > edev->bin.size)
		count = edev->bin.size - off;
	if (unlikely(!count))
		return count;

	/* only write even number of bytes on 16-bit devices */
	if (edev->addrlen == 6) {
		step = 2;
		count &= ~1;
	}

	/* erase/write enable */
	ret = eeprom_93xx46_ew(edev, 1);
	if (ret)
		return ret;

	mutex_lock(&edev->lock);

	if (edev->pdata->prepare)
		edev->pdata->prepare(edev);

	for (i = 0; i < count; i += step) {
		ret = eeprom_93xx46_write_word(edev, &buf[i], off + i);
		if (ret) {
			dev_err(&edev->spi->dev, "write failed at %d: %d\n",
				(int)off + i, ret);
			break;
		}
	}

	if (edev->pdata->finish)
		edev->pdata->finish(edev);

	mutex_unlock(&edev->lock);

	/* erase/write disable */
	eeprom_93xx46_ew(edev, 0);
	return ret ? : count;
}

static int eeprom_93xx46_eral(struct eeprom_93xx46_dev *edev)
{
	struct eeprom_93xx46_platform_data *pd = edev->pdata;
	struct spi_message m;
	struct spi_transfer t;
	int bits, ret;
	u16 cmd_addr;

	cmd_addr = OP_START << edev->addrlen;
	if (edev->addrlen == 7) {
		cmd_addr |= ADDR_ERAL << 1;
		bits = 10;
	} else {
		cmd_addr |= ADDR_ERAL;
		bits = 9;
	}

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));

	t.tx_buf = &cmd_addr;
	t.len = 2;
	t.bits_per_word = bits;
	spi_message_add_tail(&t, &m);

	mutex_lock(&edev->lock);

	if (edev->pdata->prepare)
		edev->pdata->prepare(edev);

	ret = spi_sync(edev->spi, &m);
	if (ret)
		dev_err(&edev->spi->dev, "erase error %d\n", ret);
	/* have to wait erase cycle time Tec ms */
	mdelay(6);

	if (pd->finish)
		pd->finish(edev);

	mutex_unlock(&edev->lock);
	return ret;
}

static ssize_t eeprom_93xx46_store_erase(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct eeprom_93xx46_dev *edev = dev_get_drvdata(dev);
	int erase = 0, ret;

	sscanf(buf, "%d", &erase);
	if (erase) {
		ret = eeprom_93xx46_ew(edev, 1);
		if (ret)
			return ret;
		ret = eeprom_93xx46_eral(edev);
		if (ret)
			return ret;
		ret = eeprom_93xx46_ew(edev, 0);
		if (ret)
			return ret;
	}
	return count;
}
static DEVICE_ATTR(erase, S_IWUSR, NULL, eeprom_93xx46_store_erase);

static struct eeprom_93xx46_platform_data of_pd;

static int eeprom_93xx46_probe(struct spi_device *spi)
{
	struct eeprom_93xx46_platform_data *pd;
	struct device_node *np;
	u32 val;
	struct eeprom_93xx46_dev *edev;
	int err;

	pd = spi->dev.platform_data;
	if (!pd) do {
		dev_info(&spi->dev, "no platform data, trying OF\n");
		np = spi->dev.of_node;
		if (!np)
			break;
  dev_info(&spi->dev, "clear\n");
		memset(&of_pd, 0, sizeof(of_pd));
		if (of_property_read_u32(np, "address-width", &val))
			break;
  dev_info(&spi->dev, "width %d\n", val);
		if (val == 8)
			of_pd.flags |= EE_ADDR8;
		else if (val == 16)
			of_pd.flags |= EE_ADDR16;
		else
			break;
		if (of_find_property(np, "read-only", NULL))
			of_pd.flags |= EE_READONLY;
  dev_info(&spi->dev, "ro %d\n", (of_pd.flags & EE_READONLY) ? 1 : 0);
		pd = &of_pd;
	} while (0);
	if (!pd) {
		dev_err(&spi->dev, "neither platform data nor OF specs\n");
		return -ENODEV;
	}

	edev = kzalloc(sizeof(*edev), GFP_KERNEL);
	if (!edev)
		return -ENOMEM;

	if (pd->flags & EE_ADDR8)
		edev->addrlen = 7;
	else if (pd->flags & EE_ADDR16)
		edev->addrlen = 6;
	else {
		dev_err(&spi->dev, "unspecified address type\n");
		err = -EINVAL;
		goto fail;
	}

	mutex_init(&edev->lock);

	edev->spi = spi_dev_get(spi);
	edev->pdata = pd;

	sysfs_bin_attr_init(&edev->bin);
	edev->bin.attr.name = "eeprom";
	edev->bin.attr.mode = S_IRUSR;
	edev->bin.read = eeprom_93xx46_bin_read;
	edev->bin.size = 128;
	if (!(pd->flags & EE_READONLY)) {
		edev->bin.write = eeprom_93xx46_bin_write;
		edev->bin.attr.mode |= S_IWUSR;
	}

	err = sysfs_create_bin_file(&spi->dev.kobj, &edev->bin);
	if (err)
		goto fail;

	dev_info(&spi->dev, "%d-bit eeprom %s\n",
		(pd->flags & EE_ADDR8) ? 8 : 16,
		(pd->flags & EE_READONLY) ? "(readonly)" : "");

	if (!(pd->flags & EE_READONLY)) {
		if (device_create_file(&spi->dev, &dev_attr_erase))
			dev_err(&spi->dev, "can't create erase interface\n");
	}

	spi_set_drvdata(spi, edev);
	return 0;
fail:
	kfree(edev);
	return err;
}

static int eeprom_93xx46_remove(struct spi_device *spi)
{
	struct eeprom_93xx46_dev *edev = spi_get_drvdata(spi);

	if (!(edev->pdata->flags & EE_READONLY))
		device_remove_file(&spi->dev, &dev_attr_erase);

	sysfs_remove_bin_file(&spi->dev.kobj, &edev->bin);
	spi_set_drvdata(spi, NULL);
	kfree(edev);
	return 0;
}

static struct spi_driver eeprom_93xx46_driver = {
	.driver = {
		.name	= "93xx46",
		.owner	= THIS_MODULE,
	},
	.probe		= eeprom_93xx46_probe,
	.remove		= eeprom_93xx46_remove,
};

module_spi_driver(eeprom_93xx46_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for 93xx46 EEPROMs");
MODULE_AUTHOR("Anatolij Gustschin <agust@denx.de>");
MODULE_ALIAS("spi:93xx46");
