/*
 * Designware GPIO support functions
 *
 * Copyright (C) 2012 Altera
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_data/gpio-dw.h>	/* OF compat string */
#include <linux/platform_device.h>
#include <linux/slab.h>

/* GPIO module register layout, HPS.html page 1259 */
#define DW_GPIO_DATA 		0x00	/* port A data */
#define DW_GPIO_DIR	 	0x04	/* port A direction */
#define DW_GPIO_INT_EN	 	0x30	/* interrupt enable */
#define DW_GPIO_INT_MASK 	0x34	/* interrupt mask */
#define DW_GPIO_INT_TYPE 	0x38	/* interrupt type (edge when set) */
#define DW_GPIO_INT_POL	 	0x3c	/* interrupt polarity */
#define DW_GPIO_INT_STS	 	0x40	/* interrupt status */
/* 0x44 raw interrupt status */
/* 0x48 debounce enable */
#define DW_GPIO_INT_EOI		0x4c	/* port A EOI, write 1 to clear edge interrupts */
#define DW_GPIO_EXT_DATA	0x50	/* external port A */
/* 0x60 sync level */
/* 0x64 ID code, "chip identification" */
/* 0x6c version register, ASCII '201*' for 2.01 */
/* 0x70 config register 2, 4 times 5 bits, port width minus 1, reset value 28/7/7/7 */
/* 0x74 config register 1, read only reflection of IP block configuration */

#define DRV_NAME "dw gpio"

struct dw_gpio_instance {
	struct of_mm_gpio_chip mmchip;
	u32 gpio_state;		/* GPIO state shadow register */
	u32 gpio_dir;		/* GPIO direction shadow register */
	struct irq_domain *irq;	/* GPIO controller IRQ domain */
	spinlock_t gpio_lock;	/* Lock used for synchronization */
};

static int dw_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);

	return (__raw_readl(mm_gc->regs + DW_GPIO_EXT_DATA) >> offset) & 1;
}

static void dw_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct dw_gpio_instance *chip = container_of(mm_gc, struct dw_gpio_instance, mmchip);
	unsigned long flags;
	u32 data_reg;

	spin_lock_irqsave(&chip->gpio_lock, flags);
	data_reg = __raw_readl(mm_gc->regs + DW_GPIO_DATA);
	data_reg &= ~(1 << offset);
	data_reg |= value << offset;
	__raw_writel(data_reg, mm_gc->regs + DW_GPIO_DATA);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static int dw_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct dw_gpio_instance *chip = container_of(mm_gc, struct dw_gpio_instance, mmchip);
	unsigned long flags;
	u32 gpio_ddr;

	spin_lock_irqsave(&chip->gpio_lock, flags);
	/* Set pin as input, assumes software controlled IP */
	gpio_ddr = __raw_readl(mm_gc->regs + DW_GPIO_DIR);
	gpio_ddr &= ~(1 << offset);
	__raw_writel(gpio_ddr, mm_gc->regs + DW_GPIO_DIR);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

static int dw_gpio_direction_output(struct gpio_chip *gc,
		unsigned offset, int value)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct dw_gpio_instance *chip = container_of(mm_gc, struct dw_gpio_instance, mmchip);
	unsigned long flags;
	u32 gpio_ddr;

	dw_gpio_set(gc, offset, value);
	
	spin_lock_irqsave(&chip->gpio_lock, flags);
	/* Set pin as output, assumes software controlled IP */
	gpio_ddr = __raw_readl(mm_gc->regs + DW_GPIO_DIR);
	gpio_ddr |= (1 << offset);
	__raw_writel(gpio_ddr, mm_gc->regs + DW_GPIO_DIR);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
	return 0;
}

static int dw_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct dw_gpio_instance *chip = container_of(mm_gc, struct dw_gpio_instance, mmchip);

	if (chip->irq && offset < mm_gc->gc.ngpio)
		return irq_create_mapping(chip->irq, offset);
	else
		return -ENXIO;
}

static void dw_gpio_irq_cascade(unsigned int irq, struct irq_desc *desc)
{
	struct dw_gpio_instance *gchip = irq_desc_get_handler_data(desc);
	struct irq_chip *ichip = irq_desc_get_chip(desc);
	struct of_mm_gpio_chip *mm = &gchip->mmchip;
	unsigned long mask, bit;

	/* get those enabled interrupts which are active */
	mask = __raw_readl(mm->regs + DW_GPIO_INT_STS);
	mask &= ~__raw_readl(mm->regs + DW_GPIO_INT_MASK);
	mask &= __raw_readl(mm->regs + DW_GPIO_INT_EN);

	/* handle all of them before any ACK */
	for_each_set_bit(bit, &mask, mm->gc.ngpio) {
		generic_handle_irq(irq_linear_revmap(gchip->irq, bit));
	}

	/* ACK all of them (to the GPIO module, only useful for edge INTs) */
	__raw_writel(mask, mm->regs + DW_GPIO_INT_EOI);

	/* optionally EOI in the IRQ controller chip */
	if (ichip->irq_eoi)
		ichip->irq_eoi(&desc->irq_data);
}

static void dw_irq_enable(struct irq_data *d)
{
	struct dw_gpio_instance *chip = irq_data_get_irq_chip_data(d);
	struct of_mm_gpio_chip *mm = &chip->mmchip;
	unsigned long flags;
	u32 mask, reg;

	mask = 1 << irqd_to_hwirq(d);

	spin_lock_irqsave(&chip->gpio_lock, flags);
	reg = __raw_readl(mm->regs + DW_GPIO_INT_EN);
	reg |= mask;
	__raw_writel(reg, mm->regs + DW_GPIO_INT_EN);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static void dw_irq_disable(struct irq_data *d)
{
	struct dw_gpio_instance *chip = irq_data_get_irq_chip_data(d);
	struct of_mm_gpio_chip *mm = &chip->mmchip;
	unsigned long flags;
	u32 mask, reg;

	mask = 1 << irqd_to_hwirq(d);

	spin_lock_irqsave(&chip->gpio_lock, flags);
	reg = __raw_readl(mm->regs + DW_GPIO_INT_EN);
	reg &= ~mask;
	__raw_writel(reg, mm->regs + DW_GPIO_INT_EN);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static void dw_irq_unmask(struct irq_data *d)
{
	struct dw_gpio_instance *chip = irq_data_get_irq_chip_data(d);
	struct of_mm_gpio_chip *mm = &chip->mmchip;
	unsigned long flags;
	u32 mask, reg;

	mask = 1 << irqd_to_hwirq(d);

	spin_lock_irqsave(&chip->gpio_lock, flags);
	reg = __raw_readl(mm->regs + DW_GPIO_INT_MASK);
	reg &= ~mask;
	__raw_writel(reg, mm->regs + DW_GPIO_INT_MASK);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static void dw_irq_mask(struct irq_data *d)
{
	struct dw_gpio_instance *chip = irq_data_get_irq_chip_data(d);
	struct of_mm_gpio_chip *mm = &chip->mmchip;
	unsigned long flags;
	u32 mask, reg;

	mask = 1 << irqd_to_hwirq(d);

	spin_lock_irqsave(&chip->gpio_lock, flags);
	reg = __raw_readl(mm->regs + DW_GPIO_INT_MASK);
	reg |= mask;
	__raw_writel(reg, mm->regs + DW_GPIO_INT_MASK);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static void dw_irq_ack(struct irq_data *d)
{
	struct dw_gpio_instance *chip = irq_data_get_irq_chip_data(d);
	struct of_mm_gpio_chip *mm = &chip->mmchip;
	unsigned long flags;
	u32 mask;

	mask = 1 << irqd_to_hwirq(d);

	spin_lock_irqsave(&chip->gpio_lock, flags);
	__raw_writel(mask, mm->regs + DW_GPIO_INT_EOI);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static int dw_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	struct dw_gpio_instance *chip = irq_data_get_irq_chip_data(d);
	struct of_mm_gpio_chip *mm = &chip->mmchip;
	int ret;
	u32 mask, type_reg, pol_reg, mask_reg;

	ret = 0;

	mask = 1 << irqd_to_hwirq(d);
	type_reg = __raw_readl(mm->regs + DW_GPIO_INT_TYPE);
	pol_reg = __raw_readl(mm->regs + DW_GPIO_INT_POL);
	mask_reg = __raw_readl(mm->regs + DW_GPIO_INT_MASK);

	switch (flow_type) {
	case IRQ_TYPE_EDGE_FALLING:
		type_reg |= mask;
		pol_reg &= ~mask;
		break;
	case IRQ_TYPE_EDGE_RISING:
		type_reg |= mask;
		pol_reg |= mask;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		type_reg &= ~mask;
		pol_reg &= ~mask;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		type_reg &= ~mask;
		pol_reg |= mask;
		break;
	case IRQ_TYPE_EDGE_BOTH:
	default:
		ret = -EINVAL;
		break;
	}

	if (!ret) {
		mask_reg &= ~mask;
		__raw_writel(type_reg, mm->regs + DW_GPIO_INT_TYPE);
		__raw_writel(pol_reg, mm->regs + DW_GPIO_INT_POL);
		__raw_writel(mask_reg, mm->regs + DW_GPIO_INT_MASK);
	}

	return ret;
}

static struct irq_chip dw_irq_chip = {
	.name = "dw-gpio",
	.irq_enable = dw_irq_enable,
	.irq_disable = dw_irq_disable,
	.irq_unmask = dw_irq_unmask,
	.irq_mask = dw_irq_mask,
	.irq_ack = dw_irq_ack,
	.irq_set_type = dw_irq_set_type,
};

static int dw_gpio_irq_map(struct irq_domain *h, unsigned int virq,
			   irq_hw_number_t hw)
{
	irq_set_chip_data(virq, h->host_data);
	irq_set_chip_and_handler(virq, &dw_irq_chip, handle_level_irq);
	return 0;
}

static struct irq_domain_ops dw_gpio_irq_ops = {
	.map = dw_gpio_irq_map,
	.xlate = irq_domain_xlate_twocell,
};

/* 
 * dw_gpio_probe - Probe method for the GPIO device.
 * @np: pointer to device tree node
 *
 * This function probes the GPIO device in the device tree. It initializes the
 * driver data structure. It returns 0, if the driver is bound to the GPIO
 * device, or a negative value if there is an error.
 */
static int dw_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct dw_gpio_instance *chip;
	int ret = 0;
	u32 reg;
	unsigned int hwirq;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		printk(KERN_ERR "%s 2 ERROR allocating memory", __func__);
		return -ENOMEM;
	}

	/* Update GPIO state shadow register with default value */
	if (of_property_read_u32(np, "resetvalue", &reg) == 0)
		chip->gpio_state = reg;

	/* Update GPIO direction shadow register with default value */
	chip->gpio_dir = 0; /* By default, all pins are inputs */

	/* Check device node for device width */
	if (of_property_read_u32(np, "width", &reg) == 0)
		chip->mmchip.gc.ngpio = reg;
	else
		chip->mmchip.gc.ngpio = 32; /* By default assume full GPIO controller */

	spin_lock_init(&chip->gpio_lock);

	chip->mmchip.gc.direction_input = dw_gpio_direction_input;
	chip->mmchip.gc.direction_output = dw_gpio_direction_output;
	chip->mmchip.gc.get = dw_gpio_get;
	chip->mmchip.gc.set = dw_gpio_set;
	chip->mmchip.gc.to_irq = dw_gpio_to_irq;

	/* Call the OF gpio helper to setup and register the GPIO device */
	ret = of_mm_gpiochip_add(np, &chip->mmchip);
	if (ret) {
		dev_err(&pdev->dev, "cannot add chip, error %d\n", ret);
		goto err_chip_add;
	}

	hwirq = irq_of_parse_and_map(np, 0);
	if (hwirq == NO_IRQ)
		goto skip_irq;

	chip->irq = irq_domain_add_linear(np, chip->mmchip.gc.ngpio,
					   &dw_gpio_irq_ops, chip);
	if (!chip->irq)
		goto skip_irq;

	/* ACK and mask all IRQs */
	__raw_writel( 0, chip->mmchip.regs + DW_GPIO_INT_EN);
	__raw_writel(~0, chip->mmchip.regs + DW_GPIO_INT_MASK);
	__raw_writel(~0, chip->mmchip.regs + DW_GPIO_INT_EOI);

	irq_set_handler_data(hwirq, chip);
	irq_set_chained_handler(hwirq, dw_gpio_irq_cascade);

skip_irq:
	platform_set_drvdata(pdev, chip);
	return 0;

err_chip_add:
	kfree(chip);
	return ret;
}

static int dw_gpio_remove(struct platform_device *pdev)
{
	/* todo check this and see that we don't have a memory leak */
	int status;
	
	struct dw_gpio_instance *chip = platform_get_drvdata(pdev);
	status = gpiochip_remove(&chip->mmchip.gc);
	if (status < 0)
		return status;
	
	kfree(chip);
	return -EIO;
}

#ifdef CONFIG_OF
static const struct of_device_id dwgpio_match[] = {
	{.compatible = DW_GPIO_COMPATIBLE,},
	{}
};
MODULE_DEVICE_TABLE(of, dwgpio_match);
#else
#define dwgpio_match NULL
#endif

static struct platform_driver dwgpio_driver = {
	.driver = {
		.name	= "dw_gpio",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(dwgpio_match),
	},
	.probe		= dw_gpio_probe,
	.remove		= dw_gpio_remove,
};

static int __init dwgpio_init(void)
{
	return platform_driver_register(&dwgpio_driver);
}
subsys_initcall(dwgpio_init);

static void __exit dwgpio_exit(void)
{
	platform_driver_unregister(&dwgpio_driver);
}
module_exit(dwgpio_exit);


MODULE_DESCRIPTION("Altera GPIO driver");
MODULE_AUTHOR("Thomas Chou <thomas@wytron.com.tw>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
