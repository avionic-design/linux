/*
 * Copyright (C) 2009-2012 Avionic Design GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/spinlock.h>

#define	GPIO_DDR_BASE(c)	(0x000)
#define	GPIO_DDR(c, x)		(GPIO_DDR_BASE(c) + ((x) << 2))
#define	GPIO_PLR_BASE(c)	(GPIO_DDR_BASE(c) + DIV_ROUND_UP((c)->ngpio, 8))
#define	GPIO_PLR(c, x)		(GPIO_PLR_BASE(c) + ((x) << 2))
#define	GPIO_IER_BASE(c)	(GPIO_PLR_BASE(c) + DIV_ROUND_UP((c)->ngpio, 8))
#define	GPIO_IER(c, x)		(GPIO_IER_BASE(c) + ((x) << 2))
#define	GPIO_ISR_BASE(c)	(GPIO_IER_BASE(c) + DIV_ROUND_UP((c)->ngpio, 8))
#define	GPIO_ISR(c, x)		(GPIO_ISR_BASE(c) + ((x) << 2))
#define	GPIO_PTR_BASE(c)	(GPIO_ISR_BASE(c) + DIV_ROUND_UP((c)->ngpio, 8))
#define	GPIO_PTR(c, x)		(GPIO_PTR_BASE(c) + ((x) << 2))

struct ad64p_gpio {
	struct gpio_chip chip;
	unsigned int num_regs;
	void __iomem *base;
	spinlock_t lock;

	struct irq_domain *domain;

	u32 *irq_level;
	u32 *irq_rise;
	u32 *irq_fall;
	u32 *irq_high;
	u32 *irq_low;
};

static int ad64p_gpio_set_input(struct gpio_chip *chip, unsigned int offset)
{
	struct ad64p_gpio *gpio = container_of(chip, struct ad64p_gpio, chip);
	unsigned int reg = offset / 32;
	unsigned int bit = offset % 32;
	unsigned long flags;
	int ret = 0;
	u32 ddr;

	spin_lock_irqsave(&gpio->lock, flags);

	ddr = readl(gpio->base + GPIO_DDR(chip, reg));
	ddr &= ~BIT(bit);
	writel(ddr, gpio->base + GPIO_DDR(chip, reg));

	ddr = readl(gpio->base + GPIO_DDR(chip, reg));
	if (ddr & BIT(bit))
		ret = -EACCES;

	spin_unlock_irqrestore(&gpio->lock, flags);
	return ret;
}

static int ad64p_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ad64p_gpio *gpio = container_of(chip, struct ad64p_gpio, chip);
	unsigned int reg = offset / 32;
	unsigned int bit = offset % 32;
	unsigned long flags;
	u32 plr;
	u32 ptr;

	spin_lock_irqsave(&gpio->lock, flags);

	plr = readl(gpio->base + GPIO_PLR(chip, reg));
	ptr = readl(gpio->base + GPIO_PTR(chip, reg));

	spin_unlock_irqrestore(&gpio->lock, flags);

	if (ptr & BIT(bit))
		return 2;

	if (plr & BIT(bit))
		return 1;

	return 0;
}

static void ad64p_gpio_set_pin_level(struct gpio_chip *chip, unsigned int reg,
				     unsigned int bit, unsigned int value)
{
	struct ad64p_gpio *gpio = container_of(chip, struct ad64p_gpio, chip);
	u32 plr;

	plr = readl(gpio->base + GPIO_PLR(chip, reg));

	if (value == 0)
		plr &= ~BIT(bit);
	else
		plr |= BIT(bit);

	writel(plr, gpio->base + GPIO_PLR(chip, reg));
}

static int ad64p_gpio_set_output(struct gpio_chip *chip, unsigned int offset,
				 int value)
{
	struct ad64p_gpio *gpio = container_of(chip, struct ad64p_gpio, chip);
	unsigned int reg = offset / 32;
	unsigned int bit = offset % 32;
	unsigned long flags;
	u32 ddr;

	spin_lock_irqsave(&gpio->lock, flags);

	ddr = readl(gpio->base + GPIO_DDR(chip, reg));
	ddr |= BIT(bit);
	writel(ddr, gpio->base + GPIO_DDR(chip, reg));

	ddr = readl(gpio->base + GPIO_DDR(chip, reg));
	if (!(ddr & BIT(bit))) {
		spin_unlock_irqrestore(&gpio->lock, flags);
		return -EACCES;
	}

	ad64p_gpio_set_pin_level(chip, reg, bit, value);
	spin_unlock_irqrestore(&gpio->lock, flags);
	return 0;
}

static void ad64p_gpio_set(struct gpio_chip *chip, unsigned int offset,
			   int value)
{
	struct ad64p_gpio *gpio = container_of(chip, struct ad64p_gpio, chip);
	unsigned long flags;

	spin_lock_irqsave(&gpio->lock, flags);
	ad64p_gpio_set_pin_level(chip, offset / 32, offset % 32, value);
	spin_unlock_irqrestore(&gpio->lock, flags);
}

static void ad64p_irq_ack(struct irq_data *data)
{
	struct ad64p_gpio *gpio = irq_data_get_irq_chip_data(data);
	unsigned int reg = data->hwirq / 32;
	unsigned int bit = data->hwirq % 32;

	writel(BIT(bit), gpio->base + GPIO_ISR(&gpio->chip, reg));
}

static void ad64p_irq_mask(struct irq_data *data)
{
	struct ad64p_gpio *gpio = irq_data_get_irq_chip_data(data);
	unsigned int reg = data->hwirq / 32;
	unsigned int bit = data->hwirq % 32;
	unsigned long flags;
	u32 ier;

	spin_lock_irqsave(&gpio->lock, flags);

	ier = readl(gpio->base + GPIO_IER(&gpio->chip, reg));
	ier &= ~BIT(bit);
	writel(ier, gpio->base + GPIO_IER(&gpio->chip, reg));

	spin_unlock_irqrestore(&gpio->lock, flags);
}

static void ad64p_irq_unmask(struct irq_data *data)
{
	struct ad64p_gpio *gpio = irq_data_get_irq_chip_data(data);
	unsigned int reg = data->hwirq / 32;
	unsigned int bit = data->hwirq % 32;
	unsigned long flags;
	u32 ier;

	spin_lock_irqsave(&gpio->lock, flags);

	ier = readl(gpio->base + GPIO_IER(&gpio->chip, reg));
	ier |= BIT(bit);
	writel(ier, gpio->base + GPIO_IER(&gpio->chip, reg));

	spin_unlock_irqrestore(&gpio->lock, flags);
}

static int ad64p_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct ad64p_gpio *gpio = irq_data_get_irq_chip_data(data);
	unsigned int reg = data->hwirq / 32;
	unsigned int pos = data->hwirq % 32;
	int err;

	err = ad64p_gpio_set_input(&gpio->chip, data->hwirq);
	if (err < 0)
		goto out;

	if (type & IRQ_TYPE_EDGE_RISING)
		gpio->irq_rise[reg] |= BIT(pos);
	else
		gpio->irq_rise[reg] &= ~BIT(pos);

	if (type & IRQ_TYPE_EDGE_FALLING)
		gpio->irq_fall[reg] |= BIT(pos);
	else
		gpio->irq_fall[reg] &= ~BIT(pos);

	if (type & IRQ_TYPE_LEVEL_HIGH)
		gpio->irq_high[reg] |= BIT(pos);
	else
		gpio->irq_high[reg] &= ~BIT(pos);

	if (type & IRQ_TYPE_LEVEL_LOW)
		gpio->irq_low[reg] |= BIT(pos);
	else
		gpio->irq_low[reg] &= ~BIT(pos);

out:
	return err;
}

static int ad64p_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct ad64p_gpio *gpio = container_of(chip, struct ad64p_gpio, chip);
	return irq_create_mapping(gpio->domain, offset);
}

static struct irq_chip ad64p_irq_chip = {
	.name = "gpio-ad64p",
	.irq_ack = ad64p_irq_ack,
	.irq_mask = ad64p_irq_mask,
	.irq_unmask = ad64p_irq_unmask,
	.irq_set_type = ad64p_irq_set_type,
};

static int ad64p_irq_map(struct irq_domain *domain, unsigned int irq,
			 irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &ad64p_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);

#ifdef CONFIG_ARM
	set_irq_flags(irq, IRQF_VALID);
#else
	irq_set_noprobe(irq);
#endif

	return 0;
}

static const struct irq_domain_ops ad64p_irq_domain_ops = {
	.map = ad64p_irq_map,
	.xlate = irq_domain_xlate_twocell,
};

static int ad64p_irq_setup(struct ad64p_gpio *gpio)
{
	size_t size = gpio->num_regs * 5 * sizeof(u32);
	struct gpio_chip *chip = &gpio->chip;
	unsigned int i;

	gpio->irq_level = devm_kzalloc(chip->dev, size, GFP_KERNEL);
	if (!gpio->irq_level)
		return -ENOMEM;

	gpio->irq_rise = gpio->irq_level + gpio->num_regs;
	gpio->irq_fall = gpio->irq_rise + gpio->num_regs;
	gpio->irq_high = gpio->irq_fall + gpio->num_regs;
	gpio->irq_low = gpio->irq_high + gpio->num_regs;

	for (i = 0; i < gpio->num_regs; i++)
		gpio->irq_level[i] = readl(gpio->base + GPIO_PLR(chip, i));

	gpio->domain = irq_domain_add_linear(chip->of_node, chip->ngpio,
					     &ad64p_irq_domain_ops, gpio);
	if (!gpio->domain)
		return -ENOMEM;

	gpio->chip.to_irq = ad64p_gpio_to_irq;
	return 0;
}

static void ad64p_irq_teardown(struct ad64p_gpio *gpio)
{
	unsigned int irq, i;

	for (i = 0; i < gpio->chip.ngpio; i++) {
		irq = irq_find_mapping(gpio->domain, i);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(gpio->domain);
}

static irqreturn_t ad64p_irq(int irq, void *dev_id)
{
	struct ad64p_gpio *gpio = dev_id;
	unsigned long flags;
	unsigned int i;

	for (i = 0; i < gpio->num_regs; i++) {
		unsigned long pending;
		unsigned long changed;
		unsigned long level;
		unsigned long mask;
		unsigned long isr;
		unsigned int bit;

		spin_lock_irqsave(&gpio->lock, flags);

		level = readl(gpio->base + GPIO_PLR(&gpio->chip, i));
		mask = readl(gpio->base + GPIO_IER(&gpio->chip, i));
		isr = readl(gpio->base + GPIO_ISR(&gpio->chip, i));

		spin_unlock_irqrestore(&gpio->lock, flags);

		/* determine pins that changed levels */
		changed = level ^ gpio->irq_level[i];

		/* compute edge-triggered interrupts */
		pending = changed & ((gpio->irq_fall[i] & ~level) |
				     (gpio->irq_rise[i] & level));

		/* add in level-triggered interrupts */
		pending |= (gpio->irq_high[i] & level) |
			   (gpio->irq_low[i] & ~level);

		/* mask out disabled interrupts */
		pending &= isr & mask;

		for_each_set_bit(bit, &pending, 32) {
			unsigned int virq;
			virq = irq_find_mapping(gpio->domain, i * 32 + bit);
			generic_handle_irq(virq);
		}

		gpio->irq_level[i] = level;
	}

	return IRQ_HANDLED;
}

static int ad64p_gpio_probe(struct pci_dev *dev,
			const struct pci_device_id *id)
{
	struct ad64p_gpio *gpio;
	resource_size_t start;
	resource_size_t len;
	int err;

	err = pci_enable_device(dev);
	if (err < 0) {
		dev_err(&dev->dev, "failed to enable device: %d\n", err);
		goto out;
	}

	err = pci_enable_msi(dev);
	if (err < 0) {
		dev_err(&dev->dev, "failed to enable MSI: %d\n", err);
		goto disable;
	}

	err = pci_request_regions(dev, "ad64p-gpio");
	if (err < 0) {
		dev_err(&dev->dev, "failed to request resources: %d\n", err);
		goto disable;
	}

	gpio = devm_kzalloc(&dev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio) {
		err = -ENOMEM;
		goto release;
	}

	start = pci_resource_start(dev, 0);
	len = pci_resource_len(dev, 0);

	gpio->base = devm_ioremap_nocache(&dev->dev, start, len);
	if (!gpio->base) {
		dev_err(&dev->dev, "failed to map I/O memory: %d\n", err);
		err = -ENXIO;
		goto release;
	}

	err = request_irq(dev->irq, ad64p_irq, 0, dev_name(&dev->dev), gpio);
	if (err < 0) {
		dev_err(&dev->dev, "failed to request IRQ#%d: %d\n", dev->irq,
			err);
		goto release;
	}

	gpio->chip.label = dev_name(&dev->dev);
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.dev = &dev->dev;
	gpio->chip.direction_input = ad64p_gpio_set_input;
	gpio->chip.get = ad64p_gpio_get;
	gpio->chip.direction_output = ad64p_gpio_set_output;
	gpio->chip.set = ad64p_gpio_set;
	gpio->chip.base = -1;
	gpio->chip.ngpio = 64;
	gpio->chip.of_node = dev->dev.of_node;

	gpio->num_regs = DIV_ROUND_UP(gpio->chip.ngpio, 32);

	pci_set_drvdata(dev, gpio);

	err = gpiochip_add(&gpio->chip);
	if (err < 0) {
		dev_err(&dev->dev, "failed to add GPIO chip: %d\n", err);
		goto free_irq;
	}

	if (IS_ENABLED(CONFIG_GPIO_AD64P_IRQ)) {
		err = ad64p_irq_setup(gpio);
		if (err < 0) {
			dev_err(&dev->dev, "failed to setup IRQ chip: %d\n",
				err);
			goto remove;
		}
	}

	spin_lock_init(&gpio->lock);
	goto out;

remove:
	gpiochip_remove(&gpio->chip);
free_irq:
	free_irq(dev->irq, dev);
release:
	pci_release_regions(dev);
disable:
	pci_disable_msi(dev);
	pci_disable_device(dev);
out:
	return err;
}

static void ad64p_gpio_remove(struct pci_dev *dev)
{
	struct ad64p_gpio *gpio = pci_get_drvdata(dev);

	if (IS_ENABLED(CONFIG_GPIO_AD64P_IRQ))
		ad64p_irq_teardown(gpio);

	gpiochip_remove(&gpio->chip);
	free_irq(dev->irq, dev);
	pci_release_regions(dev);
	pci_disable_msi(dev);
	pci_disable_device(dev);
}

static DEFINE_PCI_DEVICE_TABLE(ad64p_gpio_pci_table) = {
	{ PCI_VDEVICE(AVIONIC_DESIGN, 0x0002) },
	{ },
};
MODULE_DEVICE_TABLE(pci, ad64p_gpio_pci_table);

static struct pci_driver ad64p_gpio_driver = {
	.name = "ad64p-gpio",
	.id_table = ad64p_gpio_pci_table,
	.probe = ad64p_gpio_probe,
	.remove = ad64p_gpio_remove,
};
module_pci_driver(ad64p_gpio_driver);

MODULE_DESCRIPTION("Avionic Design GPIO (64 pin) expander driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
