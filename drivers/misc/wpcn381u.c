/*
 * Copyright (C) 2010 Avionic Design GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <asm/io.h>

#define	DRVNAME	"wpcn381u"
#define	WPCN_EXTENT	2

/* Index/Data Register Pair */
#define	WPCN_IDX	0x00 /* Index Register */
#define	WPCN_DAT	0x01 /* Data Register */

/* SuperI/O Control and Configuration Registers */
#define	WPCN_LDN	0x07 /* SuperI/O Logical Device Number */
#define	WPCN_ID		0x20 /* SuperI/O ID */
#define	WPCN_CONF1	0x21 /* SuperI/O Configuration 1 */
#define	WPCN_CONF2	0x22 /* SuperI/O Configuration 2 */
#define	WPCN_CONF6	0x26 /* SuperI/O Configuration 6 */
#define	WPCN_REV	0x27 /* SuperI/O Revision ID */
#define	WPCN_CONFC	0x2c /* SuperI/O Configuration C */

/* Logical Device Control and Configuration Registers */
#define	WPCN_LDC	0x30 /* Logical Device Control (Activate) */
#define	WPCN_IO_HIGH	0x60 /* I/O Base Address Descriptor 0 (Bits 15-8) */
#define	WPCN_IO_LOW	0x61 /* I/O Base Address Descriptor 0 (Bits 7-0) */
#define	WPCN_IRQ	0x70 /* Interrupt Number and Wake-Up on IRQ Enable */
#define	WPCN_IRQ_TYPE	0x71 /* IRQ Type Select */
#define	WPCN_DMA_0	0x74 /* DMA Channel Select 0 */
#define	WPCN_DMA_1	0x75 /* DMA Channel Select 1 */

struct wpcn {
	unsigned long base;
	int uart_line[2];
	u16 gpio_base;
};

static struct platform_device *pdev;

static const unsigned short bases[] = { 0x2e, 0x164e, };

static void wpcn_device_select(struct wpcn *wpcn, u8 dev)
{
	outb(WPCN_LDN, wpcn->base + WPCN_IDX);
	outb(dev, wpcn->base + WPCN_DAT);
}

static u8 wpcn_device_read(struct wpcn *wpcn, u8 dev, u8 addr)
{
	wpcn_device_select(wpcn, dev);
	outb(addr, wpcn->base + WPCN_IDX);
	return inb(wpcn->base + WPCN_DAT);
}

static void wpcn_device_write(struct wpcn *wpcn, u8 dev, u8 addr, u8 data)
{
	wpcn_device_select(wpcn, dev);
	outb(addr, wpcn->base + WPCN_IDX);
	outb(data, wpcn->base + WPCN_DAT);
}

static int wpcn_detect(unsigned short *base)
{
	unsigned int i;
	u8 id;

	if (!base)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(bases); i++) {
		outb(WPCN_ID, bases[i] + WPCN_IDX);
		id = inb(bases[i] + WPCN_DAT);

		if (id == 0xf4) {
			*base = bases[i];
			return 0;
		}
	}

	return -ENODEV;
}

static int wpcn_device_enable(struct wpcn *wpcn, u8 device)
{
	wpcn_device_write(wpcn, 0x02, WPCN_LDC, 0x1);
	msleep(250);
	return 0;
}

static int wpcn_device_get_base(struct wpcn *wpcn, u8 device, u16 *base)
{
	u8 msb, lsb;

	if (!wpcn || !base)
		return -EINVAL;

	msb = wpcn_device_read(wpcn, device, WPCN_IO_HIGH);
	lsb = wpcn_device_read(wpcn, device, WPCN_IO_LOW);
	*base = (msb << 8) | lsb;

	return 0;
}

static int wpcn_device_get_irq(struct wpcn *wpcn, u8 device, u8 *irq)
{
	if (!wpcn || !irq)
		return -EINVAL;

	*irq = wpcn_device_read(wpcn, device, WPCN_IRQ);
	return 0;
}

static int wpcn_device_add(unsigned short base)
{
	struct resource res = {
		.start = base,
		.end = base + WPCN_EXTENT - 1,
		.name = DRVNAME,
		.flags = IORESOURCE_IO,
	};
	int err;

	printk(KERN_INFO "> %s(base=%04x)\n", __func__, base);

	pdev = platform_device_alloc(DRVNAME, -1);
	if (!pdev) {
		printk(KERN_ERR DRVNAME ": device allocation failed (%d)\n", err);
		err = -ENOMEM;
		goto out;
	}

	err = platform_device_add_resources(pdev, &res, 1);
	if (err < 0) {
		printk(KERN_ERR DRVNAME ": device resource addition failed (%d)\n", err);
		goto put_device;
	}

	err = platform_device_add(pdev);
	if (err < 0) {
		printk(KERN_ERR DRVNAME ": device addition failed (%d)\n", err);
		goto out;
	}

	printk(KERN_INFO "< %s()\n", __func__);
	return 0;

put_device:
	platform_device_put(pdev);
out:
	printk(KERN_INFO "< %s() = %d\n", __func__, err);
	return err;
}

static int wpcn_register_port(struct device *dev, unsigned long base,
		unsigned int irq)
{
	struct uart_port uart;
	int i;

	dev_info(dev, "checking serial port @%04lx:\n", base);

	for (i = 0; i < 8; i++) {
		u8 val = inb(base + i);
		dev_info(dev, "  %02lx: %02x\n", base + i, val);
	}

	memset(&uart, 0, sizeof(uart));
	uart.flags = UPF_SHARE_IRQ;
	uart.uartclk = 1843200;
	uart.iobase = base;
	uart.iotype = UPIO_PORT;
	uart.irq = irq;
	uart.dev = dev;

	return serial8250_register_port(&uart);
}

static int wpcn_probe(struct platform_device *pdev)
{
	struct wpcn *wpcn;
	struct resource *res;
	struct resource *io;
	int ret = 0;

	dev_info(&pdev->dev, "> %s(pdev=%p)\n", __func__, pdev);

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res) {
		ret = -ENXIO;
		goto out;
	}

	io = devm_request_region(&pdev->dev, res->start, resource_size(res),
			DRVNAME);
	if (!io) {
		ret = -ENXIO;
		goto out;
	}

	wpcn = devm_kzalloc(&pdev->dev, sizeof(*wpcn), GFP_KERNEL);
	if (!wpcn) {
		ret = -ENOMEM;
		goto out;
	}

	wpcn->base = res->start;
	wpcn->uart_line[0] = -1;
	wpcn->uart_line[1] = -1;

	platform_set_drvdata(pdev, wpcn);

	if (wpcn_device_enable(wpcn, 0x3) == 0) {
		u16 base;
		u8 irq;

		ret = wpcn_device_get_base(wpcn, 0x3, &base);
		if (ret < 0)
			goto out;

		ret = wpcn_device_get_irq(wpcn, 0x3, &irq);
		if (ret < 0)
			goto out;

		dev_info(&pdev->dev, "  SP1 detected @%x IRQ:%u\n", base, irq);

		/* detect serial port 1 */
		ret = wpcn_register_port(&pdev->dev, base, irq);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register SP1: %d\n", ret);
			goto out;
		}

		dev_info(&pdev->dev, "SP1 registered on line %d (IO:%04x, "
				"IRQ:%d)\n", ret, base, irq);
		wpcn->uart_line[0] = ret;
	}

	if (wpcn_device_enable(wpcn, 0x2) == 0) {
		u16 base;
		u8 irq;

		ret = wpcn_device_get_base(wpcn, 0x2, &base);
		if (ret < 0)
			goto unregister_sp1;

		ret = wpcn_device_get_irq(wpcn, 0x2, &irq);
		if (ret < 0)
			goto unregister_sp1;

		dev_info(&pdev->dev, "  SP2 detected @%x IRQ:%u\n", base, irq);

		/* detect fast infrared port/serial port 2 */
		ret = wpcn_register_port(&pdev->dev, base, irq);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register FIR/SP2: %d\n", ret);
			goto unregister_sp1;
		}

		dev_info(&pdev->dev, "SP2 registered on line %d (IO:%04x, IRQ:%d)\n",
				ret, base, irq);
		wpcn->uart_line[1] = ret;
	}

	ret = 0;

out:
	dev_info(&pdev->dev, "< %s() = %d\n", __func__, ret);
	return ret;

unregister_sp1:
	serial8250_unregister_port(wpcn->uart_line[0]);
	goto out;
}

static int wpcn_remove(struct platform_device *pdev)
{
	struct wpcn *wpcn = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "> %s(pdev=%p)\n", __func__, pdev);

	platform_set_drvdata(pdev, NULL);
	serial8250_unregister_port(wpcn->uart_line[1]);
	serial8250_unregister_port(wpcn->uart_line[0]);

	dev_info(&pdev->dev, "< %s()\n", __func__);
	return -ENOSYS;
}

static struct platform_driver wpcn_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DRVNAME,
	},
	.probe = wpcn_probe,
	.remove = wpcn_remove,
};

static int __init wpcn_init(void)
{
	unsigned short base = 0;
	int err;

	err = wpcn_detect(&base);
	if (err < 0)
		return err;

	printk(KERN_INFO "WPCN381U detected at I/O port %#04x\n", base);

	err = platform_driver_register(&wpcn_driver);
	if (err < 0)
		return err;

	err = wpcn_device_add(base);
	if (err < 0) {
		printk(KERN_ERR DRVNAME ": device addition failed (%d)\n", err);
		platform_driver_unregister(&wpcn_driver);
		return err;
	}

	return 0;
}

static void __exit wpcn_exit(void)
{
	platform_device_unregister(pdev);
	platform_driver_unregister(&wpcn_driver);
}

module_init(wpcn_init);
module_exit(wpcn_exit);

MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
MODULE_DESCRIPTION("Winbond Legacy-Reduced SuperI/O WPCN381U driver");
MODULE_LICENSE("GPL v2");
