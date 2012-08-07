/**
 * Copyright (C) 2012 Avionic Design GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c-ocores.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

static unsigned int devnum = 0;

static int ocores_i2c_probe(struct pci_dev *dev,
			const struct pci_device_id *id)
{
	struct platform_device *pdev;
	struct resource res[2];
	int err;

	err = pci_enable_device_mem(dev);
	if (err < 0)
		return err;

	err = pci_enable_msi(dev);
	if (err < 0)
		goto disable;

	memset(res, 0, sizeof(res));

	res[0].start = pci_resource_start(dev, 0);
	res[0].end = pci_resource_end(dev, 0);
	res[0].flags = IORESOURCE_MEM;

	res[1].start = dev->irq;
	res[1].end = dev->irq;
	res[1].flags = IORESOURCE_IRQ;

	pdev = platform_device_alloc("ocores-i2c", devnum);
	if (!pdev) {
		err = -ENOMEM;
		goto disable_msi;
	}

	pdev->dev.of_node = dev->dev.of_node;
	pdev->dev.parent = &dev->dev;

	err = platform_device_add_resources(pdev, res, ARRAY_SIZE(res));
	if (err < 0)
		goto put;

	err = platform_device_add(pdev);
	if (err < 0)
		goto put;

	pci_set_drvdata(dev, pdev);
	devnum++;

	return 0;

put:
	platform_device_put(pdev);
disable_msi:
	pci_disable_msi(dev);
disable:
	pci_disable_device(dev);
	return err;
}

static void ocores_i2c_remove(struct pci_dev *dev)
{
	struct platform_device *pdev = pci_get_drvdata(dev);

	platform_device_unregister(pdev);
	pci_disable_msi(dev);
	pci_disable_device(dev);
}

static DEFINE_PCI_DEVICE_TABLE(ocores_i2c_pci_table) = {
	{ PCI_VDEVICE(AVIONIC_DESIGN, 0x0007) },
	{ },
};
MODULE_DEVICE_TABLE(pci, ocores_i2c_pci_table);

static struct pci_driver ocores_i2c_driver = {
	.name = "ocores-i2c-pci",
	.id_table = ocores_i2c_pci_table,
	.probe = ocores_i2c_probe,
	.remove = ocores_i2c_remove,
};
module_pci_driver(ocores_i2c_driver);

MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
MODULE_DESCRIPTION("OpenCores I2C controller driver");
MODULE_LICENSE("GPL v2");
