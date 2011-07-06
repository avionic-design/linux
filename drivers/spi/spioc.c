/*
 * linux/drivers/spi/spioc.c
 *
 * Copyright (C) 2007-2008 Avionic Design Development GmbH
 * Copyright (C) 2008-2012 Avionic Design GmbH
 *
 * Partially inspired by code from linux/drivers/spi/pxa2xx_spi.c.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Written by Thierry Reding <thierry.reding@avionic-design.de>
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/spi/spi.h>

/* register definitions */
#define	SPIOC_RX(i)	(i * 4)
#define	SPIOC_TX(i)	(i * 4)
#define	SPIOC_CTRL	0x10
#define	SPIOC_DIV	0x14
#define	SPIOC_SS	0x18

/* SPIOC_CTRL register */
#define	CTRL_LEN(x)	((x < 128) ? x : 0)
#define	CTRL_BUSY	(1 <<  8)
#define	CTRL_RXNEG	(1 <<  9)
#define	CTRL_TXNEG	(1 << 10)
#define	CTRL_LSB	(1 << 11)
#define	CTRL_IE		(1 << 12)
#define	CTRL_ASS	(1 << 13)

#define	SPIOC_MAX_LEN	((unsigned int)16)

static const u32 clock_mode[4] = {
	[SPI_MODE_0] = CTRL_RXNEG,
	[SPI_MODE_1] = CTRL_TXNEG,
	[SPI_MODE_2] = CTRL_RXNEG,
	[SPI_MODE_3] = CTRL_TXNEG,
};

/* valid SPI mode bits */
#define	MODEBITS	(SPI_CPHA | SPI_CPOL | SPI_LSB_FIRST)

struct spioc_ctldata {
	u32 ctrl;
	u32 div;
};

struct spioc {
	struct spi_master *master;
	void __iomem *base;
	unsigned long rate;

	struct workqueue_struct *workqueue;
	struct work_struct work;
	struct tasklet_struct tasklet;

	struct list_head queue;
	struct completion msg_done;
	unsigned int state;
	unsigned int busy;
	spinlock_t lock;

	struct spi_device *slave;
	struct spi_message *message;
	struct spi_transfer *transfer;
	unsigned int cur_pos;
	unsigned int cur_len;
};

/* queue states */
#define	QUEUE_STOPPED	0
#define	QUEUE_RUNNING	1

static inline u32 spioc_read(struct spioc *spioc, unsigned long offset)
{
	return readl(spioc->base + offset);
}

static inline void spioc_write(struct spioc *spioc, unsigned long offset,
		u32 value)
{
	writel(value, spioc->base + offset);
}

static u32 spioc_get_clkdiv(struct spioc *spioc, unsigned long speed)
{
	return DIV_ROUND_UP(spioc->rate, 2 * speed) - 1;
}

static void spioc_chipselect(struct spioc *spioc, struct spi_device *spi)
{
	if (spi != spioc->slave) {
		u32 ss = spi ? (1 << spi->chip_select) : 0;
		spioc_write(spioc, SPIOC_SS, ss);
		spioc->slave = spi;
	}
}

static void spioc_copy_tx(struct spioc *spioc)
{
	const void *src;
	u32 val = 0;
	int i;

	if (!spioc->transfer->tx_buf)
		return;

	src = spioc->transfer->tx_buf + spioc->cur_pos;

	for (i = 0; i < spioc->cur_len; i++) {
		int rem = spioc->cur_len - i;
		int reg = (rem - 1) / 4;
		int ofs = (rem - 1) % 4;

		val |= (((u8 *)src)[i] & 0xff) << (ofs * 8);
		if (!ofs) {
			spioc_write(spioc, SPIOC_TX(reg), val);
			val = 0;
		}
	}
}

static void spioc_copy_rx(struct spioc *spioc)
{
	void *dest;
	u32 val = 0;
	int i;

	if (!spioc->transfer->rx_buf)
		return;

	dest = spioc->transfer->rx_buf + spioc->cur_pos;

	for (i = 0; i < spioc->cur_len; i++) {
		int rem = spioc->cur_len - i;
		int reg = (rem - 1) / 4;
		int ofs = (rem - 1) % 4;

		if ((i == 0) || (rem % 4 == 0))
			val = spioc_read(spioc, SPIOC_RX(reg));

		((u8 *)dest)[i] = (val >> (ofs * 8)) & 0xff;
	}
}

static inline struct spi_transfer *next_transfer(struct list_head *head)
{
	return list_entry(head->next, struct spi_transfer, transfer_list);
}

static void finish_message(struct spioc *spioc, int ec)
{
	struct spi_message *message = spioc->message;

	spioc->transfer = NULL;
	spioc->message = NULL;

	message->status = ec;

	if (message->complete)
		message->complete(message->context);
}

static void queue_message(struct spioc *spioc)
{
	if (spioc->state == QUEUE_RUNNING)
		queue_work(spioc->workqueue, &spioc->work);

	if (spioc->state == QUEUE_STOPPED)
		complete(&spioc->msg_done);
}

static struct spi_transfer *continue_message(struct spioc *spioc)
{
	struct spi_transfer *transfer = spioc->transfer;
	struct spi_message *message = spioc->message;
	unsigned long flags;

	if (!transfer)
		return next_transfer(&message->transfers);

	if (transfer->transfer_list.next != &message->transfers)
		return next_transfer(&transfer->transfer_list);

	spin_lock_irqsave(&spioc->lock, flags);
	finish_message(spioc, 0);
	queue_message(spioc);
	spin_unlock_irqrestore(&spioc->lock, flags);

	return NULL;
}

static void process_transfers(unsigned long data)
{
	struct spioc *spioc = (struct spioc *)data;
	struct spi_transfer *transfer;
	struct spi_message *message;
	struct spioc_ctldata *ctl;
	u32 ctrl = 0;
	u32 div = 0;

	if (!spioc->message) {
		dev_alert(&spioc->master->dev, "invalid message\n");
		return;
	}

	WARN_ON(spioc->message == NULL);
	message = spioc->message;
	transfer = spioc->transfer;

	/* finish up the last partial transfer */
	if (transfer) {
		spioc_copy_rx(spioc);
		spioc->message->actual_length += spioc->cur_len;
		spioc->cur_pos += spioc->cur_len;
	}

	/* proceed to next (or first) transfer in message */
	if (!transfer || (spioc->cur_pos >= transfer->len)) {
		if (transfer) {
			if (transfer->delay_usecs)
				udelay(transfer->delay_usecs);

			/*
			if (!transfer->cs_change)
				spioc_chipselect(spioc, NULL);
			*/
		}

		transfer = continue_message(spioc);
		if (!transfer) {
			spioc_chipselect(spioc, NULL);
			return;
		}

		spioc->transfer = transfer;
		spioc->cur_pos = 0;
		spioc->cur_len = 0;
	}

	ctl = spi_get_ctldata(message->spi);
	div = ctl->div;

	if (transfer->speed_hz) {
		div = spioc_get_clkdiv(spioc, transfer->speed_hz);
		if (div > 0xffff) {
			finish_message(spioc, -EIO);
			return;
		}
	}

	spioc->cur_len = min(transfer->len - spioc->cur_pos, SPIOC_MAX_LEN);
	spioc_copy_tx(spioc);

	ctrl = ctl->ctrl;
	ctrl |= CTRL_LEN(spioc->cur_len * 8);
	ctrl |= CTRL_BUSY;
	ctrl |= CTRL_IE;

	spioc_chipselect(spioc, spioc->message->spi);
	spioc_write(spioc, SPIOC_DIV, div);
	spioc_write(spioc, SPIOC_CTRL, ctrl);
}

static void process_messages(struct work_struct *work)
{
	struct spioc *spioc = container_of(work, struct spioc, work);
	struct spi_message *message;
	unsigned long flags;

	WARN_ON(spioc->message != NULL);

	spin_lock_irqsave(&spioc->lock, flags);

	if (list_empty(&spioc->queue)) {
		spioc->busy = 0;
		goto unlock;
	}

	message = list_entry(spioc->queue.next, struct spi_message, queue);
	list_del_init(&message->queue);
	spioc->message = message;
	tasklet_schedule(&spioc->tasklet);
	spioc->busy = 1;

unlock:
	spin_unlock_irqrestore(&spioc->lock, flags);
}

static int spioc_setup(struct spi_device *spi)
{
	struct spioc *spioc = spi_master_get_devdata(spi->master);
	struct spioc_ctldata *ctl = spi_get_ctldata(spi);
	u32 div = 0;

	if (spi->mode & ~MODEBITS)
		return -EINVAL;

	if (!spi->max_speed_hz)
		return -EINVAL;

	div = spioc_get_clkdiv(spioc, spi->max_speed_hz);
	if (div > 0xffff)
		return -EINVAL;

	if (!ctl) {
		ctl = kzalloc(sizeof(*ctl), GFP_KERNEL);
		if (!ctl)
			return -EINVAL;

		spi_set_ctldata(spi, ctl);
	}

	ctl->div = div;
	ctl->ctrl = 0;

	if (spi->mode & SPI_LSB_FIRST)
		ctl->ctrl |= CTRL_LSB;

	ctl->ctrl |= clock_mode[spi->mode & 0x3];

	return 0;
}

static int spioc_transfer(struct spi_device *spi, struct spi_message *message)
{
	struct spioc *spioc = spi_master_get_devdata(spi->master);
	unsigned long flags = 0;

	spin_lock_irqsave(&spioc->lock, flags);

	if (spioc->state == QUEUE_STOPPED) {
		spin_unlock_irqrestore(&spioc->lock, flags);
		return -ESHUTDOWN;
	}

	message->status = -EINPROGRESS;
	message->actual_length = 0;

	list_add_tail(&message->queue, &spioc->queue);

	if ((spioc->state == QUEUE_RUNNING) && !spioc->busy)
		queue_work(spioc->workqueue, &spioc->work);

	spin_unlock_irqrestore(&spioc->lock, flags);

	return 0;
}

static void spioc_cleanup(struct spi_device *spi)
{
	struct spioc_ctldata *ctl = spi_get_ctldata(spi);
	spi_set_ctldata(spi, NULL);
	kfree(ctl);
}

static irqreturn_t spioc_interrupt(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct spioc *spioc;

	if (!dev_id)
		return IRQ_NONE;

	spioc = spi_master_get_devdata(master);
	tasklet_schedule(&spioc->tasklet);

	return IRQ_HANDLED;
}

static int init_queue(struct spi_master *master)
{
	struct spioc *spioc = spi_master_get_devdata(master);

	spioc->workqueue = create_workqueue(dev_name(master->dev.parent));
	if (!spioc->workqueue)
		return -ENOMEM;

	INIT_WORK(&spioc->work, process_messages);
	tasklet_init(&spioc->tasklet, process_transfers,
			(unsigned long)spioc);

	INIT_LIST_HEAD(&spioc->queue);
	init_completion(&spioc->msg_done);
	spin_lock_init(&spioc->lock);

	spioc->state = QUEUE_STOPPED;
	spioc->busy = 0;

	return 0;
}

static int start_queue(struct spi_master *master)
{
	struct spioc *spioc = spi_master_get_devdata(master);
	unsigned long flags;

	spin_lock_irqsave(&spioc->lock, flags);

	if ((spioc->state == QUEUE_RUNNING) || spioc->busy) {
		spin_unlock_irqrestore(&spioc->lock, flags);
		return -EBUSY;
	}

	spioc->state = QUEUE_RUNNING;
	spioc->message = NULL;
	spioc->transfer = NULL;

	spin_unlock_irqrestore(&spioc->lock, flags);

	queue_work(spioc->workqueue, &spioc->work);
	return 0;
}

static int stop_queue(struct spi_master *master)
{
	struct spioc *spioc = spi_master_get_devdata(master);
	unsigned long flags;
	unsigned int empty;
	unsigned int busy;

	spin_lock_irqsave(&spioc->lock, flags);

	empty = list_empty(&spioc->queue);
	spioc->state = QUEUE_STOPPED;
	busy = spioc->busy;

	spin_unlock_irqrestore(&spioc->lock, flags);

	if (!empty && busy)
		wait_for_completion(&spioc->msg_done);

	return 0;
}

static int destroy_queue(struct spi_master *master)
{
	struct spioc *spioc = spi_master_get_devdata(master);
	int ret;

	ret = stop_queue(master);
	if (ret < 0)
		return ret;

	destroy_workqueue(spioc->workqueue);

	return 0;
}

static int spioc_probe(struct pci_dev *dev,
		const struct pci_device_id *id)
{
	struct spi_master *master;
	resource_size_t start;
	resource_size_t len;
	struct spioc *spioc;
	int ret;

	ret = pci_enable_device(dev);
	if (ret < 0) {
		dev_err(&dev->dev, "failed to enable PCI device: %d\n", ret);
		goto out;
	}

	ret = pci_enable_msi(dev);
	if (ret < 0) {
		dev_err(&dev->dev, "failed to enable MSI: %d\n", ret);
		goto disable;
	}

	ret = pci_request_regions(dev, "spioc");
	if (ret < 0) {
		dev_err(&dev->dev, "failed to request resources: %d\n", ret);
		goto disable_msi;
	}

	master = spi_alloc_master(&dev->dev, sizeof(*spioc));
	if (!master) {
		dev_err(&dev->dev, "failed to allocate SPI master\n");
		ret = -ENOMEM;
		goto release;
	}

	master->dev.of_node = dev->dev.of_node;

	/* set SPI bus number and number of chipselects */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->bus_num = 0;
	master->num_chipselect = 8;
	master->setup = spioc_setup;
	master->transfer = spioc_transfer;
	master->cleanup = spioc_cleanup;
	pci_set_drvdata(dev, master);

	spioc = spi_master_get_devdata(master);
	if (!spioc) {
		ret = -ENXIO;
		goto put;
	}

	spioc->master = master;
	spioc->slave = NULL;
	spioc->rate = 99000000;

	start = pci_resource_start(dev, 0);
	len = pci_resource_len(dev, 0);

	spioc->base = devm_ioremap_nocache(&dev->dev, start, len);
	if (!spioc->base) {
		ret = -ENXIO;
		goto put;
	}

	ret = devm_request_irq(&dev->dev, dev->irq, spioc_interrupt, 0,
			"spioc", master);
	if (ret < 0) {
		dev_err(&master->dev, "failed to install handler for "
				"IRQ%d\n", dev->irq);
		goto put;
	}

	ret = init_queue(master);
	if (ret < 0) {
		dev_err(&master->dev, "failed to initialize message queue\n");
		goto put;
	}

	ret = start_queue(master);
	if (ret < 0) {
		dev_err(&master->dev, "failed to start message queue\n");
		goto put;
	}

	ret = spi_register_master(master);
	if (ret < 0) {
		dev_err(&master->dev, "failed to register SPI master\n");
		goto put;
	}

	goto out;

put:
	pci_set_drvdata(dev, NULL);
	spi_master_put(master);
release:
	pci_release_regions(dev);
disable_msi:
	pci_disable_msi(dev);
disable:
	pci_disable_device(dev);
out:
	return ret;
}

static void spioc_remove(struct pci_dev *dev)
{
	struct spi_master *master = pci_get_drvdata(dev);

	devm_free_irq(&dev->dev, dev->irq, master);
	spi_unregister_master(master);
	destroy_queue(master);
	spi_master_put(master);
	pci_release_regions(dev);
	pci_disable_msi(dev);
	pci_disable_device(dev);
}

static DEFINE_PCI_DEVICE_TABLE(spioc_pci_table) = {
	{ PCI_VDEVICE(AVIONIC_DESIGN, 0x0005) },
	{ },
};
MODULE_DEVICE_TABLE(pci, spioc_pci_table);

static struct pci_driver spioc_driver = {
	.name = "spioc",
	.id_table = spioc_pci_table,
	.probe = spioc_probe,
	.remove = spioc_remove,
};
module_pci_driver(spioc_driver);

MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
MODULE_DESCRIPTION("OpenCores SPI controller driver");
MODULE_LICENSE("GPL v2");
