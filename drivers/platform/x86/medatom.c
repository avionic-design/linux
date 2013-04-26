/*
 * Copyright (C) 2009-2012 Avionic Design GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/dmi.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c/tsc2007.h>
#include <sound/tpa6130a2-plat.h>

#define TSC2007_GPIO 193

/* SPI slave definitions */
static struct mtd_partition fpga_flash_parts[] = {
	{
		.name = "FPGA",
		.offset = 0,
		.size = 4 * 1024 * 1024,
	}, {
		.name = "FPGA (gold)",
		.offset = 4 * 1024 * 1024,
		.size = 4 * 1024 * 1024,
	},
};

static struct flash_platform_data fpga_flash_info = {
	.name = "FPGA",
	.parts = fpga_flash_parts,
	.nr_parts = ARRAY_SIZE(fpga_flash_parts),
	.type = "w25q64",
};

static struct spi_board_info __initdata spi_slaves[] = {
	{
		.modalias = "m25p80",
		.max_speed_hz = 25000000,
		.platform_data = &fpga_flash_info,
	},
};

static int tsc2003_init_irq(void)
{
	return 0;
}

static void tsc2003_exit_irq(void)
{
}

static int tsc2003_pendown(void *data)
{
	return gpio_get_value(TSC2007_GPIO) ? 0 : 1;
}

static struct tsc2007_platform_data tsc2003_platform_data = {
	.model = 2003,
	.x_plate_ohms = 50,
	.max_rt = 110,
	.poll_delay = 10,
	.poll_period = 20,
	.fuzzx = 16,
	.fuzzy = 16,
	.fuzzz = 16,
	.get_pendown_state = tsc2003_pendown,
	.init_platform_hw = tsc2003_init_irq,
	.exit_platform_hw = tsc2003_exit_irq,
};

static struct tpa6130a2_platform_data tpa6130_platform_data = {
	.power_gpio = -1,
};

static struct i2c_board_info i2c_i801[] = {
	{
		I2C_BOARD_INFO("lm63", 0x4c),
	},
};

static struct i2c_board_info i2c_misc[] = {
};

static struct i2c_board_info i2c_touch[] = {
	{ /* Texas Instruments TSC2003 I2C Touch Screen Controller */
		I2C_BOARD_INFO("tsc2007", 0x48),
		.platform_data = &tsc2003_platform_data,
	}, {
		I2C_BOARD_INFO("tpa6130a2", 0x60),
		.platform_data = &tpa6130_platform_data,
	},
};

static int __init medatom_init(void)
{
	int ret;

	if (of_have_populated_dt())
		return 0;

	ret = spi_register_board_info(spi_slaves, ARRAY_SIZE(spi_slaves));
	if (ret < 0)
		return ret;

	ret = i2c_register_board_info(0, i2c_misc, ARRAY_SIZE(i2c_i801));
	if (ret < 0)
		return ret;

	ret = i2c_register_board_info(1, i2c_touch, ARRAY_SIZE(i2c_misc));
	if (ret < 0)
		return ret;

	i2c_touch[0].irq = gpio_to_irq(TSC2007_GPIO);

	return i2c_register_board_info(2, i2c_touch, ARRAY_SIZE(i2c_touch));
}

static void __exit medatom_exit(void)
{
}

module_init(medatom_init);
module_exit(medatom_exit);

MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
MODULE_DESCRIPTION("Avionic Design Medatom board-specific setup");
MODULE_LICENSE("GPL v2");
