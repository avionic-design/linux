/*
 * Copyright (C) 2011-2012 Avionic Design GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

#include <linux/input/sx8634.h>

#define I2C_IRQ_SRC 0x00
#define I2C_IRQ_SRC_MODE (1 << 0)
#define I2C_IRQ_SRC_COMPENSATION (1 << 1)
#define I2C_IRQ_SRC_BUTTONS (1 << 2)
#define I2C_IRQ_SRC_SLIDER (1 << 3)
#define I2C_IRQ_SRC_GPI (1 << 4)
#define I2C_IRQ_SRC_SPM (1 << 5)
#define I2C_IRQ_SRC_NVM (1 << 6)
#define I2C_IRQ_SRC_READY (1 << 7)

#define I2C_CAP_STAT_MSB 0x01
#define I2C_CAP_STAT_LSB 0x02
#define I2C_SLD_POS_MSB 0x03
#define I2C_SLD_POS_LSB 0x04
#define I2C_GPI_STAT 0x07
#define I2C_SPM_STAT 0x08
#define I2C_COMP_OP_MODE 0x09
#define I2C_GPO_CTRL 0x0a
#define I2C_GPP_PIN_ID 0x0b
#define I2C_GPP_INTENSITY 0x0c
#define I2C_SPM_CFG 0x0d
#define I2C_SPM_CFG_WRITE (0 << 3)
#define I2C_SPM_CFG_READ (1 << 3)
#define I2C_SPM_CFG_OFF (0 << 4)
#define I2C_SPM_CFG_ON (1 << 4)
#define I2C_SPM_BASE 0x0e
#define I2C_SPM_KEY_MSB 0xac
#define I2C_SPM_KEY_LSB 0xad
#define I2C_SOFT_RESET 0xb1

#define SPM_CFG 0x00
#define SPM_CAP_MODE_MISC 0x09

#define SPM_CAP_MODE(x) (((x) <= 3) ? 0x0c : (((x) <= 7) ? 0x0b : 0x0a))
#define SPM_CAP_MODE_SHIFT(x) (((x) & 3) * 2)
#define SPM_CAP_MODE_MASK 0x3
#define SPM_CAP_MODE_MASK_SHIFTED(x) \
	(SPM_CAP_MODE_MASK << SPM_CAP_MODE_SHIFT(x))

#define SPM_CAP_SENS(x) (0x0d + ((x) / 2))
#define SPM_CAP_SENS_MAX 0x7
#define SPM_CAP_SENS_SHIFT(x) (((x) & 1) ? 0 : 4)
#define SPM_CAP_SENS_MASK 0x7
#define SPM_CAP_SENS_MASK_SHIFTED(x) \
	(SPM_CAP_SENS_MASK << SPM_CAP_SENS_SHIFT(x))

#define SPM_CAP_THRESHOLD(x) (0x13 + (x))
#define SPM_CAP_THRESHOLD_MAX 0xff

#define SPM_BTN_CFG 0x21
#define SPM_BTN_CFG_TOUCH_DEBOUNCE_MASK 0x03
#define SPM_BTN_CFG_TOUCH_DEBOUNCE_SHIFT 0

#define SPM_BLOCK_SIZE 8
#define SPM_NUM_BLOCKS 16
#define SPM_SIZE (SPM_BLOCK_SIZE * SPM_NUM_BLOCKS)

#define SLD_POS_STEP 12

static int sensitivity = -1;
module_param(sensitivity, int, S_IRUGO);
MODULE_PARM_DESC(sensitivity, "pad sensitivity (0-7)");

static int threshold = -1;
module_param(threshold, int, S_IRUGO);
MODULE_PARM_DESC(threshold, "sample threshold (0-100)");

static int debounce = -1;
module_param(debounce, int, S_IRUGO);
MODULE_PARM_DESC(debounce, "number of debounce samples (1-4)");

#define HACK_RETRY_IRQ 1

struct sx8634 {
	struct i2c_client *client;
	struct input_dev *input;
	unsigned short keycodes[SX8634_NUM_CAPS];
	struct completion spm_complete;
	unsigned long spm_dirty;
	u8 spm_cache[SPM_SIZE];
#ifdef HACK_RETRY_IRQ
	bool initialized;
#endif
	u16 slider_max;
	u16 status;
	int power_gpio;
};

static inline int sx8634_read(struct sx8634 *sx, unsigned offset, u8 *value)
{
	int err;

	err = i2c_smbus_read_byte_data(sx->client, offset);
	if (err < 0) {
		dev_err(&sx->client->dev, "failed to read register %02x: %d\n",
			offset, err);
		return err;
	}

	*value = err;
	return 0;
}

static inline int sx8634_write(struct sx8634 *sx, unsigned offset, u8 value)
{
	int err;

	err = i2c_smbus_write_byte_data(sx->client, offset, value);
	if (err < 0)
		dev_err(&sx->client->dev, "failed to write register %02x: %d\n",
			offset, err);

	return err;
}

static inline int sx8634_read_block(struct sx8634 *sx, void *buffer,
				    size_t size)
{
	return i2c_smbus_read_i2c_block_data(sx->client, 0, size, buffer);
}

static inline int sx8634_write_block(struct sx8634 *sx, const void *buffer,
				     size_t size)
{
	return i2c_smbus_write_i2c_block_data(sx->client, 0, size, buffer);
}

static int spm_wait(struct sx8634 *sx)
{
	unsigned long timeout = msecs_to_jiffies(50);

#if 1
	if (wait_for_completion_timeout(&sx->spm_complete, timeout) == 0)
		return -ETIMEDOUT;
#else
	if (wait_for_completion_timeout(&sx->spm_complete, timeout) == 0) {
		u8 value;
		int err;

		/*
		 * Check the interrupt source register to make sure we haven't
		 * missed the interrupt. The hardware doesn't seem to reliably
		 * report SPM write completion, but most of the time at least
		 * this bit will still be set.
		 */
		err = sx8634_read(sx, I2C_IRQ_SRC, &value);
		if (err < 0)
			return err;

		return (value & I2C_IRQ_SRC_SPM) ? 0 : -ETIMEDOUT;
	}
#endif

	return 0;
}

static inline ssize_t spm_start_read(struct sx8634 *sx, loff_t offset)
{
	u8 data[2];

	data[0] = I2C_SPM_CFG_ON | I2C_SPM_CFG_READ;
	data[1] = offset & 0xf8;

	return i2c_smbus_write_i2c_block_data(sx->client, I2C_SPM_CFG,
					      sizeof(data), data);
}

static inline ssize_t spm_start_write(struct sx8634 *sx, loff_t offset)
{
	u8 data[2];

	data[0] = I2C_SPM_CFG_ON | I2C_SPM_CFG_WRITE;
	data[1] = offset & 0xf8;

	return i2c_smbus_write_i2c_block_data(sx->client, I2C_SPM_CFG,
					      sizeof(data), data);
}

static inline int spm_stop(struct sx8634 *sx)
{
	return sx8634_write(sx, I2C_SPM_CFG, 0);
}

static ssize_t spm_read_block(struct sx8634 *sx, loff_t offset, void *buffer,
			      size_t size)
{
	int err;

	BUG_ON(size != SPM_BLOCK_SIZE);
	BUG_ON((offset & 7) != 0);

	err = spm_start_read(sx, offset);
	if (err < 0) {
		dev_dbg(&sx->client->dev,
			"failed to start SPM read cycle: %d\n", err);
		return err;
	}

	err = sx8634_read_block(sx, buffer, SPM_BLOCK_SIZE);
	if (err < 0) {
		dev_dbg(&sx->client->dev, "failed to read SPM block: %d\n",
			err);
		return err;
	}

	err = spm_stop(sx);
	if (err < 0) {
		dev_dbg(&sx->client->dev, "failed to stop SPM read cycle: %d\n",
			err);
		return err;
	}

	return 0;
}

static ssize_t spm_write_block(struct sx8634 *sx, loff_t offset,
			       const void *buffer, size_t size)
{
	int err;

	BUG_ON(size != SPM_BLOCK_SIZE);
	BUG_ON((offset & 7) != 0);

	err = spm_start_write(sx, offset);
	if (err < 0) {
		dev_dbg(&sx->client->dev,
			"failed to start SPM write cycle: %d\n", err);
		return err;
	}

	err = sx8634_write_block(sx, buffer, SPM_BLOCK_SIZE);
	if (err < 0) {
		dev_dbg(&sx->client->dev, "failed to write SPM block: %d\n",
			err);
		return err;
	}

	err = spm_stop(sx);
	if (err < 0) {
		dev_dbg(&sx->client->dev,
			"failed to stop SPM write cycle: %d\n", err);
		return err;
	}

	dev_dbg(&sx->client->dev, "SPM block %llu written\n",
		offset / SPM_BLOCK_SIZE);

	err = spm_wait(sx);
	if (err < 0) {
		dev_dbg(&sx->client->dev,
			"failed to wait for SPM write completion: %d\n", err);
		return err;
	}

	return 0;
}

static ssize_t sx8634_spm_load(struct sx8634 *sx)
{
	loff_t offset;
	ssize_t err;

	if (sx->spm_dirty != 0)
		dev_warn(&sx->client->dev, "discarding modified SPM cache\n");

	memset(sx->spm_cache, 0, SPM_SIZE);

	for (offset = 0; offset < SPM_SIZE; offset += SPM_BLOCK_SIZE) {
		void *buffer = sx->spm_cache + offset;

		err = spm_read_block(sx, offset, buffer, SPM_BLOCK_SIZE);
		if (err < 0) {
			dev_err(&sx->client->dev, "spm_read_block(): %d\n",
				err);
			return err;
		}
	}

	sx->spm_dirty = 0;

	return 0;
}

static ssize_t sx8634_spm_sync(struct sx8634 *sx)
{
	int bit;

	for_each_set_bit(bit, &sx->spm_dirty, SPM_NUM_BLOCKS) {
		loff_t offset = bit * SPM_BLOCK_SIZE;
		ssize_t err;

		err = spm_write_block(sx, offset, sx->spm_cache + offset,
				      SPM_BLOCK_SIZE);
		if (err < 0) {
			dev_err(&sx->client->dev, "spm_write_block(): %d\n",
				err);
			return err;
		}

		clear_bit(bit, &sx->spm_dirty);
	}

	return 0;
}

static int sx8634_spm_read(struct sx8634 *sx, unsigned int offset, u8 *value)
{
	if (offset >= SPM_SIZE)
		return -ENXIO;

	*value = sx->spm_cache[offset];

	return 0;
}

static int sx8634_spm_write(struct sx8634 *sx, unsigned int offset, u8 value)
{
	if (offset >= SPM_SIZE)
		return -ENXIO;

	if (value != sx->spm_cache[offset]) {
		set_bit(offset / SPM_BLOCK_SIZE, &sx->spm_dirty);
		sx->spm_cache[offset] = value;
	}

	return 0;
}

static irqreturn_t sx8634_irq(int irq, void *data)
{
	unsigned int retries = 8;
	struct sx8634 *sx = data;

	while (true) {
		bool need_sync = false;
		u8 pending;
		int err;

		err = sx8634_read(sx, I2C_IRQ_SRC, &pending);
		if (err < 0) {
			if (retries--) {
				usleep_range(8000, 16000);
				continue;
			}

			return IRQ_NONE;
		}

		if (pending == 0) {
			dev_dbg(&sx->client->dev, "no pending interrupts\n");
			break;
		}

		if (pending & I2C_IRQ_SRC_MODE)
			dev_dbg(&sx->client->dev, "operating mode changed\n");

		if (pending & I2C_IRQ_SRC_COMPENSATION)
			dev_dbg(&sx->client->dev, "compensation complete\n");

		if (pending & I2C_IRQ_SRC_BUTTONS) {
			unsigned long changed;
			unsigned int cap;
			u16 status;
			u8 value;

			dev_dbg(&sx->client->dev, "%s(): button event\n",
				__func__);

			err = sx8634_read(sx, I2C_CAP_STAT_MSB, &value);
			if (err < 0)
				return IRQ_NONE;

			status = value << 8;

			err = sx8634_read(sx, I2C_CAP_STAT_LSB, &value);
			if (err < 0)
				return IRQ_NONE;

			status |= value;

			changed = status ^ sx->status;

			for_each_set_bit(cap, &changed, SX8634_NUM_CAPS) {
				int level = (status & BIT(cap)) ? 1 : 0;
				input_report_key(sx->input, sx->keycodes[cap],
						 level);
				need_sync = true;
			}

			sx->status = status;
		}

		if (pending & I2C_IRQ_SRC_SLIDER) {
			u16 position;
			u8 value;

			dev_dbg(&sx->client->dev, "%s(): slider event\n",
				__func__);

			err = sx8634_read(sx, I2C_SLD_POS_MSB, &value);
			if (err < 0)
				return IRQ_NONE;

			position = value << 8;

			err = sx8634_read(sx, I2C_SLD_POS_LSB, &value);
			if (err < 0)
				return IRQ_NONE;

			position |= value;

			input_report_abs(sx->input, ABS_MISC, position);
		}

		if (need_sync || (pending & I2C_IRQ_SRC_SLIDER))
			input_sync(sx->input);

		if (pending & I2C_IRQ_SRC_GPI)
			dev_dbg(&sx->client->dev, "%s(): GPI event\n",
				__func__);

		if (pending & I2C_IRQ_SRC_SPM) {
			dev_dbg(&sx->client->dev, "%s(): SPM event\n",
				__func__);
			complete(&sx->spm_complete);
		}

		if (pending & I2C_IRQ_SRC_NVM)
			dev_dbg(&sx->client->dev, "%s(): NVM event\n",
				__func__);

		if (pending & I2C_IRQ_SRC_READY)
			dev_dbg(&sx->client->dev, "%s(): ready event\n",
				__func__);

#ifdef HACK_RETRY_IRQ
		if (sx->initialized)
			msleep(30);
		else
			break;
#else
		break;
#endif
	}

	return IRQ_HANDLED;
}

static int sx8634_set_mode(struct sx8634 *sx, unsigned int cap,
			   enum sx8634_cap_mode mode)
{
	u8 value = 0;
	int err;

	if ((cap >= SX8634_NUM_CAPS) || (mode == SX8634_CAP_MODE_RESERVED))
		return -EINVAL;

	err = sx8634_spm_read(sx, SPM_CAP_MODE(cap), &value);
	if (err < 0)
		return err;

	value &= ~SPM_CAP_MODE_MASK_SHIFTED(cap);
	value |= (mode & SPM_CAP_MODE_MASK) << SPM_CAP_MODE_SHIFT(cap);

	err = sx8634_spm_write(sx, SPM_CAP_MODE(cap), value);
	if (err < 0)
		return err;

	return 0;
}

static int sx8634_set_sensitivity(struct sx8634 *sx, unsigned int cap,
				  u8 sensitivity)
{
	u8 value = 0;
	int err = 0;

	if (cap >= SX8634_NUM_CAPS || sensitivity > 0x7)
		return -EINVAL;

	err = sx8634_spm_read(sx, SPM_CAP_SENS(cap), &value);
	if (err < 0)
		return err;

	value &= ~SPM_CAP_SENS_MASK_SHIFTED(cap);
	value |= (sensitivity & SPM_CAP_SENS_MASK) << SPM_CAP_SENS_SHIFT(cap);

	err = sx8634_spm_write(sx, SPM_CAP_SENS(cap), value);
	if (err < 0)
		return err;

	return 0;
}

static int sx8634_set_threshold(struct sx8634 *sx, unsigned int cap,
				u8 threshold)
{
	int err;

	if (cap >= SX8634_NUM_CAPS || threshold > 0xa0)
		return -EINVAL;

	err = sx8634_spm_write(sx, SPM_CAP_THRESHOLD(cap), threshold);
	if (err < 0)
		return err;

	return 0;
}

static int sx8634_set_debounce(struct sx8634 *sx, unsigned int samples)
{
	u8 value = 0;
	int err;

	if (samples < 1 || samples > 4)
		return -EINVAL;

	err = sx8634_spm_read(sx, SPM_BTN_CFG, &value);
	if (err < 0)
		return err;

	value &= ~SPM_BTN_CFG_TOUCH_DEBOUNCE_MASK;
	value |= (samples - 1) << SPM_BTN_CFG_TOUCH_DEBOUNCE_SHIFT;

	err = sx8634_spm_write(sx, SPM_BTN_CFG, value);
	if (err < 0)
		return err;

	return 0;
}

static int sx8634_pwm_enable(struct sx8634 *sx, unsigned int gpio,
			     unsigned int brightness)
{
	unsigned int mode = (gpio >= 4) ? 0x40 : 0x41;
	unsigned int mode_shift = (gpio & 0x3) * 2;
	unsigned int mode_mask = 0x3 << mode_shift;
	u8 value;
	int err;

	if (!sx || gpio > 7 || brightness > 255)
		return -EINVAL;

	/* use GPIO[7] as PWM */
	err = sx8634_spm_read(sx, mode, &value);
	if (err < 0)
		return err;

	value = (value & ~mode_mask) | (0x1 << mode_shift);

	err = sx8634_spm_write(sx, mode, value);
	if (err < 0)
		return err;

	/* invert polarity */
	err = sx8634_spm_read(sx, 0x44, &value);
	if (err < 0)
		return err;

	value &= ~BIT(gpio);

	err = sx8634_spm_write(sx, 0x44, value);
	if (err < 0)
		return err;

	err = sx8634_spm_sync(sx);
	if (err < 0)
		return err;

	/* set PWM brightness */
	err = sx8634_write(sx, I2C_GPP_PIN_ID, gpio);
	if (err < 0)
		return err;

	err = sx8634_write(sx, I2C_GPP_INTENSITY, brightness);
	if (err < 0)
		return err;

	return 0;
}

static int sx8634_setup(struct sx8634 *sx, struct sx8634_platform_data *pdata)
{
	bool slider = false;
	unsigned int i;
	u8 value;
	int err;

	err = sx8634_spm_load(sx);
	if (err < 0)
		return err;

	/* disable all capacitive sensors */
	for (i = 0; i < SX8634_NUM_CAPS; i++) {
		err = sx8634_set_mode(sx, i, SX8634_CAP_MODE_DISABLED);
		if (err < 0)
			return err;
	}

	err = sx8634_spm_sync(sx);
	if (err < 0)
		return err;

	/* FIXME: make this configurable */
	err = sx8634_pwm_enable(sx, 0x7, 0xff);
	if (err < 0) {
		dev_err(&sx->client->dev, "%s failed: %d\n",
			"sx8634_pwm_enable()", err);
		return err;
	}

	/* configure capacitive sensor parameters */
	for (i = 0; i < SX8634_NUM_CAPS; i++) {
		struct sx8634_cap *cap = &pdata->caps[i];

		if (sensitivity < 0)
			value = cap->sensitivity;
		else
			value = sensitivity;

		err = sx8634_set_sensitivity(sx, i, value);
		if (err < 0)
			dev_err(&sx->client->dev, "%s failed: %d\n",
				"sx8634_set_sensitivity()", err);

		if (threshold < 0)
			value = cap->threshold;
		else
			value = threshold;

		err = sx8634_set_threshold(sx, i, value);
		if (err < 0)
			dev_err(&sx->client->dev, "%s failed: %d\n",
				"sx8634_set_threshold()", err);
	}

	if (debounce < 0)
		value = pdata->debounce;
	else
		value = debounce;

	err = sx8634_set_debounce(sx, value);
	if (err < 0)
		return err;

	err = sx8634_spm_sync(sx);
	if (err < 0)
		return err;

	/* enable individual cap sensitivity */
	err = sx8634_spm_write(sx, SPM_CAP_MODE_MISC, 0x04);
	if (err < 0)
		return err;

	/* enable capacitive sensors */
	for (i = 0; i < SX8634_NUM_CAPS; i++) {
		struct sx8634_cap *cap = &pdata->caps[i];

		if (cap->mode == SX8634_CAP_MODE_BUTTON) {
			input_set_capability(sx->input, EV_KEY, cap->keycode);
			sx->keycodes[i] = cap->keycode;
		}

		if (cap->mode == SX8634_CAP_MODE_SLIDER) {
			if (slider)
				sx->slider_max += SLD_POS_STEP;

			slider = true;
		}

		err = sx8634_set_mode(sx, i, cap->mode);
		if (err < 0)
			dev_err(&sx->client->dev, "%s failed: %d\n",
				"sx8634_set_mode()", err);
	}

	err = sx8634_spm_sync(sx);
	if (err < 0)
		return err;

	sx->input->id.bustype = BUS_I2C;
	sx->input->id.product = 0;
	sx->input->id.version = 0;
	sx->input->name = "sx8634";
	sx->input->dev.parent = &sx->client->dev;

	/* setup slider */
	if (slider) {
		input_set_abs_params(sx->input, ABS_MISC, 0, sx->slider_max,
				     0, 0);
		input_set_capability(sx->input, EV_ABS, ABS_MISC);
	}

	return 0;
}

static ssize_t sx8634_spm_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sx8634 *sx = i2c_get_clientdata(client);
	ssize_t len = 0;
	size_t i, j;
	int err;

	err = sx8634_spm_load(sx);
	if (err < 0)
		return err;

	for (i = 0; i < SPM_SIZE; i += SPM_BLOCK_SIZE) {
		const char *prefix = "";

		for (j = 0; j < SPM_BLOCK_SIZE; j++) {
			len += sprintf(buf + len, "%s%02x", prefix,
				       sx->spm_cache[i + j]);
			prefix = " ";
		}

		len += sprintf(buf + len, "\n");
	}

	return len;
}

static DEVICE_ATTR(spm, 0664, sx8634_spm_show, NULL);

static struct attribute *sx8634_attributes[] = {
	&dev_attr_spm.attr,
	NULL
};

static const struct attribute_group sx8634_attr_group = {
	.attrs = sx8634_attributes,
};

static int sx8634_parse_dt(struct device *dev,
			   struct sx8634_platform_data *pdata)
{
	struct device_node *node = dev->of_node;
	struct device_node *child = NULL;
	u32 sensitivity_def = 0x00;
	u32 threshold_def = 0xa0;
	u32 debounce = 1;
	int err;

	if (!node)
		return -ENODEV;

	memset(pdata, 0, sizeof(*pdata));

	of_property_read_u32(node, "threshold", &threshold_def);

	if (threshold_def > SPM_CAP_THRESHOLD_MAX) {
		dev_info(dev, "invalid threshold: %u, using %u\n",
			 threshold_def, SPM_CAP_THRESHOLD_MAX);
		threshold_def = SPM_CAP_THRESHOLD_MAX;
	}

	of_property_read_u32(node, "sensitivity", &sensitivity_def);

	if (sensitivity_def > SPM_CAP_SENS_MAX) {
		dev_info(dev, "invalid sensitivity: %u, using %u\n",
			 sensitivity_def, SPM_CAP_SENS_MAX);
		sensitivity_def = SPM_CAP_SENS_MAX;
	}

	of_property_read_u32(node, "debounce", &debounce);

	if (debounce < 1 || debounce > 4) {
		dev_info(dev,
			 "invalid number of debounce samples: %u, using %u\n",
			 debounce, clamp_t(u32, debounce, 1, 4));
		debounce = clamp_t(u32, debounce, 1, 4);
	}

	pdata->debounce = debounce;

	while ((child = of_get_next_child(node, child))) {
		u32 sensitivity = sensitivity_def;
		u32 threshold = threshold_def;
		struct sx8634_cap *cap;
		u32 keycode;
		u32 index;

		err = of_property_read_u32(child, "reg", &index);
		if (err < 0) {
			dev_err(dev, "\"reg\" property missing for node %s\n",
				child->name);
			continue;
		}

		if (index >= SX8634_NUM_CAPS) {
			dev_err(dev, "invalid value for \"reg\" property: %u\n",
				index);
			continue;
		}

		cap = &pdata->caps[index];

		of_property_read_u32(child, "threshold", &threshold);
		cap->threshold = threshold;

		of_property_read_u32(child, "sensitivity", &sensitivity);
		cap->sensitivity = sensitivity;

		err = of_property_read_u32(child, "linux,code", &keycode);
		if (err == 0) {
			cap->mode = SX8634_CAP_MODE_BUTTON;
			cap->keycode = keycode;
		} else {
			cap->mode = SX8634_CAP_MODE_SLIDER;
		}
	}

	pdata->power_gpio = of_get_named_gpio(node, "power-gpios", 0);

	return 0;
}

static int sx8634_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sx8634_platform_data *pdata = client->dev.platform_data;
	struct device_node *node = client->dev.of_node;
	struct sx8634_platform_data defpdata;
	unsigned int retries = 8;
	struct sx8634 *sx;
	int err = 0;
	u8 value;

	if (IS_ENABLED(CONFIG_OF) && node) {
		client->irq = irq_of_parse_and_map(node, 0);
		if (client->irq == NO_IRQ)
			return -EPROBE_DEFER;
	}

	if (!pdata) {
		if (!IS_ENABLED(CONFIG_OF))
			return -ENODEV;

		err = sx8634_parse_dt(&client->dev, &defpdata);
		if (err < 0)
			return err;

		pdata = &defpdata;
	}

	sx = devm_kzalloc(&client->dev, sizeof(*sx), GFP_KERNEL);
	if (!sx)
		return -ENOMEM;

	sx->input = input_allocate_device();
	if (!sx->input)
		return -ENOMEM;

	init_completion(&sx->spm_complete);
	sx->power_gpio = pdata->power_gpio;
	sx->client = client;

	if (gpio_is_valid(sx->power_gpio)) {
		err = gpio_request_one(sx->power_gpio, GPIOF_OUT_INIT_HIGH,
				       "sx8634 power");
		if (err < 0) {
			dev_err(&client->dev,
				"failed to setup power GPIO: %d\n", err);
			goto free_input_device;
		}

		msleep(150);
	}

	err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					sx8634_irq, IRQF_ONESHOT, "sx8634",
					sx);
	if (err < 0) {
		dev_err(&client->dev, "can't allocate IRQ#%d\n", client->irq);
		goto free_power_gpio;
	}

	/* clear interrupts */
	err = sx8634_read(sx, I2C_IRQ_SRC, &value);
	if (err < 0) {
		dev_err(&client->dev, "can't clear interrupts: %d\n", err);
		goto remove_sysfs;
	}

	while (retries--) {
		err = sx8634_setup(sx, pdata);
		if (err < 0) {
			dev_dbg(&client->dev, "setup failed, retrying...\n");
			continue;
		}

		break;
	}

	if (err < 0)
		goto free_power_gpio;

#ifdef HACK_RETRY_IRQ
	sx->initialized = true;
#endif

	err = sysfs_create_group(&client->dev.kobj, &sx8634_attr_group);
	if (err < 0)
		goto free_power_gpio;

	err = input_register_device(sx->input);
	if (err < 0)
		goto remove_sysfs;

	i2c_set_clientdata(client, sx);

	return 0;

remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &sx8634_attr_group);
free_power_gpio:
	if (gpio_is_valid(sx->power_gpio)) {
		gpio_direction_output(sx->power_gpio, 0);
		gpio_free(sx->power_gpio);
	}
free_input_device:
	input_free_device(sx->input);
	return err;
}

static int sx8634_i2c_remove(struct i2c_client *client)
{
	struct sx8634 *sx = i2c_get_clientdata(client);

	devm_free_irq(&client->dev, client->irq, sx);
	input_unregister_device(sx->input);
	sysfs_remove_group(&client->dev.kobj, &sx8634_attr_group);

	if (gpio_is_valid(sx->power_gpio)) {
		gpio_direction_output(sx->power_gpio, 0);
		gpio_free(sx->power_gpio);
	}

	return 0;
}

static const struct i2c_device_id sx8634_i2c_ids[] = {
	{ "sx8634", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx8634_i2c_ids);

static struct i2c_driver sx8634_driver = {
	.driver = {
		.name = "sx8634",
		.owner = THIS_MODULE,
	},
	.probe = sx8634_i2c_probe,
	.remove = sx8634_i2c_remove,
	.id_table = sx8634_i2c_ids,
};
module_i2c_driver(sx8634_driver);

MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
MODULE_DESCRIPTION("Semtech SX8634 Controller Driver");
MODULE_LICENSE("GPL");
