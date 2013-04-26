/*
 * Copyright (C) 2011-2012 Avionic Design GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_INPUT_SX8634_H__
#define __LINUX_INPUT_SX8634_H__

#define SX8634_NUM_CAPS 12

enum sx8634_cap_mode {
	SX8634_CAP_MODE_DISABLED,
	SX8634_CAP_MODE_BUTTON,
	SX8634_CAP_MODE_SLIDER,
	SX8634_CAP_MODE_RESERVED
};

struct sx8634_cap {
	enum sx8634_cap_mode mode;
	unsigned short keycode;
	u8 sensitivity;
	u8 threshold;
};

struct sx8634_platform_data {
	struct sx8634_cap caps[SX8634_NUM_CAPS];
	int power_gpio;
	int debounce;
};

#endif
