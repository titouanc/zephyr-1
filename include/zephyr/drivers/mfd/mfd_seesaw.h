/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MFD_SEESAW_H_
#define ZEPHYR_DRIVERS_MFD_SEESAW_H_

#include <stdint.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>

/* See https://learn.adafruit.com/adafruit-seesaw-atsamd09-breakout/using-the-seesaw-platform */
enum seesaw_mod {
	SEESAW_MOD_STATUS	= 0x00,
	SEESAW_MOD_GPIO		= 0x01,
	SEESAW_MOD_SERCOM0	= 0x02,
	SEESAW_MOD_SERCOM1	= 0x03,
	SEESAW_MOD_SERCOM2	= 0x04,
	SEESAW_MOD_SERCOM3	= 0x05,
	SEESAW_MOD_SERCOM4	= 0x06,
	SEESAW_MOD_SERCOM5	= 0x07,
	SEESAW_MOD_PWM		= 0x08,
	SEESAW_MOD_ADC		= 0x09,
	SEESAW_MOD_DAC		= 0x0a,
	SEESAW_MOD_INTERRUPT	= 0x0b,
	SEESAW_MOD_DAP		= 0x0c,
	SEESAW_MOD_EEPROM	= 0x0d,
	SEESAW_MOD_NEOPIXEL	= 0x0e,
	SEESAW_MOD_TOUCH	= 0x0f,
	SEESAW_MOD_KEYPAD	= 0x10,
	SEESAW_MOD_ENCODER	= 0x11,
	SEESAW_MOD_MAX_NUM,
};

int seesaw_claim_module(const struct device *dev, uint32_t module,
			const struct device *subdevice);

int seesaw_cmd(const struct device *dev, enum seesaw_mod module, uint8_t reg);

int seesaw_write(const struct device *dev, enum seesaw_mod module, uint8_t reg,
		 const uint8_t *buf, size_t len);

int seesaw_read(const struct device *dev, enum seesaw_mod module, uint8_t reg,
		uint8_t *buf, size_t len);

static inline int seesaw_write_uint32(const struct device *dev, enum seesaw_mod module,
				      uint8_t reg, uint32_t value)
{
	uint32_t buf = sys_cpu_to_be32(value);

	return seesaw_write(dev, module, reg, (const uint8_t *) &buf, 4);
}

static inline int seesaw_write_uint16(const struct device *dev, enum seesaw_mod module,
				      uint8_t reg, uint16_t value)
{
	uint16_t buf = sys_cpu_to_be16(value);

	return seesaw_write(dev, module, reg, (const uint8_t *) &buf, 2);
}

#endif
