/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adafruit_seesaw_neopixel

#include <zephyr/drivers/led.h>
#include <zephyr/drivers/mfd/mfd_seesaw.h>
#include <zephyr/dt-bindings/led/led.h>

enum seesaw_mod_neopixel {
	NEOPIXEL_PIN = 0x01,
	NEOPIXEL_SPEED = 0x02,
	NEOPIXEL_BUF_LENGTH = 0x03,
	NEOPIXEL_BUF = 0x04,
	NEOPIXEL_SHOW = 0x05,
};

struct seesaw_neopixel_config {
	const struct device *seesaw;
	size_t length;
	const uint8_t *color_mapping;
	uint8_t num_colors;
	uint8_t output_pin;
};

static int seesaw_neopixel_set_color(const struct device *dev, uint32_t led, uint8_t num_colors,
				     const uint8_t *color)
{
	int ret;
	const struct seesaw_neopixel_config *config = dev->config;
	uint16_t offset = config->num_colors * led;
	uint8_t buf[6] = {offset >> 8, offset & 0xff};

	if (led >= config->length || num_colors != config->num_colors) {
		return -EINVAL;
	}

	memcpy(buf + 2, color, num_colors);

	ret = seesaw_write(config->seesaw, SEESAW_MOD_NEOPIXEL, NEOPIXEL_BUF, buf, 2 + num_colors);
	if (ret) {
		return ret;
	}

	return seesaw_cmd(config->seesaw, SEESAW_MOD_NEOPIXEL, NEOPIXEL_SHOW);
}

static int seesaw_neopixel_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{
	const struct seesaw_neopixel_config *config = dev->config;
	uint8_t b = 255 * value / 100;
	uint8_t color[] = {b, b, b, b};

	if (value > 100) {
		return -EINVAL;
	}

	return seesaw_neopixel_set_color(dev, led, config->num_colors, color);
}

static int seesaw_neopixel_on(const struct device *dev, uint32_t led)
{
	return seesaw_neopixel_set_brightness(dev, led, 100);
}

static int seesaw_neopixel_off(const struct device *dev, uint32_t led)
{
	return seesaw_neopixel_set_brightness(dev, led, 0);
}

static int seesaw_neopixel_init(const struct device *dev)
{
	const struct seesaw_neopixel_config *config = dev->config;
	int ret = seesaw_claim_module(config->seesaw, SEESAW_MOD_NEOPIXEL, dev);

	if (ret) {
		return ret;
	}

	seesaw_write(config->seesaw, SEESAW_MOD_NEOPIXEL, NEOPIXEL_PIN, &config->output_pin, 1);
	seesaw_write_uint16(config->seesaw, SEESAW_MOD_NEOPIXEL,
			    NEOPIXEL_BUF_LENGTH, 3 * config->length);

	return 0;
}

static DEVICE_API(led, seesaw_neopixel_api) = {
	.on = seesaw_neopixel_on,
	.off = seesaw_neopixel_off,
	.set_brightness = seesaw_neopixel_set_brightness,
	.set_color = seesaw_neopixel_set_color,
};

#define SEESAW_NEOPIXEL_INIT(inst)                                                                 \
	static const uint8_t cmap_##idx[] = DT_INST_PROP(inst, color_mapping);                     \
	static const struct seesaw_neopixel_config config_##inst = {                               \
		.seesaw = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                     \
		.length = DT_INST_PROP(inst, chain_length),                                        \
		.color_mapping = cmap_##idx,                                                       \
		.num_colors = sizeof(cmap_##idx),                                                  \
		.output_pin = DT_INST_PROP(inst, output_pin),                                      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, seesaw_neopixel_init, NULL, NULL, &config_##inst, POST_KERNEL, \
				  CONFIG_LED_INIT_PRIORITY, &seesaw_neopixel_api);

DT_INST_FOREACH_STATUS_OKAY(SEESAW_NEOPIXEL_INIT)
