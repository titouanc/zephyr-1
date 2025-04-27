/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adafruit_seesaw_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mfd/mfd_seesaw.h>

enum seesaw_mod_gpio {
	GPIO_DIRSET     = 0x02,
	GPIO_DIRCLR     = 0x03,
	GPIO_GPIO       = 0x04,
	GPIO_SET        = 0x05,
	GPIO_CLR        = 0x06,
	GPIO_TOGGLE     = 0x07,
	GPIO_INTENSET   = 0x08,
	GPIO_INTENCLR   = 0x09,
	GPIO_INTFLAG    = 0x0a,
	GPIO_PULLENSET  = 0x0b,
	GPIO_PULLENCLR  = 0x0c,
};

struct seesaw_gpio_config {
	const struct device *seesaw;
};

static int seesaw_gpio_configure(const struct device *dev,
				   gpio_pin_t pin, gpio_flags_t flags)
{
	const struct seesaw_gpio_config *const config = dev->config;

	if (flags & GPIO_OUTPUT) {
		seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_DIRSET, BIT(pin));
	} else if (flags & GPIO_INPUT) {
		seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_DIRCLR, BIT(pin));
	} else {
		return -ENOTSUP;
	}

	if (flags & GPIO_PULL_UP) {
		seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_SET, BIT(pin));
		seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_PULLENSET, BIT(pin));
	} else if (flags & GPIO_PULL_DOWN) {
		seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_CLR, BIT(pin));
		seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_PULLENSET, BIT(pin));
	}

	return 0;
}

static int seesaw_gpio_port_get_raw(const struct device *dev, uint32_t *value)
{
	uint32_t tmp;
	const struct seesaw_gpio_config *const config = dev->config;
	int ret = seesaw_read(config->seesaw, SEESAW_MOD_GPIO, GPIO_GPIO, (uint8_t *) &tmp, 4);

	if (ret == 0) {
		*value = sys_be32_to_cpu(ret);
	}

	return ret;
}

static int seesaw_gpio_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value)
{
	const struct seesaw_gpio_config *const config = dev->config;

	seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_SET, value & mask);
	seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_CLR, ~value & mask);

	return 0;
}

static int seesaw_gpio_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct seesaw_gpio_config *const config = dev->config;

	seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_SET, mask);

	return 0;
}

static int seesaw_gpio_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct seesaw_gpio_config *const config = dev->config;

	seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_CLR, mask);

	return 0;
}

static int seesaw_gpio_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	const struct seesaw_gpio_config *const config = dev->config;

	seesaw_write_uint32(config->seesaw, SEESAW_MOD_GPIO, GPIO_TOGGLE, mask);

	return 0;
}


static DEVICE_API(gpio, seesaw_gpio_api) = {
	.pin_configure = seesaw_gpio_configure,
	.port_get_raw = seesaw_gpio_port_get_raw,
	.port_set_masked_raw = seesaw_gpio_port_set_masked_raw,
	.port_set_bits_raw = seesaw_gpio_port_set_bits_raw,
	.port_clear_bits_raw = seesaw_gpio_port_clear_bits_raw,
	.port_toggle_bits = seesaw_gpio_port_toggle_bits,
};

static int seesaw_gpio_init(const struct device *dev)
{
	const struct seesaw_gpio_config *config = dev->config;

	return seesaw_claim_module(config->seesaw, SEESAW_MOD_GPIO, dev);
}

#define SEESAW_GPIO_INIT(inst)							  \
	static const struct seesaw_gpio_config config_##inst = {		  \
		.seesaw = DEVICE_DT_GET(DT_INST_PARENT(inst)),			  \
	};									  \
	DEVICE_DT_INST_DEFINE(inst, seesaw_gpio_init, NULL, NULL, &config_##inst, \
			      POST_KERNEL, CONFIG_MFD_INIT_PRIORITY, &seesaw_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(SEESAW_GPIO_INIT)
