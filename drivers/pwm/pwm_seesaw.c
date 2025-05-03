/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adafruit_seesaw_pwm

#include <zephyr/drivers/mfd/mfd_seesaw.h>
#include <zephyr/drivers/pwm.h>

/* See https://learn.adafruit.com/adafruit-seesaw-atsamd09-breakout/pwm */

#define FIXED_CYCLES 0xffff

enum seesaw_mod_pwm {
	PWM_VAL  = 0x01,
	PWM_FREQ = 0x02,
};

struct seesaw_pwm_config {
	const struct device *seesaw;
};

static int seesaw_pwm_get_cycles_per_sec(const struct device *dev,
					 uint32_t channel, uint64_t *cycles)
{
	*cycles = FIXED_CYCLES;
	return 0;
}

static inline int seesaw_pwm_write(const struct device *seesaw_dev, uint8_t reg,
				   uint8_t channel, uint16_t value)
{
	const uint8_t buf[3] = {channel, (value >> 8), (value & 0xff)};

	return seesaw_write(seesaw_dev, SEESAW_MOD_PWM, reg, buf, 3);
}

static int seesaw_pwm_set_cycles(const struct device *dev, uint32_t channel,
				 uint32_t period, uint32_t pulse,
				 pwm_flags_t flags)
{
	int ret;
	const struct seesaw_pwm_config *const config = dev->config;
	uint16_t freq = 0;
	uint16_t val = 0;

	if (period > 0) {
		freq = FIXED_CYCLES / period;
		val = 0xffff * pulse / period;
	}

	ret = seesaw_pwm_write(config->seesaw, PWM_VAL, channel, val);
	if (ret) {
		return ret;
	}

	return seesaw_pwm_write(config->seesaw, PWM_FREQ, channel, freq);
}

static DEVICE_API(pwm, seesaw_pwm_api) = {
	.set_cycles = seesaw_pwm_set_cycles,
	.get_cycles_per_sec = seesaw_pwm_get_cycles_per_sec,
};

static int seesaw_pwm_init(const struct device *dev)
{
	const struct seesaw_pwm_config *const config = dev->config;

	return seesaw_claim_module(config->seesaw, SEESAW_MOD_PWM, dev);
}

#define SEESAW_PWM_INIT(inst)							 \
	static const struct seesaw_pwm_config config_##inst = {			 \
		.seesaw = DEVICE_DT_GET(DT_INST_PARENT(inst)),			 \
	};									 \
	DEVICE_DT_INST_DEFINE(inst, seesaw_pwm_init, NULL, NULL, &config_##inst, \
			      POST_KERNEL, CONFIG_MFD_INIT_PRIORITY, &seesaw_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(SEESAW_PWM_INIT)
