/*
 * Copyright (c) 2026 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_lptim_pwm

#include <errno.h>

#include <soc.h>
#include <stm32_ll_lptim.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys_clock.h>

#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_stm32_lptim, CONFIG_PWM_LOG_LEVEL);

/*
 * Only the 2-channel LPTIM variant (LPTIM1/2/3) is supported: LPTIM4/5 expose
 * a single WAVPOL-polarity output instead of addressable CH1/CH2 channels.
 */
#define LPTIM_MAX_CH 2u

/* Bounded spin count while waiting for a write-acknowledge (xxxOK) flag.
 * The LPTIM kernel clock can be as slow as LSE/128 (~256 Hz), and the ISR/ICR
 * poll-and-clear cycle documented in RM0477 46.4.11 takes a handful of kernel
 * clock periods, but nothing here should ever legitimately take this long -
 * bail out rather than hang forever on unexpected hardware behavior.
 */
#define LPTIM_OK_FLAG_TIMEOUT_US 100000

/** PWM data. */
struct pwm_stm32_lptim_data {
	/** LPTIM kernel clock (Hz), already divided by the prescaler. */
	uint32_t tim_clk;
	/* Reset controller device configuration */
	const struct reset_dt_spec reset;
};

/** PWM configuration. */
struct pwm_stm32_lptim_config {
	LPTIM_TypeDef *timer;
	uint32_t prescaler;
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	const struct pinctrl_dev_config *pcfg;
};

static uint32_t get_polarity(pwm_flags_t flags)
{
	if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL) {
		return LL_LPTIM_OUTPUT_POLARITY_REGULAR;
	}

	return LL_LPTIM_OUTPUT_POLARITY_INVERSE;
}

/*
 * Writes to ARR/CCRx/RCR/DIER only take effect once the previous write to the
 * same register has been acknowledged by the (possibly async) LPTIM kernel
 * clock domain. Poll the corresponding ISR "OK" flag and clear it via ICR
 * before allowing the next write, per RM0477 46.4.11.
 */
static int wait_and_clear_ok_flag(LPTIM_TypeDef *timer,
				   uint32_t (*is_active)(const LPTIM_TypeDef *),
				   void (*clear)(LPTIM_TypeDef *))
{
	int timeout = LPTIM_OK_FLAG_TIMEOUT_US;

	while (!is_active(timer)) {
		if (--timeout <= 0) {
			LOG_ERR("Timed out waiting for LPTIM write-ack flag");
			return -EIO;
		}
		k_busy_wait(1);
	}

	clear(timer);

	return 0;
}

static int lptim_set_autoreload(LPTIM_TypeDef *timer, uint32_t value)
{
	LL_LPTIM_SetAutoReload(timer, value);

	return wait_and_clear_ok_flag(timer, LL_LPTIM_IsActiveFlag_ARROK,
				       LL_LPTIM_ClearFlag_ARROK);
}

static int lptim_set_compare(LPTIM_TypeDef *timer, uint32_t channel, uint32_t value)
{
	if (channel == 1u) {
		LL_LPTIM_OC_SetCompareCH1(timer, value);
		return wait_and_clear_ok_flag(timer, LL_LPTIM_IsActiveFlag_CMP1OK,
					       LL_LPTIM_ClearFlag_CMP1OK);
	}

	LL_LPTIM_OC_SetCompareCH2(timer, value);
	return wait_and_clear_ok_flag(timer, LL_LPTIM_IsActiveFlag_CMP2OK,
				       LL_LPTIM_ClearFlag_CMP2OK);
}

static int pwm_stm32_lptim_set_cycles(const struct device *dev, uint32_t channel,
				       uint32_t period_cycles, uint32_t pulse_cycles,
				       pwm_flags_t flags)
{
	const struct pwm_stm32_lptim_config *cfg = dev->config;
	LPTIM_TypeDef *timer = cfg->timer;
	uint32_t ll_channel;
	int ret;

	if (channel < 1u || channel > LPTIM_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	ll_channel = channel - 1u;

	/* LPTIM ARR/CCRx are 16-bit only, no 32-bit variant exists. */
	if (period_cycles > UINT16_MAX) {
		LOG_ERR("Cannot set PWM output, period cycles %u exceeds 16-bit timer limit.",
			period_cycles);
		return -ENOTSUP;
	}

	if (period_cycles == 0u) {
		LL_LPTIM_CC_DisableChannel(timer, ll_channel);
		return 0;
	}

	/* ARR must be strictly greater than CCRx (RM0477 46.4.11). Write
	 * whichever of the two never causes a transient violation of that
	 * invariant first, based on the value currently in ARR.
	 */
	if (pulse_cycles >= period_cycles) {
		pulse_cycles = period_cycles - 1u;
	}

	if (pulse_cycles < LL_LPTIM_GetAutoReload(timer)) {
		ret = lptim_set_compare(timer, channel, pulse_cycles);
		if (ret < 0) {
			return ret;
		}
		ret = lptim_set_autoreload(timer, period_cycles);
	} else {
		ret = lptim_set_autoreload(timer, period_cycles);
		if (ret < 0) {
			return ret;
		}
		ret = lptim_set_compare(timer, channel, pulse_cycles);
	}
	if (ret < 0) {
		return ret;
	}

	if (!LL_LPTIM_CC_IsEnabledChannel(timer, ll_channel)) {
		LL_LPTIM_OC_SetPolarity(timer, ll_channel, get_polarity(flags));
		LL_LPTIM_CC_SetChannelMode(timer, ll_channel, LL_LPTIM_CCMODE_OUTPUT_PWM);
		LL_LPTIM_CC_EnableChannel(timer, ll_channel);
	}

	return 0;
}

static int pwm_stm32_lptim_get_cycles_per_sec(const struct device *dev,
					       uint32_t channel, uint64_t *cycles)
{
	struct pwm_stm32_lptim_data *data = dev->data;

	ARG_UNUSED(channel);

	*cycles = (uint64_t)data->tim_clk;

	return 0;
}

static DEVICE_API(pwm, pwm_stm32_lptim_driver_api) = {
	.set_cycles = pwm_stm32_lptim_set_cycles,
	.get_cycles_per_sec = pwm_stm32_lptim_get_cycles_per_sec,
};

static int pwm_stm32_lptim_init(const struct device *dev)
{
	struct pwm_stm32_lptim_data *data = dev->data;
	const struct pwm_stm32_lptim_config *cfg = dev->config;
	LPTIM_TypeDef *timer = cfg->timer;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	uint32_t tim_clk;
	uint32_t prescaler_div;
	int r;

	/* Enable clock and store its speed */
	r = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (r < 0) {
		LOG_ERR("Could not initialize clock (%d)", r);
		return r;
	}

	if (cfg->pclk_len > 1) {
		/* Enable kernel clock source, if the SoC's clock driver
		 * supports selecting one for this LPTIM instance.
		 */
		r = clock_control_configure(clk, (clock_control_subsys_t)&cfg->pclken[1], NULL);
		if (r != 0) {
			LOG_ERR("Could not configure clock (%d)", r);
			return r;
		}

		r = clock_control_get_rate(clk, (clock_control_subsys_t)&cfg->pclken[1], &tim_clk);
		if (r < 0) {
			LOG_ERR("Timer clock rate get error (%d)", r);
			return r;
		}
	} else {
		r = clock_control_get_rate(clk, (clock_control_subsys_t)&cfg->pclken[0], &tim_clk);
		if (r < 0) {
			LOG_ERR("Timer clock rate get error (%d)", r);
			return r;
		}
	}

	prescaler_div = 1u << (cfg->prescaler >> LPTIM_CFGR_PRESC_Pos);
	data->tim_clk = tim_clk / prescaler_div;

	/* Reset timer to default state using RCC */
	(void)reset_line_toggle_dt(&data->reset);

	/* configure pinmux */
	r = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (r < 0) {
		LOG_ERR("PWM pinctrl setup failed (%d)", r);
		return r;
	}

	/* CFGR can only be written while the LPTIM is disabled. */
	LL_LPTIM_SetClockSource(timer, LL_LPTIM_CLK_SOURCE_INTERNAL);
	LL_LPTIM_SetPrescaler(timer, cfg->prescaler);
	LL_LPTIM_SetUpdateMode(timer, LL_LPTIM_UPDATE_MODE_IMMEDIATE);
	LL_LPTIM_SetWaveform(timer, LL_LPTIM_OUTPUT_WAVEFORM_PWM);

	LL_LPTIM_Enable(timer);

	/* ARR/CCRx/RCR can only be written, and the counter only started,
	 * once at least 2 kernel clock cycles have elapsed since ENABLE was
	 * set (RM0477 46.4.4). This is a one-time init cost.
	 */
	k_busy_wait(DIV_ROUND_UP(2 * USEC_PER_SEC, data->tim_clk));

	LL_LPTIM_StartCounter(timer, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

	return 0;
}

#define PWM_DEVICE_INIT(index)								\
	static struct pwm_stm32_lptim_data pwm_stm32_lptim_data_##index = {		\
		.reset = RESET_DT_SPEC_GET(DT_INST_PARENT(index)),			\
	};										\
											\
	PINCTRL_DT_INST_DEFINE(index);							\
											\
	static const struct stm32_pclken pclken_##index[] =				\
					STM32_DT_CLOCKS(DT_INST_PARENT(index));		\
											\
	static const struct pwm_stm32_lptim_config pwm_stm32_lptim_config_##index = {	\
		.timer = (LPTIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(index)),		\
		.prescaler = LOG2(DT_PROP(DT_INST_PARENT(index), st_prescaler))		\
				<< LPTIM_CFGR_PRESC_Pos,				\
		.pclken = pclken_##index,						\
		.pclk_len = DT_NUM_CLOCKS(DT_INST_PARENT(index)),			\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),				\
	};										\
											\
	DEVICE_DT_INST_DEFINE(index, &pwm_stm32_lptim_init, NULL,			\
			    &pwm_stm32_lptim_data_##index,				\
			    &pwm_stm32_lptim_config_##index, POST_KERNEL,		\
			    CONFIG_PWM_INIT_PRIORITY,					\
			    &pwm_stm32_lptim_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT)
