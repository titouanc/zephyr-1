/*
 * Copyright (c) 2025 Titouan Christophe
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/audio/codec.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>

#include "cs47l63.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cs47l63, CONFIG_AUDIO_CODEC_LOG_LEVEL);

#define DT_DRV_COMPAT cirrus_cs47l63

struct cs47l63_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec irq_gpio;
};

static int cs47l63_read_spi_regs(const struct spi_dt_spec *spi,
				 uint32_t start_reg, uint32_t *values, size_t count)
{
	int ret;
	/* Register address + READ bit set */
	uint32_t addr = sys_cpu_to_be32(BIT(31) | start_reg);
	struct spi_buf tx_buf = {.buf = (void *) &addr, .len = sizeof(addr)};
	struct spi_buf_set tx_bufset = {.buffers = &tx_buf, .count = 1};
	uint32_t rx_padbuf[2];
	struct spi_buf rx_bufs[2] = {
		/* 1 empty rx word while we write out the address
		 * + 1 padding word (see datasheet 4.13.1)
		 */
		{.buf = rx_padbuf, .len = sizeof(rx_padbuf)},
		/* The actual reading result */
		{.buf = values, .len = count * sizeof(uint32_t)},
	};

	struct spi_buf_set rx_bufset = {.buffers = rx_bufs, .count = 2};

	ret = spi_transceive_dt(spi, &tx_bufset, &rx_bufset);
	if (ret != 0) {
		LOG_ERR("Unable to read register: %d", ret);
		return ret;
	}

	for (size_t i = 0; i < count; i++) {
		values[i] = sys_be32_to_cpu(values[i]);
	}

	return 0;
}

static int cs47l63_write_spi_blob(const struct spi_dt_spec *spi,
				  uint32_t addr, const uint8_t *data, size_t len)
{
	int ret;
	uint32_t setup_buf[2] = {
		sys_cpu_to_be32(addr & 0x7FFFFFFF),
		0, /* Padding (see datasheet 4.13.1) */
	};
	struct spi_buf tx_bufs[2] = {
		{.buf = setup_buf, .len = sizeof(setup_buf)},
		{.buf = (void *) data, .len = len},
	};
	struct spi_buf_set bufset = {.buffers = tx_bufs, .count = 2};

	ret = spi_write_dt(spi, &bufset);
	if (ret != 0) {
		LOG_ERR("Unable to write blob[%d]: %d", len, ret);
	}

	return ret;
}

static inline int cs47l63_write_spi_reg(const struct spi_dt_spec *spi,
					uint32_t reg, uint32_t value)
{
	uint32_t encoded_value = sys_cpu_to_be32(value);

	return cs47l63_write_spi_blob(spi, reg, (const uint8_t *) &encoded_value, 4);
}

static inline int cs47l63_update_spi_reg(const struct spi_dt_spec *spi,
					 uint32_t reg, uint32_t value, uint32_t mask)
{
	int ret;
	uint32_t prev_value;

	ret = cs47l63_read_spi_regs(spi, reg, &prev_value, 1);
	if (ret != 0) {
		return ret;
	}

	return cs47l63_write_spi_reg(spi, reg, (prev_value & ~mask) | (value & mask));
}

static inline int cs47l63_sample_rate(uint32_t frame_clk_freq)
{
	switch (frame_clk_freq) {
	case 8000:
		return SAMPLE_RATE_8_kHz;
	case 11025:
		return SAMPLE_RATE_11_025_kHz;
	case 12000:
		return SAMPLE_RATE_12_kHz;
	case 16000:
		return SAMPLE_RATE_16_kHz;
	case 22050:
		return SAMPLE_RATE_22_05_kHz;
	case 24000:
		return SAMPLE_RATE_24_kHz;
	case 32000:
		return SAMPLE_RATE_32_kHz;
	case 44100:
		return SAMPLE_RATE_44_1_kHz;
	case 48000:
		return SAMPLE_RATE_48_kHz;
	case 88200:
		return SAMPLE_RATE_88_2_kHz;
	case 96000:
		return SAMPLE_RATE_96_kHz;
	case 176400:
		return SAMPLE_RATE_176_4_kHz;
	case 192000:
		return SAMPLE_RATE_192_kHz;
	default:
		LOG_WRN("Unsupported sample rate: %dHz", frame_clk_freq);
		return -ENOTSUP;
	}
}

static int cs47l63_configure(const struct device *dev, struct audio_codec_cfg *audiocfg)
{
	int ret;
	uint8_t format, sample_rate;
	uint8_t wordsize = audiocfg->dai_cfg.i2s.word_size;
	const struct cs47l63_config *cfg = dev->config;

	if (audiocfg->dai_route != AUDIO_ROUTE_PLAYBACK) {
		return -ENOTSUP;
	}

	if (wordsize < 16 || wordsize > 32) {
		LOG_WRN("Unsupported word size");
		return -ENOTSUP;
	}

	switch (audiocfg->dai_type) {
	case AUDIO_DAI_TYPE_LEFT_JUSTIFIED:
		format = ASP_FMT_LEFTJUST;
		break;
	case AUDIO_DAI_TYPE_I2S:
		format = ASP_FMT_I2S;
		break;
	default:
		LOG_WRN("Unsupported interface type");
		return -ENOTSUP;
	}

	sample_rate = cs47l63_sample_rate(audiocfg->dai_cfg.i2s.frame_clk_freq);
	if (sample_rate < 0) {
		return -ENOTSUP;
	}

	ret = cs47l63_write_spi_reg(&cfg->spi, CS47L63_REG_ASP1_CONTROL2,
				    (wordsize << 24) | (format << 8));
	if (ret != 0) {
		LOG_ERR("Unable to set DAI format on ASP1");
		return -EIO;
	}

	/* Set the number of valid data bits per slot */
	ret = cs47l63_write_spi_reg(&cfg->spi, CS47L63_REG_ASP1_DATA_CONTROL5, wordsize);
	if (ret != 0) {
		return -EIO;
	}

	/* Enable reception of channel 1 on ASP1 */
	ret = cs47l63_write_spi_reg(&cfg->spi, CS47L63_REG_ASP1_ENABLES1, BIT(16));
	if (ret != 0) {
		return -EIO;
	}

	ret = cs47l63_write_spi_reg(&cfg->spi, CS47L63_REG_SAMPLE_RATE1, sample_rate);
	if (ret != 0) {
		return -EIO;
	}

	/* Route ASP1 Rx channel 1 to the output */
	ret = cs47l63_write_spi_reg(&cfg->spi, CS47L63_REG_OUT1L_INPUT1, MIXER_SRC_ASP1_RX1);
	if (ret != 0) {
		return -EIO;
	}

	/* Enable output data path */
	ret = cs47l63_write_spi_reg(&cfg->spi, CS47L63_REG_OUTPUT_ENABLE_1, BIT(1));
	if (ret != 0) {
		return -EIO;
	}

	/* Unmute output */
	ret = cs47l63_update_spi_reg(&cfg->spi, CS47L63_REG_OUT1L_VOLUME_1,
				     BIT(9), BIT(9)|BIT(8));
	if (ret != 0) {
		return -EIO;
	}

	return 0;
}

static void cs47l63_start_output(const struct device *dev)
{
}

static void cs47l63_stop_output(const struct device *dev)
{
}

static int cs47l63_set_property(const struct device *dev, audio_property_t property,
				audio_channel_t channel, audio_property_value_t val)
{
	const struct cs47l63_config *cfg = dev->config;
	int ret;
	uint32_t value, mask;

	if (channel != AUDIO_CHANNEL_HEADPHONE_LEFT) {
		return -ENOTSUP;
	}

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_MUTE:
		mask = BIT(8);
		value = val.mute ? BIT(8) : 0;
		break;
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		mask = 0xFF;
		value = 0xBF * val.vol / 100;
		break;
	default:
		return -ENOTSUP;
	}

	ret = cs47l63_update_spi_reg(&cfg->spi, CS47L63_REG_OUT1L_VOLUME_1,
				     BIT(9)|value, BIT(9)|mask);
	return (ret == 0) ? 0 : -EIO;
}

static int cs47l63_apply_properties(const struct device *dev)
{
	return 0;
}

static const struct audio_codec_api cs47l63_driver_api = {
	.configure = cs47l63_configure,
	.start_output = cs47l63_start_output,
	.stop_output = cs47l63_stop_output,
	.set_property = cs47l63_set_property,
	.apply_properties = cs47l63_apply_properties,
};

static inline bool wait_gpio(const struct gpio_dt_spec *gpio, int expected_state,
			     int timeout_ms)
{
	while (timeout_ms-- > 0) {
		if (gpio_pin_get_dt(gpio) == expected_state) {
			return true;
		}

		k_msleep(1);
	}

	return false;
}

static int cs47l63_init(const struct device *dev)
{
	const struct cs47l63_config *cfg = dev->config;
	uint32_t devid[2];
	uint32_t zeros[4];
	int ret;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Unable to setup IRQ pin");
		return -EIO;
	}

	/* Put device in hardware RESET */
	ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Unable to setup nreset GPIO");
		return -EIO;
	}

	/* Wait for device to be in reset state */
	if (!wait_gpio(&cfg->irq_gpio, 0, 10)) {
		LOG_ERR("Timed out while waiting for device reset");
		return -ETIMEDOUT;
	}

	/* Release RESET state */
	ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
	if (ret != 0) {
		LOG_ERR("Unable to complete hw reset");
		return -EIO;
	}

	/* Wait for device to be ready */
	if (!wait_gpio(&cfg->irq_gpio, 1, 10)) {
		LOG_ERR("Timed out while waiting for device start");
		return -ETIMEDOUT;
	}

	ret = cs47l63_read_spi_regs(&cfg->spi, CS47L63_REG_DEVID, devid, ARRAY_SIZE(devid));
	if (ret != 0) {
		LOG_ERR("Error reading DEVID: %d", ret);
		return -EIO;
	}

	if (devid[0] != 0x47A63) {
		LOG_WRN("Invalid device ID");
		return -ENODEV;
	}

	LOG_DBG("Found CS47L63 rev %d.%d", (devid[1] >> 4) & 0xF, devid[1] & 0xF);

	/* Set GPIO1-4 to pin-specific alternate function (ASP1) */
	ret = cs47l63_write_spi_blob(&cfg->spi, CS47L63_REG_GPIO1_CTRL1,
				     (const uint8_t *) zeros, 4);
	if (ret != 0) {
		LOG_ERR("Unable to setup GPIO1-4 to ASP1");
		return -EIO;
	}

	/* Enable SYSCLK */
	ret = cs47l63_update_spi_reg(&cfg->spi, CS47L63_REG_SYSTEM_CLOCK1, BIT(6), BIT(6));
	if (ret != 0) {
		LOG_ERR("Unable to enable SYSCLK");
		return -EIO;
	}

	return 0;
}

#define CS47L63_SPI_OPMODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8))

#define CS47L63_INIT(inst)                                                \
	static const struct cs47l63_config cs47l63_config_##inst = {      \
		.spi = SPI_DT_SPEC_INST_GET(inst, CS47L63_SPI_OPMODE, 0), \
		.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),   \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),       \
	};                                                                \
	DEVICE_DT_INST_DEFINE(inst, cs47l63_init, NULL, NULL,             \
			      &cs47l63_config_##inst, POST_KERNEL,        \
			      CONFIG_AUDIO_CODEC_INIT_PRIORITY,           \
			      &cs47l63_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CS47L63_INIT)
