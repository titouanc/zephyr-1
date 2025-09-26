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
LOG_MODULE_REGISTER(cirrus_cs47l63);

#define DT_DRV_COMPAT cirrus_cs47l63

struct cs47l63_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec irq_gpio;
};

#define LOG_REG(spi, reg) \
	do {uint32_t v; \
	    cs47l63_read_spi_regs(spi, reg, &v, 1); \
	    LOG_INF("%20s (%4X) = %08X", &(#reg)[12], reg, v);} while (0)

/**
 * @brief      Read 32b registers from the CS47L63
 * @param[in]  spi        The spi spec onto which the transfer happens
 * @param[in]  start_reg  The address of the first register to read from
 * @param      values     Where to store the register values.
 *                        The resulting values are in native CPU endianness
 * @param[in]  count      The number of 32b registers to read
 * @return     same as spi_transceive
 */
static int cs47l63_read_spi_regs(const struct spi_dt_spec *spi,
				 uint32_t start_reg, uint32_t *values, size_t count)
{
	int ret;
	/* Register address + READ bit set */
	uint32_t addr = sys_cpu_to_be32(BIT(31) | start_reg);
	struct spi_buf tx_buf = {.buf = (void*) &addr, .len = sizeof(addr)};
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

	for (size_t i=0; i<count; i++) {
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

static int cs47l63_configure(const struct device *dev, struct audio_codec_cfg *audiocfg)
{
	int ret;
	uint8_t format, wordsize = audiocfg->dai_cfg.i2s.word_size;
	const struct cs47l63_config *cfg = dev->config;

	if (audiocfg->dai_route != AUDIO_ROUTE_PLAYBACK) {
		return -ENOTSUP;
	}

	if (wordsize < 16 || wordsize > 32) {
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
		return -ENOTSUP;
	}

	ret = cs47l63_write_spi_reg(&cfg->spi, CS47L63_REG_ASP1_CONTROL2,
				    (wordsize << 24) | (format << 8));
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
	return 0;
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

static int apply_titou_config(const struct spi_dt_spec *spi)
{
	// Set GPIO1-4 to pin-specific alternate function (ASP1)
	uint8_t zeros[16];
	memset(zeros, 0, sizeof(zeros));
	cs47l63_write_spi_blob(spi, CS47L63_REG_GPIO1_CTRL1, zeros, sizeof(zeros));

	// Set GPIO10 to alternate function "Output signal path status"
	cs47l63_write_spi_reg(spi, CS47L63_REG_GPIO10_CTRL1, 0x1FA);

	// 16 BCLK cycles per slot, I2S format
	cs47l63_write_spi_reg(spi, CS47L63_REG_ASP1_CONTROL2,
		              (16 << 24) | (ASP_FMT_I2S << 8));

	// Rx: Channel 1 on slot 0, channel 2 on slot 1
	cs47l63_write_spi_reg(spi, CS47L63_REG_ASP1_FRAME_CONTROL5, (0x01 << 8));

	// 16 valid data bits per Rx slot
	cs47l63_write_spi_reg(spi, CS47L63_REG_ASP1_DATA_CONTROL5, 16);

	// Enable Rx chan 1&2 from ASP1
	cs47l63_write_spi_reg(spi, CS47L63_REG_ASP1_ENABLES1, BIT(16) | BIT(17));

	// Route to output
	cs47l63_write_spi_reg(spi, CS47L63_REG_OUT1L_INPUT1, MIXER_SRC_ASP1_RX1);
	cs47l63_write_spi_reg(spi, CS47L63_REG_OUT1L_INPUT2, MIXER_SRC_ASP1_RX2);

	// Enable output
	cs47l63_write_spi_reg(spi, CS47L63_REG_OUTPUT_ENABLE_1, BIT(1));

	// Unmute, set to 0dB
	cs47l63_write_spi_reg(spi, CS47L63_REG_OUT1L_VOLUME_1, 0x280);

	// Set to 16kHz sample rate
	cs47l63_write_spi_reg(spi, CS47L63_REG_SAMPLE_RATE1, SAMPLE_RATE_16_kHz);

	// System clock enabled & derived from ASP1_BCLK
	uint32_t current_system_clock;
	cs47l63_read_spi_regs(spi, CS47L63_REG_SYSTEM_CLOCK1, &current_system_clock, 1);
	cs47l63_write_spi_reg(spi, CS47L63_REG_SYSTEM_CLOCK1, BIT(6) | current_system_clock);

	LOG_INF("Titou's config applied !");

	return 0;
}

static void print_all_regs(const struct spi_dt_spec *spi)
{
	LOG_INF("--------------------------------------");
	LOG_REG(spi, CS47L63_REG_DEVID);
	LOG_REG(spi, CS47L63_REG_REVID);
	LOG_REG(spi, CS47L63_REG_GPIO_STATUS1);
	LOG_REG(spi, CS47L63_REG_GPIO1_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO2_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO3_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO4_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO5_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO6_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO7_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO8_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO9_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO10_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO11_CTRL1);
	LOG_REG(spi, CS47L63_REG_GPIO12_CTRL1);
	LOG_REG(spi, CS47L63_REG_CLKGEN_PAD_CTRL);
	LOG_REG(spi, CS47L63_REG_CLOCK32K);
	LOG_REG(spi, CS47L63_REG_SYSTEM_CLOCK1);
	LOG_REG(spi, CS47L63_REG_SYSTEM_CLOCK2);
	LOG_REG(spi, CS47L63_REG_SAMPLE_RATE1);
	LOG_REG(spi, CS47L63_REG_SAMPLE_RATE2);
	LOG_REG(spi, CS47L63_REG_SAMPLE_RATE3);
	LOG_REG(spi, CS47L63_REG_SAMPLE_RATE4);
	LOG_REG(spi, CS47L63_REG_SAMPLE_RATE_STATUS1);
	LOG_REG(spi, CS47L63_REG_SAMPLE_RATE_STATUS2);
	LOG_REG(spi, CS47L63_REG_SAMPLE_RATE_STATUS3);
	LOG_REG(spi, CS47L63_REG_SAMPLE_RATE_STATUS4);
	LOG_REG(spi, CS47L63_REG_OUTPUT_ENABLE_1);
	LOG_REG(spi, CS47L63_REG_OUTPUT_STATUS_1);
	LOG_REG(spi, CS47L63_REG_OUTPUT_CONTROL_1);
	LOG_REG(spi, CS47L63_REG_OUT1L_VOLUME_1);
	LOG_REG(spi, CS47L63_REG_OUT1L_CONTROL_1);
	LOG_REG(spi, CS47L63_REG_ASP1_ENABLES1);
	LOG_REG(spi, CS47L63_REG_ASP1_CONTROL1);
	LOG_REG(spi, CS47L63_REG_ASP1_CONTROL2);
	LOG_REG(spi, CS47L63_REG_ASP1_CONTROL3);
	LOG_REG(spi, CS47L63_REG_ASP1_FRAME_CONTROL1);
	LOG_REG(spi, CS47L63_REG_ASP1_FRAME_CONTROL2);
	LOG_REG(spi, CS47L63_REG_ASP1_FRAME_CONTROL5);
	LOG_REG(spi, CS47L63_REG_ASP1_FRAME_CONTROL6);
	LOG_REG(spi, CS47L63_REG_ASP1_DATA_CONTROL1);
	LOG_REG(spi, CS47L63_REG_ASP1_DATA_CONTROL5);
	LOG_REG(spi, CS47L63_REG_OUT1L_INPUT1);
	LOG_REG(spi, CS47L63_REG_OUT1L_INPUT2);
	LOG_REG(spi, CS47L63_REG_OUT1L_INPUT3);
	LOG_REG(spi, CS47L63_REG_OUT1L_INPUT4);
	LOG_INF("======================================");
}

static int cs47l63_init(const struct device *dev)
{
	const struct cs47l63_config *cfg = dev->config;
	uint32_t devid[2];
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
	while (gpio_pin_get_dt(&cfg->irq_gpio));

	/* Release RESET state */
	ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
	if (ret != 0) {
		LOG_ERR("Unable to complete hw reset");
		return -EIO;
	}

	/* Wait for device to be ready */
	while (!gpio_pin_get_dt(&cfg->irq_gpio));

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

	/* ================================ */
	print_all_regs(&cfg->spi);
	ret = apply_titou_config(&cfg->spi);
	if (ret != 0) {
		return -EIO;
	}
	print_all_regs(&cfg->spi);
	/* ================================ */

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
