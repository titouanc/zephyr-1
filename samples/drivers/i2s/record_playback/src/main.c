/*
 * Copyright (c) 2026 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(record_playback);

#if DT_NODE_EXISTS(DT_NODELABEL(i2s_rxtx))
#define I2S_RX_NODE DT_NODELABEL(i2s_rxtx)
#define I2S_TX_NODE I2S_RX_NODE
#else
#define I2S_RX_NODE DT_NODELABEL(i2s_rx)
#define I2S_TX_NODE DT_NODELABEL(i2s_tx)
#endif

#define BYTES_PER_SAMPLE   sizeof(int16_t)
#define SAMPLE_BIT_WIDTH   (8 * BYTES_PER_SAMPLE)
#define NUMBER_OF_CHANNELS 2
#define TIMEOUT            1000

#define BLOCK_DURATION_MS  10
#define BLOCKS_PER_SECOND  (1000 / BLOCK_DURATION_MS)
#define SAMPLES_PER_BLOCK  (NUMBER_OF_CHANNELS * CONFIG_SAMPLE_FREQ / BLOCKS_PER_SECOND)
#define BLOCK_SIZE         (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define INITIAL_TX_BLOCKS  2
#define BLOCK_COUNT        (INITIAL_TX_BLOCKS + 2)

K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

#define RECORD_NUM_BLOCKS (BLOCKS_PER_SECOND * CONFIG_RECORD_SECONDS)
#define RECORD_BUFFER_SIZE (BLOCK_SIZE * RECORD_NUM_BLOCKS)
static uint8_t record_buffer[RECORD_NUM_BLOCKS][BLOCK_SIZE];

static bool record(const struct device *i2s_dev, const struct i2s_config *config)
{
	int ret;

	ret = i2s_configure(i2s_dev, I2S_DIR_RX, config);
	if (ret < 0) {
		LOG_ERR("Failed to configure RX stream: %d", ret);
		return false;
	}

	ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("Failed to start RX stream: %d", ret);
		return false;
	}

	for (size_t i=0; i<RECORD_NUM_BLOCKS; i++) {
		size_t read_size = BLOCK_SIZE;

		ret = i2s_buf_read(i2s_dev, record_buffer[i], &read_size);
		if (ret != 0) {
			LOG_ERR("Failed to I2S read: %d", ret);
			i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
			return false;
		}
	}

	ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
	if (ret < 0) {
		LOG_ERR("Failed to stop RX stream: %d", ret);
		return false;
	}

	return true;
}

static bool replay(const struct device *i2s_dev, const struct i2s_config *config)
{
	int ret;

	ret = i2s_configure(i2s_dev, I2S_DIR_TX, config);
	if (ret < 0) {
		LOG_ERR("Failed to configure TX stream: %d", ret);
		return false;
	}

	size_t i = 0;

	for (i=0; i < INITIAL_TX_BLOCKS; i++) {
		ret = i2s_buf_write(i2s_dev, record_buffer[i], BLOCK_SIZE);
		if (ret < 0) {
			LOG_ERR("Failed to queue initial TX block %d: %d", i, ret);
			return false;
		}
	}

	ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("Failed to start TX stream: %d", ret);
		return false;
	}

	for (; i < RECORD_NUM_BLOCKS; i++) {
		ret = i2s_buf_write(i2s_dev, record_buffer[i], BLOCK_SIZE);
		if (ret < 0) {
			LOG_ERR("Failed to write TX block %d: %d", i, ret);
			i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
			return false;
		}
	}

	ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		LOG_ERR("Failed to drain TX stream: %d", ret);
		return false;
	}

	/*
	 * i2s_trigger(DRAIN) returns as soon as the request is queued, not
	 * once playback actually completes, and the I2S API exposes no way
	 * to poll for that. Sleep for the time still buffered downstream so
	 * the peripheral is not reconfigured for RX while it is still
	 * shifting out the last blocks.
	 */
	k_msleep(1000);

	return true;
}

int main(void)
{
	const struct device *const i2s_dev_rx = DEVICE_DT_GET(I2S_RX_NODE);
	const struct device *const i2s_dev_tx = DEVICE_DT_GET(I2S_TX_NODE);
	const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
	struct i2s_config config = {
		.word_size = SAMPLE_BIT_WIDTH,
		.channels = NUMBER_OF_CHANNELS,
		.format = I2S_FMT_DATA_FORMAT_I2S,
#if IS_ENABLED(CONFIG_USE_CODEC_CLOCK)
		.options = I2S_OPT_BIT_CLK_TARGET | I2S_OPT_FRAME_CLK_TARGET,
#else
		.options = I2S_OPT_BIT_CLK_CONTROLLER | I2S_OPT_FRAME_CLK_CONTROLLER,
#endif
		.frame_clk_freq = CONFIG_SAMPLE_FREQ,
		.mem_slab = &mem_slab,
		.block_size = BLOCK_SIZE,
		.timeout = TIMEOUT,
	};
	struct audio_codec_cfg audio_cfg = {
		.dai_route = AUDIO_ROUTE_PLAYBACK_CAPTURE,
		.dai_type = AUDIO_DAI_TYPE_I2S,
		.dai_cfg.i2s = config,
#if IS_ENABLED(CONFIG_USE_CODEC_CLOCK)
		.dai_cfg.i2s.options = I2S_OPT_BIT_CLK_CONTROLLER | I2S_OPT_FRAME_CLK_CONTROLLER,
#else
		.dai_cfg.i2s.options = I2S_OPT_BIT_CLK_TARGET | I2S_OPT_FRAME_CLK_TARGET,
#endif
	};

	LOG_INF("I2S record/playback sample starting");

	if (!device_is_ready(i2s_dev_rx)) {
		LOG_ERR("%s is not ready", i2s_dev_rx->name);
		return 0;
	}

	if (i2s_dev_rx != i2s_dev_tx && !device_is_ready(i2s_dev_tx)) {
		LOG_ERR("%s is not ready", i2s_dev_tx->name);
		return 0;
	}

	if (!device_is_ready(codec_dev)) {
		LOG_ERR("%s is not ready", codec_dev->name);
		return 0;
	}

	audio_codec_configure(codec_dev, &audio_cfg);
	k_msleep(1000);

	while (1) {
		LOG_INF("Recording %d seconds...", CONFIG_RECORD_SECONDS);
		if (!record(i2s_dev_rx, &config)) {
			return 0;
		}

		LOG_INF("Replaying %d seconds...", CONFIG_RECORD_SECONDS);
		if (!replay(i2s_dev_tx, &config)) {
			return 0;
		}
	}
}
