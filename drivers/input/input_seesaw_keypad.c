/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adafruit_seesaw_keypad

#include <zephyr/input/input.h>
#include <zephyr/drivers/mfd/mfd_seesaw.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(seesaw_keypad, CONFIG_MFD_SEESAW_LOG_LEVEL);

enum {
	KEYPAD_STATUS = 0x00,
	KEYPAD_EVENT = 0x01,
	KEYPAD_INTENSET = 0x02,
	KEYPAD_INTENCLR = 0x03,
	KEYPAD_COUNT = 0x04,
	KEYPAD_FIFO = 0x10,
};

enum key_edge {
	EDGE_HIGH = 0,
	EDGE_LOW = 1,
	EDGE_FALLING = 2,
	EDGE_RISING = 3,
};

struct seesaw_keypad_config {
	const struct device *seesaw;
};

struct seesaw_keypad_data {
	const struct device *dev;
	struct k_work_delayable work;
};

static int seesaw_keypad_get_event(const struct device *dev)
{
	const struct seesaw_keypad_config *config = dev->config;
	uint8_t keypad_event;
	int ret = seesaw_read(config->seesaw, SEESAW_MOD_KEYPAD, KEYPAD_FIFO, &keypad_event, 1);

	if (ret < 0) {
		LOG_ERR("Error polling events FIFO: %d", ret);
		return ret;
	}

	if (keypad_event == 0xff) {
		/* Reached end of FIFO */
		return 0;
	}

	input_report(dev, INPUT_EV_KEY, (keypad_event >> 2), (keypad_event & 1), true, K_NO_WAIT);
	return 1;
}

static void seesaw_keypad_poll(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct seesaw_keypad_data *drv_data = CONTAINER_OF(dwork, struct seesaw_keypad_data, work);

	while (seesaw_keypad_get_event(drv_data->dev) > 0) {
		continue;
	}

	k_work_reschedule(dwork, K_MSEC(CONFIG_INPUT_SEESAW_KEYPAD_POLL_INTERVAL));
}

static int seesaw_keypad_init(const struct device *dev)
{
	int ret;
	struct seesaw_keypad_data *drv_data = dev->data;
	const struct seesaw_keypad_config *config = dev->config;
	/* Enable RISE/FALL events on all keys */
	uint8_t edges = BIT(EDGE_RISING) | BIT(EDGE_FALLING);

	ret = seesaw_claim_module(config->seesaw, SEESAW_MOD_KEYPAD, dev);
	if (ret) {
		return ret;
	}

	drv_data->dev = dev;
	k_work_init_delayable(&drv_data->work, seesaw_keypad_poll);

	for (int i = 0; i < 16; i++) {
		uint8_t buf[] = {
			(i & 0xc) << 1 | (i & 0x03),
			(edges << 1) | 1,
		};

		ret = seesaw_write(config->seesaw, SEESAW_MOD_KEYPAD, KEYPAD_EVENT,
				   buf, sizeof(buf));
		if (ret) {
			LOG_ERR("Unable to configure keypad event: %d", ret);
			return ret;
		}
	}

	k_work_reschedule(&drv_data->work, K_MSEC(CONFIG_INPUT_SEESAW_KEYPAD_POLL_INTERVAL));
	LOG_DBG("%s initialized", dev->name);

	return 0;
}

#define SEESAW_KEYPAD_INIT(inst)                                                                   \
	static struct seesaw_keypad_data data_##inst;                                              \
	static const struct seesaw_keypad_config config_##inst = {                                 \
		.seesaw = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, seesaw_keypad_init, NULL, &data_##inst, &config_##inst,        \
				  POST_KERNEL, CONFIG_MFD_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(SEESAW_KEYPAD_INIT)
