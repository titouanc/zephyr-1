/*
 * Copyright (c) 2024 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/led.h>
#include <zephyr/input/input.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

const struct device *neopixel = DEVICE_DT_GET(DT_NODELABEL(neotrellis_neopixel));

static void input_cb(struct input_event *evt, void *user_data)
{
	size_t led_num = ((evt->code >> 1) & 0xc) | (evt->code & 0x3);
	if (evt->value) {
		led_on(neopixel, led_num);
	} else {
		led_off(neopixel, led_num);
	}
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(neotrellis_keypad)), input_cb, NULL);

int main()
{
	return 0;
}
