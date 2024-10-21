/*
 * Copyright (c) 2024 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample loopback app for USB-MIDI device class
 */

#include <sample_usbd.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/usb/class/usb_midi.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sample_usb_midi, LOG_LEVEL_INF);

static const struct device *const midi = DEVICE_DT_GET(DT_NODELABEL(usb_midi));

static void key_press(struct input_event *evt, void *user_data)
{
	if (!device_is_ready(midi)) {
		LOG_ERR("MIDI device not ready");
	}

	uint8_t command = evt->value ? MIDI_NOTE_ON : MIDI_NOTE_OFF;
	usb_midi_send(midi, &UMP_MIDI1(0, command, 0, 0x40, 0x7f));
}
INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(user_button))), key_press, NULL);

int main(void)
{
	if (!device_is_ready(midi)) {
		LOG_ERR("MIDI device not ready");
		return -1;
	}

	struct usbd_context *sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -1;
	}

	if (!usbd_can_detect_vbus(sample_usbd) && usbd_enable(sample_usbd)) {
		LOG_ERR("Failed to enable device support");
		return -1;
	}

	LOG_INF("USB device support enabled");
	return 0;
}
