/*
 * Copyright (c) 2024 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file
 * @brief Sample application for USB MIDI 2.0 device class
 */

#include <sample_usbd.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/usb/class/usb_midi.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sample_usb_midi, LOG_LEVEL_INF);

static const struct device *const midi = DEVICE_DT_GET(DT_NODELABEL(usb_midi));

/* On boards that have a user button; button press/release -> MIDI note on/off */
#if DT_NODE_EXISTS(DT_NODELABEL(user_button))
static const struct device *const button = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(user_button)));

static void key_press(struct input_event *evt, void *user_data)
{
	if (!device_is_ready(midi)) {
		LOG_ERR("MIDI device not ready");
	}

	uint8_t command = evt->value ? MIDI_NOTE_ON : MIDI_NOTE_OFF;
	usb_midi_send(midi, &UMP_MIDI1(0, command, 0, 0x40, 0x7f));
}
INPUT_CALLBACK_DEFINE(button, key_press, NULL);
#endif

static void on_midi_packet(const struct device *dev, const union ump *midi_packet)
{
	LOG_INF("Received MIDI packet (MT=%X)", midi_packet->mt);
	/* Only send MIDI1 channel voice messages back to the host */
	if (midi_packet->mt == MT_MIDI1_CHANNEL_VOICE) {
		const struct ump_midi1 *midi1 = &midi_packet->midi1;
		LOG_INF("Send back MIDI1 message chan=%d status=%d %02X %02X", midi1->channel,
			midi1->status, midi1->p1, midi1->p2);
		usb_midi_send(dev, midi_packet);
	}
}

int main(void)
{
	if (!device_is_ready(midi)) {
		LOG_ERR("MIDI device not ready");
		return -1;
	}

	usb_midi_set_callback(midi, on_midi_packet);

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
