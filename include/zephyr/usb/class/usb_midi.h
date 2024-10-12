#ifndef ZEPHYR_INCLUDE_USB_CLASS_USB_MIDI_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USB_MIDI_H_

#include <zephyr/device.h>

struct usb_midi_packet {
	unsigned cableNum: 4;
	unsigned codeIdxNum: 4;
	uint8_t midi[3];
} __packed;

int usb_midi_send(const struct device *dev, struct usb_midi_packet pkt);

#define MIDI_PKT(cable, cmd, chan, p1, p2)                                                         \
	(struct usb_midi_packet)                                                                   \
	{                                                                                          \
		.cableNum = cable, .codeIdxNum = cmd, .midi = {(cmd << 4) | chan, p1, p2 }         \
	}

#define MIDI_NOTE_ON(cable, chan, note, velocity) MIDI_PKT(cable, 0x9, chan, note, velocity)

#define MIDI_NOTE_OFF(cable, chan, note, velocity) MIDI_PKT(cable, 0x8, chan, note, velocity)

#define MIDI_CONTROL_CHANGE(cable, chan, cc, value) MIDI_PKT(cable, 0xB, chan, cc, value)

#define MIDI_PROGRAM_CHANGE(cable, chan, program) MIDI_PKT(cable, 0xC, chan, program, 0)

#endif
