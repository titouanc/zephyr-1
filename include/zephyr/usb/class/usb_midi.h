#ifndef ZEPHYR_INCLUDE_USB_CLASS_USB_MIDI_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USB_MIDI_H_

#include <zephyr/device.h>

// Universal MIDI Packet: 2.1.4 Message Type (MT) Allocation
#define MT_UTILITY             0x00
#define MT_SYS_RT_COMMON       0x01
#define MT_MIDI1_CHANNEL_VOICE 0x02
#define MT_DATA                0x03
#define MT_MIDI2_CHANNEL_VOICE 0x04
#define MT_FLEX_DATA           0x0d
#define MT_UMP_STREAM          0x0f

static inline size_t ump_words(uint8_t mt)
{
	return ((uint8_t[16]){1, 1, 1, 2, 2, 4, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4})[mt & 0x0f];
}

// Universal MIDI Packet: 7.3 MIDI 1.0 Channel Voice Messages
struct ump_midi1 {
#ifdef CONFIG_LITTLE_ENDIAN
	uint8_t p2;
	uint8_t p1;
	unsigned channel: 4;
	unsigned status: 4;
	unsigned group: 4;
	unsigned mt: 4;
#else
	unsigned mt: 4;
	unsigned group: 4;
	unsigned status: 4;
	unsigned channel: 4;
	uint8_t p1;
	uint8_t p2;
#endif
} __packed;

// midi20 3.2 Data Format: Universal MIDI Packet (UMP)
union ump {
	uint32_t words[4];
	struct {
#ifdef CONFIG_LITTLE_ENDIAN
		uint8_t data[3];
		unsigned group: 4;
		unsigned mt: 4;
#else
		unsigned mt: 4;
		unsigned group: 4;
		uint8_t data[3];
#endif
	} __packed;
	struct ump_midi1 midi1;
} __packed;

#define UMP_MIDI1(_group, _status, _channel, _p1, _p2)                                             \
	((union ump){.midi1 = {.mt = MT_MIDI1_CHANNEL_VOICE,                                       \
			       .group = _group,                                                    \
			       .status = _status,                                                  \
			       .channel = _channel,                                                \
			       .p1 = _p1,                                                          \
			       .p2 = _p2}})

#define MIDI_NOTE_OFF		0x8
#define MIDI_NOTE_ON		0x9
#define MIDI_AFTERTOUCH		0xa
#define MIDI_CONTROL_CHANGE	0xb
#define MIDI_PROGRAM_CHANGE	0xc
#define MIDI_CHAN_AFTERTOUCH	0xd
#define MIDI_PITCH_BEND		0xe

int usb_midi_send(const struct device *dev, union ump *pkt);

#endif
