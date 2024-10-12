/*
 * Copyright (c) 2024 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
TODO:
- Handle multiple terminals IN/OUT from DT
- Proper delivery/send to/from "userspace"
- USB-MIDI2 alternate config

usbd config add FS1
usbd class register midi_0 fs 1
usbd init
usbd enable
*/

#define DT_DRV_COMPAT zephyr_usb_midi

#include <zephyr/drivers/usb/udc.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/class/usb_midi.h>
#include <zephyr/usb/usbd.h>

#include "usbd_uac2_macros.h"

LOG_MODULE_REGISTER(usbd_midi, CONFIG_USBD_MIDI_LOG_LEVEL);

UDC_BUF_POOL_DEFINE(usb_midi_buf_pool, DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 4, 64,
		    sizeof(struct udc_buf_info), NULL);

#define MIDI_QUEUE_SIZE 64

#define ENDPOINT_FROM_HOST 0x01
#define ENDPOINT_TO_HOST   0x81

// See midi10 Appendix A. Audio Device Class Codes: MIDIStreaming
// A.1 MS Class-Specific Interface Descriptor Subtypes
#define MS_DESCRIPTOR_UNDEFINED 0x00
#define MS_HEADER               0x01
#define MIDI_IN_JACK            0x02
#define MIDI_OUT_JACK           0x03
#define ELEMENT                 0x04

// A.3 MS MIDI IN and OUT Jack types
#define JACK_TYPE_UNDEFINED 0x00
#define JACK_EMBEDDED       0x01
#define JACK_EXTERNAL       0x02

struct usbd_midi_cs_ac_header {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint16_t bcdADC;
	uint16_t wTotalLenth;
	uint8_t bInCollection;
	uint8_t baInterfaceNr;
} __packed;

struct usbd_midi_header_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint16_t bcdMSC;
	uint16_t wTotalLenth;
} __packed;

struct usbd_midi_jackin_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bJackType;
	uint8_t bJackId;
	uint8_t iJack;
} __packed;

struct usbd_midi_jackout_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bJackType;
	uint8_t bJackId;
	uint8_t bNrInputPins;
	uint8_t baSourceID;
	uint8_t BaSourcePin;
	uint8_t iJack;
} __packed;

struct usbd_midi_io_desc {
	struct usbd_midi_jackin_descriptor in;
	struct usbd_midi_jackout_descriptor out;
} __packed;

#define USBD_MIDI_IF_DESCRIPTOR_T(n_inputs, n_outputs)                                             \
	struct {                                                                                   \
		struct usb_if_descriptor if0;                                                      \
		struct usbd_midi_cs_ac_header if0_header;                                          \
		struct usb_if_descriptor if1;                                                      \
		struct usbd_midi_header_descriptor if1_header;                                     \
		struct usbd_midi_io_desc outputs[n_outputs];                                       \
		struct usbd_midi_io_desc inputs[n_inputs];                                         \
		struct usb_ep_descriptor if1_out_ep;                                               \
		struct {                                                                           \
			uint8_t bLength;                                                           \
			uint8_t bDescriptorType;                                                   \
			uint8_t bDescriptorSubtype;                                                \
			uint8_t bNumEmbMIDIJack;                                                   \
			uint8_t baAssocJackID[n_inputs];                                           \
		} __packed if1_cs_out_ep;                                                          \
		struct usb_ep_descriptor if1_in_ep;                                                \
		struct {                                                                           \
			uint8_t bLength;                                                           \
			uint8_t bDescriptorType;                                                   \
			uint8_t bDescriptorSubtype;                                                \
			uint8_t bNumEmbMIDIJack;                                                   \
			uint8_t baAssocJackID[n_inputs];                                           \
		} __packed if1_cs_in_ep;                                                           \
	} __packed

struct usbd_midi_data {
	/* Pointer to the associated USBD class node */
	struct usbd_class_data *class_data;
	/* Pointer to the class interface descriptors */
	size_t n_desc;
	struct usb_desc_header const **descs;
	struct k_work rx_work;
	struct k_work tx_work;
	uint8_t tx_queue_buf[MIDI_QUEUE_SIZE];
	struct ring_buf tx_queue;
};

static int usbd_midi_class_init(struct usbd_class_data *const class_data)
{
	const struct device *dev = usbd_class_get_private(class_data);
	LOG_DBG("Init USB-MIDI device class for %s", dev->name);
	return 0;
}

static void *usbd_midi_class_get_desc(struct usbd_class_data *const class_data,
				      const enum usbd_speed speed)
{
	const struct device *dev = usbd_class_get_private(class_data);
	struct usbd_midi_data *data = dev->data;
	LOG_DBG("Get descriptors for %s", dev->name);
	return data->descs;
}

static int usbd_midi_class_request(struct usbd_class_data *const class_data, struct net_buf *buf,
				   int err)
{
	struct usbd_context *uds_ctx = usbd_class_get_ctx(class_data);
	const struct device *dev = usbd_class_get_private(class_data);
	struct usbd_midi_data *data = dev->data;
	struct udc_buf_info *info = udc_get_buf_info(buf);
	LOG_DBG("USB-MIDI request for %s len=%d", dev->name, buf->len);

	if (err) {
		LOG_ERR("Class request error: %d", err);
	} else if (info->ep == ENDPOINT_FROM_HOST) {
		LOG_HEXDUMP_DBG(buf->data, buf->len, "Rx DATA");
		k_work_submit(&data->rx_work);
		usb_midi_send(dev, MIDI_NOTE_ON(0, 0, 64, 127));
	} else if (info->ep == ENDPOINT_TO_HOST) {
		LOG_HEXDUMP_DBG(buf->data, buf->len, "Tx DATA complete");
		if (ring_buf_size_get(&data->tx_queue)) {
			k_work_submit(&data->tx_work);
		}
	}

	return usbd_ep_buf_free(uds_ctx, buf);
}

static void usbd_midi_class_update(struct usbd_class_data *const class_data, uint8_t iface,
				   uint8_t alternate)
{
	const struct device *dev = usbd_class_get_private(class_data);
	LOG_DBG("USB-MIDI update for %s: if=%d, alt=%d", dev->name, (int)iface, (int)alternate);
}

static void usbd_midi_class_enable(struct usbd_class_data *const class_data)
{
	const struct device *dev = usbd_class_get_private(class_data);
	struct usbd_midi_data *data = dev->data;
	LOG_DBG("USB-MIDI enable for %s", dev->name);
	k_work_submit(&data->rx_work);
}

static void usbd_midi_class_disable(struct usbd_class_data *const class_data)
{
	const struct device *dev = usbd_class_get_private(class_data);
	struct usbd_midi_data *data = dev->data;
	LOG_DBG("USB-MIDI disable for %s", dev->name);
	k_work_cancel(&data->rx_work);
}

static void usbd_midi_class_suspended(struct usbd_class_data *const class_data)
{
	const struct device *dev = usbd_class_get_private(class_data);
	struct usbd_midi_data *data = dev->data;
	LOG_DBG("USB-MIDI suspended for %s", dev->name);
	k_work_cancel(&data->rx_work);
}

static void usbd_midi_class_resumed(struct usbd_class_data *const class_data)
{
	const struct device *dev = usbd_class_get_private(class_data);
	struct usbd_midi_data *data = dev->data;
	LOG_DBG("USB-MIDI resumed for %s", dev->name);
	k_work_submit(&data->rx_work);
}

static int usbd_midi_class_cth(struct usbd_class_data *const class_data,
			       const struct usb_setup_packet *const setup,
			       struct net_buf *const buf)
{
	const struct device *dev = usbd_class_get_private(class_data);
	LOG_DBG("USB-MIDI control to host for %s", dev->name);
}

static int usbd_midi_class_ctd(struct usbd_class_data *const class_data,
			       const struct usb_setup_packet *const setup,
			       const struct net_buf *const buf)
{
	const struct device *dev = usbd_class_get_private(class_data);
	LOG_DBG("USB-MIDI control to device for %s", dev->name);
}

static struct usbd_class_api usbd_midi_class_api = {
	.request = usbd_midi_class_request,
	.update = usbd_midi_class_update,
	.enable = usbd_midi_class_enable,
	.disable = usbd_midi_class_disable,
	.suspended = usbd_midi_class_suspended,
	.resumed = usbd_midi_class_resumed,
	.control_to_host = usbd_midi_class_cth,
	.control_to_dev = usbd_midi_class_ctd,
	.init = usbd_midi_class_init,
	.get_desc = usbd_midi_class_get_desc,
};

static struct net_buf *usbd_midi_buf_alloc(uint8_t ep)
{
	struct net_buf *buf = net_buf_alloc(&usb_midi_buf_pool, K_NO_WAIT);
	if (!buf) {
		return NULL;
	}

	struct udc_buf_info *info = udc_get_buf_info(buf);
	memset(info, 0, sizeof(struct udc_buf_info));
	info->ep = ep;
	return buf;
}

static void usbd_midi_rx_work(struct k_work *work)
{
	struct usbd_midi_data *data = CONTAINER_OF(work, struct usbd_midi_data, rx_work);
	struct net_buf *buf = usbd_midi_buf_alloc(ENDPOINT_FROM_HOST);
	if (buf == NULL) {
		return;
	}

	int r = usbd_ep_enqueue(data->class_data, buf);
	if (r) {
		LOG_ERR("Failed to enqueue Rx net_buf -> %d", r);
		net_buf_unref(buf);
	}
}

static void usbd_midi_tx_work(struct k_work *work)
{
	struct usbd_midi_data *data = CONTAINER_OF(work, struct usbd_midi_data, tx_work);
	struct net_buf *buf = usbd_midi_buf_alloc(ENDPOINT_TO_HOST);
	if (buf == NULL) {
		LOG_ERR("Unable to allocate Tx net_buf");
		return;
	}

	net_buf_add(buf, ring_buf_get(&data->tx_queue, buf->data, buf->size));

	int r = usbd_ep_enqueue(data->class_data, buf);
	if (r) {
		LOG_ERR("Failed to enqueue Tx net_buf -> %d", r);
		net_buf_unref(buf);
	}
}

static int usbd_midi_preinit(const struct device *dev)
{
	struct usbd_midi_data *data = dev->data;
	k_work_init(&data->rx_work, usbd_midi_rx_work);
	k_work_init(&data->tx_work, usbd_midi_tx_work);
	ring_buf_init(&data->tx_queue, MIDI_QUEUE_SIZE, data->tx_queue_buf);
	LOG_DBG("Init USB-MIDI device %s", dev->name);

	return 0;
}

static inline int write_packet(struct ring_buf *rbuf, const struct usb_midi_packet *pkt)
{
	uint8_t *buf = NULL;
	int r = ring_buf_put_claim(rbuf, &buf, 4);
	if (r != 4) {
		ring_buf_put_finish(rbuf, 0);
		return -EAGAIN;
	}

	buf[0] = (pkt->cableNum << 4) | pkt->codeIdxNum;
	memcpy(&buf[1], pkt->midi, 3);

	ring_buf_put_finish(rbuf, 4);
	return 0;
}

static inline int get_packet(struct ring_buf *rbuf, struct usb_midi_packet *pkt)
{
	uint8_t *buf = NULL;
	int r = ring_buf_get_claim(rbuf, &buf, 4);
	if (r != 4) {
		ring_buf_get_finish(rbuf, 0);
		return -EAGAIN;
	}

	pkt->cableNum = buf[0] >> 4;
	pkt->codeIdxNum = buf[0] & 0x0f;
	memcpy(pkt->midi, &buf[1], 3);

	ring_buf_get_finish(rbuf, 4);
	return 0;
}

int usb_midi_send(const struct device *dev, struct usb_midi_packet pkt)
{
	struct usbd_midi_data *data = dev->data;
	LOG_DBG("%s send %02X %02X %02X", dev->name, pkt.midi[0], pkt.midi[1], pkt.midi[2]);
	if (write_packet(&data->tx_queue, &pkt) == 0) {
		k_work_submit(&data->tx_work);
	} else {
		LOG_ERR("Unable to write");
	}
	return 0;
}

#define USBD_MIDI_INPUT_DESCRIPTOR(input_no)                                                       \
	{                                                                                          \
		.in =                                                                              \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_jackin_descriptor),             \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MIDI_IN_JACK,                                \
				.bJackType = JACK_EXTERNAL,                                        \
				.bJackId = input_no,                                               \
			},                                                                         \
		.out =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_jackout_descriptor),            \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MIDI_OUT_JACK,                               \
				.bJackType = JACK_EMBEDDED,                                        \
				.bJackId = input_no,                                               \
				.bNrInputPins = 1,                                                 \
				.baSourceID = input_no,                                            \
				.BaSourcePin = 1,                                                  \
			},                                                                         \
	}

#define USBD_MIDI_OUTPUT_DESCRIPTOR(output_no)                                                     \
	{                                                                                          \
		.in =                                                                              \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_jackin_descriptor),             \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MIDI_IN_JACK,                                \
				.bJackType = JACK_EMBEDDED,                                        \
				.bJackId = output_no,                                              \
			},                                                                         \
		.out =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_jackout_descriptor),            \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MIDI_OUT_JACK,                               \
				.bJackType = JACK_EXTERNAL,                                        \
				.bJackId = output_no,                                              \
				.bNrInputPins = 1,                                                 \
				.baSourceID = output_no,                                           \
				.BaSourcePin = 1,                                                  \
			},                                                                         \
	}

#define USBD_MIDI_DEFINE_DESCRIPTOR(inst)                                                          \
	static USBD_MIDI_IF_DESCRIPTOR_T(1, 1) usbd_midi_desc_##inst = {                           \
		.if0 =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usb_if_descriptor),                       \
				.bDescriptorType = USB_DESC_INTERFACE,                             \
				.bInterfaceNumber = 0,                                             \
				.bAlternateSetting = 0,                                            \
				.bNumEndpoints = 0,                                                \
				.bInterfaceClass = AUDIO,                                          \
				.bInterfaceSubClass = AUDIOCONTROL,                                \
			},                                                                         \
		.if0_header =                                                                      \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_cs_ac_header),                  \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MS_HEADER,                                   \
				.bcdADC = sys_cpu_to_le16(0x0100),                                 \
				.wTotalLenth = sizeof(struct usbd_midi_cs_ac_header),              \
				.bInCollection = 1,                                                \
				.baInterfaceNr = 1,                                                \
			},                                                                         \
		.if1 =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usb_if_descriptor),                       \
				.bDescriptorType = USB_DESC_INTERFACE,                             \
				.bInterfaceNumber = 1,                                             \
				.bAlternateSetting = 0,                                            \
				.bNumEndpoints = 2,                                                \
				.bInterfaceClass = AUDIO,                                          \
				.bInterfaceSubClass = MIDISTREAMING,                               \
			},                                                                         \
		.if1_header =                                                                      \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_header_descriptor),             \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MS_HEADER,                                   \
				.bcdMSC = sys_cpu_to_le16(0x0100),                                 \
				.wTotalLenth =                                                     \
					sys_cpu_to_le16(sizeof(USBD_MIDI_IF_DESCRIPTOR_T(1, 1)) -  \
							sizeof(struct usb_if_descriptor)),         \
			},                                                                         \
		.inputs = {USBD_MIDI_INPUT_DESCRIPTOR(1)},                                         \
		.outputs = {USBD_MIDI_INPUT_DESCRIPTOR(2)},                                        \
		.if1_in_ep =                                                                       \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = FIRST_IN_EP_ADDR,                              \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64U),                            \
			},                                                                         \
		.if1_cs_in_ep =                                                                    \
			{                                                                          \
				.bLength = 4 + 1,                                                  \
				.bDescriptorType = CS_ENDPOINT,                                    \
				.bDescriptorSubtype = EP_GENERAL,                                  \
				.bNumEmbMIDIJack = 1,                                              \
				.baAssocJackID = {1},                                              \
			},                                                                         \
		.if1_out_ep =                                                                      \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = FIRST_OUT_EP_ADDR,                             \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64U),                            \
			},                                                                         \
		.if1_cs_out_ep =                                                                   \
			{                                                                          \
				.bLength = 4 + 1,                                                  \
				.bDescriptorType = CS_ENDPOINT,                                    \
				.bDescriptorSubtype = EP_GENERAL,                                  \
				.bNumEmbMIDIJack = 1,                                              \
				.baAssocJackID = {2},                                              \
			},                                                                         \
	};                                                                                         \
	static const struct usb_desc_header *usbd_midi_desc_array_##inst[] = {                     \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if0,                              \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if0_header,                       \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1,                              \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_header,                       \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.inputs[0].in,                     \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.inputs[0].out,                    \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.outputs[0].in,                    \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.outputs[0].out,                   \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_out_ep,                       \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_cs_out_ep,                    \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_in_ep,                        \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_cs_in_ep,                     \
	};

#define USBD_MIDI_DT_DEVICE_DEFINE(inst)                                                           \
	USBD_MIDI_DEFINE_DESCRIPTOR(inst);                                                         \
	USBD_DEFINE_CLASS(midi_##inst, &usbd_midi_class_api, DEVICE_DT_GET(DT_DRV_INST(inst)),     \
			  NULL);                                                                   \
	static struct usbd_midi_data usbd_midi_data_##n = {                                        \
		.class_data = &midi_##inst,                                                        \
		.n_desc = ARRAY_SIZE(usbd_midi_desc_array_##inst),                                 \
		.descs = usbd_midi_desc_array_##inst,                                              \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, usbd_midi_preinit, NULL, &usbd_midi_data_##n, NULL,            \
			      POST_KERNEL, CONFIG_SERIAL_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(USBD_MIDI_DT_DEVICE_DEFINE)
