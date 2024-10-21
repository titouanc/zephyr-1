/*
 * Copyright (c) 2024 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
TODO:
- Handle multiple terminals IN/OUT from DT
    - Inputs/Outputs naming with string descriptor
- Proper delivery/send to/from "userspace"


Test with sample "USB Shell":
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

#define ENDPOINT_FROM_HOST 0x01
#define ENDPOINT_TO_HOST   0x81
#define MIDI_QUEUE_SIZE    64

// midi20 A.1 MS Class-Specific Interface Descriptor Subtypes
#define MS_DESCRIPTOR_UNDEFINED 0x00
#define MS_HEADER               0x01
#define MIDI_IN_JACK            0x02
#define MIDI_OUT_JACK           0x03
#define ELEMENT                 0x04

// midi20 A.3 MS Class-Specific Group Terminal Block Descriptor Subtypes
#define GR_TRM_BLOCK_UNDEFINED 0x00
#define GR_TRM_BLOCK_HEADER    0x01
#define GR_TRM_BLOCK           0x02

// midi20 A.5 MS MIDI IN and OUT Jack types
#define JACK_TYPE_UNDEFINED 0x00
#define JACK_EMBEDDED       0x01
#define JACK_EXTERNAL       0x02

// midi20 A.6 Group Terminal Block Type
#define GR_TRM_BIDIRECTIONAL 0x00
#define GR_TRM_INPUT_ONLY    0x01
#define GR_TRM_OUTPUT_ONLY   0x02

// midi20 A.7 Group Terminal Default MIDI Protocol
#define USE_MIDI_CI                      0x00
#define MIDI_1_0_UP_TO_64_BITS           0x01
#define MIDI_1_0_UP_TO_64_BITS_AND_JRTS  0x02
#define MIDI_1_0_UP_TO_128_BITS          0x03
#define MIDI_1_0_UP_TO_128_BITS_AND_JRTS 0x04
#define MIDI_2_0                         0x11
#define MIDI_2_0_AND_JRTS                0x12

#define ALT_USB_MIDI_1 0x00
#define ALT_USB_MIDI_2 0x01

// audio10 4.3.2 Class-Specific AC Interface Descriptor
struct usbd_midi_cs_ac_header_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint16_t bcdADC;
	uint16_t wTotalLength;
	uint8_t bInCollection;
	uint8_t baInterfaceNr;
} __packed;

// midi10 6.1.2.1 Class-Specific MS Interface Header Descriptor
struct usbd_midi_header_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint16_t bcdMSC;
	uint16_t wTotalLength;
} __packed;

// midi10 6.1.2.2 MIDI IN Jack Descriptor
struct usbd_midi_in_jack_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bJackType;
	uint8_t bJackId;
	uint8_t iJack;
} __packed;

// midi10 6.1.2.3 MIDI OUT Jack Descriptor
struct usbd_midi_out_jack_descriptor {
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

// midi20 5.4.1 Class Specific Group Terminal Block Header Descriptor
struct usbd_midi_grptrm_header_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint16_t wTotalLength;
} __packed;

// midi20 5.4.2.1 Group Terminal Block Descriptor
struct usbd_midi_grptrm_block_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bGrpTrmBlkID;
	uint8_t bGrpTrmBlkType;
	uint8_t nGroupTrm;
	uint8_t nNumGroupTrm;
	uint8_t iBlockItem;
	uint8_t bMIDIProtocol;
	uint16_t wMaxInputBandwidth;
	uint16_t wMaxOutputBandwidth;
} __packed;

// Pair input+output jacks in MIDI1.0 that forms the full path HOST<->EXTERNAL
struct usbd_midi_io_desc {
	struct usbd_midi_in_jack_descriptor in;
	struct usbd_midi_out_jack_descriptor out;
} __packed;

// midi20: 5.1 Core Descriptors: Standard AudioControl (AC) Interface Descriptor
struct usbd_midi_ac_descriptor {
	struct usb_if_descriptor std;
	struct usbd_midi_cs_ac_header_descriptor cs;
} __packed;

// midi10 6.2.2 Class-Specific MS Bulk Data Endpoint Descriptor
#define USBD_MIDI_CS_EP_DESCRIPTOR_T(n_jacks)                                                      \
	struct {                                                                                   \
		uint8_t bLength;                                                                   \
		uint8_t bDescriptorType;                                                           \
		uint8_t bDescriptorSubtype;                                                        \
		uint8_t bNumEmbMIDIJack;                                                           \
		uint8_t baAssocJackID[n_jacks];                                                    \
	} __packed

// Entire MIDIStreaming 1.0 interface descriptor
#define USBD_MIDI1_IF_DESCRIPTOR_T(n_inputs, n_outputs)                                            \
	struct {                                                                                   \
		struct usb_if_descriptor std;                                                      \
		struct usbd_midi_header_descriptor header;                                         \
		struct usbd_midi_io_desc outputs[n_outputs];                                       \
		struct usbd_midi_io_desc inputs[n_inputs];                                         \
		struct usb_ep_descriptor out_ep;                                                   \
		USBD_MIDI_CS_EP_DESCRIPTOR_T(n_outputs) cs_out_ep;                                 \
		struct usb_ep_descriptor in_ep;                                                    \
		USBD_MIDI_CS_EP_DESCRIPTOR_T(n_inputs) cs_in_ep;                                   \
	} __packed

// Entire MIDIStreaming 2.0 interface descriptor
#define USBD_MIDI2_IF_DESCRIPTOR_T(n_inputs, n_outputs)                                            \
	struct {                                                                                   \
		struct usb_if_descriptor std;                                                      \
		struct usbd_midi_header_descriptor header;                                         \
		struct usb_ep_descriptor out_ep;                                                   \
		USBD_MIDI_CS_EP_DESCRIPTOR_T(n_outputs) cs_out_ep;                                 \
		struct usb_ep_descriptor in_ep;                                                    \
		USBD_MIDI_CS_EP_DESCRIPTOR_T(n_inputs) cs_in_ep;                                   \
	} __packed

#ifndef CONFIG_USBD_MIDI_CLASS_MIDI2
#define USBD_MIDI_IF_DESCRIPTOR_T(n_inputs, n_outputs)                                             \
	struct {                                                                                   \
		struct usbd_midi_ac_descriptor if0;                                                \
		USBD_MIDI1_IF_DESCRIPTOR_T(n_inputs, n_outputs) if1_0;                             \
	} __packed
#else
// midi20 3.1.1 MIDI Streaming Interface with Two Alternate Settings
#define USBD_MIDI_IF_DESCRIPTOR_T(n_inputs, n_outputs)                                             \
	struct {                                                                                   \
		struct usbd_midi_ac_descriptor if0;                                                \
		USBD_MIDI1_IF_DESCRIPTOR_T(n_inputs, n_outputs) if1_0;                             \
		USBD_MIDI2_IF_DESCRIPTOR_T(n_inputs, n_outputs) if1_1;                             \
	} __packed
#endif

// midi20 5.4 Class-Specific Group Terminal Block Descriptors
#define USBD_MIDI2_GRPTRM_DESCRIPTOR_T(n_inputs, n_outputs)                                        \
	struct {                                                                                   \
		struct usbd_midi_grptrm_header_descriptor head;                                    \
		struct usbd_midi_grptrm_block_descriptor inputs[n_inputs];                         \
		struct usbd_midi_grptrm_block_descriptor outputs[n_outputs];                       \
	} __packed

// Device configuration
struct usbd_midi_config {
	/* Pointer to the class interface descriptors */
	size_t n_desc;
	struct usb_desc_header const **descs;
#ifdef CONFIG_USBD_MIDI_CLASS_MIDI2
	/* Pointer to the MIDI2.0 group terminal descriptors */
	const void *grptrm_desc;
	size_t grptrm_desc_size;
#endif
};

// Device driver data
struct usbd_midi_data {
	/* Pointer to the associated USBD class node */
	struct usbd_class_data *class_data;

	struct k_work rx_work;
	struct k_work tx_work;
	uint8_t tx_queue_buf[MIDI_QUEUE_SIZE];
	struct ring_buf tx_queue;
	uint8_t altsetting;
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
	const struct usbd_midi_config *config = dev->config;
	LOG_DBG("Get descriptors for %s", dev->name);
	return config->descs;
}

static void usbd_midi1_recv(const struct device *dev, struct net_buf *buf)
{
	LOG_HEXDUMP_DBG(buf->data, buf->len, "MIDI1 - Rx DATA");
}

#ifdef CONFIG_USBD_MIDI_CLASS_MIDI2
static void usbd_midi2_recv(const struct device *dev, struct net_buf *buf)
{
	// midi20 3.2.2 UMP Messages in a USB Packet: Byte Ordering
	uint32_t *words = (uint32_t *)buf->data;
	for (int i = 0; i < buf->len / 4; i++) {
		words[i] = sys_le32_to_cpu(words[i]);
	}
	LOG_HEXDUMP_DBG(buf->data, buf->len, "MIDI2 - Rx DATA");
}
#endif

static int usbd_midi_class_request(struct usbd_class_data *const class_data, struct net_buf *buf,
				   int err)
{
	struct usbd_context *uds_ctx = usbd_class_get_ctx(class_data);
	const struct device *dev = usbd_class_get_private(class_data);
	struct usbd_midi_data *data = dev->data;
	struct udc_buf_info *info = udc_get_buf_info(buf);
	LOG_DBG("USB-MIDI request for %s ep=%d len=%d err=%d", dev->name, info->ep, buf->len, err);

	if (err) {
		LOG_ERR("Class request error: %d", err);
	}
	if (info->ep == ENDPOINT_FROM_HOST) {
		if (data->altsetting == ALT_USB_MIDI_1) {
			usbd_midi1_recv(dev, buf);
		}
#ifdef CONFIG_USBD_MIDI_CLASS_MIDI2
		else if (data->altsetting == ALT_USB_MIDI_2) {
			usbd_midi2_recv(dev, buf);
		}
#endif
		k_work_submit(&data->rx_work);
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
	struct usbd_midi_data *data = dev->data;
	LOG_DBG("USB-MIDI update for %s: if=%d, alt=%d", dev->name, (int)iface, (int)alternate);

	// Only interface 1 can be configured
	if (iface != 1) {
		LOG_WRN("Unable to configure USB interface %d for %s", iface, dev->name);
		return;
	}

	switch (alternate) {
	case ALT_USB_MIDI_1:
		data->altsetting = ALT_USB_MIDI_1;
		LOG_INF("%s set USB-MIDI1.0 altsetting", dev->name);
		break;
#ifdef CONFIG_USBD_MIDI_CLASS_MIDI2
	case ALT_USB_MIDI_2:
		data->altsetting = ALT_USB_MIDI_2;
		LOG_INF("%s set USB-MIDI2.0 altsetting", dev->name);
		break;
#endif
	default:
		LOG_WRN("Unknown alt setting %d for %s", alternate, dev->name);
	}
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
	LOG_DBG("  bmRequestType=%02X bRequest=%02X wValue=%04X wIndex=%04X wLength=%04X",
		setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex,
		setup->wLength);

#ifdef CONFIG_USBD_MIDI_CLASS_MIDI2
	const struct usbd_midi_config *config = dev->config;
	struct usbd_midi_data *data = dev->data;

	// midi20 6. Class Specific Command: Group Terminal Blocks Descriptors Request
	if (data->altsetting == ALT_USB_MIDI_2 && setup->bRequest == USB_SREQ_GET_DESCRIPTOR) {
		if (buf == NULL) {
			errno = -ENOMEM;
			return 0;
		}

		uint8_t desc_type = setup->wValue >> 8;
		uint8_t altsetting = setup->wValue & 0xff;
		if (desc_type != CS_GR_TRM_BLOCK || altsetting != data->altsetting) {
			return 0;
		}

		size_t min_len = MIN(config->grptrm_desc_size, setup->wLength);
		net_buf_add_mem(buf, config->grptrm_desc, min_len);
		LOG_HEXDUMP_DBG(buf->data, buf->len, "Control to host");
	}
#endif
	return 0;
}

static int usbd_midi_class_ctd(struct usbd_class_data *const class_data,
			       const struct usb_setup_packet *const setup,
			       const struct net_buf *const buf)
{
	const struct device *dev = usbd_class_get_private(class_data);
	LOG_DBG("USB-MIDI control to device for %s", dev->name);
	LOG_DBG("  bmRequestType=%04X bRequest=%04X wValue=%04X wIndex=%04X wLength=%04X",
		setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex,
		setup->wLength);
	return 0;
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
		LOG_WRN("Unable to allocate Rx net_buf");
		return;
	}

	LOG_DBG("Enqueue Rx...");
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

int usb_midi_send(const struct device *dev, union ump *pkt)
{
	LOG_DBG("Send MT=%02X group=%02X", pkt->mt, pkt->group);

	struct usbd_midi_data *data = dev->data;
	size_t words = ump_words(pkt->mt);
	size_t buflen = 4 * words;

	uint32_t *buf = NULL;
	int r = ring_buf_put_claim(&data->tx_queue, (uint8_t **)&buf, buflen);
	if (r != buflen) {
		LOG_WRN("Not enough space in tx queue");
		ring_buf_put_finish(&data->tx_queue, 0);
		return -EAGAIN;
	}

	for (size_t i = 0; i < words; i++) {
		buf[i] = sys_cpu_to_le32(pkt->words[i]);
	}

	ring_buf_put_finish(&data->tx_queue, buflen);
	k_work_submit(&data->tx_work);
	return 0;
}

#define USBD_MIDI_AC_INIT_DESCRIPTORS                                                              \
	{                                                                                          \
		.std =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usb_if_descriptor),                       \
				.bDescriptorType = USB_DESC_INTERFACE,                             \
				.bInterfaceNumber = 0,                                             \
				.bAlternateSetting = 0,                                            \
				.bNumEndpoints = 0,                                                \
				.bInterfaceClass = AUDIO,                                          \
				.bInterfaceSubClass = AUDIOCONTROL,                                \
			},                                                                         \
		.cs =                                                                              \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_cs_ac_header_descriptor),       \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MS_HEADER,                                   \
				.bcdADC = sys_cpu_to_le16(0x0100),                                 \
				.wTotalLength = sizeof(struct usbd_midi_cs_ac_header_descriptor),  \
				.bInCollection = 1,                                                \
				.baInterfaceNr = 1,                                                \
			},                                                                         \
	}

#define USBD_MIDI1_INPUT_DESCRIPTORS(input_no)                                                     \
	{                                                                                          \
		.in =                                                                              \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_in_jack_descriptor),            \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MIDI_IN_JACK,                                \
				.bJackType = JACK_EXTERNAL,                                        \
				.bJackId = input_no,                                               \
			},                                                                         \
		.out =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_out_jack_descriptor),           \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MIDI_OUT_JACK,                               \
				.bJackType = JACK_EMBEDDED,                                        \
				.bJackId = input_no,                                               \
				.bNrInputPins = 1,                                                 \
				.baSourceID = input_no,                                            \
				.BaSourcePin = 1,                                                  \
			},                                                                         \
	}

#define USBD_MIDI1_OUTPUT_DESCRIPTORS(output_no)                                                   \
	{                                                                                          \
		.in =                                                                              \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_in_jack_descriptor),            \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MIDI_IN_JACK,                                \
				.bJackType = JACK_EMBEDDED,                                        \
				.bJackId = output_no,                                              \
			},                                                                         \
		.out =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_out_jack_descriptor),           \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MIDI_OUT_JACK,                               \
				.bJackType = JACK_EXTERNAL,                                        \
				.bJackId = output_no,                                              \
				.bNrInputPins = 1,                                                 \
				.baSourceID = output_no,                                           \
				.BaSourcePin = 1,                                                  \
			},                                                                         \
	}

#define USBD_MIDI1_INIT_DESCRIPTORS(n_inputs, n_outputs)                                           \
	{                                                                                          \
		.std =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usb_if_descriptor),                       \
				.bDescriptorType = USB_DESC_INTERFACE,                             \
				.bInterfaceNumber = 1,                                             \
				.bAlternateSetting = 0,                                            \
				.bNumEndpoints = 2,                                                \
				.bInterfaceClass = AUDIO,                                          \
				.bInterfaceSubClass = MIDISTREAMING,                               \
			},                                                                         \
		.header =                                                                          \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_header_descriptor),             \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MS_HEADER,                                   \
				.bcdMSC = sys_cpu_to_le16(0x0100),                                 \
				.wTotalLength = sys_cpu_to_le16(                                   \
					sizeof(USBD_MIDI_IF_DESCRIPTOR_T(n_inputs, n_outputs)) -   \
					sizeof(struct usb_if_descriptor)),                         \
			},                                                                         \
		.inputs = {USBD_MIDI1_INPUT_DESCRIPTORS(1)},                                       \
		.outputs = {USBD_MIDI1_OUTPUT_DESCRIPTORS(2)},                                     \
		.out_ep =                                                                          \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = FIRST_OUT_EP_ADDR,                             \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64U),                            \
			},                                                                         \
		.cs_out_ep =                                                                       \
			{                                                                          \
				.bLength = sizeof(USBD_MIDI_CS_EP_DESCRIPTOR_T(n_outputs)),        \
				.bDescriptorType = CS_ENDPOINT,                                    \
				.bDescriptorSubtype = EP_GENERAL,                                  \
				.bNumEmbMIDIJack = 1,                                              \
				.baAssocJackID = {2},                                              \
			},                                                                         \
		.in_ep =                                                                           \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = FIRST_IN_EP_ADDR,                              \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64U),                            \
			},                                                                         \
		.cs_in_ep =                                                                        \
			{                                                                          \
				.bLength = sizeof(USBD_MIDI_CS_EP_DESCRIPTOR_T(n_inputs)),         \
				.bDescriptorType = CS_ENDPOINT,                                    \
				.bDescriptorSubtype = EP_GENERAL,                                  \
				.bNumEmbMIDIJack = 1,                                              \
				.baAssocJackID = {1},                                              \
			},                                                                         \
	}

#define USBD_MIDI2_INIT_DESCRIPTORS(n_inputs, n_outputs)                                           \
	{                                                                                          \
		.std =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usb_if_descriptor),                       \
				.bDescriptorType = USB_DESC_INTERFACE,                             \
				.bInterfaceNumber = 1,                                             \
				.bAlternateSetting = 1,                                            \
				.bNumEndpoints = 2,                                                \
				.bInterfaceClass = AUDIO,                                          \
				.bInterfaceSubClass = MIDISTREAMING,                               \
			},                                                                         \
		.header =                                                                          \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_header_descriptor),             \
				.bDescriptorType = CS_INTERFACE,                                   \
				.bDescriptorSubtype = MS_HEADER,                                   \
				.bcdMSC = sys_cpu_to_le16(0x0200),                                 \
				.wTotalLength = sys_cpu_to_le16(                                   \
					sizeof(struct usbd_midi_header_descriptor)),               \
			},                                                                         \
		.out_ep =                                                                          \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = FIRST_OUT_EP_ADDR,                             \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64U),                            \
			},                                                                         \
		.cs_out_ep =                                                                       \
			{                                                                          \
				.bLength = sizeof(USBD_MIDI_CS_EP_DESCRIPTOR_T(n_outputs)),        \
				.bDescriptorType = CS_ENDPOINT,                                    \
				.bDescriptorSubtype = EP_GENERAL_2_0,                              \
				.bNumEmbMIDIJack = 1,                                              \
				.baAssocJackID = {1},                                              \
			},                                                                         \
		.in_ep =                                                                           \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = FIRST_IN_EP_ADDR,                              \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64U),                            \
			},                                                                         \
		.cs_in_ep =                                                                        \
			{                                                                          \
				.bLength = sizeof(USBD_MIDI_CS_EP_DESCRIPTOR_T(n_inputs)),         \
				.bDescriptorType = CS_ENDPOINT,                                    \
				.bDescriptorSubtype = EP_GENERAL_2_0,                              \
				.bNumEmbMIDIJack = 1,                                              \
				.baAssocJackID = {1},                                              \
			},                                                                         \
	}

#define USBD_MIDI2_DEFINE_GRPTRM_DESCRIPTOR(inst)                                                  \
	static const USBD_MIDI2_GRPTRM_DESCRIPTOR_T(1, 0) usbd_midi_grptrm_##inst = {              \
		.head =                                                                            \
			{                                                                          \
				.bLength = sizeof(struct usbd_midi_grptrm_header_descriptor),      \
				.bDescriptorType = CS_GR_TRM_BLOCK,                                \
				.bDescriptorSubtype = GR_TRM_BLOCK_HEADER,                         \
				.wTotalLength = sys_cpu_to_le16(                                   \
					sizeof(USBD_MIDI2_GRPTRM_DESCRIPTOR_T(1, 0))),             \
			},                                                                         \
		.inputs =                                                                          \
			{                                                                          \
				{                                                                  \
					.bLength =                                                 \
						sizeof(struct usbd_midi_grptrm_block_descriptor),  \
					.bDescriptorType = CS_GR_TRM_BLOCK,                        \
					.bDescriptorSubtype = GR_TRM_BLOCK,                        \
					.bGrpTrmBlkID = 1,                                         \
					.bGrpTrmBlkType = GR_TRM_BIDIRECTIONAL,                    \
					.nGroupTrm = 0,                                            \
					.nNumGroupTrm = 1,                                         \
					.iBlockItem = 0,                                           \
					.bMIDIProtocol = MIDI_1_0_UP_TO_128_BITS,                  \
					.wMaxInputBandwidth = 0x0000,                              \
					.wMaxOutputBandwidth = 0x0000,                             \
				},                                                                 \
			},                                                                         \
		.outputs = {},                                                                     \
	};

#ifndef CONFIG_USBD_MIDI_CLASS_MIDI2
#define USBD_MIDI_DEFINE_DESCRIPTOR(inst)                                                          \
	static USBD_MIDI_IF_DESCRIPTOR_T(1, 1) usbd_midi_desc_##inst = {                           \
		.if0 = USBD_MIDI_AC_INIT_DESCRIPTORS,                                              \
		.if1_0 = USBD_MIDI1_INIT_DESCRIPTORS(1, 1),                                        \
	};                                                                                         \
	static const struct usb_desc_header *usbd_midi_desc_array_##inst[] = {                     \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if0.std,                          \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if0.cs,                           \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.std,                        \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.header,                     \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.inputs[0].in,               \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.inputs[0].out,              \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.outputs[0].in,              \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.outputs[0].out,             \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.out_ep,                     \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.cs_out_ep,                  \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.in_ep,                      \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.cs_in_ep,                   \
	};
#define USBD_MIDI_DEFINE_CONFIG(inst)                                                              \
	static const struct usbd_midi_config usbd_midi_config_##inst = {                           \
		.n_desc = ARRAY_SIZE(usbd_midi_desc_array_##inst),                                 \
		.descs = usbd_midi_desc_array_##inst,                                              \
	};

#else /* CONFIG_USBD_MIDI_CLASS_MIDI2 */

#define USBD_MIDI_DEFINE_DESCRIPTOR(inst)                                                          \
	USBD_MIDI2_DEFINE_GRPTRM_DESCRIPTOR(inst)                                                  \
	static USBD_MIDI_IF_DESCRIPTOR_T(1, 1) usbd_midi_desc_##inst = {                           \
		.if0 = USBD_MIDI_AC_INIT_DESCRIPTORS,                                              \
		.if1_0 = USBD_MIDI1_INIT_DESCRIPTORS(1, 1),                                        \
		.if1_1 = USBD_MIDI2_INIT_DESCRIPTORS(1, 1),                                        \
	};                                                                                         \
	static const struct usb_desc_header *usbd_midi_desc_array_##inst[] = {                     \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if0.std,                          \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if0.cs,                           \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.std,                        \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.header,                     \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.inputs[0].in,               \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.inputs[0].out,              \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.outputs[0].in,              \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.outputs[0].out,             \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.out_ep,                     \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.cs_out_ep,                  \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.in_ep,                      \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_0.cs_in_ep,                   \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_1.std,                        \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_1.header,                     \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_1.out_ep,                     \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_1.cs_out_ep,                  \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_1.in_ep,                      \
		(struct usb_desc_header *)&usbd_midi_desc_##inst.if1_1.cs_in_ep,                   \
	};

#define USBD_MIDI_DEFINE_CONFIG(inst)                                                              \
	static const struct usbd_midi_config usbd_midi_config_##inst = {                           \
		.n_desc = ARRAY_SIZE(usbd_midi_desc_array_##inst),                                 \
		.descs = usbd_midi_desc_array_##inst,                                              \
		.grptrm_desc = &usbd_midi_grptrm_##inst,                                           \
		.grptrm_desc_size = sizeof(usbd_midi_grptrm_##inst),                               \
	};
#endif

#define USBD_MIDI_DT_DEVICE_DEFINE(inst)                                                           \
	USBD_MIDI_DEFINE_DESCRIPTOR(inst);                                                         \
	USBD_DEFINE_CLASS(midi_##inst, &usbd_midi_class_api,                                       \
			  (void *)DEVICE_DT_GET(DT_DRV_INST(inst)), NULL);                         \
	USBD_MIDI_DEFINE_CONFIG(inst);                                                             \
	static struct usbd_midi_data usbd_midi_data_##inst = {.class_data = &midi_##inst};         \
	DEVICE_DT_INST_DEFINE(inst, usbd_midi_preinit, NULL, &usbd_midi_data_##inst,               \
			      &usbd_midi_config_##inst, POST_KERNEL, CONFIG_SERIAL_INIT_PRIORITY,  \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(USBD_MIDI_DT_DEVICE_DEFINE)
