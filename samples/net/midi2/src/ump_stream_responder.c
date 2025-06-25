#include "ump_stream_responder.h"

#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ump_stream_responder);

#define BIT_IF(cond, n) ((cond) ? BIT(n) : 0)

/**
 * @brief      Build an Endpoint Info Notification Universal MIDI Packet
 * @see        ump112: 7.1.2 Endpoint Info Notification Message
 */
static inline struct midi_ump make_endpoint_info(const struct ump_endpoint_dt_spec *ep)
{
	bool has_midi1 = false;
	bool has_midi2 = false;
	struct midi_ump res;

	for (size_t i=0; i<ep->n_blocks; i++) {
		if (ep->blocks[i].is_midi1) {
			has_midi1 = true;
		} else {
			has_midi2 = true;
		}
	}

	res.data[0] = (UMP_MT_UMP_STREAM << 28)
                    | (UMP_STREAM_STATUS_EP_INFO << 16)
                    | ((1) << 8) | (1); /* UMP version 1.1 */

        res.data[1] = (1 << 31) /* Static function blocks */
                    | ((ep->n_blocks) << 24)
                    | BIT_IF(has_midi2, 9)
                    | BIT_IF(has_midi1, 8);

	return res;
}

/**
 * @brief      Build a Function Block Info Notification Universal MIDI Packet
 * @see        ump112: 7.1.8 Function Block Info Notification
 */
static inline struct midi_ump make_function_block_info(const struct ump_endpoint_dt_spec *ep, size_t block_num)
{
	const struct ump_block_dt_spec *block = &ep->blocks[block_num];
	struct midi_ump res;
	uint8_t midi1_mode = block->is_31250bps ? 2 : block->is_midi1 ? 1 : 0;

	res.data[0] = (UMP_MT_UMP_STREAM << 28)
		    | (UMP_STREAM_STATUS_FB_INFO << 16)
		    | (1 << 15)  /* Block is active */
		    | (block_num << 8)
		    | BIT_IF(block->is_output, 5) /* UI hint Sender */
		    | BIT_IF(block->is_input, 4)  /* UI hint Receiver */
		    | (midi1_mode << 2)
		    | BIT_IF(block->is_output, 1) /* Function block is output */
		    | BIT_IF(block->is_input, 0); /* Function block is input */

	res.data[1] = (block->first_group << 24)
		    | (block->groups_spanned << 16)
		    | (0x01 << 8)  /* MIDI-CI support for UMP spec 1.1 */
		    | 0xff;  /* At most 255 simultaneous Sysex streams */

	return res;
}

static inline size_t fill_str(struct midi_ump *ump, size_t offset,
			      const char *src, size_t len)
{
	size_t i, j;

	if (offset >= sizeof(struct midi_ump)) {
		return 0;
	}

	for (i=0; i<len && (j = i + offset) < sizeof(struct midi_ump); i++) {
		ump->data[j / 4] |= src[i] << (8 * (3 - (j % 4)));
	}

	return i;
}

/**
 * @brief      Send a string as UMP Stream, possibly splitting into multiple
 *             packets if the string length is larger
 * @param[in]  cfg     The responder configuration
 * @param[in]  string  The string to send
 * @param[in]  prefix  The fixed prefix of UMP packets to send
 * @param[in]  offset  The offset the strings starts in the packet, in bytes
 *
 * @return     The number of packets sent
 */
static inline int send_string(const struct ump_stream_responder_cfg *cfg,
			      const char *string,
			      uint32_t prefix, size_t offset)
{
	struct midi_ump reply;
	size_t stringlen = strlen(string);
	size_t strwidth = sizeof(reply) - offset;
	uint8_t format;
	size_t i = 0;
	int res = 0;

	while (i < stringlen) {
		memset(&reply, 0, sizeof(reply));
		format = (i == 0)
			? (stringlen - i <= strwidth)
				? UMP_STREAM_FORMAT_COMPLETE
				: UMP_STREAM_FORMAT_START
			: (stringlen - i > strwidth)
				? UMP_STREAM_FORMAT_CONTINUE
				: UMP_STREAM_FORMAT_END;

		reply.data[0] = (UMP_MT_UMP_STREAM << 28)
			      | (format << 26)
			      | prefix;

		i += fill_str(&reply, offset, &string[i], stringlen - i);
		cfg->send(cfg->dev, reply);
		res++;
	}

	return res;
}

static inline int ump_ep_discover(const struct ump_stream_responder_cfg *cfg,
				  const struct midi_ump pkt)
{
	int res = 0;
	uint8_t vmaj = UMP_STREAM_EP_DISCOVERY_VMAJ(pkt);
	uint8_t vmin = UMP_STREAM_EP_DISCOVERY_VMIN(pkt);
	uint8_t filter = UMP_STREAM_EP_DISCOVERY_FILTER(pkt);
	LOG_DBG("Endpoint discovery ump v%d.%d filter=%02X", vmaj, vmin, filter);

	/* Request for Endpoint Info Notification */
	if (filter & UMP_EP_DISC_FILTER_EP_INFO) {
		cfg->send(cfg->dev, make_endpoint_info(cfg->ep_spec));
		res++;
	}

	/* Request for Endpoint Name Notification */
	if (filter & UMP_EP_DISC_FILTER_EP_NAME) {
		res += send_string(cfg, cfg->ep_spec->name,
				   UMP_STREAM_STATUS_EP_NAME << 16, 2);
	}

	return res;
}

static inline int ump_fb_discover(const struct ump_stream_responder_cfg *cfg,
				  const struct midi_ump pkt)
{
	int res = 0;
	uint8_t block_num = UMP_STREAM_FB_DISCOVERY_NUM(pkt);
	uint8_t filter = UMP_STREAM_FB_DISCOVERY_FILTER(pkt);

	if (block_num >= cfg->ep_spec->n_blocks) {
		LOG_WRN("Function block discovery block=%d does not exist", block_num);
		return 0;
	}

	const struct ump_block_dt_spec *blk = &cfg->ep_spec->blocks[block_num];

	LOG_DBG("Function block discovery block=%d filter=%02X", block_num, filter);

	if (filter & UMP_FB_DISC_FILTER_INFO) {
		cfg->send(cfg->dev, make_function_block_info(cfg->ep_spec, block_num));
		res++;
	}

	if (filter & UMP_FB_DISC_FILTER_NAME) {
		res += send_string(cfg, blk->name,
				   (UMP_STREAM_STATUS_FB_NAME << 16) | (block_num << 8), 3);
	}

	return res;
}

int ump_stream_responder(const struct ump_stream_responder_cfg *cfg,
			 const struct midi_ump pkt)
{
	if (! cfg->send) {
		return -EINVAL;
	}

	if (UMP_MT(pkt) != UMP_MT_UMP_STREAM) {
		return 0;
	}

	switch (UMP_STREAM_STATUS(pkt)) {
	case UMP_STREAM_STATUS_EP_DISCOVERY:
		return ump_ep_discover(cfg, pkt);
	case UMP_STREAM_STATUS_FB_DISCOVERY:
		return ump_fb_discover(cfg, pkt);
	}

	return 0;
}
