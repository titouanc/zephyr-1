#include "ump_stream_responder.h"

#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ump_stream_responder);

static inline size_t fill_str(struct midi_ump *ump, size_t offset,
			      const char *src, size_t len)
{
	size_t i, j;
	uint32_t *word;

	if (offset >= sizeof(struct midi_ump)) {
		return 0;
	}

	for (i=0; i<len && (j = i + offset) < sizeof(struct midi_ump); i++) {
		ump->data[j / 4] |= src[i] << (8 * (3 - (j % 4)));
	}

	return i;
}

static inline int send_string(const struct ump_stream_responder_cfg *cfg,
			      uint32_t prefix, size_t offset)
{
	struct midi_ump reply;
	const char *name = cfg->ep_spec->name;
	size_t namelen = strlen(name);
	size_t strwidth = sizeof(reply) - offset;
	uint8_t format;
	size_t i = 0;
	int res = 0;

	while (i < namelen) {
		memset(reply, 0, sizeof(reply));
		format = (i == 0)
			? (namelen - i <= strwidth)
				? UMP_STREAM_FORMAT_COMPLETE
				: UMP_STREAM_FORMAT_START
			: (namelen - i > strwidth)
				? UMP_STREAM_FORMAT_CONTINUE
				: UMP_STREAM_FORMAT_END;

		reply.data[0] = (UMP_MT_UMP_STREAM << 28)
			      | (format << 26)
			      | prefix;

		i += fill_str(&reply, offset, &name[i], namelen - i);
		cfg->send(cfg->dev, reply);
		res++;
	}

	return res;
}

static inline int ump_ep_discover(const struct ump_stream_responder_cfg *cfg,
				  const struct midi_ump pkt)
{
	int res = 0;
	LOG_INF("Endpoint discovery ump v%d.%d filter=%02X",
		UMP_STREAM_EP_DISCOVERY_VMAJ(pkt),
		UMP_STREAM_EP_DISCOVERY_VMIN(pkt),
		UMP_STREAM_EP_DISCOVERY_FILTER(pkt));

	/* Request for Endpoint Info Notification */
	if (UMP_STREAM_EP_DISCOVERY_FILTER(pkt) & UMP_EP_DISC_FILTER_EP_INFO) {
		cfg->send(cfg->dev, UMP_STREAM_EP_INFO(1, 1, 1, cfg->ep_spec->n_blocks, 1, 1, 0, 0));
		res++;
	}

	/* Request for Endpoint Name Notification */
	if (UMP_STREAM_EP_DISCOVERY_FILTER(pkt) & UMP_EP_DISC_FILTER_EP_NAME) {
		res += send_string(cfg, UMP_STREAM_STATUS_EP_NAME << 16, 2);
	}

	return res;
}

static inline int ump_fb_discover(struct rpmsg_endpoint *ept, const struct midi_ump pkt)
{
	int res = 0;
	uint8_t block_num = UMP_STREAM_FB_DISCOVERY_NUM(pkt);

	if (block_num >= dt_ump_ep.n_blocks) {
		LOG_WRN("Function block discovery block=%d does not exist", block_num);
		return 0;
	}

	const struct ump_block_dt_spec *blk = &dt_ump_ep.blocks[block_num];

	LOG_INF("Function block discovery block=%d filter=%02X", block_num,
		UMP_STREAM_FB_DISCOVERY_FILTER(pkt));

	if (UMP_STREAM_FB_DISCOVERY_FILTER(pkt) & UMP_FB_DISC_FILTER_INFO) {
		cfg->send(cfg->dev, UMP_STREAM_FB_INFO(1, block_num, blk->is_midi1, blk->is_input, blk->is_output, blk->first_group, blk->groups_spanned, 0, 0));
		res++;
	}

	if (UMP_STREAM_FB_DISCOVERY_FILTER(pkt) & UMP_FB_DISC_FILTER_NAME) {
		res += send_string(cfg, (UMP_STREAM_STATUS_FB_NAME << 16) | (block_num << 8), 3);
	}

	return res;
}

int ump_stream_responder(const struct ump_stream_responder_cfg *cfg,
			 const struct midi_ump pkt)
{
	if (! cfg->send) {
		return -EINVAL;
	}

	if (UMP_MT(pkt) != UMP_MT_STREAM) {
		return 0;
	}

	switch (UMP_STREAM_STATUS(pkt)) {
	case UMP_STREAM_STATUS_EP_DISCOVERY:
		return ump_ep_discover(cfg, pkt);
	case UMP_STREAM_STATUS_FB_DISCOVERY:
		return ump_fb_discover(cfg, pkt);
	}
}
