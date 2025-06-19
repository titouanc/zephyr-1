#ifndef ZYNTH_UMP_DSICOVERY_H_
#define ZYNTH_UMP_DSICOVERY_H_

#include <zephyr/kernel.h>
#include <zephyr/audio/midi.h>

struct ump_block_dt_spec {
	const char *name;
	uint8_t first_group;
	uint8_t groups_spanned;
	bool is_input;
	bool is_output;
	bool is_midi1;
	bool is_31250bps;
};

struct ump_endpoint_dt_spec {
	const char *name;
	size_t n_blocks;
	struct ump_block_dt_spec blocks[];
};

struct ump_stream_responder_cfg {
	void *dev;
	void(*send)(void *dev, const struct midi_ump ump);
	const struct ump_endpoint_dt_spec *ep_spec;
};

#define UMP_BLOCK_DT_SPEC_GET(_node)					   \
{									   \
	.name = DT_PROP_OR(_node, label, DT_NODE_FULL_NAME(_node)),	   \
	.first_group = DT_REG_ADDR(_node),				   \
	.groups_spanned = DT_REG_SIZE(_node),				   \
	.is_input = !DT_ENUM_HAS_VALUE(_node, terminal_type, output_only), \
	.is_output = !DT_ENUM_HAS_VALUE(_node, terminal_type, input_only), \
	.is_midi1 = !DT_ENUM_HAS_VALUE(_node, protocol, midi2),		   \
	.is_31250bps = DT_PROP(_node, serial_31250bps),			   \
}

#define UMP_BLOCK_SEP_IF_OKAY(_node) \
	COND_CODE_1(DT_NODE_HAS_STATUS_OKAY(_node), (UMP_BLOCK_DT_SPEC_GET(_node),), ())

#define UMP_ENDPOINT_DT_SPEC_GET(_node) \
{ \
	.name = DT_PROP_OR(_node, label, DT_NODE_FULL_NAME(_node)), \
	.n_blocks = DT_FOREACH_CHILD_SEP(_node, DT_NODE_HAS_STATUS_OKAY, (+)), \
	.blocks = {DT_FOREACH_CHILD(_node, UMP_BLOCK_SEP_IF_OKAY)}, \
}

int ump_stream_responder(const struct ump_stream_responder_cfg *cfg,
			 const struct midi_ump pkt);

#endif
