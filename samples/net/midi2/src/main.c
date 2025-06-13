/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <errno.h>
#include <stdio.h>

#include <zephyr/net/socket.h>
#include <zephyr/audio/midi.h>

#include "netmidi2.h"
#include "ump_stream_responder.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_midi2_sample, LOG_LEVEL_DBG);

#define MY_PORT 5673

UDP_MIDI_EP_DECLARE(midi_server, 10);

static const struct ump_endpoint_dt_spec ump_ep = UMP_ENDPOINT_DT_SPEC_GET(DT_NODELABEL(midi2));

static void netmidi2_callback(struct udp_midi_session *session,
			      const struct midi_ump ump)
{
	LOG_INF("Rx MIDI MT=%02X", UMP_MT(ump));
	if (UMP_MT(ump) == UMP_MT_UMP_STREAM) {
		const struct ump_stream_responder_cfg responder_cfg = {
			.dev = session,
			.send = (void (*)(void *, const struct midi_ump)) udp_midi_send,
			.ep_spec = &ump_ep,
		};
		ump_stream_responder(&responder_cfg, ump);
	}
}

int main(void)
{
	struct sockaddr_in addr4;

	memset(&addr4, 0, sizeof(addr4));
	addr4.sin_family = AF_INET;
	addr4.sin_port = htons(MY_PORT);

	midi_server.rx_packet_cb = netmidi2_callback;
	udp_midi_ep_start(&midi_server, (const struct sockaddr *) &addr4, sizeof(addr4));

	return 0;
}
