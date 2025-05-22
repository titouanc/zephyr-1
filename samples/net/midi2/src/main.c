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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_midi2_sample, LOG_LEVEL_DBG);

#define MY_PORT 5673

static void netmidi2_callback(struct udp_midi_ep *ep, const struct midi_ump ump)
{
	LOG_INF("[%p] Rx MIDI MT=%02X", ep, UMP_MT(ump));
	udp_midi_send(ep, ump);
}

UDP_MIDI_EP_DECLARE(midi_server, 10);

int main(void)
{
	int ret;
	struct sockaddr_in addr4;

	memset(&addr4, 0, sizeof(addr4));
	addr4.sin_family = AF_INET;
	addr4.sin_port = htons(MY_PORT);

	midi_server.rx_packet_cb = netmidi2_callback;
	udp_midi_ep_start(&midi_server, (const struct sockaddr *) &addr4, sizeof(addr4));

	return 0;
}
