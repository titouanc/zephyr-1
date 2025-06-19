/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <errno.h>
#include <stdio.h>

#include <zephyr/audio/midi.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/net/socket.h>

#include "netmidi2.h"
#include "ump_stream_responder.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_midi2_sample, LOG_LEVEL_DBG);

#define MY_PORT 5673

#if ! DT_HAS_CHOSEN(midi_tx_uart)
static inline void send_midi1(...) {}
#else
#include <zephyr/drivers/uart.h>

static const struct device *const uart_dev =
	DEVICE_DT_GET(DT_CHOSEN(midi_tx_uart));

#if DT_HAS_CHOSEN(midi_tx_led)
#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec act_led =
	GPIO_DT_SPEC_GET_OR(DT_CHOSEN(midi_tx_led), gpios, {0});
#endif  /* DT_HAS_CHOSEN(midi_tx_led) */

static inline void send_midi1(const struct midi_ump ump)
{
	switch (UMP_MIDI_COMMAND(ump)) {
	case UMP_MIDI_NOTE_OFF:
	case UMP_MIDI_NOTE_ON:
	case UMP_MIDI_AFTERTOUCH:
	case UMP_MIDI_CONTROL_CHANGE:
	case UMP_MIDI_PITCH_BEND:

#if DT_HAS_CHOSEN(midi_tx_led)
		gpio_pin_set_dt(&act_led, 1);
#endif /* DT_HAS_CHOSEN(midi_tx_led) */

		uart_poll_out(uart_dev, UMP_MIDI_STATUS(ump));
		uart_poll_out(uart_dev, UMP_MIDI1_P1(ump));
		uart_poll_out(uart_dev, UMP_MIDI1_P2(ump));

#if DT_HAS_CHOSEN(midi_tx_led)
		gpio_pin_set_dt(&act_led, 0);
#endif /* DT_HAS_CHOSEN(midi_tx_led) */

	}
}
#endif /* DT_HAS_CHOSEN(midi_tx_uart) */

static const struct ump_endpoint_dt_spec ump_ep_dt =
	UMP_ENDPOINT_DT_SPEC_GET(DT_NODELABEL(midi2));

static inline void handle_ump_stream(struct udp_midi_session *session,
				     const struct midi_ump ump)
{
	const struct ump_stream_responder_cfg responder_cfg = {
		.dev = session,
		.send = (void (*)(void *, const struct midi_ump)) udp_midi_send,
		.ep_spec = &ump_ep_dt,
	};
	ump_stream_responder(&responder_cfg, ump);
}

static void netmidi2_callback(struct udp_midi_session *session,
			      const struct midi_ump ump)
{
	switch (UMP_MT(ump)) {
	case UMP_MT_MIDI1_CHANNEL_VOICE:
		send_midi1(ump);
		break;
	case UMP_MT_UMP_STREAM:
		handle_ump_stream(session, ump);
		break;
	}
}

static const struct udp_midi_userlist authorized_users = {
	.n_users=1,
	.users={
		{.name="Titou", .password="Coucou"}
	},
};

UDP_MIDI_EP_DECLARE(midi_server, 10);

DNS_SD_REGISTER_UDP_SERVICE(midi_dns, CONFIG_NET_HOSTNAME, "_midi2", "local",
			    DNS_SD_EMPTY_TXT, MY_PORT);

int main(void)
{
	struct sockaddr_in addr4;

#if DT_HAS_CHOSEN(midi_tx_led)
	gpio_pin_configure_dt(&act_led, GPIO_OUTPUT_INACTIVE);
#endif /* DT_HAS_CHOSEN(midi_tx_led) */

	memset(&addr4, 0, sizeof(addr4));
	addr4.sin_family = AF_INET;
	addr4.sin_port = htons(MY_PORT);

	midi_server.rx_packet_cb = netmidi2_callback;
	midi_server.auth_type = UDP_MIDI_NO_AUTH;
	midi_server.userlist = &authorized_users;
	udp_midi_ep_start(&midi_server, (const struct sockaddr *) &addr4, sizeof(addr4));

	return 0;
}
