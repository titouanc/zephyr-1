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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_midi2_sample, LOG_LEVEL_DBG);

#define MY_PORT 5673
#define BUFSIZE 256

#define COMMAND_INVITATION 0x01
#define COMMAND_INVITATION_WITH_AUTH 0x02
#define COMMAND_INVITATION_WITH_USER_AUTH 0x03
#define COMMAND_INVITATION_REPLY_ACCEPTED 0x10
#define COMMAND_INVITATION_REPLY_PENDING 0x11
#define COMMAND_INVITATION_REPLY_AUTH_REQUIRED 0x12
#define COMMAND_INVITATION_REPLY_USER_AUTH_REQUIRED 0x13
#define COMMAND_PING 0x20
#define COMMAND_PING_REPLY 0x21
#define COMMAND_RETRANSMIT_REQUEST 0x80
#define COMMAND_RETRANSMIT_ERROR 0x81
#define COMMAND_SESSION_RESET 0x82
#define COMMAND_SESSION_RESET_REPLY 0x83
#define COMMAND_NAK 0x8F
#define COMMAND_BYE 0xF0
#define COMMAND_BYE_REPLY 0xF1
#define COMMAND_UMP_DATA 0xFF

static int sock = 0;

void netmidi2_callback(const struct midi_ump ump);

static int dispatch_command_packet(const uint8_t *cmd_pkt, size_t cmd_pkt_len, uint8_t *reply, size_t reply_len)
{
	struct midi_ump ump;
	const uint32_t *words;

	switch (cmd_pkt[0]) {
	case COMMAND_PING:
		if (cmd_pkt_len != 8) {
			LOG_ERR("Invalid PING packet length (%d)", cmd_pkt_len);
			return -EINVAL;
		}

		if (reply_len < 8) {
			LOG_WRN("Not enough buffer space for PING_REPLY");
			return -ENOBUFS;
		}

		memcpy(reply, cmd_pkt, 8);
		reply[0] = COMMAND_PING_REPLY;
		return 8;

	case COMMAND_INVITATION:
		// TODO: add non-empty endpoint name and product id
		if (reply_len < 4) {
			LOG_WRN("Not enough buffer space for INVITATION_REPLY");
			return -ENOBUFS;
		}
		reply[0] = COMMAND_INVITATION_REPLY_ACCEPTED;
		memset(&reply[1], 0, 3);
		return 4;

	case COMMAND_UMP_DATA:
		if (cmd_pkt[1] > 4) {
			LOG_ERR("Invalid UMP payload length (%d)", cmd_pkt[1]);
			return 0;
		}
		words = (const uint32_t *) &cmd_pkt[4];
		for (size_t i=0; i<cmd_pkt[1]; i++) {
			ump.data[i] = sys_be32_to_cpu(words[i]);
		}

		if (UMP_NUM_WORDS(ump) != cmd_pkt[1]) {
			LOG_ERR("Invalid packet size fot MT=%02X", UMP_MT(ump));
			return 0;
		}

		netmidi2_callback(ump);
		return 0;

	default:
		LOG_WRN("Unknown command code %02X", cmd_pkt[0]);
		return 0;
	}
	return 0;
}

static int do_rx()
{
	int ret;
	uint8_t rx_buf[BUFSIZE], tx_buf[BUFSIZE] = "MIDI";
	size_t rx_len = 0, rx_off = 0, tx_off = 4, plen;
	struct sockaddr client_addr;
	socklen_t client_addr_len = sizeof(client_addr);

	LOG_DBG("Waiting for new UDP packet...");

	ret = zsock_recvfrom(sock, rx_buf, sizeof(rx_buf), 0, &client_addr, &client_addr_len);
	if (ret <= 0) {
		LOG_ERR("Rx error: %d (%d)", ret, errno);
		return -1;
	}

	rx_len = ret;
	LOG_HEXDUMP_DBG(rx_buf, ret, "Received UDP packet");
	if (rx_len < 4 || memcmp(rx_buf, "MIDI", 4) != 0) {
		LOG_WRN("Not a MIDI packet");
		return 0;
	}

	rx_off = 4;
	while (rx_off < rx_len) {
		plen = 4 * rx_buf[rx_off + 1];
		if (rx_off + 4 + plen > rx_len) {
			LOG_ERR("Malformed packet");
			return -1;
		}

		ret = dispatch_command_packet(&rx_buf[rx_off], 4 + plen, &tx_buf[tx_off], sizeof(tx_buf) - tx_off);
		if (ret < 0) {
			return ret;
		}

		tx_off += ret;
		rx_off += 4 + plen;
	}

	if (tx_off > 4) {
		ret = zsock_sendto(sock, tx_buf, tx_off, 0, &client_addr, client_addr_len);
		if (ret < 0) {
			LOG_ERR("Unable to send reply: %d", errno);
			return -1;
		}
	}

	return 0;
}

void netmidi2_callback(const struct midi_ump ump)
{
	LOG_INF("Rx MIDI MT=%02X", UMP_MT(ump));
}

int main(void)
{
	int ret;
	struct sockaddr_in addr4;

	memset(&addr4, 0, sizeof(addr4));
	addr4.sin_family = AF_INET;
	addr4.sin_port = htons(MY_PORT);

	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Unable to create socket: %d", errno);
		return -1;
	}

	ret = zsock_bind(sock, (const struct sockaddr *) &addr4, sizeof(addr4));
	if (ret < 0) {
		LOG_ERR("Failed to bind UDP socket: %d", errno);
		ret = -errno;
	}

	while (do_rx() == 0);

	return 0;
}
