/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NETMIDI2_H_
#define NETMIDI2_H_

#include <stdint.h>
#include <zephyr/net/socket.h>
#include <zephyr/audio/midi.h>

#define CLIENT_CAP_INV_WITH_AUTH	BIT(0)
#define CLIENT_CAP_INV_WITH_USER_AUTH	BIT(1)

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

#define UDP_MIDI_EP_DECLARE(_name, _n_peers) \
	static struct udp_midi_session peers_of_##_name[_n_peers]; \
	static struct udp_midi_ep _name = {.n_peers = _n_peers, .peers = peers_of_##_name}

enum udp_midi_session_state {
	NOT_INITIALIZED = 0,
	IDLE,
	PENDING_INVITATION,
	AUTHENTICATION_REQUIRED,
	ESTABLISHED_SESSION,
	PENDING_SESSION_RESET,
	PENDING_BYTE,
};

struct udp_midi_ep;

#define UDP_MIDI_NONCE_SIZE 16

struct udp_midi_user {
	const char *name;
	const char *password;
};

struct udp_midi_userlist {
	size_t n_users;
	const struct udp_midi_user users[];
};

struct udp_midi_session {
	enum udp_midi_session_state state;
	uint16_t tx_ump_seq;
	uint16_t rx_ump_seq;
	struct sockaddr addr;
	socklen_t addr_len;
	const struct udp_midi_ep *ep;
	const struct udp_midi_user *user;
	char nonce[UDP_MIDI_NONCE_SIZE];
	struct net_buf *tx_buf;
	struct k_work tx_work;
};

enum udp_midi_auth_type {
	UDP_MIDI_NO_AUTH,
	UDP_MIDI_SHARED_SECRET,
	UDP_MIDI_USER_PASSWORD,
};

struct udp_midi_ep {
	int sock;
	struct k_work rx_work;
	void (*rx_packet_cb)(struct udp_midi_session *session,
			     const struct midi_ump ump);
	size_t n_peers;
	struct udp_midi_session *peers;
	enum udp_midi_auth_type auth_type;
	union {
		const char *shared_auth_secret;
		const struct udp_midi_userlist *userlist;
	};
};

int udp_midi_ep_start(struct udp_midi_ep *ep,
		      const struct sockaddr *addr, socklen_t addr_len);

/**
 * @brief      Send a Universal MIDI Packet to all clients connected to the endpoint
 * @param      ep    The endpoint
 * @param[in]  ump   The packet to send
 */
void udp_midi_broadcast(struct udp_midi_ep *ep, const struct midi_ump ump);

/**
 * @brief      Send a Universal MIDI Packet to a single client
 * @param      sess  The session identifying the single client
 * @param[in]  ump   The packet to send
 */
void udp_midi_send(struct udp_midi_session *sess, const struct midi_ump ump);

#endif
