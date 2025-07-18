/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NETMIDI2_H_
#define NETMIDI2_H_

#include <stdint.h>
#include <zephyr/audio/midi.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/poll.h>

#define UDP_MIDI_NONCE_SIZE 16

#define UDP_MIDI_EP_DECLARE_NO_AUTH(_name, _n_peers, _port) \
	static struct udp_midi_session peers_of_##_name[_n_peers]; \
	static struct udp_midi_ep _name = { \
		.n_peers = _n_peers, .peers = peers_of_##_name, \
		.auth_type = UDP_MIDI_AUTH_NONE, \
		.addr4.sin_port = (_port) \
	}

#if CONFIG_MIDI2_UDP_HOST_AUTH
#define UDP_MIDI_EP_DECLARE_WITH_AUTH(_name, _n_peers, _port, _secret) \
	static struct udp_midi_session peers_of_##_name[_n_peers]; \
	static struct udp_midi_ep _name = { \
		.n_peers = (_n_peers), .peers = peers_of_##_name, \
		.auth_type = UDP_MIDI_AUTH_SHARED_SECRET, \
		.shared_auth_secret = (_secret), .addr4.sin_port = (_port) \
	}

#define UDP_MIDI_EP_DECLARE_WITH_USERS(_name, _n_peers, _port, ...) \
	static struct udp_midi_session peers_of_##_name[_n_peers]; \
	static const struct udp_midi_userlist users_of_##_name = { \
		.n_users = ARRAY_SIZE(((struct udp_midi_user []) { __VA_ARGS__ })), \
		.users = { __VA_ARGS__ }, \
	}; \
	static struct udp_midi_ep _name = { \
		.n_peers = (_n_peers), .peers = peers_of_##_name, \
		.auth_type = UDP_MIDI_USER_PASSWORD, \
		.userlist = &users_of_##_name, \
		.addr4.sin_port = (_port), \
	}
#endif /* CONFIG_MIDI2_UDP_HOST_AUTH */

enum udp_midi_session_state {
	NOT_INITIALIZED = 0,
	IDLE,
	PENDING_INVITATION,
	AUTHENTICATION_REQUIRED,
	ESTABLISHED_SESSION,
	PENDING_SESSION_RESET,
	PENDING_BYE,
};

struct udp_midi_ep;

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
#if CONFIG_MIDI2_UDP_HOST_AUTH
	const struct udp_midi_user *user;
	char nonce[UDP_MIDI_NONCE_SIZE];
#endif
	struct net_buf *tx_buf;
	struct k_work tx_work;
};

enum udp_midi_auth_type {
	UDP_MIDI_AUTH_NONE,
	UDP_MIDI_AUTH_SHARED_SECRET,
	UDP_MIDI_USER_PASSWORD,
};

struct udp_midi_ep {
	struct sockaddr_in addr4;
	struct pollfd pollsock;
	void (*rx_packet_cb)(struct udp_midi_session *session,
			     const struct midi_ump ump);
	size_t n_peers;
	struct udp_midi_session *peers;
	enum udp_midi_auth_type auth_type;
#if CONFIG_MIDI2_UDP_HOST_AUTH
	union {
		const char *shared_auth_secret;
		const struct udp_midi_userlist *userlist;
	};
#endif
};

/**
 * @brief      Initialize a network (UDP) Universal MIDI Packet endpoint
 * @param      ep    The network endpoint to initializer
 * @return     0 on success, -errno on error
 */
int udp_midi_ep_init(struct udp_midi_ep *ep);

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
