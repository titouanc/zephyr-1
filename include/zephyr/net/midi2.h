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

#define NETMIDI2_NONCE_SIZE 16

#define NETMIDI2_EP_DECLARE_NO_AUTH(_name, _port) \
	static struct netmidi2_ep _name = { \
		.addr4.sin_port = (_port), \
		.auth_type = NETMIDI2_AUTH_NONE, \
	}

#if CONFIG_NETMIDI2_HOST_AUTH
#define NETMIDI2_EP_DECLARE_WITH_AUTH(_name, _port, _secret) \
	static struct netmidi2_ep _name = { \
		.addr4.sin_port = (_port), \
		.auth_type = NETMIDI2_AUTH_SHARED_SECRET, \
		.shared_auth_secret = (_secret), \
	}

#define NETMIDI2_EP_DECLARE_WITH_USERS(_name, _port, ...) \
	static const struct netmidi2_userlist users_of_##_name = { \
		.n_users = ARRAY_SIZE(((struct netmidi2_user []) { __VA_ARGS__ })), \
		.users = { __VA_ARGS__ }, \
	}; \
	static struct netmidi2_ep _name = { \
		.addr4.sin_port = (_port), \
		.auth_type = NETMIDI2_USER_PASSWORD, \
		.userlist = &users_of_##_name, \
	}
#endif /* CONFIG_NETMIDI2_HOST_AUTH */

enum netmidi2_session_state {
	NOT_INITIALIZED = 0,
	IDLE,
	PENDING_INVITATION,
	AUTHENTICATION_REQUIRED,
	ESTABLISHED_SESSION,
	PENDING_SESSION_RESET,
	PENDING_BYE,
};

struct netmidi2_ep;

struct netmidi2_user {
	const char *name;
	const char *password;
};

struct netmidi2_userlist {
	size_t n_users;
	const struct netmidi2_user users[];
};

struct netmidi2_session {
	enum netmidi2_session_state state;
	uint16_t tx_ump_seq;
	uint16_t rx_ump_seq;
	struct sockaddr addr;
	socklen_t addr_len;
	const struct netmidi2_ep *ep;
#if CONFIG_NETMIDI2_HOST_AUTH
	const struct netmidi2_user *user;
	char nonce[NETMIDI2_NONCE_SIZE];
#endif
	struct net_buf *tx_buf;
	struct k_work tx_work;
};

enum netmidi2_auth_type {
	NETMIDI2_AUTH_NONE,
	NETMIDI2_AUTH_SHARED_SECRET,
	NETMIDI2_USER_PASSWORD,
};

struct netmidi2_ep {
	struct sockaddr_in addr4;
	struct pollfd pollsock;
	void (*rx_packet_cb)(struct netmidi2_session *session,
			     const struct midi_ump ump);
	struct netmidi2_session peers[CONFIG_NETMIDI2_HOST_MAX_CLIENTS];
	enum netmidi2_auth_type auth_type;
#if CONFIG_NETMIDI2_HOST_AUTH
	union {
		const char *shared_auth_secret;
		const struct netmidi2_userlist *userlist;
	};
#endif
};

/**
 * @brief      Initialize a network (UDP) Universal MIDI Packet endpoint
 * @param      ep    The network endpoint to initializer
 * @return     0 on success, -errno on error
 */
int netmidi2_ep_init(struct netmidi2_ep *ep);

/**
 * @brief      Send a Universal MIDI Packet to all clients connected to the endpoint
 * @param      ep    The endpoint
 * @param[in]  ump   The packet to send
 */
void netmidi2_broadcast(struct netmidi2_ep *ep, const struct midi_ump ump);

/**
 * @brief      Send a Universal MIDI Packet to a single client
 * @param      sess  The session identifying the single client
 * @param[in]  ump   The packet to send
 */
void netmidi2_send(struct netmidi2_session *sess, const struct midi_ump ump);

#endif
