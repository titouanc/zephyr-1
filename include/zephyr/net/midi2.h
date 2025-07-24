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

#define NETMIDI2_EP_DECLARE(_var_name, _ep_name, _piid, _port) \
	static struct netmidi2_ep _var_name = { \
		.name = (_ep_name), \
		.piid = (_piid), \
		.addr4.sin_port = (_port), \
		.auth_type = NETMIDI2_AUTH_NONE, \
	}

#if CONFIG_NETMIDI2_HOST_AUTH
#define NETMIDI2_EP_DECLARE_WITH_AUTH(_var_name, _ep_name, _piid, _port, _secret) \
	static struct netmidi2_ep _var_name = { \
		.name = (_ep_name), \
		.piid = (_piid), \
		.addr4.sin_port = (_port), \
		.auth_type = NETMIDI2_AUTH_SHARED_SECRET, \
		.shared_auth_secret = (_secret), \
	}

#define NETMIDI2_EP_DECLARE_WITH_USERS(_var_name, _ep_name, _piid, _port, ...) \
	static const struct netmidi2_userlist users_of_##_var_name = { \
		.n_users = ARRAY_SIZE(((struct netmidi2_user []) { __VA_ARGS__ })), \
		.users = { __VA_ARGS__ }, \
	}; \
	static struct netmidi2_ep _var_name = { \
		.name = (_ep_name), \
		.piid = (_piid), \
		.addr4.sin_port = (_port), \
		.auth_type = NETMIDI2_AUTH_USER_PASSWORD, \
		.userlist = &users_of_##_var_name, \
	}
#endif /* CONFIG_NETMIDI2_HOST_AUTH */

struct netmidi2_ep;

struct netmidi2_user {
	const char *name;
	const char *password;
};

struct netmidi2_userlist {
	size_t n_users;
	const struct netmidi2_user users[];
};

/**
 * @brief      A Network MIDI2 session, representing a connection to a peer
 */
struct netmidi2_session {
	/** State of this session */
	enum {
		NOT_INITIALIZED = 0,
		IDLE,
		PENDING_INVITATION,
		AUTHENTICATION_REQUIRED,
		ESTABLISHED_SESSION,
		PENDING_SESSION_RESET,
		PENDING_BYE,
	} state;
	/** Sequence number of the next universal MIDI packet to send */
	uint16_t tx_ump_seq;
	/** Sequence number of the next universal MIDI packet to receive */
	uint16_t rx_ump_seq;
	/** Remote address of the peer */
	struct sockaddr addr;
	/** Length of the peer's remote address */
	socklen_t addr_len;
	/** The Network MIDI2 endpoint to which this session belongs */
	struct netmidi2_ep *ep;
#if CONFIG_NETMIDI2_HOST_AUTH
	/** The username to which this session belongs */
	const struct netmidi2_user *user;
	/** The crypto nonce used to authorize this session */
	char nonce[NETMIDI2_NONCE_SIZE];
#endif
	/** The transmission buffer for that peer */
	struct net_buf *tx_buf;
	/** The transmission work for that peer */
	struct k_work tx_work;
};

/**
 * @brief      Type of authentication in Network MIDI2
 */
enum netmidi2_auth_type {
	/** No authentication required */
	NETMIDI2_AUTH_NONE,
	/** Authentication with a shared secret key */
	NETMIDI2_AUTH_SHARED_SECRET,
	/** Authentication with username and password */
	NETMIDI2_AUTH_USER_PASSWORD,
};

/**
 * @brief      A Network MIDI2.0 Endpoint
 */
struct netmidi2_ep {
	/** The endpoint name */
	const char *name;
	/** The endpoint product instance id */
	const char *piid;
	/** The endpoint local address (ipv4) */
	struct sockaddr_in addr4;
	/** The listening socket wrapped in a poll descriptor */
	struct pollfd pollsock;
	/** The function to call when data is received from a client */
	void (*rx_packet_cb)(struct netmidi2_session *session,
			     const struct midi_ump ump);
	/** List of peers to this endpoint */
	struct netmidi2_session peers[CONFIG_NETMIDI2_HOST_MAX_CLIENTS];
	/** The type of authentication required to establish a session
	 *  with this host endpoint
	 */
	enum netmidi2_auth_type auth_type;
#if CONFIG_NETMIDI2_HOST_AUTH
	union {
		/** A shared authentication key */
		const char *shared_auth_secret;
		/** A list of users/passwords */
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
