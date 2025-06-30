#include "netmidi2.h"

#include <stdio.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/net/socket_service.h>
#include <zephyr/random/random.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udp_midi, LOG_LEVEL_INF);

/* See udp-ump 5.5: Command Codes and Packet Types */
enum udp_midi_cmd {
	COMMAND_INVITATION = 0x01,
	COMMAND_INVITATION_WITH_AUTH = 0x02,
	COMMAND_INVITATION_WITH_USER_AUTH = 0x03,
	COMMAND_INVITATION_REPLY_ACCEPTED = 0x10,
	COMMAND_INVITATION_REPLY_PENDING = 0x11,
	COMMAND_INVITATION_REPLY_AUTH_REQUIRED = 0x12,
	COMMAND_INVITATION_REPLY_USER_AUTH_REQUIRED = 0x13,
	COMMAND_PING = 0x20,
	COMMAND_PING_REPLY = 0x21,
	COMMAND_RETRANSMIT_REQUEST = 0x80,
	COMMAND_RETRANSMIT_ERROR = 0x81,
	COMMAND_SESSION_RESET = 0x82,
	COMMAND_SESSION_RESET_REPLY = 0x83,
	COMMAND_NAK = 0x8F,
	COMMAND_BYE = 0xF0,
	COMMAND_BYE_REPLY = 0xF1,
	COMMAND_UMP_DATA = 0xFF,
};

#define BUFSIZE 256

NET_BUF_POOL_DEFINE(udp_midi_pool, 10, BUFSIZE, 0, NULL);

#define SESS_LOG_DBG(_s, _fmt, ...) SESS_LOG(DBG, _s, _fmt, ##__VA_ARGS__)
#define SESS_LOG_INF(_s, _fmt, ...) SESS_LOG(INF, _s, _fmt, ##__VA_ARGS__)
#define SESS_LOG_WRN(_s, _fmt, ...) SESS_LOG(WRN, _s, _fmt, ##__VA_ARGS__)
#define SESS_LOG_ERR(_s, _fmt, ...) SESS_LOG(ERR, _s, _fmt, ##__VA_ARGS__)

#define SESS_LOG(_lvl, _s, _fmt, ...) \
	{ \
		const struct sockaddr_in *__pa = (const struct sockaddr_in *) &(_s)->addr; \
		char __pn[INET_ADDRSTRLEN]; \
		net_addr_ntop(AF_INET, &__pa->sin_addr, __pn, sizeof(__pn)); \
		LOG_##_lvl("%s:%d " _fmt, __pn, __pa->sin_port, ##__VA_ARGS__); \
	}

#define SESSION_HAS_STATE(session, expected_state) \
	((session) && (session)->state == expected_state)

static inline void udp_midi_free_session(struct udp_midi_session *session)
{
	SESS_LOG_INF(session, "Free client session");

	k_work_cancel(&session->tx_work);
	if (session->tx_buf) {
		net_buf_unref(session->tx_buf);
	}
	memset(session, 0, sizeof(*session) - sizeof(struct k_work));
}

static inline struct udp_midi_session *udp_midi_match_session(
	struct udp_midi_ep *ep,
	struct sockaddr *peer_addr,
	socklen_t peer_addr_len
)
{
	for (size_t i=0; i<ep->n_peers; i++) {
		if (
			ep->peers[i].addr_len == peer_addr_len &&
			memcmp(&ep->peers[i].addr, peer_addr, peer_addr_len) == 0
		) {
			LOG_DBG("Found matching client session %d", i);
			return &ep->peers[i];
		}
	}

	return NULL;
}

static inline void udp_midi_free_inactive_sessions(struct udp_midi_ep *ep)
{
	struct udp_midi_session *sess;
	const uint8_t bye_timeout[] = {COMMAND_BYE, 0, 0x04, 0};

	for (size_t i=0; i<ep->n_peers; i++) {
		sess = &ep->peers[i];
		if (! SESSION_HAS_STATE(sess, ESTABLISHED_SESSION)) {
			SESS_LOG_WRN(sess, "Cleanup inactive session");
			zsock_sendto(ep->pollsock.fd, bye_timeout, sizeof(bye_timeout),
				     0, &sess->addr, sess->addr_len);
			udp_midi_free_session(sess);
		}
	}
}

static inline struct udp_midi_session *udp_midi_try_alloc_session(
	struct udp_midi_ep *ep,
	struct sockaddr *peer_addr,
	socklen_t peer_addr_len
)
{
	struct udp_midi_session *sess;

	for (size_t i=0; i<ep->n_peers; i++) {
		sess = &ep->peers[i];
		if (sess->state == NOT_INITIALIZED) {
			sess->state = IDLE;
			sess->addr_len = peer_addr_len;
			sess->ep = ep;
			memcpy(&sess->addr, peer_addr, peer_addr_len);
			SESS_LOG_INF(sess, "new client session (%d)", i);
			return sess;
		}
	}

	return NULL;
}

static inline struct udp_midi_session *udp_midi_alloc_session(
	struct udp_midi_ep *ep,
	struct sockaddr *peer_addr,
	socklen_t peer_addr_len
)
{
	struct udp_midi_session *res = udp_midi_try_alloc_session(ep, peer_addr, peer_addr_len);
	if (! res) {
		udp_midi_free_inactive_sessions(ep);
		res = udp_midi_try_alloc_session(ep, peer_addr, peer_addr_len);
	}

	if (! res) {
		LOG_ERR("No available client session");
	}

	return res;
}

static inline const struct udp_midi_user *udp_midi_find_user(
	const struct udp_midi_ep *ep, const char *name, size_t namelen
)
{
	if (ep->auth_type != UDP_MIDI_USER_PASSWORD) {
		return NULL;
	}

	for (size_t i=0; i<ep->userlist->n_users; i++) {
		if (strncmp(ep->userlist->users[i].name, name, namelen) == 0) {
			return &ep->userlist->users[i];
		}
	}

	return NULL;
}

static inline bool udp_midi_auth_session(const struct udp_midi_session *sess,
					 struct net_buf_simple *buf)
{
	const struct device *hasher = device_get_binding(CONFIG_CRYPTO_MBEDTLS_SHIM_DRV_NAME);
	struct hash_ctx ctx = {.flags = crypto_query_hwcaps(hasher)};
	const uint8_t *auth_digest = buf->data;
	const struct udp_midi_user *user;
	uint8_t input[64];
	uint8_t output[32];
	size_t secret_len = strlen(sess->ep->shared_auth_secret);
	struct hash_pkt hash = {
		.in_buf = input,
		.in_len = UDP_MIDI_NONCE_SIZE,
		.out_buf = output,
	};

	/* Remove digest from buffer */
	net_buf_simple_pull(buf, 32);

	if (! hasher) {
		LOG_ERR("mbedtls crypto pseudo-device unavailable");
		return false;
	}

	memcpy(input, sess->nonce, UDP_MIDI_NONCE_SIZE);

	if (sess->ep->auth_type == UDP_MIDI_AUTH_SHARED_SECRET) {
		memcpy(&input[UDP_MIDI_NONCE_SIZE], sess->ep->shared_auth_secret, secret_len);
		hash.in_len += secret_len;
	} else if (sess->ep->auth_type == UDP_MIDI_USER_PASSWORD) {
		/* TODO: better handling of username length !
		 * It's actually NOT always the buffer length,
		 * as other command packets may follow */
		user = udp_midi_find_user(sess->ep, buf->data, buf->len);
		if (! user) {
			LOG_ERR("No matching user found");
			return false;
		}
		hash.in_len += snprintf(&input[UDP_MIDI_NONCE_SIZE],
					sizeof(input) - UDP_MIDI_NONCE_SIZE,
					"%s%s", user->name, user->password);
		/* Remove username from buffer */
		net_buf_simple_pull(buf, ROUND_UP(strlen(user->name), 4));
	}

	hash_begin_session(hasher, &ctx, CRYPTO_HASH_ALGO_SHA256);

	if (hash_compute(&ctx, &hash)) {
		SESS_LOG_ERR(sess, "Hashing error");
		return false;
	}

	hash_free_session(hasher, &ctx);

	return memcmp(hash.out_buf, auth_digest, 32) == 0;
}

static void udp_midi_session_tx_work(struct k_work *work)
{
	struct udp_midi_session *session = CONTAINER_OF(work, struct udp_midi_session, tx_work);
	struct net_buf *buf = session->tx_buf;
	session->tx_buf = NULL;

	zsock_sendto(session->ep->pollsock.fd, buf->data, buf->len, 0,
		     &session->addr, session->addr_len);
	net_buf_unref(buf);
}

static int udp_midi_dispatch_command_packet(struct udp_midi_ep *ep,
					    struct sockaddr *peer_addr,
					    socklen_t peer_addr_len,
					    struct net_buf_simple *rx,
					    struct net_buf_simple *tx)
{
	struct midi_ump ump;
	size_t payload_len;
	uint8_t cmd_code, payload_len_words;
	uint16_t cmd_data;
	struct udp_midi_session *session;

	if (rx->len < 4) {
		LOG_ERR("Incomplete UDP MIDI command packet header");
		return -1;
	}

	cmd_code = net_buf_simple_pull_u8(rx);
	payload_len_words = net_buf_simple_pull_u8(rx);
	payload_len = 4 * payload_len_words;
	cmd_data = net_buf_simple_pull_be16(rx);

	if (payload_len > rx->len) {
		LOG_ERR("Incomplete UDP MIDI command packet payload");
		return -1;
	}

	switch (cmd_code) {
	case COMMAND_PING:
		if (payload_len_words != 1) {
			LOG_ERR("Invalid payload length for PING packet");
			return -1;
		}
		net_buf_simple_add_u8(tx, COMMAND_PING_REPLY);
		net_buf_simple_add_u8(tx, 1);
		net_buf_simple_add_be16(tx, 0);
		/* PING id */
		net_buf_simple_add_be32(tx, net_buf_simple_pull_be32(rx));
		return 0;

	case COMMAND_INVITATION:
		net_buf_simple_pull(rx, payload_len);

		session = udp_midi_alloc_session(ep, peer_addr, peer_addr_len);
		if (! session) {
			return -1;
		}

		if (ep->auth_type == UDP_MIDI_AUTH_NONE) {
			net_buf_simple_add_u8(tx, COMMAND_INVITATION_REPLY_ACCEPTED);
			net_buf_simple_add_be24(tx, 0);
			session->state = ESTABLISHED_SESSION;
		} else {
			/* TODO: if client has no auth caps; send BYE with reason 0x45 */
			net_buf_simple_add_u8(tx, ep->auth_type == UDP_MIDI_AUTH_SHARED_SECRET
						  ? COMMAND_INVITATION_REPLY_AUTH_REQUIRED
						  : COMMAND_INVITATION_REPLY_USER_AUTH_REQUIRED);
			net_buf_simple_add_u8(tx, 4);
			net_buf_simple_add_be16(tx, 0);
			sys_rand_get(session->nonce, UDP_MIDI_NONCE_SIZE);
			net_buf_simple_add_mem(tx, session->nonce, UDP_MIDI_NONCE_SIZE);
			session->state = AUTHENTICATION_REQUIRED;
		}

		return 0;

	case COMMAND_INVITATION_WITH_AUTH:
	case COMMAND_INVITATION_WITH_USER_AUTH:
		session = udp_midi_match_session(ep, peer_addr, peer_addr_len);
		if (! SESSION_HAS_STATE(session, AUTHENTICATION_REQUIRED)) {
			LOG_WRN("No session to authenticate found");
			return -1;
		}

		if (payload_len_words < 8) {
			SESS_LOG_WRN(session, "Invalid auth digest length");
			return -1;
		}

		if (! udp_midi_auth_session(session, rx)) {
			SESS_LOG_WRN(session, "Invalid auth digest");
			return -1;
		}

		net_buf_simple_add_u8(tx, COMMAND_INVITATION_REPLY_ACCEPTED);
		net_buf_simple_add_be24(tx, 0);
		session->state = ESTABLISHED_SESSION;
		return 0;

	case COMMAND_BYE:
		session = udp_midi_match_session(ep, peer_addr, peer_addr_len);
		if (! session) {
			LOG_WRN("Receiving BYE without session");
			return -1;
		}
		net_buf_simple_pull(rx, payload_len);
		net_buf_simple_add_u8(tx, COMMAND_BYE_REPLY);
		net_buf_simple_add_be24(tx, 0);
		udp_midi_free_session(session);
		return 0;

	case COMMAND_UMP_DATA:
		session = udp_midi_match_session(ep, peer_addr, peer_addr_len);
		if (! SESSION_HAS_STATE(session, ESTABLISHED_SESSION)) {
			LOG_WRN("Receiving UMP data without established session");
			return -1;
		}

		if (session->rx_ump_seq == cmd_data) {
			session->rx_ump_seq++;
		} else {
			SESS_LOG_WRN(session, "UMP Rx sequence mismatch (got %d, expected %d)",
				     cmd_data, session->rx_ump_seq);
			session->rx_ump_seq = 1 + cmd_data;
		}

		if (payload_len_words < 1 || payload_len_words > 4) {
			SESS_LOG_ERR(session, "Invalid UMP length");
			return -1;
		}

		for (size_t i=0; i<payload_len_words; i++) {
			ump.data[i] = net_buf_simple_pull_be32(rx);
		}

		if (UMP_NUM_WORDS(ump) != payload_len_words) {
			SESS_LOG_ERR(session, "Invalid UMP payload size for its message type");
			return -1;
		}

		if (ep->rx_packet_cb){
			ep->rx_packet_cb(session, ump);
		}
		return 0;

	case COMMAND_SESSION_RESET:
		session = udp_midi_match_session(ep, peer_addr, peer_addr_len);
		if (! SESSION_HAS_STATE(session, ESTABLISHED_SESSION)) {
			LOG_WRN("Receiving session reset without established session");
			return -1;
		}

		session->tx_ump_seq = 0;
		session->rx_ump_seq = 0;
		SESS_LOG_INF(session, "Reset session");
		net_buf_simple_add_u8(tx, COMMAND_SESSION_RESET_REPLY);
		net_buf_simple_add_be24(tx, 0);
		return 0;

	default:
		LOG_WRN("Unknown command code %02X", cmd_code);
		net_buf_simple_pull(rx, payload_len);
		// TODO: send NAK "Command not supported"
		return 0;
	}
	return 0;
}

static void udp_midi_service_handler(struct net_socket_service_event *pev)
{
	int ret;
	struct udp_midi_ep *ep = pev->user_data;
	struct pollfd *pfd = &pev->event;
	struct sockaddr peer_addr;
	socklen_t peer_addr_len = sizeof(peer_addr);
	struct net_buf_simple *rxbuf = NET_BUF_SIMPLE(BUFSIZE);
	struct net_buf_simple *txbuf = NET_BUF_SIMPLE(BUFSIZE);

	net_buf_simple_init(rxbuf, 0);
	net_buf_simple_init(txbuf, 0);

	ret = zsock_recvfrom(pfd->fd, rxbuf->data, rxbuf->size, 0,
			     &peer_addr, &peer_addr_len);
	if (ret < 0) {
		LOG_ERR("Rx error: %d (%d)", ret, errno);
		return;
	}
	rxbuf->len = ret;

	LOG_HEXDUMP_DBG(rxbuf->data, rxbuf->len, "Received UDP packet");

	/* Check for magic header */
	if (rxbuf->len < 4 || memcmp(rxbuf->data, "MIDI", 4) != 0) {
		LOG_WRN("Not a MIDI packet");
		return;
	}

	net_buf_simple_pull(rxbuf, 4);
	net_buf_simple_add_mem(txbuf, "MIDI", 4);

	/* Parse contained command packets */
	while (
		rxbuf->len >= 4 &&
		udp_midi_dispatch_command_packet(ep, &peer_addr, peer_addr_len, rxbuf, txbuf) == 0
	);

	/* Send reply if non empty */
	if (txbuf->len > 4) {
		LOG_HEXDUMP_DBG(txbuf->data, txbuf->len, "Sending UDP packet...");
		ret = zsock_sendto(ep->pollsock.fd, txbuf->data, txbuf->len, 0, &peer_addr, peer_addr_len);
		if (ret < 0) {
			LOG_ERR("Unable to send reply: %d", errno);
		}
	}
}

NET_SOCKET_SERVICE_SYNC_DEFINE_STATIC(udp_midi_service, udp_midi_service_handler, 1);

int udp_midi_ep_init(struct udp_midi_ep *ep)
{
	int ret;
	int sock;
	memset(ep->peers, 0, ep->n_peers * sizeof(ep->peers[0]));
	memset(&ep->addr4, 0, sizeof(ep->addr4));
	ep->addr4.sin_family = AF_INET;

	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Unable to create socket: %d", errno);
		return -ENOMEM;
	}

	ret = zsock_bind(sock, (const struct sockaddr *) &ep->addr4, sizeof(ep->addr4));
	if (ret < 0) {
		zsock_close(sock);
		LOG_ERR("Failed to bind UDP socket: %d", errno);
		return -EIO;
	}

	for (size_t i=0; i<ep->n_peers; i++) {
		k_work_init(&ep->peers[i].tx_work, udp_midi_session_tx_work);
	}

	ep->pollsock.fd = sock;
	ep->pollsock.events = POLLIN;
	ret = net_socket_service_register(&udp_midi_service, &ep->pollsock, 1, ep);
	if (ret < 0) {
		zsock_close(sock);
		LOG_ERR("Failed to bind UDP socket: %d", errno);
		return -EIO;
	}

	LOG_INF("Started UDP-MIDI2 server");
	return 0;
}

void udp_midi_broadcast(struct udp_midi_ep *ep, const struct midi_ump ump)
{
	for (size_t i=0; i<ep->n_peers; i++) {
		if (ep->peers[i].state == ESTABLISHED_SESSION){
			udp_midi_send(&ep->peers[i], ump);
		}
	}
}

void udp_midi_send(struct udp_midi_session *sess, const struct midi_ump ump)
{
	if (! sess->tx_buf){
		sess->tx_buf = net_buf_alloc(&udp_midi_pool, K_FOREVER);
		net_buf_add_be32(sess->tx_buf, 0x4d494449);
	}

	if (net_buf_tailroom(sess->tx_buf) < 4*(1 + UMP_NUM_WORDS(ump))) {
		LOG_WRN("Not enough room in Tx buffer");
		return;
	}

	net_buf_add_u8(sess->tx_buf, COMMAND_UMP_DATA);
	net_buf_add_u8(sess->tx_buf, UMP_NUM_WORDS(ump));
	net_buf_add_be16(sess->tx_buf, sess->tx_ump_seq++);

	for (size_t i=0; i<UMP_NUM_WORDS(ump); i++){
		net_buf_add_be32(sess->tx_buf, ump.data[i]);
	}
	k_work_submit(&sess->tx_work);
}
