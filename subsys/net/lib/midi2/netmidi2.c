#include <zephyr/net/midi2.h>

#include <stdio.h>
#include <zephyr/net/socket_service.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_midi2, CONFIG_NET_MIDI2_LOG_LEVEL);

#define BUFSIZE 256

/* See udp-ump 5.5: Command Codes and Packet Types */
#define COMMAND_INVITATION				0x01
#define COMMAND_INVITATION_WITH_AUTH			0x02
#define COMMAND_INVITATION_WITH_USER_AUTH		0x03
#define COMMAND_INVITATION_REPLY_ACCEPTED		0x10
#define COMMAND_INVITATION_REPLY_PENDING		0x11
#define COMMAND_INVITATION_REPLY_AUTH_REQUIRED		0x12
#define COMMAND_INVITATION_REPLY_USER_AUTH_REQUIRED	0x13
#define COMMAND_PING					0x20
#define COMMAND_PING_REPLY				0x21
#define COMMAND_RETRANSMIT_REQUEST			0x80
#define COMMAND_RETRANSMIT_ERROR			0x81
#define COMMAND_SESSION_RESET				0x82
#define COMMAND_SESSION_RESET_REPLY			0x83
#define COMMAND_NAK					0x8F
#define COMMAND_BYE					0xF0
#define COMMAND_BYE_REPLY				0xF1
#define COMMAND_UMP_DATA				0xFF

/* See udp-ump 6.4 / Table 11: Capabilities for Invitation */
#define CLIENT_CAP_INV_WITH_AUTH	BIT(0)
#define CLIENT_CAP_INV_WITH_USER_AUTH	BIT(1)

/* See udp-ump 6.15 / Table 25: List of NAK Reasons */
#define NAK_OTHER			0x00
#define NAK_COMMAND_NOT_SUPPORTED	0x01
#define NAK_COMMAND_NOT_EXPECTED	0x02
#define NAK_COMMAND_MALFORMED		0x03
#define NAK_BAD_PING_REPLY		0x20

#define SESS_LOG_DBG(_s, _fmt, ...) SESS_LOG(DBG, _s, _fmt, ##__VA_ARGS__)
#define SESS_LOG_INF(_s, _fmt, ...) SESS_LOG(INFO, _s, _fmt, ##__VA_ARGS__)
#define SESS_LOG_WRN(_s, _fmt, ...) SESS_LOG(WARN, _s, _fmt, ##__VA_ARGS__)
#define SESS_LOG_ERR(_s, _fmt, ...) SESS_LOG(ERR, _s, _fmt, ##__VA_ARGS__)

#define SESS_LOG(_lvl, _s, _fmt, ...) \
	{ \
		const struct sockaddr_in *__pa = (const struct sockaddr_in *) &(_s)->addr; \
		char __pn[INET6_ADDRSTRLEN]; \
		net_addr_ntop(__pa->sin_family, &__pa->sin_addr, __pn, sizeof(__pn)); \
		NET_##_lvl("%s:%d " _fmt, __pn, __pa->sin_port, ##__VA_ARGS__); \
	}

#define SESSION_HAS_STATE(session, expected_state) \
	((session) && (session)->state == expected_state)

NET_BUF_POOL_DEFINE(udp_midi_pool, 10, BUFSIZE, 0, NULL);

#if CONFIG_MIDI2_UDP_HOST_AUTH
#include <zephyr/crypto/crypto.h>
#include <zephyr/random/random.h>

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

static bool udp_midi_auth_session(const struct udp_midi_session *sess,
				  struct net_buf *buf)
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

	if (! hasher) {
		LOG_ERR("mbedtls crypto pseudo-device unavailable");
		return false;
	}

	/* Remove leading auth_digest from buffer */
	net_buf_pull(buf, 32);

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
		net_buf_pull(buf, ROUND_UP(strlen(user->name), 4));
	}

	hash_begin_session(hasher, &ctx, CRYPTO_HASH_ALGO_SHA256);

	if (hash_compute(&ctx, &hash)) {
		SESS_LOG_ERR(sess, "Hashing error");
		return false;
	}

	hash_free_session(hasher, &ctx);

	return memcmp(hash.out_buf, auth_digest, 32) == 0;
}
#endif /* CONFIG_MIDI2_UDP_HOST_AUTH */

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

static void udp_midi_session_tx_work(struct k_work *work)
{
	struct udp_midi_session *session = CONTAINER_OF(work, struct udp_midi_session, tx_work);
	struct net_buf *buf = session->tx_buf;
	session->tx_buf = NULL;

	zsock_sendto(session->ep->pollsock.fd, buf->data, buf->len, 0,
		     &session->addr, session->addr_len);
	net_buf_unref(buf);
}

static inline int udp_midi_session_cmdheader(struct udp_midi_session *sess,
					     const uint8_t command_code,
					     const uint16_t command_specific_data,
					     const uint8_t payload_len_words)
{
	if (! sess->tx_buf){
		sess->tx_buf = net_buf_alloc(&udp_midi_pool, K_FOREVER);
		if (! sess->tx_buf) {
			SESS_LOG_ERR(sess, "Unable to allocate Tx buffer");
			return -ENOBUFS;
		}
		net_buf_add_mem(sess->tx_buf, "MIDI", 4);
	}

	if (net_buf_tailroom(sess->tx_buf) < 4*(1 + payload_len_words)) {
		SESS_LOG_WRN(sess, "Not enough room in Tx buffer");
		return -ENOMEM;
	}

	net_buf_add_u8(sess->tx_buf, command_code);
	net_buf_add_u8(sess->tx_buf, payload_len_words);
	net_buf_add_be16(sess->tx_buf, command_specific_data);

	return 0;
}

/**
 * @brief      Send a Command Packet to a client session
 * @param      sess                   The recipient session
 * @param[in]  command_code           The command code
 * @param[in]  command_specific_data  The command specific data
 * @param[in]  payload                The command payload
 * @param[in]  payload_len_words      Payload length, in words (4B)
 * @return     0 on sucess, -errno otherwise
 *
 * @see udp-ump 5.4 Command Packet Header and Payload
 */
static int udp_midi_session_sendcmd(struct udp_midi_session *sess,
			            const uint8_t command_code,
			            const uint16_t command_specific_data,
			            const uint32_t *payload,
			            const uint8_t payload_len_words)
{
	int ret = udp_midi_session_cmdheader(sess, command_code,
					     command_specific_data,
					     payload_len_words);
	if (ret) {
		return ret;
	}

	for (size_t i=0; i<payload_len_words; i++){
		net_buf_add_be32(sess->tx_buf, payload[i]);
	}
	k_work_submit(&sess->tx_work);
	return 0;
}

static int udp_midi_session_sendcmd_u8(struct udp_midi_session *sess,
			               const uint8_t command_code,
			               const uint16_t command_specific_data,
			               const uint8_t *payload,
			               const uint8_t payload_len)
{
	uint8_t payload_len_words = DIV_ROUND_UP(payload_len, 4);
	int ret = udp_midi_session_cmdheader(sess, command_code,
					     command_specific_data,
					     payload_len_words);
	if (ret) {
		return ret;
	}

	net_buf_add_mem(sess->tx_buf, payload, payload_len);
	switch (payload_len % 4) {
	case 1: net_buf_add_be24(sess->tx_buf, 0); break;
	case 2: net_buf_add_be16(sess->tx_buf, 0); break;
	case 3: net_buf_add_u8(sess->tx_buf, 0); break;
	}

	k_work_submit(&sess->tx_work);
	return 0;
}

/**
 * @brief      Immediately send a Command Packet to a remote without client session
 * @param[in]  ep                     The emitting UMP endpoint
 * @param[in]  peer_addr              The recipient's address
 * @param[in]  peer_addr_len          The recipient's address length
 * @param[in]  command_specific_data  The command specific data
 * @param[in]  payload                The command payload
 * @param[in]  payload_len_words      Payload length, in words (4B)
 */
static inline int udp_midi_quick_reply(const struct udp_midi_ep *ep,
				       const struct sockaddr *peer_addr,
				       const socklen_t peer_addr_len,
				       const uint8_t command_code,
			               const uint16_t command_specific_data,
			               const uint32_t *payload,
			               const uint8_t payload_len_words)
{
	NET_BUF_SIMPLE_DEFINE(txbuf, 24);

	if (4 * (1 + payload_len_words) > txbuf.size) {
		return -ENOBUFS;
	}

	net_buf_simple_add_u8(&txbuf, command_code);
	net_buf_simple_add_u8(&txbuf, payload_len_words);
	net_buf_simple_add_be16(&txbuf, command_specific_data);

	for (size_t i=0; i<payload_len_words; i++){
		net_buf_simple_add_be32(&txbuf, payload[i]);
	}

	zsock_sendto(ep->pollsock.fd, txbuf.data, txbuf.len, 0,
		     peer_addr, peer_addr_len);
	return 0;
}


static inline int udp_midi_quick_nak(const struct udp_midi_ep *ep,
				       const struct sockaddr *peer_addr,
				       const socklen_t peer_addr_len,
				       const uint8_t nak_reason,
			               const uint32_t nakd_cmd_header)
{
	return udp_midi_quick_reply(ep, peer_addr, peer_addr_len,
				    COMMAND_NAK, nak_reason << 8,
				    &nakd_cmd_header, 1);
}

static int udp_midi_dispatch_command_packet(struct udp_midi_ep *ep,
					    struct sockaddr *peer_addr,
					    socklen_t peer_addr_len,
					    struct net_buf *rx)
{
	struct midi_ump ump;
	size_t payload_len;
	uint32_t cmd_header;
	uint8_t cmd_code, payload_len_words;
	uint16_t cmd_data;
	struct udp_midi_session *session;

	if (rx->len < 4) {
		LOG_ERR("Incomplete UDP MIDI command packet header");
		return -1;
	}

	cmd_header = net_buf_pull_be32(rx);
	cmd_code = cmd_header >> 24;
	payload_len_words = (cmd_header >> 16) & 0xff;
	payload_len = 4 * payload_len_words;
	cmd_data = cmd_header & 0xffff;

	if (payload_len > rx->len) {
		udp_midi_quick_nak(ep, peer_addr, peer_addr_len,
				   NAK_COMMAND_MALFORMED, cmd_header);
		LOG_ERR("Incomplete UDP MIDI command packet payload");
		return -1;
	}

	switch (cmd_code) {
	case COMMAND_PING:
		if (payload_len_words != 1) {
			udp_midi_quick_nak(ep, peer_addr, peer_addr_len,
					   NAK_COMMAND_MALFORMED, cmd_header);
			LOG_ERR("Invalid payload length for PING packet");
			return -1;
		}
		udp_midi_quick_reply(ep, peer_addr, peer_addr_len,
				     COMMAND_PING_REPLY, 0,
				     (uint32_t [1]) {net_buf_pull_be32(rx)}, 1);
		return 0;

	case COMMAND_INVITATION:
		net_buf_pull(rx, payload_len);

		session = udp_midi_alloc_session(ep, peer_addr, peer_addr_len);
		if (! session) {
			return -1;
		}

		if (ep->auth_type == UDP_MIDI_AUTH_NONE) {
			udp_midi_session_sendcmd(session, COMMAND_INVITATION_REPLY_ACCEPTED,
						 0, NULL, 0);
			session->state = ESTABLISHED_SESSION;
		}

#if ! CONFIG_MIDI2_UDP_HOST_AUTH
		return 0;
#else
		else {
			/* TODO: if client has no auth caps; send BYE with reason 0x45 */
			sys_rand_get(session->nonce, UDP_MIDI_NONCE_SIZE);
			udp_midi_session_sendcmd_u8(
				session,
				ep->auth_type == UDP_MIDI_AUTH_SHARED_SECRET
					? COMMAND_INVITATION_REPLY_AUTH_REQUIRED
					: COMMAND_INVITATION_REPLY_USER_AUTH_REQUIRED,
				0,
				session->nonce,
				UDP_MIDI_NONCE_SIZE
			);
			session->state = AUTHENTICATION_REQUIRED;
		}

		return 0;

	case COMMAND_INVITATION_WITH_AUTH:
	case COMMAND_INVITATION_WITH_USER_AUTH:
		session = udp_midi_match_session(ep, peer_addr, peer_addr_len);
		if (! SESSION_HAS_STATE(session, AUTHENTICATION_REQUIRED)) {
			udp_midi_quick_nak(ep, peer_addr, peer_addr_len,
					   NAK_COMMAND_NOT_EXPECTED, cmd_header);
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

		udp_midi_session_sendcmd(session, COMMAND_INVITATION_REPLY_ACCEPTED,
					 0, NULL, 0);
		session->state = ESTABLISHED_SESSION;
		return 0;
#endif /* CONFIG_MIDI2_UDP_HOST_AUTH */

	case COMMAND_BYE:
		session = udp_midi_match_session(ep, peer_addr, peer_addr_len);
		if (! session) {
			udp_midi_quick_nak(ep, peer_addr, peer_addr_len,
					   NAK_COMMAND_NOT_EXPECTED, cmd_header);
			LOG_WRN("Receiving BYE without session");
			return -1;
		}
		net_buf_pull(rx, payload_len);
		udp_midi_quick_reply(ep, peer_addr, peer_addr_len,
				     COMMAND_BYE_REPLY, 0, NULL, 0);
		udp_midi_free_session(session);
		return 0;

	case COMMAND_UMP_DATA:
		session = udp_midi_match_session(ep, peer_addr, peer_addr_len);
		if (! SESSION_HAS_STATE(session, ESTABLISHED_SESSION)) {
			udp_midi_quick_nak(ep, peer_addr, peer_addr_len,
					   NAK_COMMAND_NOT_EXPECTED, cmd_header);
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
			udp_midi_quick_nak(ep, peer_addr, peer_addr_len,
					   NAK_COMMAND_MALFORMED, cmd_header);
			SESS_LOG_ERR(session, "Invalid UMP length");
			return -1;
		}

		for (size_t i=0; i<payload_len_words; i++) {
			ump.data[i] = net_buf_pull_be32(rx);
		}

		if (UMP_NUM_WORDS(ump) != payload_len_words) {
			udp_midi_quick_nak(ep, peer_addr, peer_addr_len,
					   NAK_COMMAND_MALFORMED, cmd_header);
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
			udp_midi_quick_nak(ep, peer_addr, peer_addr_len,
					   NAK_COMMAND_NOT_EXPECTED, cmd_header);
			return -1;
		}

		session->tx_ump_seq = 0;
		session->rx_ump_seq = 0;
		SESS_LOG_INF(session, "Reset session");
		udp_midi_session_sendcmd(session, COMMAND_SESSION_RESET_REPLY, 0, NULL, 0);
		return 0;

	default:
		LOG_WRN("Unknown command code %02X", cmd_code);
		net_buf_pull(rx, payload_len);
		udp_midi_quick_nak(ep, peer_addr, peer_addr_len,
				   NAK_COMMAND_NOT_SUPPORTED, cmd_header);
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
	struct net_buf *rxbuf;

	rxbuf = net_buf_alloc(&udp_midi_pool, K_FOREVER);
	if (! rxbuf) {
		NET_ERR("Cannot allocate Rx buf");
		return;
	}

	ret = zsock_recvfrom(pfd->fd, rxbuf->data, rxbuf->size, 0,
			     &peer_addr, &peer_addr_len);
	if (ret < 0) {
		LOG_ERR("Rx error: %d (%d)", ret, errno);
		goto end;
	}
	rxbuf->len = ret;

	NET_HEXDUMP_DBG(rxbuf->data, rxbuf->len, "Received UDP packet");

	/* Check for magic header */
	if (rxbuf->len < 4 || memcmp(rxbuf->data, "MIDI", 4) != 0) {
		LOG_WRN("Not a MIDI packet");
		goto end;
	}

	net_buf_pull(rxbuf, 4);

	/* Parse contained command packets */
	while (
		rxbuf->len >= 4 &&
		udp_midi_dispatch_command_packet(ep, &peer_addr, peer_addr_len, rxbuf) == 0
	);

	end:
	net_buf_unref(rxbuf);
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

	LOG_INF("Started UDP-MIDI2 server (%d)", ep->addr4.sin_port);
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
	udp_midi_session_sendcmd(sess, COMMAND_UMP_DATA, sess->tx_ump_seq++,
				 ump.data, UMP_NUM_WORDS(ump));
}
