#include "netmidi2.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udp_midi, LOG_LEVEL_DBG);

#define BUFSIZE 256

static inline void udp_midi_session_free(struct udp_midi_session *session)
{
	memset(session, 0, sizeof(struct udp_midi_session));
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

static inline struct udp_midi_session *udp_midi_alloc_session(
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
			LOG_DBG("Allocated new client session %d", i);
			return sess;
		}
	}

	LOG_ERR("Not any free slot for a new client session");
	return NULL;
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
		/* Peer EP name & product instance ID */
		net_buf_simple_pull(rx, payload_len_words);
		net_buf_simple_add_u8(tx, COMMAND_INVITATION_REPLY_ACCEPTED);
		net_buf_simple_add_le24(tx, 0);
		session = udp_midi_alloc_session(ep, peer_addr, peer_addr_len);
		session->state = ESTABLISHED_SESSION;
		return 0;

	case COMMAND_UMP_DATA:
		session = udp_midi_match_session(ep, peer_addr, peer_addr_len);
		if (! session) {
			LOG_WRN("Receiving UMP data without an active session");
			return -1;
		}

		if (payload_len_words < 1 || payload_len_words > 4) {
			LOG_ERR("Invalid UMP length");
			return -1;
		}

		for (size_t i=0; i<payload_len_words; i++) {
			ump.data[i] = net_buf_simple_pull_be32(rx);
		}

		if (UMP_NUM_WORDS(ump) != payload_len_words) {
			LOG_ERR("Invalid UMP payload size for its message type");
			return -1;
		}

		if (ep->rx_packet_cb){
			ep->rx_packet_cb(session, ump);
		}
		return 0;

	default:
		LOG_WRN("Unknown command code %02X", cmd_code);
		net_buf_simple_pull(rx, payload_len);
		return 0;
	}
	return 0;
}

static void udp_midi_rx_work(struct k_work *work)
{
	int ret;
	struct sockaddr peer_addr;
	socklen_t peer_addr_len = sizeof(peer_addr);
	struct udp_midi_ep *ep = CONTAINER_OF(work, struct udp_midi_ep, rx_work);
	struct net_buf_simple *rxbuf = NET_BUF_SIMPLE(BUFSIZE);
	struct net_buf_simple *txbuf = NET_BUF_SIMPLE(BUFSIZE);

	net_buf_simple_init(rxbuf, 0);
	net_buf_simple_init(txbuf, 0);

	/* Receive packet */
	LOG_DBG("Waiting for new UDP packet...");

	ret = zsock_recvfrom(ep->sock, rxbuf->data, rxbuf->size, 0, &peer_addr, &peer_addr_len);
	if (ret < 0) {
		LOG_ERR("Rx error: %d (%d)", ret, errno);
		goto end;
	}
	rxbuf->len = ret;

	LOG_HEXDUMP_DBG(rxbuf->data, rxbuf->len, "Received UDP packet");

	/* Check for magic header */
	if (rxbuf->len < 4 || memcmp(rxbuf->data, "MIDI", 4) != 0) {
		LOG_WRN("Not a MIDI packet");
		goto end;
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
		ret = zsock_sendto(ep->sock, txbuf->data, txbuf->len, 0, &peer_addr, peer_addr_len);
		if (ret < 0) {
			LOG_ERR("Unable to send reply: %d", errno);
		}
	}

	end:
		k_work_submit(&ep->rx_work);
}

int udp_midi_ep_start(struct udp_midi_ep *ep,
		      const struct sockaddr *addr, socklen_t addr_len)
{
	int ret;
	memset(ep->peers, 0, ep->n_peers * sizeof(ep->peers[0]));

	ep->sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (ep->sock < 0) {
		LOG_ERR("Unable to create socket: %d", errno);
		return -1;
	}

	ret = zsock_bind(ep->sock, addr, addr_len);
	if (ret < 0) {
		LOG_ERR("Failed to bind UDP socket: %d", errno);
		ret = -errno;
	}

	k_work_init(&ep->rx_work, udp_midi_rx_work);
	k_work_submit(&ep->rx_work);

	return ret;
}

void udp_midi_broadcast(struct udp_midi_ep *ep, const struct midi_ump ump)
{
	uint8_t buf[8 + sizeof(ump)] = "MIDI";
	buf[4] = COMMAND_UMP_DATA;
	buf[5] = UMP_NUM_WORDS(ump);
	uint32_t *buf32 = (uint32_t *) &buf[8];
	for (size_t i=0; i<UMP_NUM_WORDS(ump); i++) {
		buf32[i] = sys_cpu_to_be32(ump.data[i]);
	}

	for (size_t i=0; i<ep->n_peers; i++) {
		if (ep->peers[i].state != ESTABLISHED_SESSION){
			continue;
		}
		LOG_HEXDUMP_DBG(buf, 8+4*UMP_NUM_WORDS(ump), "Send UDP");
		zsock_sendto(ep->sock, buf, 8+4*UMP_NUM_WORDS(ump), 0,
			     &ep->peers[i].addr, ep->peers[i].addr_len);
	}
}

void udp_midi_send(struct udp_midi_session *sess, const struct midi_ump ump)
{
	uint8_t buf[8 + sizeof(ump)] = "MIDI";
	uint32_t *buf32 = (uint32_t *) &buf[8];

	if (sess->state != ESTABLISHED_SESSION){
		LOG_WRN("Attempting to send data on an un-established session");
		return;
	}

	buf[4] = COMMAND_UMP_DATA;
	buf[5] = UMP_NUM_WORDS(ump);
	for (size_t i=0; i<UMP_NUM_WORDS(ump); i++) {
		buf32[i] = sys_cpu_to_be32(ump.data[i]);
	}

	LOG_HEXDUMP_DBG(buf, 8+4*UMP_NUM_WORDS(ump), "Send UDP");
	zsock_sendto(sess->ep->sock, buf, 8+4*UMP_NUM_WORDS(ump), 0,
		     &sess->addr, sess->addr_len);
}
