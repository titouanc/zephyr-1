#include "netmidi2.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udp_midi, LOG_LEVEL_DBG);

#define BUFSIZE 256

static int udp_midi_dispatch_command_packet(struct udp_midi_ep *ep, const uint8_t *cmd_pkt, size_t cmd_pkt_len, uint8_t *reply, size_t reply_len)
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
		if (cmd_pkt[1] < 1 || cmd_pkt[1] > 4) {
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

		if (ep->rx_packet_cb){
			ep->rx_packet_cb(ep, ump);
		}
		return 0;

	default:
		LOG_WRN("Unknown command code %02X", cmd_pkt[0]);
		return 0;
	}
	return 0;
}

static void udp_midi_rx_do_work(struct k_work *work)
{
	int ret;
	size_t rx_len, rx_off, tx_off, cmd_pkt_len;
	uint8_t rx_buf[BUFSIZE], tx_buf[BUFSIZE] = "MIDI";
	struct sockaddr peer_addr;
	socklen_t peer_addr_len;
	struct udp_midi_ep *ep = CONTAINER_OF(work, struct udp_midi_ep, rx_work);

	LOG_DBG("Waiting for new UDP packet...");

	ret = zsock_recvfrom(ep->sock, rx_buf, sizeof(rx_buf), 0, &peer_addr, &peer_addr_len);
	if (ret <= 0) {
		LOG_ERR("Rx error: %d (%d)", ret, errno);
		goto end;
	}

	rx_len = ret;
	LOG_HEXDUMP_DBG(rx_buf, ret, "Received UDP packet");

	if (rx_len % 4) {
		LOG_ERR("Misaligned packet, length is not a multiple of 4");
		goto end;
	}

	if (rx_len < 4 || memcmp(rx_buf, "MIDI", 4) != 0) {
		LOG_WRN("Not a MIDI packet");
	}

	rx_off = tx_off = 4;
	while (rx_off < rx_len) {
		cmd_pkt_len = 4 * rx_buf[rx_off + 1];
		if (rx_off + 4 + cmd_pkt_len > rx_len) {
			LOG_ERR("Incomplete command packet");
			goto end;
		}

		ret = udp_midi_dispatch_command_packet(ep, &rx_buf[rx_off], 4 + cmd_pkt_len, &tx_buf[tx_off], sizeof(tx_buf) - tx_off);
		if (ret < 0) {
			break;
		}

		tx_off += ret;
		rx_off += 4 + cmd_pkt_len;
	}

	if (tx_off > 4) {
		ret = zsock_sendto(ep->sock, tx_buf, tx_off, 0, &peer_addr, peer_addr_len);
		if (ret < 0) {
			LOG_ERR("Unable to send reply: %d", errno);
		}
	}

	end: k_work_submit(&ep->rx_work);
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

	k_work_init(&ep->rx_work, udp_midi_rx_do_work);
	k_work_submit(&ep->rx_work);

	return ret;
}

void udp_midi_send(struct udp_midi_ep *ep, const struct midi_ump ump)
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
		zsock_sendto(ep->sock, buf, 8+4*UMP_NUM_WORDS(ump), 0,
			     &ep->peers[i].addr, ep->peers[i].addr_len);
	}
	LOG_WRN("Not implemented !");
}
