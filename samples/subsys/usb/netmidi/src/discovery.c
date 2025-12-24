/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/dns_resolve.h>

#include "discovery.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sample_usb_net_midi_discovery, LOG_LEVEL_INF);

struct disc_state {
	struct k_work work;
	bool discovery_enabled;
	uint16_t discovery_id;
	struct netmidi2_disc_ep srv;
	ep_found_cb cb;
};

static void dns_result_cb(enum dns_resolve_status status,
			  struct dns_addrinfo *info,
			  void *user_data)
{
	struct disc_state *state = user_data;

	if (status == DNS_EAI_ALLDONE) {
		LOG_INF("Resolution complete");
		state->cb(&state->srv);
	}

	if (status != DNS_EAI_INPROGRESS) {
		k_work_submit(&state->work);
	}
}

static void parse_srv_name(struct net_buf_simple *buf, struct netmidi2_disc_ep *srv)
{
	uint8_t c;
	char *dest = NULL;

	do {
		c = net_buf_simple_pull_u8(buf);
		switch (c) {
		case 0x00:
			if (dest) {*dest = '\x00';}
			return;
		case 0x04:
			if (dest) {*dest = '\x00';}
			dest = srv->name.proto;
			break;
		case 0x05:
			if (dest) {*dest = '\x00';}
			dest = srv->name.domain;
			break;
		case 0x06:
			if (dest) {*dest = '\x00';}
			dest = srv->name.service;
			break;
		case 0xC0:
			net_buf_simple_pull_u8(buf);
			return;
		default:
			if (dest) {*(dest++) = c;}
			break;
		}
	} while (c);
}

static int parse_dns_anwser(const uint8_t *dns_pkt, struct net_buf_simple *buf, struct netmidi2_disc_ep *srv)
{
	parse_srv_name(buf, srv);

	uint16_t type = net_buf_simple_pull_be16(buf);
	uint16_t flags = net_buf_simple_pull_be16(buf);
	uint32_t ttl = net_buf_simple_pull_be32(buf);
	uint16_t data_len = net_buf_simple_pull_be16(buf);
	uint8_t namelen;

	switch (type) {
	case DNS_QUERY_TYPE_A:
		if (data_len != 4) {
			LOG_ERR("Don't know what to do with A record with %d Bytes", data_len);
			return -1;
		}
		LOG_INF("Got IPv4 %d.%d.%d.%d",
			buf->data[0], buf->data[1], buf->data[2], buf->data[3]);
		srv->addr.sin_family = NET_AF_INET;
		memcpy(srv->addr.sin_addr.s4_addr, net_buf_simple_pull_mem(buf, 4), 4);
		srv->has_address = true;
		break;

	case DNS_QUERY_TYPE_SRV:
		if (data_len < 7) {
			LOG_ERR("Don't know what to do with SRV record with %d Bytes", data_len);
			return -1;
		}

		(void) net_buf_simple_pull_be16(buf);
		(void) net_buf_simple_pull_be16(buf);
		memcpy(&srv->addr.sin_port, net_buf_simple_pull_mem(buf, 2), 2);
		srv->has_port = true;
		LOG_INF("Got port %d", ntohs(srv->addr.sin_port));

		namelen = net_buf_simple_pull_u8(buf);
		if (namelen > data_len-7) {
			LOG_ERR("Invalid service name length");
			return -1;
		}

		memcpy(srv->name.host, net_buf_simple_pull_mem(buf, namelen), namelen);
		srv->name.host[namelen] = '\x00';
		LOG_INF("Got service name '%s'", srv->name.host);
		net_buf_simple_pull(buf, data_len - namelen - 7);
		break;

	default:
		net_buf_simple_pull(buf, data_len);
		break;
	}

	return 0;
}

static void parse_dns_data(struct net_buf *dns_data, size_t buf_len, void *user_data)
{
	struct net_buf_simple buf;
	net_buf_simple_init_with_data(&buf, dns_data->data, buf_len);

	if (buf_len < 12) {
		LOG_ERR("DNS answer too short");
		return;
	}

	uint16_t transaction_id = net_buf_simple_pull_be16(&buf);
	uint16_t flags = net_buf_simple_pull_be16(&buf);
	uint16_t questions = net_buf_simple_pull_be16(&buf);
	uint16_t answer_rrs = net_buf_simple_pull_be16(&buf);
	uint16_t authority_rrs = net_buf_simple_pull_be16(&buf);
	uint16_t additional_rrs = net_buf_simple_pull_be16(&buf);

	struct disc_state *state = user_data;
	while (buf.len > 0 && parse_dns_anwser(dns_data->data, &buf, &state->srv) == 0);
}

static void discovery_work_fn(struct k_work *work)
{
	struct disc_state *state = CONTAINER_OF(work, struct disc_state, work);

	if (! state->discovery_enabled) {
		return;
	}

	struct dns_resolve_context *resolver = dns_resolve_get_default();
	dns_resolve_enable_packet_forwarding(resolver, parse_dns_data);
	memset(&state->srv, 0, sizeof(struct netmidi2_disc_ep));
	dns_resolve_service(resolver, "_midi2._udp.local", NULL,
			    dns_result_cb, state, 2*MSEC_PER_SEC);
}

static struct disc_state global_disc_state;

void start_discovery(ep_found_cb cb)
{
	if (global_disc_state.discovery_enabled) {
		LOG_INF("Discovery is already started");
		return;
	}

	LOG_INF("Starting discovery");
	global_disc_state.cb = cb;
	global_disc_state.discovery_enabled = true;
	k_work_init(&global_disc_state.work, discovery_work_fn);
	k_work_submit(&global_disc_state.work);
}

void stop_discovery()
{
	LOG_INF("Stopping discovery");
	global_disc_state.discovery_enabled = false;
	k_work_cancel(&global_disc_state.work);
}

bool is_discovery_started()
{
	return global_disc_state.discovery_enabled;
}
