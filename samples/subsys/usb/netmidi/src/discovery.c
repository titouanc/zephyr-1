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
	bool cb_called = false;
	uint16_t discovery_id;
	struct netmidi2_disc_ep srv;
	ep_found_cb cb;
};

void extract_dns_info(struct dns_addrinfo *info, struct netmidi2_disc_ep *srv)
{
	if (info->ai_family == NET_AF_INET) {
		uint16_t port = srv->addr.sin_port;
		memcpy(&srv->addr, &info->ai_addr, info->ai_addrlen);
		srv->addr.sin_port = port;
		srv->has_address = true;
	} else if (info->ai_family == AF_UNSPEC) {
		if (info->ai_extension == DNS_RESOLVE_SRV) {
			srv->addr.sin_port = htons(info->ai_srv.port);
			memcpy(srv->target, info->ai_srv.target, info->ai_srv.targetlen);
			srv->has_port = true;
		}
	}
}

static void dns_result_cb(enum dns_resolve_status status,
			  struct dns_addrinfo *info,
			  void *user_data)
{
	struct disc_state *state = user_data;

	LOG_INF("DNS result callback status=%d", status);

	if (status == DNS_EAI_INPROGRESS) {
		extract_dns_info(info, &state->srv);
	}

	if (status == DNS_EAI_ALLDONE || (state->srv.has_address && state->srv.has_port)) {
		if (state->cb && ! state->cb_called) {
			state->cb(&state->srv);
		}
		state->cb_called = true;
	}

	if (status != DNS_EAI_INPROGRESS) {
		k_work_submit(&state->work);
	}
}

static void discovery_work_fn(struct k_work *work)
{
	struct disc_state *state = CONTAINER_OF(work, struct disc_state, work);
	struct dns_resolve_context *resolver = dns_resolve_get_default();

	if (! state->discovery_enabled) {
		return;
	}

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
