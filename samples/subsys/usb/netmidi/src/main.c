/*
 * Copyright (c) 2025 Titouan Christophe
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/net/midi2.h>
#include <zephyr/shell/shell.h>
#include <zephyr/usb/class/usbd_midi2.h>

#include <sample_usbd.h>
#include "discovery.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(unmidi2, LOG_LEVEL_INF);

static struct usbd_context *sample_usbd = NULL;
static const struct device *const usb_midi = DEVICE_DT_GET(DT_NODELABEL(usb_midi));

static struct netmidi2_session *established_session = NULL;
NETMIDI2_EP_CLIENT_DEFINE(net_midi, "Zephyr MIDI USB<>Network", NULL, 0);

static bool autodiscover = true;

static void rx_usb_midi(const struct device *dev, const struct midi_ump ump)
{
	if (established_session != NULL) {
		netmidi2_send(established_session, ump);
	}
}

static void endpoint_found(const struct netmidi2_disc_ep *srv)
{
	if (! srv->has_address) {
		LOG_WRN("No IP address for endpoint %s", srv->target);
		return;
	}

	if (! srv->has_port) {
		LOG_WRN("No port for endpoint %s", srv->target);
		return;
	}

	LOG_INF("Found %s @ %d.%d.%d.%d:%d",
		srv->target,
		srv->addr.sin_addr.s4_addr[0], srv->addr.sin_addr.s4_addr[1],
		srv->addr.sin_addr.s4_addr[2], srv->addr.sin_addr.s4_addr[3],
		ntohs(srv->addr.sin_port));

	stop_discovery();
	netmidi2_ep_invite(&net_midi, (struct net_sockaddr *) &srv->addr,
			   sizeof(struct net_sockaddr_in));
}

static void rx_net_midi(struct netmidi2_session *session, const struct midi_ump ump)
{
	usbd_midi_send(usb_midi, ump);
}

static void session_established(struct netmidi2_session *session)
{
	established_session = session;
	if (usbd_enable(sample_usbd)) {
		LOG_ERR("Failed to enable USB");
	} else {
		LOG_INF("USB enabled");
	}
}

static void session_closed(struct netmidi2_session *session)
{
	if (usbd_disable(sample_usbd)) {
		LOG_ERR("Failed to disable USB");
	} else {
		LOG_INF("USB disabled");
	}
	established_session = NULL;
	if (autodiscover) {
		start_discovery(endpoint_found);
	}
}

int main(void)
{
	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -1;
	}

	usbd_midi_set_ops(usb_midi, &((const struct usbd_midi_ops) {
		.rx_packet_cb = rx_usb_midi,
	}));

	netmidi2_set_ops(&net_midi, &((const struct netmidi2_ops) {
		.rx_packet_cb = rx_net_midi,
		.session_established_cb = session_established,
		.session_closed_cb = session_closed,
	}));
	netmidi2_host_ep_start(&net_midi);

	start_discovery(endpoint_found);

	return 0;
}

int cmd_midi2_discover(const struct shell *sh, int argc, char *argv[])
{
	if (argc > 1) {
		if (argv[1][0] == '0') {
			stop_discovery();
		} else if (argv[1][0] == '1') {
			start_discovery(endpoint_found);
		} else {
			shell_print(sh, "Usage: %s [ 0/1 ]\n", argv[0]);
			return -ENOEXEC;
		}
	}
	shell_print(sh, "Discovery %s",
		    is_discovery_started() ? "started" : "stopped");
	return 0;
}

int cmd_midi2_autodiscover(const struct shell *sh, int argc, char *argv[])
{
	if (argc > 1) {
		autodiscover = (argv[1][0] == '1');
	}
	shell_print(sh, "Auto-Discovery %s", autodiscover ? "enabled" : "disabled");
	return 0;
}

int cmd_midi2_current(const struct shell *sh, int argc, char *argv[])
{
	if (established_session == NULL) {
		shell_print(sh, "No established session");
	} else {
		struct net_sockaddr *addr = net_sad(&established_session->addr);
		struct net_sockaddr_in6 *addr6 = net_sin6(addr);
		char peer[NET_INET6_ADDRSTRLEN];

		net_addr_ntop(addr->sa_family, &addr6->sin6_addr, peer, sizeof(peer));
		shell_print(sh, "Session established to %s:%d", peer, ntohs(addr6->sin6_port));
	}

	return 0;
}

int cmd_midi2_bye(const struct shell *sh, int argc, char *argv[])
{
	if (established_session == NULL) {
		shell_print(sh, "No established session");
	} else {
		netmidi2_session_bye(established_session);
	}

	return 0;
}

/* Creating subcommands (level 1 command) array for command "demo". */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_midi2,
	SHELL_CMD(discover, NULL,
		  "Discover MIDI2.0 endpoints on the local network",
		  cmd_midi2_discover),
	SHELL_CMD(autodiscover, NULL,
		  "Discover MIDI2.0 endpoints on the local network",
		  cmd_midi2_autodiscover),
	SHELL_CMD(current, NULL,
		  "Print current session (if any)",
		  cmd_midi2_current),
	SHELL_CMD(bye, NULL,
		  "Bye (disconnect) current session",
		  cmd_midi2_bye),
	SHELL_SUBCMD_SET_END
);

/* Creating root (level 0) command "demo" */
SHELL_CMD_REGISTER(demo, &sub_midi2, "Demo commands", NULL);

