#ifndef SAMPLE_USB_NETMIDI_DISCOVERY_H_
#define SAMPLE_USB_NETMIDI_DISCOVERY_H_

struct netmidi2_disc_ep {
	bool has_address;
	bool has_port;
	struct net_sockaddr_in addr;
	char target[128];
};

typedef void (*ep_found_cb)(const struct netmidi2_disc_ep *disc);

void start_discovery(ep_found_cb cb);

void stop_discovery();

bool is_discovery_started();

#endif
