#include <zephyr/net/midi2.h>
#include <zephyr/shell/shell.h>

static inline void print_session(const struct shell *sh, const struct netmidi2_ep *ep, size_t session_num)
{
	const struct netmidi2_session *session = &ep->peers[session_num];

	if (session->state < NETMIDI2_SESSION_PENDING_INVITATION) {
		shell_print(sh, " - session %d: <Unused>", session_num);
	} else {
		const struct net_sockaddr *addr = net_sad(&session->addr);
		const struct net_sockaddr_in6 *addr6 = net_sin6(addr);
		char peer_name[NET_INET6_ADDRSTRLEN];

		const char *state_name;
		switch (session->state) {
			case NETMIDI2_SESSION_PENDING_INVITATION:
				state_name = "Invitation ongoing";
				break;
			case NETMIDI2_SESSION_AUTH_REQUIRED:
				state_name = "Invitation ongoing (auth required)";
				break;
			case NETMIDI2_SESSION_ESTABLISHED:
				state_name = "Established";
				break;
			case NETMIDI2_SESSION_PENDING_RESET:
				state_name = "Pending reset";
				break;
			case NETMIDI2_SESSION_PENDING_BYE:
				state_name = "Closing";
				break;
			default:
				state_name = "Unused";
				break;
		}

		net_addr_ntop(addr->sa_family, &addr6->sin6_addr,
			      peer_name, sizeof(peer_name));
		shell_print(sh, " - session %d <%s> peer udp://%s:%d",
			    session_num, state_name, peer_name, ntohs(addr6->sin6_port));
	}
}

static inline void print_ep(const struct shell *sh, const struct netmidi2_ep *ep, size_t ep_num)
{
	const char *accepting = ep->accept_invitations ? "accepting" : "rejecting";
	if (ep->name) {
		shell_print(sh, "ep %d: %s (%s invitations)",
			    ep_num, ep->name, accepting);
	} else {
		shell_print(sh, "ep %d: ep@%p (%s invitations)",
			    ep_num, ep, accepting);
	}

	for (size_t i=0; i<ep->n_peers; i++) {
		print_session(sh, ep, i);
	}
}

static inline struct netmidi2_ep *get_ep(const char *ep_num_text)
{
	struct netmidi2_ep *ep;
	char *end;
	size_t ep_num;
	size_t max_ep_num;

	STRUCT_SECTION_COUNT(netmidi2_ep, &max_ep_num);

	ep_num = strtoul(ep_num_text, &end, 10);
	if (ep_num_text == end || ep_num >= max_ep_num) {
		return NULL;
	}

	STRUCT_SECTION_GET(netmidi2_ep, ep_num, &ep);
	return ep;
}

static inline struct netmidi2_session *get_session(const char *ep_num_text, const char *session_num_text)
{
	struct netmidi2_ep *ep = get_ep(ep_num_text);
	if (! ep) {
		return NULL;
	}

	char *end;
	size_t session_num = strtol(session_num_text, &end, 10);
	if (end == session_num_text) {
		return NULL;
	}

	if (session_num >= ep->n_peers) {
		return NULL;
	}

	return &ep->peers[session_num];
}

static int cmd_netmidi2_status(const struct shell *sh, int argc, char *argv[])
{
	size_t ep_num = 0;

	if (argc < 2) {
		STRUCT_SECTION_FOREACH(netmidi2_ep, ep) {
			print_ep(sh, ep, ep_num++);
		}
	} else {
		struct netmidi2_ep *ep = get_ep(argv[1]);

		if (ep == NULL) {
			shell_fprintf_error(sh, "Invalid endpoint number %s\n", argv[1]);
			shell_print(sh, "USAGE: %s [ <ep_num> ]", argv[0]);
			return -EINVAL;
		}

		print_ep(sh, ep, ep_num);
	}

	return 0;
}

static int cmd_netmidi2_invite(const struct shell *sh, int argc, char *argv[])
{
	if (argc < 4) {
		shell_print(sh, "USAGE: %s <ep_num> <address> <port>", argv[0]);
		return -ENOEXEC;
	}

	struct netmidi2_ep *ep = get_ep(argv[1]);
	if (ep == NULL) {
		shell_fprintf_error(sh, "Invalid endpoint number %s\n", argv[1]);
		return -EINVAL;
	}

	struct net_sockaddr_in addr = {.sin_family = AF_INET};

	if (net_addr_pton(AF_INET, argv[2], &addr.sin_addr) != 0) {
		shell_fprintf_error(sh, "Invalid ipv4 %s\n", argv[2]);
		return -EINVAL;
	}

	uint16_t port = atoi(argv[3]);
	if (port == 0) {
		shell_fprintf_error(sh, "Invalid port %s\n", argv[3]);
		return -EINVAL;
	}

	addr.sin_port = htons(port);

	struct netmidi2_session *session = netmidi2_ep_invite(ep, (struct net_sockaddr *) &addr, sizeof(addr));
	if (session == NULL) {
		shell_fprintf_error(sh, "Invite error\n");
		return -ENOEXEC;
	}

	size_t session_idx = session - ep->peers;
	print_session(sh, ep, session_idx);

	return 0;
}

static int cmd_netmidi2_bye(const struct shell *sh, int argc, char *argv[])
{
	if (argc < 3) {
		shell_print(sh, "USAGE: %s <ep_num> <session_num>", argv[0]);
		return -ENOEXEC;
	}

	struct netmidi2_session *session = get_session(argv[1], argv[2]);
	if (session == NULL) {
		shell_fprintf_error(sh, "Invalid endpoint/session number %s/%s\n", argv[1], argv[2]);
		return -EINVAL;
	}

	netmidi2_session_bye(session);

	return 0;
}

static inline struct midi_ump parse_ump(const char *repr)
{
	size_t i = 0;
	struct midi_ump res;

	while (i < 4 && *repr) {
		if ('0' <= *repr && *repr <= '9') {
			res.data[i] = (res.data[i] << 4) | (*repr - '0');
		} else if ('a' <= *repr && *repr <= 'f') {
			res.data[i] = (res.data[i] << 4) | (10 + *repr - 'a');
		} else if ('A' <= *repr && *repr <= 'F') {
			res.data[i] = (res.data[i] << 4) | (10 + *repr - 'A');
		} else if (*repr == ',') {
			i++;
		}

		repr++;
	}

	return res;
}

static inline void print_ump_format(const struct shell *sh)
{
	shell_print(sh, "---------------------------------------------------");
	shell_print(sh, "UMP format in this shell:");
	shell_print(sh, " - Write UMP data words (32 bits) in hex");
	shell_print(sh, " - Separate data words with comma (,)");
	shell_print(sh, " - Separate UMP packets with space ( )");
	shell_print(sh, " - Example: 41923700,FFFF0000 2291407F");
	shell_print(sh, "   This means: 2 UMP packets");
	shell_print(sh, "    - MIDI2 channel voice packet (with 2 words)");
	shell_print(sh, "    - MIDI1 channel voice packet (with only 1 word)");
	shell_print(sh, "---------------------------------------------------");
}

static int cmd_netmidi2_send(const struct shell *sh, int argc, char *argv[])
{
	if (argc < 3) {
		shell_print(sh, "USAGE: %s <ep_num> <session_num> UMP DATA... ", argv[0]);
		print_ump_format(sh);
		return -ENOEXEC;
	}

	struct netmidi2_session *session = get_session(argv[1], argv[2]);
	if (session == NULL) {
		shell_fprintf_error(sh, "Invalid endpoint/session number %s/%s\n", argv[1], argv[2]);
		return -EINVAL;
	}

	for (size_t i=3; i<argc; i++) {
		netmidi2_send(session, parse_ump(argv[i]));
	}

	return 0;
}

static int cmd_netmidi2_broadcast(const struct shell *sh, int argc, char *argv[])
{
	if (argc < 2) {
		shell_print(sh, "USAGE: %s <ep_num> UMP DATA... ", argv[0]);
		print_ump_format(sh);
		return -ENOEXEC;
	}

	struct netmidi2_ep *ep = get_ep(argv[1]);
	if (ep == NULL) {
		shell_fprintf_error(sh, "Invalid endpoint number %s\n", argv[1]);
		return -EINVAL;
	}

	for (size_t i=2; i<argc; i++) {
		netmidi2_broadcast(ep, parse_ump(argv[i]));
	}

	return 0;
}

static int cmd_netmidi2_accept(const struct shell *sh, int argc, char *argv[])
{
	if (argc < 2) {
		shell_print(sh, "USAGE: %s <ep_num> [ 1/0 ] ", argv[0]);
		return -ENOEXEC;
	}

	struct netmidi2_ep *ep = get_ep(argv[1]);
	if (ep == NULL) {
		shell_fprintf_error(sh, "Invalid endpoint number %s\n", argv[1]);
		return -EINVAL;
	}

	if (argc > 2) {
		ep->accept_invitations = (argv[2][0] == '1');
	}

	shell_print(sh, "Endpoint is %s invitations",
		    ep->accept_invitations ? "accepting" : "rejecting");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(netmidi2_subcmd,
	SHELL_CMD(status, NULL,
		  "Show current status of Network MIDI2.0 endpoints",
		  cmd_netmidi2_status),
	SHELL_CMD(invite, NULL,
		  "Invite another Network MIDI2.0 endpoint",
		  cmd_netmidi2_invite),
	SHELL_CMD(bye, NULL,
		  "Send a Bye command and close session",
		  cmd_netmidi2_bye),
	SHELL_CMD(send, NULL,
		  "Send UMP data over an established session",
		  cmd_netmidi2_send),
	SHELL_CMD(broadcast, NULL,
		  "Send UMP data to all established sessions",
		  cmd_netmidi2_broadcast),
	SHELL_CMD(accept, NULL,
		  "Enable or disable accepting invitations on endpoint",
		  cmd_netmidi2_accept),
	SHELL_SUBCMD_SET_END,
);

SHELL_CMD_REGISTER(netmidi2, &netmidi2_subcmd, "Network MIDI2.0 commands", NULL);
