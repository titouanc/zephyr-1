#include <zephyr/net/midi2.h>
#include <zephyr/shell/shell.h>

static int cmd_netmidi2_status(const struct shell *sh, int argc, char *argv[])
{
	if (argc < 2) {
		STRUCT_SECTION_FOREACH(netmidi2_ep, ep) {
		        shell_print(sh, "ep@%p", ep);
		}
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(netmidi2,
	SHELL_CMD(status, NULL,
		  "Show current status of Network MIDI2.0 endpoints",
		  cmd_netmidi2_status),
	SHELL_SUBCMD_SET_END,
);

SHELL_CMD_REGISTER(midi2, &netmidi2, "Network MIDI2.0 commands", NULL);
