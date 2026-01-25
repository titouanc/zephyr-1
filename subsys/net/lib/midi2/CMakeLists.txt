# Copyright (c) 2025 Titouan Christophe
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(netmidi2.c)
zephyr_library_sources_ifdef(CONFIG_NETMIDI2_SHELL netmidi2_shell.c)

zephyr_linker_sources(DATA_SECTIONS netmidi2.ld)
