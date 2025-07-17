# Copyright (c) 2025 Titouan Christophe
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_MIDI2_UDP_HOST netmidi2.c)

