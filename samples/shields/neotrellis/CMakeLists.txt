# Copyright (c) 2024 Titouan Christophe
# SPDX-License-Identifier: Apache-2.0

set(DTC_OVERLAY_FILE neotrellis.dts)

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(neotrellis)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
