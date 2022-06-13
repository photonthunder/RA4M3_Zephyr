# Makefile - Renesas RA MCU family
#
# Copyright (c) 2016 Piotr Mienkowski
# SPDX-License-Identifier: Apache-2.0
#

add_subdirectory(${SOC_NAME})
add_subdirectory(common)
zephyr_include_directories(${SOC_NAME})
zephyr_include_directories(common)

