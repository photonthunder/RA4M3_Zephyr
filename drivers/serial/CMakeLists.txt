# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_UART_RA uart_ra.c)
message(STATUS "adding Renesas Serial driver")
