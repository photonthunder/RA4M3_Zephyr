# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_GPIO_RA gpio_ra.c)
message(STATUS "adding Renesas gpio driver")

