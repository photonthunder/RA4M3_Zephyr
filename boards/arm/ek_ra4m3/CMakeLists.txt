# SPDX-License-Identifier: Apache-2.0

# if (0)
# message(STATUS "adding Renesas data")

zephyr_sources(ra_gen/hal_data.c)
zephyr_sources(ra_gen/common_data.c)
zephyr_sources(ra_gen/vector_data.c)
zephyr_sources(ra_gen/pin_data.c)

zephyr_include_directories(fsp_cfg)
zephyr_include_directories(ra_gen)

# endif()