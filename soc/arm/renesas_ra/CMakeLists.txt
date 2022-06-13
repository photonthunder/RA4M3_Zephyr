# SPDX-License-Identifier: Apache-2.0

add_subdirectory(common)
zephyr_include_directories(common)
add_subdirectory(${SOC_SERIES})