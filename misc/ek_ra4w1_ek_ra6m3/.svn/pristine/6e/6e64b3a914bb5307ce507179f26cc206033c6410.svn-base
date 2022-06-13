# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(beacon)

target_sources(app PRIVATE src/main.c)

zephyr_sources(src/ble_app.c)

zephyr_sources(ra_gen/hal_data.c)
zephyr_sources(ra_gen/common_data.c)
zephyr_sources(ra_gen/vector_data.c)
zephyr_sources(ra_gen/pin_data.c)

zephyr_include_directories(fsp_cfg)
zephyr_include_directories(ra_gen)
zephyr_include_directories(qe_gen)
add_subdirectory(qe_gen)

#zephyr_include_directories($ENV{GNUARMEMB_TOOLCHAIN_PATH}/arm-none-eabi/include)

# Create a wrapper CMake library that our app can link with
target_link_libraries(app PUBLIC r_ble)
add_library(r_ble STATIC IMPORTED GLOBAL)

set_target_properties(r_ble PROPERTIES IMPORTED_LOCATION             $ENV{ZEPHYR_BASE}/../modules/hal/renesas/fsp/lib/r_ble/cm4_gcc/all/libr_ble.a)
set_target_properties(r_ble PROPERTIES INTERFACE_INCLUDE_DIRECTORIES $ENV{ZEPHYR_BASE}/../modules/hal/renesas/fsp/inc/api)

target_link_libraries(app PUBLIC r_ble)