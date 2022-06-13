# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=R7FA4W1AD" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
