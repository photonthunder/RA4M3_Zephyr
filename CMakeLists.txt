# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).

add_subdirectory(drivers)
zephyr_include_directories(include)
# message(STATUS "adding out of box drivers folder")