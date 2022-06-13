add_subdirectory_ifdef(CONFIG_BLE_RA4W1 lib)
add_subdirectory_ifdef(CONFIG_SOC_FAMILY_RA inc)
add_subdirectory_ifdef(CONFIG_SOC_FAMILY_RA src)
zephyr_include_directories(inc)
