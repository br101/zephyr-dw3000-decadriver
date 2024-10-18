# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_DW3000 platform)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_DW3000
    dwt_uwb_driver/deca_interface.c
    dwt_uwb_driver/deca_compat.c
    dwt_uwb_driver/deca_rsl.c
    dwt_uwb_driver/lib/qmath/src/qmath.c
)

zephyr_library_sources_ifdef(CONFIG_DW3000_CHIP_DW3000 dwt_uwb_driver/dw3000/dw3000_device.c)
zephyr_library_sources_ifdef(CONFIG_DW3000_CHIP_DW3720 dwt_uwb_driver/dw3720/dw3720_device.c)

zephyr_include_directories(dwt_uwb_driver)
zephyr_include_directories(dwt_uwb_driver/lib/qmath/include)
