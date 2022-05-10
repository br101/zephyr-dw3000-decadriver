# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_include_directories(inc)

add_library(dwtlib STATIC IMPORTED GLOBAL)
if(CONFIG_FPU)
    set_target_properties(dwtlib PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdwt_uwb_driver-m4-hfp-6.0.7.a)
else()
    set_target_properties(dwtlib PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdwt_uwb_driver-m4-sfp-6.0.7.a)
endif()
target_link_libraries(app PUBLIC dwtlib)
