# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_include_directories(inc)

add_library(dwtlib STATIC IMPORTED GLOBAL)
set_target_properties(dwtlib PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdwt_uwb_driver-m4-sfp-6.0.7.a)
target_link_libraries(app PUBLIC dwtlib)

zephyr_linker_section(NAME .dw_drivers) 
zephyr_linker_section_configure(SECTION .dw_drivers ALIGN 4 KEEP) 
zephyr_linker_sources(SECTIONS custom-sections.ld)
