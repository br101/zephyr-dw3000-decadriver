# SPDX-License-Identifier: Apache-2.0

if(CONFIG_FPU)
    set(DWTLIBNAME libdwt_uwb_driver-m4-hfp-6.0.14.a)
else()
    set(DWTLIBNAME libdwt_uwb_driver-m4-sfp-6.0.14.a)
endif()

# use zephyr_library_import in order to get it linked with the
# -Wl,--whole-archive flag (keep all symbols)
zephyr_library_import(dwtlib ${CMAKE_CURRENT_SOURCE_DIR}/lib/${DWTLIBNAME})

zephyr_include_directories(inc)

# the library needs a custom linker script
zephyr_linker_sources(SECTIONS custom-sections.ld)
