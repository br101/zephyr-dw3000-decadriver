# SPDX-License-Identifier: Apache-2.0

if(CONFIG_FPU)
    if(CONFIG_SOC_SERIES_NRF53X)
        set(DWTLIBNAME libdwt_uwb_driver-m33-hfp-6.0.7.a)
    else()
        set(DWTLIBNAME libdwt_uwb_driver-m4-hfp-6.0.7.a)
    endif()
else()
    if(CONFIG_SOC_SERIES_NRF53X)
        set(DWTLIBNAME libdwt_uwb_driver-m33-sfp-6.0.7.a)
    else()
        set(DWTLIBNAME libdwt_uwb_driver-m4-sfp-6.0.7.a)
    endif()
endif()

# use zephyr_library_import in order to get it linked with the
# -Wl,--whole-archive flag (keep all symbols)
zephyr_library_import(dwtlib ${CMAKE_CURRENT_SOURCE_DIR}/lib/${DWTLIBNAME})

zephyr_include_directories(inc)

# the library needs a custom linker script
zephyr_linker_sources(SECTIONS custom-sections.ld)
