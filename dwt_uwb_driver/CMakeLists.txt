#
# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2
#
cmake_minimum_required(VERSION 3.13.1)

project(dw3xxx)

add_library(uwb_driver_itf INTERFACE)
target_include_directories(uwb_driver_itf INTERFACE .)

add_library(uwb_driver STATIC
                deca_interface.c
                deca_compat.c
                deca_rsl.c)

target_link_libraries(uwb_driver 
    PUBLIC uwb_driver_itf
)

if(DWT_DW3000)
    message("Building drivers for DW3000 devices")
    add_subdirectory(dw3000)
    target_link_libraries(uwb_driver PRIVATE dw3000_uwb_driver)
endif()

if(DWT_DW3720)
    message("Building drivers for DW3720 devices")
    add_subdirectory(dw3720)
    target_link_libraries(uwb_driver PRIVATE dw3720_uwb_driver)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/qmath)
target_link_libraries(uwb_driver PRIVATE qmath)