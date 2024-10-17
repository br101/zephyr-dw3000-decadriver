#
# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2
#
cmake_minimum_required(VERSION 3.13.1)

add_library(dw3000_uwb_driver STATIC
            dw3000_device.c)
            
target_include_directories(dw3000_uwb_driver PRIVATE .)

target_link_libraries(dw3000_uwb_driver 
    PRIVATE uwb_driver
)