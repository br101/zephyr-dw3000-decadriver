#
# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2
#
cmake_minimum_required(VERSION 3.13.1)

add_library(dw3720_uwb_driver STATIC
            dw3720_device.c)

target_include_directories(dw3720_uwb_driver PRIVATE .)

target_link_libraries(dw3720_uwb_driver 
    PRIVATE uwb_driver
)