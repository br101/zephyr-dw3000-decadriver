#
# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2
#
cmake_minimum_required(VERSION 3.13)

add_library(qmath STATIC
	src/qmath.c
)
target_include_directories(qmath PUBLIC include)
