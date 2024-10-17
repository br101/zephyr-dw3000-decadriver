#
# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2
#
cmake_minimum_required(VERSION 3.13)
project(test_qmath)

if (NOT EXISTS ${PROJECT_SOURCE_DIR}/../../../../../tools/cmake)
  message(FATAL_ERROR "\
  Unit tests and coverage uses tools from uwb-stack. \
  You currently need to run this from uwb-stack/deps/dwt_uwb_driver. "
  )
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../../../tools/cmake)

option(ENABLE_TEST_COVERAGE "Enable test coverage" OFF)

add_subdirectory(../../../../../deps/googletest/googletest gtest EXCLUDE_FROM_ALL)

add_subdirectory(.. qmath)
add_executable(test_qmath
  test_qmath.cc
)

target_link_libraries(test_qmath PUBLIC qmath gmock_main)
target_compile_options(test_qmath PUBLIC -Wall -Werror -Wextra)

add_test(NAME test_qmath COMMAND test_qmath)

if(ENABLE_TEST_COVERAGE)
  include(Coverage)
  target_coverage(qmath)
  add_coverage(NAME test_qmath GTEST_JUNIT)
else()
  include(Sanitize)
  target_sanitize(qmath)
endif()
