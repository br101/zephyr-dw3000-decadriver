#
# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2
#
cmake_minimum_required(VERSION 3.13)
project(test_deca)

if (NOT EXISTS ${PROJECT_SOURCE_DIR}/../../../tools/cmake)
  message(FATAL_ERROR "\
  Unit tests and coverage uses tools from uwb-stack. \
  You currently need to run this from uwb-stack/deps/dwt_uwb_driver. "
  )
endif()

# Currently dependencies from uwb-stack are used
# $ cd .../uwb-stack/deps/dwt_uwb_driver

# Unit tests can be run like so:
# $ cmake -S utest -B ./build-san -G Ninja -DCMAKE_BUILD_TYPE=Debug
# $ cmake --build ./build-san
# $ ./build-san/utest

# Coverage:
# $ cmake -S utest -B ./build/cov -G Ninja -DCMAKE_BUILD_TYPE=Debug -DENABLE_TEST_COVERAGE=ON
# $ cmake --build ./build/cov
# $ ninja -C ./build/cov ./test_deca_coverage
# $ open build/cov/test_deca_coverage_html/index.html

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../tools/cmake)

option(ENABLE_TEST_COVERAGE "Enable test coverage" OFF)

add_subdirectory(../../../deps/googletest/googletest gtest EXCLUDE_FROM_ALL)

set(DWT_DW3000 ON)

add_subdirectory(.. uwb_driver)
add_executable(utest
  src/test_rsl.cc
  src/test_tx_power.cc
)

target_link_libraries(utest PUBLIC qmath gmock_main uwb_driver)
target_compile_options(utest PUBLIC -Wall -Werror -Wextra)

target_include_directories(utest PRIVATE ${PROJECT_SOURCE_DIR}/../dw3000)

add_test(NAME utest COMMAND utest)

if(ENABLE_TEST_COVERAGE)
  include(Coverage)
  target_coverage(uwb_driver)
  target_coverage(qmath)
  add_coverage(NAME utest GTEST_JUNIT)
else()
  include(Sanitize)
  target_sanitize(utest)
endif()
