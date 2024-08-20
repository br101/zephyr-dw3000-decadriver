# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(dw3000_hw.c dw3000_spi.c deca_port.c)
zephyr_include_directories(.)
