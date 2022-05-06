# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_DW3000 dwt_uwb_driver)
add_subdirectory_ifdef(CONFIG_DW3000 platform)
