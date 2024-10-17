#
# SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
# SPDX-License-Identifier: LicenseRef-QORVO-2
#

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PREFIX arm-none-eabi-)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)

set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy CACHE INTERNAL "objcopy tool")
set(CMAKE_SIZE_UTIL ${TOOLCHAIN_PREFIX}size CACHE INTERNAL "size tool")

# Architecture-related flags
set(ARCH_GCC_FLAGS
"-mthumb \
 -mcpu=cortex-m33+nodsp \
 -mabi=aapcs \
 -mfpu=fpv5-sp-d16 \
 -specs=nano.specs \
 -mcmse"
)

# Select warnings, C standard, etc.
set(MISC_GCC_FLAGS 
"-fdata-sections \
 -ffunction-sections \
 -funsigned-char \
 -Wall \
 -Wdouble-promotion \
 -Wundef \
 -Wno-format \
 -Wno-unused-but-set-variable \
 -Wno-pedantic \
 -fsingle-precision-constant \
 -fno-builtin \
 -fshort-enums \
 -Werror \
 -std=c11 \
 -nostdlib"
)

set(CMAKE_C_FLAGS "${ARCH_GCC_FLAGS} ${MISC_GCC_FLAGS}")
set(CMAKE_ASM_FLAGS "${ARCH_GCC_FLAGS} ${MISC_GCC_FLAGS}")

    
# Note that these config-specific flags will be added to the general flags above
# NOTE: -O3 raises somw unaligned accesses due to compiler bug (str.w aligned on 2 bytes for uint16)
set(CMAKE_C_FLAGS_DEBUG          "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE        "-O2 -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g3 -DNDEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG")

# Note https://github.com/raspberrypi/pico-sdk/issues/1029
set(CMAKE_EXE_LINKER_FLAGS 
"-fuse-ld=bfd \
 -Wl,--gc-sections \
 -Wl,--print-memory-usage \
 -Wl,--no-warn-rwx-segments")


