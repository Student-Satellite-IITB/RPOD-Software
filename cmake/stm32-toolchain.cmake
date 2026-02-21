set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Pinned Arm GNU Toolchain (installed outside repo)
set(TOOLCHAIN_DIR "$ENV{HOME}/toolchains/arm-none-eabi-gcc-15")

set(CMAKE_C_COMPILER   "${TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_DIR}/bin/arm-none-eabi-g++")
set(CMAKE_LINKER       "${TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc")

set(CMAKE_OBJCOPY      "${TOOLCHAIN_DIR}/bin/arm-none-eabi-objcopy")
set(CMAKE_SIZE         "${TOOLCHAIN_DIR}/bin/arm-none-eabi-size")
set(CMAKE_AR           "${TOOLCHAIN_DIR}/bin/arm-none-eabi-ar")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Prevent host contamination
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)