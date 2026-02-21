cmake_minimum_required(VERSION 3.10)
project(RPOD-Software LANGUAGES C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED TARGET_PLATFORM)
    set(TARGET_PLATFORM "linux")
endif()

if(TARGET_PLATFORM STREQUAL "linux")
    include(${CMAKE_SOURCE_DIR}/cmake/linux-platform.cmake)

elseif(TARGET_PLATFORM STREQUAL "stm32")
    include(${CMAKE_SOURCE_DIR}/cmake/stm32-platform.cmake)

else()
    message(FATAL_ERROR "Unknown TARGET_PLATFORM: ${TARGET_PLATFORM}")

endif()